#ifndef __LED_STRIP_CONTROLLER_H__
#define __LED_STRIP_CONTROLLER_H__

#include "mcp_server.h"
#include "settings.h"

#include <driver/gpio.h>
#include <esp_log.h>
#include <esp_random.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <led_strip.h>

#include <cctype>
#include <cstdio>
#include <string>

// 可编程 RGB 灯板/灯带控制器（WS2812 / WS2812B / SK6812 单线灯珠）
//
// 用法和 LampController 一样，一行创建：
//     static LedStripController light(LED_STRIP_GPIO, LED_STRIP_MAX, "light", "彩灯板", LED_STRIP_COUNT);
//
// 注册给大模型的工具（name = "light"）：
//     self.light.turn_on / turn_off
//     self.light.set_brightness   0~100
//     self.light.set_color        red / green / blue 各 0~255（任意颜色）
//     self.light.random_color     随机生成一个鲜艳颜色
//     self.light.set_effect       solid / blink / breath / flow / marquee / rainbow / meteor / strobe
//                                 + color: auto（默认，自动彩色）/ rainbow（强制彩虹）/ single（用当前颜色）
//     self.light.set_led_count    灯珠数量（存 NVS，换灯板不用重烧）
//     self.light.get_status       返回当前状态
//
// 实现要点：
//   1. 灯珠由 RMT 硬件发送时序，不占用 CPU，也不占用 LEDC（和舵机互不冲突）；
//   2. 所有 MCP 工具只改“状态变量”，真正的刷灯由后台任务统一按 30 帧/秒渲染，
//      这样不会出现两个任务同时操作灯带；
//   3. 亮度是软件调光：把 RGB 三个通道同时按比例缩小；
//   4. 动画进度全部由“毫秒时间戳”算，不用帧计数器，长时间运行不会溢出。

class LedStripController {
private:
    enum Effect : int {
        kSolid = 0,  // 常亮
        kBlink,      // 闪烁
        kBreath,     // 呼吸
        kFlow,       // 流水灯（亮块带拖尾前进）
        kMarquee,    // 跑马灯（每隔几颗亮一颗，整圈滚动）
        kRainbow,    // 彩虹流动
        kMeteor,     // 流星（单点长拖尾）
        kStrobe,     // 爆闪（白色快速闪光）
    };

    led_strip_handle_t strip_ = nullptr;
    gpio_num_t         gpio_num_;
    int                count_;        // 缓冲区大小（硬件上限）
    std::string        label_;
    std::string        nvs_ns_;       // 每块灯板独立的 NVS 命名空间（"led_" + name）

    // 实际亮几颗：可以用语音改（self.light.set_led_count），并记在 NVS 里，
    // 这样换不同数量的灯板不用重新烧固件。
    volatile int  active_     = 1;

    // 下面这些变量只被“工具回调”写、被“渲染任务”读，加 volatile 防止被优化掉
    volatile bool power_      = false;   // 上电不亮，等用户喊“开灯”
    volatile int  brightness_ = 80;
    volatile int  base_r_     = 255;     // 当前基础色
    volatile int  base_g_     = 255;
    volatile int  base_b_     = 255;
    volatile int  effect_     = kSolid;
    volatile int  speed_      = 5;       // 1~10

    // 跑动类特效（流水灯、跑马灯、流星）用什么颜色：
    //   kColorRainbow = 每颗灯珠按位置取彩虹色（彩色流动，观感最好）
    //   kColorSingle  = 用当前基础色（例如"红色流水灯"）
    // 由 set_effect 的 color 参数决定，默认自动：没设过颜色就彩虹，设过就跟随。
    enum ColorMode : int {
        kColorAuto    = 0,
        kColorRainbow = 1,
        kColorSingle  = 2,
    };
    volatile int  color_mode_ = kColorRainbow;

    bool lit_   = false; // 上一帧是否亮着（关灯时只清一次，省 RMT 带宽）

    static int Clamp(int v, int lo, int hi) {
        return v < lo ? lo : (v > hi ? hi : v);
    }

    // 速度 1~10 → 每走一格需要的毫秒数（speed 越大越短，最小 20ms）
    int StepMs(int base_ms) const {
        int s = speed_;
        if (s < 1) s = 1;
        if (s > 10) s = 10;
        int ms = base_ms / s;
        return ms < 20 ? 20 : ms;
    }

    // H（0~360）S（0~255）V（0~255）→ RGB（0~255）
    static void HsvToRgb(int h, int s, int v, int& r, int& g, int& b) {
        h = ((h % 360) + 360) % 360;
        int i = h / 60;
        int f = h % 60;
        int p = v * (255 - s) / 255;
        int q = v * (255 - s * f / 60) / 255;
        int t = v * (255 - s * (60 - f) / 60) / 255;
        switch (i) {
            case 0:  r = v; g = t; b = p; break;
            case 1:  r = q; g = v; b = p; break;
            case 2:  r = p; g = v; b = t; break;
            case 3:  r = p; g = q; b = v; break;
            case 4:  r = t; g = p; b = v; break;
            default: r = v; g = p; b = q; break;
        }
    }

    // 写一颗灯珠：scale 是叠加在总亮度之上的额外比例（0~100），用于拖尾渐暗
    void Put(int index, int r, int g, int b, int scale = 100) {
        if (strip_ == nullptr || index < 0 || index >= active_) {
            return;
        }
        int k = brightness_ * scale / 100;   // 0~100
        if (k <= 0) {
            led_strip_set_pixel(strip_, index, 0, 0, 0);
            return;
        }
        led_strip_set_pixel(strip_, index,
                            (uint32_t)r * k / 100,
                            (uint32_t)g * k / 100,
                            (uint32_t)b * k / 100);
    }

    // 随机出一个“鲜艳”的颜
    void MakeRandomColor() {
        int hue = esp_random() % 360;
        int sat = 210 + esp_random() % 46;   // 82%~100%
        int val = 210 + esp_random() % 46;
        int r, g, b;
        HsvToRgb(hue, sat, val, r, g, b);
        base_r_ = r;
        base_g_ = g;
        base_b_ = b;
    }

    // 把大模型传来的颜色名换成 RGB。支持英文名、中文名、"#RRGGBB"、random。
    bool ParseColor(const std::string& raw, int& r, int& g, int& b) {
        std::string s;
        for (char c : raw) {
            s += static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        }
        auto has = [&s](const char* kw) { return s.find(kw) != std::string::npos; };

        if (has("random") || has("随机") || has("随便") || has("换一个") || has("换色")) {
            MakeRandomColor();
            r = base_r_;
            g = base_g_;
            b = base_b_;
            return true;
        }

        // #RRGGBB
        if (s.size() == 7 && s[0] == '#') {
            auto hex = [](char c) -> int {
                if (c >= '0' && c <= '9') return c - '0';
                if (c >= 'a' && c <= 'f') return c - 'a' + 10;
                return -1;
            };
            int v[6];
            bool ok = true;
            for (int i = 0; i < 6; i++) {
                v[i] = hex(s[i + 1]);
                if (v[i] < 0) ok = false;
            }
            if (ok) {
                r = v[0] * 16 + v[1];
                g = v[2] * 16 + v[3];
                b = v[4] * 16 + v[5];
                return true;
            }
        }

        // 顺序有讲究：具体的名字放前面，宽泛的（红/白/绿）放后面，
        // 否则“暖白”会被“白”抢先匹配、“玫红”会被“红”抢先匹配。
        struct Named {
            const char* key;
            int r, g, b;
        };
        static const Named kNamed[] = {
            {"暖白", 255, 180, 120}, {"warm", 255, 180, 120},
            {"冷白", 180, 220, 255}, {"cool", 180, 220, 255},
            {"薰衣草", 200, 160, 255}, {"lavender", 200, 160, 255},
            {"绿松", 64, 224, 208}, {"turquoise", 64, 224, 208},
            {"薄荷", 150, 255, 190}, {"mint", 150, 255, 190},
            {"珊瑚", 255, 127, 80}, {"coral", 255, 127, 80},
            {"洋红", 255, 0, 255}, {"品红", 255, 0, 255}, {"magenta", 255, 0, 255},
            {"玫", 255, 0, 128}, {"rose", 255, 0, 128},
            {"粉", 255, 105, 180}, {"pink", 255, 105, 180},
            {"琥珀", 255, 150, 0}, {"amber", 255, 150, 0},
            {"金", 255, 200, 0}, {"gold", 255, 200, 0},
            {"棕", 150, 75, 0}, {"咖啡", 150, 75, 0}, {"褐", 150, 75, 0},
            {"brown", 150, 75, 0},
            {"紫", 160, 32, 240}, {"purple", 160, 32, 240}, {"violet", 138, 43, 226},
            {"靛", 75, 0, 130}, {"indigo", 75, 0, 130},
            {"蓝绿", 0, 128, 128}, {"青绿", 0, 255, 128}, {"teal", 0, 128, 128},
            {"青", 0, 255, 255}, {"cyan", 0, 255, 255},
            {"天蓝", 0, 128, 255}, {"azure", 0, 128, 255},
            {"蓝", 0, 0, 255}, {"blue", 0, 0, 255},
            {"草绿", 128, 255, 0}, {"黄绿", 128, 255, 0}, {"lime", 128, 255, 0},
            {"绿", 0, 255, 0}, {"green", 0, 255, 0},
            {"橙", 255, 128, 0}, {"橘", 255, 128, 0}, {"orange", 255, 128, 0},
            {"黄", 255, 255, 0}, {"yellow", 255, 255, 0},
            {"白", 255, 255, 255}, {"white", 255, 255, 255},
            {"红", 255, 0, 0}, {"red", 255, 0, 0},
        };
        for (const auto& c : kNamed) {
            if (has(c.key)) {
                r = c.r;
                g = c.g;
                b = c.b;
                return true;
            }
        }
        return false;
    }

    // 把大模型传来的英文/中文名字折成内部枚举，认不出来就常亮
    void SetEffectByName(const std::string& name) {
        std::string s;
        for (char c : name) {
            s += static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        }
        auto has = [&s](const char* kw) { return s.find(kw) != std::string::npos; };

        if (has("strobe") || has("爆")) {
            effect_ = kStrobe;
        } else if (has("breath") || has("呼吸")) {
            effect_ = kBreath;
        } else if (has("marquee") || has("跑马") || has("追逐")) {
            effect_ = kMarquee;
        } else if (has("flow") || has("流水")) {
            effect_ = kFlow;
        } else if (has("rainbow") || has("彩虹") || has("渐变")) {
            effect_ = kRainbow;
        } else if (has("meteor") || has("流星") || has("彗星")) {
            effect_ = kMeteor;
        } else if (has("blink") || has("闪")) {
            effect_ = kBlink;
        } else {
            effect_ = kSolid;   // solid / 常亮 / 停止特效 / 认不出来
        }
    }

    // ---------- 渲染 ----------

    void Render() {
        if (strip_ == nullptr) {
            return;
        }
        if (!power_ || brightness_ <= 0) {
            if (lit_) {
                led_strip_clear(strip_);
                led_strip_refresh(strip_);   // clear 只改缓存，必须 refresh 才会真正灭灯
                lit_ = false;
            }
            return;
        }
        lit_ = true;

        const int     n  = active_;                        // 本帧按几颗来画（快照，避免画一半被改）
        const int64_t ms = esp_timer_get_time() / 1000;
        const int     r  = base_r_, g = base_g_, b = base_b_;
        const bool    rainbow = (color_mode_ == kColorRainbow);

        // 彩虹模式下每颗灯珠的色相：按位置铺满一圈，再随时间缓慢旋转（约 6 秒转一圈）
        auto RainbowAt = [&](int index, int& rr, int& gg, int& bb) {
            int hue = index * 360 / n + static_cast<int>(ms / 16) % 360;
            HsvToRgb(hue, 255, 255, rr, gg, bb);
        };

        switch (effect_) {
            case kSolid: {
                for (int i = 0; i < n; i++) Put(i, r, g, b);
                break;
            }
            case kBlink: {
                int period = Clamp(1300 - speed_ * 120, 120, 1300);
                bool on = (ms % period) < period / 2;
                for (int i = 0; i < n; i++) Put(i, on ? r : 0, on ? g : 0, on ? b : 0);
                break;
            }
            case kBreath: {
                int period = Clamp(4000 - speed_ * 320, 700, 4000);
                int phase  = static_cast<int>(ms % period) * 2;   // 0 ~ 2*period
                int level  = (phase <= period)
                                 ? phase * 100 / period
                                 : (2 * period - phase) * 100 / period;
                for (int i = 0; i < n; i++) Put(i, r, g, b, level);
                break;
            }
            case kFlow: {
                // 一颗主灯珠 + 3 颗渐暗的尾巴，整体向前挪
                int head = static_cast<int>((ms / StepMs(300)) % n);
                for (int i = 0; i < n; i++) Put(i, 0, 0, 0);
                for (int k = 0; k <= 3; k++) {
                    int idx = ((head - k) % n + n) % n;
                    int rr = r, gg = g, bb = b;
                    if (rainbow) RainbowAt(idx, rr, gg, bb);
                    Put(idx, rr, gg, bb, 100 - k * 30);
                }
                break;
            }
            case kMarquee: {
                // 每隔 3 颗亮 1 颗，位移让亮点绕圈跑
                int offset = static_cast<int>((ms / StepMs(900)) % 3);
                for (int i = 0; i < n; i++) {
                    bool on = ((i + offset) % 3) == 0;
                    int rr = r, gg = g, bb = b;
                    if (rainbow && on) RainbowAt(i, rr, gg, bb);
                    Put(i, on ? rr : 0, on ? gg : 0, on ? bb : 0);
                }
                break;
            }
            case kRainbow: {
                int hue0 = static_cast<int>(ms / Clamp(60 - speed_ * 4, 8, 60)) % 360;
                for (int i = 0; i < n; i++) {
                    int rr, gg, bb;
                    HsvToRgb(hue0 + i * 360 / n, 255, 255, rr, gg, bb);
                    Put(i, rr, gg, bb);
                }
                break;
            }
            case kMeteor: {
                int head = static_cast<int>((ms / StepMs(300)) % (n + 5));
                for (int i = 0; i < n; i++) Put(i, 0, 0, 0);
                for (int k = 0; k < 5; k++) {
                    int idx = head - k;
                    if (idx >= 0 && idx < n) {
                        int rr = r, gg = g, bb = b;
                        if (rainbow) RainbowAt(idx, rr, gg, bb);
                        Put(idx, rr, gg, bb, 100 - k * 20);
                    }
                }
                break;
            }
            case kStrobe: {
                int period = Clamp(700 - speed_ * 55, 120, 700);
                bool on = (ms % period) < 40;
                for (int i = 0; i < n; i++) Put(i, on ? 255 : 0, on ? 255 : 0, on ? 255 : 0);
                break;
            }
            default:
                for (int i = 0; i < n; i++) Put(i, r, g, b);
                break;
        }

        led_strip_refresh(strip_);
    }

    static void TaskEntry(void* arg) {
        static_cast<LedStripController*>(arg)->TaskLoop();
    }

    void TaskLoop() {
        while (true) {
            Render();
            vTaskDelay(pdMS_TO_TICKS(33));   // 约 30 帧/秒
        }
    }

    static const char* EffectName(int effect) {
        switch (effect) {
            case kBlink:   return "blink";
            case kBreath:  return "breath";
            case kFlow:    return "flow";
            case kMarquee: return "marquee";
            case kRainbow: return "rainbow";
            case kMeteor:  return "meteor";
            case kStrobe:  return "strobe";
            default:       return "solid";
        }
    }

public:
    // gpio_num   : 灯板数据线（DIN / DI）接的引脚
    // max_leds   : 缓冲区大小，也就是这块固件最多能驱动多少颗灯珠（超出部分不显示）
    // name       : 工具名前缀，语音端体现为 self.<name>.xxx（只用英文/数字）
    // label      : 语音里怎么称呼它，如 "彩灯板"、"灯带"
    // active     : 实际点亮几颗，0 表示等于 max_leds；之后还可以用语音改
    // grb_order  : WS2812/WS2812B 是 GRB 顺序（true）；个别老灯板是 RGB（false）
    LedStripController(gpio_num_t gpio_num,
                       int max_leds,
                       const std::string& name  = "light",
                       const std::string& label = "彩灯板",
                       int  active              = 0,
                       bool grb_order           = true)
        : gpio_num_(gpio_num), count_(max_leds), label_(label), nvs_ns_("led_" + name) {
        if (gpio_num_ == GPIO_NUM_NC || count_ <= 0) {
            return;
        }
        if (nvs_ns_.size() > 15) {   // NVS 命名空间上限 15 字符
            nvs_ns_.resize(15);
        }

        // 实际亮几颗：优先用 NVS 里上次记住的值（换灯板后不用重新烧固件）
        // 命名空间带设备名（"led_light" / "led_light2"），多块灯板互不覆盖
        active_ = (active > 0 && active <= count_) ? active : count_;
        {
            Settings settings(nvs_ns_.c_str(), false);
            active_ = Clamp(settings.GetInt("count", active_), 1, count_);
        }

        // 1. 初始化灯带：WS2812 用 GRB 顺序，RMT 分辨率 10MHz
        led_strip_config_t strip_config = {};
        strip_config.strip_gpio_num           = gpio_num_;
        strip_config.max_leds                 = count_;
        strip_config.color_component_format   = grb_order
                                                    ? LED_STRIP_COLOR_COMPONENT_FMT_GRB
                                                    : LED_STRIP_COLOR_COMPONENT_FMT_RGB;
        strip_config.led_model                = LED_MODEL_WS2812;

        led_strip_rmt_config_t rmt_config = {};
        rmt_config.resolution_hz = 10 * 1000 * 1000;
        rmt_config.mem_block_symbols = 64;

        ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &strip_));
        led_strip_clear(strip_);

        // 2. 注册 MCP 工具
        auto& mcp_server = McpServer::GetInstance();

        // 开关合并成一个工具（原来是 turn_on + turn_off 两个）
        mcp_server.AddTool("self." + name + ".power",
            "打开或关闭" + label_ + "。on 传 true 是开灯，false 是关灯。"
            "用户说“开灯”“打开彩灯”“亮起来”传 true；"
            "说“关灯”“把灯关掉”“灭了”传 false。",
            PropertyList({ Property("on", kPropertyTypeBoolean, true) }),
            [this](const PropertyList& properties) -> ReturnValue {
                power_ = properties["on"].value<bool>();
                if (power_ && brightness_ <= 0) brightness_ = 80;
                return true;
            });

        mcp_server.AddTool("self." + name + ".set_brightness",
            "设置" + label_ + "的亮度，0 到 100：0 相当于熄灭，100 最亮。"
            "用户说“亮一点”“调暗一点”“亮度调到 50”时调用；"
            "如果不知道当前亮度，先调用 self." + name + ".get_status 查看再增减。",
            PropertyList({ Property("brightness", kPropertyTypeInteger, 100, 0, 100) }),
            [this](const PropertyList& properties) -> ReturnValue {
                brightness_ = Clamp(properties["brightness"].value<int>(), 0, 100);
                if (brightness_ > 0) power_ = true;
                return true;
            });

        // 颜色合并成一个字符串参数（原来是 set_color 三整数 + random_color 两个工具）
        mcp_server.AddTool("self." + name + ".set_color",
            "把" + label_ + "变成指定颜色并常亮。color 直接传颜色名字，不要传数字："
            "红 red、橙 orange、黄 yellow、绿 green、青 cyan、蓝 blue、紫 purple、"
            "粉 pink、洋红 magenta、玫红 rose、白 white、暖白 warm、冷白 cool、金 gold、"
            "棕 brown、薰衣草 lavender、天蓝 azure、靛 indigo、琥珀 amber、薄荷 mint。"
            "也接受 #RRGGBB 这种写法。传 random 表示随机换一个鲜艳颜色。"
            "用户说“变成红色”“换成暖白”“来个紫色”“换个颜色”“随机颜色”时都调用本工具。",
            PropertyList({ Property("color", kPropertyTypeString, std::string("red")) }),
            [this](const PropertyList& properties) -> ReturnValue {
                int r = base_r_, g = base_g_, b = base_b_;
                ParseColor(properties["color"].value<std::string>(), r, g, b);
                base_r_ = Clamp(r, 0, 255);
                base_g_ = Clamp(g, 0, 255);
                base_b_ = Clamp(b, 0, 255);
                color_mode_ = kColorSingle;   // 用户明确挑了颜色，特效就用这个颜色
                effect_ = kSolid;
                power_   = true;
                if (brightness_ <= 0) brightness_ = 80;
                return true;
            });

        mcp_server.AddTool("self." + name + ".set_effect",
            "让" + label_ + "播放灯光特效。effect 只能取以下值之一："
            "solid(常亮) blink(闪烁) breath(呼吸灯) flow(流水灯) marquee(跑马灯) "
            "rainbow(彩虹流动) meteor(流星) strobe(爆闪)。"
            "speed 取 1 到 10，数字越大跑得越快，用户没说就用 5。"
            "color 取 auto / rainbow / single：用户没说颜色用 auto（自动给彩色）；"
            "用户说“彩虹流水灯”“彩色跑马灯”用 rainbow；"
            "用户指定了具体颜色（如“红色流水灯”）先用 set_color 设颜色，这里传 single。"
            "用户说“流水灯”“跑马灯”“呼吸灯”“彩虹”“流星”“爆闪”“闪烁”“停止特效”时调用。",
            PropertyList({
                Property("effect", kPropertyTypeString, std::string("flow")),
                Property("speed",  kPropertyTypeInteger, 5, 1, 10),
                Property("color",  kPropertyTypeString, std::string("auto")),
            }),
            [this](const PropertyList& properties) -> ReturnValue {
                SetEffectByName(properties["effect"].value<std::string>());
                speed_ = Clamp(properties["speed"].value<int>(), 1, 10);

                // 颜色模式：auto 时按“是否设过颜色”自动决定，避免没配色时出来一条白灯
                std::string mode = properties["color"].value<std::string>();
                for (char& c : mode) {
                    c = static_cast<char>(tolower(static_cast<unsigned char>(c)));
                }
                if (mode == "rainbow" || mode == "彩色" || mode == "彩虹") {
                    color_mode_ = kColorRainbow;
                } else if (mode == "single" || mode == "单色" || mode == "solid" ||
                           mode == "current") {
                    color_mode_ = kColorSingle;
                } else {
                    // auto：纯白说明用户还没挑过颜色 → 用彩虹；否则跟随当前颜色
                    color_mode_ = (base_r_ == 255 && base_g_ == 255 && base_b_ == 255)
                                      ? kColorRainbow
                                      : kColorSingle;
                }

                power_ = true;
                if (brightness_ <= 0) brightness_ = 80;
                return true;
            });

        // 灯珠数量改成在 config.h 的 LED_STRIP_COUNT 里设，不再占一个工具名额
        // （服务端 MCP 工具总数上限 32 个，超了会被截断）

        mcp_server.AddTool("self." + name + ".get_status",
            "查询" + label_ + "当前的状态：是否亮着、亮度、颜色、灯珠数量、正在播放的特效。",
            PropertyList(),
            [this](const PropertyList& properties) -> ReturnValue {
                char buf[224];
                snprintf(buf, sizeof(buf),
                         "{\"power\":%s,\"brightness\":%d,\"color\":[%d,%d,%d],"
                         "\"effect\":\"%s\",\"speed\":%d,\"leds\":%d,\"color_mode\":\"%s\"}",
                         power_ ? "true" : "false", (int)brightness_,
                         (int)base_r_, (int)base_g_, (int)base_b_,
                         EffectName(effect_), (int)speed_, (int)active_,
                         color_mode_ == kColorRainbow ? "rainbow" : "single");
                return std::string(buf);
            });

        // 3. 起一个后台任务专门刷灯（栈 4KB，优先级低于音频任务）
        if (xTaskCreate(TaskEntry, "led_strip", 4096, this, 3, nullptr) != pdPASS) {
            ESP_LOGE("LedStrip", "刷灯任务创建失败，灯板将不工作");
        }

        ESP_LOGI("LedStrip", "%s(%s) on GPIO%d, %d leds active (buffer %d)",
                 label_.c_str(), name.c_str(), (int)gpio_num_, (int)active_, count_);
    }
};

#endif // __LED_STRIP_CONTROLLER_H__
