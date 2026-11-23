#ifndef __PCA9685_SERVO_H__
#define __PCA9685_SERVO_H__

#include "mcp_server.h"

#include <driver/gpio.h>
#include <driver/i2c_master.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include <cstdio>
#include <cstring>
#include <string>

// ===================================================================
// PCA9685 十六路舵机总线（I2C）
//
// 一行创建，所有舵机都挂在它下面：
//     static Pca9685ServoBus servo(SERVO_I2C_SDA, SERVO_I2C_SCL,
//                                  kServoChannels, kServoCount, PCA9685_ADDR, "servo");
//
// 只注册 4 个工具，管全部舵机（以后加到 16 路，工具数也不变）：
//     self.servo.set_angle    id + angle   180° 位置舵机
//     self.servo.set_speed    id + speed   360° 连续旋转舵机
//     self.servo.stop         id           0 = 全部
//     self.servo.get_status   id           0 = 全部
//
// 设计要点：
//   1. 不 abort：初始化失败只打日志，工具照样注册，调用时返回错误说明，
//      方便用户在语音里听到"驱动板未就绪"而不是板子无限重启；
//   2. 写 I2C 加了互斥锁，两条语音指令同时下发也不会把寄存器写乱；
//   3. 总线句柄对外开放（bus_handle()），以后接 I2C 传感器可以直接并联；
//   4. 每路舵机的脉宽范围独立配置，180°/360°/数字舵机混接不用改代码。
// ===================================================================

// 一路舵机的硬件配置
struct ServoChannelDef {
    uint8_t     channel;      // PCA9685 输出通道 0~15（板子上的丝印编号）
    const char* label;        // 语音里的叫法，如 "1号"
    bool        continuous;   // true = 360° 连续旋转（调速），false = 180° 位置（调角度）
    int         min_us;       // 脉宽下限（微秒），决定 0° 或反向满速
    int         max_us;       // 脉宽上限（微秒），决定 180° 或正向满速
};

class Pca9685ServoBus {
public:
    static constexpr int kMaxChannels = 16;   // PCA9685 硬件上限

private:
    // ---- PCA9685 寄存器 ----
    static constexpr uint8_t kRegMode1    = 0x00;
    static constexpr uint8_t kRegMode2    = 0x01;
    static constexpr uint8_t kRegLed0Base = 0x06;   // 通道 n 的 ON_L 地址 = 0x06 + 4n
    static constexpr uint8_t kRegPrescale = 0xFE;

    static constexpr int kI2cTimeoutMs = 50;
    static constexpr int kPwmPeriodUs  = 20000;     // 50Hz → 20ms
    static constexpr int kPwmTicks     = 4096;      // 12 位

    i2c_master_bus_handle_t bus_   = nullptr;
    i2c_master_dev_handle_t dev_   = nullptr;
    const ServoChannelDef*  defs_  = nullptr;
    int                     count_ = 0;
    uint8_t                 addr_  = 0x40;
    int                     prescale_ = 121;
    bool                    ready_ = false;
    std::string             name_;
    std::string             roster_;
    int                     angle_[kMaxChannels];
    int                     speed_[kMaxChannels];
    SemaphoreHandle_t       lock_  = nullptr;

    // ---------------- 底层 I2C 写（调用者负责持锁） ----------------
    bool WriteRaw(const uint8_t* buf, size_t len) {
        if (dev_ == nullptr) {
            return false;
        }
        return i2c_master_transmit(dev_, buf, len, kI2cTimeoutMs) == ESP_OK;
    }

    bool WriteRegRaw(uint8_t reg, uint8_t value) {
        uint8_t buf[2] = {reg, value};
        return WriteRaw(buf, sizeof(buf));
    }

    // 写一路 PWM：full_off = true 时不输出脉冲（舵机松力、不耗电）
    bool WriteChannelRaw(int ch, int off_ticks, bool full_off) {
        if (ch < 0 || ch >= kMaxChannels) {
            return false;
        }
        uint8_t buf[5];
        buf[0] = static_cast<uint8_t>(kRegLed0Base + 4 * ch);
        buf[1] = 0;                                                  // ON_L
        buf[2] = 0;                                                  // ON_H
        buf[3] = full_off ? 0 : static_cast<uint8_t>(off_ticks & 0xFF);
        buf[4] = full_off ? 0x10 : static_cast<uint8_t>((off_ticks >> 8) & 0x0F);
        return WriteRaw(buf, sizeof(buf));                           // 依赖 MODE1 的 AI 位自动递增
    }

    // ---------------- 工具函数 ----------------
    // 微秒 → 12 位计数
    static int UsToTicks(int us) {
        if (us < 0) us = 0;
        if (us > kPwmPeriodUs) us = kPwmPeriodUs;
        return (us * kPwmTicks + kPwmPeriodUs / 2) / kPwmPeriodUs;
    }

    // 预分频 = round(振荡器频率 / (4096 × PWM 频率)) − 1
    static int CalcPrescale(int pwm_hz, int osc_hz) {
        if (pwm_hz <= 0)  pwm_hz  = 50;
        if (osc_hz <= 0)  osc_hz  = 25000000;
        int div = kPwmTicks * pwm_hz;
        int p   = (osc_hz + div / 2) / div - 1;
        if (p < 3)   p = 3;       // 手册最小 3
        if (p > 255) p = 255;
        return p;
    }

    void Lock()   { if (lock_) xSemaphoreTake(lock_, portMAX_DELAY); }
    void Unlock() { if (lock_) xSemaphoreGive(lock_); }

    // id 从 1 开始，转成数组下标
    int Find(int id) const {
        int index = id - 1;
        return (index >= 0 && index < count_) ? index : -1;
    }

    // ---------------- 初始化 ----------------
    bool InitBus(gpio_num_t sda, gpio_num_t scl) {
        if (sda == GPIO_NUM_NC || scl == GPIO_NUM_NC) {
            ESP_LOGE("Servo", "I2C 引脚未配置");
            return false;
        }

        i2c_master_bus_config_t bus_cfg = {};
        bus_cfg.i2c_port                     = -1;   // 自动选空闲控制器（屏幕已占 port 0）
        bus_cfg.sda_io_num                   = sda;
        bus_cfg.scl_io_num                   = scl;
        bus_cfg.clk_source                   = I2C_CLK_SRC_DEFAULT;
        bus_cfg.glitch_ignore_cnt            = 7;
        bus_cfg.flags.enable_internal_pullup = 1;

        esp_err_t err = i2c_new_master_bus(&bus_cfg, &bus_);
        if (err != ESP_OK) {
            ESP_LOGE("Servo", "I2C 总线创建失败(0x%X)，SDA%d/SCL%d", err, (int)sda, (int)scl);
            bus_ = nullptr;
            return false;
        }

        i2c_device_config_t dev_cfg = {};
        dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
        dev_cfg.device_address  = addr_;
        dev_cfg.scl_speed_hz    = 400 * 1000;
        err = i2c_master_bus_add_device(bus_, &dev_cfg, &dev_);
        if (err != ESP_OK) {
            ESP_LOGE("Servo", "挂载 PCA9685 失败(0x%X)", err);
            dev_ = nullptr;
            return false;
        }

        // 探测一下，接错线时能明确告诉用户是哪个地址没应答
        err = i2c_master_probe(bus_, addr_, kI2cTimeoutMs);
        if (err != ESP_OK) {
            ESP_LOGE("Servo", "0x%02X 无应答(0x%X)：查 SDA/SCL 是否接反、VCC 是否 3.3V、模块是否上电",
                     addr_, err);
            return false;
        }
        return true;
    }

    bool ChipInit() {
        // 顺序按 NXP 手册建议：先配输出方式，再进睡眠改预分频，最后唤醒并重启
        if (!WriteRegRaw(kRegMode2, 0x04)) return false;   // 推挽输出、非反相
        WriteRegRaw(kRegMode1, 0x00);                      // 唤醒
        WriteRegRaw(kRegMode1, 0x10);                      // 进睡眠（改 PRESCALE 前必须）
        if (!WriteRegRaw(kRegPrescale, static_cast<uint8_t>(prescale_))) return false;
        WriteRegRaw(kRegMode1, 0x20);                      // 唤醒 + 自动递增
        vTaskDelay(pdMS_TO_TICKS(2));                      // 等内部振荡器稳定
        WriteRegRaw(kRegMode1, 0xA0);                      // RESTART + 自动递增

        // 上电不给任何脉冲：8 只舵机同时归中会瞬间拉出几安培，容易 brownout
        for (int i = 0; i < count_; i++) {
            WriteChannelRaw(defs_[i].channel, 0, true);
            angle_[i] = -1;   // -1 = 还没动作过
            speed_[i] = 0;
        }
        return true;
    }

    std::string BuildRoster() const {
        std::string s;
        char buf[96];
        for (int i = 0; i < count_; i++) {
            snprintf(buf, sizeof(buf), "%s%s(%s)", i ? "、" : "", defs_[i].label,
                     defs_[i].continuous ? "360度旋转" : "180度位置");
            s += buf;
        }
        return s;
    }

    // 检查通道表：通道重复、越界、脉宽不合理都要在串口里说清楚
    bool ValidateDefs() const {
        bool ok = true;
        bool used[kMaxChannels] = {};
        for (int i = 0; i < count_; i++) {
            const ServoChannelDef& d = defs_[i];
            if (d.channel >= kMaxChannels) {
                ESP_LOGE("Servo", "%s 的通道号 %d 超出 0~15", d.label, (int)d.channel);
                ok = false;
                continue;
            }
            if (used[d.channel]) {
                ESP_LOGE("Servo", "通道 %d 被重复占用（%s）", (int)d.channel, d.label);
                ok = false;
            }
            used[d.channel] = true;
            if (d.min_us < 200 || d.max_us > 3000 || d.min_us >= d.max_us) {
                ESP_LOGE("Servo", "%s 的脉宽范围 %d~%d 不合理（常见 500~2500）",
                         d.label, d.min_us, d.max_us);
                ok = false;
            }
        }
        return ok;
    }

    // ---------------- 工具注册 ----------------
    void RegisterTools() {
        auto& mcp = McpServer::GetInstance();
        const std::string roster = "当前已接：" + roster_ + "。";

        mcp.AddTool("self." + name_ + ".set_angle",
            "把指定编号的 180 度位置舵机转到某个角度。id 是舵机编号，angle 取 0 到 180 度。"
            + roster +
            "用户说“3号舵机转到90度”时调用；用户说“归中”就是 90 度，说“转到最左/最右”就是 0/180。",
            PropertyList({
                Property("id",    kPropertyTypeInteger, 1,  1, count_),
                Property("angle", kPropertyTypeInteger, 90, 0, 180),
            }),
            [this](const PropertyList& p) -> ReturnValue {
                if (!ready_) {
                    return std::string("舵机驱动板未就绪，请检查 I2C 接线和 VCC 是否接 3.3V");
                }
                int index = Find(p["id"].value<int>());
                if (index < 0) {
                    return std::string("没有这个编号的舵机");
                }
                if (defs_[index].continuous) {
                    return std::string(defs_[index].label) +
                           "是360度连续旋转舵机，只能用转速控制，不能指定角度";
                }
                SetAngle(index, p["angle"].value<int>());
                return true;
            });

        mcp.AddTool("self." + name_ + ".set_speed",
            "控制指定编号的 360 度连续旋转舵机的转速。id 是舵机编号，"
            "speed 取 -100 到 100：正数正转、0 停止、负数反转，绝对值越大越快。"
            + roster +
            "用户说“让9号舵机正转”“9号舵机慢一点停下来”时调用：正转一半用 50，慢速用 25，反转用 -50。",
            PropertyList({
                Property("id",    kPropertyTypeInteger, 1, 1, count_),
                Property("speed", kPropertyTypeInteger, 0, -100, 100),
            }),
            [this](const PropertyList& p) -> ReturnValue {
                if (!ready_) {
                    return std::string("舵机驱动板未就绪，请检查 I2C 接线和 VCC 是否接 3.3V");
                }
                int index = Find(p["id"].value<int>());
                if (index < 0) {
                    return std::string("没有这个编号的舵机");
                }
                if (!defs_[index].continuous) {
                    return std::string(defs_[index].label) +
                           "是180度位置舵机，只能指定角度，不能调速";
                }
                SetSpeed(index, p["speed"].value<int>());
                return true;
            });

        mcp.AddTool("self." + name_ + ".stop",
            "让舵机停下。id 填 0 表示全部舵机一起停。" + roster,
            PropertyList({ Property("id", kPropertyTypeInteger, 0, 0, count_) }),
            [this](const PropertyList& p) -> ReturnValue {
                if (!ready_) {
                    return std::string("舵机驱动板未就绪，请检查 I2C 接线和 VCC 是否接 3.3V");
                }
                int id = p["id"].value<int>();
                if (id == 0) {
                    for (int i = 0; i < count_; i++) {
                        Stop(i);
                    }
                    return true;
                }
                int index = Find(id);
                if (index < 0) {
                    return std::string("没有这个编号的舵机");
                }
                Stop(index);
                return true;
            });

        mcp.AddTool("self." + name_ + ".get_status",
            "查询舵机当前的角度或转速。id 填 0 表示查看全部。" + roster,
            PropertyList({ Property("id", kPropertyTypeInteger, 0, 0, count_) }),
            [this](const PropertyList& p) -> ReturnValue {
                int id = p["id"].value<int>();
                if (id < 0 || id > count_) {
                    return std::string("没有这个编号的舵机");
                }
                std::string out;
                char buf[72];
                for (int i = 0; i < count_; i++) {
                    if (id != 0 && i != id - 1) continue;
                    if (!out.empty()) out += "，";
                    if (defs_[i].continuous) {
                        snprintf(buf, sizeof(buf), "%s：转速%d", defs_[i].label, speed_[i]);
                    } else if (angle_[i] < 0) {
                        snprintf(buf, sizeof(buf), "%s：未动作", defs_[i].label);
                    } else {
                        snprintf(buf, sizeof(buf), "%s：%d度", defs_[i].label, angle_[i]);
                    }
                    out += buf;
                    if (out.size() > 700) break;   // 防止超出对话返回上限
                }
                if (out.empty()) out = "没有舵机";
                return out;
            });
    }

public:
    // sda/scl  : I2C 引脚（与屏幕那条总线独立，屏幕通常占用 port 0）
    // defs     : 通道表，增删舵机只改这张表
    // count    : 表里的条目数（自动算，不要超过 16）
    // addr     : 芯片地址，A0~A5 全不短接时为 0x40
    // name     : 工具名前缀，语音端体现为 self.<name>.xxx
    Pca9685ServoBus(gpio_num_t sda, gpio_num_t scl,
                    const ServoChannelDef* defs, int count,
                    uint8_t addr = 0x40,
                    const std::string& name = "servo",
                    int pwm_hz = 50, int osc_hz = 25000000)
        : defs_(defs), count_(count), addr_(addr), name_(name) {
        for (int i = 0; i < kMaxChannels; i++) {
            angle_[i] = -1;
            speed_[i] = 0;
        }
        prescale_ = CalcPrescale(pwm_hz, osc_hz);
        lock_ = xSemaphoreCreateMutex();

        if (defs_ == nullptr || count_ <= 0) {
            ESP_LOGE("Servo", "通道表为空，舵机功能不可用");
            return;
        }
        if (count_ > kMaxChannels) {
            ESP_LOGW("Servo", "PCA9685 只有 %d 路，多余的 %d 路被忽略",
                     kMaxChannels, count_ - kMaxChannels);
            count_ = kMaxChannels;
        }
        if (lock_ == nullptr) {
            ESP_LOGW("Servo", "互斥锁创建失败，I2C 写将不加锁");
        }

        ValidateDefs();   // 有问题只告警，不阻止启动（方便对照串口排查）

        ready_ = InitBus(sda, scl) && ChipInit();
        if (!ready_) {
            ESP_LOGE("Servo", "PCA9685 初始化失败：工具仍会注册，但下发指令不会生效");
        }

        roster_ = BuildRoster();
        RegisterTools();

        ESP_LOGI("Servo", "PCA9685 @0x%02X  SDA%d/SCL%d  %d 路  %dHz(prescale %d) · %s",
                 addr_, (int)sda, (int)scl, count_, pwm_hz, prescale_,
                 ready_ ? "就绪" : "未就绪");
        for (int i = 0; i < count_; i++) {
            ESP_LOGI("Servo", "  %s → 通道%d  %s  %d~%dus", defs_[i].label,
                     (int)defs_[i].channel,
                     defs_[i].continuous ? "360°调速" : "180°角度",
                     defs_[i].min_us, defs_[i].max_us);
        }
    }

    // 总线句柄：以后接 I2C 传感器（温湿度、气压、颜色…）可以直接挂这条总线
    i2c_master_bus_handle_t bus_handle() const { return bus_; }
    bool ready() const { return ready_; }

    // index 从 0 开始（对应通道表下标）
    void SetAngle(int index, int angle) {
        if (index < 0 || index >= count_) return;
        if (angle < 0)   angle = 0;
        if (angle > 180) angle = 180;

        const ServoChannelDef& d = defs_[index];
        int us = d.min_us + (d.max_us - d.min_us) * angle / 180;

        Lock();
        angle_[index] = angle;
        WriteChannelRaw(d.channel, UsToTicks(us), false);
        Unlock();
    }

    // -100 反转满速，0 停止，+100 正转满速
    void SetSpeed(int index, int speed) {
        if (index < 0 || index >= count_) return;
        if (speed < -100) speed = -100;
        if (speed >  100) speed =  100;

        const ServoChannelDef& d = defs_[index];
        const int mid = (d.min_us + d.max_us) / 2;   // 停止点≈中位；停不住就微调 min/max

        int us;
        if (speed >= 0) {
            us = mid + (d.max_us - mid) * speed / 100;
        } else {
            us = mid + (mid - d.min_us) * speed / 100;
        }

        Lock();
        speed_[index] = speed;
        WriteChannelRaw(d.channel, UsToTicks(us), false);
        Unlock();
    }

    void Stop(int index) {
        if (index < 0 || index >= count_) return;
        const ServoChannelDef& d = defs_[index];

        if (d.continuous) {
            SetSpeed(index, 0);        // 回到中位 = 停转
            return;
        }

        Lock();
        angle_[index] = -1;
        WriteChannelRaw(d.channel, 0, true);   // 不给脉冲 → 松力，可被外力扳动
        Unlock();
    }
};

#endif // __PCA9685_SERVO_H__
