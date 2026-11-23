#ifndef __DC_MOTOR_CONTROLLER_H__
#define __DC_MOTOR_CONTROLLER_H__

#include "mcp_server.h"

#include <driver/gpio.h>
#include <driver/ledc.h>
#include <esp_log.h>
#include <esp_timer.h>

#include <cctype>
#include <cstdio>
#include <string>

// ===================================================================
// 直流电机 + 差速小车（一套代码兼容市面主流驱动板）
//
// 两种接线方式（Mode）：
//   1. kInIn —— 两线，每个电机 2 个 GPIO、2 个 LEDC 通道（默认，最通用）
//      PWM 打到 IN1、IN2 给低 → 正转；两个对调 → 反转。
//      适用：ZK-BM1、L9110S、DRV8833、TB6612、L298N/MRL298（ENA/ENB 跳线帽必须在）
//   2. kEnIn —— 三线，每个电机 3 个 GPIO，但只占 1 个 LEDC 通道
//      EN 接 PWM 调速，IN1/IN2 接普通 GPIO 当方向电平。
//      适用：L298N 拔掉跳线帽、TB6612 的 STBY、带 EN/nSLEEP 脚的模块
//      好处：4 个电机只占 4 个通道，能给舵机留位置
//
// 不适用（要另写驱动）：
//   - 步进电机（28BYJ-48 + ULN2003、A4988/DRV8825）：四相脉冲时序，不是 H 桥 PWM
//   - 无刷电机 + 电调（ESC）：电调吃 50Hz 舵机信号，直接用 ServoController 的
//     360° 连续模式，把 speed 映射成 1000~2000us 就行
//
// 资源分配（和舵机、灯板互不冲突）：
//   舵机 : LEDC_TIMER_0, 50Hz,  通道 0 往上（servo_controller.h）
//   电机 : LEDC_TIMER_1, 10kHz, 通道 7 往下（本文件）
//   灯板 : RMT，不占 LEDC
//
// 用法（一行一个电机，可以叫“1号”“2号”）：
//   static DcMotor motor1(MOTOR1_IN1, MOTOR1_IN2, "motor1", "1号电机");
//   static DcMotor motor2(MOTOR2_IN1, MOTOR2_IN2, "motor2", "2号电机");
// 装上轮子要做小车时，再把这两个电机交给 CarController：
//   static CarController car(motor1, motor2, "car", "小车");
// ===================================================================

class DcMotor {
public:
    // kInIn : 两线 IN1/IN2（默认，最通用）
    // kEnIn : 三线 EN + IN1/IN2（省 LEDC 通道，需要驱动板有 EN/STBY 脚）
    enum Mode : int {
        kInIn = 0,
        kEnIn = 1,
    };

private:
    Mode           mode_     = kInIn;
    gpio_num_t     in1_      = GPIO_NUM_NC;   // kInIn: IN1；kEnIn: IN1（电平）
    gpio_num_t     in2_      = GPIO_NUM_NC;   // kInIn: IN2；kEnIn: IN2（电平）
    gpio_num_t     en_pin_   = GPIO_NUM_NC;   // 只有 kEnIn 用
    ledc_channel_t ch1_      = LEDC_CHANNEL_7;
    ledc_channel_t ch2_      = LEDC_CHANNEL_6;
    bool           invert_   = false;         // 方向反了改这里，不用拆线
    bool           ready_    = false;

    int level_ = 5;    // 档位 1~10
    int dir_   = 0;    // 1 正转, -1 反转, 0 停
    int pct_   = 0;    // 最后一次实际输出的百分比（-100~100）

    std::string        name_;
    std::string        label_;
    esp_timer_handle_t timer_ = nullptr;

    static constexpr int kFreqHz      = 10000;  // 10kHz：L298N 跟得上，也在 ZK-BM1 的 20kHz 上限内
    static constexpr int kDutyBits    = 12;     // 12 位 = 4096 级
    static constexpr int kDutyMax     = 4095;
    static constexpr int kMinDrivePct = 28;     // 起步占空比：低于它 N20 堵转不动

    static void EnsureTimer() {
        static bool inited = false;
        if (inited) {
            return;
        }
        ledc_timer_config_t timer = {
            .speed_mode      = LEDC_LOW_SPEED_MODE,
            .duty_resolution = static_cast<ledc_timer_bit_t>(kDutyBits),
            .timer_num       = LEDC_TIMER_1,
            .freq_hz         = kFreqHz,
            .clk_cfg         = LEDC_AUTO_CLK,
        };
        ESP_ERROR_CHECK(ledc_timer_config(&timer));
        inited = true;
    }

    // 通道从 7 往下分：7、6、5、4。舵机从 0 往上，两边留出余量。
    static ledc_channel_t AllocChannel() {
        static int next = LEDC_CHANNEL_MAX - 1;
        if (next < 4) {
            ESP_LOGE("Motor", "LEDC 通道用尽（舵机太多），电机可能无法输出");
            return LEDC_CHANNEL_4;
        }
        return static_cast<ledc_channel_t>(next--);
    }

    static int PctToDuty(int pct) {
        if (pct <= 0) {
            return 0;
        }
        if (pct >= 100) {
            return kDutyMax;
        }
        int real = kMinDrivePct + (100 - kMinDrivePct) * pct / 100;   // 28 ~ 100
        return real * kDutyMax / 100;
    }

    static void WriteDuty(ledc_channel_t ch, int pct) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, ch, PctToDuty(pct));
        ledc_update_duty(LEDC_LOW_SPEED_MODE, ch);
    }

    static void OnTimer(void* arg) {
        static_cast<DcMotor*>(arg)->Stop();
    }

    void EnsureStopTimer() {
        if (timer_ != nullptr) {
            return;
        }
        esp_timer_create_args_t args = {};
        args.callback = &DcMotor::OnTimer;
        args.arg      = this;
        args.name     = "motor_stop";
        esp_timer_create(&args, &timer_);
    }

    void ArmTimeout(int seconds) {
        EnsureStopTimer();
        if (timer_ == nullptr) {
            return;
        }
        esp_timer_stop(timer_);
        if (seconds > 0) {
            esp_timer_start_once(timer_, static_cast<uint64_t>(seconds) * 1000000ULL);
        }
    }

    // 注册给大模型的工具（name 为空表示不注册）
    void RegisterTools() {
        if (name_.empty() || !ready_) {
            return;
        }
        auto& mcp = McpServer::GetInstance();
        const std::string base = "self." + name_;

        mcp.AddTool(base + ".run",
            "让" + label_ + "转起来。direction 取 forward(正转) / backward(反转) / stop(停)；"
            "level 是速度档 1 到 10，填 0 表示沿用当前档位。"
            "seconds 可选，表示转几秒后自动停（0 或不给就是一直转）。"
            "用户说“1号电机正转”“反转”“转三秒”时调用；"
            "只说了快慢没说方向，就先调用 speed 调档，不要调这个工具。",
            PropertyList({
                Property("direction", kPropertyTypeString, std::string("forward")),
                Property("level",     kPropertyTypeInteger, 0, 0, 10),
                Property("seconds",   kPropertyTypeInteger, 0, 0, 60),
            }),
            [this](const PropertyList& properties) -> ReturnValue {
                std::string dir = properties["direction"].value<std::string>();
                for (char& c : dir) {
                    c = static_cast<char>(tolower(static_cast<unsigned char>(c)));
                }
                int level = properties["level"].value<int>();
                if (level >= 1 && level <= 10) {
                    level_ = level;
                }
                if (dir == "stop" || dir == "停") {
                    Stop();
                    return std::string("stopped");
                }
                dir_ = (dir == "backward" || dir == "reverse" || dir == "反转") ? -1 : 1;
                EnsureStopTimer();
                Pct(dir_ * level_ * 10);
                ArmTimeout(properties["seconds"].value<int>());

                char buf[96];
                snprintf(buf, sizeof(buf), "{\"direction\":\"%s\",\"level\":%d}",
                         dir_ > 0 ? "forward" : "backward", level_);
                return std::string(buf);
            });

        // 速度调节合并：原 faster / slower / set_speed 三个工具 → 一个 speed
        // （服务端 MCP 工具总数上限 32，超了会被截断，所以这里做合并）
        mcp.AddTool(base + ".speed",
            "调节" + label_ + "的速度档位（1 到 10）。delta 是相对变化量："
            "用户说“快一点”“再快一点”“加速”传 1；说“慢一点”“减速”传 -1；"
            "想一次跳两级就传 2。已经在最高或最低档时保持不变。"
            "正在转的话会立刻生效。要精确指定某一档，请改用 run 的 level 参数。",
            PropertyList({ Property("delta", kPropertyTypeInteger, 1, -9, 9) }),
            [this](const PropertyList& properties) -> ReturnValue {
                int target = level_ + properties["delta"].value<int>();
                if (target < 1) target = 1;
                if (target > 10) target = 10;
                level_ = target;
                if (dir_ != 0) {
                    Pct(dir_ * level_ * 10);
                }
                return level_;
            });

        mcp.AddTool(base + ".stop",
            "让" + label_ + "立刻停下。用户说“停”“停下”“别转了”时调用。",
            PropertyList(),
            [this](const PropertyList& properties) -> ReturnValue {
                Stop();
                return true;
            });
    }

    void InitOutputs() {
        if (mode_ == kEnIn) {
            if (en_pin_ == GPIO_NUM_NC || in1_ == GPIO_NUM_NC || in2_ == GPIO_NUM_NC) {
                return;
            }
            EnsureTimer();

            // 方向脚：普通 GPIO 电平输出
            gpio_config_t cfg = {};
            cfg.pin_bit_mask = (1ULL << in1_) | (1ULL << in2_);
            cfg.mode         = GPIO_MODE_OUTPUT;
            cfg.pull_up_en   = GPIO_PULLUP_DISABLE;
            cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
            cfg.intr_type    = GPIO_INTR_DISABLE;
            gpio_config(&cfg);
            gpio_set_level(in1_, 0);
            gpio_set_level(in2_, 0);

            // EN 脚：LEDC PWM，占 1 个通道
            ch1_ = AllocChannel();
            ledc_channel_config_t ch = {
                .gpio_num   = en_pin_,
                .speed_mode = LEDC_LOW_SPEED_MODE,
                .channel    = ch1_,
                .timer_sel  = LEDC_TIMER_1,
                .duty       = 0,
                .hpoint     = 0,
            };
            ESP_ERROR_CHECK(ledc_channel_config(&ch));
            ready_ = true;
            return;
        }

        if (in1_ == GPIO_NUM_NC || in2_ == GPIO_NUM_NC) {
            return;
        }
        EnsureTimer();
        ch1_ = AllocChannel();
        ch2_ = AllocChannel();
        ledc_channel_config_t c1 = {
            .gpio_num   = in1_,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel    = ch1_,
            .timer_sel  = LEDC_TIMER_1,
            .duty       = 0,
            .hpoint     = 0,
        };
        ledc_channel_config_t c2 = {
            .gpio_num   = in2_,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel    = ch2_,
            .timer_sel  = LEDC_TIMER_1,
            .duty       = 0,
            .hpoint     = 0,
        };
        ESP_ERROR_CHECK(ledc_channel_config(&c1));
        ESP_ERROR_CHECK(ledc_channel_config(&c2));
        ready_ = true;
    }

public:
    // ---------- 两线模式：IN1 / IN2 都接 PWM ----------
    DcMotor(gpio_num_t in1, gpio_num_t in2,
            const std::string& name  = "",
            const std::string& label = "",
            bool invert = false)
        : mode_(kInIn), in1_(in1), in2_(in2), invert_(invert),
          name_(name), label_(label.empty() ? name : label) {
        InitOutputs();
        RegisterTools();
        if (ready_) {
            ESP_LOGI("Motor", "%s(%s) IN-IN GPIO%d/%d, LEDC ch%d/%d%s",
                     label_.c_str(), name_.c_str(), (int)in1_, (int)in2_,
                     (int)ch1_, (int)ch2_, invert_ ? " inverted" : "");
        }
    }

    // ---------- 三线模式：EN(PWM) + IN1/IN2(电平) ----------
    DcMotor(gpio_num_t en, gpio_num_t in1, gpio_num_t in2,
            const std::string& name,
            const std::string& label = "",
            bool invert = false)
        : mode_(kEnIn), in1_(in1), in2_(in2), en_pin_(en), invert_(invert),
          name_(name), label_(label.empty() ? name : label) {
        InitOutputs();
        RegisterTools();
        if (ready_) {
            ESP_LOGI("Motor", "%s(%s) EN-IN EN=%d IN %d/%d, LEDC ch%d%s",
                     label_.c_str(), name_.c_str(), (int)en_pin_, (int)in1_, (int)in2_,
                     (int)ch1_, invert_ ? " inverted" : "");
        }
    }

    // 直接给百分比（-100~100），CarController 用它做差速
    void Pct(int percent) {
        if (!ready_) {
            return;
        }
        if (percent > 100)  percent = 100;
        if (percent < -100) percent = -100;
        pct_ = percent;
        dir_ = percent > 0 ? 1 : (percent < 0 ? -1 : 0);

        int out = invert_ ? -percent : percent;

        if (mode_ == kEnIn) {
            if (out > 0) {
                gpio_set_level(in1_, 1);
                gpio_set_level(in2_, 0);
            } else if (out < 0) {
                gpio_set_level(in1_, 0);
                gpio_set_level(in2_, 1);
            } else {
                gpio_set_level(in1_, 0);
                gpio_set_level(in2_, 0);
            }
            WriteDuty(ch1_, out < 0 ? -out : out);
            return;
        }

        if (out > 0) {
            WriteDuty(ch1_, out);
            WriteDuty(ch2_, 0);
        } else if (out < 0) {
            WriteDuty(ch1_, 0);
            WriteDuty(ch2_, -out);
        } else {
            WriteDuty(ch1_, 0);
            WriteDuty(ch2_, 0);
        }
    }

    void Stop() {
        if (timer_ != nullptr) {
            esp_timer_stop(timer_);
        }
        dir_ = 0;
        Pct(0);
    }

    int  percent() const { return pct_; }
    int  level() const { return level_; }
    bool ready() const { return ready_; }
    const std::string& label() const { return label_; }
};

// ============================ 差速小车 ============================
class CarController {
private:
    enum Action : int {
        kStop = 0,
        kForward,
        kBackward,
        kTurnLeft,
        kTurnRight,
        kSpinLeft,
        kSpinRight,
    };

    DcMotor&           left_;
    DcMotor&           right_;
    esp_timer_handle_t timer_  = nullptr;
    int                speed_  = 5;      // 1~10 级
    int                action_ = kStop;
    std::string        label_;
    std::string        name_;

    int BasePct() const { return speed_ * 10; }

    void Apply() {
        const int base  = BasePct();
        int inner = base * 30 / 100;     // 转向时内侧轮 = 外侧的 30%
        if (inner < 1) {
            inner = 1;
        }
        switch (action_) {
            case kForward:   left_.Pct(base);   right_.Pct(base);  break;
            case kBackward:  left_.Pct(-base);  right_.Pct(-base); break;
            case kTurnLeft:  left_.Pct(inner);  right_.Pct(base);  break;   // 左轮慢 → 左拐
            case kTurnRight: left_.Pct(base);   right_.Pct(inner); break;
            case kSpinLeft:  left_.Pct(-base);  right_.Pct(base);  break;
            case kSpinRight: left_.Pct(base);   right_.Pct(-base); break;
            default:         left_.Stop(); right_.Stop(); break;
        }
    }

    void StopAll() {
        if (timer_ != nullptr) {
            esp_timer_stop(timer_);
        }
        action_ = kStop;
        left_.Stop();
        right_.Stop();
    }

    static void OnTimer(void* arg) {
        static_cast<CarController*>(arg)->StopAll();
    }

    void ArmTimeout(int seconds) {
        if (timer_ == nullptr) {
            esp_timer_create_args_t args = {};
            args.callback = &CarController::OnTimer;
            args.arg      = this;
            args.name     = "car_stop";
            esp_timer_create(&args, &timer_);
        }
        if (timer_ == nullptr) {
            return;
        }
        esp_timer_stop(timer_);
        if (seconds > 0) {
            esp_timer_start_once(timer_, static_cast<uint64_t>(seconds) * 1000000ULL);
        }
    }

    static const char* ActionName(int action) {
        switch (action) {
            case kForward:   return "forward";
            case kBackward:  return "backward";
            case kTurnLeft:  return "turn_left";
            case kTurnRight: return "turn_right";
            case kSpinLeft:  return "spin_left";
            case kSpinRight: return "spin_right";
            default:         return "stop";
        }
    }

    // 把大模型传来的动作名折成内部枚举
    static int ParseAction(const std::string& raw) {
        std::string a;
        for (char c : raw) {
            a += static_cast<char>(tolower(static_cast<unsigned char>(c)));
        }
        auto has = [&a](const char* kw) { return a.find(kw) != std::string::npos; };

        const bool left  = has("left")  || has("左");
        const bool right = has("right") || has("右");

        if (has("掉头")) {
            return kSpinLeft;
        }
        if (has("spin") || has("原地") || has("转圈")) {
            return right ? kSpinRight : kSpinLeft;
        }
        if (left) {
            return kTurnLeft;
        }
        if (right) {
            return kTurnRight;
        }
        if (has("back") || has("reverse") || has("后") || has("退")) {
            return kBackward;
        }
        return kForward;
    }

    // 只注册 4 个工具：move / speed / stop / get_status
    // （原先是 6 个方向工具 + 3 个调速工具 + stop + 状态 = 11 个，会撑爆服务端 32 个上限）
    void RegisterTools() {
        auto& mcp = McpServer::GetInstance();

        mcp.AddTool("self." + name_ + ".move",
            "让" + label_ + "动起来。action 取以下值之一："
            "forward(前进) backward(后退) turn_left(左转，走弧线) turn_right(右转，走弧线) "
            "spin_left(原地左转、掉头) spin_right(原地右转)；"
            "level 是速度档 1 到 10，填 0 表示沿用当前档位；"
            "seconds 可选，跑几秒后自动停（0 到 60，0 或不给就是一直跑）。"
            "用户说“前进”“往前走”“后退”“左转”“右转”“掉头”“原地转”时调用。",
            PropertyList({
                Property("action",  kPropertyTypeString, std::string("forward")),
                Property("level",   kPropertyTypeInteger, 0, 0, 10),
                Property("seconds", kPropertyTypeInteger, 0, 0, 60),
            }),
            [this](const PropertyList& properties) -> ReturnValue {
                int level = properties["level"].value<int>();
                if (level >= 1 && level <= 10) {
                    speed_ = level;
                }
                action_ = ParseAction(properties["action"].value<std::string>());
                Apply();
                ArmTimeout(properties["seconds"].value<int>());
                return true;
            });

        // 原 set_speed / speed_up / speed_down 三个工具 → 一个 speed
        mcp.AddTool("self." + name_ + ".speed",
            "调节" + label_ + "的速度档位（1 到 10）。delta 是相对变化量："
            "用户说“快一点”“再快一点”“加速”传 1；说“慢一点”“减速”传 -1。"
            "正在走的话会立刻按新速度继续。要精确指定某一档，用 move 的 level 参数。",
            PropertyList({ Property("delta", kPropertyTypeInteger, 1, -9, 9) }),
            [this](const PropertyList& properties) -> ReturnValue {
                int target = speed_ + properties["delta"].value<int>();
                if (target < 1) target = 1;
                if (target > 10) target = 10;
                speed_ = target;
                Apply();
                return speed_;
            });

        mcp.AddTool("self." + name_ + ".stop",
            "让" + label_ + "立刻停下来。用户说“停”“停车”“别动了”时调用。",
            PropertyList(),
            [this](const PropertyList& properties) -> ReturnValue {
                StopAll();
                return true;
            });

        mcp.AddTool("self." + name_ + ".get_status",
            "查询" + label_ + "当前在做什么、速度几级。",
            PropertyList(),
            [this](const PropertyList& properties) -> ReturnValue {
                char buf[128];
                snprintf(buf, sizeof(buf), "{\"action\":\"%s\",\"speed\":%d}",
                         ActionName(action_), speed_);
                return std::string(buf);
            });
    }

public:
    // 两个电机必须是已经构造好的 DcMotor（它们自己也会注册“1号/2号”工具）
    CarController(DcMotor& left, DcMotor& right,
                  const std::string& name  = "car",
                  const std::string& label = "小车")
        : left_(left), right_(right), label_(label), name_(name) {
        RegisterTools();
        ESP_LOGI("Car", "%s(%s) ready (move/speed/stop/status)", label_.c_str(), name_.c_str());
    }
};

#endif // __DC_MOTOR_CONTROLLER_H__
