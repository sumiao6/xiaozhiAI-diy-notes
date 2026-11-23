#include "wifi_board.h"
#include "codecs/no_audio_codec.h"
#include "display/oled_display.h"
#include "system_reset.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "mcp_server.h"
#include "lamp_controller.h"
#include "pca9685_servo.h"        // 舵机走 PCA9685（I2C）
// #include "servo_controller.h"  // 旧的 LEDC 直驱版本
#include "led_strip_controller.h"
#include "dc_motor_controller.h"
#include "led/single_led.h"
#include "assets/lang_config.h"

#include <esp_log.h>
#include <driver/i2c_master.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_vendor.h>

#ifdef SH1106
#include <esp_lcd_panel_sh1106.h>
#endif

#define TAG "CompactWifiBoard"


class CompactWifiBoard : public WifiBoard {
private:
    i2c_master_bus_handle_t display_i2c_bus_;
    esp_lcd_panel_io_handle_t panel_io_ = nullptr;
    esp_lcd_panel_handle_t panel_ = nullptr;
    Display* display_ = nullptr;
    Button boot_button_;
    Button touch_button_;
    Button volume_up_button_;
    Button volume_down_button_;

    void InitializeDisplayI2c() {
        i2c_master_bus_config_t bus_config = {
            .i2c_port = (i2c_port_t)0,
            .sda_io_num = DISPLAY_SDA_PIN,
            .scl_io_num = DISPLAY_SCL_PIN,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags = {
                .enable_internal_pullup = 1,
            },
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &display_i2c_bus_));
    }

    void InitializeSsd1306Display() {
        // SSD1306 config
        esp_lcd_panel_io_i2c_config_t io_config = {
            .dev_addr = 0x3C,
            .scl_speed_hz = 400 * 1000,
            .control_phase_bytes = 1,
            .dc_bit_offset = 6,
            .lcd_cmd_bits = 8,
            .lcd_param_bits = 8,
            .on_color_trans_done = nullptr,
            .user_ctx = nullptr,
            .flags = {
                .dc_low_on_data = 0,
                .disable_control_phase = 0,
            },
        };

        ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(display_i2c_bus_, &io_config, &panel_io_));

        ESP_LOGI(TAG, "Install SSD1306 driver");
        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = GPIO_NUM_NC;
        panel_config.bits_per_pixel = 1;

        esp_lcd_panel_ssd1306_config_t ssd1306_config = {
            .height = static_cast<uint8_t>(DISPLAY_HEIGHT),
        };
        panel_config.vendor_config = &ssd1306_config;

#ifdef SH1106
        ESP_ERROR_CHECK(esp_lcd_new_panel_sh1106(panel_io_, &panel_config, &panel_));
#else
        ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1306(panel_io_, &panel_config, &panel_));
#endif
        ESP_LOGI(TAG, "SSD1306 driver installed");

        // Reset the display
        ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_));
        if (esp_lcd_panel_init(panel_) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize display");
            display_ = new NoDisplay();
            return;
        }
        ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_, false));

        // Set the display to on
        ESP_LOGI(TAG, "Turning display on");
        ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_, true));

        display_ = new OledDisplay(panel_io_, panel_, DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);
    }

    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting) {
                EnterWifiConfigMode();
                return;
            }
            app.ToggleChatState();
        });
        touch_button_.OnPressDown([this]() {
            Application::GetInstance().StartListening();
        });
        touch_button_.OnPressUp([this]() {
            Application::GetInstance().StopListening();
        });

        volume_up_button_.OnClick([this]() {
            auto codec = GetAudioCodec();
            auto volume = codec->output_volume() + 10;
            if (volume > 100) {
                volume = 100;
            }
            codec->SetOutputVolume(volume);
            GetDisplay()->ShowNotification(Lang::Strings::VOLUME + std::to_string(volume));
        });

        volume_up_button_.OnLongPress([this]() {
            GetAudioCodec()->SetOutputVolume(100);
            GetDisplay()->ShowNotification(Lang::Strings::MAX_VOLUME);
        });

        volume_down_button_.OnClick([this]() {
            auto codec = GetAudioCodec();
            auto volume = codec->output_volume() - 10;
            if (volume < 0) {
                volume = 0;
            }
            codec->SetOutputVolume(volume);
            GetDisplay()->ShowNotification(Lang::Strings::VOLUME + std::to_string(volume));
        });

        volume_down_button_.OnLongPress([this]() {
            GetAudioCodec()->SetOutputVolume(0);
            GetDisplay()->ShowNotification(Lang::Strings::MUTED);
        });
    }

    // 物联网初始化，逐步迁移到 MCP 协议
    void InitializeTools() {
        // 板载单色小灯（GPIO18）。现在有了彩灯板，两个都叫“灯”会让语音分不清，
        // static LampController lamp(LAMP_GPIO);

        // 字段：PCA9685 通道号、语音称呼、是否360度、脉宽下限、脉宽上限
        static const ServoChannelDef kServoChannels[] = {
            { 0, "1号", false, SERVO_PULSE_MIN_US, SERVO_PULSE_MAX_US },   // 180° 位置
            { 1, "2号", false, SERVO_PULSE_MIN_US, SERVO_PULSE_MAX_US },
            { 2, "3号", false, SERVO_PULSE_MIN_US, SERVO_PULSE_MAX_US },
            { 3, "4号", false, SERVO_PULSE_MIN_US, SERVO_PULSE_MAX_US },
            { 4, "5号", false, SERVO_PULSE_MIN_US, SERVO_PULSE_MAX_US },
            { 5, "6号", false, SERVO_PULSE_MIN_US, SERVO_PULSE_MAX_US },
            { 6, "7号", false, SERVO_PULSE_MIN_US, SERVO_PULSE_MAX_US },
            { 7, "8号", false, SERVO_PULSE_MIN_US, SERVO_PULSE_MAX_US },
            { 8, "9号", true,  SERVO_PULSE_MIN_US, SERVO_PULSE_MAX_US },   // 360° 舵机1
            { 9, "10号", true,  SERVO_PULSE_MIN_US, SERVO_PULSE_MAX_US },   // 360° 舵机2
            { 10, "11号", true,  SERVO_PULSE_MIN_US, SERVO_PULSE_MAX_US },   // 360° 舵机3
        };
        constexpr int kServoCount = sizeof(kServoChannels) / sizeof(kServoChannels[0]);

        static Pca9685ServoBus servo(SERVO_I2C_SDA, SERVO_I2C_SCL,
                                     kServoChannels, kServoCount,
                                     PCA9685_ADDR, "servo",
                                     PCA9685_PWM_HZ, PCA9685_OSC_HZ);

        // 彩灯板：参数 = 引脚、缓冲区上限、工具名、语音称呼、实际灯珠数
        static LedStripController light(LED_STRIP_GPIO, LED_STRIP_MAX, "light", "彩灯板",
                                       LED_STRIP_COUNT);

        // 电机：一行一个，语音里就叫“1号电机”“2号电机”
        // 参数：IN1 脚、IN2 脚、工具名、语音称呼、方向是否取反
        static DcMotor motor1(MOTOR1_IN1, MOTOR1_IN2, "motor1", "1号电机", MOTOR1_INVERT);
        static DcMotor motor2(MOTOR2_IN1, MOTOR2_IN2, "motor2", "2号电机", MOTOR2_INVERT);
        // 想加 3、4 号：先按 config.h 注释腾出引脚，再照抄一行

        // 差速小车：把两个电机交给它，就能“前进 / 后退 / 左转 / 右转”
        static CarController car(motor1, motor2, "car", "小车");
    }


public:
    CompactWifiBoard() :
        boot_button_(BOOT_BUTTON_GPIO),
        touch_button_(TOUCH_BUTTON_GPIO),
        volume_up_button_(VOLUME_UP_BUTTON_GPIO),
        volume_down_button_(VOLUME_DOWN_BUTTON_GPIO) {
        InitializeDisplayI2c();
        InitializeSsd1306Display();
        InitializeButtons();
        InitializeTools();
    }

    virtual Led* GetLed() override {
        static SingleLed led(BUILTIN_LED_GPIO);
        return &led;
    }

    virtual AudioCodec* GetAudioCodec() override {
#ifdef AUDIO_I2S_METHOD_SIMPLEX
        static NoAudioCodecSimplex audio_codec(AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_SPK_GPIO_BCLK, AUDIO_I2S_SPK_GPIO_LRCK, AUDIO_I2S_SPK_GPIO_DOUT, AUDIO_I2S_MIC_GPIO_SCK, AUDIO_I2S_MIC_GPIO_WS, AUDIO_I2S_MIC_GPIO_DIN);
#else
        static NoAudioCodecDuplex audio_codec(AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_GPIO_BCLK, AUDIO_I2S_GPIO_WS, AUDIO_I2S_GPIO_DOUT, AUDIO_I2S_GPIO_DIN);
#endif
        return &audio_codec;
    }

    virtual Display* GetDisplay() override {
        return display_;
    }
};

DECLARE_BOARD(CompactWifiBoard);




