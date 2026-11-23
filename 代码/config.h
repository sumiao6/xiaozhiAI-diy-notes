#ifndef _BOARD_CONFIG_H_
#define _BOARD_CONFIG_H_

#include <driver/gpio.h>

#define AUDIO_INPUT_SAMPLE_RATE  16000
#define AUDIO_OUTPUT_SAMPLE_RATE 24000

// 如果使用 Duplex I2S 模式，请注释下面一行
#define AUDIO_I2S_METHOD_SIMPLEX

#ifdef AUDIO_I2S_METHOD_SIMPLEX

#define AUDIO_I2S_MIC_GPIO_WS   GPIO_NUM_4 //LRC
#define AUDIO_I2S_MIC_GPIO_SCK  GPIO_NUM_5 //BCK
#define AUDIO_I2S_MIC_GPIO_DIN  GPIO_NUM_6  //DATD
#define AUDIO_I2S_SPK_GPIO_DOUT GPIO_NUM_7
#define AUDIO_I2S_SPK_GPIO_BCLK GPIO_NUM_15
#define AUDIO_I2S_SPK_GPIO_LRCK GPIO_NUM_16

#else

#define AUDIO_I2S_GPIO_WS GPIO_NUM_4
#define AUDIO_I2S_GPIO_BCLK GPIO_NUM_5
#define AUDIO_I2S_GPIO_DIN  GPIO_NUM_6
#define AUDIO_I2S_GPIO_DOUT GPIO_NUM_7

#endif


#define BUILTIN_LED_GPIO        GPIO_NUM_48
#define BOOT_BUTTON_GPIO        GPIO_NUM_0
#define TOUCH_BUTTON_GPIO       GPIO_NUM_47
#define VOLUME_UP_BUTTON_GPIO   GPIO_NUM_40
#define VOLUME_DOWN_BUTTON_GPIO GPIO_NUM_39

#define DISPLAY_SDA_PIN GPIO_NUM_41 //屏幕sda
#define DISPLAY_SCL_PIN GPIO_NUM_42 //屏幕scl
#define DISPLAY_WIDTH   128

#if CONFIG_OLED_SSD1306_128X32
#define DISPLAY_HEIGHT  32
#elif CONFIG_OLED_SSD1306_128X64
#define DISPLAY_HEIGHT  64
#elif CONFIG_OLED_SH1106_128X64
#define DISPLAY_HEIGHT  64
#define SH1106
#else
#error "OLED display type is not selected"
#endif

#define DISPLAY_MIRROR_X true
#define DISPLAY_MIRROR_Y true


// A MCP Test: Control a lamp
#define LAMP_GPIO GPIO_NUM_18

// ===================================================================
// 舵机：PCA9685 十六路舵机驱动板（I2C）
// 舵机脉冲由 PCA9685 硬件发出，完全不占 ESP32 的 LEDC 通道，
// 所以 LEDC 的 8 个通道完整留给直流电机；舵机上限 16 路（级联可到 32/48）。
// 接线：VCC→3V3（逻辑，绝不能接 5V）、GND→主板 GND、
//       SDA→GPIO9、SCL→GPIO10、V+→独立 5~6V 电源正、电源负与主板共地、OE 悬空。
// 说明：屏幕用的是 I2C port 0（41/42），这里是另一条独立总线，互不影响。
// ===================================================================
#define SERVO_I2C_SDA   GPIO_NUM_9
#define SERVO_I2C_SCL   GPIO_NUM_10
#define PCA9685_ADDR    0x40        // A0~A5 全不短接时的默认地址

// 舵机脉宽范围（微秒）：SG90 / MG90S / MG996R 常用 500~2500，个别数字舵机 600~2400。
// 360° 连续舵机的“停止点”就是这两个数的中点；若 speed=0 还在慢慢转，
// 把两个数一起平移（如 520/2520 让中点变 1520，480/2480 变 1480）。
#define SERVO_PULSE_MIN_US 500
#define SERVO_PULSE_MAX_US 2500

// PCA9685 输出频率。舵机标准 50Hz；若买到 27MHz 晶振的模块导致抖动，
// 把 OSC 改成 27000000，预分频会自动重算。
#define PCA9685_PWM_HZ  50
#define PCA9685_OSC_HZ  25000000

// ===================================================================
// 彩灯板（WS2812 / WS2812B 单线可编程 RGB 灯珠）
// 数据线 DIN 接 GPIO21，灯板 5V 和 GND 接独立 5V 电源，GND 必须和主板共地
// 当前这款：TurboBot "Rainbow LED" 3×3 = 9 颗
// ===================================================================
#define LED_STRIP_GPIO  GPIO_NUM_21
#define LED_STRIP_COUNT 9    // 实际灯珠数（9/16/24/64…）；也能用语音改，改完存 NVS
#define LED_STRIP_MAX   64   // 缓冲区上限：这块固件最多驱动 64 颗，换更大的灯带才需要改

// ===================================================================
// 直流电机（N20 / 130 / 370 / TT 马达 / 减速电机）
// 每个电机两根信号线，PWM 直接打 IN 调速，两个脚对调即换向。
//  MRL298 / L298N 板上的 ENA、ENB 跳线帽必须保留（否则完全不转）。
// 电机用 LEDC_TIMER_1（10kHz），舵机用 LEDC_TIMER_0（50Hz），互不干扰。
// 引脚选择：1 / 2 / 38 / 18 都空闲，且避开 strapping 脚（0/3/45/46）
// ===================================================================
// 1 号电机
#define MOTOR1_IN1  GPIO_NUM_1
#define MOTOR1_IN2  GPIO_NUM_2
#define MOTOR1_INVERT 0     // 方向反了改成 1，不用拆线

// 2 号电机（右轮）
#define MOTOR2_IN1  GPIO_NUM_38
#define MOTOR2_IN2  GPIO_NUM_18   // 原来的单色小灯位，LampController 已停用
// 左右轮在车体两侧是镜像安装的：同一个信号到一个轮子向前、到另一个轮子向后。
// 所以必须有一侧反相，否则“前进”会变成原地打转。默认反相右轮。
#define MOTOR2_INVERT 1

// 3、4 号电机：舵机改走 I2C 后，GPIO8 / 11 / 12 / 13 / 14 / 17 全部空出，
// 下面这组可以直接用（取消注释 + 在 compact_wifi_board.cc 里加两行 static DcMotor）。
// ⚠️ 注意：语音工具总数上限 32，现在 29 个；再加两个电机（各 3 个工具）会到 35，
//    必须先改成带 id 的聚合工具（self.motor.run(id, ...)）才能启用。
// 还想更多引脚，只能动功能：
//   GPIO39 / GPIO40 —— 要关掉音量加/减按键（VOLUME_*_BUTTON_GPIO）
//   GPIO47          —— 要关掉触摸按键（TOUCH_BUTTON_GPIO）
//   GPIO48          —— 要关掉板载 RGB 灯（BUILTIN_LED_GPIO / GetLed）
// #define MOTOR3_IN1  GPIO_NUM_8
// #define MOTOR3_IN2  GPIO_NUM_11
// #define MOTOR3_INVERT 0
// #define MOTOR4_IN1  GPIO_NUM_12
// #define MOTOR4_IN2  GPIO_NUM_13
// #define MOTOR4_INVERT 0

#endif // _BOARD_CONFIG_H_
