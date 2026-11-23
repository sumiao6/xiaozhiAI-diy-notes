# 源码副本

这里是从工程里复制出来的 5 个文件，**内容和工程完全一致**，方便单独打包带走或对照阅读。

| 文件 | 作用 | 对应教程章节 |
|---|---|---|
| `config.h` | 全部引脚与参数宏 | 01 章 1.7 |
| `compact_wifi_board.cc` | 板型入口，外设实例化 | 02 章 2.4 |
| `led_strip_controller.h` | 彩灯板（WS2812 + RMT） | 03 章 |
| `dc_motor_controller.h` | 直流电机 + 差速控制 | 04 章 |
| `pca9685_servo.h` | PCA9685 十六路舵机 | 05 章 |

## 怎么用

**只想看代码**：直接在这里读，教程里引用的片段都能对上。

**想实际编译**：把文件放回工程对应位置

```
xiaozhi-esp32/main/boards/bread-compact-wifi/
```

注意 `compact_wifi_board.cc` 是**完整可编译的板型文件**，直接覆盖即可（它会自动实例化全部外设）。

## 原件位置

```
<你的 xiaozhiAI-diy>\xiaozhi-diy\xiaozhi-esp32\main\boards\bread-compact-wifi\
```

**改代码请改工程里的原件，不要改这里的副本。** 这里的文件是快照，不会自动同步。

## 如果工程里的代码变了

重新复制一次即可（从本目录 `代码/` 出发，往上两级就是 `xiaozhiAI-diy`）：

```bash
cp ../../xiaozhi-diy/xiaozhi-esp32/main/boards/bread-compact-wifi/*.h ./
cp ../../xiaozhi-diy/xiaozhi-esp32/main/boards/bread-compact-wifi/*.cc ./
```
