# Gsimulator

STM32F103 片上外设模拟器（第一阶段）。

用 Rust 实现的、可在 PC 上运行真实 ARM Cortex-M3 固件 ELF 的 STM32F103 外设模拟器：
外设模型按 RM0008 寄存器级建模，CPU 指令集来自开源生态项目
[armagnac](https://github.com/Ledger-Donjon/armagnac)（纯 Rust 的 Thumb/ARMv7-M 解释器），
ELF 解析来自 [object](https://github.com/gimli-rs/object)。

本阶段范围：**F103 的片上外设 + 两个真实片外器件**。CAN1 控制器（bxCAN）与
`gsim-devices` 里的 MAX7219 点阵、CAN 收发器/虚拟总线已实现；CAN2/USB、外部 I2C 器件、
位级仲裁仍不在范围内（见文末的已知简化）。

项目由 6 个 crate 组成：

| crate | 作用 |
| --- | --- |
| `gsim-core` | 与芯片无关的基础设施（总线、NVIC/SCB/SysTick、事件、时间） |
| `gsim-stm32f1` | STM32F103 片上外设模型 |
| `gsim-devices` | **片外器件模型**：UART 终端、旋转编码器、MAX7219、CAN 收发器/位级总线、电阻、示波器 |
| `gsim-cpu` | Cortex-M3 执行前端 + ELF 装载 |
| `gsim-gdb` | **虚拟 ST-Link**：GDB 远程协议服务器（烧录 + 调试） |
| `gsimulator-cli` | 命令行（`run` / `gdb` / `demo` / `info`） |
| `gsimulator-gui` | **桌面图形前端**：PCB/原理图风格的虚拟接线台 |

---

## 0. 图形前端（虚拟接线台）

![Gsimulator 图形前端](docs/gui.png)

*（上图是打开 [`docs/example.gsim`](docs/example.gsim) 后的样子；新建工程默认只有一颗 HSE 晶振，摆在芯片左侧 PD0/PD1 旁并已接好线。）*

```powershell
cargo run -p gsimulator-gui                     # 新工程：自动摆好 HSE 晶振，其余从器件库放置
cargo run -p gsimulator-gui -- path\to\app.elf  # 用真实固件的引脚输出驱动外设
```

界面上可放置的器件全部来自工具栏 **＋ 添加器件** 打开的「器件库」窗口（分类 + 搜索，
像嘉立创 EDA 的器件面板）：LED、UART 终端、旋转编码器、`VCC`/`GND` 电源符号、
CAN 收发器、SPI 灯阵（MAX7219 8×8）、HSE/LSE 晶振、接线节点。见下文“可放置的虚拟外设”。

界面用 [egui/eframe](https://github.com/emilk/egui)（纯 Rust 立即模式 GUI）实现，分成几块：

* **画布**：中间是方形的 STM32F103C8T6 符号，按 **LQFP48 真实引脚排列**绘制
  （1–12 在左边自上而下、13–24 在底边自左向右、25–36 在右边自下而上、37–48 在顶边自右向左，
  外侧标引脚号），其中 37 个信号脚（PA0–PA15 / PB0–PB15 / PC13–PC15 / PD0–PD1）可连线，
  电源与复位脚（VBAT、VSS、VDD、VSSA、VDDA、NRST、BOOT0）按数据手册位置画出但不参与仿真；
  每个信号脚带电平指示。
* **器件库（＋ 添加器件）**：窗口按「电源 / 时钟 / 指示 / 通信 / 交互 / 显示」分类列出所有可放置的器件，
  带符号预览、引脚名和搜索框。点卡片上的「放置」后画布跟随鼠标显示半透明符号，
  单击落点即可放下（`Esc` 结束）；也可以直接把卡片拖到画布上。
  电源符号可以放任意多个，和器件一样能拖动、能 `Delete` 删除。
  符号的引脚按**功能分区**排布：与 MCU 相连的信号（`RX`/`TX`/`A`/`B`/`SW`/`TXD`/`RXD`/`DIN`/`CLK`/`CS`…）在**左侧**，
  总线与输出（`CANH`/`CANL`、`DOUT`、LED 的 `K`）在**右侧**，`VCC` 与 `GND` 都在**顶边**（`VCC` 靠左、`GND` 靠右）。
* **接线（类似嘉立创 EDA / EasyEDA）**：点一个引脚 → 点另一个引脚 即完成一根连线；
  也可以从一个引脚按住拖到目标引脚。连线自动走**正交折线**，从引脚沿符号短桩出线，
  并且**绕开画布上所有器件的外形**（布线在 `crates/gsimulator-gui/src/route.rs`，
  先试直角/Z 形，拥挤时才走 A\*）；颜色跟随网络电平（绿=高、蓝=低、红=短路、暗=悬空）。
  **自定义拐点**：选中连线后拖动线段即可拉出一个拐点，拐点可以拖动（2 单位栅格吸附），
  选中拐点按 `Delete` 删掉它。
  **接线节点（分支点）**：双击一根连线就在点击处插入一个节点，原来的一根线在那里分成两段，
  节点本身是一个可点选的**接线端子**——点它就能再引一根线出去（例如从地线中间取一根到 LED 阴极），
  画布上它显示成导线上的圆点；双击这个圆点即删除节点，两段线又并回一根。
  `Delete` 删除选中的连线或器件，中键拖动平移，滚轮缩放。
* **状态面板**：仿真时间、实时倍率、指令数、中断/复位次数、总线访问与错误、完整时钟树
  （SYSCLK/HCLK/PCLK1/PCLK2/TIM2/ADC）、每个引脚的高低电平矩阵、虚拟外设列表（点亮状态 + 接线关系）、
  USART1 输出窗口。
* **工程文件（💾 保存工程… / 📂 打开工程…，`Ctrl+S` / `Ctrl+O`）**：把整张原理图
  （器件类型与位置、每根连线及其拐点、电阻阻值/终端串口号/CAN 模式/示波器时基等器件选项）
  存成可读的 `.gsim` 文本文件，下次打开即原样恢复；文件是逐行 UTF-8 文本，
  可以直接看 diff、也能手改（格式见下）。仓库自带两份示例工程：
  [`docs/example.gsim`](docs/example.gsim)（VCC/GND、LED/PA5、UART1、编码器、HSE/LSE 晶振）
  和 [`docs/selftest.gsim`](docs/selftest.gsim)（`firmware/CAN` 的自检固件用的整套接线，
  见第 1 节），用「📂 打开工程…」选它就能得到文档里的演示接线。

驱动引脚的两个来源：

| 来源 | 说明 |
| --- | --- |
| 内置激励 | 默认关闭，点工具栏「内置激励」才启动：前端按配置的引脚与波形通过 `GPIOx->BSRR` 翻转（走真实寄存器路径，不是伪造电平），默认 PA5、120 ms 半周期方波 |
| 真实固件 | `加载 ELF…` 选择一个 Cortex-M3 ELF，前端在 UI 线程内按预算执行指令并刷新状态 |

工具栏的「内置激励」是一个开关：再次点击即停止波形、把引脚置低并交还为输入。
**仿真时钟不依赖激励**——停止后只是没有引脚被驱动，`运行/暂停` 照常推进时钟，
外设、虚拟外设与 CAN 总线都继续工作（复位也不会把激励重新打开）。
旁边的「⚙ 激励」打开配置窗口：

* **引脚**：PA0–PA15 / PB0–PB15 / PC13–PC15（画面上的全部引脚）；
* **波形**：方波、恒定高电平、恒定低电平、自定义函数；
* **方波**：半周期（1 µs–5 s，窗口里同时显示频率）与占空比（1 %–99 %）；
* **自定义函数**：用表达式描述电平，变量 `t`（秒）、`ms`、`us`，常量 `pi`/`tau`/`e`，
  函数 `sin/cos/tan/sqrt/abs/floor/ceil/round/fract/min/max/pow/clamp/step`，
  支持 `+ - * / % ^`、比较（`< <= > >= == !=`）与 `&& || !`；表达式结果 **> 0.5 即为高电平**，
  例如 `sin(2*pi*2*t) > 0` 是 2 Hz 方波、`t >= 0.1 && t < 0.3` 是 100–300 ms 的脉冲。
  函数按 1 ms 采样，非法表达式会在窗口里提示且不会影响正在运行的激励。

电气规则（`crates/gsimulator-gui/src/board.rs`，全部有单元测试）：
连线把引脚并成网络，网络电平 = 高/低/悬空/短路；LED 亮 = 阳极所在网络为高 **且** 阴极所在网络为低。
所以 `PA5 → LED.A`、`GND → LED.K` 是高电平点亮，`VCC → LED.A`、`PC13 → LED.K` 则是低电平点亮
（Blue Pill 板载 LED 就是这种接法）。

### 可放置的虚拟外设

| 外设 | 引脚 | 行为 | 操作 |
| --- | --- | --- | --- |
| **LED** | `A`（阳） `K`（阴） | 纯消费端，按网络电平点亮 | — |
| **UART 终端** | `RX` `TX` `VCC` `GND` | 把 MCU 在所选 USART 上**真实移位输出**的字节显示成文本，并把输入的文本送进该 USART 的接收器 | 面板里选 USART1/2/3、输入后回车或点“发送”、“清空” |
| **旋转编码器** | `A` `B` `SW` `VCC` `GND` | 正交编码器（自带 10 k 上拉，接触点被主动驱动）：一个定位点 = 4 个四分之一相位（格雷码），每 2 ms 释放一个边沿；轴按下时把 `SW` 拉低 | 画布上**滚轮**转旋钮、**点击**按轴；也可用面板按钮 |
| **VCC / GND 电源** | `+` / `-` | 电源符号：接到的整个网络被拉成高电平 / 低电平（`VCC` 和 `GND` 接到同一网络即短路告警） | 可放多个，拖动、`Delete` 删除；状态面板的「电源」按钮从第一个符号起线 |
| **CAN 收发器** | `TXD` `RXD` `CANH` `CANL` `VCC` `GND` | 物理层：`TXD` 的电平按位驱动总线对，`RXD` 回送总线（线与）采样到的电平；`CANH`/`CANL` 才是真正的两条线，要接到同一个网络的终端电阻与探头 | 画布显示模式与收发帧数；「CAN 总线」窗口可看帧、注入帧、切换模式、查看总线质量 |
| **电阻** | `1` `2` | 无源器件，本身不驱动任何东西：跨接在 `CANH`/`CANL` 上就是**终端电阻**，并联值决定总线质量（默认 120 Ω，可在面板改值） | 面板里改阻值；拆掉/减少终端电阻，总线会开始丢帧 |
| **示波器** | `H` `L` | 两通道探头，`H` 接 `CANH`、`L` 接 `CANL` 后每个仿真位采样一次，画出两条线的电压与差分；振铃导致的翻转会用红色竖线标出，并在波形下方逐帧标出每一 bit 的字段定义 | 「📈 示波器」窗口：时基、档位、差分视图、冻结/清空、位标尺与逐位定义 |
| **SPI 灯阵** | `DIN` `CLK` `CS` `DOUT` `VCC` `GND` | MAX7219 驱动的 8×8 点阵：按 SPI 时钟移位、`CS` 上升沿锁存一个 16 位字，`DOUT` 回送 16 位前的字节（供固件回读） | 画布上直接显示点阵画面；「外设浏览」里有寄存器细节与放大预览 |
| **HSE 晶振** | `X1` `X2` | 8 MHz 高速晶体，接在 `OSC_IN`/`OSC_OUT`（PD0/PD1）。只有接好晶体、固件又置位 `HSEON`，`HSERDY` 才会在起振时间后置位；没接晶体时就像裸片一样永远等不到 ready | **新建工程已经把它摆在芯片左侧、PD0/PD1 旁并接好线**（无极性）。起振时 PD0/PD1 与连线按晶振频率真实翻转（采样到画布上就是闪烁），停止后自动释放 |
| **LSE 晶振** | `X1` `X2` | 32.768 kHz 手表晶振，接在 `OSC32_IN`/`OSC32_OUT`（PC14/PC15），给 RTC 当时钟源；同样只有固件置位 `LSEON` 才起振 | 接到 PC14/PC15（无极性）。起振时 PC14/PC15 与连线同样真实翻转 |
| **接线节点** | `T` | 导线上的分支点：**双击一根连线**就在点击处插入一个节点，那根线从此分成两段（两段都接到节点的 `T` 上，网络不变），所以可以把它当**接线端子**再引线出去；也可以从器件库直接放一个独立节点当汇流点 | 点节点（或从它拖出）继续连线；双击节点删除并让两段线并回一根；`Delete` 删除选中的节点 |

新建工程**默认只自动摆好 HSE 晶振**（放在芯片左侧 PD0/PD1 旁，已接好线，MCU 起振就靠它），除此之外画布是空的、
也没有默认激励；其他器件都从「器件库」里放。接好线后用「💾 保存工程…」把整张图纸存成 `.gsim`，
下次「📂 打开工程…」即可原样恢复；
想直接看文档里的演示接线，就打开仓库自带的 [`docs/example.gsim`](docs/example.gsim)。

工程文件是逐行文本，第一行固定是 `gsimulator project 1`，之后每个器件一行、每根连线一行：

```text
gsimulator project 1
device 1 vcc 430.00 110.00
device 2 led 600.00 120.00
device 3 resistor 700.00 300.00 ohms=120
device 4 scope 760.00 420.00 window=20000 frozen=0
wire mcu:0:5 dev:2:0
wire dev:1:0 dev:2:1
wire dev:3:0 dev:4:0 680.00,360.00 720.00,360.00
```

* `device <编号> <类型> <x> <y> [选项…]`：类型是 `led`/`uart`/`encoder`/`vcc`/`gnd`/`can`/
  `matrix`/`hse`/`lse`/`resistor`/`scope`/`junction`；可选选项有 `ohms=`（电阻）、
  `unit=`（终端监听哪个 USART）、`mode=normal|silent|standby`（CAN 收发器）、`window=`/`frozen=`（示波器）。
* `wire <引脚> <引脚> [拐点…]`：引脚写成 `mcu:<端口>:<引脚>` 或 `dev:<编号>:<引脚>`，
  拐点是板坐标里的 `x,y`；同一根线可以带任意多个拐点。
  接线节点也是器件（`junction`），被它切开的导线存成两根都以 `dev:<编号>:0` 结尾的 `wire`。
* 以 `#` 开头的整行是注释；器件按文件顺序重新放置，所以自动编号的名字（`LED1`、`UART2`…）与保存时一致。
  解析失败会指出具体行号（如 `第 3 行：未知器件类型`），并保持当前图纸不变。

**时钟源由固件 ELF 决定**：仿真只负责板上有哪些晶振（HSE/LSE 是否接着），
固件自己的 RCC 配置决定用 HSI、HSE、PLL 还是 LSE/LSI（`SW`、`PLLSRC`、`RTCSEL`）。
两颗晶体都接着也不会互相抢占，只有被 `HSEON`/`LSEON` 打开的那颗才开始起振。
「外设浏览」的 RCC 行会显示「HSE 晶振 8 MHz · 稳定 / 未起振 / 未接晶振」与 LSE 的同样信息。

接线规则（EDA 风格，模型在 `crates/gsim-devices/src/{uart,encoder,max7219,can_bus,crystal}.rs`）：

* **供电（必须）**：UART 终端、旋转编码器、CAN 收发器、SPI 灯阵的符号上都有 `VCC` 与 `GND` 引脚（都在顶边），
  用「器件库 → 电源」里的 `VCC`/`GND` 符号各接一根线（仓库的 `docs/example.gsim` 已经把终端与编码器接好）。
  **没接电或接反的器件不工作**：终端不收也不发（输入的内容继续排队）、编码器不驱动任何引脚、
  收发器不上总线、灯阵不吃 SPI 帧。状态面板的器件行显示「VCC/GND 已接 / 未接（不工作）/ 接错（不工作）」。
  供电在每个仿真时间片重新判定（`bridge::service` 调用 `Board::supplies`），
  所以运行中补上电源会立刻恢复工作，拆掉电源也会立刻停机。
* **UART**：只有当终端的 `RX` 真的接在该 USART 的 `TX` 引脚（USART1=PA9、USART2=PA2、USART3=PB10）上时，
  才收得到字节；只有 `TX` 接在该 USART 的 `RX` 引脚（PA10 / PA3 / PB11）上时，输入的内容才会进 MCU。
  没接通时终端把字节排队，面板显示“N 字节待发送（等待连线）”。
* **编码器**：`A`/`B`/`SW` 各自驱动所连的 MCU 引脚（走 `Soc::gpio_set_input`，
  所以 EXTI 边沿与 `IDR` 轮询都能看到）；没接线的接触点不会动任何引脚。
* **CAN**：`TXD` 接 `CAN1_TX`（PA12）、`RXD` 接 `CAN1_RX`（PA11）之后控制器才真正接上总线；
  `CANH`/`CANL` 分别组成两条网络，同一条总线上的收发器、终端电阻、探头都按网络连通性识别。
  跨接在两条网络之间的电阻**并联**成终端电阻（两只 120 Ω = 60 Ω，教科书值），
  没有终端电阻时线路会振铃并翻转位（见下文“CAN 物理层”）。
* **SPI 灯阵**：`CLK` 与 `DIN` 必须落在同一个 SPI 口的 `SCK`/`MOSI`（SPI1=PA5/PA7、SPI2=PB13/PB15、
  SPI3=PC10/PC12）上才会收到字节，`CS` 接任意 GPIO 作 `LOAD`；`DOUT` 接该口的 `MISO`
  （PA6 / PB14 / PC11）时固件才能读回回声。换到别的口或拆掉连线，点阵立刻听不到数据。
* **晶振**：`X1`/`X2` 必须真的跨接在对应振荡脚上（HSE = PD0/PD1，LSE = PC14/PC15）才算装了晶体，
  方向随意、只接一根不算；拔掉器件或拆掉线，RCC 会立刻看到振荡器消失（`HSERDY`/`LSERDY` 掉下来）。

引脚注入与采样在**指令之间**进行（`crates/gsimulator-gui/src/bridge.rs` 把一帧切成约 1 ms 的仿真时间片，
每片先服务外设再推进 CPU），因此固件轮询 `IDR` 或等待 EXTI 都能真正捕捉到编码器边沿。

`cargo test -p gsimulator-gui` 会：跑板级电气规则测试、无窗口渲染整帧 UI（`egui_kittest`，GPU 不可用时跳过）、
验证“内置激励 → 引脚 → 连线 → LED 点亮”和引脚/波形/占空比/自定义函数的可配置性，并跑两条端到端链路：
**真实 ELF 的 USART1 输出出现在虚拟终端里**、**转旋钮改变 MCU 引脚电平（完整格雷码序列）**。
设置 `GSIM_SNAPSHOT=1` 可以把当前渲染结果保存到 `target/gui-snapshot.png`。

字体：中文标签需要一个 CJK 字体，程序会自动查找系统字体
（`C:\Windows\Fonts\msyh.ttc`、`simhei.ttf`、Noto Sans CJK、PingFang 等），
也可以用环境变量 `GSIM_FONT` 指定字体文件。

---

## 1. 快速开始

```powershell
cargo build                    # 编译全部 crate
cargo test  --workspace        # 全部测试（含真实固件执行测试）
cargo clippy --workspace --all-targets
```

几个可以直接运行的能力：

```powershell
cargo run -p gsimulator-cli -- demo                # 无 ELF：脚本化巡览全部外设并打印时序
cargo run -p gsimulator-cli -- info                # 打印模拟的内存映射
cargo run -p gsimulator-cli -- run firmware.elf    # 加载并执行真实 Cortex-M3 ELF 固件
cargo run -p gsimulator-cli -- flash firmware.elf  # 烧进模拟 flash（擦除+编程+回读校验）
cargo run -p gsimulator-cli -- gdb                 # 虚拟 ST-Link：给 GDB 烧录 + 调试（见下）
cargo run -p gsimulator-gui                        # 图形前端（虚拟接线台，见第 0 节）
```

`run` 的选项：

| 选项 | 说明 |
| --- | --- |
| `-n, --max-instructions <N>` | 指令预算（默认 5,000,000） |
| `-t, --max-microseconds <N>` | 按仿真时间停止 |
| `--hse <HZ>` | OSC_IN 晶振频率（默认 8 MHz） |
| `--trace` | 在 stderr 上打印外设事件（IRQ、引脚、SPI 帧、日志） |
| `--registers` | 结束时打印 r0-r3/r12/sp/lr/pc |

固件的 USART1 输出直接写到 stdout，其余诊断信息写到 stderr，方便管道处理。

`flash` 只有一个选项 `--hse <HZ>`：按 ELF 的加载地址**逐页擦除 → 半字编程 → 回读校验**，
最后打印向量表（SP 与复位入口符号），等价于 `openocd ... -c "program app.elf verify"`：

```powershell
cargo run -p gsimulator-cli -- flash firmware/CAN/build/Debug/CAN.elf
# flashed ... (12040 bytes in 13 pages, read back and verified)
# vector table: sp=0x20005000 reset=0x08002B40 (Reset_Handler)
```

想烧完直接跑就用 `run`，想烧完断点调试就用 `gdb`（下一节）。

### 运行真实 ELF

任意用 `arm-none-eabi-gcc`/Keil/GCC 生成的 Cortex-M3 ELF（向量表放在 flash 起始处）都可以直接跑：

```powershell
arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -O1 -nostdlib -nostartfiles `
    -T link.ld -o app.elf startup.c main.c
cargo run -- run app.elf
```

`crates/gsim-cpu/tests/firmware/` 里有一个完整的示例固件（`main.c` + `startup.c` + `link.ld`），
它在模拟器中启动、配置时钟/GPIO/USART/EXTI/TIM/ADC，进入 SysTick 与 EXTI 中断，用两个位带
别名区改写单个比特，最后通过 USART1 打印：

```
GSIM
T3
E1
M250
A2047
B13
OK
```

`cargo test` 会自动用 `arm-none-eabi-gcc` 编译这个固件并逐字节断言上面的输出；
如果机器上没有该交叉编译器，相关测试会跳过并给出提示。

工作区里 `firmware/CAN` 的 CubeMX 工程是一份**外设自检固件**：上电后不经过 HAL，
直接用寄存器把模拟器建模过的每个外设跑一遍，并在 USART1（115200 8N1，PA9 = TX、PA10 = RX）
上逐行打印结果：

```
GSIM-SELFTEST 1
CLK 72000000 HSE
SYSTICK OK
GPIO OK
BITBAND OK
EXTI OK
USART1 OK
USART2 OK
USART3 OK
SPI1 OK
I2C1 OK
TIM1 OK
TIM2 OK
TIM3 OK
TIM4 OK
ADC OK 2047
DMA OK
FLASH OK
CRC OK 0xC704DD7B
RTC OK
PWR OK
BKP OK
CAN OK
IWDG OK
WWDG OK
ENV HSE=READY LSE=WAIT RTC=LSI RESET=POR
ONCHIP 23/23
DEV MATRIX=NA ECHO=0x00000000
LOOP
EVT READY
```

* `CLK`：HSE 8 MHz → PLL ×9 = 72 MHz。板上没接 HSE 晶振时自动退回 HSI 8 MHz
  （`CLK 8000000 HSI`、`HSE=ABSENT`），不会卡在 `HSERDY` 上。
* 每个 `OK` 都是一次真实的寄存器往返：GPIO 读 `IDR`/`LCKR`、位带别名改单个比特、EXTI 软件请求进中断、
  USART2/3 发到 `TC`、SPI1 双字节收发、I2C1 在空总线上收到 NACK、TIM1/3 的比较标志、TIM2 双向计数、
  TIM4 单脉冲、ADC1 单次转换、DMA1 存储器到存储器 + ADC 循环搬运、FLASH 末页擦除/编程/回读、
  CRC 参考值、RTC 预分频与计数器、IWDG/WWDG 配置与计数、CAN1 内部回环收发。
* `ENV` 给出板上的晶振、RTC 时钟源与复位原因；`ONCHIP n/m` 是成功的检查数。
  示例里 `ADC OK 2047` 对应通道 1 上的 1650 mV（没有模拟输入时是 `ADC OK 0`）。

自检之后固件进入主循环，服务原理图画好的器件：

| 器件 | 接线 | 固件行为 |
| --- | --- | --- |
| UART 终端 | `RX`→PA9、`TX`→PA10 | 打印报告；终端输入的字会回显并打印 `EVT UART1 RX n` |
| SPI 灯阵 | `DIN`→PA7、`CLK`→PA5、`CS`→PA4、`DOUT`→PA6 | 上电做 DOUT 回读自检（`DEV MATRIX=OK`），之后每秒在“对号 / 叉号”之间切换 |
| 旋转编码器 | `A`→PB0、`B`→PB1、`SW`→PB2 | EXTI 解码四分之一相位，转动/按下轴打印 `EVT ENC <steps> <presses>` |
| LED | `A`→PB12、`K`→GND | 自检时亮 1 ms，之后主循环每 250 ms 翻转 |
| CAN 收发器 | `TXD`→PA12、`RXD`→PA11 | 回环自检后每 500 ms 以标准帧 `0x123` 发 `"Hello"`；收到的帧打印 `EVT CAN RX`，每秒打印 `EVT CAN STATE TEC=… REC=…`（总线上没有对端时，发送会像真芯片一样报 ACK 错误） |
| HSE 晶振 | `X1`/`X2`→PD0/PD1 | 决定是否切到 72 MHz PLL |
| LSE 晶振 | `X1`/`X2`→PC14/PC15 | 起振（约 2 s）后打印 `EVT LSE READY`；报告里先是 `LSE=WAIT` |
| 电阻 / 示波器 | `CANH`/`CANL` | 终端电阻决定总线质量，示波器看差分波形 |

终端、灯阵、编码器、收发器都要像其它器件一样各接一对 `VCC`/`GND`（没电不工作）。

```powershell
# 编译固件（需要 arm-none-eabi-gcc 与 CMake）
cd firmware\CAN
cmake --preset Debug
cmake --build --preset Debug        # -> build/Debug/CAN.elf

# 只跑固件、看它打印的报告
cargo run --release -- run firmware\CAN\build\Debug\CAN.elf -t 3000000 --trace

# 或者打开配套的原理图，在画布上看每个器件跟着固件动
cargo run -p gsimulator-gui         # 📂 打开工程… docs/selftest.gsim，再 加载 ELF… CAN.elf
```

`docs/selftest.gsim` 就是上表的接线（终端、LED/编码器、SPI 灯阵、CAN 收发器 + 120 Ω 终端 + 示波器、
HSE/LSE 晶振、VCC/GND）。

> 注意：`--release` 建议用于长时间运行；debug 构建每秒仿真时间约需数十秒真实时间。

`cargo test -p gsim-cpu` 会逐字节断言上面这份报告（接晶振与不接晶振两种情况），
`cargo test -p gsimulator-gui --test selftest_firmware` 则把真实 ELF 装进带器件的接线台，
验证终端、LED、灯阵、编码器与 CAN 对端都能与固件互通。

### 用 GDB 调试 / 烧录（虚拟 ST-Link）

`gsimulator gdb` 起一个**虚拟 ST-Link**：它监听 OpenOCD 使用的同一个端口（3333），
用 GDB Remote Serial Protocol 把调试器的每个请求落到真实的片上模型上——flash 用
`FLASH_KEYR`/`CR`/`AR` 真解锁、按半字编程、写完回读校验；寄存器与内存访问走 `Soc` 总线；
断点由核心的 PC 比较实现。GDB 侧的命令与接真实板子时完全一致：

```powershell
# 终端 1：起调试服务器（占用 3333，和 openocd 一样）
cargo run -p gsimulator-cli -- gdb

# 终端 2：像调试真实板子一样烧录并调试
arm-none-eabi-gdb firmware/CAN/build/Debug/CAN.elf `
    -ex "target extended-remote 127.0.0.1:3333" `
    -ex "load" `
    -ex "break main" `
    -ex "run"
```

`gdb` 子命令的选项：

| 选项 | 说明 |
| --- | --- |
| `-p, --port <PORT>` | 监听端口（默认 3333，即 OpenOCD 的 `gdb_port`） |
| `--bind <ADDR:PORT>` | 完整监听地址（默认 `127.0.0.1:3333`） |
| `<firmware.elf>` / `--elf <PATH>` | 连接前先把镜像装进 flash（可选） |
| `--hse <HZ>` | OSC_IN 晶振频率（默认 8 MHz） |
| `--no-uart` | 不把固件的 USART1 输出转发到 stdout |
| `-v, --verbose` | 在 stderr 上逐条打印 RSP 收发（排查 GDB 问题用） |

GDB 里可用的能力：

| 操作 | 实现 |
| --- | --- |
| `load` | `vFlashErase`/`vFlashWrite` 擦写模拟 flash（1 KiB 页），每次写完回读校验 |
| `break` / `hbreak` | `Z0`/`Z1`，核心在每条指令前比较 PC，不用改写指令流，flash 里也能打断点 |
| `watch` / `rwatch` / `awatch` | `Z2`/`Z3`/`Z4`，运行中逐条指令比较被监视的内存 |
| `step` / `stepi` / `next` | `vCont;s`，精确执行一条指令 |
| `continue` + Ctrl-C | `vCont;c`；服务器每 10 万条指令轮询一次 socket，收到 `0x03` 立即停下并回 `T02` |
| `info registers` / `print` / `x/…` | `g`/`p`/`m` 包；外设寄存器也能直接读，读未映射地址回报错误而不是崩溃 |
| `monitor reset` | 复位外设并把核心从向量表重启（等价于按复位键），停下后可用 |
| `monitor regs` / `monitor info` | 打印核心寄存器 / 指令数、仿真时间、中断数、时钟树 |
| `monitor peripherals` | 每个已建模外设的基址、时钟门控与状态 |

三个需要知道的细节：

- **为什么不是 OpenOCD？** `interface/stlink.cfg` 走的是真实 USB 硬件；而且常见发行版
  （包括 EIDE 自带的那份）没有编译 `remote_bitbang` 一类的网络适配器，OpenOCD 没有可以
  指向模拟器的后端（`openocd -c "adapter list"` 里那 19 个适配器全是 USB 探针）。所以本仓库
  把 GDB 服务器的这一半自己做掉了：端口、连接方式、`load` 语义都和 OpenOCD 一致，GDB/IDE
  侧不用改；只想“烧一下”而不调试时用 `gsimulator flash app.elf`，它等价于
  `openocd ... -c "program app.elf verify"`。
- `load` 之后核心的 SP/PC 仍是加载前的值（真实芯片也一样），所以要么 `monitor reset`，
  要么直接用 `run`（`vRun` 会先复位再从向量表跑）。
- GDB 的寄存器缓存只在“停下”时刷新：`monitor reset` 之后如果想立刻看寄存器，先 `stepi`
  一次（`run` 之后的停点不受影响）。

---

## 2. 架构

```
crates/
  gsim-core/      与具体芯片无关的基础设施
  gsim-stm32f1/   STM32F103 片上外设模型（Soc + 各外设，含 bxCAN）
  gsim-devices/   片外器件模型（MAX7219 点阵、CAN 收发器 + 虚拟总线）
  gsim-cpu/       Cortex-M3 执行前端 + ELF 装载 + SoC 桥接
  gsimulator-cli/ 命令行（run / demo / info）
  gsimulator-gui/ 图形前端（虚拟接线台：画布 + 虚拟外设 + 状态面板 + 外设浏览 + CAN 总线窗口 + 示波器）
firmware/         已有的 CubeMX 固件工程（全外设自检固件，见第 1 节“运行真实 ELF”）
```

### 数据流

```
                  ┌─────────────── gsim-cpu ───────────────┐
  ELF 文件 ──▶ ElfImage ──▶ load_into(Soc)
                  │                                       │
             armagnac 内核  ◀── SocInterface ── Soc 总线 ──▶ gsim-stm32f1 外设
             (Thumb-2)            (MemoryInterface)              │
                  ▲                                            │
                  └────────── NVIC 仲裁 ────────────────────────┘
```

* **gsim-core**：`AccessWidth`/`BusError` 总线访问、`Ctx`（时钟快照 + `Event` 收集器）、
  `Peripheral` trait、`Irq` 编号、Cortex-M3 的 `Nvic`/`Scb`/`SysTick`、纳秒↔周期换算。
* **gsim-stm32f1**：`Soc` 拥有全部外设，实现 `MemoryBus`（`read`/`write` 按地址分发），
  负责仿真步进、EXTI 引脚采样、DMA 引擎与事件汇聚。外设不直接访问 NVIC，而是把副作用
  以 `Event` 上报，由 `Soc` 统一落到 NVIC / 观察者。
* **gsim-cpu**：把内核的每一条访存转发到 `Soc` 总线；执行一条指令记一个核心时钟周期；
  按固件写入的 `ISER`/`IPR`/`AIRCR.PRIGROUP` 做中断仲裁，再把胜者交给内核
  （由 armagnac 完成 ARMv7-M 的入栈、向量跳转与 `EXC_RETURN` 返回）。

### 时间模型

* 全局时间是**纳秒**，每个外设用 `NanoAccumulator` 把纳秒换算成自己的时钟周期，余数保留，无漂移。
* CPU 侧：1 条指令 = 1 个 HCLK 周期，每执行一批指令后按实际周期数推进 `Soc`
  （默认批量 32 条，可用 `Cpu::set_batch` 调整，`1` 表示逐周期）。
* 无 CPU 时（寄存器级测试）默认步进粒度 1 µs，可用 `Soc::set_step_ns` 调整。

### Cortex-M 系统控制空间的所有权

* **无 CPU（寄存器级测试）**：`Soc` 自己实现 `SCB`/`NVIC`/`SysTick`（`0xE000_E000`）。
* **接入 CPU 后**：该空间由内核模型（armagnac）拥有，`Soc` 只保留外设中断的 **pending** 状态并从内核
  读取 enable/priority 做仲裁；因此固件写入 `NVIC_ISER/IPR` 之后，中断才会被投递。
  `Soc::set_external_systick(true)` 会在接入内核时自动设置，避免 SysTick 被计数两次。

---

## 3. 已实现的外设

| 模块 | 关键行为 |
| --- | --- |
| RCC | HSI/HSE/PLL 启动延时与 ready 标志、SW/SWS 切换、AHB/APB 分频表、TIM/ADC 倍频、外设时钟门控、`APBxRSTR` 复位、`BDCR/CSR`、MCO |
| GPIO A–E | `CRL/CRH/IDR/ODR/BSRR/BRR/LCKR`，输出/上拉下拉/浮空/模拟输入语义，外部驱动覆盖弱上拉，引脚电平变化事件 |
| AFIO | `EVCR/MAPR/EXTICR1-4`，EXTI 线到端口的映射、各类 remap 位、`SWJ_CFG` |
| EXTI | `IMR/EMR/RTSR/FTSR/SWIER/PR`，边沿锁存、软件触发、按线映射到 NVIC 中断 |
| USART1-3 | `SR/DR/BRR/CR1-3`，真实帧时间（`frame_bits × BRR / fCK`）、TXE/TC/RXNE/ORE、中断、DMA 请求、可注入接收字节 |
| SPI1-2 | `CR1/CR2/SR/DR`，主/从、BR 分频、8/16 位帧、TXE/RXNE/BSY/OVR、按帧时长推进、MISO 注入、DMA 请求 |
| I2C1-2 | `CR1/CR2/OAR/SR1/SR2/CCR/TRISE`，START/STOP、地址/数据阶段、`AF` NACK（本阶段总线无器件）、事件/错误中断 |
| CAN1 (bxCAN) | `MCR/MSR/TSR/RF0R/RF1R/IER/ESR/BTR`、3 个发送邮箱（RQCP/TXOK/ALST/TERR/ABRQ、按 ID 或请求顺序的发送优先级）、2 个接收 FIFO（3 帧深、FMP/FULL/FOVR/RFOM）、14 组验收滤波器（16/32 位、列表/掩码）、初始化/睡眠/回环/静默模式、**位级收发**（位填充、CRC-15、线与仲裁、ACK 槽、错误帧、TEC/REC/bus-off）、CAN1_TX/RX0/RX1/SCE 四条中断线 |
| TIM1-4 | `PSC/ARR/RCR/CCRx/CCMRx/CCER/BDTR`，上/下/中央对齐计数、更新事件与重复计数、比较匹配、PWM 输出电平、输入捕获、单脉冲、DMA 请求 |
| ADC1-2 | `SR/CR1/CR2/SMPR/SQR/JOFR/DR`，采样时间→转换时间、单次/连续/扫描、校准序列、EOC/STRT、DMA 请求、按通道注入电压 |
| DMA1/2 | `ISR/IFCR/CCRx/CNDTR/CPAR/CMAR`，请求路由表（RM0008）、软件优先级、增量、循环、HT/TC 标志与中断 |
| FLASH | `ACR` 等待周期、`KEYR` 解锁、`CR.PG/PER/MER/STRT`、`SR.EOP/WRPRTERR`、页擦除/全擦除、半字编程 |
| PWR / BKP | 低功耗模式选择、`WUF/SBF`、`DBP` 保护的备份寄存器 |
| RTC | `CRH/CRL/PRL/DIV/CNT/ALR`，按 `RTCSEL` 时钟源预分频到 1 Hz、秒/闹钟中断 |
| IWDG / WWDG | 键序列启动/重装、LSI 计数超时、窗口看门狗提前唤醒与复位 |
| CRC | F1 的 CRC-32 单元（多项式 `0x04C11DB7`，MSB-first） |
| DBGMCU | `IDCODE`＝`0x20036410`、`CR` |
| Cortex-M3 | 寄存器级 `NVIC`/`SCB`/`SysTick`（含 `PRIGROUP` 优先级分组、嵌套、`PRIMASK`/`BASEPRI`），接入内核后由内核接管 |
| 位带别名 | `0x4200_0000`（外设）/ `0x2200_0000`（SRAM）两个 alias 区，读写单比特语义与硬件一致 |

> **位带（bit-band）**：`stm32f1xx.h` 里的 `RCC_CR_PLLON_BB`、`GPIOA_ODR_BB`、`RCC_CSR_LSION_BB`
> 等宏都是位带别名地址，HAL 的 `__HAL_RCC_PLL_DISABLE()`、`HAL_RCC_EnableCSS()` 等直接写这些
> 地址。`Soc` 在总线入口解析别名：读回 0/1，写入只取最低位（1 置位、0 清位），其余位不受影响。
> 没有这个特性，任何 HAL 固件都会在时钟配置阶段写飞。

### 片外器件（`gsim-devices`）

片外器件不属于 SoC：它们只看到**引脚事件**和**总线上的帧**，通过 `Soc::add_event_sink`
订阅 `Event`，再通过 `Soc::spi_set_miso` / `Soc::can_receive` 把信号送回控制器。

| 器件 | 接线 | 行为 |
| --- | --- | --- |
| **MAX7219 点阵** | SPI1（`PA5` SCK、`PA7` MOSI）+ `PA4` LOAD | 在 LOAD 上升沿锁存 16 位字（地址+数据），支持级联、`DIGIT0..7`/`DECODE`/`INTENSITY`/`SCAN_LIMIT`/`SHUTDOWN`/`DISPLAY_TEST` 寄存器、8×8 像素渲染；`DOUT` 回显 16 位前的数据（`MAX7219_EchoTest()` 可回读） |
| **CAN 收发器** | CAN1（`PA12` TXD、`PA11` RXD） | 把控制器发出的帧送上虚拟总线、把总线上的帧交给控制器 FIFO；三种模式（正常/只听/待机），统计收发/回环/拒发帧数；丢弃的帧不会占用总线 |
| **虚拟 CAN 总线** | `Rc<RefCell<CanBus>>` 共享 | 位级线与总线：一次推进一位，所有节点的驱动电平做线与；终端电阻并联值 → 线路质量 → 按概率翻转位；虚拟节点（注入面板/第二个 ECU）跑同一套位机器，因此真会仲裁、应答、报错；记录每帧的时间戳、节点、方向、ID、数据与应答状态 |

GUI 里这两个器件来自器件库（工具栏 `＋ 添加器件`，或把卡片拖到画布），接线与 CubeMX 工程一致
（MAX7219：`PA5`/`PA7`/`PA4`；CAN 收发器：`PA11`/`PA12` + 一条虚拟总线）。**它们只存在于画布上**：
没放置就没有模型、也没有外设浏览条目；拆掉连线立刻听不到总线/SPI 数据。放好之后分别出现在：

* **外设状态浏览窗口**（工具栏 `🔍 外设浏览`）：左侧列出全部片上外设，以及画布上放好的片外器件
  （按画布上的器件名，如 `MX1`、`CAN1`、`R1`、`OSC1`），右侧是实时解码状态；选中点阵时还会画出 8×8 点阵预览。
* **画布符号**：CAN 收发器直接显示模式与收发帧数，SPI 灯阵直接显示 8×8 画面，
  电阻显示阻值，示波器在符号里直接画出最近的波形。
* **CAN 总线状态窗口**（工具栏 `🚌 CAN 总线`）：总线电气状态（终端电阻/总线质量/位时间/位错误率/
  尖峰/仲裁失败/错误帧/节点）、控制器寄存器（模式/波特率/邮箱/FIFO/滤波器/错误计数）、
  收发器状态与计数、按时间排序的帧记录（时间/节点/方向/ID/DLC/数据/ACK），
  以及一帧注入面板（可扮演第二个 ECU，注入的帧真的走线与仲裁）。
* **示波器窗口**（工具栏 `📈 示波器`）：探头抓到的 `CANH`/`CANL` 电压、差分视图、时基与冻结，
  线路振铃造成的翻转用红色竖线标出（没有放探头时窗口会提示怎么接线）。
  抓到完整 CAN 帧时，波形下面会多出一条**位标尺**：每个采样位一格，按字段着色（`SOF`/`ID`/`RTR`/
  `IDE`/`r0`/`DLC`/数据/`CRC`/`ACK`/`EOF`/`IFS`/填充位），悬停显示该位的名字、电平与逻辑值；
  标尺下方还能逐帧翻看**每一 bit 的定义**（`12·RTR=0` 这种形式）以及帧摘要（ID/DLC/数据/CRC/应答/位数）。

超时复位、看门狗复位都会走同一条 `Soc::system_reset()` 路径，并在 `RCC_CSR` 里留下复位原因。

### CAN 物理层（位级总线、仲裁与终端电阻）

CAN 不是“帧队列”，而是真实的两线差分总线。这条链路上每一层都有自己的模型：

| 层 | 位置 | 做什么 |
| --- | --- | --- |
| 帧编码 | `gsim-core/src/can.rs` | `CanFrame` ↔ `WireFrame`：SOF、标准/扩展标识符、RTR/IDE、DLC、数据、CRC-15（多项式 `x^15+x^14+x^10+x^8+x^7+x^4+x^3+1`）、**位填充**（连续 5 位相同后插一位相反电平，CRC 序列末尾同样会插）、CRC 分隔符、ACK 槽、EOF 与 3 位间歇 |
| 解码 | `gsim-core/src/can.rs` | `FrameDecoder`：去填充、CRC 校验、ACK 槽、EOF；任何违规立即回报 `Stuff/Crc/Form/Acknowledge/Bit` 错误。`TraceDecoder` 把示波器采到的位流重建成帧，并给**每一位**标上字段（`SOF`/`ID`/`SRR`/`IDE`/`RTR`/`r0`/`DLC`/数据/`CRC`/`ACK`/`EOF`/`IFS`/填充位），只发布走到间歇场的完整帧，所以探头在中途接入也不会画出半帧乱码 |
| 控制器 | `gsim-stm32f1/src/can.rs` | bxCAN 把邮箱内容逐位移出（`driven_level`），并采样总线（`sample_bit`）：**线与**上出现“自己发隐性、线上是显性”即**仲裁失败**（保留邮箱、置 `TSR.ALSTx`、等总线空闲重试）；接收方在 ACK 槽拉显性，没人应答就是 `Acknowledge` 错误 → 发 6 显性 + 8 隐性的**错误帧**、`TEC += 8`、`REC += 8`，`REC/TEC ≥ 128` 错误被动，`TEC ≥ 256` bus-off（`ABOM` 时等 11 位隐性自动恢复） |
| 总线 | `gsim-devices/src/can_bus.rs` | 一次一位地推进：所有节点驱动的电平做**线与**（任何显性 → 显性），终端电阻的并联值算出线路质量，质量不够就按概率翻转位（`glitches`）；虚拟节点（注入面板 / 第二个 ECU）跑同一套位机器，所以真的会仲裁、应答、报错 |
| 接线 | `gsimulator-gui/src/bridge.rs` | 每片仿真时间按 CAN 位时间（`BTR` 决定）切片，每片：读控制器 `driven_level` → `bus.step` → 把采样电平喂回控制器、喂给示波器探头 |

终端电阻与通讯质量的关系（`quality_of` / `bit_error_rate`）：

| 并联终端 | 质量 | 位错误率 | 效果 |
| --- | --- | --- | --- |
| 60 Ω（两个 120 Ω，一端一个） | 100 % | 0 | 教科书接法，帧全部干净送达 |
| 120 Ω（只留一个） | 67 % | 0.11 %/位 | 能用，但偶发振铃与重传（示波器上会看到尖峰） |
| 30 Ω / 240 Ω | 67 % / 0 % | 0.11 % / 3 % | 负载过重 / 严重失配 |
| 开路（没有终端电阻） | 3 % | 2.7 %/位 | 大部分帧 CRC 错 / 应答错，控制器 `TEC` 一路上涨 |

`cargo run -p gsimulator-cli -- demo` 会现场演示：`0x321` 与对手节点的 `0x100` 同时起跑时
**仲裁失败 1 次**（低 ID 胜出，高 ID 保留邮箱重试），随后对比“两个 120 Ω / 一个 120 Ω / 没有终端电阻”
三种接法的尖峰与错误帧数量。

### 已知简化（有意为之）

* 定时器的预装载寄存器（`ARPE`/`OCxPE`）立即生效，不做影子寄存器；PWM 占空比修改当期可见。
* 中断的 pending 跟随外设中断线电平（ISR 清标志即撤销 pending），不是严格的“边沿置位 + 显式清 pending”。
* 本阶段 I2C 总线上没有器件，因此主机寻址一律 NACK（`SR1.AF`），符合“空总线”的真实行为。
* SPI 的 MISO 由外部器件在每一帧之前给定（`Spi::set_miso`/`Soc::spi_set_miso`）；MAX7219 用
  这个通道实现 DOUT 回显。
* CAN 的位定时只用来定“一位多长”（`BTR` 的 `BRP/TS1/TS2`），不细分采样点位置；
  同步段/传播段不单独建模，位填充也只按 5 位规则插入与检查。
* 终端电阻只影响位错误率与振铃幅度，不做真正的传输线反射计算；`CAN2`/USB 未建模。
* 外设时钟门控默认只影响“是否推进仿真”，不阻止寄存器访问；`Soc::set_strict_clock_gating(true)`
  可让未开时钟的外设访问返回总线错误。

---

## 4. 对外接口（供测试、脚本与后续阶段使用）

```rust
use gsim_core::{AccessWidth, MemoryBus};
use gsim_cpu::Cpu;
use gsim_stm32f1::Soc;
use std::{cell::RefCell, rc::Rc};

let soc = Rc::new(RefCell::new(Soc::new()));
let mut cpu = Cpu::new(Rc::clone(&soc))?;
cpu.load_file("firmware.elf")?;          // 装载 + 从向量表复位
cpu.run(1_000_000)?;                     // 执行
soc.borrow_mut().adc_set_channel_mv(0, 1, 1650);   // 注入模拟输入
cpu.pend_interrupt(gsim_core::irq::lines::EXTI0);  // 注入中断
```

输入注入只通过这些“接线”接口：`gpio_set_input`、`usart_receive`、`spi_set_miso`、
`adc_set_channel_mv`、`tim_set_channel_input`、`can_receive`；CAN 另有位级接口
`can_set_on_bus` / `can_driven_level` / `can_sample_bit`（GUI 的桥与 CLI 的 demo 用它跑真实总线）。
片外器件模型（`gsim-devices`）就是它们的驱动者：

```rust
use gsim_devices::{CanBus, CanTransceiver, Max7219};
use std::rc::Rc;

// MAX7219 挂在 SPI1 + PA4 上，订阅 SoC 的引脚/SPI 事件。
let matrix = Max7219::firmware_default();          // unit 0, CS = PA4, 1 片
soc.borrow_mut().add_event_sink(Box::new(move |event| matrix.on_event(event)));

// CAN 收发器挂在 CAN1 上，接一条可共享的位级虚拟总线。
let bus = Rc::new(RefCell::new(CanBus::new()));
bus.borrow_mut().set_termination(Some(60.0));      // 两只 120 Ω 终端电阻的并联值
let mut transceiver = CanTransceiver::attach(Rc::clone(&bus), "节点A", 0);
while let Some(frame) = transceiver.poll() {
    soc.borrow_mut().can_receive(0, frame);         // 帧级注入（测试/脚本用）
}

// 位级收发：总线一次推进一位，控制器逐位驱动与采样（GUI 的桥就是这样跑的）。
soc.borrow_mut().can_set_on_bus(0, true);           // 由总线提供位时钟
for _ in 0..200 {
    let bit_ns = soc.borrow().can_bit_time_ns(0);
    let now = soc.borrow().now_ns();
    let level = bus.borrow_mut().step(soc.borrow().can_driven_level(0), now);
    soc.borrow_mut().can_sample_bit(0, level);      // 线与结果回送给控制器
    soc.borrow_mut().advance_ns(bit_ns);
}

// 一个不带控制器的节点（第二个 ECU / 注入面板）走同一套位机器。
let rival = bus.borrow_mut().add_peer("对手节点");
bus.borrow_mut()
    .queue_frame(rival, gsim_core::CanFrame::standard(0x100, b"win"));
```

---

## 5. 后续阶段的接入点（本阶段不包含其代码）

1. **更多片上控制器（CAN2、USB……）**
   * 在 `gsim-stm32f1` 新增模块，实现 `read_reg`/`write_reg`/`advance`/`reset`
     （`can.rs` 就是照这个套路写出来的完整例子）；
   * 用 `crate::impl_peripheral!(Type, "NAME")` 实现 `Peripheral`；
   * 在 `Soc` 增加字段、在 `dispatch_apbX_read/write` 增加一条 `region_offset` 分支，
     并在 `advance_once` 里按时钟门控推进；
   * 中断编号直接用 `gsim_core::irq::Irq::new(n)`（`lines` 里已按 F103 表填好编号）。
   * 若不想改核心解码表，也可以走 `Soc::attach_peripheral(base, size, Box<dyn Peripheral>)`
     ——运行期注册、读写与推进都会自动转发过去（`gsim-cpu` 的 `cpu_core` 测试即用它验证过同类机制）。
2. **更多片外器件**
   * 在 `gsim-devices` 里加一个模块：订阅 `Event`（引脚变化、SPI 帧、CAN 帧），
     需要回灌时调用 `Soc` 的注入接口（`spi_set_miso`、`gpio_set_input`、`can_receive`）；
   * I2C 侧：`I2c::transmit` 目前对空总线返回 NACK，器件模型接入时改为向总线请求 ACK/数据。
3. **多节点 CAN 网络**
   * `CanBus` 已经支持任意多个节点：每个 `Soc` 配一个 `CanTransceiver`，
     共享同一个 `Rc<RefCell<CanBus>>` 即可组成多点网络（GUI 的注入面板就是第四种节点）。
4. **更多虚拟外设（GUI 侧）**
   * 前端的外设与仿真核心完全解耦：`board.rs` 里加一个 `DeviceKind` 变体（引脚表 + 尺寸），
     在 `canvas.rs` 画符号、在 `Board::evaluate` 里写它的电气规则即可；
   * 需要反向驱动引脚（按键、编码器、外部芯片）时，调用 `Soc::gpio_set_input` 等已有的输入注入接口，
     不需要改动总线或 CPU 前端。

---

## 6. 第三方组件

| crate | 用途 | 许可证 |
| --- | --- | --- |
| [`armagnac`](https://crates.io/crates/armagnac) | ARMv7-M/Thumb-2 指令集解释器（含异常入栈/返回、SysTick、SCB/NVIC 寄存器） | MIT OR Apache-2.0 |
| [`object`](https://crates.io/crates/object) | ELF 解析（`gimli` 项目） | MIT OR Apache-2.0 |
| [`eframe` / `egui`](https://github.com/emilk/egui) | 图形前端（窗口、绘制、控件） | MIT OR Apache-2.0 |
| [`rfd`](https://crates.io/crates/rfd) | 原生文件选择对话框（加载 ELF） | MIT |
| [`egui_kittest`](https://crates.io/crates/egui_kittest) | 测试用离屏渲染器（仅 dev-dependency） | MIT OR Apache-2.0 |

选择这两个项目而不是自研，是为了把精力放在**外设行为**上，同时避免手写 ELF 解析与指令解码。

## 7. 测试

* `crates/gsim-core`：总线访问、NVIC 仲裁（优先级/嵌套/掩码）、SCB、SysTick、时间换算。
* `crates/gsim-stm32f1`：每个外设的寄存器级单元测试 + 跨模块集成测试
  （时钟树、GPIO/EXTI/NVIC 链路、位带别名、USART、SPI、TIM、ADC+DMA、I2C、看门狗、CAN 收发的完整 HAL 序列…）。
* `crates/gsim-cpu`：手写 Thumb 程序（不依赖交叉编译器）+ 真实 GCC 固件端到端执行
  （含“多中断使能不丢位”“`.data` 从 flash LMA 装载”两条回归测试）。
* `crates/gsim-gdb`：RSP 帧编解码单元测试，以及真实 TCP 上的调试会话
  （擦写 flash 并回读校验、断点/单步、数据断点、Ctrl-C、`monitor`、`qXfer` 描述）；
  另有一条用真实 `arm-none-eabi-gdb` 跑 `load` / `break` / `run` / `step` 的端到端测试。
* `crates/gsim-devices`：片外器件模型的单元测试 + 两条端到端链路：
  SPI1 主机写寄存器 → MAX7219 显示点阵；CAN 控制器发帧 → 收发器 → 虚拟总线 → 另一节点的接收 FIFO。
* `crates/gsimulator-gui`：板级电气规则（网络合并、短路、LED 极性）、EDA 式点击连线状态机、
  无窗口整帧渲染、可配置激励（引脚 / 方波 / 占空比 / 自定义函数表达式求值器），
  外设状态浏览与 CAN 总线两个窗口的离屏渲染，
  以及“内置激励 → 引脚 → 连线 → LED 点亮”“编码器 → MCU 引脚”
  “真实固件 → 虚拟终端”“终端输入 → USART 接收器”四条端到端链路。

当前 360 个测试全部通过（GPU 离屏渲染、真实固件执行、真实 GDB 会话等测试在缺少
GPU / 交叉编译器 / `arm-none-eabi-gdb` 时会自动跳过）。

```powershell
cargo test --workspace
```

