# Gsimulator

STM32F103 片上外设模拟器（第一阶段）。

用 Rust 实现的、可在 PC 上运行真实 ARM Cortex-M3 固件 ELF 的 STM32F103 外设模拟器：
外设模型按 RM0008 寄存器级建模，CPU 指令集来自开源生态项目
[armagnac](https://github.com/Ledger-Donjon/armagnac)（纯 Rust 的 Thumb/ARMv7-M 解释器），
ELF 解析来自 [object](https://github.com/gimli-rs/object)。

本阶段范围：**只实现 F103 的片上外设**。CAN 控制器、外部 SPI/I2C 器件、虚拟 CAN 总线不在本阶段，
代码中也没有它们的任何占位实现；架构上为它们预留了明确的接入点（见文末）。

项目由 6 个 crate 组成：

| crate | 作用 |
| --- | --- |
| `gsim-core` | 与芯片无关的基础设施（总线、NVIC/SCB/SysTick、事件、时间） |
| `gsim-stm32f1` | STM32F103 片上外设模型 |
| `gsim-devices` | **片外器件模型**：UART 终端、旋转编码器、MAX7219、CAN 收发器/总线 |
| `gsim-cpu` | Cortex-M3 执行前端 + ELF 装载 |
| `gsim-gdb` | **虚拟 ST-Link**：GDB 远程协议服务器（烧录 + 调试） |
| `gsimulator-cli` | 命令行（`run` / `gdb` / `demo` / `info`） |
| `gsimulator-gui` | **桌面图形前端**：PCB/原理图风格的虚拟接线台 |

---

## 0. 图形前端（虚拟接线台）

![Gsimulator 图形前端](docs/gui.png)

```powershell
cargo run -p gsimulator-gui                     # 自动运行内置 PA5 方波，接上 LED 立刻能看到闪烁
cargo run -p gsimulator-gui -- path\to\app.elf  # 用真实固件的引脚输出驱动外设
```

界面上可放置三类虚拟外设（LED、UART 终端、旋转编码器），都从工具栏的 **＋ 添加器件** 打开的
「器件库」窗口里挑选（分类 + 搜索，像嘉立创 EDA 的器件面板），见下文“可放置的虚拟外设”。

界面用 [egui/eframe](https://github.com/emilk/egui)（纯 Rust 立即模式 GUI）实现，分成几块：

* **画布**：中间是 STM32F103C8T6 符号（PA0–PA15 / PB0–PB15 / PC13–PC15），
  每个引脚带电平指示；右侧可放置虚拟外设与 `VCC`/`GND` 电源符号。
* **器件库（＋ 添加器件）**：窗口按「指示 / 通信 / 交互」分类列出所有可放置的器件，
  带符号预览、引脚名和搜索框。点卡片上的「放置」后画布跟随鼠标显示半透明符号，
  单击落点即可放下（`Esc` 结束）；也可以直接把卡片拖到画布上。
* **接线（类似嘉立创 EDA / EasyEDA）**：点一个引脚 → 点另一个引脚 即完成一根连线；
  也可以从一个引脚按住拖到目标引脚。连线按正交折线绘制，颜色跟随网络电平
  （绿=高、蓝=低、红=短路、暗=悬空）。`Delete` 删除选中的连线或器件，中键拖动平移，滚轮缩放。
* **状态面板**：仿真时间、实时倍率、指令数、中断/复位次数、总线访问与错误、完整时钟树
  （SYSCLK/HCLK/PCLK1/PCLK2/TIM2/ADC）、每个引脚的高低电平矩阵、虚拟外设列表（点亮状态 + 接线关系）、
  USART1 输出窗口。

驱动引脚的两个来源：

| 来源 | 说明 |
| --- | --- |
| 内置激励 | 没有固件时默认运行：前端按 120 ms 半周期通过 `GPIOA->BSRR` 翻转 PA5（走真实寄存器路径，不是伪造电平） |
| 真实固件 | `加载 ELF…` 选择一个 Cortex-M3 ELF，前端在 UI 线程内按预算执行指令并刷新状态 |

工具栏的「内置激励」是一个开关：再次点击即停止方波、把 PA5 置低并交还为输入，
仿真进入空闲状态（没有驱动源时「运行」按钮变灰，复位也不会把它重新打开）。

电气规则（`crates/gsimulator-gui/src/board.rs`，全部有单元测试）：
连线把引脚并成网络，网络电平 = 高/低/悬空/短路；LED 亮 = 阳极所在网络为高 **且** 阴极所在网络为低。
所以 `PA5 → LED.A`、`GND → LED.K` 是高电平点亮，`VCC → LED.A`、`PC13 → LED.K` 则是低电平点亮
（Blue Pill 板载 LED 就是这种接法）。

### 可放置的虚拟外设

| 外设 | 引脚 | 行为 | 操作 |
| --- | --- | --- | --- |
| **LED** | `A`（阳） `K`（阴） | 纯消费端，按网络电平点亮 | — |
| **UART 终端** | `RX` `TX` | 把 MCU 在所选 USART 上**真实移位输出**的字节显示成文本，并把输入的文本送进该 USART 的接收器 | 面板里选 USART1/2/3、输入后回车或点“发送”、“清空” |
| **旋转编码器** | `A` `B` `SW` | 正交编码器（自带 10 k 上拉，接触点被主动驱动）：一个定位点 = 4 个四分之一相位（格雷码），每 2 ms 释放一个边沿；轴按下时把 `SW` 拉低 | 画布上**滚轮**转旋钮、**点击**按轴；也可用面板按钮 |

接线规则（EDA 风格，模型在 `crates/gsim-devices/src/{uart,encoder}.rs`）：

* **UART**：只有当终端的 `RX` 真的接在该 USART 的 `TX` 引脚（USART1=PA9、USART2=PA2、USART3=PB10）上时，
  才收得到字节；只有 `TX` 接在该 USART 的 `RX` 引脚（PA10 / PA3 / PB11）上时，输入的内容才会进 MCU。
  没接通时终端把字节排队，面板显示“N 字节待发送（等待连线）”。
* **编码器**：`A`/`B`/`SW` 各自驱动所连的 MCU 引脚（走 `Soc::gpio_set_input`，
  所以 EXTI 边沿与 `IDR` 轮询都能看到）；没接线的接触点不会动任何引脚。

引脚注入与采样在**指令之间**进行（`crates/gsimulator-gui/src/bridge.rs` 把一帧切成约 1 ms 的仿真时间片，
每片先服务外设再推进 CPU），因此固件轮询 `IDR` 或等待 EXTI 都能真正捕捉到编码器边沿。

`cargo test -p gsimulator-gui` 会：跑板级电气规则测试、无窗口渲染整帧 UI（`egui_kittest`，GPU 不可用时跳过）、
验证“内置激励 → PA5 → 连线 → LED 点亮”，并跑两条端到端链路：
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

三个可以直接运行的能力：

```powershell
cargo run -p gsimulator-cli -- demo                # 无 ELF：脚本化巡览全部外设并打印时序
cargo run -p gsimulator-cli -- info                # 打印模拟的内存映射
cargo run -p gsimulator-cli -- run firmware.elf    # 加载并执行真实 Cortex-M3 ELF 固件
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

工作区里 `firmware/CAN` 的 CubeMX 工程（CAN + SPI + MAX7219）属于后续阶段：
它现在可以完整跑完 `HAL_Init` 与 `SystemClock_Config`（HSE 8 MHz → PLL ×9 → 72 MHz，
期间大量使用位带别名写 RCC 寄存器），随后停在第一个 CAN1 寄存器访问处：

```
loaded firmware\CAN\build\Debug\CAN.elf (12040 bytes, entry 0x08002B41)
error: bus error on the read of 0x40006400: CAN1 is out of scope for phase 1
            while executing 0x080001D0 (HAL_CAN_Init)
```

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
  指向模拟器的后端。所以本仓库只实现 GDB 服务器的这一半：端口、连接方式、`load` 语义都
  和 OpenOCD 一致，GDB/IDE 侧不用改。
- `load` 之后核心的 SP/PC 仍是加载前的值（真实芯片也一样），所以要么 `monitor reset`，
  要么直接用 `run`（`vRun` 会先复位再从向量表跑）。
- GDB 的寄存器缓存只在“停下”时刷新：`monitor reset` 之后如果想立刻看寄存器，先 `stepi`
  一次（`run` 之后的停点不受影响）。

---

## 2. 架构

```
crates/
  gsim-core/      与具体芯片无关的基础设施
  gsim-stm32f1/   STM32F103 片上外设模型（Soc + 各外设）
  gsim-cpu/       Cortex-M3 执行前端 + ELF 装载 + SoC 桥接
  gsimulator-cli/ 命令行（run / demo / info）
  gsimulator-gui/ 图形前端（虚拟接线台：画布 + 虚拟外设 + 状态面板）
firmware/         已有的 CubeMX 固件工程（CAN，属于后续阶段）
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


超时复位、看门狗复位都会走同一条 `Soc::system_reset()` 路径，并在 `RCC_CSR` 里留下复位原因。

### 已知简化（有意为之）

* 定时器的预装载寄存器（`ARPE`/`OCxPE`）立即生效，不做影子寄存器；PWM 占空比修改当期可见。
* 中断的 pending 跟随外设中断线电平（ISR 清标志即撤销 pending），不是严格的“边沿置位 + 显式清 pending”。
* 本阶段 I2C 总线上没有器件，因此主机寻址一律 NACK（`SR1.AF`），符合“空总线”的真实行为。
* SPI 只有一条 MISO 输入（由 `Spi::set_miso`/`Soc::spi_set_miso` 提供），没有外部器件模型。
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
`adc_set_channel_mv`、`tim_set_channel_input`。后续阶段的外部器件模型就是它们的替代驱动者。

---

## 5. 后续阶段的接入点（本阶段不包含其代码）

1. **新的片上控制器（例如 CAN）**
   * 在 `gsim-stm32f1` 新增模块，实现 `read_reg`/`write_reg`/`advance`/`reset`；
   * 用 `crate::impl_peripheral!(Type, "NAME")` 实现 `Peripheral`；
   * 在 `Soc` 增加字段、在 `dispatch_apbX_read/write` 增加一条 `region_offset` 分支，
     并在 `advance_once` 里按时钟门控推进；
   * 中断编号直接用 `gsim_core::irq::Irq::new(n)`（本阶段已按 F103 表填好了 `lines` 常量，
     未建模的编号留空，添加时不会与既有编号冲突）。
   * 若不想改核心解码表，也可以走 `Soc::attach_peripheral(base, size, Box<dyn Peripheral>)`
     ——运行期注册、读写与推进都会自动转发过去（`gsim-cpu` 的 `cpu_core` 测试即用它验证过同类机制）。
2. **外部 SPI/I2C 器件**
   * SPI 侧：把 `Soc::spi_set_miso` 的调用者从测试替换为器件模型，或在 `Spi` 内引入“总线端点”抽象；
   * I2C 侧：`I2c::transmit` 目前对空总线返回 NACK，器件模型接入时改为向总线请求 ACK/数据；
   * 两者都只需要驱动 `Event`/`Ctx` 与既有的定时推进，不需要改动 CPU 前端。
3. **虚拟 CAN 总线**
   * 多节点场景可用 `Rc<RefCell<...>>` 共享的总线对象，由各 `Soc` 的 CAN 模型收发；
   * 事件通道（`EventSink`）已经可以把帧投递给 CLI、GUI 或日志。
4. **更多虚拟外设（GUI 侧）**
   * 前端的外设与仿真核心完全解耦：`board.rs` 里加一个 `DeviceKind` 变体（引脚表 + 尺寸），
     在 `canvas.rs` 画符号、在 `Board::evaluate` 里写它的电气规则即可；
   * 需要反向驱动引脚（按键、编码器、外部芯片）时，调用 `Soc::gpio_set_input` 等已有的输入注入接口，
     不需要改动总线或 CPU 前端。

`firmware/CAN` 是现成的 CubeMX 工程，其中的 CAN 相关部分属于第二阶段；本阶段的模拟器加载
它会因为缺少 CAN 外设而在访问 `0x40006400` 时产生总线错误（这是预期行为）。

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
  （时钟树、GPIO/EXTI/NVIC 链路、位带别名、USART、SPI、TIM、ADC+DMA、I2C、看门狗…）。
* `crates/gsim-cpu`：手写 Thumb 程序（不依赖交叉编译器）+ 真实 GCC 固件端到端执行。
* `crates/gsim-devices`：片外器件模型（UART 终端、旋转编码器、MAX7219、CAN 收发器/总线）的单元测试。
* `crates/gsimulator-gui`：板级电气规则（网络合并、短路、LED 极性）、EDA 式点击连线状态机、
  无窗口整帧渲染，以及“内置激励 → PA5 → 连线 → LED 点亮”“编码器 → MCU 引脚”
  “真实固件 → 虚拟终端”“终端输入 → USART 接收器”四条端到端链路。

当前共 211 个测试全部通过（GPU 离屏渲染、真实固件执行等测试在缺少 GPU / 交叉编译器时会自动跳过）。

```powershell
cargo test --workspace
```

