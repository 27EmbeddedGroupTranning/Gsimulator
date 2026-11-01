# AGENTS.md — Gsimulator 开发指南

面向在本仓库工作的 AI/人类贡献者的操作手册。重点记录**改动流程**，架构说明见 `README.md`。

## 0. 工程速览

```
crates/
  gsim-core/      与芯片无关的基础设施：MemoryBus/AccessWidth/BusError、Ctx+Event、Irq、Nvic/Scb/SysTick、纳秒↔周期
  gsim-stm32f1/   STM32F103 片上外设（Soc + rcc/gpio/afio/exti/usart/spi/i2c/tim/adc/dma/flash/pwr/rtc/watchdog/crc/can/bitband）
  gsim-devices/   片外器件模型（uart 终端、encoder 旋转编码器、max7219、can_bus、crystal 晶振）——只依赖 gsim-core
  gsim-cpu/       Cortex-M3 执行前端（armagnac）+ ELF 装载 + Soc 桥接
  gsim-gdb/       虚拟 ST-Link：GDB 远程协议服务器（rsp 帧 / target.xml / flash 编程 / 断点）
  gsimulator-cli/ 命令行：run / gdb / flash / demo / info
  gsimulator-gui/ 图形前端：原理图画布 + 虚拟外设（board / canvas / bridge / sim / app）
```

```powershell
cargo test --workspace                       # 全部测试
cargo clippy --workspace --all-targets       # 提交前必须零告警
cargo fmt --all                              # 提交前必须格式化
cargo run -p gsimulator-gui                  # 跑图形前端
```

通用约定：

- **注释与文档注释用英文，界面/错误文案用中文**；测试名用英文描述行为（如 `led_wired_to_vcc_and_a_low_pin_is_active_low`）。
- 时间只能通过 `Ctx` / `NanoAccumulator` / `Soc::advance_ns` 推进，**禁止**读真实时钟、禁止 `sleep`、禁止忙等。
- 外设状态改动后必须通过 `cx.emit(Event::...)` 上报副作用，不要直接操作别的外设或 NVIC。
- 片外器件（LED、按键、外部芯片、虚拟总线）的模型**只能**放在 `gsim-devices`，
  接线与仿真调度放在 `gsimulator-gui`（`board.rs` 图纸、`bridge.rs` 接线总线）；
  片上控制器（RCC/GPIO/CAN…）放在 `gsim-stm32f1`。两侧只通过 `Soc` 的公共接口
  （`gpio_set_input`、`Event`、`attach_peripheral`）交互，不许互相 include 内部字段。
- 提交前跑一遍第 1 节末尾的“完成定义（Definition of Done）”。

---

## 1. 给图形前端加一个虚拟外设（主要流程）

目标：在原理图画布上放一个新器件（按钮、蜂鸣器、数码管、电阻、外部芯片…），能像嘉立创 EDA 那样把 MCU 引脚连到它的引脚上，并让器件状态跟随仿真变化（或反过来驱动 MCU 引脚）。

### 步骤 0：先把“电气行为”写成一句话

先决定三件事，再动手写代码：

| 问题 | 影响 |
| --- | --- |
| 器件状态由什么决定？（网络电平 / 组合逻辑 / 需要时间推进） | 决定 `Board::evaluate` 里怎么写，以及 `BoardState` 要加什么字段 |
| 器件是否有**内部状态**（终端日志、旋钮相位、显存…）？ | 放进 `gsim_devices` 的器件模型，挂在 `Device::hardware` 上（见步骤 1.5） |
| 器件是否**驱动**引脚？（按键、外部芯片、编码器会；LED、终端不会） | 决定是否要在 `bridge.rs` 里调用 `Soc::gpio_set_input`（见步骤 4） |

已有约定：连线把引脚并成**网络**，网络电平 ∈ `高 / 低 / 悬空 / 短路`（`board.rs::NetLevel`）。
两级分工是本仓库的硬约束：

* **`crates/gsim-devices`**：纯状态机（只依赖 `gsim-core`），看得见引脚事件与总线帧，看不见寄存器；
  `on_event(&Event)` 消费 SoC 事件，导出可读状态；有交互输入的器件补 `send()`/`rotate()` 之类方法。
* **`crates/gsimulator-gui`**：图纸（`Board`）、符号（`canvas.rs`）、面板（`app.rs`）、
  以及**接线总线**（`bridge.rs`，把两边按实际连线接起来）。

### 步骤 1：`crates/gsimulator-gui/src/board.rs` — 数据模型

1. 在 `DeviceKind` 里加变体，并把它加进 `DeviceKind::ALL`：
   ```rust
   pub enum DeviceKind { Led, Uart, Encoder, Button /* 新增 */ }
   ```
2. 定义引脚表 `PinDef { name, role, side }`，补 `DeviceKind::pins()`、`body_size()`、`prefix()`、`label()`。
   `PinRole` 现在有 `Positive / Negative / Rx / Tx / Phase / Button`，需要新语义就往里加。
   引脚顺序就是 `PinRef::Device { device, pin }` 的索引，**只能追加，不能改序**；
   多引脚器件用 `pub const XXX_PIN: u8` 常量引用（如 `UART_RX`、`ENCODER_A`）。
   **引脚分区是约定**：所有接 MCU 的信号放 `Side::Left`，总线/输出（`CANH`/`CANL`、`DOUT`、LED 的 `K`）放 `Side::Right`，
   `VCC` 与 `GND` 都放 `Side::Top`（数组里 `VCC` 在前，符号上就在左边）。`pins_are_grouped_by_electrical_role` 会校验这张表。
3. 引脚在符号上的位置由 `Device::pin_pos` 自动沿边均匀分布，多引脚不必手工排布。
4. 放置走统一的 `Board::add_device(kind, pos)`（`add_led`/`add_uart`/`add_encoder` 只是薄封装），
   名字前缀与强调色都在里面生成（`Device.accent`，LED 用它当灯色）。
5. 有内部状态的器件：在 `Hardware` 里加一个变体，并在 `add_device` 里构造它：
   ```rust
   pub enum Hardware { None, Uart(gsim_devices::Uart), Encoder(gsim_devices::Encoder), Button(gsim_devices::Button) }
   ```
   运行时状态**不要**塞进 `Board` 的其他字段，也不要在 `evaluate` 里改它。
6. 纯电气规则（LED 亮不亮、网络是否短路）写在 `Board::evaluate` + `BoardState` 里；
   需要时间推进或事件输入的器件（编码器、终端）**不要**写进 `evaluate`，交给 `bridge.rs`。
7. 带电源的器件：在引脚表里补 `PinRole::Vcc` 和 `PinRole::Ground` 各一个 `PinDef`（都放顶边），
   `DeviceKind::supply_pins()` 会自动认出它们。`bridge::service` 每片都用 `Board::supplies` 判定供电，
   **`VCC`/`GND` 没接好（或接反）的器件不参加仿真**：不收事件、不驱动引脚、不上总线，
   状态面板显示「未接（不工作）」。给带电源的器件写端到端测试时要先接电源
   （参考 `crates/gsimulator-gui/tests/devices.rs` 的 `power()` 助手）。
8. 存档（`.gsim`）：在 `crates/gsimulator-gui/src/project.rs` 的 `kind_token` 里给新器件一个稳定小写标记
   （反查走 `DeviceKind::ALL`）；有需要保存的器件选项时同时扩展 `apply_option`（`key=value` 形式），
   并在该文件的 `mod tests` 里补一条 round-trip 用例。

### 步骤 2：`crates/gsimulator-gui/src/canvas.rs` — 画符号

1. 只需要改 `Schematic::paint_device`：按 `device.kind` 分支调用一个 `paint_xxx_symbol`。
   **以下都是通用的，不用改**：引脚桩与引脚名、命中测试（`Schematic::pin_at`）、拖动移动、
   选中高亮、连线端点定位、`content_bounds`。
2. 要显示动态状态就从 `device.hardware` 读（终端日志、旋钮角度），参考 `paint_uart_symbol` /
   `paint_encoder_symbol`；文字记得按框宽截断，别画到框外。
3. 需要鼠标手势（滚轮/点击改变器件）时，在 `show()` 里像 `encoder_at` 那样先做命中测试，
   并注意**不要**抢走画布自身的缩放/平移（滚轮在器件上时优先给器件）。

### 步骤 3：`crates/gsimulator-gui/src/app.rs` — 工具栏与面板

1. 工具栏不需要逐个加按钮：`for kind in DeviceKind::ALL { ui.button("＋ " + kind.label()) }`。
2. `device_list()` 里按 `device.kind` 加一个分支写状态文案，再补一个 `xxx_controls()` 给需要交互的器件
   （参考 `uart_controls` 的端口选择 + 终端输入、`encoder_controls` 的“逆/顺时针/按下轴”）。
3. 器件的输入控件要直接改 `board.device_mut(id).hardware.xxx_mut()`，不要绕道全局状态。

### 步骤 4：器件要驱动 MCU 引脚时（按键、外部芯片、编码器）

引脚驱动的**唯一**入口是 `bridge.rs`：它是“每片仿真时间服务一次外设”的地方，
新增驱动型器件时把逻辑加进 `bridge::service`，不要散落到 `App` 或器件模型里。
`Simulation` 提供：

```rust
// crates/gsimulator-gui/src/sim.rs
sim.drive_pin((port, pin), high);        // 等价于 Soc::gpio_set_input
sim.send_uart(unit, byte) -> bool;       // 器件往 MCU 送“数据”时用，忙时返回 false
```

参考实现（编码器）：`bridge.rs` 里先 `encoder.poll(now_ns)` 释放到期边沿，再
`sim.drive_pin(wiring.encoder_pins[0], phase.a)` 把接触点写到 MCU 引脚上——
**每次服务都写**，这样暂停或刚开始时引脚也保持在器件状态上。

时序要点：

- 引脚注入发生在**指令切片之间**：`bridge::run_frame` 把一帧切成约 1 ms 的仿真时间片，
  每片先 `service()` 再 `advance_frame()`，所以固件轮询 `IDR` 或等 EXTI 都能看到边沿。
- 切片粒度由 `app.rs` 的 `SLICE_NS` 决定；比 1 ms 更密的边沿会被合并成一次跳变。
- 只驱动“真的连到该器件”的引脚：用 `Board::is_connected(...)` / `Board::mcu_pin_for(pin)` 解析连线，
  参考 `bridge::Wiring::collect`（UART 的监听/回话、编码器的三个接触点都是这么判断的）。
- 同一引脚被多个器件驱动时没有仲裁（保持单驱动）；要支持时先定规则再实现。

注意：

- 需要边沿/中断语义时不用额外处理：`Soc::gpio_set_input` 之后 EXTI 会在下一次推进时采样到电平变化。
- 不要在前端直接改 `GpioPort` 的私有状态；要么用 `Soc::gpio_set_input`，要么给 `Soc` 加一个明确的公共方法。
- 器件往 MCU 送“数据”（而不是电平）时不要伪造 `Event`：UART 终端走 `Simulation::send_uart`，
  它在接收器没开（`UE`/`RE` 未置位）或 RXNE 仍满时返回 `false`，调用方把字节排队等下一片再试。

### 步骤 5：测试（必须）

1. **器件模型单元测试**（`crates/gsim-devices/src/xxx.rs` 底部 `mod tests`）：
   真值表、边界值、时序（如编码器的格雷码顺序与边沿间隔、终端的文本化规则）。
2. **电气规则单元测试**（`board.rs` 底部 `mod tests`）：极性、悬空、短接、`VCC/GND` 直连短路。
3. **接线端到端测试**（`crates/gsimulator-gui/tests/devices.rs`）：器件连上引脚后必须在
   `Simulation` 里看到效果，例如“转旋钮 → MCU 引脚走完整格雷码”“真实固件 → 虚拟终端出现 `GSIM`”。
4. **交互与渲染测试**：`tests/ui_smoke.rs` 的点击连线断言 + `tests/snapshot.rs` 离屏渲染；
   `GSIM_SNAPSHOT=1` 可导出 `target/gui-snapshot*.png` 人工复核（GPU 不可用时测试自动跳过）。

### 步骤 6：文档

- `README.md` 第 0 节的“可放置的虚拟外设”表格：补一行器件、引脚、行为、操作方式；
- 若接线规则不直观（数码管要 8 根线、UART 要 TX/RX 交叉），在表格下的“接线规则”里写清楚。

### 完成定义（Definition of Done）

```powershell
cargo fmt --all
cargo clippy --workspace --all-targets   # 零告警
cargo test -p gsim-devices -p gsimulator-gui
cargo run -p gsimulator-gui              # 手动确认：能放置、能连线、状态随仿真变化
```

并且：

- [ ] 器件模型只依赖 `gsim-core`，`gsim-stm32f1` / `gsim-cpu` 里**没有**出现任何片外器件代码；
- [ ] 器件不依赖真实时间，仿真暂停时状态冻结（只有画布上的鼠标手势用真实时间）；
- [ ] 新增代码有测试，且测试在没有 GPU / 没有交叉编译器的机器上能跳过而不是失败。

---

## 2. 给核心加一个片上外设（片上控制器流程）

参考实现：`crates/gsim-stm32f1/src/usart.rs`（带时序推进）、`adc.rs`（带 DMA 请求）、
`can.rs`（bxCAN，第二阶段引入）——以仓库里当前实际存在的模块为准。

1. `map.rs` 加基址与寄存器偏移（`pub mod xxx { BASE, ... }`），中断线用 `gsim_core::irq::lines` 里已有的常量。
2. 新模块实现 `read_reg(cx, offset, width)` / `write_reg(cx, offset, width, value)` / `advance(cx, elapsed_ns)` / `reset()`，
   并用 `impl_peripheral!(Type, "NAME")` 实现 `Peripheral`。
3. `Soc` 里加字段：在 `dispatch_apbX_read/write` 加 `region_offset` 分支、在 `advance_once` 里按 RCC 门控推进、
   在 `reset`/`system_reset` 里复位它。
4. 中断：状态变化时 `cx.emit(Event::Irq { irq, high })`，由 `Soc` 落到 NVIC（`Event::Irq` 是**电平跟随**的）。
5. DMA：请求路由表在 `dma.rs::Request`，需要时加一条并能被 `Soc` 的 DMA 引擎识别。
6. 测试：模块内单元测试（真值表 + 边界值）+ `crates/gsim-stm32f1/tests/` 里的跨模块集成测试；
   能跑真实固件时，在 `crates/gsim-cpu/tests/` 增加端到端用例（会调用 `arm-none-eabi-gcc`，缺失时跳过）。
7. 若寄存器访问需要位带别名（HAL 大量使用），确认 `bitband.rs` 的 `decode` 能覆盖到——片内外设区 0x4000_0000 起 1 MiB 内自适应。

---

## 3. 给调试服务器加能力（虚拟 ST-Link）

`crates/gsim-gdb` 把 GDB 接到 `Soc` 上：`rsp.rs` 负责 `$…#xx` 帧编解码，`target.rs` 是“探针视角”
的设备操作（寄存器、内存、断点、用 FLASH 控制器编程），`server.rs` 是 RSP 分发，`connection.rs`
是带缓冲的 socket，`gsimulator gdb` 只是薄薄一层 CLI。

协议层与传输层是分开的，两处共用同一份分发：

* `DebugTarget`（`target.rs`）描述“探针能对设备做什么”。`Target` 为 CLI 实现它（自带 `Soc`/`Cpu`），
  图形前端为 `Simulation` 实现它（`crates/gsimulator-gui/src/debugger.rs` 的 `CoreTarget`）。
* `GdbServer::dispatch(target, payload) -> Reply` 不碰 socket；`Reply::Resume` / `Reply::Step`
  把“要不要跑”交回调用方：CLI 在 `session()` 里阻塞着跑到停止，GUI 只在帧循环里推进仿真，
  所以**断点停的是画布上那份仿真**。
* GUI 的 `DebugService` 用非阻塞 `Connection`（`poll_wait`、`set_non_blocking`），
  每帧最多处理 `PACKETS_PER_FRAME` 个包；窗内的「数据包日志」与 `GSIM_DEBUG_PACKETS=1`（stderr）
  都能看收发。三条**不能破坏**的约束：
  * **socket 只在这个帧循环里被服务**，所以 `App::draw` 在 `DebugWindow::needs_frames()` 为真时必须
    `ctx.request_repaint()`；否则暂停时窗口不重绘，GDB 等不到回包就报 `Ignoring packet error`。
  * **`vCont;c` 不能先回 `OK`**：真探针在 continue 之后下一条包就是停止回复，多一个 `OK` 会让部分 GDB 版本报错。
  * **新连接取代旧连接**：VS Code 每次会话重连而旧 socket 常常还开着，只服务"第一个"客户端会把新调试器饿死。
  * **每个新连接都要 `GdbServer::begin_session()`**：`QStartNoAckMode` 只属于一个连接，
    泄漏到下一次会话会少发 `+`，GDB 报 `Packet instead of Ack, ignoring it` 并最终超时。
  * **`Connection::send` 允许半写**：socket 非阻塞时可能只写出一部分，剩余字节留在 `outgoing`
    里下次 `poll_wait` 补发；直接 `write_all` 会留下半个包，GDB 会报 packet error 并失步。

新增一个 RSP 请求的流程：

1. 在 `server.rs::dispatch`（或 `query` / `extended`）里加分支；未知包返回 `Reply::None`
   （不回包），GDB 会当作“不支持”继续跑，不要回 `E`。
2. 需要新的设备能力时加到 `target.rs` 的 `DebugTarget` 上（CLI 的 `Target` 用控制器寄存器实现，
   GUI 的 `CoreTarget` 用 `Soc` 的公共接口实现），**不要**在 `server.rs` 里直接摸 `Soc`：
   flash 编程在 CLI 里必须走 `FLASH_KEYR` / `CR` / `AR` / `SR`（照 `Target::flash_erase` / `flash_write` 写），
   这样固件、GDB、以后可能加的其它传输层看到的是同一份控制器状态。
3. 在 `tests/gdb_session.rs` 里加一条真实 socket 的用例（客户端就是文件顶部的 `Client`）；
   带二进制载荷的包（`X`、`vFlashWrite`）用 `send_bytes` 发。
   GUI 那份服务在 `crates/gsimulator-gui/tests/debug_service.rs` 里测（含一条跑真实 `gdb` 的用例）。
4. `tests/real_gdb.rs` 会用真实的 `arm-none-eabi-gdb` 跑 `load` / `break` / `run` / `step`，
   机器上没有交叉工具链时自动跳过。

约束：

- **单线程**：`Cpu` / `Soc` 不是 `Send`，模拟器必须和服务器在同一线程。`continue` 用
  “跑 10 万条指令 → 非阻塞 poll 一次 socket”的切片循环实现 Ctrl-C，不要为共享 `Soc` 引入线程。
- 一次只服务一个 GDB 连接（和真实探针一致）；断开后 `Target` 保留，重连能接着调。
- `Target::attach` 只在核心还没执行过指令时从向量表复位，重连不要偷偷改动现场。
- 时间推进只能走 `Cpu` 的接口，禁止在调试服务器里读真实时钟或 sleep。

---

## 4. 常见陷阱

- **不要**在前端直接读 `Soc` 的内部寄存器来判断引脚电平，用 `Simulation::pin_levels()`（内部走 `GpioPort::input_level`）。
- **不要**把 `Schematic` 的状态放进 `Board`：画布状态（缩放、选中、正在拉的线）属于 UI，`Board` 只存拓扑。
- 连线是**点对点**的，网络靠 `Board::nets()` 传递闭包合并；新增器件时不要假设“一根线 = 一个网络”。
- UI 每帧都会重算 `board.evaluate(...)`，规则函数必须是纯函数且足够快（器件数量级 < 100，保持 O(n²) 以内）。
- 器件引脚索引一旦发布就不能改序（测试、用户连线、以及后续可能加的工程存档都依赖它）。
- `egui` 里所有绘制都走 `Painter`，坐标要先从 board 空间换算（`Schematic::to_screen`），别用屏幕绝对坐标。


