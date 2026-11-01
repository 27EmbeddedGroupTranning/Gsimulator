//! Communication protocol document of the CAN motor, shown inside the app.
//!
//! The part's panel has a button that brings this window up, so the wire format
//! can be read without leaving the simulator or opening a datasheet. The text
//! is written from the model's own constants and the payload table
//! ([`EXAMPLES`]) is replayed through `gsim_devices::Motor` by a test, so the
//! document and the decoder cannot drift apart.

use eframe::egui;
use egui::{Color32, RichText};
use gsim_devices::motor::{DEFAULT_ACCEL_RPM_PER_S, DEFAULT_COMMAND_ID, MAX_RPM};

/// Colour the schematic uses for the motor, repeated here so the window and the
/// drawing read as the same part.
const ACCENT: Color32 = Color32::from_rgb(110, 205, 165);

/// Payload of `data[0..2]`, the target speed the part decodes from it, and what
/// that means for the shaft. The test below replays every row.
pub const EXAMPLES: [(u16, f32, &str); 5] = [
    (0x04B0, 1200.0, "正转"),
    (0x0BB8, 3000.0, "正转上限"),
    (0xF830, -2000.0, "反转"),
    (0x8000, -3000.0, "−32768 被钳到下限"),
    (0x0000, 0.0, "停止"),
];

/// Minimal transmit sequence of the CubeMX firmware (`firmware/CANSPI`).
const HAL_EXAMPLE: &str = r#"uint8_t data[2];
data[0] = (uint8_t)((uint16_t)rpm >> 8);   /* 高字节先上线 */
data[1] = (uint8_t)((uint16_t)rpm & 0xFFU);

CAN_TxHeaderTypeDef header = {0};
header.StdId = CAN_MOTOR_COMMAND_ID;       /* 默认 0x200 */
header.IDE   = CAN_ID_STD;
header.RTR   = CAN_RTR_DATA;
header.DLC   = 2U;

HAL_CAN_AddTxMessage(&hcan, &header, data, &mailbox);"#;

/// Identifier as the panels print it.
fn id_label(id: u16) -> String {
    format!("0x{id:03X}")
}

/// `data[0] data[1]` of a payload word, high byte first.
fn payload_label(payload: u16) -> String {
    let [high, low] = payload.to_be_bytes();
    format!("{high:02X} {low:02X}")
}

/// Live state of the part the document is attached to.
#[derive(Clone, Debug, PartialEq)]
pub struct MotorLive {
    pub name: String,
    pub command_id: u16,
    pub target_rpm: f32,
    pub rpm: f32,
    /// Frames the node accepted from the wire.
    pub frames: u64,
    /// Frames that carried a command this motor follows.
    pub commands: u64,
    /// `false` while the part is unpowered or off the pair.
    pub enabled: bool,
}

/// State of the protocol window.
#[derive(Debug, Default)]
pub struct MotorProtocolWindow {
    open: bool,
    /// Part the document is attached to, so the live rows follow that motor.
    device: Option<u32>,
}

impl MotorProtocolWindow {
    /// Shows the document of `device`, as the panel button does.
    pub fn open_for(&mut self, device: u32) {
        self.device = Some(device);
        self.open = true;
    }

    pub fn close(&mut self) {
        self.open = false;
    }

    pub fn is_open(&self) -> bool {
        self.open
    }

    pub fn device(&self) -> Option<u32> {
        self.device
    }

    pub fn show(&mut self, ctx: &egui::Context, live: Option<&MotorLive>) {
        if !self.open {
            return;
        }
        let mut open = self.open;
        egui::Window::new("CAN 电机通讯协议")
            .id(egui::Id::new("motor-protocol-doc"))
            .open(&mut open)
            .default_size([720.0, 620.0])
            .default_pos([380.0, 120.0])
            .resizable(true)
            .show(ctx, |ui| {
                egui::ScrollArea::vertical()
                    .auto_shrink([false, false])
                    .show(ui, |ui| self.body(ui, live));
            });
        self.open = open;
    }

    fn body(&self, ui: &mut egui::Ui, live: Option<&MotorLive>) {
        ui.horizontal(|ui| {
            ui.heading("CAN 电机通讯协议");
            match live {
                Some(live) => {
                    ui.label(RichText::new(&live.name).strong().color(ACCENT));
                    let (state, color) = if live.enabled {
                        ("在线", Color32::from_rgb(120, 200, 130))
                    } else {
                        ("断电或脱线", Color32::from_rgb(240, 190, 90))
                    };
                    ui.label(RichText::new(state).small().color(color));
                }
                None => {
                    ui.label(RichText::new("器件已删除").small().color(Color32::GRAY));
                }
            }
        });
        ui.label(
            RichText::new("本器件是总线上的一个独立节点：自己解码、自己应答，只听一条标准数据帧。")
                .color(Color32::GRAY),
        );
        ui.label(
            RichText::new(
                "器件模型 crates/gsim-devices/src/motor.rs · 固件参考 firmware/CANSPI/Core/Inc/can_motor.h",
            )
            .small()
            .color(Color32::from_rgb(130, 138, 150)),
        );

        self.frame_section(ui);
        self.speed_section(ui);
        self.shaft_section(ui);
        self.ack_section(ui);
        self.firmware_section(ui);
        self.live_section(ui, live);
    }

    fn frame_section(&self, ui: &mut egui::Ui) {
        ui.add_space(6.0);
        heading(ui, "1 · 帧格式");
        let rows = [
            (
                "帧类型",
                "标准数据帧".to_string(),
                "11 位标识符；扩展帧（IDE=1）一律忽略",
            ),
            (
                "标识符",
                format!("默认 {}，可改", id_label(DEFAULT_COMMAND_ID)),
                "面板的「指令 ID」或工程文件里的 cmd=；只有完全相等的 ID 才算指令",
            ),
            ("RTR", "数据帧".to_string(), "远程帧（RTR=1）忽略"),
            ("DLC", "≥ 2".to_string(), "少于 2 个数据字节的帧不算指令"),
            ("data[0]", "转速高字节".to_string(), "最先上线（大端）"),
            ("data[1]", "转速低字节".to_string(), ""),
            (
                "data[2..7]",
                "忽略".to_string(),
                "填什么都行；示例固件把余下字节清零",
            ),
        ];
        egui::Grid::new("motor-frame-fields")
            .num_columns(3)
            .striped(true)
            .spacing([14.0, 4.0])
            .show(ui, |ui| {
                for header in ["字段", "取值", "说明"] {
                    ui.label(RichText::new(header).small().color(Color32::GRAY));
                }
                ui.end_row();
                for (field, value, note) in rows {
                    ui.monospace(field);
                    ui.label(value);
                    ui.label(RichText::new(note).small().color(Color32::GRAY));
                    ui.end_row();
                }
            });
        ui.label(
            RichText::new(format!(
                "例：ID {}，data = [{}]，DLC = 2 → 目标 {:+.0} rpm",
                id_label(DEFAULT_COMMAND_ID),
                payload_label(EXAMPLES[0].0),
                EXAMPLES[0].1,
            ))
            .small()
            .color(Color32::GRAY),
        );
    }

    fn speed_section(&self, ui: &mut egui::Ui) {
        ui.add_space(6.0);
        heading(ui, "2 · 转速编码");
        for line in [
            "大端有符号 16 位（i16，高字节在前），单位 rpm。".to_string(),
            "正数 → 正转，负数 → 反转，0 → 目标为 0，轴滑行停下。".to_string(),
            format!(
                "有效范围 −{MAX_RPM:.0}…+{MAX_RPM:.0} rpm；超出范围按边界处理（例如 0x8000 是 −32768，只当成 −{MAX_RPM:.0} rpm）。"
            ),
            "每一帧只改「目标转速」，实际转速由轴按加速度逼近。".to_string(),
        ] {
            ui.label(RichText::new(format!("• {line}")).small());
        }
        ui.add_space(4.0);
        egui::Grid::new("motor-payload-examples")
            .num_columns(3)
            .striped(true)
            .spacing([14.0, 4.0])
            .show(ui, |ui| {
                for header in ["data[0] data[1]", "目标转速", "含义"] {
                    ui.label(RichText::new(header).small().color(Color32::GRAY));
                }
                ui.end_row();
                for (payload, rpm, note) in EXAMPLES {
                    ui.monospace(payload_label(payload));
                    ui.monospace(format!("{rpm:+.0} rpm"));
                    ui.label(RichText::new(note).small().color(Color32::GRAY));
                    ui.end_row();
                }
            });
    }

    fn shaft_section(&self, ui: &mut egui::Ui) {
        ui.add_space(6.0);
        heading(ui, "3 · 轴的运动");
        let seconds = f64::from(MAX_RPM) / f64::from(DEFAULT_ACCEL_RPM_PER_S);
        for line in [
            format!(
                "加速度 {DEFAULT_ACCEL_RPM_PER_S:.0} rpm/s：从静止到 ±{MAX_RPM:.0} rpm 约需 {seconds:.1} s。"
            ),
            "按仿真时间积分转过的圈数；暂停仿真时轴也停住。".to_string(),
            "面板与符号上显示当前转速、目标转速、圈数，转轴标记跟着角度转。".to_string(),
        ] {
            ui.label(RichText::new(format!("• {line}")).small());
        }
    }

    fn ack_section(&self, ui: &mut egui::Ui) {
        ui.add_space(6.0);
        heading(ui, "4 · 应答与时序");
        for line in [
            "电机只应答、不发送：它在自己解码的帧的 ACK 槽拉显性，总线上没有来自电机的数据帧，也没有状态回传。",
            "断电，或 CANH/CANL 没有和收发器接在同一对线上时，节点退出总线：既不转也不应答。",
            "位速率由控制器的 BTR 决定（示例固件 200 kbit/s）；线路没有终端电阻时质量变差，位可能被翻转，控制器会记错误。",
            "面板上的「正转 / 反转 / 停止」不会直接改模型，而是让外部节点在总线上真的发一帧（走仲裁、被电机应答）。",
        ] {
            ui.label(RichText::new(format!("• {line}")).small());
        }
    }

    fn firmware_section(&self, ui: &mut egui::Ui) {
        ui.add_space(6.0);
        heading(ui, "5 · 固件示例（HAL）");
        ui.label(RichText::new(HAL_EXAMPLE).monospace().small());
    }

    fn live_section(&self, ui: &mut egui::Ui, live: Option<&MotorLive>) {
        ui.add_space(6.0);
        heading(ui, "6 · 本机实时状态");
        let Some(live) = live else {
            ui.label(
                RichText::new("这个器件已经不在图纸上了，没有可显示的状态。")
                    .small()
                    .color(Color32::GRAY),
            );
            return;
        };
        let rows = [
            ("名称", live.name.clone()),
            ("指令 ID", id_label(live.command_id)),
            ("目标转速", format!("{:+.0} rpm", live.target_rpm)),
            ("实际转速", format!("{:+.1} rpm", live.rpm)),
            ("收到帧", format!("{}", live.frames)),
            ("生效指令", format!("{}", live.commands)),
        ];
        egui::Grid::new("motor-live-rows")
            .num_columns(4)
            .striped(true)
            .spacing([14.0, 4.0])
            .show(ui, |ui| {
                for (index, (label, value)) in rows.into_iter().enumerate() {
                    ui.label(RichText::new(label).color(Color32::GRAY));
                    ui.monospace(value);
                    if index % 2 == 1 {
                        ui.end_row();
                    }
                }
                ui.end_row();
            });
        let speed = live.target_rpm.round().clamp(-32768.0, 32767.0) as i16;
        ui.label(
            RichText::new(format!(
                "本机当前会执行的帧：ID {}  data = [{}]  DLC = 2",
                id_label(live.command_id),
                payload_label(u16::from_be_bytes(speed.to_be_bytes())),
            ))
            .monospace()
            .small(),
        );
    }
}

fn heading(ui: &mut egui::Ui, text: &str) {
    ui.label(RichText::new(text).strong().color(ACCENT));
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::cell::RefCell;
    use std::rc::Rc;

    use gsim_core::can::CanFrame;
    use gsim_devices::{CanBus, Motor};

    fn motor() -> Motor {
        Motor::attach(Rc::new(RefCell::new(CanBus::new())), "M1")
    }

    fn raw_input() -> egui::RawInput {
        egui::RawInput {
            screen_rect: Some(egui::Rect::from_min_size(
                egui::Pos2::ZERO,
                egui::Vec2::new(960.0, 720.0),
            )),
            ..Default::default()
        }
    }

    #[test]
    fn the_documented_payloads_are_the_ones_the_part_decodes() {
        let mut motor = motor();
        for (payload, target, note) in EXAMPLES {
            let frame = CanFrame::standard(motor.command_id(), &payload.to_be_bytes());
            assert!(
                motor.on_frame(&frame),
                "payload {payload:#06X} ({note}) must be a command"
            );
            let expected = (payload as i16) as f32;
            let expected = expected.clamp(-MAX_RPM, MAX_RPM);
            assert!(
                (motor.target_rpm() - expected).abs() < 1e-3,
                "payload {payload:#06X} ({note}): document says {target}, motor decodes {}",
                motor.target_rpm()
            );
        }
    }

    #[test]
    fn the_frames_the_document_calls_ignored_are_ignored() {
        let mut motor = motor();
        let id = motor.command_id();
        let data = 1200i16.to_be_bytes();

        assert!(!motor.on_frame(&CanFrame::extended(u32::from(id), &data)));
        assert!(!motor.on_frame(&CanFrame::remote(CanFrame::standard(id, &data))));
        assert!(!motor.on_frame(&CanFrame::standard(id, &data[..1])));
        assert!(!motor.on_frame(&CanFrame::standard(id + 1, &data)));
        assert_eq!(motor.target_rpm(), 0.0, "nothing must have been decoded");
    }

    #[test]
    fn the_window_paints_only_while_it_is_open() {
        let ctx = egui::Context::default();
        let mut window = MotorProtocolWindow::default();
        let live = MotorLive {
            name: "M1".to_string(),
            command_id: DEFAULT_COMMAND_ID,
            target_rpm: 1200.0,
            rpm: 800.0,
            frames: 3,
            commands: 2,
            enabled: true,
        };

        let output = ctx.run(raw_input(), |ctx| window.show(ctx, Some(&live)));
        assert!(output.shapes.is_empty(), "a closed window must not paint");

        window.open_for(4);
        assert!(window.is_open());
        assert_eq!(window.device(), Some(4));
        let output = ctx.run(raw_input(), |ctx| window.show(ctx, Some(&live)));
        assert!(!output.shapes.is_empty(), "the document must paint");

        // A part that was deleted while the window is up still renders.
        let output = ctx.run(raw_input(), |ctx| window.show(ctx, None));
        assert!(!output.shapes.is_empty());

        window.close();
        assert!(!window.is_open());
    }
}
