//! Saving and loading a schematic as a small text project file.
//!
//! The format is line based so a saved circuit diffs cleanly and can be fixed
//! by hand:
//!
//! ```text
//! gsimulator project 1
//! # device <id> <kind> <x> <y> [key=value ...]
//! device 1 led 600.00 120.00
//! device 2 resistor 700.00 300.00 ohms=120
//! # wire <pin> <pin> [corner ...]
//! wire mcu:0:5 dev:1:0
//! wire dev:2:0 dev:1:1 430.00,150.00 500.00,150.00
//! ```
//!
//! Pins are `mcu:<port>:<pin>` or `dev:<device id>:<pin>`, corners are
//! `x,y` waypoints in board space. Devices are re-placed in file order, so the
//! automatic names (`LED1`, `UART2`, …) and accents match the saved circuit.
//! A junction is stored like any other part (`device 4 junction 320.00 150.00`):
//! the wire it taps is split in two, both halves ending on its only pin.

use std::fmt::Write as _;
use std::fs;
use std::path::Path;

use eframe::egui::Pos2;

use crate::board::{Board, DeviceId, DeviceKind, Hardware, PinRef};
use gsim_devices::TransceiverMode;

/// First line of every project file; also carries the format version.
pub const MAGIC: &str = "gsimulator project 1";
/// Extension the file dialogs suggest.
pub const EXTENSION: &str = "gsim";

/// Serialises the schematic: one line per device, then one line per wire.
pub fn to_text(board: &Board) -> String {
    let mut text = String::new();
    let _ = writeln!(text, "{MAGIC}");
    for device in board.devices() {
        let _ = write!(
            text,
            "device {} {} {} {}",
            device.id,
            kind_token(device.kind),
            number(device.pos.x),
            number(device.pos.y),
        );
        match &device.hardware {
            Hardware::Resistor(resistor) => {
                let _ = write!(text, " ohms={}", resistor.ohms());
            }
            Hardware::Uart(uart) => {
                let _ = write!(text, " unit={}", uart.unit());
            }
            Hardware::Can(transceiver) => {
                let _ = write!(text, " mode={}", mode_token(transceiver.borrow().mode()));
            }
            Hardware::Scope(scope) => {
                let scope = scope.borrow();
                let _ = write!(
                    text,
                    " window={} frozen={}",
                    scope.window_ns(),
                    u8::from(scope.frozen())
                );
            }
            Hardware::None | Hardware::Encoder(_) | Hardware::Matrix(_) | Hardware::Crystal(_) => {}
        }
        text.push('\n');
    }
    for wire in board.wires() {
        let _ = write!(text, "wire {} {}", pin_token(wire.a), pin_token(wire.b));
        for bend in &wire.bends {
            let _ = write!(text, " {},{}", number(bend.x), number(bend.y));
        }
        text.push('\n');
    }
    text
}

/// Parses a project file back into a board.
pub fn from_text(text: &str) -> Result<Board, String> {
    let mut board = Board::new();
    let mut ids: Vec<(u32, DeviceId)> = Vec::new();
    let mut wires: Vec<(usize, PinToken, PinToken, Vec<Pos2>)> = Vec::new();
    let mut magic = false;

    for (index, raw) in text.lines().enumerate() {
        let line_no = index + 1;
        let line = raw.trim();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        if !magic {
            if line != MAGIC {
                return Err(format!(
                    "第 {line_no} 行：不是 Gsimulator 工程文件（缺少 `{MAGIC}`）"
                ));
            }
            magic = true;
            continue;
        }
        let fields: Vec<&str> = line.split_whitespace().collect();
        match fields[0] {
            "device" => {
                if fields.len() < 5 {
                    return Err(format!("第 {line_no} 行：`device` 需要编号、类型、x、y"));
                }
                let saved: u32 = fields[1]
                    .parse()
                    .map_err(|_| format!("第 {line_no} 行：器件编号 `{}` 不是整数", fields[1]))?;
                if ids.iter().any(|(old, _)| *old == saved) {
                    return Err(format!("第 {line_no} 行：器件编号 {saved} 重复"));
                }
                let kind = kind_from_token(fields[2])
                    .ok_or_else(|| format!("第 {line_no} 行：未知器件类型 `{}`", fields[2]))?;
                let x = parse_number(fields[3], line_no, "x")?;
                let y = parse_number(fields[4], line_no, "y")?;
                let id = board.add_device(kind, Pos2::new(x, y));
                for field in &fields[5..] {
                    let (key, value) = field.split_once('=').ok_or_else(|| {
                        format!("第 {line_no} 行：选项 `{field}` 不是 key=value 形式")
                    })?;
                    apply_option(&mut board, id, key, value)
                        .map_err(|error| format!("第 {line_no} 行：{error}"))?;
                }
                ids.push((saved, id));
            }
            "wire" => {
                if fields.len() < 3 {
                    return Err(format!("第 {line_no} 行：`wire` 需要两个引脚"));
                }
                let a =
                    parse_pin(fields[1]).map_err(|error| format!("第 {line_no} 行：{error}"))?;
                let b =
                    parse_pin(fields[2]).map_err(|error| format!("第 {line_no} 行：{error}"))?;
                let mut bends = Vec::new();
                for field in &fields[3..] {
                    let (x, y) = field
                        .split_once(',')
                        .ok_or_else(|| format!("第 {line_no} 行：拐点 `{field}` 不是 x,y 形式"))?;
                    bends.push(Pos2::new(
                        parse_number(x, line_no, "拐点 x")?,
                        parse_number(y, line_no, "拐点 y")?,
                    ));
                }
                wires.push((line_no, a, b, bends));
            }
            other => return Err(format!("第 {line_no} 行：未知条目 `{other}`")),
        }
    }
    if !magic {
        return Err(format!("文件是空的（缺少 `{MAGIC}`）"));
    }

    for (line_no, a, b, bends) in wires {
        let a =
            resolve_pin(&board, &ids, a).map_err(|error| format!("第 {line_no} 行：{error}"))?;
        let b =
            resolve_pin(&board, &ids, b).map_err(|error| format!("第 {line_no} 行：{error}"))?;
        if !board.connect(a, b) {
            return Err(format!("第 {line_no} 行：连线重复，或把引脚连到了自己"));
        }
        if !bends.is_empty() {
            let index = board.wires().len() - 1;
            let Some(wire) = board.wire_mut(index) else {
                return Err(format!("第 {line_no} 行：连线没有建立"));
            };
            wire.bends = bends;
        }
    }
    Ok(board)
}

/// Writes the schematic to `path`.
pub fn save(path: impl AsRef<Path>, board: &Board) -> std::io::Result<()> {
    fs::write(path, to_text(board))
}

/// Reads a schematic from `path`; the error is ready to show in the UI.
pub fn open(path: impl AsRef<Path>) -> Result<Board, String> {
    let path = path.as_ref();
    let text = fs::read_to_string(path)
        .map_err(|error| format!("无法读取 {}：{error}", path.display()))?;
    from_text(&text).map_err(|error| format!("{}：{error}", path.display()))
}

/// Applies one `key=value` option to a freshly placed device.
fn apply_option(board: &mut Board, id: DeviceId, key: &str, value: &str) -> Result<(), String> {
    let Some(device) = board.device_mut(id) else {
        return Err("器件不存在".to_string());
    };
    match (key, &mut device.hardware) {
        ("ohms", Hardware::Resistor(resistor)) => {
            let ohms: f64 = value
                .parse()
                .map_err(|_| format!("电阻值 `{value}` 不是数字"))?;
            resistor.set_ohms(ohms);
        }
        ("unit", Hardware::Uart(uart)) => {
            let unit: u8 = value
                .parse()
                .map_err(|_| format!("串口号 `{value}` 不是整数"))?;
            if unit > 2 {
                return Err(format!("串口号 `{value}` 超出 0..2"));
            }
            uart.set_unit(unit);
        }
        ("mode", Hardware::Can(transceiver)) => {
            let mode =
                mode_from_token(value).ok_or_else(|| format!("收发器模式 `{value}` 未知"))?;
            transceiver.borrow_mut().set_mode(mode);
        }
        ("window", Hardware::Scope(scope)) => {
            let ns: u64 = value
                .parse()
                .map_err(|_| format!("示波器时基 `{value}` 不是整数"))?;
            scope.borrow_mut().set_window_ns(ns);
        }
        ("frozen", Hardware::Scope(scope)) => {
            let frozen = match value {
                "0" => false,
                "1" => true,
                other => return Err(format!("示波器冻结标志 `{other}` 应是 0 或 1")),
            };
            scope.borrow_mut().set_frozen(frozen);
        }
        _ => return Err(format!("该器件不支持选项 `{key}`")),
    }
    Ok(())
}

/// One end of a wire as it appears in the file, before ids are remapped.
#[derive(Clone, Copy, Debug)]
enum PinToken {
    Mcu { port: u8, pin: u8 },
    Device { device: u32, pin: u8 },
}

fn parse_pin(token: &str) -> Result<PinToken, String> {
    let fields: Vec<&str> = token.split(':').collect();
    let bad = || format!("引脚 `{token}` 应写成 mcu:端口:引脚 或 dev:编号:引脚");
    match fields.as_slice() {
        ["mcu", port, pin] => Ok(PinToken::Mcu {
            port: parse_index(port).ok_or_else(bad)?,
            pin: parse_index(pin).ok_or_else(bad)?,
        }),
        ["dev", device, pin] => Ok(PinToken::Device {
            device: device.parse().map_err(|_| bad())?,
            pin: parse_index(pin).ok_or_else(bad)?,
        }),
        _ => Err(bad()),
    }
}

fn resolve_pin(board: &Board, ids: &[(u32, DeviceId)], token: PinToken) -> Result<PinRef, String> {
    match token {
        PinToken::Mcu { port, pin } => {
            if usize::from(port) >= gsim_stm32f1::PORT_NAMES.len() || pin > 15 {
                return Err(format!("MCU 引脚 P{port}{pin} 不存在"));
            }
            Ok(PinRef::Mcu { port, pin })
        }
        PinToken::Device { device, pin } => {
            let id = ids
                .iter()
                .find(|(saved, _)| *saved == device)
                .map(|(_, id)| *id)
                .ok_or_else(|| format!("连线引用了不存在的器件 {device}"))?;
            let count = board
                .device(id)
                .map(|device| device.kind.pins().len())
                .unwrap_or(0);
            if usize::from(pin) >= count {
                return Err(format!("器件 {device} 没有 {pin} 号引脚"));
            }
            Ok(PinRef::Device { device: id, pin })
        }
    }
}

fn pin_token(pin: PinRef) -> String {
    match pin {
        PinRef::Mcu { port, pin } => format!("mcu:{port}:{pin}"),
        PinRef::Device { device, pin } => format!("dev:{device}:{pin}"),
    }
}

fn kind_token(kind: DeviceKind) -> &'static str {
    match kind {
        DeviceKind::Led => "led",
        DeviceKind::Uart => "uart",
        DeviceKind::Encoder => "encoder",
        DeviceKind::Vcc => "vcc",
        DeviceKind::Gnd => "gnd",
        DeviceKind::Can => "can",
        DeviceKind::Matrix => "matrix",
        DeviceKind::HseCrystal => "hse",
        DeviceKind::LseCrystal => "lse",
        DeviceKind::Resistor => "resistor",
        DeviceKind::Scope => "scope",
        DeviceKind::Junction => "junction",
    }
}

fn kind_from_token(token: &str) -> Option<DeviceKind> {
    DeviceKind::ALL
        .iter()
        .copied()
        .find(|kind| kind_token(*kind) == token)
}

fn mode_token(mode: TransceiverMode) -> &'static str {
    match mode {
        TransceiverMode::Normal => "normal",
        TransceiverMode::Silent => "silent",
        TransceiverMode::Standby => "standby",
    }
}

fn mode_from_token(token: &str) -> Option<TransceiverMode> {
    match token {
        "normal" => Some(TransceiverMode::Normal),
        "silent" => Some(TransceiverMode::Silent),
        "standby" => Some(TransceiverMode::Standby),
        _ => None,
    }
}

/// `600.00` style coordinates: fixed decimals keep saved files diffable.
fn number(value: f32) -> String {
    format!("{value:.2}")
}

fn parse_index(text: &str) -> Option<u8> {
    text.parse().ok()
}

fn parse_number(text: &str, line_no: usize, what: &str) -> Result<f32, String> {
    text.parse()
        .map_err(|_| format!("第 {line_no} 行：{what} `{text}` 不是数字"))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::board::{RESISTOR_A, RESISTOR_B};

    fn round_trip(board: &Board) -> Board {
        let text = to_text(board);
        from_text(&text).expect("the saved text must parse back")
    }

    #[test]
    fn the_saved_text_starts_with_the_format_header() {
        let text = to_text(&Board::demo());
        assert!(text.starts_with(MAGIC));
    }

    #[test]
    fn a_demo_board_round_trips_through_the_text_format() {
        let board = Board::demo();
        let text = to_text(&board);
        // Re-saving the loaded board must produce the same bytes: ids, order,
        // positions and every wire (with its corners) survive the trip.
        assert_eq!(to_text(&round_trip(&board)), text);
    }

    #[test]
    fn device_options_survive_the_round_trip() {
        let mut board = Board::new();
        let resistor = board.add_resistor(Pos2::new(120.0, 80.5));
        board
            .device_mut(resistor)
            .and_then(|device| device.hardware.resistor_mut())
            .expect("a resistor")
            .set_ohms(60.0);
        board.connect(
            PinRef::Device {
                device: resistor,
                pin: RESISTOR_A,
            },
            PinRef::Device {
                device: resistor,
                pin: RESISTOR_B,
            },
        );
        let text = to_text(&board);
        assert!(text.contains("ohms=60"), "saved text: {text}");

        let loaded = round_trip(&board);
        let device = loaded.devices().first().expect("one device");
        assert_eq!(
            device.hardware.resistor().map(|resistor| resistor.ohms()),
            Some(60.0)
        );
    }

    #[test]
    fn wire_corners_survive_the_round_trip() {
        let mut board = Board::new();
        let led = board.add_led(Pos2::new(300.0, 120.0));
        board.connect(
            PinRef::Mcu { port: 0, pin: 5 },
            PinRef::Device {
                device: led,
                pin: 0,
            },
        );
        board.insert_bend(0, 0, Pos2::new(250.0, 160.0));
        board.insert_bend(0, 1, Pos2::new(320.0, 160.0));

        let loaded = round_trip(&board);
        let wire = loaded.wires().first().expect("one wire");
        assert_eq!(
            wire.bends,
            vec![Pos2::new(250.0, 160.0), Pos2::new(320.0, 160.0)]
        );
    }

    #[test]
    fn wire_junctions_survive_the_round_trip() {
        let mut board = Board::new();
        let led = board.add_led(Pos2::new(300.0, 120.0));
        board.connect(
            PinRef::Mcu { port: 0, pin: 5 },
            PinRef::Device {
                device: led,
                pin: 0,
            },
        );
        let junction = board
            .add_junction(0, 0, Pos2::new(250.0, 160.0))
            .expect("the wire exists");

        let text = to_text(&board);
        assert!(text.contains("junction"), "saved text: {text}");
        let loaded = round_trip(&board);
        let tap = loaded
            .first_of(DeviceKind::Junction)
            .expect("the junction is saved like any other part");
        assert_eq!(tap.id, junction);
        assert_eq!(
            loaded.wires().len(),
            board.wires().len(),
            "the tapped wire stays split in two"
        );
        assert!(loaded.same_net(
            PinRef::Mcu { port: 0, pin: 5 },
            PinRef::Device {
                device: led,
                pin: 0
            }
        ));
    }

    #[test]
    fn a_project_can_be_written_and_read_back_from_disk() {
        let board = Board::demo();
        let path = std::env::temp_dir().join(format!(
            "gsimulator-project-{}-{}.gsim",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::SystemTime::UNIX_EPOCH)
                .map(|elapsed| elapsed.subsec_nanos())
                .unwrap_or(0)
        ));
        save(&path, &board).expect("the file is writable");
        let loaded = open(&path).expect("the file reads back");
        let _ = std::fs::remove_file(&path);
        assert_eq!(to_text(&loaded), to_text(&board));
    }

    /// The example project shipped in `docs/` must stay identical to
    /// [`Board::demo`], so opening it gives exactly the documented wiring.
    #[test]
    fn the_shipped_example_project_matches_the_demo_board() {
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../docs/example.gsim");
        let text = to_text(&Board::demo());
        if std::env::var_os("GSIM_DUMP_EXAMPLE").is_some() {
            std::fs::write(&path, &text).expect("the example project is writable");
        }
        let shipped = std::fs::read_to_string(&path).expect("docs/example.gsim is missing");
        assert_eq!(shipped, text, "regenerate docs/example.gsim");
    }

    #[test]
    fn a_file_without_the_header_is_rejected() {
        let error = from_text("device 1 led 0 0").expect_err("no header");
        assert!(error.contains(MAGIC), "unexpected message: {error}");
    }

    #[test]
    fn an_unknown_kind_is_reported_with_its_line() {
        let error = from_text(&format!("{MAGIC}\ndevice 1 toaster 10 10\n")).expect_err("kind");
        assert!(
            error.contains("未知器件类型"),
            "unexpected message: {error}"
        );
        assert!(error.starts_with("第 2 行"), "unexpected message: {error}");
    }

    #[test]
    fn a_wire_to_a_missing_device_is_rejected() {
        let error = from_text(&format!("{MAGIC}\nwire mcu:0:5 dev:9:0\n")).expect_err("device");
        assert!(
            error.contains("不存在的器件 9"),
            "unexpected message: {error}"
        );
    }

    #[test]
    fn a_wire_to_a_pin_that_does_not_exist_is_rejected() {
        let error = from_text(&format!(
            "{MAGIC}\ndevice 1 vcc 10 10\nwire mcu:0:5 dev:1:4\n"
        ))
        .expect_err("pin");
        assert!(
            error.contains("没有 4 号引脚"),
            "unexpected message: {error}"
        );
    }

    #[test]
    fn comments_and_blank_lines_are_ignored() {
        let text = format!("# a comment\n\n{MAGIC}\n\n# another\ndevice 1 led 5 6\n");
        let board = from_text(&text).expect("comments are fine");
        assert_eq!(board.devices().len(), 1);
        assert_eq!(board.devices()[0].pos, Pos2::new(5.0, 6.0));
    }
}
