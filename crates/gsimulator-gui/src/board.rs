//! Schematic board: MCU pins, power rails and virtual peripherals wired
//! together the way an EDA tool would do it.
//!
//! The model is deliberately independent of the simulation: it only consumes a
//! [`PinLevels`] snapshot and produces a [`BoardState`] (which LED is lit, which
//! nets are shorted, ...). That keeps the electrical rules unit-testable
//! without opening a window, and keeps the simulator free of any external
//! device code - the board lives entirely in the front end.

use egui::{Pos2, Vec2};
use std::cell::RefCell;
use std::collections::{BTreeMap, HashMap};
use std::rc::Rc;

/// Identifier of a device placed on the board.
pub type DeviceId = u32;

/// Which side of a device symbol a pin is drawn on.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Side {
    Left,
    Right,
    Top,
    Bottom,
}

/// Electrical role of a device pin.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum PinRole {
    /// Wants a positive voltage (LED anode, `+`, ...).
    Positive,
    /// Wants the return path (LED cathode, `-`, ...).
    Negative,
    /// Data arriving into the device (UART `RX`).
    Rx,
    /// Data leaving the device (UART `TX`).
    Tx,
    /// One of the quadrature contacts of an encoder.
    Phase,
    /// Momentary switch contact (encoder shaft).
    Button,
    /// Supplies a logic high (the `VCC` power symbol).
    PowerHigh,
    /// Supplies a logic low (the `GND` power symbol).
    PowerLow,
    /// Clock of a synchronous bus (`SPI` `CLK`).
    Clock,
    /// Active low chip select of a synchronous bus (`SPI` `CS`).
    Select,
    /// One end of a passive part (the resistor).
    Terminal,
    /// Contact of the differential CAN pair (`CANH`/`CANL`).
    Bus,
    /// Supply contact of a part (the `VCC` pin of a chip): wants the high rail.
    Vcc,
    /// Ground return contact of a part: wants the low rail.
    Ground,
}

/// One pin of a device symbol.
#[derive(Clone, Copy, Debug)]
pub struct PinDef {
    pub name: &'static str,
    pub role: PinRole,
    pub side: Side,
}

/// A connectable point on the schematic.
#[derive(Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord, Debug)]
pub enum PinRef {
    Mcu { port: u8, pin: u8 },
    Device { device: DeviceId, pin: u8 },
}

impl PinRef {
    /// Short human readable name, used in labels and messages.
    pub fn label(&self, board: &Board) -> String {
        match self {
            PinRef::Mcu { port, pin } => {
                format!(
                    "P{}{}",
                    gsim_stm32f1::PORT_NAMES
                        .get(*port as usize)
                        .copied()
                        .unwrap_or("?"),
                    pin
                )
            }
            PinRef::Device { device, pin } => match board.device(*device) {
                Some(device) => format!("{}.{}", device.name, device.pin_name(*pin)),
                None => format!("device{device}.{pin}"),
            },
        }
    }
}

/// Highlight colour of a device symbol (the LED uses it as its light colour).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Accent {
    pub name: &'static str,
    pub rgb: [u8; 3],
}

impl Accent {
    pub const ALL: [Accent; 4] = [
        Accent {
            name: "红",
            rgb: [230, 60, 50],
        },
        Accent {
            name: "绿",
            rgb: [60, 210, 90],
        },
        Accent {
            name: "蓝",
            rgb: [70, 140, 255],
        },
        Accent {
            name: "黄",
            rgb: [245, 200, 60],
        },
    ];

    pub fn color32(self) -> egui::Color32 {
        egui::Color32::from_rgb(self.rgb[0], self.rgb[1], self.rgb[2])
    }
}

/// The kinds of virtual peripherals the board can hold.
///
/// Adding a device means adding a variant here, a pin table below, a symbol in
/// `canvas.rs` and its electrical rule in [`Board::evaluate`].
#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
pub enum DeviceKind {
    Led,
    Uart,
    Encoder,
    /// `VCC` power symbol: every net it touches is pulled high.
    Vcc,
    /// `GND` power symbol: every net it touches is pulled low.
    Gnd,
    /// CAN transceiver between the controller's `TXD`/`RXD` and the virtual bus.
    Can,
    /// MAX7219 8x8 LED matrix chain on an SPI port.
    Matrix,
    /// High-speed crystal on `OSC_IN`/`OSC_OUT` (`PD0`/`PD1`).
    HseCrystal,
    /// 32.768 kHz watch crystal on `OSC32_IN`/`OSC32_OUT` (`PC14`/`PC15`).
    LseCrystal,
    /// Plain resistor, used to build the CAN termination network.
    Resistor,
    /// Oscilloscope probe on the two wires of the CAN pair.
    Scope,
    /// Gear motor with a CAN driver: it follows the speed command frame.
    Motor,
    /// Tap point on a wire (also usable as a free standing terminal).
    ///
    /// Double clicking a wire splices one in: the wire is split into two halves
    /// that both end on the junction's only pin, so anything wired to that pin
    /// joins the same net.
    Junction,
}

/// Pin index of the terminal's receive contact (data from the MCU).
pub const UART_RX: u8 = 0;
/// Pin index of the terminal's transmit contact (data into the MCU).
pub const UART_TX: u8 = 1;
/// Pin index of the terminal's `VCC` contact.
pub const UART_VCC: u8 = 2;
/// Pin index of the terminal's `GND` contact.
pub const UART_GND: u8 = 3;
/// Pin index of the encoder contact `A`.
pub const ENCODER_A: u8 = 0;
/// Pin index of the encoder contact `B`.
pub const ENCODER_B: u8 = 1;
/// Pin index of the encoder push button.
pub const ENCODER_SW: u8 = 2;
/// Pin index of the encoder's `VCC` contact.
pub const ENCODER_VCC: u8 = 3;
/// Pin index of the encoder's `GND` contact.
pub const ENCODER_GND: u8 = 4;
/// Pin index of the only pin of a power symbol.
pub const POWER_PIN: u8 = 0;
/// Pin index of the transceiver's controller transmit input (`CANx_TX`).
pub const CAN_TXD: u8 = 0;
/// Pin index of the transceiver's controller receive output (`CANx_RX`).
pub const CAN_RXD: u8 = 1;
/// Pin index of the transceiver's dominant-low wire (`CANH`).
pub const CAN_CANH: u8 = 2;
/// Pin index of the transceiver's dominant-high wire (`CANL`).
pub const CAN_CANL: u8 = 3;
/// Pin index of the transceiver's `VCC` contact.
pub const CAN_VCC: u8 = 4;
/// Pin index of the transceiver's `GND` contact.
pub const CAN_GND: u8 = 5;
/// Pin index of the first resistor terminal.
pub const RESISTOR_A: u8 = 0;
/// Pin index of the second resistor terminal.
pub const RESISTOR_B: u8 = 1;
/// Pin index of the probe on `CANH`.
pub const SCOPE_CANH: u8 = 0;
/// Pin index of the probe on `CANL`.
pub const SCOPE_CANL: u8 = 1;
/// Pin index of the motor's `CANH` contact.
pub const MOTOR_CANH: u8 = 0;
/// Pin index of the motor's `CANL` contact.
pub const MOTOR_CANL: u8 = 1;
/// Pin index of the motor's `VCC` contact.
pub const MOTOR_VCC: u8 = 2;
/// Pin index of the motor's `GND` contact.
pub const MOTOR_GND: u8 = 3;
/// Pin index of the matrix data input (`MOSI`).
pub const MATRIX_DIN: u8 = 0;
/// Pin index of the matrix shift clock (`SCK`).
pub const MATRIX_CLK: u8 = 1;
/// Pin index of the matrix latch (`CS`/`LOAD`).
pub const MATRIX_CS: u8 = 2;
/// Pin index of the matrix data output (`MISO`).
pub const MATRIX_DOUT: u8 = 3;
/// Pin index of the matrix module's `VCC` contact.
pub const MATRIX_VCC: u8 = 4;
/// Pin index of the matrix module's `GND` contact.
pub const MATRIX_GND: u8 = 5;
/// Pin index of the only pin of a junction (the tap point itself).
pub const JUNCTION_PIN: u8 = 0;
/// Pin index of the first crystal terminal.
pub const CRYSTAL_X1: u8 = 0;
/// Pin index of the second crystal terminal.
pub const CRYSTAL_X2: u8 = 1;

/// MCU pins of the HSE oscillator (`OSC_IN`, `OSC_OUT`).
pub const HSE_PINS: [(u8, u8); 2] = [(3, 0), (3, 1)];
/// MCU pins of the LSE oscillator (`OSC32_IN`, `OSC32_OUT`).
pub const LSE_PINS: [(u8, u8); 2] = [(2, 14), (2, 15)];

pub const LED_PINS: [PinDef; 2] = [
    PinDef {
        name: "A",
        role: PinRole::Positive,
        side: Side::Left,
    },
    PinDef {
        name: "K",
        role: PinRole::Negative,
        side: Side::Right,
    },
];

pub const UART_PINS: [PinDef; 4] = [
    PinDef {
        name: "RX",
        role: PinRole::Rx,
        side: Side::Left,
    },
    PinDef {
        name: "TX",
        role: PinRole::Tx,
        side: Side::Left,
    },
    PinDef {
        name: "VCC",
        role: PinRole::Vcc,
        side: Side::Top,
    },
    PinDef {
        name: "GND",
        role: PinRole::Ground,
        side: Side::Top,
    },
];

pub const ENCODER_PINS: [PinDef; 5] = [
    PinDef {
        name: "A",
        role: PinRole::Phase,
        side: Side::Left,
    },
    PinDef {
        name: "B",
        role: PinRole::Phase,
        side: Side::Left,
    },
    PinDef {
        name: "SW",
        role: PinRole::Button,
        side: Side::Left,
    },
    PinDef {
        name: "VCC",
        role: PinRole::Vcc,
        side: Side::Top,
    },
    PinDef {
        name: "GND",
        role: PinRole::Ground,
        side: Side::Top,
    },
];

pub const VCC_PINS: [PinDef; 1] = [PinDef {
    name: "+",
    role: PinRole::PowerHigh,
    side: Side::Right,
}];

pub const GND_PINS: [PinDef; 1] = [PinDef {
    name: "-",
    role: PinRole::PowerLow,
    side: Side::Right,
}];

pub const CAN_PINS: [PinDef; 6] = [
    PinDef {
        name: "TXD",
        role: PinRole::Rx,
        side: Side::Left,
    },
    PinDef {
        name: "RXD",
        role: PinRole::Tx,
        side: Side::Left,
    },
    // The two wire contacts sit on the bus side of the part.
    PinDef {
        name: "CANH",
        role: PinRole::Bus,
        side: Side::Right,
    },
    PinDef {
        name: "CANL",
        role: PinRole::Bus,
        side: Side::Right,
    },
    PinDef {
        name: "VCC",
        role: PinRole::Vcc,
        side: Side::Top,
    },
    PinDef {
        name: "GND",
        role: PinRole::Ground,
        side: Side::Top,
    },
];

pub const RESISTOR_PINS: [PinDef; 2] = [
    PinDef {
        name: "1",
        role: PinRole::Terminal,
        side: Side::Left,
    },
    PinDef {
        name: "2",
        role: PinRole::Terminal,
        side: Side::Right,
    },
];

pub const SCOPE_PINS: [PinDef; 2] = [
    PinDef {
        name: "H",
        role: PinRole::Bus,
        side: Side::Left,
    },
    PinDef {
        name: "L",
        role: PinRole::Bus,
        side: Side::Left,
    },
];

pub const MOTOR_PINS: [PinDef; 4] = [
    PinDef {
        name: "CANH",
        role: PinRole::Bus,
        side: Side::Left,
    },
    PinDef {
        name: "CANL",
        role: PinRole::Bus,
        side: Side::Left,
    },
    PinDef {
        name: "VCC",
        role: PinRole::Vcc,
        side: Side::Top,
    },
    PinDef {
        name: "GND",
        role: PinRole::Ground,
        side: Side::Top,
    },
];

/// The junction has a single contact, the tap point itself; it is drawn at the
/// centre of the symbol rather than on a stub.
pub const JUNCTION_PINS: [PinDef; 1] = [PinDef {
    name: "T",
    role: PinRole::Terminal,
    side: Side::Right,
}];

pub const CRYSTAL_PINS: [PinDef; 2] = [
    PinDef {
        name: "X1",
        role: PinRole::Clock,
        side: Side::Left,
    },
    PinDef {
        name: "X2",
        role: PinRole::Clock,
        side: Side::Right,
    },
];

pub const MATRIX_PINS: [PinDef; 6] = [
    PinDef {
        name: "DIN",
        role: PinRole::Rx,
        side: Side::Left,
    },
    PinDef {
        name: "CLK",
        role: PinRole::Clock,
        side: Side::Left,
    },
    PinDef {
        name: "CS",
        role: PinRole::Select,
        side: Side::Left,
    },
    PinDef {
        name: "DOUT",
        role: PinRole::Tx,
        side: Side::Right,
    },
    PinDef {
        name: "VCC",
        role: PinRole::Vcc,
        side: Side::Top,
    },
    PinDef {
        name: "GND",
        role: PinRole::Ground,
        side: Side::Top,
    },
];

impl DeviceKind {
    pub const ALL: [DeviceKind; 13] = [
        DeviceKind::Led,
        DeviceKind::Uart,
        DeviceKind::Encoder,
        DeviceKind::Vcc,
        DeviceKind::Gnd,
        DeviceKind::Can,
        DeviceKind::Matrix,
        DeviceKind::HseCrystal,
        DeviceKind::LseCrystal,
        DeviceKind::Resistor,
        DeviceKind::Scope,
        DeviceKind::Motor,
        DeviceKind::Junction,
    ];

    pub fn pins(self) -> &'static [PinDef] {
        match self {
            DeviceKind::Led => &LED_PINS,
            DeviceKind::Uart => &UART_PINS,
            DeviceKind::Encoder => &ENCODER_PINS,
            DeviceKind::Vcc => &VCC_PINS,
            DeviceKind::Gnd => &GND_PINS,
            DeviceKind::Can => &CAN_PINS,
            DeviceKind::Matrix => &MATRIX_PINS,
            DeviceKind::HseCrystal | DeviceKind::LseCrystal => &CRYSTAL_PINS,
            DeviceKind::Resistor => &RESISTOR_PINS,
            DeviceKind::Scope => &SCOPE_PINS,
            DeviceKind::Motor => &MOTOR_PINS,
            DeviceKind::Junction => &JUNCTION_PINS,
        }
    }

    pub fn body_size(self) -> Vec2 {
        match self {
            DeviceKind::Led => Vec2::new(92.0, 58.0),
            DeviceKind::Uart => Vec2::new(120.0, 74.0),
            DeviceKind::Encoder => Vec2::new(78.0, 82.0),
            DeviceKind::Vcc | DeviceKind::Gnd => Vec2::new(56.0, 46.0),
            DeviceKind::Can => Vec2::new(124.0, 78.0),
            DeviceKind::Matrix => Vec2::new(124.0, 100.0),
            DeviceKind::HseCrystal | DeviceKind::LseCrystal => Vec2::new(84.0, 56.0),
            DeviceKind::Resistor => Vec2::new(96.0, 44.0),
            DeviceKind::Scope => Vec2::new(132.0, 88.0),
            DeviceKind::Motor => Vec2::new(140.0, 96.0),
            DeviceKind::Junction => Vec2::splat(16.0),
        }
    }

    /// Name prefix used when a device of this kind is placed.
    pub fn prefix(self) -> &'static str {
        match self {
            DeviceKind::Led => "LED",
            DeviceKind::Uart => "UART",
            DeviceKind::Encoder => "ENC",
            DeviceKind::Vcc => "VCC",
            DeviceKind::Gnd => "GND",
            DeviceKind::Can => "CAN",
            DeviceKind::Matrix => "MX",
            DeviceKind::HseCrystal => "HSE",
            DeviceKind::LseCrystal => "LSE",
            DeviceKind::Resistor => "R",
            DeviceKind::Scope => "OSC",
            DeviceKind::Motor => "M",
            DeviceKind::Junction => "J",
        }
    }

    /// Label shown in the toolbar, the device list and the library.
    pub fn label(self) -> &'static str {
        match self {
            DeviceKind::Led => "LED",
            DeviceKind::Uart => "UART 终端",
            DeviceKind::Encoder => "旋转编码器",
            DeviceKind::Vcc => "VCC 电源",
            DeviceKind::Gnd => "GND 地",
            DeviceKind::Can => "CAN 收发器",
            DeviceKind::Matrix => "SPI 灯阵",
            DeviceKind::HseCrystal => "HSE 晶振",
            DeviceKind::LseCrystal => "LSE 晶振",
            DeviceKind::Resistor => "电阻",
            DeviceKind::Scope => "示波器",
            DeviceKind::Motor => "CAN 电机",
            DeviceKind::Junction => "接线节点",
        }
    }

    /// Group the part is filed under in the device library.
    pub fn category(self) -> &'static str {
        match self {
            DeviceKind::Led => "指示",
            DeviceKind::Uart => "通信",
            DeviceKind::Encoder => "交互",
            DeviceKind::Vcc | DeviceKind::Gnd => "电源",
            DeviceKind::Can => "通信",
            DeviceKind::Matrix => "显示",
            DeviceKind::HseCrystal | DeviceKind::LseCrystal => "时钟",
            DeviceKind::Resistor => "无源",
            DeviceKind::Scope => "仪器",
            DeviceKind::Motor => "执行",
            DeviceKind::Junction => "连接",
        }
    }

    /// One line explanation shown on the library card.
    pub fn summary(self) -> &'static str {
        match self {
            DeviceKind::Led => "按网络电平点亮的指示灯",
            DeviceKind::Uart => "收发串口字节的文本终端",
            DeviceKind::Encoder => "带按键的正交旋转编码器",
            DeviceKind::Vcc => "高电平电源符号，可放多个",
            DeviceKind::Gnd => "地符号，可放多个",
            DeviceKind::Can => "把 CAN1 接到虚拟总线",
            DeviceKind::Matrix => "MAX7219 驱动的 8×8 点阵",
            DeviceKind::HseCrystal => "8 MHz 高速晶振，接到 PD0/PD1",
            DeviceKind::LseCrystal => "32.768 kHz 晶振，接到 PC14/PC15",
            DeviceKind::Resistor => "跨接 CANH/CANL 的终端电阻",
            DeviceKind::Scope => "抓取 CANH/CANL 波形的示波器",
            DeviceKind::Motor => "听一条 CAN 指令转动的电机",
            DeviceKind::Junction => "双击导线插入的分支点，可当接线端子",
        }
    }

    /// Extra words the library search matches, so both `led` and `二极管`
    /// find the same part.
    pub fn keywords(self) -> &'static str {
        match self {
            DeviceKind::Led => "led 发光二极管 指示灯 灯",
            DeviceKind::Uart => "uart usart 串口 串行 终端 terminal 控制台",
            DeviceKind::Encoder => "encoder 编码器 旋钮 旋转 ec11",
            DeviceKind::Vcc => "vcc 电源 正极 高电平 供电",
            DeviceKind::Gnd => "gnd 地 负极 零伏 接地",
            DeviceKind::Can => "can canbus 总线 收发器 收发 tja1050",
            DeviceKind::Matrix => "max7219 灯阵 点阵 数码管 spi 显示 oled",
            DeviceKind::HseCrystal => "hse 晶振 crystal 8mhz 8m 晶体 振荡器 osc",
            DeviceKind::LseCrystal => "lse 晶振 crystal 32.768khz 32768 晶体 振荡器 rtc osc32",
            DeviceKind::Resistor => "resistor 电阻 终端电阻 120r 120欧 termination 无源",
            DeviceKind::Scope => "oscilloscope scope 示波器 波形 探头 抓波 仪器",
            DeviceKind::Motor => "motor 电机 马达 驱动 转速 伺服 执行器 舵机 can",
            DeviceKind::Junction => "junction 节点 接线 端子 连接点 分支 tap 引出",
        }
    }

    /// Stable slug used in widget ids.
    pub fn slug(self) -> &'static str {
        match self {
            DeviceKind::Led => "led",
            DeviceKind::Uart => "uart",
            DeviceKind::Encoder => "encoder",
            DeviceKind::Vcc => "vcc",
            DeviceKind::Gnd => "gnd",
            DeviceKind::Can => "can",
            DeviceKind::Matrix => "matrix",
            DeviceKind::HseCrystal => "hse-crystal",
            DeviceKind::LseCrystal => "lse-crystal",
            DeviceKind::Resistor => "resistor",
            DeviceKind::Scope => "scope",
            DeviceKind::Motor => "motor",
            DeviceKind::Junction => "junction",
        }
    }

    /// True for the two power symbols, which are drawn without a body box.
    pub fn is_power(self) -> bool {
        matches!(self, DeviceKind::Vcc | DeviceKind::Gnd)
    }

    /// Pin names of the symbol, e.g. `A K`.
    pub fn pin_summary(self) -> String {
        self.pins()
            .iter()
            .map(|def| def.name)
            .collect::<Vec<_>>()
            .join(" ")
    }

    /// True when `needle` (already lowercased) matches the library entry.
    pub fn matches(self, needle: &str) -> bool {
        [
            self.label(),
            self.category(),
            self.summary(),
            self.keywords(),
        ]
        .iter()
        .any(|text| text.to_lowercase().contains(needle))
    }

    /// Index of the pin with the given role, if the symbol has one.
    pub fn pin_with_role(self, role: PinRole) -> Option<u8> {
        self.pins()
            .iter()
            .position(|def| def.role == role)
            .map(|index| index as u8)
    }

    /// The `(VCC, GND)` contacts of the part, when its symbol has a supply.
    pub fn supply_pins(self) -> Option<(u8, u8)> {
        Some((
            self.pin_with_role(PinRole::Vcc)?,
            self.pin_with_role(PinRole::Ground)?,
        ))
    }

    /// Connection point of `pin` for a symbol of this kind whose body occupies
    /// `rect`, at the tip of its stub.
    ///
    /// Pins are spread evenly along the side they belong to, so a symbol with
    /// two contacts on the left (`encoder A`/`B`) does not stack them.
    pub fn pin_pos(self, rect: egui::Rect, pin: u8) -> Pos2 {
        if self == DeviceKind::Junction {
            // A tap point: the contact is the dot itself, without a stub.
            return rect.center();
        }
        let Some(def) = self.pins().get(pin as usize) else {
            return rect.center();
        };
        let (index, count) = self.pins_on_side(def.side, pin);
        let fraction = (index as f32 + 1.0) / (count as f32 + 1.0);
        match def.side {
            Side::Left => Pos2::new(rect.left() - STUB, rect.top() + rect.height() * fraction),
            Side::Right => Pos2::new(rect.right() + STUB, rect.top() + rect.height() * fraction),
            Side::Top => Pos2::new(rect.left() + rect.width() * fraction, rect.top() - STUB),
            Side::Bottom => Pos2::new(rect.left() + rect.width() * fraction, rect.bottom() + STUB),
        }
    }

    /// Position of `pin` within the pins of its side, and how many there are.
    fn pins_on_side(self, side: Side, pin: u8) -> (usize, usize) {
        let mut index = 0;
        let mut count = 0;
        for (position, def) in self.pins().iter().enumerate() {
            if def.side != side {
                continue;
            }
            if position as u8 == pin {
                index = count;
            }
            count += 1;
        }
        (index, count)
    }
}

/// External oscillators found on the board.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Crystals {
    /// Frequency of an HSE crystal wired across `PD0`/`PD1`.
    pub hse_hz: Option<u32>,
    /// True when an LSE crystal is wired across `PC14`/`PC15`.
    pub lse: bool,
}

/// Runtime state of a placed device.
///
/// The schematic itself stays a pure topology; the parts that need memory
/// (a terminal's log, an encoder's phase, a matrix's frame buffer) keep it
/// here. The two bus parts are shared with the simulation through `Rc`, which
/// is how the bridge lets them see the events of the slice being executed.
#[derive(Clone, Debug)]
pub enum Hardware {
    /// Stateless part (the LED and the power symbols are evaluated from the
    /// nets alone).
    None,
    Uart(gsim_devices::Uart),
    Encoder(gsim_devices::Encoder),
    Can(Rc<RefCell<gsim_devices::CanTransceiver>>),
    Matrix(Rc<RefCell<gsim_devices::Max7219>>),
    Crystal(gsim_devices::Crystal),
    /// Passive resistor: the value is read by the board rules, nothing drives.
    Resistor(gsim_devices::Resistor),
    /// Oscilloscope probe: the bridge pushes one sample per simulated bit.
    Scope(Rc<RefCell<gsim_devices::Oscilloscope>>),
    /// CAN controlled motor: the bridge hands it the frames its bus node
    /// decoded and the time to integrate its shaft over.
    Motor(Rc<RefCell<gsim_devices::Motor>>),
}

impl Hardware {
    pub fn crystal(&self) -> Option<&gsim_devices::Crystal> {
        match self {
            Hardware::Crystal(crystal) => Some(crystal),
            _ => None,
        }
    }

    pub fn uart(&self) -> Option<&gsim_devices::Uart> {
        match self {
            Hardware::Uart(uart) => Some(uart),
            _ => None,
        }
    }

    pub fn uart_mut(&mut self) -> Option<&mut gsim_devices::Uart> {
        match self {
            Hardware::Uart(uart) => Some(uart),
            _ => None,
        }
    }

    pub fn encoder(&self) -> Option<&gsim_devices::Encoder> {
        match self {
            Hardware::Encoder(encoder) => Some(encoder),
            _ => None,
        }
    }

    pub fn encoder_mut(&mut self) -> Option<&mut gsim_devices::Encoder> {
        match self {
            Hardware::Encoder(encoder) => Some(encoder),
            _ => None,
        }
    }

    pub fn can(&self) -> Option<&Rc<RefCell<gsim_devices::CanTransceiver>>> {
        match self {
            Hardware::Can(transceiver) => Some(transceiver),
            _ => None,
        }
    }

    pub fn matrix(&self) -> Option<&Rc<RefCell<gsim_devices::Max7219>>> {
        match self {
            Hardware::Matrix(matrix) => Some(matrix),
            _ => None,
        }
    }

    pub fn resistor(&self) -> Option<&gsim_devices::Resistor> {
        match self {
            Hardware::Resistor(resistor) => Some(resistor),
            _ => None,
        }
    }

    pub fn resistor_mut(&mut self) -> Option<&mut gsim_devices::Resistor> {
        match self {
            Hardware::Resistor(resistor) => Some(resistor),
            _ => None,
        }
    }

    pub fn scope(&self) -> Option<&Rc<RefCell<gsim_devices::Oscilloscope>>> {
        match self {
            Hardware::Scope(scope) => Some(scope),
            _ => None,
        }
    }

    pub fn motor(&self) -> Option<&Rc<RefCell<gsim_devices::Motor>>> {
        match self {
            Hardware::Motor(motor) => Some(motor),
            _ => None,
        }
    }

    pub fn motor_mut(&mut self) -> Option<&mut Rc<RefCell<gsim_devices::Motor>>> {
        match self {
            Hardware::Motor(motor) => Some(motor),
            _ => None,
        }
    }
}

/// A device instance placed on the board.
#[derive(Clone, Debug)]
pub struct Device {
    pub id: DeviceId,
    pub kind: DeviceKind,
    pub name: String,
    /// Top-left corner in board space.
    pub pos: Pos2,
    /// Highlight colour of the symbol; the LED uses it as its light colour.
    pub accent: Accent,
    pub hardware: Hardware,
}

impl Device {
    pub fn pin_def(&self, pin: u8) -> Option<&'static PinDef> {
        self.kind.pins().get(pin as usize)
    }

    pub fn pin_name(&self, pin: u8) -> &'static str {
        self.pin_def(pin).map(|def| def.name).unwrap_or("?")
    }

    pub fn rect(&self) -> egui::Rect {
        egui::Rect::from_min_size(self.pos, self.kind.body_size())
    }

    /// Connection point of `pin` in board space, at the tip of its stub.
    pub fn pin_pos(&self, pin: u8) -> Pos2 {
        self.kind.pin_pos(self.rect(), pin)
    }
}

/// Length of the pin stub that sticks out of a symbol, in board units.
pub const STUB: f32 = 22.0;

/// A point to point wire. Every wire joins exactly two pins.
///
/// `bends` are user placed waypoints in board space: the wire routes through
/// them in order, which is how a schematic is drawn the way an EDA tool does.
#[derive(Clone, Debug)]
pub struct Wire {
    pub a: PinRef,
    pub b: PinRef,
    pub bends: Vec<Pos2>,
}

/// The schematic: devices and the wires between them.
#[derive(Clone, Debug)]
pub struct Board {
    devices: Vec<Device>,
    wires: Vec<Wire>,
    next_id: DeviceId,
    /// Virtual CAN bus every transceiver on the board is attached to.
    can_bus: Rc<RefCell<gsim_devices::CanBus>>,
}

impl Default for Board {
    fn default() -> Self {
        Board::new()
    }
}

impl Board {
    pub fn new() -> Self {
        Board {
            devices: Vec::new(),
            wires: Vec::new(),
            next_id: 1,
            can_bus: Rc::new(RefCell::new(gsim_devices::CanBus::new())),
        }
    }

    /// A newly opened project: only the HSE crystal the chip needs, already
    /// wired across `OSC_IN`/`OSC_OUT`. Everything else is placed from the
    /// device library.
    pub fn new_project() -> Self {
        let mut board = Board::new();
        // Left of the chip, level with PD0/PD1 (OSC_IN/OSC_OUT).
        let hse = board.add_hse_crystal(Pos2::new(-64.0, 148.0));
        for (index, (port, pin)) in [(CRYSTAL_X1, HSE_PINS[0]), (CRYSTAL_X2, HSE_PINS[1])] {
            board.connect(
                PinRef::Device {
                    device: hse,
                    pin: index,
                },
                PinRef::Mcu { port, pin },
            );
        }
        board
    }

    /// Board shipped on first start: `VCC`/`GND` power symbols, an LED wired to
    /// PA5 and GND (so the built-in stimulus makes it blink right away), a UART
    /// terminal on USART1's pins, a rotary encoder on PA0/PA1/PA2 and both
    /// crystals (8 MHz HSE on `PD0`/`PD1`, 32.768 kHz LSE on `PC14`/`PC15`) so
    /// the firmware can clock itself however its RCC setup wants.
    pub fn demo() -> Self {
        let mut board = Board::new();

        let vcc = board.add_vcc(Pos2::new(430.0, 110.0));
        let gnd = board.add_gnd(Pos2::new(430.0, 200.0));

        let led = board.add_led(Pos2::new(600.0, 120.0));
        board.connect(
            PinRef::Mcu { port: 0, pin: 5 },
            PinRef::Device {
                device: led,
                pin: 0,
            },
        );
        board.connect(
            PinRef::Device {
                device: gnd,
                pin: POWER_PIN,
            },
            PinRef::Device {
                device: led,
                pin: 1,
            },
        );

        let uart = board.add_uart(Pos2::new(600.0, 300.0));
        // MCU TX (PA9) -> terminal RX, terminal TX -> MCU RX (PA10).
        board.connect(
            PinRef::Mcu { port: 0, pin: 9 },
            PinRef::Device {
                device: uart,
                pin: UART_RX,
            },
        );
        board.connect(
            PinRef::Device {
                device: uart,
                pin: UART_TX,
            },
            PinRef::Mcu { port: 0, pin: 10 },
        );

        let encoder = board.add_encoder(Pos2::new(600.0, 480.0));
        for (index, mcu_pin) in [(ENCODER_A, 0u8), (ENCODER_B, 1), (ENCODER_SW, 2)] {
            board.connect(
                PinRef::Device {
                    device: encoder,
                    pin: index,
                },
                PinRef::Mcu {
                    port: 0,
                    pin: mcu_pin,
                },
            );
        }

        // Both parts with a supply are fed from the two power symbols.
        for (device, vcc_pin, gnd_pin) in [
            (uart, UART_VCC, UART_GND),
            (encoder, ENCODER_VCC, ENCODER_GND),
        ] {
            board.connect(
                PinRef::Device {
                    device: vcc,
                    pin: POWER_PIN,
                },
                PinRef::Device {
                    device,
                    pin: vcc_pin,
                },
            );
            board.connect(
                PinRef::Device {
                    device: gnd,
                    pin: POWER_PIN,
                },
                PinRef::Device {
                    device,
                    pin: gnd_pin,
                },
            );
        }

        // 8 MHz HSE between OSC_IN and OSC_OUT, left of the chip like the
        // real part sits next to PD0/PD1.
        let hse = board.add_hse_crystal(Pos2::new(-64.0, 148.0));
        for (index, (port, pin)) in [(CRYSTAL_X1, HSE_PINS[0]), (CRYSTAL_X2, HSE_PINS[1])] {
            board.connect(
                PinRef::Device {
                    device: hse,
                    pin: index,
                },
                PinRef::Mcu { port, pin },
            );
        }

        // 32.768 kHz LSE between OSC32_IN and OSC32_OUT.
        let lse = board.add_lse_crystal(Pos2::new(300.0, 570.0));
        for (index, (port, pin)) in [(CRYSTAL_X1, LSE_PINS[0]), (CRYSTAL_X2, LSE_PINS[1])] {
            board.connect(
                PinRef::Device {
                    device: lse,
                    pin: index,
                },
                PinRef::Mcu { port, pin },
            );
        }

        board
    }

    pub fn devices(&self) -> &[Device] {
        &self.devices
    }

    pub fn wires(&self) -> &[Wire] {
        &self.wires
    }

    /// The virtual bus every CAN transceiver on the board talks to.
    pub fn can_bus(&self) -> Rc<RefCell<gsim_devices::CanBus>> {
        Rc::clone(&self.can_bus)
    }

    /// Handles of every CAN transceiver placed on the board, in placement order.
    pub fn transceivers(&self) -> Vec<Rc<RefCell<gsim_devices::CanTransceiver>>> {
        self.devices
            .iter()
            .filter_map(|device| device.hardware.can().cloned())
            .collect()
    }

    /// Handles of every LED matrix placed on the board, in placement order.
    pub fn matrices(&self) -> Vec<Rc<RefCell<gsim_devices::Max7219>>> {
        self.devices
            .iter()
            .filter_map(|device| device.hardware.matrix().cloned())
            .collect()
    }

    pub fn device(&self, id: DeviceId) -> Option<&Device> {
        self.devices.iter().find(|device| device.id == id)
    }

    pub fn device_mut(&mut self, id: DeviceId) -> Option<&mut Device> {
        self.devices.iter_mut().find(|device| device.id == id)
    }

    /// Placed device of `kind` with the lowest id, if any.
    pub fn first_of(&self, kind: DeviceKind) -> Option<&Device> {
        self.devices.iter().find(|device| device.kind == kind)
    }

    /// Places a new device of any kind and returns its id.
    pub fn add_device(&mut self, kind: DeviceKind, pos: Pos2) -> DeviceId {
        let id = self.next_id;
        self.next_id += 1;
        let index = self.devices.iter().filter(|d| d.kind == kind).count() + 1;
        let hardware = match kind {
            DeviceKind::Led | DeviceKind::Vcc | DeviceKind::Gnd | DeviceKind::Junction => {
                Hardware::None
            }
            DeviceKind::Uart => Hardware::Uart(gsim_devices::Uart::new(0)),
            DeviceKind::Encoder => Hardware::Encoder(gsim_devices::Encoder::new()),
            DeviceKind::Can => {
                Hardware::Can(Rc::new(RefCell::new(gsim_devices::CanTransceiver::attach(
                    Rc::clone(&self.can_bus),
                    &format!("CAN{index} 节点"),
                    0,
                ))))
            }
            DeviceKind::Matrix => Hardware::Matrix(Rc::new(RefCell::new(
                gsim_devices::Max7219::new(0, 0, 4, 1),
            ))),
            DeviceKind::HseCrystal => Hardware::Crystal(gsim_devices::Crystal::hse(8_000_000)),
            DeviceKind::LseCrystal => Hardware::Crystal(gsim_devices::Crystal::lse()),
            DeviceKind::Resistor => Hardware::Resistor(gsim_devices::Resistor::default()),
            DeviceKind::Scope => Hardware::Scope(Rc::new(RefCell::new(
                gsim_devices::Oscilloscope::new(&format!("OSC{index}")),
            ))),
            // The motor is a node of its own on the pair: it hears the frames
            // bit by bit and acknowledges them like any other part.
            DeviceKind::Motor => Hardware::Motor(Rc::new(RefCell::new(
                gsim_devices::Motor::attach(Rc::clone(&self.can_bus), &format!("M{index}")),
            ))),
        };
        self.devices.push(Device {
            id,
            kind,
            name: format!("{}{index}", kind.prefix()),
            pos,
            accent: Accent::ALL[(index - 1) % Accent::ALL.len()],
            hardware,
        });
        id
    }

    /// Places a new LED and returns its id.
    pub fn add_led(&mut self, pos: Pos2) -> DeviceId {
        self.add_device(DeviceKind::Led, pos)
    }

    /// Places a new UART terminal and returns its id.
    pub fn add_uart(&mut self, pos: Pos2) -> DeviceId {
        self.add_device(DeviceKind::Uart, pos)
    }

    /// Places a new rotary encoder and returns its id.
    pub fn add_encoder(&mut self, pos: Pos2) -> DeviceId {
        self.add_device(DeviceKind::Encoder, pos)
    }

    /// Places a new `VCC` symbol and returns its id.
    pub fn add_vcc(&mut self, pos: Pos2) -> DeviceId {
        self.add_device(DeviceKind::Vcc, pos)
    }

    /// Places a new `GND` symbol and returns its id.
    pub fn add_gnd(&mut self, pos: Pos2) -> DeviceId {
        self.add_device(DeviceKind::Gnd, pos)
    }

    /// Places a new CAN transceiver and returns its id.
    pub fn add_can(&mut self, pos: Pos2) -> DeviceId {
        self.add_device(DeviceKind::Can, pos)
    }

    /// Places a new LED matrix chain and returns its id.
    pub fn add_matrix(&mut self, pos: Pos2) -> DeviceId {
        self.add_device(DeviceKind::Matrix, pos)
    }

    /// Places an 8 MHz HSE crystal and returns its id.
    pub fn add_hse_crystal(&mut self, pos: Pos2) -> DeviceId {
        self.add_device(DeviceKind::HseCrystal, pos)
    }

    /// Places a 32.768 kHz LSE crystal and returns its id.
    pub fn add_lse_crystal(&mut self, pos: Pos2) -> DeviceId {
        self.add_device(DeviceKind::LseCrystal, pos)
    }

    /// Places a termination resistor and returns its id.
    pub fn add_resistor(&mut self, pos: Pos2) -> DeviceId {
        self.add_device(DeviceKind::Resistor, pos)
    }

    /// Places an oscilloscope probe and returns its id.
    pub fn add_scope(&mut self, pos: Pos2) -> DeviceId {
        self.add_device(DeviceKind::Scope, pos)
    }

    /// Handles of every oscilloscope placed on the board, in placement order.
    pub fn scopes(&self) -> Vec<Rc<RefCell<gsim_devices::Oscilloscope>>> {
        self.devices
            .iter()
            .filter_map(|device| device.hardware.scope().cloned())
            .collect()
    }

    /// Places a CAN controlled motor and returns its id.
    pub fn add_motor(&mut self, pos: Pos2) -> DeviceId {
        self.add_device(DeviceKind::Motor, pos)
    }

    /// Crystals wired across the oscillator pins, whatever the firmware later
    /// decides to clock itself (or the RTC) from.
    pub fn crystals(&self) -> Crystals {
        let mut found = Crystals::default();
        for device in &self.devices {
            let Some(crystal) = device.hardware.crystal() else {
                continue;
            };
            let [input, output] = match crystal.kind() {
                gsim_devices::CrystalKind::Hse => HSE_PINS,
                gsim_devices::CrystalKind::Lse => LSE_PINS,
            };
            let x1 = PinRef::Device {
                device: device.id,
                pin: CRYSTAL_X1,
            };
            let x2 = PinRef::Device {
                device: device.id,
                pin: CRYSTAL_X2,
            };
            let in_pin = PinRef::Mcu {
                port: input.0,
                pin: input.1,
            };
            let out_pin = PinRef::Mcu {
                port: output.0,
                pin: output.1,
            };
            let wired = (self.same_net(x1, in_pin) && self.same_net(x2, out_pin))
                || (self.same_net(x1, out_pin) && self.same_net(x2, in_pin));
            if !wired {
                continue;
            }
            match crystal.kind() {
                gsim_devices::CrystalKind::Hse => {
                    found.hse_hz.get_or_insert(crystal.hz());
                }
                gsim_devices::CrystalKind::Lse => found.lse = true,
            }
        }
        found
    }

    /// Mutable access to the placed devices.
    pub fn devices_mut(&mut self) -> &mut [Device] {
        &mut self.devices
    }

    /// The MCU pin on the same net as `pin`, if any.
    ///
    /// Junctions (and the corners they split a wire into) are transparent here,
    /// so tapping a wire does not stop the part from seeing the controller.
    pub fn mcu_pin_for(&self, pin: PinRef) -> Option<(u8, u8)> {
        self.net_pins(pin).into_iter().find_map(|node| match node {
            PinRef::Mcu { port, pin } => Some((port, pin)),
            _ => None,
        })
    }

    /// Every pin reachable from `pin` through wires, `pin` included.
    ///
    /// Wires are the only thing that joins two pins, so this is the net `pin`
    /// belongs to; a device never shorts its own contacts.
    fn net_pins(&self, pin: PinRef) -> Vec<PinRef> {
        let mut net = vec![pin];
        let mut next = 0;
        while next < net.len() {
            let node = net[next];
            next += 1;
            for wire in &self.wires {
                let other = if wire.a == node {
                    wire.b
                } else if wire.b == node {
                    wire.a
                } else {
                    continue;
                };
                if !net.contains(&other) {
                    net.push(other);
                }
            }
        }
        net
    }

    /// True when the two pins sit on the same net, junctions included.
    ///
    /// [`Board::is_connected`] only looks at a single wire, which is what the
    /// duplicate check of [`Board::connect`] needs; this one answers the
    /// electrical question the bridge asks.
    pub fn same_net(&self, a: PinRef, b: PinRef) -> bool {
        a == b || self.net_pins(a).contains(&b)
    }

    /// Removes a device together with every wire that touched it.
    ///
    /// Removing a junction joins the two wire halves it spliced back into one
    /// wire, so deleting a tap leaves the original connection intact.
    pub fn remove_device(&mut self, id: DeviceId) {
        if self
            .device(id)
            .is_some_and(|device| device.kind == DeviceKind::Junction)
        {
            self.remove_junction(id);
            return;
        }
        self.devices.retain(|device| device.id != id);
        self.wires.retain(|wire| {
            !matches!(wire.a, PinRef::Device { device, .. } if device == id)
                && !matches!(wire.b, PinRef::Device { device, .. } if device == id)
        });
    }

    /// Splices a junction into `wire` at `point` and returns its id.
    ///
    /// `leg` is the route leg the user double clicked (`0` = right after pin
    /// `a`), exactly like [`Board::insert_bend`]. The wire becomes two wires
    /// that both end on the junction's only pin, which is how the tap joins the
    /// net without breaking it.
    pub fn add_junction(&mut self, wire: usize, leg: usize, point: Pos2) -> Option<DeviceId> {
        let mut original = self.wires.get(wire)?.clone();
        if leg > original.bends.len() {
            return None;
        }
        let id = self.add_device(
            DeviceKind::Junction,
            point - DeviceKind::Junction.body_size() * 0.5,
        );
        let pin = PinRef::Device {
            device: id,
            pin: JUNCTION_PIN,
        };
        let after = original.bends.split_off(leg);
        self.wires[wire] = Wire {
            a: original.a,
            b: pin,
            bends: original.bends,
        };
        self.wires.push(Wire {
            a: pin,
            b: original.b,
            bends: after,
        });
        Some(id)
    }

    /// Dissolves a junction, fusing the two halves it spliced into one wire.
    ///
    /// Any extra wire the user ran to the tap keeps its end: when the junction
    /// carries more (or fewer) than two wires it is dropped like any other
    /// device, taking those wires with it.
    fn remove_junction(&mut self, id: DeviceId) {
        let pin = PinRef::Device {
            device: id,
            pin: JUNCTION_PIN,
        };
        // End of the wire away from the junction, with the corners listed in
        // the direction junction -> other.
        let mut halves: Vec<(usize, PinRef, Vec<Pos2>)> = Vec::new();
        for (index, wire) in self.wires.iter().enumerate() {
            if wire.a == pin {
                halves.push((index, wire.b, wire.bends.clone()));
            } else if wire.b == pin {
                let mut bends = wire.bends.clone();
                bends.reverse();
                halves.push((index, wire.a, bends));
            }
        }
        self.devices.retain(|device| device.id != id);
        let mut indices: Vec<usize> = halves.iter().map(|(index, _, _)| *index).collect();
        indices.sort_unstable();
        for index in indices.into_iter().rev() {
            self.wires.remove(index);
        }
        let [(_, from, bends), (_, to, after)] = halves.as_slice() else {
            return;
        };
        if from == to || self.is_connected(*from, *to) {
            return;
        }
        let mut merged: Vec<Pos2> = bends.iter().rev().copied().collect();
        merged.extend(after.iter().copied());
        self.wires.push(Wire {
            a: *from,
            b: *to,
            bends: merged,
        });
    }

    /// Connects two pins. Returns `false` when the wire would be a self loop or
    /// already exists.
    pub fn connect(&mut self, a: PinRef, b: PinRef) -> bool {
        if a == b || self.is_connected(a, b) {
            return false;
        }
        self.wires.push(Wire {
            a,
            b,
            bends: Vec::new(),
        });
        true
    }

    pub fn is_connected(&self, a: PinRef, b: PinRef) -> bool {
        self.wires
            .iter()
            .any(|wire| (wire.a == a && wire.b == b) || (wire.a == b && wire.b == a))
    }

    pub fn wire_mut(&mut self, index: usize) -> Option<&mut Wire> {
        self.wires.get_mut(index)
    }

    /// Inserts a waypoint at `position` of the wire (`0` = right after pin
    /// `a`, `bends.len()` = right before pin `b`).
    ///
    /// The canvas passes the leg the user grabbed, so the wire keeps the shape
    /// it had before the new corner was added.
    pub fn insert_bend(&mut self, index: usize, position: usize, point: Pos2) -> bool {
        let Some(wire) = self.wires.get_mut(index) else {
            return false;
        };
        if position > wire.bends.len() {
            return false;
        }
        wire.bends.insert(position, point);
        true
    }

    /// Moves one waypoint of a wire.
    pub fn move_bend(&mut self, index: usize, bend: usize, point: Pos2) -> bool {
        let Some(slot) = self
            .wires
            .get_mut(index)
            .and_then(|wire| wire.bends.get_mut(bend))
        else {
            return false;
        };
        *slot = point;
        true
    }

    /// Removes one waypoint of a wire.
    pub fn remove_bend(&mut self, index: usize, bend: usize) -> bool {
        let Some(wire) = self.wires.get_mut(index) else {
            return false;
        };
        if bend >= wire.bends.len() {
            return false;
        }
        wire.bends.remove(bend);
        true
    }

    pub fn remove_wire(&mut self, index: usize) -> Option<Wire> {
        (index < self.wires.len()).then(|| self.wires.remove(index))
    }

    /// Removes every wire, and with them the junctions that tapped one.
    pub fn clear_wires(&mut self) {
        self.wires.clear();
        self.devices
            .retain(|device| device.kind != DeviceKind::Junction);
    }

    /// How the CAN pair is wired: the two nets, the terminators bridging them
    /// and the probes tapping them.
    ///
    /// `powered` decides which transceivers count as being on the pair, so the
    /// bridge can leave a part that has no supply out of the circuit.
    pub fn can_network_with(&self, powered: impl Fn(DeviceId) -> bool) -> CanNetwork {
        let (_, net_of) = self.nets_and_index();
        let net_of_pin =
            |device: DeviceId, pin: u8| net_of.get(&PinRef::Device { device, pin }).copied();

        let mut network = CanNetwork::default();
        for device in &self.devices {
            if !powered(device.id) {
                continue;
            }
            // A transceiver or a motor defines the pair; everything else on it
            // (terminators, probes, more motors) is found from the topology.
            let (high, low, controller) = match device.kind {
                DeviceKind::Can => (CAN_CANH, CAN_CANL, true),
                DeviceKind::Motor => (MOTOR_CANH, MOTOR_CANL, false),
                _ => continue,
            };
            let (Some(canh), Some(canl)) =
                (net_of_pin(device.id, high), net_of_pin(device.id, low))
            else {
                continue;
            };
            if canh == canl {
                // Both wires on one net: a dead short, not a bus.
                continue;
            }
            if network.canh_net.is_none() {
                network.canh_net = Some(canh);
                network.canl_net = Some(canl);
            }
            if network.canh_net == Some(canh) && network.canl_net == Some(canl) && controller {
                network.nodes += 1;
            }
        }
        let (Some(canh), Some(canl)) = (network.canh_net, network.canl_net) else {
            return network;
        };

        for device in &self.devices {
            match &device.hardware {
                Hardware::Resistor(resistor) => {
                    let (a, b) = (
                        net_of_pin(device.id, RESISTOR_A),
                        net_of_pin(device.id, RESISTOR_B),
                    );
                    let bridges = matches!((a, b), (Some(a), Some(b))
                        if (a == canh && b == canl) || (a == canl && b == canh));
                    if bridges && resistor.ohms() > 0.0 {
                        network.terminators.push(resistor.ohms());
                        network.terminator_ids.push(device.id);
                    }
                }
                Hardware::Scope(scope) => {
                    if net_of_pin(device.id, SCOPE_CANH) == Some(canh)
                        && net_of_pin(device.id, SCOPE_CANL) == Some(canl)
                    {
                        network.scopes.push(Rc::clone(scope));
                    }
                }
                Hardware::Motor(motor) => {
                    if net_of_pin(device.id, MOTOR_CANH) == Some(canh)
                        && net_of_pin(device.id, MOTOR_CANL) == Some(canl)
                    {
                        network.motors.push((device.id, Rc::clone(motor)));
                    }
                }
                _ => {}
            }
        }
        if !network.terminators.is_empty() {
            let conductance: f64 = network.terminators.iter().map(|ohms| 1.0 / ohms).sum();
            if conductance > 0.0 {
                network.termination_ohm = Some(1.0 / conductance);
            }
        }
        network
    }

    /// How the CAN pair is wired, with every placed transceiver taking part.
    pub fn can_network(&self) -> CanNetwork {
        self.can_network_with(|_| true)
    }

    /// Pins grouped into electrically connected nets, including single pins.
    pub fn nets(&self) -> Vec<Vec<PinRef>> {
        let mut sets: Vec<Vec<PinRef>> = Vec::new();
        for wire in &self.wires {
            let a = sets.iter().position(|set| set.contains(&wire.a));
            let b = sets.iter().position(|set| set.contains(&wire.b));
            match (a, b) {
                (Some(a), Some(b)) if a == b => {}
                (Some(a), Some(b)) => {
                    let other = sets.remove(b.max(a));
                    let keep = sets.get_mut(b.min(a)).expect("index is valid");
                    keep.extend(other);
                }
                (Some(a), None) => sets[a].push(wire.b),
                (None, Some(b)) => sets[b].push(wire.a),
                (None, None) => sets.push(vec![wire.a, wire.b]),
            }
        }
        sets
    }

    /// Nets plus the net index of every pin on them.
    fn nets_and_index(&self) -> (Vec<Vec<PinRef>>, HashMap<PinRef, usize>) {
        let nets = self.nets();
        let mut net_of = HashMap::new();
        for (index, net) in nets.iter().enumerate() {
            for node in net {
                net_of.insert(*node, index);
            }
        }
        (nets, net_of)
    }

    /// Supply state of every part whose symbol has `VCC`/`GND` contacts.
    ///
    /// The bridge asks for this once per slice: a part whose supply is open or
    /// sits on the wrong rails is left out of the wiring, so it really stops
    /// working instead of quietly running without power.
    pub fn supplies(&self, levels: &PinLevels) -> BTreeMap<DeviceId, Supply> {
        let (nets, net_of) = self.nets_and_index();
        self.supplies_in(&nets, &net_of, levels)
    }

    fn supplies_in(
        &self,
        nets: &[Vec<PinRef>],
        net_of: &HashMap<PinRef, usize>,
        levels: &PinLevels,
    ) -> BTreeMap<DeviceId, Supply> {
        let mut supplies = BTreeMap::new();
        for device in &self.devices {
            let Some((vcc, gnd)) = device.kind.supply_pins() else {
                continue;
            };
            let level_of = |pin: u8| {
                net_of
                    .get(&PinRef::Device {
                        device: device.id,
                        pin,
                    })
                    .map(|index| self.net_level(&nets[*index], levels))
            };
            supplies.insert(device.id, Supply::of(level_of(vcc), level_of(gnd)));
        }
        supplies
    }

    /// Level of every net, plus the state of every device, for one snapshot of
    /// the simulated pin levels.
    pub fn evaluate(&self, levels: &PinLevels) -> BoardState {
        let (nets, net_of) = self.nets_and_index();

        let mut levels_of_nets = Vec::with_capacity(nets.len());
        let mut high_of_nets = Vec::with_capacity(nets.len());
        let mut shorts = Vec::new();
        for net in &nets {
            let level = self.net_level(net, levels);
            if level == NetLevel::Shorted {
                shorts.push(net.clone());
            }
            levels_of_nets.push(level);
            high_of_nets.push(self.net_high_fraction(net, levels));
        }

        let mut leds = BTreeMap::new();
        for device in &self.devices {
            if device.kind == DeviceKind::Led {
                let positive = net_of.get(&PinRef::Device {
                    device: device.id,
                    pin: device.kind.pin_with_role(PinRole::Positive).unwrap_or(0),
                });
                let negative = net_of.get(&PinRef::Device {
                    device: device.id,
                    pin: device.kind.pin_with_role(PinRole::Negative).unwrap_or(1),
                });
                // The LED conducts while its anode net sits above its cathode
                // net, so the duty cycles of a PWM driven pair decide how
                // bright it looks instead of a single sampled instant.
                let brightness = match (positive, negative) {
                    (Some(positive), Some(negative)) if positive != negative => {
                        let (anode, cathode) =
                            (levels_of_nets[*positive], levels_of_nets[*negative]);
                        if Self::open(anode) || Self::open(cathode) {
                            0.0
                        } else {
                            (high_of_nets[*positive] - high_of_nets[*negative]).clamp(0.0, 1.0)
                        }
                    }
                    _ => 0.0,
                };
                leds.insert(device.id, brightness);
            }
        }

        let supplies = self.supplies_in(&nets, &net_of, levels);

        BoardState {
            nets,
            levels_of_nets,
            shorts,
            leds,
            supplies,
        }
    }

    /// Effective level of one net.
    fn net_level(&self, net: &[PinRef], levels: &PinLevels) -> NetLevel {
        let mut high = false;
        let mut low = false;
        for node in net {
            match node {
                PinRef::Mcu { port, pin } => {
                    if levels.level[*port as usize][*pin as usize] {
                        high = true;
                    } else {
                        low = true;
                    }
                }
                PinRef::Device { device, .. } => match self.device(*device).map(|d| d.kind) {
                    Some(DeviceKind::Vcc) => high = true,
                    Some(DeviceKind::Gnd) => low = true,
                    // Signal parts do not drive anything in this phase.
                    _ => {}
                },
            }
        }
        match (high, low) {
            (true, true) => NetLevel::Shorted,
            (true, false) => NetLevel::High,
            (false, true) => NetLevel::Low,
            (false, false) => NetLevel::Floating,
        }
    }

    /// Fraction of the time a net is high, taken from its strongest driver.
    ///
    /// A pin carrying a PWM waveform contributes its measured duty, so a 25 %
    /// channel reads `0.25` instead of a random sampled level, and a pin that
    /// only listens to the net does not dilute it.
    fn net_high_fraction(&self, net: &[PinRef], levels: &PinLevels) -> f32 {
        let mut high: Option<f32> = None;
        for node in net {
            let duty = match node {
                PinRef::Mcu { port, pin } => levels.duty_of(*port, *pin),
                PinRef::Device { device, .. } => match self.device(*device).map(|d| d.kind) {
                    Some(DeviceKind::Vcc) => 1.0,
                    Some(DeviceKind::Gnd) => 0.0,
                    _ => continue,
                },
            };
            high = Some(high.map_or(duty, |current: f32| current.max(duty)));
        }
        high.unwrap_or(0.0)
    }

    /// A net that neither side drives carries no LED current.
    fn open(level: NetLevel) -> bool {
        matches!(level, NetLevel::Floating | NetLevel::Shorted)
    }
}

/// How the CAN pair is wired on this drawing.
///
/// The transceiver's `CANH`/`CANL` contacts form the two nets of the pair; the
/// terminators that bridge them and the probes that tap them are found from the
/// topology, which is what makes "add a 120 Ω resistor" change the electrical
/// behaviour of the simulated bus.
#[derive(Clone, Debug, Default)]
pub struct CanNetwork {
    /// Net index of `CANH`.
    pub canh_net: Option<usize>,
    /// Net index of `CANL`.
    pub canl_net: Option<usize>,
    /// Parallel resistance bridging the two wires, `None` when nothing does.
    pub termination_ohm: Option<f64>,
    /// Values of the resistors that terminate the pair.
    pub terminators: Vec<f64>,
    /// Ids of those resistors, in the same order.
    pub terminator_ids: Vec<DeviceId>,
    /// Oscilloscopes wired across the pair.
    pub scopes: Vec<Rc<RefCell<gsim_devices::Oscilloscope>>>,
    /// Motors whose two wire contacts sit on the pair, with their device ids.
    pub motors: Vec<(DeviceId, Rc<RefCell<gsim_devices::Motor>>)>,
    /// Parts whose wire contacts sit on the pair.
    pub nodes: usize,
}

impl CanNetwork {
    /// Electrical quality of the pair, `1.0` for the textbook `60 Ω`.
    pub fn quality(&self) -> f64 {
        gsim_devices::quality_of(self.termination_ohm)
    }
}

/// Effective level of one net.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum NetLevel {
    High,
    Low,
    Floating,
    /// VCC tied to GND, or a push-pull output fighting the rail.
    Shorted,
}

/// Pin levels read out of the simulated device.
#[derive(Clone, Debug, Default)]
pub struct PinLevels {
    pub level: [[bool; 16]; 5],
    pub output: [[bool; 16]; 5],
    /// Measured high-time fraction of pins driven by a timer compare output,
    /// `None` for pins that do not modulate.
    pub pwm: [[Option<f32>; 16]; 5],
}

impl PinLevels {
    pub fn level_of(&self, port: u8, pin: u8) -> bool {
        self.level
            .get(port as usize)
            .and_then(|port| port.get(pin as usize))
            .copied()
            .unwrap_or(false)
    }

    pub fn is_output(&self, port: u8, pin: u8) -> bool {
        self.output
            .get(port as usize)
            .and_then(|port| port.get(pin as usize))
            .copied()
            .unwrap_or(false)
    }

    /// Fraction of the time the pin is high: the measured duty of a PWM output,
    /// or `1.0`/`0.0` for a pin that holds one level.
    pub fn duty_of(&self, port: u8, pin: u8) -> f32 {
        self.pwm
            .get(port as usize)
            .and_then(|levels| levels.get(pin as usize))
            .copied()
            .flatten()
            .unwrap_or(if self.level_of(port, pin) { 1.0 } else { 0.0 })
            .clamp(0.0, 1.0)
    }
}

/// Result of evaluating the board.
#[derive(Clone, Debug, Default)]
pub struct BoardState {
    pub nets: Vec<Vec<PinRef>>,
    pub levels_of_nets: Vec<NetLevel>,
    pub shorts: Vec<Vec<PinRef>>,
    /// LED brightness, `0.0` (dark) to `1.0` (full on). A PWM channel dims the
    /// LED down to its duty cycle.
    pub leds: BTreeMap<DeviceId, f32>,
    /// Supply state of every part whose symbol has `VCC`/`GND` contacts.
    pub supplies: BTreeMap<DeviceId, Supply>,
}

impl BoardState {
    /// How bright an LED is, from dark to fully on.
    pub fn led_brightness(&self, id: DeviceId) -> f32 {
        self.leds.get(&id).copied().unwrap_or(0.0)
    }

    /// True while an LED emits any light at all.
    pub fn led_lit(&self, id: DeviceId) -> bool {
        self.led_brightness(id) > 0.0
    }

    /// How the supply of `id` is wired, `None` for parts without power pins.
    pub fn supply(&self, id: DeviceId) -> Option<Supply> {
        self.supplies.get(&id).copied()
    }
}

/// How the `VCC`/`GND` contacts of a placed part are wired.
///
/// Only a part whose supply sits on the right rails takes part in the
/// simulation: the bridge leaves unpowered (or miswired) parts out of the
/// wiring, so a terminal without power really stays silent.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Supply {
    /// At least one of the two contacts is not wired yet.
    Unwired,
    /// `VCC` sits on a high net and `GND` on a low one.
    Powered,
    /// Both contacts are wired, but not to the rails they belong to.
    Fault,
}

impl Supply {
    fn of(vcc: Option<NetLevel>, gnd: Option<NetLevel>) -> Self {
        match (vcc, gnd) {
            (Some(NetLevel::High), Some(NetLevel::Low)) => Supply::Powered,
            (None, _) | (_, None) => Supply::Unwired,
            _ => Supply::Fault,
        }
    }

    /// True when the part is allowed to run.
    pub fn works(self) -> bool {
        matches!(self, Supply::Powered)
    }

    /// Chinese label for the device list.
    pub fn label(self) -> &'static str {
        match self {
            Supply::Powered => "VCC/GND 已接",
            Supply::Unwired => "VCC/GND 未接（不工作）",
            Supply::Fault => "VCC/GND 接错（不工作）",
        }
    }
}

/// Views of the off-chip parts for the status windows.
///
/// The parts only exist while they are placed on the drawing, so the frame's
/// snapshot is completed from the board instead of from the simulation.
impl Board {
    /// Copies the state of the placed transceiver and LED matrix into
    /// `snapshot`, and appends their rows to the peripheral browser.
    ///
    /// `levels` decides which transceivers are powered, so the CAN window shows
    /// the same pair the bridge is actually driving.
    pub fn fill_snapshot(&self, snapshot: &mut crate::sim::Snapshot, levels: &PinLevels) {
        if let Some(matrix) = self.matrices().first() {
            snapshot.matrix = crate::sim::MatrixSnapshot::of(&matrix.borrow());
        }

        let transceivers = self.transceivers();
        match transceivers.first() {
            Some(handle) => {
                let transceiver = handle.borrow();
                snapshot
                    .can
                    .observe(&self.can_bus.borrow(), Some(&transceiver));
            }
            None => snapshot.can.observe(&self.can_bus.borrow(), None),
        }
        let supplies = self.supplies(levels);
        snapshot.can.network = self.can_network_with(|id| {
            !matches!(supplies.get(&id), Some(Supply::Unwired | Supply::Fault))
        });
        snapshot.scopes = self
            .scopes()
            .iter()
            .map(|scope| crate::sim::ScopeView::of(&scope.borrow()))
            .collect();

        for device in &self.devices {
            match &device.hardware {
                Hardware::Matrix(matrix) => {
                    let matrix = matrix.borrow();
                    snapshot.peripherals.push(status(
                        &device.name,
                        "点阵显示（MAX7219）",
                        matrix_rows(&matrix),
                    ));
                }
                Hardware::Can(transceiver) => {
                    let transceiver = transceiver.borrow();
                    let bus = self.can_bus.borrow();
                    let (tx, rx) = transceiver.pins();
                    let network = &snapshot.can.network;
                    snapshot.peripherals.push(status(
                        &device.name,
                        "CAN 收发器（外部）",
                        vec![
                            ("节点", transceiver.name().to_string()),
                            ("模式", transceiver.mode().label().to_string()),
                            (
                                "引脚",
                                format!("TXD = {}，RXD = {}", pin_label(tx), pin_label(rx)),
                            ),
                            ("已发送帧", format!("{}", transceiver.stats().tx_frames)),
                            ("已接收帧", format!("{}", transceiver.stats().rx_frames)),
                            ("回环帧", format!("{}", transceiver.stats().loopback_frames)),
                            ("总线帧数", format!("{}", bus.stats().frames)),
                            ("终端电阻", termination_label(network.termination_ohm)),
                            ("总线质量", format!("{:.0}%", network.quality() * 100.0)),
                        ],
                    ));
                }
                Hardware::Resistor(resistor) => {
                    let network = &snapshot.can.network;
                    let on_pair = network.terminator_ids.contains(&device.id);
                    snapshot.peripherals.push(status(
                        &device.name,
                        "电阻（无源）",
                        vec![
                            ("阻值", resistor.label()),
                            (
                                "接在总线上",
                                if on_pair {
                                    "是（终端电阻）"
                                } else {
                                    "否"
                                }
                                .to_string(),
                            ),
                            ("总线终端", termination_label(network.termination_ohm)),
                        ],
                    ));
                }
                Hardware::Motor(motor) => {
                    let motor = motor.borrow();
                    let on_pair = snapshot
                        .can
                        .network
                        .motors
                        .iter()
                        .any(|(id, _)| *id == device.id);
                    snapshot.peripherals.push(status(
                        &device.name,
                        "CAN 电机（外部）",
                        vec![
                            ("节点", motor.name().to_string()),
                            ("接在总线上", if on_pair { "是" } else { "否" }.to_string()),
                            (
                                "指令帧",
                                format!("0x{:03X} · data[0..2] = 目标转速", motor.command_id()),
                            ),
                            ("状态", motor.spin().label().to_string()),
                            ("转速", format!("{:+.0} rpm", motor.rpm())),
                            ("目标转速", format!("{:+.0} rpm", motor.target_rpm())),
                            ("累计圈数", format!("{:+.2} 圈", motor.turns())),
                            ("已收帧", format!("{}", motor.frames())),
                            ("已收指令", format!("{}", motor.commands())),
                            (
                                "供电",
                                if motor.enabled() { "正常" } else { "断开" }.to_string(),
                            ),
                        ],
                    ));
                }
                Hardware::Scope(scope) => {
                    let scope = scope.borrow();
                    snapshot.peripherals.push(status(
                        &device.name,
                        "示波器（仪器）",
                        vec![
                            ("窗口", format!("{:.2} ms", scope.window_ns() as f64 / 1e6)),
                            ("采样点数", format!("{}", scope.samples().len())),
                            ("尖峰", format!("{}", scope.glitches())),
                            ("峰峰值", format!("{:.2} V", scope.peak_differential())),
                            ("冻结", if scope.frozen() { "是" } else { "否" }.to_string()),
                        ],
                    ));
                }
                _ => {}
            }
        }
    }
}

/// Text for the parallel resistance of the termination network.
pub fn termination_label(ohms: Option<f64>) -> String {
    match ohms {
        Some(ohms) => format!("{} 并联", gsim_devices::format_ohms(ohms)),
        None => "未接（开路）".to_string(),
    }
}

fn status(
    name: &str,
    kind: &'static str,
    rows: Vec<(&'static str, String)>,
) -> gsim_stm32f1::status::PeripheralStatus {
    gsim_stm32f1::status::PeripheralStatus {
        name: name.to_string(),
        kind,
        base: 0,
        clock_hz: 0,
        rows,
    }
}

fn pin_label((port, pin): (u8, u8)) -> String {
    format!(
        "P{}{}",
        gsim_stm32f1::PORT_NAMES
            .get(usize::from(port))
            .copied()
            .unwrap_or("?"),
        pin
    )
}

/// Rows describing one LED matrix module in the peripheral browser.
fn matrix_rows(matrix: &gsim_devices::Max7219) -> Vec<(&'static str, String)> {
    let mut rows = vec![
        ("级联片数", format!("{}", matrix.chips().len())),
        ("LOAD 引脚", pin_label(matrix.cs_pin())),
        ("SPI 单元", format!("SPI{}", matrix.unit() + 1)),
        ("已接收字节", format!("{}", matrix.frames())),
        ("已锁存字", format!("{}", matrix.words())),
    ];
    if let Some(chip) = matrix.chips().first() {
        rows.push(("亮度", format!("{}/15", chip.intensity())));
        rows.push(("扫描行", format!("{}", chip.scanned_rows())));
        rows.push(("关断", if chip.enabled() { "否" } else { "是" }.to_string()));
        rows.push((
            "显示测试",
            if chip.display_test() { "是" } else { "否" }.to_string(),
        ));
    }
    rows
}

#[cfg(test)]
mod tests {
    use super::*;

    fn levels(low: &[(u8, u8)], high: &[(u8, u8)]) -> PinLevels {
        let mut levels = PinLevels::default();
        for (port, pin) in low {
            levels.level[*port as usize][*pin as usize] = false;
            levels.output[*port as usize][*pin as usize] = true;
        }
        for (port, pin) in high {
            levels.level[*port as usize][*pin as usize] = true;
            levels.output[*port as usize][*pin as usize] = true;
        }
        levels
    }

    #[test]
    fn demo_board_lights_the_led_only_while_pa5_is_high() {
        let board = Board::demo();
        let led = board.first_of(DeviceKind::Led).expect("demo LED").id;

        let state = board.evaluate(&levels(&[], &[(0, 5)]));
        assert!(state.led_lit(led), "LED must light when PA5 is high");
        assert!(state.shorts.is_empty());

        let state = board.evaluate(&levels(&[(0, 5)], &[]));
        assert!(!state.led_lit(led), "LED must go dark when PA5 is low");
    }

    /// Levels where `PA5` carries a measured PWM duty instead of one level.
    fn pwm_levels(duty: f32) -> PinLevels {
        let mut levels = levels(&[], &[]);
        levels.output[0][5] = true;
        levels.level[0][5] = duty > 0.5;
        levels.pwm[0][5] = Some(duty);
        levels
    }

    #[test]
    fn a_pwm_duty_dims_the_led_instead_of_flickering() {
        let board = Board::demo();
        let led = board.first_of(DeviceKind::Led).expect("demo LED").id;

        for (duty, expected) in [(0.0, 0.0), (0.25, 0.25), (0.75, 0.75), (1.0, 1.0)] {
            let state = board.evaluate(&pwm_levels(duty));
            let brightness = state.led_brightness(led);
            assert!(
                (brightness - expected).abs() < 1e-6,
                "duty {duty} became brightness {brightness}"
            );
            assert_eq!(state.led_lit(led), duty > 0.0);
        }
    }

    #[test]
    fn a_dimming_channel_on_the_cathode_is_active_low() {
        let mut board = Board::new();
        let vcc = board.add_vcc(Pos2::new(10.0, 10.0));
        let led = board.add_led(Pos2::new(100.0, 10.0));
        board.connect(
            PinRef::Device {
                device: vcc,
                pin: POWER_PIN,
            },
            PinRef::Device {
                device: led,
                pin: 0,
            },
        );
        board.connect(
            PinRef::Mcu { port: 0, pin: 5 },
            PinRef::Device {
                device: led,
                pin: 1,
            },
        );

        let state = board.evaluate(&pwm_levels(0.25));
        let brightness = state.led_brightness(led);
        assert!(
            (brightness - 0.75).abs() < 1e-6,
            "the LED conducts while the pin is low, got {brightness}"
        );
    }

    #[test]
    fn led_wired_to_vcc_and_a_low_pin_is_active_low() {
        let mut board = Board::new();
        let vcc = board.add_vcc(Pos2::new(10.0, 10.0));
        let led = board.add_led(Pos2::new(100.0, 10.0));
        assert!(board.connect(
            PinRef::Device {
                device: vcc,
                pin: POWER_PIN,
            },
            PinRef::Device {
                device: led,
                pin: 0
            }
        ));
        assert!(board.connect(
            PinRef::Mcu { port: 2, pin: 13 },
            PinRef::Device {
                device: led,
                pin: 1
            }
        ));

        assert!(board.evaluate(&levels(&[(2, 13)], &[])).led_lit(led));
        assert!(!board.evaluate(&levels(&[], &[(2, 13)])).led_lit(led));
    }

    #[test]
    fn led_with_floating_cathode_stays_dark() {
        let mut board = Board::new();
        let led = board.add_led(Pos2::new(10.0, 10.0));
        board.connect(
            PinRef::Mcu { port: 0, pin: 5 },
            PinRef::Device {
                device: led,
                pin: 0,
            },
        );
        assert!(!board.evaluate(&levels(&[], &[(0, 5)])).led_lit(led));
    }

    #[test]
    fn shorted_led_pins_never_light() {
        let mut board = Board::new();
        let led = board.add_led(Pos2::new(10.0, 10.0));
        board.connect(
            PinRef::Mcu { port: 0, pin: 5 },
            PinRef::Device {
                device: led,
                pin: 0,
            },
        );
        board.connect(
            PinRef::Mcu { port: 0, pin: 5 },
            PinRef::Device {
                device: led,
                pin: 1,
            },
        );
        assert!(!board.evaluate(&levels(&[], &[(0, 5)])).led_lit(led));
    }

    #[test]
    fn rails_tied_together_report_a_short() {
        let mut board = Board::new();
        let vcc = board.add_vcc(Pos2::new(10.0, 10.0));
        let gnd = board.add_gnd(Pos2::new(10.0, 90.0));
        assert!(board.connect(
            PinRef::Device {
                device: vcc,
                pin: POWER_PIN,
            },
            PinRef::Device {
                device: gnd,
                pin: POWER_PIN,
            },
        ));
        let state = board.evaluate(&levels(&[], &[]));
        assert_eq!(state.shorts.len(), 1);
        assert_eq!(state.levels_of_nets[0], NetLevel::Shorted);
    }

    #[test]
    fn duplicate_and_self_wires_are_rejected() {
        let mut board = Board::new();
        let gnd = board.add_gnd(Pos2::new(10.0, 10.0));
        let a = PinRef::Mcu { port: 0, pin: 5 };
        let b = PinRef::Device {
            device: gnd,
            pin: POWER_PIN,
        };
        assert!(board.connect(a, b));
        assert!(!board.connect(a, b), "duplicate wire");
        assert!(!board.connect(b, a), "duplicate wire, reversed");
        assert!(!board.connect(a, a), "self loop");
        assert_eq!(board.wires().len(), 1);
    }

    #[test]
    fn nets_merge_transitively() {
        let mut board = Board::new();
        board.add_vcc(Pos2::new(0.0, 0.0));
        let vcc = PinRef::Device {
            device: board.first_of(DeviceKind::Vcc).expect("vcc").id,
            pin: POWER_PIN,
        };
        let led = board.add_led(Pos2::new(100.0, 0.0));
        board.connect(PinRef::Mcu { port: 0, pin: 5 }, vcc);
        board.connect(
            vcc,
            PinRef::Device {
                device: led,
                pin: 0,
            },
        );
        let nets = board.nets();
        assert_eq!(nets.len(), 1);
        assert_eq!(nets[0].len(), 3);
    }

    #[test]
    fn removing_a_device_drops_its_wires() {
        let mut board = Board::demo();
        let led = board.first_of(DeviceKind::Led).expect("demo LED").id;
        let before = board.wires().len();
        board.remove_device(led);
        assert!(board.device(led).is_none());
        assert_eq!(
            board.wires().len(),
            before - 2,
            "only the LED wires go away"
        );
        assert!(!board.wires().iter().any(
            |wire| matches!(wire.a, PinRef::Device { device, .. } if device == led)
                || matches!(wire.b, PinRef::Device { device, .. } if device == led)
        ));
    }

    #[test]
    fn the_demo_board_ships_the_power_symbols_three_parts_and_the_hse_crystal() {
        let board = Board::demo();
        let kinds: Vec<DeviceKind> = board.devices().iter().map(|device| device.kind).collect();
        assert_eq!(
            kinds,
            vec![
                DeviceKind::Vcc,
                DeviceKind::Gnd,
                DeviceKind::Led,
                DeviceKind::Uart,
                DeviceKind::Encoder,
                DeviceKind::HseCrystal,
                DeviceKind::LseCrystal,
            ]
        );
        assert_eq!(board.first_of(DeviceKind::Uart).unwrap().name, "UART1");
        assert_eq!(board.first_of(DeviceKind::Encoder).unwrap().name, "ENC1");
        // LED: 2 wires, UART: 2 + VCC/GND, encoder: 3 + VCC/GND, HSE: 2, LSE: 2.
        assert_eq!(board.wires().len(), 15);
        assert_eq!(
            board.crystals(),
            Crystals {
                hse_hz: Some(8_000_000),
                lse: true,
            }
        );
    }

    #[test]
    fn a_new_project_only_carries_the_hse_crystal() {
        let board = Board::new_project();
        let kinds: Vec<DeviceKind> = board.devices().iter().map(|device| device.kind).collect();
        assert_eq!(kinds, vec![DeviceKind::HseCrystal]);
        let crystal = board.first_of(DeviceKind::HseCrystal).expect("HSE");
        assert!(
            crystal.pos.x < 0.0 && (140.0..200.0).contains(&crystal.pos.y),
            "the crystal belongs left of the chip, next to PD0/PD1"
        );
        assert_eq!(board.wires().len(), 2, "X1/X2 already wired to OSC_IN/OUT");
        assert_eq!(
            board.crystals(),
            Crystals {
                hse_hz: Some(8_000_000),
                lse: false,
            }
        );
    }

    #[test]
    fn crystals_only_count_when_wired_across_the_oscillator_pins() {
        let mut board = Board::new();
        let hse = board.add_hse_crystal(Pos2::new(100.0, 100.0));
        assert_eq!(board.crystals(), Crystals::default(), "no wires yet");

        board.connect(
            PinRef::Device {
                device: hse,
                pin: CRYSTAL_X1,
            },
            PinRef::Mcu {
                port: HSE_PINS[0].0,
                pin: HSE_PINS[0].1,
            },
        );
        assert_eq!(
            board.crystals(),
            Crystals::default(),
            "one terminal is not enough"
        );

        // The other terminal may be wired the other way round: crystals have
        // no polarity.
        board.connect(
            PinRef::Device {
                device: hse,
                pin: CRYSTAL_X2,
            },
            PinRef::Mcu {
                port: HSE_PINS[1].0,
                pin: HSE_PINS[1].1,
            },
        );
        assert_eq!(
            board.crystals(),
            Crystals {
                hse_hz: Some(8_000_000),
                lse: false,
            }
        );
    }

    #[test]
    fn an_lse_crystal_is_detected_on_the_osc32_pins() {
        let mut board = Board::new();
        let lse = board.add_lse_crystal(Pos2::new(100.0, 100.0));
        for (index, (port, pin)) in [(CRYSTAL_X1, LSE_PINS[0]), (CRYSTAL_X2, LSE_PINS[1])] {
            board.connect(
                PinRef::Device {
                    device: lse,
                    pin: index,
                },
                PinRef::Mcu { port, pin },
            );
        }
        assert_eq!(
            board.crystals(),
            Crystals {
                hse_hz: None,
                lse: true,
            }
        );
    }

    #[test]
    fn an_hse_crystal_wired_to_the_lse_pins_is_not_an_hse() {
        let mut board = Board::new();
        let hse = board.add_hse_crystal(Pos2::new(100.0, 100.0));
        for (index, (port, pin)) in [(CRYSTAL_X1, LSE_PINS[0]), (CRYSTAL_X2, LSE_PINS[1])] {
            board.connect(
                PinRef::Device {
                    device: hse,
                    pin: index,
                },
                PinRef::Mcu { port, pin },
            );
        }
        assert_eq!(board.crystals(), Crystals::default());
    }

    #[test]
    fn a_uart_terminal_only_answers_on_the_pins_it_is_wired_to() {
        let mut board = Board::new();
        let uart = board.add_uart(Pos2::new(0.0, 0.0));
        let rx = PinRef::Device {
            device: uart,
            pin: UART_RX,
        };
        let tx = PinRef::Device {
            device: uart,
            pin: UART_TX,
        };
        assert_eq!(board.mcu_pin_for(rx), None);
        board.connect(PinRef::Mcu { port: 0, pin: 9 }, rx);
        board.connect(tx, PinRef::Mcu { port: 0, pin: 10 });
        assert_eq!(board.mcu_pin_for(rx), Some((0, 9)));
        assert_eq!(board.mcu_pin_for(tx), Some((0, 10)));
        // A power symbol is just another device pin.
        let gnd = board.add_gnd(Pos2::new(0.0, 100.0));
        let gnd_pin = PinRef::Device {
            device: gnd,
            pin: POWER_PIN,
        };
        board.connect(gnd_pin, PinRef::Mcu { port: 0, pin: 0 });
        assert_eq!(board.mcu_pin_for(gnd_pin), Some((0, 0)));
    }

    #[test]
    fn encoder_pins_are_spread_along_their_side() {
        let mut board = Board::new();
        let encoder = board.add_encoder(Pos2::new(100.0, 100.0));
        let device = board.device(encoder).unwrap();
        let a = device.pin_pos(ENCODER_A);
        let b = device.pin_pos(ENCODER_B);
        let sw = device.pin_pos(ENCODER_SW);
        assert_eq!(a.x, b.x, "A and B are on the same side");
        assert_eq!(
            a.x, sw.x,
            "SW is a controller signal, so it joins A/B on the left"
        );
        assert!(a.y < b.y && b.y < sw.y, "the contacts must not overlap");
        assert!(a.x < device.rect().left());
        assert!((a.y - b.y).abs() > 8.0);
        // The two supply contacts share the top edge, VCC left of GND.
        let vcc = device.pin_pos(ENCODER_VCC);
        let gnd = device.pin_pos(ENCODER_GND);
        assert!(vcc.y < device.rect().top() && gnd.y < device.rect().top());
        assert_eq!(vcc.y, gnd.y);
        assert!(vcc.x < gnd.x);
    }

    #[test]
    fn pins_are_grouped_by_electrical_role() {
        // Signals that talk to the controller sit on the left, bus/output
        // contacts on the right, and both supply contacts along the top.
        let expected: [(DeviceKind, &[(u8, Side)]); 4] = [
            (
                DeviceKind::Uart,
                &[
                    (UART_RX, Side::Left),
                    (UART_TX, Side::Left),
                    (UART_VCC, Side::Top),
                    (UART_GND, Side::Top),
                ],
            ),
            (
                DeviceKind::Encoder,
                &[
                    (ENCODER_A, Side::Left),
                    (ENCODER_B, Side::Left),
                    (ENCODER_SW, Side::Left),
                    (ENCODER_VCC, Side::Top),
                    (ENCODER_GND, Side::Top),
                ],
            ),
            (
                DeviceKind::Can,
                &[
                    (CAN_TXD, Side::Left),
                    (CAN_RXD, Side::Left),
                    (CAN_CANH, Side::Right),
                    (CAN_CANL, Side::Right),
                    (CAN_VCC, Side::Top),
                    (CAN_GND, Side::Top),
                ],
            ),
            (
                DeviceKind::Matrix,
                &[
                    (MATRIX_DIN, Side::Left),
                    (MATRIX_CLK, Side::Left),
                    (MATRIX_CS, Side::Left),
                    (MATRIX_DOUT, Side::Right),
                    (MATRIX_VCC, Side::Top),
                    (MATRIX_GND, Side::Top),
                ],
            ),
        ];
        for (kind, pins) in expected {
            let table = kind.pins();
            assert_eq!(table.len(), pins.len(), "{kind:?} pin table size");
            for &(index, side) in pins {
                assert_eq!(
                    table[usize::from(index)].side,
                    side,
                    "{kind:?} pin {index} ({}) is on the wrong side",
                    table[usize::from(index)].name
                );
            }
        }
    }

    #[test]
    fn placed_devices_carry_their_own_runtime_state() {
        let mut board = Board::new();
        let uart = board.add_uart(Pos2::new(0.0, 0.0));
        let encoder = board.add_encoder(Pos2::new(200.0, 0.0));
        let led = board.add_led(Pos2::new(400.0, 0.0));

        assert_eq!(
            board.device(uart).unwrap().hardware.uart().unwrap().unit(),
            0
        );
        assert_eq!(
            board
                .device(encoder)
                .unwrap()
                .hardware
                .encoder()
                .unwrap()
                .detents(),
            0
        );
        assert!(matches!(
            board.device(led).unwrap().hardware,
            Hardware::None
        ));
    }

    #[test]
    fn device_names_are_unique_and_numbered() {
        let mut board = Board::new();
        let first = board.add_led(Pos2::new(0.0, 0.0));
        let second = board.add_led(Pos2::new(0.0, 100.0));
        assert_eq!(board.device(first).unwrap().name, "LED1");
        assert_eq!(board.device(second).unwrap().name, "LED2");
        assert_ne!(first, second);
    }

    #[test]
    fn device_pin_positions_stick_out_of_the_body() {
        let mut board = Board::new();
        let led = board.add_led(Pos2::new(100.0, 100.0));
        let device = board.device(led).unwrap();
        let anode = device.pin_pos(0);
        let cathode = device.pin_pos(1);
        assert!(anode.x < device.rect().left());
        assert!(cathode.x > device.rect().right());
        assert_eq!(anode.y, cathode.y);
    }

    #[test]
    fn supply_pins_are_appended_without_moving_the_signal_pins() {
        // Published pin indices must never move: wires, tests and saved
        // drawings all address pins by index.
        assert_eq!(
            (
                UART_RX,
                UART_TX,
                UART_VCC,
                UART_GND,
                ENCODER_A,
                ENCODER_B,
                ENCODER_SW,
                ENCODER_VCC,
                ENCODER_GND
            ),
            (0, 1, 2, 3, 0, 1, 2, 3, 4)
        );
        assert_eq!(
            (CAN_TXD, CAN_RXD, CAN_CANH, CAN_CANL, CAN_VCC, CAN_GND),
            (0, 1, 2, 3, 4, 5)
        );
        assert_eq!(
            (
                MATRIX_DIN,
                MATRIX_CLK,
                MATRIX_CS,
                MATRIX_DOUT,
                MATRIX_VCC,
                MATRIX_GND
            ),
            (0, 1, 2, 3, 4, 5)
        );
        for kind in [
            DeviceKind::Uart,
            DeviceKind::Encoder,
            DeviceKind::Can,
            DeviceKind::Matrix,
        ] {
            let (vcc, gnd) = kind.supply_pins().expect("part must have a supply");
            assert_eq!(kind.pins()[vcc as usize].name, "VCC");
            assert_eq!(kind.pins()[vcc as usize].role, PinRole::Vcc);
            assert_eq!(kind.pins()[gnd as usize].name, "GND");
            assert_eq!(kind.pins()[gnd as usize].role, PinRole::Ground);
        }
        for kind in [
            DeviceKind::Led,
            DeviceKind::Vcc,
            DeviceKind::Gnd,
            DeviceKind::Resistor,
            DeviceKind::Scope,
            DeviceKind::HseCrystal,
            DeviceKind::LseCrystal,
        ] {
            assert_eq!(
                kind.supply_pins(),
                None,
                "{} must not grow a supply",
                kind.label()
            );
        }
    }

    #[test]
    fn the_demo_board_powers_every_part_that_has_a_supply() {
        let board = Board::demo();
        let state = board.evaluate(&PinLevels::default());
        for kind in [DeviceKind::Uart, DeviceKind::Encoder] {
            let id = board.first_of(kind).expect("demo part").id;
            assert_eq!(state.supply(id), Some(Supply::Powered), "{kind:?}");
        }
        // Parts without supply contacts report nothing at all.
        for kind in [
            DeviceKind::Led,
            DeviceKind::HseCrystal,
            DeviceKind::LseCrystal,
        ] {
            let id = board.first_of(kind).expect("demo part").id;
            assert_eq!(state.supply(id), None, "{kind:?}");
        }
    }

    #[test]
    fn an_open_supply_contact_is_reported_as_unwired() {
        let mut board = Board::new();
        let uart = board.add_uart(Pos2::new(0.0, 0.0));
        let state = board.evaluate(&PinLevels::default());
        assert_eq!(state.supply(uart), Some(Supply::Unwired));

        // Only the `VCC` contact wired: still incomplete.
        let vcc = board.add_vcc(Pos2::new(200.0, 0.0));
        board.connect(
            PinRef::Device {
                device: vcc,
                pin: POWER_PIN,
            },
            PinRef::Device {
                device: uart,
                pin: UART_VCC,
            },
        );
        let state = board.evaluate(&PinLevels::default());
        assert_eq!(state.supply(uart), Some(Supply::Unwired));

        let gnd = board.add_gnd(Pos2::new(200.0, 100.0));
        board.connect(
            PinRef::Device {
                device: gnd,
                pin: POWER_PIN,
            },
            PinRef::Device {
                device: uart,
                pin: UART_GND,
            },
        );
        let state = board.evaluate(&PinLevels::default());
        assert_eq!(state.supply(uart), Some(Supply::Powered));
    }

    #[test]
    fn a_supply_on_the_wrong_rail_is_a_fault() {
        let mut board = Board::new();
        let uart = board.add_uart(Pos2::new(0.0, 0.0));
        let vcc = board.add_vcc(Pos2::new(200.0, 0.0));
        let gnd = board.add_gnd(Pos2::new(200.0, 100.0));
        // Crossed: `VCC` on the ground rail and `GND` on the supply rail.
        board.connect(
            PinRef::Device {
                device: gnd,
                pin: POWER_PIN,
            },
            PinRef::Device {
                device: uart,
                pin: UART_VCC,
            },
        );
        board.connect(
            PinRef::Device {
                device: vcc,
                pin: POWER_PIN,
            },
            PinRef::Device {
                device: uart,
                pin: UART_GND,
            },
        );
        let state = board.evaluate(&PinLevels::default());
        assert_eq!(state.supply(uart), Some(Supply::Fault));
    }

    #[test]
    fn a_junction_taps_a_wire_without_breaking_its_net() {
        let mut board = Board::new();
        let led = board.add_led(Pos2::new(600.0, 120.0));
        let mcu = PinRef::Mcu { port: 0, pin: 5 };
        let anode = PinRef::Device {
            device: led,
            pin: 0,
        };
        board.connect(mcu, anode);
        board.insert_bend(0, 0, Pos2::new(400.0, 80.0));

        // Split between the corner and the LED, i.e. on the second leg.
        let junction = board
            .add_junction(0, 1, Pos2::new(500.0, 200.0))
            .expect("the wire exists");
        let tap = PinRef::Device {
            device: junction,
            pin: JUNCTION_PIN,
        };
        assert_eq!(
            board.device(junction).map(|device| device.kind),
            Some(DeviceKind::Junction)
        );
        assert_eq!(board.wires().len(), 2, "the wire became two halves");
        assert_eq!(board.wires()[0].a, mcu);
        assert_eq!(board.wires()[0].b, tap);
        assert_eq!(board.wires()[0].bends, vec![Pos2::new(400.0, 80.0)]);
        assert_eq!(board.wires()[1].a, tap);
        assert_eq!(board.wires()[1].b, anode);
        assert!(board.wires()[1].bends.is_empty());

        // The tap does not break the net, and it is a terminal of its own.
        assert!(board.same_net(mcu, anode));
        assert_eq!(board.mcu_pin_for(anode), Some((0, 5)));
        let gnd = board.add_gnd(Pos2::new(200.0, 300.0));
        assert!(board.connect(
            tap,
            PinRef::Device {
                device: gnd,
                pin: POWER_PIN,
            }
        ));
        let nets = board.nets();
        assert_eq!(nets.len(), 1);
        assert_eq!(nets[0].len(), 4, "both halves and the new wire share it");
    }

    #[test]
    fn removing_a_junction_fuses_the_two_halves_back() {
        let mut board = Board::new();
        let led = board.add_led(Pos2::new(600.0, 120.0));
        let mcu = PinRef::Mcu { port: 0, pin: 5 };
        let anode = PinRef::Device {
            device: led,
            pin: 0,
        };
        board.connect(mcu, anode);
        board.insert_bend(0, 0, Pos2::new(400.0, 80.0));
        board.insert_bend(0, 1, Pos2::new(420.0, 200.0));
        let junction = board
            .add_junction(0, 1, Pos2::new(500.0, 200.0))
            .expect("the wire exists");

        board.remove_device(junction);
        assert!(board.first_of(DeviceKind::Junction).is_none());
        assert_eq!(board.wires().len(), 1, "the halves were fused back");
        let wire = &board.wires()[0];
        assert_eq!((wire.a, wire.b), (mcu, anode));
        assert_eq!(
            wire.bends,
            vec![Pos2::new(400.0, 80.0), Pos2::new(420.0, 200.0)],
            "the corners keep their order from `a` to `b`"
        );
        assert!(board.is_connected(mcu, anode));
    }

    #[test]
    fn a_junction_can_only_split_a_leg_that_exists() {
        let mut board = Board::new();
        let led = board.add_led(Pos2::new(600.0, 120.0));
        board.connect(
            PinRef::Mcu { port: 0, pin: 5 },
            PinRef::Device {
                device: led,
                pin: 0,
            },
        );
        assert!(board.add_junction(0, 1, Pos2::new(10.0, 10.0)).is_none());
        assert!(board.add_junction(9, 0, Pos2::new(10.0, 10.0)).is_none());
        assert_eq!(board.wires().len(), 1);
        assert!(board.first_of(DeviceKind::Junction).is_none());
    }

    #[test]
    fn clearing_the_wires_takes_the_junctions_with_them() {
        let mut board = Board::new();
        let led = board.add_led(Pos2::new(600.0, 120.0));
        board.connect(
            PinRef::Mcu { port: 0, pin: 5 },
            PinRef::Device {
                device: led,
                pin: 0,
            },
        );
        board.add_junction(0, 0, Pos2::new(300.0, 100.0));
        board.clear_wires();
        assert!(board.wires().is_empty());
        assert!(board.first_of(DeviceKind::Junction).is_none());
        assert!(board.device(led).is_some(), "parts stay on the board");
    }
}
