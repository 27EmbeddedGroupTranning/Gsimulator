//! Configuration window for the built-in stimulus.
//!
//! Like the CAN window this is a pure view: it turns the user's edits into
//! [`StimulusAction`] values and leaves `app.rs` in charge of talking to the
//! simulation.

use eframe::egui;
use egui::{Color32, RichText};

use crate::sim::{Snapshot, StimulusConfig, StimulusWaveform};

/// Commands the window sends back to the application.
#[derive(Clone, Debug, PartialEq)]
pub enum StimulusAction {
    /// Install this configuration; `run` also starts it when it was idle.
    Apply { config: StimulusConfig, run: bool },
    /// Stop the waveform and release the pin.
    Stop,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum WaveformDraft {
    Square,
    LevelHigh,
    LevelLow,
    Function,
}

impl WaveformDraft {
    fn all() -> [WaveformDraft; 4] {
        [
            WaveformDraft::Square,
            WaveformDraft::LevelHigh,
            WaveformDraft::LevelLow,
            WaveformDraft::Function,
        ]
    }

    fn label(self) -> &'static str {
        match self {
            WaveformDraft::Square => "方波",
            WaveformDraft::LevelHigh => "恒定高电平",
            WaveformDraft::LevelLow => "恒定低电平",
            WaveformDraft::Function => "自定义函数",
        }
    }
}

/// State of the stimulus settings window.
#[derive(Debug)]
pub struct StimulusWindow {
    /// `true` while the window is open.
    pub open: bool,
    port: u8,
    pin: u8,
    waveform: WaveformDraft,
    half_period_ms: f32,
    duty_percent: f32,
    expression: String,
    error: Option<String>,
}

impl Default for StimulusWindow {
    fn default() -> Self {
        StimulusWindow::from_config(&StimulusConfig::default())
    }
}

impl StimulusWindow {
    /// Seeds the draft from the simulation's current settings.
    pub fn from_config(config: &StimulusConfig) -> Self {
        let mut window = StimulusWindow {
            open: false,
            port: config.port,
            pin: config.pin,
            waveform: WaveformDraft::Square,
            half_period_ms: 120.0,
            duty_percent: 50.0,
            expression: "sin(2*pi*2*t) > 0".to_string(),
            error: None,
        };
        match &config.waveform {
            StimulusWaveform::Square {
                half_period_ns,
                duty,
            } => {
                window.waveform = WaveformDraft::Square;
                window.half_period_ms = *half_period_ns as f32 / 1e6;
                window.duty_percent = duty * 100.0;
            }
            StimulusWaveform::Level(true) => window.waveform = WaveformDraft::LevelHigh,
            StimulusWaveform::Level(false) => window.waveform = WaveformDraft::LevelLow,
            StimulusWaveform::Function { expression } => {
                window.waveform = WaveformDraft::Function;
                window.expression = expression.clone();
            }
        }
        window
    }

    pub fn toggle(&mut self) {
        self.open = !self.open;
    }

    /// Draft settings, as they would be applied.
    pub fn config(&self) -> StimulusConfig {
        let waveform = match self.waveform {
            WaveformDraft::Square => StimulusWaveform::Square {
                half_period_ns: (f64::from(self.half_period_ms) * 1e6).round() as u64,
                duty: (self.duty_percent / 100.0).clamp(0.0, 1.0),
            },
            WaveformDraft::LevelHigh => StimulusWaveform::Level(true),
            WaveformDraft::LevelLow => StimulusWaveform::Level(false),
            WaveformDraft::Function => StimulusWaveform::Function {
                expression: self.expression.clone(),
            },
        };
        StimulusConfig {
            port: self.port,
            pin: self.pin,
            waveform,
        }
    }

    pub fn show(&mut self, ctx: &egui::Context, snapshot: &Snapshot) -> Option<StimulusAction> {
        if !self.open {
            return None;
        }
        let mut action = None;
        let mut open = self.open;
        egui::Window::new("激励配置")
            .id(egui::Id::new("stimulus-config"))
            .open(&mut open)
            .default_size([430.0, 360.0])
            .default_pos([360.0, 160.0])
            .resizable(true)
            .show(ctx, |ui| self.body(ui, snapshot, &mut action));
        self.open = open;
        action
    }

    fn body(
        &mut self,
        ui: &mut egui::Ui,
        snapshot: &Snapshot,
        action: &mut Option<StimulusAction>,
    ) {
        ui.heading("激励设置");
        ui.label(
            RichText::new("不加载固件时，由它按寄存器路径驱动一个 MCU 引脚")
                .small()
                .color(Color32::GRAY),
        );
        ui.add_space(6.0);

        ui.horizontal(|ui| {
            ui.label(RichText::new("引脚").color(Color32::GRAY));
            egui::ComboBox::from_id_salt("stimulus-port")
                .selected_text(format!("P{}x", port_name(self.port)))
                .show_ui(ui, |ui| {
                    for (index, name) in gsim_stm32f1::PORT_NAMES.iter().take(3).enumerate() {
                        if ui
                            .selectable_label(usize::from(self.port) == index, format!("P{name}x"))
                            .clicked()
                        {
                            self.port = index as u8;
                            let range = pin_range(self.port);
                            self.pin = self.pin.clamp(*range.start(), *range.end());
                        }
                    }
                });
            let range = pin_range(self.port);
            ui.add(egui::Slider::new(&mut self.pin, range).text("引脚号"));
        });

        ui.horizontal(|ui| {
            ui.label(RichText::new("波形").color(Color32::GRAY));
            egui::ComboBox::from_id_salt("stimulus-waveform")
                .selected_text(self.waveform.label())
                .show_ui(ui, |ui| {
                    for candidate in WaveformDraft::all() {
                        if ui
                            .selectable_label(self.waveform == candidate, candidate.label())
                            .clicked()
                        {
                            self.waveform = candidate;
                        }
                    }
                });
        });

        match self.waveform {
            WaveformDraft::Square => {
                ui.add(
                    egui::Slider::new(&mut self.half_period_ms, 0.001..=5000.0)
                        .logarithmic(true)
                        .text("半周期"),
                );
                let hz = 1e3 / (2.0 * self.half_period_ms.max(1e-3));
                ui.label(
                    RichText::new(format!(
                        "频率 ≈ {hz:.3} Hz，单位 ms；占空比为高电平所占比例"
                    ))
                    .small()
                    .color(Color32::GRAY),
                );
                ui.add(egui::Slider::new(&mut self.duty_percent, 1.0..=99.0).text("占空比 (%)"));
            }
            WaveformDraft::Function => {
                ui.add(
                    egui::TextEdit::singleline(&mut self.expression)
                        .desired_width(f32::INFINITY)
                        .hint_text("sin(2*pi*2*t) > 0"),
                );
                ui.label(
                    RichText::new(
                        "t = 仿真时间（秒），ms = 毫秒；结果 > 0.5 为高电平。\
                         可用 sin/cos/tan/sqrt/abs/floor/ceil/fract/min/max/pow/clamp/step。",
                    )
                    .small()
                    .color(Color32::GRAY),
                );
                if ui.small_button("插入 1 Hz 方波示例").clicked() {
                    self.expression = "sin(2*pi*1*t) > 0".to_string();
                }
            }
            WaveformDraft::LevelHigh | WaveformDraft::LevelLow => {
                ui.label(
                    RichText::new("引脚保持在所选电平，不再翻转")
                        .small()
                        .color(Color32::GRAY),
                );
            }
        }

        ui.add_space(8.0);
        self.state_row(ui, snapshot);
        ui.separator();

        let active = snapshot.stimulus.is_some();
        ui.horizontal(|ui| {
            let run = ui
                .button("应用并运行")
                .on_hover_text("保存设置并从头开始产生波形")
                .clicked();
            let apply = ui
                .button("应用")
                .on_hover_text("保存设置；正在运行时从当前相位重新开始")
                .clicked();
            let stop = ui
                .add_enabled(active, egui::Button::new("停止"))
                .on_hover_text("停止波形并把引脚交还为输入")
                .clicked();
            if run || apply {
                let config = self.config();
                match config.validate() {
                    Ok(()) => {
                        self.error = None;
                        *action = Some(StimulusAction::Apply { config, run });
                    }
                    Err(error) => self.error = Some(error),
                }
            } else if stop {
                self.error = None;
                *action = Some(StimulusAction::Stop);
            }
        });

        if let Some(error) = &self.error {
            ui.colored_label(Color32::from_rgb(240, 120, 120), error);
        }
    }

    fn state_row(&self, ui: &mut egui::Ui, snapshot: &Snapshot) {
        match &snapshot.stimulus {
            Some(stimulus) => {
                ui.horizontal(|ui| {
                    ui.label(RichText::new("当前").color(Color32::GRAY));
                    ui.monospace(stimulus.config.pin_label());
                    let (text, color) = if stimulus.high {
                        ("高电平", Color32::from_rgb(90, 226, 120))
                    } else {
                        ("低电平", Color32::from_rgb(140, 150, 164))
                    };
                    ui.colored_label(color, text);
                });
                if let Some(next) = stimulus.next_toggle_ns {
                    let remaining = next.saturating_sub(snapshot.now_ns) as f64 / 1e6;
                    ui.label(
                        RichText::new(format!("{remaining:.1} ms 后翻转"))
                            .small()
                            .color(Color32::GRAY),
                    );
                }
            }
            None => {
                ui.label(RichText::new("激励未运行").color(Color32::GRAY));
            }
        }
    }
}

fn port_name(port: u8) -> &'static str {
    gsim_stm32f1::PORT_NAMES
        .get(usize::from(port))
        .copied()
        .unwrap_or("?")
}

fn pin_range(port: u8) -> std::ops::RangeInclusive<u8> {
    if port <= 1 {
        0..=15
    } else {
        13..=15
    }
}

#[cfg(test)]
mod tests {
    use super::StimulusWindow;
    use crate::sim::{StimulusConfig, StimulusWaveform};

    #[test]
    fn the_draft_round_trips_the_simulation_settings() {
        let config = StimulusConfig {
            port: 1,
            pin: 13,
            waveform: StimulusWaveform::Square {
                half_period_ns: 10_000_000,
                duty: 0.25,
            },
        };
        let window = StimulusWindow::from_config(&config);
        let draft = window.config();
        assert_eq!(draft.port, 1);
        assert_eq!(draft.pin, 13);
        match draft.waveform {
            StimulusWaveform::Square {
                half_period_ns,
                duty,
            } => {
                assert_eq!(half_period_ns, 10_000_000);
                assert!((duty - 0.25).abs() < 1e-6);
            }
            other => panic!("unexpected waveform {other:?}"),
        }
    }

    #[test]
    fn function_drafts_keep_their_expression() {
        let config = StimulusConfig {
            port: 0,
            pin: 5,
            waveform: StimulusWaveform::Function {
                expression: "t > 0.1".to_string(),
            },
        };
        let window = StimulusWindow::from_config(&config);
        assert_eq!(
            window.config().waveform,
            StimulusWaveform::Function {
                expression: "t > 0.1".to_string()
            }
        );
    }

    #[test]
    fn level_drafts_map_to_the_right_level() {
        let high = StimulusWindow::from_config(&StimulusConfig {
            port: 0,
            pin: 5,
            waveform: StimulusWaveform::Level(true),
        });
        assert_eq!(high.config().waveform, StimulusWaveform::Level(true));
        let low = StimulusWindow::from_config(&StimulusConfig {
            port: 0,
            pin: 5,
            waveform: StimulusWaveform::Level(false),
        });
        assert_eq!(low.config().waveform, StimulusWaveform::Level(false));
    }
}
