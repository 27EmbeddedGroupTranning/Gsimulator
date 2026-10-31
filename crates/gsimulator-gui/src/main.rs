//! Gsimulator desktop front end.
//!
//! A schematic-style window on top of the register level STM32F103 simulator:
//! it shows the simulated state, the level of every pin, and lets pins be wired
//! to virtual peripherals the way an EDA tool would.
//!
//! ```text
//! cargo run -p gsimulator-gui                     # built-in PA5 square wave
//! cargo run -p gsimulator-gui -- path\to\app.elf  # run a real firmware
//! ```

use std::path::PathBuf;

use gsimulator_gui::app::App;

const USAGE: &str = "\
Gsimulator 图形前端

用法：
    gsimulator-gui [固件.elf]

参数：
    固件.elf   要执行的 Cortex-M3 ELF；省略时画布为空且不加载驱动源
               （工具栏的「内置激励」按钮可手动产生 PA5 方波）

界面：
    器件库                     工具栏「＋ 添加器件」：挑器件后点画布放置，或把卡片拖到画布
    点击引脚 → 点击另一个引脚   完成连线（自动绕开器件外形）
    拖动线段 / 双击连线         添加拐点；拐点可拖动、Delete 删除
    Delete                     删除选中的连线、拐点或外设
    中键拖动 / 滚轮             平移 / 缩放
    ⚙ 激励                     配置内置激励的引脚 / 波形 / 半周期 / 占空比 / 自定义函数
    空格                       运行 / 暂停
    R                          复位
    Esc                        取消连线 / 结束放置

环境变量：
    GSIM_FONT   指定中文字体文件（.ttf/.otf/.ttc）";

fn main() -> eframe::Result<()> {
    let mut elf: Option<PathBuf> = None;
    for argument in std::env::args().skip(1) {
        match argument.as_str() {
            "-h" | "--help" => {
                println!("{USAGE}");
                return Ok(());
            }
            "-V" | "--version" => {
                println!("gsimulator-gui {}", env!("CARGO_PKG_VERSION"));
                return Ok(());
            }
            other if other.starts_with('-') => {
                eprintln!("未知选项 {other}\n\n{USAGE}");
                std::process::exit(2);
            }
            path => elf = Some(PathBuf::from(path)),
        }
    }

    let options = eframe::NativeOptions {
        viewport: eframe::egui::ViewportBuilder::default()
            .with_inner_size([1200.0, 760.0])
            .with_min_inner_size([640.0, 420.0])
            .with_maximized(true)
            .with_title("Gsimulator · STM32F103 仿真前端"),
        ..Default::default()
    };

    eframe::run_native(
        "Gsimulator",
        options,
        Box::new(move |cc| Ok(Box::new(App::new(cc, elf.clone())))),
    )
}
