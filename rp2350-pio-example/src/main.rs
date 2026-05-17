#![no_std]
#![no_main]

// For string formatting.
// The macro for our start-up function
// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use rp235x_hal as hal;

use hal::{
    entry,
    clocks::init_clocks_and_plls,
    gpio::{FunctionPio0, Pins},
    pac,
    pac::PIO0,
    pio::PIOExt,
    pio::SM0,
    pio::SM1,
    sio::Sio,
    watchdog::Watchdog,
    Clock,
};

use rp_pio_serial::{
    pio_bprintln, pio_print, pio_println, DataBits, Parity, RpPioSerial, ServiceMode, StopBits,
    UartConfig,
};

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

/// External high-speed crystal on the Raspberry Pi Pico board is 12 MHz. Adjust
/// if your board has a different frequency
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

/// The linker will place this boot block at the start of our program image. We
/// need this to help the ROM bootloader get our code up and running.
/// Note: This boot block is not necessary when using a rp-hal based BSP
/// as the BSPs already perform this step.
#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

/// Entry point to our bare-metal application.
///
/// The `#[entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables are initialised.
///
/// The function configures the RP2040 peripherals,
/// gets a handle on the I2C peripheral,
/// initializes the SSD1306 driver, initializes the text builder
/// and then draws some text on the display.
///
///

#[entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();
    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    //
    // The default is to generate a 125 MHz system clock
    let clocks = init_clocks_and_plls(
        XTAL_FREQ_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    //let mut delay = cortex_m::delay::Delay::new(core.PLL_SYS, clocks.system_clock.freq().to_Hz());
    //let mut timer = hal::Timer::new_timer0(pac.TIMER0, &mut pac.RESETS, &clocks);
    // The single-cycle I/O block controls our GPIO pins
     let sio = Sio::new(pac.SIO);
     let pins = Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

     // PIO0 -> FunctionPio0
     let _tx_pin = pins.gpio10.into_function::<FunctionPio0>();
     let _rx_pin = pins.gpio9.into_function::<FunctionPio0>();
     //_rx_pin.into_push_pull_output_in_state(PinState::High); //设置高电平，给rx引脚
 
     // 演示“任意状态机组合”：这里使用 SM2 / SM3
     let (mut pio0, sm0, sm1, _sm2, _sm3) = pac.PIO0.split(&mut pac.RESETS);
 
     let config = UartConfig {
         baud: 115_200u32,
         data_bits: DataBits::Eight,
         parity: Parity::None,
         stop_bits: StopBits::One,
         auto_echo: false,
         service_mode: ServiceMode::Polling,
         tx_pipeline_chars: 5,
     };
 
     let mut serial: RpPioSerial<PIO0, SM0, SM1, 512, 512> = RpPioSerial::new(
         &mut pio0,
         sm0, // TX SM
         sm1, // RX SM
         10,  // TX pin
         9,   // RX pin
         clocks.system_clock.freq().to_Hz(),
         config,
     )
     .unwrap();
 
     // 启动日志使用阻塞发送：不依赖后续 poll()
     /*
     pio_bprintln!(serial, "================================");
     pio_bprintln!(serial, "RP2350 PIO serial boot");
     pio_bprintln!(serial, "PIO = PIO0, TX_SM = SM1, RX_SM = SM2");
     pio_bprintln!(serial, "TX = GPIO10, RX = GPIO9");
     pio_bprintln!(serial, "baud = {}", serial.baud());
     pio_bprintln!(serial, "================================");
     */
     pio_println!(serial, "rp2350 PIO serial boot");
 
     //serial.clear_rx();
     let mut rx_buf = [0u8; 128];
 
     loop {
         serial.poll();
 
         let n = serial.read(&mut rx_buf);
         
         if n == 0 {
             continue;
         }
     
         // 1) 先打印 hex（最可靠）
         pio_println!(serial, "RX n={}", n);
         for &b in &rx_buf[..n] {
             pio_println!(serial, "0x{:02X}", b);
         }
     
         // 2) 再尝试按 UTF-8 打印（不成就说明是非文本或帧错）
         match str::from_utf8(&rx_buf[..n]) {
             Ok(s) => pio_println!(serial, "RX str: {}", s),
             Err(_) => pio_println!(serial, "RX non-utf8"),
         }
         
     }
}

