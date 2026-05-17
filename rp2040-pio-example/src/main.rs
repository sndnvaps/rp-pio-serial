#![no_std]
#![no_main]

use core::usize;

use hal::entry;
use panic_halt as _;
use rp2040_hal as hal;
use rp2040_hal::pio::SM0;
use rp2040_hal::pio::SM1;

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;

use hal::{
    clocks::init_clocks_and_plls,
    gpio::{FunctionPio0, Pins},
    pac,
    pac::PIO0,
    pio::PIOExt,
    sio::Sio,
    watchdog::Watchdog,
    Clock,
};

use rp_pio_serial::{
    pio_bprintln, pio_print, pio_println, DataBits, Parity, RpPioSerial, ServiceMode, StopBits,
    UartConfig,
};

const XTAL_FREQ_HZ: u32 = 12_000_000u32;

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);

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
    pio_bprintln!(serial, "RP2040 PIO serial boot");
    pio_bprintln!(serial, "PIO = PIO0, TX_SM = SM2, RX_SM = SM3");
    pio_bprintln!(serial, "TX = GPIO10, RX = GPIO9");
    pio_bprintln!(serial, "baud = {}", serial.baud());
    pio_bprintln!(serial, "================================");
    */
    pio_println!(serial, "rp2040 PIO serial boot");

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
        /*
        //此代码片段只打印最前面一个字符
                if n > 0 {
                    pio_print!(serial, "n={}", n);
                    //pio_println!(serial,"");
                    // 只打印前几个字节
                    for &b in &rx_buf[..n] {
                        pio_print!(serial, " {:02X}", b);
                    }
                    pio_println!(serial, "");
                }
                */
        
    }
}
