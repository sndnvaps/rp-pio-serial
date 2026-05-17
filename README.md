## rp-pio-serial

PIO-based software serial for RP2040 & RP2350 using arbitrary GPIO pins

## support rp2040-hal & rp235x-hal


### Test for

|Platform | Chip    |Yes/No |
|:-------:|:-------:|:------:|
|PicoW    | rp2040  | Y  |
|YD-rp2040| rp2040  | Y  |
|ProMicro | rp2040  | Y  |
|Waveshare-Rp2040-zero| rp2040| X|
|Pico2    | rp2350  | Y  |

### How to install 

```bash
cargo add rp-pio-serial
```

### How to use it in you project

```toml
# add to cargo.toml
# rp2040 is default-feature
rp-pio-serial = { version = "0.6.0" }
# rp2350
# rp-pio-serial = {  version = "0.6.0", default-features = false, features = [ "rp2350"] }
```


```rust
//import it in main.rs
use rp_pio_serial::{
    pio_bprintln, pio_print, pio_println, DataBits, Parity, RpPioSerial, ServiceMode, StopBits,
    UartConfig,
};

```

### Add to main func

```rust
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

```

```rust
//base use

    pio_println!(serial, "rp2040 PIO serial boot");

    loop {
        serial.poll()
    }
```

## For example 

[rp2040-pio-example](rp2040-pio-example)

[rp2350-pio-example](rp2350-pio-example)


