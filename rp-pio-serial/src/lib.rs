#![no_std]

use core::fmt::{self, Write as FmtWrite};
use core::hint::spin_loop;

use cortex_m::asm;
use heapless::spsc::Queue;
use pio_proc::pio_asm;
use rp2040_hal::pio::{
    PIOBuilder, PIOExt, PinDir, Running, Rx, ShiftDirection, StateMachine, StateMachineIndex, Tx,
    UninitStateMachine, PIO,
};

const DEFAULT_TX_BUF_SIZE: usize = 512;
const DEFAULT_RX_BUF_SIZE: usize = 512;

/// 经验值：TX FIFO(4) + 在途/移位路径(1)
const DEFAULT_TX_PIPELINE_CHARS: u32 = 5;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InitError {
    InvalidBaud,
    UnsupportedConfig,
    InstallTxProgram,
    InstallRxProgram,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DataBits {
    Five = 5,
    Six = 6,
    Seven = 7,
    Eight = 8,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StopBits {
    One,
    Two,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Parity {
    None,
    Even,
    Odd,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ServiceMode {
    Polling,
    InterruptDriven,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct UartConfig {
    pub baud: u32,
    pub data_bits: DataBits,
    pub parity: Parity,
    pub stop_bits: StopBits,
    pub auto_echo: bool,
    pub service_mode: ServiceMode,
    pub tx_pipeline_chars: u32,
}

impl Default for UartConfig {
    fn default() -> Self {
        Self {
            baud: 115_200,
            data_bits: DataBits::Eight,
            parity: Parity::None,
            stop_bits: StopBits::One,
            auto_echo: false,
            service_mode: ServiceMode::Polling,
            tx_pipeline_chars: DEFAULT_TX_PIPELINE_CHARS,
        }
    }
}

impl UartConfig {
    #[inline]
    pub fn frame_bits(&self) -> u32 {
        let data_bits = self.data_bits as u32;
        let parity_bits = match self.parity {
            Parity::None => 0,
            Parity::Even | Parity::Odd => 1,
        };
        let stop_bits = match self.stop_bits {
            StopBits::One => 1,
            StopBits::Two => 2,
        };

        1 + data_bits + parity_bits + stop_bits
    }

    #[inline]
    pub fn is_supported_by_current_backend(&self) -> bool {
        self.baud != 0
            && self.data_bits == DataBits::Eight
            && self.parity == Parity::None
            && self.stop_bits == StopBits::One
    }
}

pub struct RpPioSerial<
    P: PIOExt,
    SMTX: StateMachineIndex,
    SMRX: StateMachineIndex,
    const TX_BUF: usize = DEFAULT_TX_BUF_SIZE,
    const RX_BUF: usize = DEFAULT_RX_BUF_SIZE,
> {
    _sm_tx: StateMachine<(P, SMTX), Running>,
    _sm_rx: StateMachine<(P, SMRX), Running>,
    tx: Tx<(P, SMTX)>,
    rx: Rx<(P, SMRX)>,

    tx_buf: Queue<u8, TX_BUF>,
    rx_buf: Queue<u8, RX_BUF>,

    config: UartConfig,
    clock_hz: u32,
    char_delay_cycles: u32,

    dropped_tx: u32,
    dropped_rx: u32,

    /// 只要写入过硬件 TX FIFO，就置 true。
    /// flush_blocking / write_all_blocking 在等待尾部清空后再清零。
    hw_drain_pending: bool,
}

impl<
        P: PIOExt,
        SMTX: StateMachineIndex,
        SMRX: StateMachineIndex,
        const TX_BUF: usize,
        const RX_BUF: usize,
    > RpPioSerial<P, SMTX, SMRX, TX_BUF, RX_BUF>
{
    /// 创建一个基于任意 PIO/任意状态机组合的 UART。
    ///
    /// 注意：
    /// - TX / RX 引脚应已切换到正确的 PIO function
    /// - tx_pin / rx_pin 是 GPIO 号
    /// - sm_tx / sm_rx 必须来自同一个 PIO block
    /// - 当前稳定支持配置：8N1
    pub fn new(
        pio: &mut PIO<P>,
        sm_tx: UninitStateMachine<(P, SMTX)>,
        sm_rx: UninitStateMachine<(P, SMRX)>,
        tx_pin: u8,
        rx_pin: u8,
        clock_hz: u32,
        config: UartConfig,
    ) -> Result<Self, InitError> {
        if clock_hz == 0 || config.baud == 0 {
            return Err(InitError::InvalidBaud);
        }

        if !config.is_supported_by_current_backend() {
            return Err(InitError::UnsupportedConfig);
        }

        let (div_int, div_frac) = calc_clkdiv(clock_hz, config.baud)?;
        let char_delay_cycles = calc_char_delay_cycles(clock_hz, config.frame_bits(), config.baud);

        // =========================================================
        // TX program (8N1)
        // 8 cycles / bit
        // =========================================================
        
        //this version is ok for tx
        let tx_program = pio_asm!(
            ".side_set 1 opt"
            "pull       side 1 [7]"
            "set x, 7   side 0 [7]"
            "bitloop:"
            "out pins, 1"
            "jmp x-- bitloop [6]"
        );
        
        let installed_tx = pio
            .install(&tx_program.program)
            .map_err(|_| InitError::InstallTxProgram)?;

        let (mut sm_tx, _, tx) = PIOBuilder::from_installed_program(installed_tx)
            .out_pins(tx_pin, 1)
            .side_set_pin_base(tx_pin)
            .clock_divisor_fixed_point(div_int, div_frac)
            .out_shift_direction(ShiftDirection::Right)
            .autopull(false)
            //.pull_threshold(8u8)
            .build(sm_tx);

        sm_tx.set_pindirs([(tx_pin, PinDir::Output)]);
        let sm_tx = sm_tx.start();

        // =========================================================
        // RX program (8N1)
        // 8 cycles / bit
        // =========================================================
        let rx_program = pio_asm!(
            "idle_wait:"
            "wait 1 pin 0"      // 要求空闲为高，避免假 start
        
            "start_wait:"
            "wait 0 pin 0"      // start: 下降沿
        
            "set x, 7 [10]"     // 对齐到第1个数据位中心附近（按你现有位周期调参点）
        
            "bitloop:"
            "in pins, 1"
            "jmp x-- bitloop [6]"
        
            "push"
            "jmp idle_wait"
        );
        let installed_rx = pio
            .install(&rx_program.program)
            .map_err(|_| InitError::InstallRxProgram)?;

        let (mut sm_rx, rx, _) = PIOBuilder::from_installed_program(installed_rx)
            .in_pin_base(rx_pin)
            .clock_divisor_fixed_point(div_int, div_frac)
            .in_shift_direction(ShiftDirection::Left)
            .autopush(false)
            .build(sm_rx);

        sm_rx.set_pindirs([(rx_pin, PinDir::Input)]);
        let sm_rx = sm_rx.start();

        Ok(Self {
            _sm_tx: sm_tx,
            _sm_rx: sm_rx,
            tx,
            rx,
            tx_buf: Queue::new(),
            rx_buf: Queue::new(),
            config,
            clock_hz,
            char_delay_cycles,
            dropped_tx: 0,
            dropped_rx: 0,
            hw_drain_pending: false,
        })
    }

    // =========================================================
    // 配置/状态查询
    // =========================================================

    pub fn config(&self) -> UartConfig {
        self.config
    }

    pub fn baud(&self) -> u32 {
        self.config.baud
    }

    pub fn clock_hz(&self) -> u32 {
        self.clock_hz
    }

    pub fn data_bits(&self) -> DataBits {
        self.config.data_bits
    }

    pub fn parity(&self) -> Parity {
        self.config.parity
    }

    pub fn stop_bits(&self) -> StopBits {
        self.config.stop_bits
    }

    pub fn auto_echo(&self) -> bool {
        self.config.auto_echo
    }

    pub fn service_mode(&self) -> ServiceMode {
        self.config.service_mode
    }

    pub fn set_auto_echo(&mut self, enable: bool) {
        self.config.auto_echo = enable;
    }

    pub fn set_service_mode(&mut self, mode: ServiceMode) {
        self.config.service_mode = mode;
    }

    pub fn dropped_tx(&self) -> u32 {
        self.dropped_tx
    }

    pub fn dropped_rx(&self) -> u32 {
        self.dropped_rx
    }

    pub fn available(&self) -> usize {
        self.rx_buf.len()
    }

    pub fn tx_pending(&self) -> usize {
        self.tx_buf.len()
    }

    pub fn clear_rx(&mut self) {
        while self.rx_buf.dequeue().is_some() {}
    }

    pub fn clear_tx_buffer(&mut self) {
        while self.tx_buf.dequeue().is_some() {}
    }

    // =========================================================
    // Poll / IRQ service
    // =========================================================

    /// 轮询服务入口
    pub fn poll(&mut self) {
        self.pump_rx();
        self.flush_tx_nonblocking();
    }

    /// 中断服务入口
    ///
    /// 在 PIO IRQ handler 中调用即可。
    pub fn on_interrupt(&mut self) {
        self.pump_rx();
        self.flush_tx_nonblocking();
    }

    /// 当使用“中断驱动”时，可在写入数据后主动 kick 一次 TX
    pub fn kick_tx(&mut self) {
        self.flush_tx_nonblocking();
    }

    // =========================================================
    // 非阻塞发送
    // =========================================================

    /// 非阻塞写入。
    ///
    /// 返回成功写入软件 TX 缓冲的字节数。
    pub fn write(&mut self, data: &[u8]) -> usize {
        let n = self.enqueue_tx(data);
        self.flush_tx_nonblocking();
        n
    }

    pub fn write_byte(&mut self, b: u8) -> bool {
        self.write(&[b]) == 1
    }

    pub fn print(&mut self, s: &str) -> usize {
        self.write(s.as_bytes())
    }

    pub fn println(&mut self, s: &str) -> usize {
        let mut n = 0;
        n += self.write(s.as_bytes());
        n += self.write(b"\r\n");
        n
    }

    // =========================================================
    // 强阻塞发送
    // =========================================================

    /// 阻塞直到软件 TX 缓冲中的数据全部进入硬件，并等待尾部发完。
    pub fn flush_blocking(&mut self) {
        while let Some(b) = self.tx_buf.dequeue() {
            while self.tx.is_full() {
                spin_loop();
            }
            self.tx.write(u32::from(b));
            self.hw_drain_pending = true;
        }

        if self.hw_drain_pending {
            self.wait_chars(self.config.tx_pipeline_chars.max(1));
            self.hw_drain_pending = false;
        }
    }

    /// 更强的阻塞发送：
    /// - 不依赖后续 `poll()`
    /// - 不依赖 tx_buf 能否装下整段数据
    /// - 会先清空之前的发送，再直接流式喂硬件 FIFO
    pub fn write_all_blocking(&mut self, data: &[u8]) {
        self.flush_blocking();

        for &b in data {
            while self.tx.is_full() {
                spin_loop();
            }
            self.tx.write(u32::from(b));
            self.hw_drain_pending = true;
        }

        if self.hw_drain_pending {
            self.wait_chars(self.config.tx_pipeline_chars.max(1));
            self.hw_drain_pending = false;
        }
    }

    pub fn write_byte_blocking(&mut self, b: u8) {
        self.write_all_blocking(&[b]);
    }

    pub fn print_blocking(&mut self, s: &str) {
        self.write_all_blocking(s.as_bytes());
    }

    pub fn println_blocking(&mut self, s: &str) {
        self.write_all_blocking(s.as_bytes());
        self.write_all_blocking(b"\r\n");
    }

    /// 阻塞重复发送单字节
    pub fn write_repeated_blocking(&mut self, byte: u8, count: usize) {
        self.flush_blocking();

        for _ in 0..count {
            while self.tx.is_full() {
                spin_loop();
            }
            self.tx.write(u32::from(byte));
            self.hw_drain_pending = true;
        }

        if self.hw_drain_pending {
            self.wait_chars(self.config.tx_pipeline_chars.max(1));
            self.hw_drain_pending = false;
        }
    }

    /// 阻塞格式化输出
    pub fn fmt_write_blocking(&mut self, args: fmt::Arguments<'_>) -> fmt::Result {
        struct BlockingFmt<
            'a,
            P: PIOExt,
            SMTX: StateMachineIndex,
            SMRX: StateMachineIndex,
            const TX_BUF: usize,
            const RX_BUF: usize,
        > {
            serial: &'a mut RpPioSerial<P, SMTX, SMRX, TX_BUF, RX_BUF>,
        }

        impl<
                'a,
                P: PIOExt,
                SMTX: StateMachineIndex,
                SMRX: StateMachineIndex,
                const TX_BUF: usize,
                const RX_BUF: usize,
            > FmtWrite for BlockingFmt<'a, P, SMTX, SMRX, TX_BUF, RX_BUF>
        {
            fn write_str(&mut self, s: &str) -> fmt::Result {
                self.serial.write_all_blocking(s.as_bytes());
                Ok(())
            }
        }

        let mut w = BlockingFmt { serial: self };
        w.write_fmt(args)
    }

    // =========================================================
    // 接收
    // =========================================================

    /// 非阻塞读取 RX 缓冲
    pub fn read(&mut self, buf: &mut [u8]) -> usize {
        self.pump_rx();

        let mut n = 0;
        while n < buf.len() {
            match self.rx_buf.dequeue() {
                Some(b) => {
                    buf[n] = b;
                    n += 1;
                }
                None => break,
            }
        }
        n
    }

    pub fn read_byte(&mut self) -> Option<u8> {
        self.pump_rx();
        self.rx_buf.dequeue()
    }

    /// 阻塞读取 1 字节
    pub fn read_byte_blocking(&mut self) -> u8 {
        loop {
            self.pump_rx();
            if let Some(b) = self.rx_buf.dequeue() {
                return b;
            }
            spin_loop();
        }
    }

    /// 阻塞读取恰好 buf.len() 个字节
    pub fn read_exact_blocking(&mut self, buf: &mut [u8]) {
        for b in buf {
            *b = self.read_byte_blocking();
        }
    }

    // =========================================================
    // 内部实现
    // =========================================================

    fn enqueue_tx(&mut self, data: &[u8]) -> usize {
        let mut n = 0;
        for &b in data {
            if self.tx_buf.enqueue(b).is_ok() {
                n += 1;
            } else {
                self.dropped_tx = self.dropped_tx.wrapping_add(1);
                break;
            }
        }
        n
    }

    fn pump_rx(&mut self) {
        while let Some(word) = self.rx.read() {
            let b = ((word & 0xFF) as u8).reverse_bits();
            //let b = ((word >> 24) & 0xFF) as u8;

            if self.rx_buf.enqueue(b).is_err() {
                self.dropped_rx = self.dropped_rx.wrapping_add(1);
            }

            if self.config.auto_echo {
                if self.tx_buf.enqueue(b).is_err() {
                    self.dropped_tx = self.dropped_tx.wrapping_add(1);
                }
            }
        }
    }

    fn flush_tx_nonblocking(&mut self) {
        while !self.tx.is_full() {
            let Some(b) = self.tx_buf.dequeue() else {
                break;
            };
            self.tx.write(u32::from(b));
            self.hw_drain_pending = true;
        }
    }

    #[inline]
    fn wait_chars(&self, chars: u32) {
        let cycles = self.char_delay_cycles.saturating_mul(chars.max(1));
        asm::delay(cycles);
    }
}

// =========================================================
// fmt::Write（默认走非阻塞发送）
// =========================================================

impl<
        P: PIOExt,
        SMTX: StateMachineIndex,
        SMRX: StateMachineIndex,
        const TX_BUF: usize,
        const RX_BUF: usize,
    > FmtWrite for RpPioSerial<P, SMTX, SMRX, TX_BUF, RX_BUF>
{
    fn write_str(&mut self, s: &str) -> fmt::Result {
        let _ = self.write(s.as_bytes());
        Ok(())
    }
}

// =========================================================
// 打印宏
// =========================================================

#[macro_export]
macro_rules! pio_print {
    ($serial:expr, $($tt:tt)*) => {{
        let _ = core::fmt::Write::write_fmt(&mut $serial, format_args!($($tt)*));
    }};
}

#[macro_export]
macro_rules! pio_println {
    ($serial:expr) => {{
        let _ = $serial.write(b"\r\n");
    }};
    ($serial:expr, $($tt:tt)*) => {{
        let _ = core::fmt::Write::write_fmt(&mut $serial, format_args!($($tt)*));
        let _ = $serial.write(b"\r\n");
    }};
}

/// 阻塞打印：整句输出不依赖后续 poll()
#[macro_export]
macro_rules! pio_bprint {
    ($serial:expr, $($tt:tt)*) => {{
        let _ = $serial.fmt_write_blocking(format_args!($($tt)*));
    }};
}

#[macro_export]
macro_rules! pio_bprintln {
    ($serial:expr) => {{
        $serial.write_all_blocking(b"\r\n");
    }};
    ($serial:expr, $($tt:tt)*) => {{
        let _ = $serial.fmt_write_blocking(format_args!($($tt)*));
        $serial.write_all_blocking(b"\r\n");
    }};
}

// =========================================================
// 工具函数
// =========================================================

fn calc_clkdiv(clock_hz: u32, baud: u32) -> Result<(u16, u8), InitError> {
    if baud == 0 {
        return Err(InitError::InvalidBaud);
    }

    // 当前 PIO UART 程序按 8 cycles / bit
    let denom = (baud as u64) * 8;
    if denom == 0 {
        return Err(InitError::InvalidBaud);
    }

    let div = ((clock_hz as u64) << 8) / denom;
    let int = (div >> 8) as u16;
    let frac = (div & 0xFF) as u8;

    if int == 0 && frac == 0 {
        return Err(InitError::InvalidBaud);
    }

    Ok((int, frac))
}

fn calc_char_delay_cycles(clock_hz: u32, frame_bits: u32, baud: u32) -> u32 {
    let cycles = (clock_hz as u64).saturating_mul(frame_bits.max(1) as u64) / (baud.max(1) as u64);

    cycles.max(1).min(u32::MAX as u64) as u32
}
