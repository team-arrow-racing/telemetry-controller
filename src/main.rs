//! Telemetry Transponder
//!
//! # Task priority assignment
//!
//! It would be more idiomatic to have these assigned in a enum or some constants
//! but RTIC doesn't yet support variables (static or otherwise) in task
//! definitions.
//!
//! | Priority | Use |
//! | --- | --- |
//! | 0 | `idle` task and background tasks. |
//! | 1 (default) | General and asynchronous tasks. |
//! | 2 | Synchronous comms tasks. |
//! | 3 | System critical tasks. |

#![no_main]
#![no_std]

use defmt_rtt as _;
use panic_probe as _;

use at_commands::builder::CommandBuilder;
use at_commands::parser::CommandParser;

use heapless::{
    pool,
    pool::singleton::{Box, Pool},
};

// The pool gives out `Box<DMAFrame>`s that can hold 8 bytes
pool!(
    #[allow(non_upper_case_globals)]
    SerialDMAPool: DMAFrame<32>
);

use bxcan::{self, filter::Mask32, Frame, Id, Interrupts};

use dwt_systick_monotonic::{fugit, DwtSystick};

use embedded_hal::spi::{Mode, Phase, Polarity};

use embedded_sdmmc::{SdCard, TimeSource, Timestamp};

use solar_car::{
    com, device, j1939,
    j1939::pgn::{Number, Pgn},
};
use stm32l4xx_hal::{
    can::Can,
    delay::DelayCM,
    device::CAN1,
    dma::{self, DMAFrame, FrameReader, FrameSender, RxDma, TxDma},
    flash::FlashExt,
    gpio::{
        Alternate, OpenDrain, Output, PushPull, Speed, PA11, PA12, PA4, PA5,
        PA6, PA7, PB13, PB8, PB9, PC12, PC8, PC9, PD2,
    },
    pac::{SDMMC, SPI1, SPI2, SPI3, USART2},
    prelude::*,
    serial::{self, Config, Rx, Serial, Tx},
    spi::Spi,
    watchdog::IndependentWatchdog,
};

const FILE_TO_WRITE: &str = "LOGS.TXT";
const DEVICE: device::Device = device::Device::SteeringWheel;
const SYSCLK: u32 = 80_000_000;

/// SPI mode
pub const MODE: Mode = Mode {
    phase: Phase::CaptureOnFirstTransition,
    polarity: Polarity::IdleLow,
};

use core::marker::PhantomData;

struct TimeSink {
    _marker: PhantomData<*const ()>,
}

impl TimeSink {
    fn new() -> Self {
        TimeSink {
            _marker: PhantomData,
        }
    }
}

impl TimeSource for TimeSink {
    fn get_timestamp(&self) -> Timestamp {
        Timestamp {
            year_since_1970: 0,
            zero_indexed_month: 0,
            zero_indexed_day: 0,
            hours: 0,
            minutes: 0,
            seconds: 0,
        }
    }
}
// This is for formatting strings
pub mod write_to {
    use core::cmp::min;
    use core::fmt;

    pub struct WriteTo<'a> {
        buffer: &'a mut [u8],
        // on write error (i.e. not enough space in buffer) this grows beyond
        // `buffer.len()`.
        used: usize,
    }

    impl<'a> WriteTo<'a> {
        pub fn new(buffer: &'a mut [u8]) -> Self {
            WriteTo { buffer, used: 0 }
        }

        pub fn as_str(self) -> Option<&'a str> {
            if self.used <= self.buffer.len() {
                // only successful concats of str - must be a valid str.
                use core::str::from_utf8_unchecked;
                Some(unsafe { from_utf8_unchecked(&self.buffer[..self.used]) })
            } else {
                None
            }
        }
    }

    impl<'a> fmt::Write for WriteTo<'a> {
        fn write_str(&mut self, s: &str) -> fmt::Result {
            if self.used > self.buffer.len() {
                return Err(fmt::Error);
            }
            let remaining_buf = &mut self.buffer[self.used..];
            let raw_s = s.as_bytes();
            let write_num = min(raw_s.len(), remaining_buf.len());
            remaining_buf[..write_num].copy_from_slice(&raw_s[..write_num]);
            self.used += raw_s.len();
            if write_num < raw_s.len() {
                Err(fmt::Error)
            } else {
                Ok(())
            }
        }
    }

    pub fn show<'a>(
        buffer: &'a mut [u8],
        args: fmt::Arguments,
    ) -> Result<&'a str, fmt::Error> {
        let mut w = WriteTo::new(buffer);
        fmt::write(&mut w, args)?;
        w.as_str().ok_or(fmt::Error)
    }
}

#[rtic::app(device = stm32l4xx_hal::pac, dispatchers = [SPI1, SPI2, SPI3, QUADSPI])]
mod app {
    use bxcan::{filter::Mask32, Interrupts};

    use super::*;

    #[monotonic(binds = SysTick, default = true)]
    type MonoTimer = DwtSystick<SYSCLK>;
    pub type Duration = fugit::TimerDuration<u64, SYSCLK>;
    pub type Instant = fugit::TimerInstant<u64, SYSCLK>;

    #[shared]
    struct Shared {
        can: bxcan::Can<
            Can<
                CAN1,
                (PB9<Alternate<PushPull, 9>>, PB8<Alternate<PushPull, 9>>),
            >,
        >,
        frame_reader: FrameReader<
            Box<SerialDMAPool>,
            RxDma<Rx<USART2>, dma::dma1::C6>,
            32,
        >,
        frame_sender: FrameSender<
            Box<SerialDMAPool>,
            TxDma<Tx<USART2>, dma::dma1::C7>,
            32,
        >,
        send_ready: bool,
        delay: DelayCM,
        can_packet_buf: [u8; 128],
    }

    #[local]
    struct Local {
        watchdog: IndependentWatchdog,
        status_led: PB13<Output<PushPull>>,

        spi_dev: SdCard<
            Spi<
                SPI1,
                (
                    PA5<Alternate<PushPull, 5>>,
                    PA6<Alternate<PushPull, 5>>,
                    PA7<Alternate<PushPull, 5>>,
                ),
            >,
            PA4<Output<OpenDrain>>,
            DelayCM,
        >,
    }

    #[init]
    fn init(mut cx: init::Context) -> (Shared, Local, init::Monotonics) {
        defmt::trace!("task: init");

        static mut MEMORY: [u8; 1024] = [0; 1024];

        // increase the capacity of the pool by ~8 blocks
        unsafe { SerialDMAPool::grow(&mut MEMORY) };

        // peripherals
        let mut flash = cx.device.FLASH.constrain();
        let mut rcc = cx.device.RCC.constrain();
        let mut pwr = cx.device.PWR.constrain(&mut rcc.apb1r1);
        let mut gpioa = cx.device.GPIOA.split(&mut rcc.ahb2);
        let mut gpiob = cx.device.GPIOB.split(&mut rcc.ahb2);
        let mut gpioc = cx.device.GPIOC.split(&mut rcc.ahb2);
        let mut gpiod = cx.device.GPIOD.split(&mut rcc.ahb2);

        // configure system clock
        let clocks = rcc.cfgr.sysclk(80.MHz()).freeze(&mut flash.acr, &mut pwr);

        // configure monotonic time
        let mono = DwtSystick::new(
            &mut cx.core.DCB,
            cx.core.DWT,
            cx.core.SYST,
            clocks.sysclk().to_Hz(),
        );

        // let timer2 = cx.device.TIM2.timer(1.kHz(), device.peripheral.TIM2, &mut device.clocks);
        let mut delay = DelayCM::new(clocks);
        // delay.delay_ms(500_u32);

        // configure status led
        let status_led = gpiob
            .pb13
            .into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper);

        // configure can bus
        let can = {
            // let rx = gpioa.pa11.into_alternate(
            //     &mut gpioa.moder,
            //     &mut gpioa.otyper,
            //     &mut gpioa.afrh,
            // );
            // let tx = gpioa.pa12.into_alternate(
            //     &mut gpioa.moder,
            //     &mut gpioa.otyper,
            //     &mut gpioa.afrh,
            // );

            let rx = gpiob.pb8.into_alternate(
                &mut gpiob.moder,
                &mut gpiob.otyper,
                &mut gpiob.afrh,
            );
            let tx = gpiob.pb9.into_alternate(
                &mut gpiob.moder,
                &mut gpiob.otyper,
                &mut gpiob.afrh,
            );

            let can = bxcan::Can::builder(Can::new(
                &mut rcc.apb1r1,
                cx.device.CAN1,
                (tx, rx),
            ))
            .set_bit_timing(0x001c_0009); // 500kbit/s

            let mut can = can.enable();

            // configure filters
            can.modify_filters().enable_bank(0, Mask32::accept_all());

            // configure interrupts
            can.enable_interrupts(
                Interrupts::TRANSMIT_MAILBOX_EMPTY
                    | Interrupts::FIFO0_MESSAGE_PENDING
                    | Interrupts::FIFO1_MESSAGE_PENDING,
            );
            nb::block!(can.enable_non_blocking()).unwrap();

            // broadcast startup message.
            can.transmit(&com::startup::message(DEVICE)).unwrap();

            can
        };

        // Configure SPI
        let sck = gpioa.pa5.into_alternate(
            &mut gpioa.moder,
            &mut gpioa.otyper,
            &mut gpioa.afrl,
        );

        let miso = gpioa.pa6.into_alternate(
            &mut gpioa.moder,
            &mut gpioa.otyper,
            &mut gpioa.afrl,
        );

        let mosi = gpioa.pa7.into_alternate(
            &mut gpioa.moder,
            &mut gpioa.otyper,
            &mut gpioa.afrl,
        );

        let spi = Spi::spi1(
            cx.device.SPI1,
            (sck, miso, mosi),
            MODE,
            16.MHz(),
            clocks,
            &mut rcc.apb2,
        );

        let spi_cs_pin = gpioa
            .pa4
            .into_open_drain_output(&mut gpioa.moder, &mut gpioa.otyper);

        let spi_dev = SdCard::new(spi, spi_cs_pin, delay);

        // Configure UART
        let mut uart = {
            let tx = gpioa.pa2.into_alternate(
                &mut gpioa.moder,
                &mut gpioa.otyper,
                &mut gpioa.afrl,
            );

            let rx = gpioa.pa3.into_alternate(
                &mut gpioa.moder,
                &mut gpioa.otyper,
                &mut gpioa.afrl,
            );

            Serial::usart2(
                cx.device.USART2,
                (tx, rx),
                Config::default()
                    .baudrate(921_600.bps())
                    .parity_even()
                    .character_match(b'\n'),
                clocks,
                &mut rcc.apb1r1,
            )
        };

        uart.listen(serial::Event::CharacterMatch);
        let (mut tx, mut rx) = uart.split();

        let watchdog = {
            let mut wd = IndependentWatchdog::new(cx.device.IWDG);
            wd.stop_on_debug(&cx.device.DBGMCU, true);
            wd.start(fugit::MillisDurationU32::millis(100));

            wd
        };

        let channels = cx.device.DMA1.split(&mut rcc.ahb1);
        let mut dma_ch6 = channels.6;
        let mut dma_ch7 = channels.7;
        dma_ch6.listen(dma::Event::TransferComplete);
        dma_ch7.listen(dma::Event::TransferComplete);

        // Serial frame reader (DMA based), give it a buffer to start reading into
        let frame_reader = if let Some(dma_buf) = SerialDMAPool::alloc() {
            // Set up the first reader frame
            let dma_buf = dma_buf.init(DMAFrame::new());
            rx.with_dma(dma_ch6).frame_reader(dma_buf)
        } else {
            unreachable!()
        };

        let mut send_ready = true;
        // Serial frame sender (DMA based)
        let mut frame_sender: FrameSender<Box<SerialDMAPool>, _, 32> =
            tx.with_dma(dma_ch7).frame_sender();

        let can_packet_buf = [0u8; 128];
        // Send initial packet to calypso to get it into AT command mode
        // The calypso will send an error message back which can be ignored
        send_frame(b"AT+test\r\n", &mut send_ready, &mut frame_sender);

        // start main loop
        run::spawn().unwrap();
        // wait for bit for calypso to boot up
        // write_dma_frames::spawn_after(Duration::millis(2000)).unwrap();

        (
            Shared {
                can,
                frame_reader,
                frame_sender,
                send_ready,
                delay,
                can_packet_buf,
            },
            Local {
                watchdog,
                status_led,
                spi_dev,
            },
            init::Monotonics(mono),
        )
    }

    #[task(priority = 1, local = [watchdog])]
    fn run(cx: run::Context) {
        defmt::trace!("task: run");

        cx.local.watchdog.feed();

        run::spawn_after(Duration::millis(10)).unwrap();
    }

    /// This task handles the character match interrupt at required by the `FrameReader`
    #[task(binds = USART2, shared = [frame_reader, frame_sender, send_ready])]
    fn serial_isr(mut cx: serial_isr::Context) {
        // Check for character match
        cx.shared.frame_reader.lock(|fr| {
            if fr.check_character_match(true) {
                if let Some(dma_buf) = SerialDMAPool::alloc() {
                    let dma_buf = dma_buf.init(DMAFrame::new());
                    let buf = fr.character_match_interrupt(dma_buf);

                    let res = core::str::from_utf8(buf.read()).unwrap();
                    defmt::debug!("RESPONSE: {:?}", res);
                    // TODO this will be instructions given from Profinity
                    // Need to then forward the comms to the relevant device
                    cx.shared.send_ready.lock(|sr| {
                        *sr = true;
                    });
                }
            }
        });
    }

    /// This task handles the RX transfer complete interrupt at required by the `FrameReader`
    /// In this case we are discarding if a frame gets full as no character match was received
    #[task(binds = DMA1_CH6, shared = [frame_reader])]
    fn serial_rx_dma_isr(mut cx: serial_rx_dma_isr::Context) {
        if let Some(dma_buf) = SerialDMAPool::alloc() {
            let dma_buf = dma_buf.init(DMAFrame::new());

            // Erroneous packet as it did not fit in a buffer, throw away the buffer
            cx.shared.frame_reader.lock(|fr| {
                let buf = fr.transfer_complete_interrupt(dma_buf);
                let res = core::str::from_utf8(buf.read()).unwrap();
                defmt::debug!("Discarding: {:?}", res);
            });
        }
    }

    // This task handles the TX transfer complete interrupt at required by the `FrameSender`
    #[task(binds = DMA1_CH7, shared = [frame_sender, send_ready])]
    fn serial_tx_dma_isr(mut cx: serial_tx_dma_isr::Context) {
        cx.shared.frame_sender.lock(|fs| {
            if let Some(buf) = fs.transfer_complete_interrupt() {
                let res = core::str::from_utf8(buf.read()).unwrap();
                defmt::debug!("Sent: {:?}", res);
                // Frame sent, drop the buffer to return it too the pool
                cx.shared.send_ready.lock(|sr| {
                    *sr = true;
                });
            }
        });
    }

    #[task(shared = [frame_sender, send_ready, can_packet_buf])]
    fn calypso_write(mut cx: calypso_write::Context) {
        // let mut buffer = [0; 128];
        // TODO consider using at commands crate, but this does not append
        // string parameters properly
        // let cmd: &[u8; 21] = b"AT+send=0,0,5,hello\r\n";
        // let cmd = write_to::show(
        //     &mut buf,
        //     format_args!("AT+{:?}{:?}\r\n", id, frame),
        // ).unwrap();

        cx.shared.send_ready.lock(|sr| {
            cx.shared.frame_sender.lock(|fs| {
                cx.shared.can_packet_buf.lock(|buf| {
                    defmt::debug!(
                        "Writing {:?}",
                        core::str::from_utf8(buf).unwrap()
                    );
                    send_frame(buf, sr, fs);
                });
            });
        });
    }

    #[task(shared = [frame_sender, send_ready])]
    fn calypso_read(mut cx: calypso_read::Context) {
        // let mut buffer = [0; 128];
        // TODO consider using at commands crate, but this does not append
        // string parameters properly
        let cmd = b"AT+recv=1,0,32\r\n";

        defmt::debug!("Writing {:?}", core::str::from_utf8(cmd).unwrap());

        cx.shared.send_ready.lock(|sr| {
            cx.shared.frame_sender.lock(|fs| {
                send_frame(cmd, sr, fs);
                // We should receive OK followed by +recv:1,0,32,data, followed by another OK in UART
            });
        });

        calypso_read::spawn_after(Duration::millis(2000)).unwrap();
    }

    /// Triggers on RX mailbox event.
    #[task(priority = 1, shared = [can], binds = CAN1_RX0)]
    fn can_rx0_pending(_: can_rx0_pending::Context) {
        defmt::trace!("task: can rx0 pending");

        can_receive::spawn().unwrap();
    }

    /// Triggers on RX mailbox event.
    #[task(priority = 1, shared = [can], binds = CAN1_RX1)]
    fn can_rx1_pending(_: can_rx1_pending::Context) {
        defmt::trace!("task: can rx1 pending");

        can_receive::spawn().unwrap();
    }

    #[task(priority = 2, shared = [can, can_packet_buf])]
    fn can_receive(mut cx: can_receive::Context) {
        defmt::trace!("task: can receive");

        cx.shared.can.lock(|can| loop {
            let frame = match can.receive() {
                Ok(frame) => frame,
                Err(nb::Error::WouldBlock) => break, // done
                Err(nb::Error::Other(_)) => continue, // go to next frame
            };

            let id = match frame.id() {
                Id::Standard(_) => {
                    continue; // go to next frame
                }
                Id::Extended(id) => id,
            };
            let id: j1939::ExtendedId = id.into();

            match id.pgn {
                Pgn::Destination(pgn) => {
                    // Serialize can frame and send over Wifi
                    cx.shared.can_packet_buf.lock(|buf| {
                        let _can_packet = write_to::show(
                            buf,
                            // hardcode length for now but this may need to be calculated
                            format_args!(
                                "AT+send=0,0,23{:?}{:?}\r\n",
                                id.to_bits(),
                                frame.data().unwrap()
                            ),
                        )
                        .unwrap();
                        calypso_write::spawn().unwrap();
                    });
                }
                _ => {} // ignore broadcast messages
            }
        });
    }

    fn send_frame(
        data: &[u8],
        send_ready: &mut bool,
        frame_sender: &mut FrameSender<
            Box<SerialDMAPool>,
            TxDma<Tx<USART2>, dma::dma1::C7>,
            32,
        >,
    ) {
        // TODO there is probably a better way to handle this but I am too dum to figure it out rn
        defmt::debug!(
            "waiting to send {:?}",
            core::str::from_utf8(data).unwrap()
        );
        while !*send_ready {}

        *send_ready = false;

        if let Some(dma_buf) = SerialDMAPool::alloc() {
            let mut dma_buf = dma_buf.init(DMAFrame::new());
            let _buf_size = dma_buf.write_slice(data);
            frame_sender.send(dma_buf).unwrap();
        }
    }
}

// same panicking *behavior* as `panic-probe` but doesn't print a panic message
// this prevents the panic message being printed *twice* when `defmt::panic` is invoked
#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}

// Show a millisecond timestamp next to debug output.
// Unit conversion isn't required because ticks = milliseconds for our case.
defmt::timestamp!("time={=u64}ms", {
    app::monotonics::MonoTimer::now()
        .duration_since_epoch()
        .to_millis()
});
