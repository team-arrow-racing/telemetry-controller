//! Telemetry Transponder
//!
//! # Task priority assignment
//!
//! It would be more idomatic to have these assigned in a enum or some constants
//! but RTIC doesn't yet support variables (static or otherwise) in task
//! definitions.
//!
//! | Priority | Use |
//! | --- | --- |
//! | 0 | `idle` task and background tasks. |
//! | 1 (default) | General and asychronous tasks. |
//! | 2 | Synchronous comms tasks. |
//! | 3 | System critical tasks. |

#![no_main]
#![no_std]

use defmt_rtt as _;
use panic_probe as _;

#[macro_use(singleton)]
extern crate cortex_m;

use at_commands::builder::CommandBuilder;
use at_commands::parser::CommandParser;

use dwt_systick_monotonic::{fugit, DwtSystick};
use solar_car::{com, device};
use stm32l4xx_hal::{
    can::Can,
    device::CAN1,
    dma::{self, CircBuffer, CircReadDma, RxDma},
    flash::FlashExt,
    gpio::{Alternate, Output, PushPull, PA10, PA11, PA12, PA9, PB13},
    pac::{USART1, USART2},
    prelude::*,
    serial::{self, Config, Serial, Tx, Rx},
    watchdog::IndependentWatchdog,
};

use wurth_calypso::{
    Calypso
};

const DEVICE: device::Device = device::Device::SteeringWheel;
const SYSCLK: u32 = 80_000_000;

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
                (PA12<Alternate<PushPull, 9>>, PA11<Alternate<PushPull, 9>>),
            >,
        >,
        // uart: Serial<
        //     USART1,
        //     (PA9<Alternate<PushPull, 7>>, PA10<Alternate<PushPull, 7>>),
        // >,
        tx: Tx<USART2>,
        // rx: Rx<USART1>,
    }

    #[local]
    struct Local {
        watchdog: IndependentWatchdog,
        status_led: PB13<Output<PushPull>>,
        circ_buffer: CircBuffer<[u8; 32], RxDma<Rx<USART2>, dma::dma1::C6>>
    }

    #[init]
    fn init(mut cx: init::Context) -> (Shared, Local, init::Monotonics) {
        defmt::trace!("task: init");

        // peripherals
        let mut flash = cx.device.FLASH.constrain();
        let mut rcc = cx.device.RCC.constrain();
        let mut pwr = cx.device.PWR.constrain(&mut rcc.apb1r1);
        let mut gpioa = cx.device.GPIOA.split(&mut rcc.ahb2);
        let mut gpiob = cx.device.GPIOB.split(&mut rcc.ahb2);

        // configure system clock
        let clocks = rcc.cfgr.sysclk(80.MHz()).freeze(&mut flash.acr, &mut pwr);

        // configure monotonic time
        let mono = DwtSystick::new(
            &mut cx.core.DCB,
            cx.core.DWT,
            cx.core.SYST,
            clocks.sysclk().to_Hz(),
        );

        // configure status led
        let status_led = gpiob
            .pb13
            .into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper);

        // configure can bus
        let can = {
            let rx = gpioa.pa11.into_alternate(
                &mut gpioa.moder,
                &mut gpioa.otyper,
                &mut gpioa.afrh,
            );
            let tx = gpioa.pa12.into_alternate(
                &mut gpioa.moder,
                &mut gpioa.otyper,
                &mut gpioa.afrh,
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

        uart.listen(serial::Event::Rxne);

        let watchdog = {
            let mut wd = IndependentWatchdog::new(cx.device.IWDG);
            wd.stop_on_debug(&cx.device.DBGMCU, true);
            wd.start(fugit::MillisDurationU32::millis(100));

            wd
        };

        let (tx, rx) = uart.split();

        let channels = cx.device.DMA1.split(&mut rcc.ahb1);
        let buf = singleton!(: [u8; 32] = [0; 32]).unwrap();
        let circ_buffer = rx.with_dma(channels.6).circ_read(buf);

        // start main loop
        run::spawn().unwrap();
        calypso_comms::spawn().unwrap();
        
        (
            Shared { 
                can,
                tx,
                // rx
            },
            Local {
                watchdog,
                status_led,
                circ_buffer
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

    #[task(priority = 1, shared = [tx])]
    fn calypso_comms(mut cx: calypso_comms::Context) {
        let mut buffer = [0; 128];

        let command = CommandBuilder::create_execute(&mut buffer, true)
            .named("+test")
            .finish()
            .unwrap();

        defmt::debug!("Writing {:?}", core::str::from_utf8(command).unwrap());

        cx.shared.tx.lock(|tx| {
            let _ = command
                .iter()
                .map(|c| nb::block!(tx.write(*c)))
                .last();
        });

        calypso_comms::spawn_after(Duration::millis(2000)).unwrap();
    }

    #[task(binds = USART2, local = [circ_buffer])]
    fn uart_read(cx: uart_read::Context) {
        let mut rx_buf = [0; 32];

        let rx_res = cx.local.circ_buffer.read(&mut rx_buf);
        match rx_res {
            Err(_) => (),
            Ok(rx_len) => {
                defmt::debug!("{:?}", core::str::from_utf8(&rx_buf[..rx_len]).unwrap());
                // TODO for parsing AT commands
                // let a = CommandParser::parse(&rx_buf).finish().unwrap();
                // defmt::debug!("{:?}", a);
            }
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
