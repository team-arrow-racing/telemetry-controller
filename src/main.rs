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

use dwt_systick_monotonic::{fugit, DwtSystick};
use solar_car::{com, device};
use stm32l4xx_hal::{
    can::Can,
    device::CAN1,
    flash::FlashExt,
    gpio::{Alternate, Output, PushPull, PA10, PA11, PA12, PA9, PB13},
    prelude::*,
    serial::{Config, Serial},
    watchdog::IndependentWatchdog,
};

const DEVICE: device::Device = device::Device::SteeringWheel;
const SYSCLK: u32 = 80_000_000;

#[rtic::app(device = stm32l4xx_hal::pac, dispatchers = [SPI1, SPI2, SPI3, QUADSPI])]
mod app {

    use bxcan::{filter::Mask32, Interrupts};
    use stm32l4xx_hal::device::USART1;

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
        uart: Serial<
            USART1,
            (PA9<Alternate<PushPull, 7>>, PA10<Alternate<PushPull, 7>>),
        >,
    }

    #[local]
    struct Local {
        watchdog: IndependentWatchdog,
        status_led: PB13<Output<PushPull>>,
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
            let tx = gpioa.pa9.into_alternate(
                &mut gpioa.moder,
                &mut gpioa.otyper,
                &mut gpioa.afrh,
            );

            let rx = gpioa.pa10.into_alternate(
                &mut gpioa.moder,
                &mut gpioa.otyper,
                &mut gpioa.afrh,
            );

            Serial::usart1(
                cx.device.USART1,
                (tx, rx),
                Config::default().baudrate(9_600.bps()),
                clocks,
                &mut rcc.apb2,
            )
        };

        let watchdog = {
            let mut wd = IndependentWatchdog::new(cx.device.IWDG);
            wd.stop_on_debug(&cx.device.DBGMCU, true);
            wd.start(fugit::MillisDurationU32::millis(100));

            wd
        };
        
        (
            Shared { can, uart },
            Local {
                watchdog,
                status_led,
            },
            init::Monotonics(mono),
        )
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
