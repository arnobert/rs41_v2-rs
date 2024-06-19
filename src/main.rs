#![no_std]
#![no_main]


use embedded_hal::spi::{Mode, Phase, Polarity};
use panic_halt as _;

use systick_monotonic::{fugit::Duration, Systick};
use stm32l4xx_hal::{
    adc::*,
    gpio::{gpioa::*, gpiob::*, gpioc::*,
           Input, Alternate, Floating,
           Output, PinState, PushPull},
    pac,
    prelude::*,
    rcc::{ClockSecuritySystem, CrystalBypass, MsiFreq},
    timer::Timer,
    serial::{Config, Serial},
    spi::Spi,
};

pub const SPIMODE: Mode = Mode {
    phase: Phase::CaptureOnSecondTransition,
    polarity: Polarity::IdleHigh,
};
// USER CONFIG -------------------------------------------------------------------------------------
// Carrier freq in MHz
const CAR_FREQ: f32 = 432.2;

const HBSEL: bool = true;
const HBSEL_F32: f32 = if HBSEL {1.0} else {0.0};


const FREQBAND: u8 = if CAR_FREQ < 433.0 {0} else {1};

const FC: f32 = (CAR_FREQ / ((26.0/3.0) * (HBSEL_F32 + 1.0)) - FREQBAND as f32 - 24.0)  * 64000.0;
const FCU: u32 = FC as u32;

const F_C_UPPER: u8 = ((FCU & 0xFF00) >> 8) as u8;
const F_C_LOWER: u8 = (FCU & 0x00FF) as u8;

const GFSK_DATA_RATE: u16 = 0xB6D;

#[rtic::app(device = stm32l4xx_hal::stm32, peripherals = true)]
mod app {
    use stm32l4xx_hal::rcc::{APB1R1, ClockSecuritySystem, CrystalBypass};
    use systick_monotonic::fugit::HertzU32;
    use super::*;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {}

    #[monotonic(binds = SysTick, default = true)]
    type MonoTimer = Systick<1000>;

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        /*
        fn init_profile(device: &pac::Peripherals) {
            // On development, keep the DBG module powered on during wfi()
            device.DBGMCU.cr.modify(|_, w| w.dbg_standby().set_bit());
            device.DBGMCU.cr.modify(|_, w| w.dbg_sleep().set_bit());
            device.DBGMCU.cr.modify(|_, w| w.dbg_stop().set_bit());
        }
        init_profile(&cx.device);
         */

        // Pend interrupts during init -------------------------------------------------------------
        rtic::pend(stm32l4xx_hal::pac::interrupt::TIM2);

        // Peripherals -----------------------------------------------------------------------------
        let mut flash = cx.device.FLASH.constrain();
        let mut rcc = cx.device.RCC.constrain();
        let mut pwr = cx.device.PWR.constrain(&mut rcc.apb1r1);

        let mono = Systick::new(cx.core.SYST, 24_000_000);



        let clocks = rcc
            .cfgr
            .hse(24.MHz(), CrystalBypass::Disable, ClockSecuritySystem::Disable)
            .freeze(&mut flash.acr, &mut pwr);



        let mut gpioa = cx.device.GPIOA.split(&mut rcc.ahb2);
        let mut gpiob = cx.device.GPIOB.split(&mut rcc.ahb2);
        let mut gpioc = cx.device.GPIOC.split(&mut rcc.ahb2);

        //let channels = cx.device.DMA1.split(&mut rcc.ahb1);

        //PC8 LED

        let mut led_r = gpioc.pc8.into_open_drain_output(&mut gpioc.moder, &mut gpioc.otyper);
        let mut led_g = gpioc.pc7.into_open_drain_output(&mut gpioc.moder, &mut gpioc.otyper);



        // Disable JTAG ----------------------------------------------------------------------------
        //let (mut pa15, pb3, pb4) = afio.mapr.disable_jtag(gpioa.pa15, gpiob.pb3, gpiob.pb4);

        // RADIO: SPI2
        let mut spi2_sck = gpiob.pb13.into_alternate(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrh);
        let mut spi2_do = gpiob.pb15.into_alternate(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrh);
        let mut spi2_di = gpiob.pb14.into_alternate(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrh);

        let mut spi2_cs = gpioc.pc13.into_push_pull_output(&mut gpioc.moder, &mut gpioc.otyper);

        let mut rs_spi = Spi::spi2(
            cx.device.SPI2,
            (spi2_sck, spi2_di, spi2_do),
            SPIMODE,
            1.MHz(),
            clocks,
            &mut rcc.apb1r1
        );


        // -- Init radio ---
        let mut radio = si4032_driver::Si4032::new(rs_spi, spi2_cs);

        radio.swreset();

        while!(radio.chip_ready()){};

        radio.set_hb_sel(HBSEL);
        radio.set_freq_band(FREQBAND);
        radio.set_freq(F_C_UPPER, F_C_LOWER);
        radio.set_tx_pwr(si4032_driver::ETxPower::P1dBm);
        radio.set_modulation_type(si4032_driver::ModType::GFSK);
        radio.set_freq_deviation(0x0A);
        //radio.set_freq_offset(0x002);
        radio.set_trxdrtscale(true);
        radio.set_data_rate(GFSK_DATA_RATE);

        radio.set_auto_packet_handler(true);
        radio.set_modulation_source(si4032_driver::ModDataSrc::Fifo);

        // Preamble
        radio.set_tx_prealen(0x0E);

        // Sync Word
        // F8D8 = 11100110 11011000
        radio.set_sync_wrd(0x4242 << 16);

        // 00 -> Sync Word 3
        // 01 -> Sync Word 3, 2
        radio.set_tx_sync_len(0x01);


        // TX Header
        radio.set_tx_header_len(0);


        // Packet Length
        radio.set_packet_len(16);
        radio.set_tx_fixplen(false);


        radio.enter_tx();
        // --- END RADIO CFG ---


        led_r.set_high();
        led_g.set_low();


        


        // End init --------------------------------------------------------------------------------
        (
            Shared {},
            Local {},
            init::Monotonics(mono),
        )
    }


    #[idle()]
    fn idle(_cx: idle::Context) -> ! {
        loop {
            // DO NOT UNCOMMENT UNLESS YOU WANT TO LIFT THE BOOT0 PIN
            //cortex_m::asm::wfi();
            cortex_m::asm::delay(100);
        }
    }

}
