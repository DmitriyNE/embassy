use core::future::poll_fn;

use embassy_sync::waitqueue::AtomicWaker;
use embedded_hal_02::blocking::delay::DelayUs;
use pac::adc::regs::{Smpr, Sqr1};
use pac::adc::vals::Adstp;
#[allow(unused)]
use pac::adc::vals::{Difsel, Exten, Pcsel};
use pac::adccommon::vals::Presc;

use core::marker::PhantomData;
use core::task::Poll;

use super::{Adc, AdcPin, Instance, InternalChannel, Resolution, SampleTime};
use crate::interrupt::typelevel::Interrupt;
use crate::time::Hertz;
use crate::{dma, interrupt};
use crate::{pac, Peripheral};

static ADC_WAKER: AtomicWaker = AtomicWaker::new();

pub struct InterruptHandler<T: Instance> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> interrupt::typelevel::Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        let bits = T::regs().isr().read();
        if bits.eoc() {
            T::regs().ier().modify(|m| m.set_eocie(false));
            ADC_WAKER.wake();
        }
    }
}

/// Default VREF voltage used for sample conversion to millivolts.
pub const VREF_DEFAULT_MV: u32 = 1800;
/// VREF voltage used for factory calibration of VREFINTCAL register.
pub const VREF_CALIB_MV: u32 = 1800;

const MAX_ADC_CLK_FREQ: Hertz = Hertz::mhz(55);

const VBAT_CHANNEL: u8 = 18;
const VREF_CHANNEL: u8 = 0;
const TEMP_CHANNEL: u8 = 19;

// NOTE: Vrefint/Temperature/Vbat are not available on all ADCs, this currently cannot be modeled with stm32-data, so these are available from the software on all ADCs
/// Internal voltage reference channel.
pub struct VrefInt;
impl<T: Instance> InternalChannel<T> for VrefInt {}
impl<T: Instance> super::sealed::InternalChannel<T> for VrefInt {
    fn channel(&self) -> u8 {
        VREF_CHANNEL
    }
}

/// Internal temperature channel.
pub struct Temperature;
impl<T: Instance> InternalChannel<T> for Temperature {}
impl<T: Instance> super::sealed::InternalChannel<T> for Temperature {
    fn channel(&self) -> u8 {
        TEMP_CHANNEL
    }
}

/// Internal battery voltage channel.
pub struct Vbat;
impl<T: Instance> InternalChannel<T> for Vbat {}
impl<T: Instance> super::sealed::InternalChannel<T> for Vbat {
    fn channel(&self) -> u8 {
        VBAT_CHANNEL
    }
}

// NOTE (unused): The prescaler enum closely copies the hardware capabilities,
// but high prescaling doesn't make a lot of sense in the current implementation and is ommited.
#[allow(unused)]
enum Prescaler {
    NotDivided,
    DividedBy2,
    DividedBy4,
    DividedBy6,
    DividedBy8,
    DividedBy10,
    DividedBy12,
    DividedBy16,
    DividedBy32,
    DividedBy64,
    DividedBy128,
    DividedBy256,
}

impl Prescaler {
    fn from_ker_ck(frequency: Hertz) -> Self {
        let raw_prescaler = frequency.0 / MAX_ADC_CLK_FREQ.0;
        match raw_prescaler {
            0 => Self::NotDivided,
            1 => Self::DividedBy2,
            2..=3 => Self::DividedBy4,
            4..=5 => Self::DividedBy6,
            6..=7 => Self::DividedBy8,
            8..=9 => Self::DividedBy10,
            10..=11 => Self::DividedBy12,
            _ => unimplemented!(),
        }
    }

    fn divisor(&self) -> u32 {
        match self {
            Prescaler::NotDivided => 1,
            Prescaler::DividedBy2 => 2,
            Prescaler::DividedBy4 => 4,
            Prescaler::DividedBy6 => 6,
            Prescaler::DividedBy8 => 8,
            Prescaler::DividedBy10 => 10,
            Prescaler::DividedBy12 => 12,
            Prescaler::DividedBy16 => 16,
            Prescaler::DividedBy32 => 32,
            Prescaler::DividedBy64 => 64,
            Prescaler::DividedBy128 => 128,
            Prescaler::DividedBy256 => 256,
        }
    }

    fn presc(&self) -> Presc {
        match self {
            Prescaler::NotDivided => Presc::DIV1,
            Prescaler::DividedBy2 => Presc::DIV2,
            Prescaler::DividedBy4 => Presc::DIV4,
            Prescaler::DividedBy6 => Presc::DIV6,
            Prescaler::DividedBy8 => Presc::DIV8,
            Prescaler::DividedBy10 => Presc::DIV10,
            Prescaler::DividedBy12 => Presc::DIV12,
            Prescaler::DividedBy16 => Presc::DIV16,
            Prescaler::DividedBy32 => Presc::DIV32,
            Prescaler::DividedBy64 => Presc::DIV64,
            Prescaler::DividedBy128 => Presc::DIV128,
            Prescaler::DividedBy256 => Presc::DIV256,
        }
    }
}

impl<'d, T: Instance> Adc<'d, T> {
    /// Create a new ADC driver.
    pub fn new(
        adc: impl Peripheral<P = T> + 'd,
        delay: &mut impl DelayUs<u16>,
        _irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
    ) -> Self {
        embassy_hal_internal::into_ref!(adc);
        T::enable_and_reset();

        let prescaler = Prescaler::from_ker_ck(T::frequency());

        T::common_regs().ccr().modify(|w| w.set_presc(prescaler.presc()));

        let frequency = Hertz(T::frequency().0 / prescaler.divisor());
        info!("ADC frequency set to {} Hz", frequency.0);

        if frequency > MAX_ADC_CLK_FREQ {
            panic!("Maximal allowed frequency for the ADC is {} MHz and it varies with different packages, refer to ST docs for more information.", MAX_ADC_CLK_FREQ.0 /  1_000_000 );
        }

        let mut s = Self {
            adc,
            sample_time: Default::default(),
        };
        s.power_up(delay);
        s.configure_differential_inputs();

        s.calibrate();
        delay.delay_us(1);

        s.enable();
        s.configure();

        T::Interrupt::unpend();
        unsafe { T::Interrupt::enable() };

        s
    }

    fn power_up(&mut self, delay: &mut impl DelayUs<u16>) {
        // TODO: This does not belong here
        // Enable Vdda load switches
        pac::PWR.svmcr().modify(|w| w.set_asv(true));
        // Enable power to VREFBUF
        pac::RCC.apb3enr().modify(|w| w.set_vrefen(true));
        // Power up VREFBUF
        pac::VREFBUF.csr().write(|w| w.set_envr(true));

        T::regs().cr().modify(|reg| {
            reg.set_deeppwd(false);
            reg.set_advregen(true);
        });

        // delay.delay_us(10);
        while !T::regs().isr().read().ldordy() {}
    }

    fn configure_differential_inputs(&mut self) {
        T::regs().difsel().modify(|w| {
            for n in 0..20 {
                w.set_difsel(n, Difsel::SINGLEENDED);
            }
        });
    }

    fn calibrate(&mut self) {
        T::regs().cr().modify(|w| {
            w.set_adcallin(true);
        });

        T::regs().cr().modify(|w| w.set_adcal(true));

        while T::regs().cr().read().adcal() {}
    }

    fn enable(&mut self) {
        T::regs().isr().write(|w| w.set_adrdy(true));
        T::regs().cr().modify(|w| w.set_aden(true));
        while !T::regs().isr().read().adrdy() {}
        // T::regs().isr().write(|w| w.set_adrdy(true));
    }

    fn configure(&mut self) {
        // single conversion mode, software trigger
        T::regs().cfgr().modify(|w| {
            w.set_cont(false);
            w.set_exten(Exten::DISABLED);
        });
    }

    /// Enable reading the voltage reference internal channel.
    pub fn enable_vrefint(&self) -> VrefInt {
        T::common_regs().ccr().modify(|reg| {
            reg.set_vrefen(true);
        });

        VrefInt {}
    }

    /// Enable reading the temperature internal channel.
    pub fn enable_temperature(&self) -> Temperature {
        T::common_regs().ccr().modify(|reg| {
            reg.set_vsenseen(true);
        });

        Temperature {}
    }

    /// Enable reading the vbat internal channel.
    pub fn enable_vbat(&self) -> Vbat {
        T::common_regs().ccr().modify(|reg| {
            reg.set_vbaten(true);
        });

        Vbat {}
    }

    /// Set the ADC sample time.
    pub fn set_sample_time(&mut self, sample_time: SampleTime) {
        self.sample_time = sample_time;
    }

    /// Set the ADC resolution.
    pub fn set_resolution(&mut self, resolution: Resolution) {
        T::regs().cfgr().modify(|reg| reg.set_res(resolution.into()));
    }

    pub fn set_oversampling(&mut self, oversampling_factor: u16) {
        assert!(
            oversampling_factor < 1024,
            "Oversampling factor should be less than 1024"
        );
        T::regs().cfgr2().modify(|w| {
            w.set_osr(oversampling_factor);
            w.set_rovse(oversampling_factor != 0)
        });
    }

    /// Perform a single conversion.
    fn convert(&mut self) -> u32 {
        T::regs().isr().modify(|reg| {
            reg.set_eoc(true);
        });

        // Start conversion
        T::regs().cr().modify(|reg| {
            reg.set_adstart(true);
        });

        while !T::regs().isr().read().eoc() {
            // spin
        }

        T::regs().isr().modify(|reg| {
            reg.set_eoc(true);
        });

        T::regs().dr().read().0 as u32
    }

    /// Read an ADC pin.
    pub fn read<P>(&mut self, pin: &mut P) -> u32
    where
        P: AdcPin<T>,
        P: crate::gpio::sealed::Pin,
    {
        pin.set_as_analog();

        self.read_channel(pin.channel())
    }

    pub async fn read_async<P>(&mut self, pin: &mut P) -> u32
    where
        P: AdcPin<T>,
        P: crate::gpio::sealed::Pin,
    {
        pin.set_as_analog();

        self.read_channel_async(pin.channel()).await
    }

    /// Read an ADC internal channel.
    pub fn read_internal(&mut self, channel: &mut impl InternalChannel<T>) -> u32 {
        self.read_channel(channel.channel())
    }

    pub async fn read_internal_async(&mut self, channel: &mut impl InternalChannel<T>) -> u32 {
        self.read_channel_async(channel.channel()).await
    }

    pub fn read_channel(&mut self, channel: u8) -> u32 {
        // Configure channel
        Self::set_channel_sample_time(channel, self.sample_time);

        T::regs().cfgr2().modify(|w| w.set_lshift(0));
        T::regs()
            .pcsel()
            .write(|w| w.set_pcsel(channel as _, Pcsel::PRESELECTED));

        T::regs().sqr1().write(|reg| {
            reg.set_sq(0, channel);
            reg.set_l(0);
        });

        self.convert()
    }

    pub async fn read_channel_async(&mut self, channel: u8) -> u32 {
        Self::set_channel_sample_time(channel, self.sample_time);

        T::regs().cfgr2().modify(|w| w.set_lshift(0));
        T::regs()
            .pcsel()
            .write(|w| w.set_pcsel(channel as _, Pcsel::PRESELECTED));

        T::regs().sqr1().write(|reg| {
            reg.set_sq(0, channel);
            reg.set_l(0);
        });

        T::regs().isr().modify(|reg| {
            reg.set_eoc(true);
        });

        // Start conversion
        T::regs().cr().modify(|reg| {
            reg.set_adstart(true);
        });

        poll_fn(|cx| {
            if T::regs().isr().read().eoc() {
                Poll::Ready(T::regs().dr().read().0)
            } else {
                T::regs().ier().modify(|m| m.set_eocie(true));
                ADC_WAKER.register(cx.waker());
                Poll::Pending
            }
        })
        .await
    }

    pub async fn fill_buffer_async<D>(
        &mut self,
        buf: &mut [u32],
        channels: &[(&dyn AdcPin<T>, SampleTime)],
        dma_channel: impl Peripheral<P = D>,
    ) where
        D: dma::Channel,
    {
        assert!(channels.len() <= 16);

        for (idx, c) in channels.iter().enumerate() {
            Self::set_channel_sample_time(c.0.channel(), c.1);
            Self::set_sqr(idx, c.0.channel());
            T::regs()
                .pcsel()
                .modify(|m| m.set_pcsel(c.0.channel() as usize, Pcsel::PRESELECTED));
        }

        T::regs().sqr1().modify(|m| m.set_l(channels.len() as u8));
        T::regs().cfgr2().modify(|w| w.set_lshift(0));
        T::regs().cfgr().modify(|w| {
            w.set_dmngt(pac::adc::vals::Dmngt::DMA_ONESHOT);
            w.set_cont(true)
        });

        const ADC_DMA_REQUEST: u8 = 0; // WARNING: HARDCODED REQUEST ID THIS TIME
        let transfer = unsafe {
            dma::Transfer::new_read(
                dma_channel,
                ADC_DMA_REQUEST,
                T::regs().dr().as_ptr() as *mut u32,
                buf,
                dma::TransferOptions::default(),
            )
        };

        T::regs().cr().modify(|reg| {
            reg.set_adstart(true);
        });

        transfer.await;

        T::regs().cr().modify(|reg| {
            reg.set_adstp(Adstp::STOP);
        });

        while T::regs().cr().read().adstart() {}
    }

    fn set_sqr(index: usize, channel: u8) {
        if index < 4 {
            T::regs().sqr1().modify(|m| m.set_sq(index, channel));
        } else if index < 9 {
            T::regs().sqr2().modify(|m| m.set_sq(index - 4, channel));
        } else if index < 14 {
            T::regs().sqr1().modify(|m| m.set_sq(index - 9, channel));
        } else if index < 16 {
            T::regs().sqr1().modify(|m| m.set_sq(index - 14, channel));
        }
    }

    fn set_channel_sample_time(ch: u8, sample_time: SampleTime) {
        let sample_time = sample_time.into();
        if ch <= 9 {
            T::regs().smpr(0).modify(|reg| reg.set_smp(ch as _, sample_time));
        } else {
            T::regs().smpr(1).modify(|reg| reg.set_smp((ch - 10) as _, sample_time));
        }
    }
}
