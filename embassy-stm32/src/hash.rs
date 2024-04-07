//! Hash processor
#![macro_use]

use core::marker::PhantomData;

use embassy_hal_internal::{into_ref, PeripheralRef};
use embassy_sync::waitqueue::AtomicWaker;

use crate::interrupt::typelevel::Interrupt;
use crate::{interrupt, pac, peripherals, Peripheral};

static HASH_WAKER: AtomicWaker = AtomicWaker::new();

/// HASH interrupt handler
pub struct InterruptHandler<T: Instance> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> interrupt::typelevel::Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        let bits = T::regs().sr().read();
        if bits.dcis() || bits.dinis() {
            HASH_WAKER.wake();
        }
    }
}

/// HASH driver
pub struct Hash<'d, T: Instance> {
    _inner: PeripheralRef<'d, T>,
}

impl<'d, T: Instance> Hash<'d, T> {
    /// Create a new HASH driver
    pub fn new(
        inner: impl Peripheral<P = T> + 'd,
        config: Config,
        _irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
    ) -> Self {
        T::enable_and_reset();
        into_ref!(inner);
        let mut hash = Self { _inner: inner };
        hash.init(config);

        T::Interrupt::unpend();
        unsafe { T::Interrupt::enable() };

        hash
    }

    /// Init the hash generator
    #[cfg(hash_v4)]
    pub fn init(&mut self, config: Config) {
        T::regs().str().write(|reg| reg.set_nblw(config.n_valid_bits));
        T::regs().cr().write(|reg| {
            reg.set_algo(match config.algo {
                Algo::Sha1 => pac::hash::vals::Algo::SHA1,
                Algo::MD5 => pac::hash::vals::Algo::MD5,
                Algo::Sha224 => pac::hash::vals::Algo::SHA2224,
                Algo::Sha256 => pac::hash::vals::Algo::SHA2256,
            });
            reg.set_datatype(match config.datatype {
                Datatype::NoSwap => pac::hash::vals::Datatype::NOSWAP,
                Datatype::HalfWord => pac::hash::vals::Datatype::HALFWORD,
                Datatype::ByteSwap => pac::hash::vals::Datatype::BYTESWAP,
                Datatype::BitSwap => pac::hash::vals::Datatype::BITSWAP,
            });
            if config.dma_enable {
                reg.set_dmae(true);
            }
            reg.set_init(true);
        });
    }

    pub fn get_digest(&self, valuе: &[u8]) -> usize {}
}

pub(crate) mod sealed {
    use super::*;

    pub trait Instance {
        fn regs() -> pac::hash::Hash;
    }
}

/// Hash generator configuration
pub struct Config {
    algo: Algo,
    datatype: Datatype,
    dma_enable: bool,
    n_valid_bits: u8,
}
impl Default for Config {
    fn default() -> Self {
        Self {
            algo: Algo::Sha256,
            datatype: Datatype::ByteSwap,
            dma_enable: false,
            n_valid_bits: 32,
        }
    }
}

/// Algorithm
pub enum Algo {
    /// Sha1 algorithm
    Sha1,
    /// MD5 algorithm
    MD5,
    /// Sha224 algorithm
    Sha224,
    /// Sha256 algorithm
    Sha256,
}

/// Datatype (swapping mode)
pub enum Datatype {
    /// Full word (no swap)
    NoSwap,
    /// Half word swap (16-bit data)
    HalfWord,
    /// Byte swap (8-bit data)
    ByteSwap,
    /// Bit-string (bit data)
    BitSwap,
}

/// HASH instance trait
pub trait Instance: sealed::Instance + Peripheral<P = Self> + crate::rcc::RccPeripheral + 'static + Send {
    /// Interrupt for this HASH instance
    type Interrupt: interrupt::typelevel::Interrupt;
}

foreach_interrupt!(
    ($inst:ident, hash, HASH, GLOBAL, $irq:ident) => {
        impl Instance for peripherals::$inst {
            type Interrupt = crate::interrupt::typelevel::$irq;
        }

        impl sealed::Instance for peripherals::$inst {
            fn regs() -> crate::pac::hash::Hash {
                crate::pac::$inst
            }
        }
    };
);
