//! Hash processor
#![macro_use]

use core::array::from_fn;
use core::future::poll_fn;
use core::marker::PhantomData;
use core::task::Poll;
use cortex_m_semihosting::hprintln;

use embassy_hal_internal::{into_ref, PeripheralRef};
use embassy_sync::waitqueue::AtomicWaker;

use crate::interrupt::typelevel::Interrupt;
use crate::{interrupt, pac, peripherals, Peripheral};

static DCIS_WAKER: AtomicWaker = AtomicWaker::new();
static DINIS_WAKER: AtomicWaker = AtomicWaker::new();

/// HASH interrupt handler
pub struct InterruptHandler<T: Instance> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> interrupt::typelevel::Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        let (imr, sr) = (T::regs().imr().read(), T::regs().sr().read());
        if sr.dcis() && imr.dcie() {
            T::regs().imr().modify(|w| w.set_dcie(false));
            DCIS_WAKER.wake();
        }
        if sr.dinis() && imr.dinie() {
            T::regs().imr().modify(|w| w.set_dinie(false));
            DINIS_WAKER.wake();
        }
        // HASH_WAKER.wake();
    }
}

/// HASH driver
pub struct Hash<'d, T: Instance> {
    _inner: PeripheralRef<'d, T>,
    buffer: Buffer,
}

impl<'d, T: Instance> Hash<'d, T> {
    /// Create a new HASH driver
    pub fn new(
        inner: impl Peripheral<P = T> + 'd,
        config: &Config,
        _irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
    ) -> Self {
        T::enable_and_reset();
        into_ref!(inner);
        let mut hash = Self {
            _inner: inner,
            buffer: Buffer {
                data: [0u8; 4],
                free: 4,
            },
        };
        hash.init(config);

        T::Interrupt::unpend();
        unsafe { T::Interrupt::enable() };

        hash
    }

    /// Init the hash generator
    #[cfg(hash_v4)]
    pub fn init(&mut self, config: &Config) {
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

    async fn feed_u32(&mut self) {
        // hprintln!("Feeding");
        let bits = T::regs().sr().read();
        if !bits.dinis() && bits.nbwe() == 0 {
            // hprintln!("Poll for input started");
            poll_fn(|cx| {
                let bits = T::regs().sr().read();
                if bits.dinis() {
                    return Poll::Ready(());
                }
                DINIS_WAKER.register(cx.waker());
                T::regs().imr().modify(|w| w.set_dinie(true));
                let bits = T::regs().sr().read();
                if bits.dinis() {
                    Poll::Ready(())
                } else {
                    Poll::Pending
                }
            })
            .await;
            T::regs().sr().read();
        }
        if bits.dinis() | (bits.nbwe() > 0) {
            // hprintln!("Ready to write, DINIS = {}, NBWE = {:?}", bits.dinis(), bits.nbwe());
            let data = u32::from_be_bytes(self.buffer.data);
            T::regs().din().write_value(data);
            // hprintln!("Data written, DINIS = {:?}", T::regs().sr().read().dinis());
            // hprintln!("NBWE after write = {:?}", T::regs().sr().read().nbwe());
            self.buffer.data = [0u8; 4];
            self.buffer.free = 4;
        }
    }

    async fn finalize(&mut self) -> Digest {
        if self.buffer.free != 4 {
            // hprintln!("Free are {}", self.buffer.free);
            // hprintln!("Some leftovers found");
            let nblw = 8 * (4 - self.buffer.free) as u8;
            T::regs().str().modify(|w| w.set_nblw(nblw));
            // hprintln!("{}", nblw);
            self.feed_u32().await;
        } else {
            // hprintln!("No leftovers");
        }
        T::regs().str().modify(|w| w.set_dcal(true));
        let bits = T::regs().sr().read();
        if !bits.dcis() {
            // hprintln!("Poll output started");
            poll_fn(|cx| {
                let bits = T::regs().sr().read();
                if bits.dcis() {
                    return Poll::Ready(());
                }
                DCIS_WAKER.register(cx.waker());
                T::regs().imr().modify(|w| w.set_dcie(true));
                let bits = T::regs().sr().read();
                if bits.dcis() {
                    Poll::Ready(())
                } else {
                    Poll::Pending
                }
            })
            .await;
            T::regs().sr().read();
        }
        if bits.dcis() {
            // hprintln!("Calculation ready");
        }
        let read_hr = |i| T::regs().hr(i).read();
        let result = match T::regs().cr().read().algo() {
            pac::hash::vals::Algo::MD5 => Digest::MD5(from_fn(read_hr)),
            pac::hash::vals::Algo::SHA1 => Digest::Sha1(from_fn(read_hr)),
            pac::hash::vals::Algo::SHA2224 => Digest::Sha224(from_fn(read_hr)),
            pac::hash::vals::Algo::SHA2256 => Digest::Sha256(from_fn(read_hr)),
        };
        result
    }

    /// Feed data to hash block
    pub async fn hash_next(&mut self, value: &[u8]) {
        for i in 0..value.len() {
            if self.buffer.free > 0 {
                // hprintln!("Free was {}", self.buffer.free);
                self.buffer.data[4 - self.buffer.free] = value[i];
                self.buffer.free -= 1;
            }
            if self.buffer.free == 0 {
                // hprintln!("Feeding");
                self.feed_u32().await;
            }
        }
    }

    /// Feed data and het hash result
    pub async fn hash_and_finalize(&mut self, value: &[u8]) -> Digest {
        self.hash_next(value).await;
        self.finalize().await
    }
}

pub(crate) mod sealed {
    use super::*;

    pub trait Instance {
        fn regs() -> pac::hash::Hash;
    }
}

/// Hash generator configuration
pub struct Config {
    pub algo: Algo,
    pub datatype: Datatype,
    pub dma_enable: bool,
    pub n_valid_bits: u8,
}
impl Default for Config {
    fn default() -> Self {
        Self {
            algo: Algo::Sha256,
            datatype: Datatype::NoSwap,
            dma_enable: false,
            n_valid_bits: 32,
        }
    }
}

struct Buffer {
    data: [u8; 4],
    free: usize,
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

/// Hash result
pub enum Digest {
    /// Sha1 result (160-bit)
    Sha1([u32; 5]),
    /// MD5 result (128-bit)
    MD5([u32; 4]),
    /// Sha224 result (224-bit)
    Sha224([u32; 7]),
    /// Sha256 result (256-bit)
    Sha256([u32; 8]),
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
