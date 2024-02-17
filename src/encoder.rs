use core::{cell::RefCell, ops::Deref};

use critical_section::Mutex;
use embedded_hal::digital::v2::InputPin;
use rp_pico::{hal, pac};

pub struct Encoder<PinA, PinB, P>
where
    PinA: hal::gpio::PinId,
    PinB: hal::gpio::PinId,
    P: hal::gpio::PullType,
    PinA: hal::gpio::ValidFunction<hal::gpio::FunctionSioInput>,
    PinB: hal::gpio::ValidFunction<hal::gpio::FunctionSioInput>,
{
    src: Mutex<RefCell<Option<EncoderSrc<PinA, PinB, P>>>>,
    last_speed_update_time: Mutex<RefCell<u64>>,
    last_speed_update_count: Mutex<RefCell<i32>>,
}

impl<PinA, PinB, P> Encoder<PinA, PinB, P>
where
    PinA: hal::gpio::PinId,
    PinB: hal::gpio::PinId,
    P: hal::gpio::PullType,
    PinA: hal::gpio::ValidFunction<hal::gpio::FunctionSioInput>,
    PinB: hal::gpio::ValidFunction<hal::gpio::FunctionSioInput>,
{
    pub fn configure<'a, F1A, P1A, F1B, P1B>(
        &self,
        pin_a: hal::gpio::Pin<PinA, F1A, P1A>,
        pin_b: hal::gpio::Pin<PinB, F1B, P1B>,
    ) where
        F1A: crate::gpio::Function,
        P1A: hal::gpio::PullType,
        F1B: crate::gpio::Function,
        P1B: hal::gpio::PullType,
    {
        critical_section::with(|cs| {
            self.src
                .borrow(cs)
                .replace(Some(EncoderSrc::new(pin_a, pin_b)));
        });
    }
    pub fn interrupt(&self) {
        if let Some(mut encoder) = critical_section::with(|cs| self.src.borrow(cs).take()) {
            encoder.interrupt();
            critical_section::with(|cs| self.src.replace(cs, Some(encoder)));
        }
    }
    pub fn read(&self) -> Result<i32, ()> {
        critical_section::with(|cs| {
            let binding = self.src.borrow(cs).borrow();
            if let Some(encoder) = binding.deref() {
                Ok(encoder.read())
            } else {
                Err(())
            }
        })
    }
    // 23bit real 8bit under point
    pub fn read_speed(&self, now_us: u64) -> Result<i32, ()> {
        let count = self.read()?;
        let count_delta = count
            - critical_section::with(|cs| {
                let binding = self.last_speed_update_count.borrow(cs).borrow();
                binding.deref().clone()
            });
        let delta: i64 = (now_us
            - critical_section::with(|cs| {
                let binding = self.last_speed_update_time.borrow(cs).borrow();
                binding.deref().clone()
            }))
        .try_into()
        .unwrap();

        critical_section::with(|cs| {
            self.last_speed_update_time.borrow(cs).replace(now_us);
            self.last_speed_update_count.borrow(cs).replace(count);
        });

        return Ok((Into::<i64>::into(count_delta) * 0x100 * 1_000_000 / delta)
            .try_into()
            .unwrap());
    }
    pub const fn none() -> Self {
        Encoder::<PinA, PinB, P> {
            src: Mutex::new(RefCell::new(None)),
            last_speed_update_time: Mutex::new(RefCell::new(0)),
            last_speed_update_count: Mutex::new(RefCell::new(0)),
        }
    }
}

pub struct EncoderSrc<PinA, PinB, P>
where
    PinA: hal::gpio::PinId,
    PinB: hal::gpio::PinId,
    P: hal::gpio::PullType,
    PinA: hal::gpio::ValidFunction<hal::gpio::FunctionSioInput>,
    PinB: hal::gpio::ValidFunction<hal::gpio::FunctionSioInput>,
{
    pin_a: hal::gpio::Pin<PinA, hal::gpio::FunctionSioInput, P>,
    pin_b: hal::gpio::Pin<PinB, hal::gpio::FunctionSioInput, P>,
    counter: i32,
}

impl<PinA, PinB, P> EncoderSrc<PinA, PinB, P>
where
    PinA: hal::gpio::PinId,
    PinB: hal::gpio::PinId,
    P: hal::gpio::PullType,
    PinA: hal::gpio::ValidFunction<hal::gpio::FunctionSioInput>,
    PinB: hal::gpio::ValidFunction<hal::gpio::FunctionSioInput>,
{
    pub fn new<'a, F1A, P1A, F1B, P1B>(
        pin_a: hal::gpio::Pin<PinA, F1A, P1A>,
        pin_b: hal::gpio::Pin<PinB, F1B, P1B>,
    ) -> Self
    where
        F1A: crate::gpio::Function,
        P1A: hal::gpio::PullType,
        F1B: crate::gpio::Function,
        P1B: hal::gpio::PullType,
    {
        let pin_a = pin_a.reconfigure();
        let pin_b = pin_b.reconfigure();
        pin_a.set_interrupt_enabled(hal::gpio::Interrupt::EdgeHigh, true);
        pin_a.set_interrupt_enabled(hal::gpio::Interrupt::EdgeLow, true);
        pin_b.set_interrupt_enabled(hal::gpio::Interrupt::EdgeHigh, true);
        pin_b.set_interrupt_enabled(hal::gpio::Interrupt::EdgeLow, true);
        unsafe {
            pac::NVIC::unmask(pac::Interrupt::IO_IRQ_BANK0);
        }
        EncoderSrc {
            pin_a,
            pin_b,
            counter: 0,
        }
    }
    pub fn read(&self) -> i32 {
        self.counter
    }
    pub fn interrupt(&mut self) {
        if self.pin_a.interrupt_status(hal::gpio::Interrupt::EdgeHigh) {
            if self.pin_b.is_high().unwrap() {
                self.counter += 1;
            } else {
                self.counter -= 1;
            }
            self.pin_a.clear_interrupt(hal::gpio::Interrupt::EdgeHigh);
        }
        if self.pin_b.interrupt_status(hal::gpio::Interrupt::EdgeHigh) {
            if self.pin_a.is_high().unwrap() {
                self.counter -= 1;
            } else {
                self.counter += 1;
            }
            self.pin_b.clear_interrupt(hal::gpio::Interrupt::EdgeHigh);
        }
        if self.pin_a.interrupt_status(hal::gpio::Interrupt::EdgeLow) {
            if self.pin_b.is_high().unwrap() {
                self.counter -= 1;
            } else {
                self.counter += 1;
            }
            self.pin_a.clear_interrupt(hal::gpio::Interrupt::EdgeLow);
        }
        if self.pin_b.interrupt_status(hal::gpio::Interrupt::EdgeLow) {
            if self.pin_a.is_high().unwrap() {
                self.counter += 1;
            } else {
                self.counter -= 1;
            }
            self.pin_b.clear_interrupt(hal::gpio::Interrupt::EdgeLow);
        }
    }
}
