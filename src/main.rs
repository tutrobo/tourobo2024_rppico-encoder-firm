//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
// #![feature(generic_const_exprs)]
#![no_std]
#![no_main]

use bsp::{
    entry,
    hal::gpio::{FunctionSioOutput, Pin},
    pac,
};
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::v2::OutputPin;
use panic_probe as _;

// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use rp_pico as bsp;
// use sparkfun_pro_micro_rp2040 as bsp;

use bsp::hal::{clocks::init_clocks_and_plls, gpio, pac::interrupt, sio::Sio, watchdog::Watchdog};

use bsp::hal;

// USB Device support
use usb_device::{class_prelude::*, prelude::*};

// USB Communications Class Device support
use usbd_serial::SerialPort;

mod buffered_cobs;
use crate::buffered_cobs::BufferedCobs;
mod scheduler;
use crate::scheduler::Scheduler;
mod encoder;
use crate::encoder::Encoder;

static ENCODER1: Encoder<gpio::bank0::Gpio16, gpio::bank0::Gpio17, gpio::PullUp> = Encoder::none();
static ENCODER2: Encoder<gpio::bank0::Gpio18, gpio::bank0::Gpio19, gpio::PullUp> = Encoder::none();
static ENCODER3: Encoder<gpio::bank0::Gpio20, gpio::bank0::Gpio21, gpio::PullUp> = Encoder::none();
static ENCODER4: Encoder<gpio::bank0::Gpio22, gpio::bank0::Gpio26, gpio::PullUp> = Encoder::none();

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let _core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    let clocks = init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    // let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    #[cfg(feature = "rp2040-e5")]
    {
        let sio = hal::Sio::new(pac.SIO);
        let _pins = rp_pico::Pins::new(
            pac.IO_BANK0,
            pac.PADS_BANK0,
            sio.gpio_bank0,
            &mut pac.RESETS,
        );
    }

    // Set up the USB driver
    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    // Set up the USB Communications Class Device driver
    let mut serial = SerialPort::new(&usb_bus);

    // Create a USB device with a fake VID and PID
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .manufacturer("paripal_NWRC")
        .product("Serial port")
        .serial_number("01234567")
        .device_class(2) // from: https://www.usb.org/defined-class-codes
        .build();

    let mut led_pin: Pin<gpio::bank0::Gpio25, FunctionSioOutput, gpio::PullNone> =
        pins.led.reconfigure();
    led_pin.set_high().unwrap();
    while timer.get_counter().ticks() < 1_000_000 {
        usb_dev.poll(&mut [&mut serial]);
    }
    led_pin.set_low().unwrap();

    ENCODER1.configure(pins.gpio16, pins.gpio17);
    ENCODER2.configure(pins.gpio18, pins.gpio19);
    ENCODER3.configure(pins.gpio20, pins.gpio21);
    ENCODER4.configure(pins.gpio22, pins.gpio26);

    let mut scheduler = Scheduler::new(10_000, &timer);
    loop {
        // A welcome message at the beginning
        if scheduler.update() {
            let time = timer.get_counter().ticks();
            let counter = [ENCODER1.read(), ENCODER2.read(), ENCODER3.read()];
            let speeds = [
                ENCODER1.read_speed(time),
                ENCODER2.read_speed(time),
                ENCODER3.read_speed(time),
            ];
            let mut send_data: [u8; 25] = [0; 25];
            send_data[0] = 0x01;
            for (i, count) in counter.iter().enumerate() {
                let data = count.unwrap().to_le_bytes();
                send_data[1 + i * 4..5 + i * 4].copy_from_slice(&data[0..4]);
            }
            for (i, speed) in speeds.iter().enumerate() {
                let data = speed.unwrap().to_le_bytes();
                send_data[13 + i * 4..17 + i * 4].copy_from_slice(&data[0..4]);
            }
            let _ = serial.write(&BufferedCobs::<0>::encode::<25, 27>(send_data));

            // led_pin.toggle().unwrap();
        }

        // Check for new data
        usb_dev.poll(&mut [&mut serial]);
    }
}

#[interrupt]
fn IO_IRQ_BANK0() {
    ENCODER1.interrupt();
    ENCODER2.interrupt();
    ENCODER3.interrupt();
    ENCODER4.interrupt();
}

// End of file
