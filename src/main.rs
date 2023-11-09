//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

use bsp::{
    entry,
    hal::{gpio, rom_data::reset_to_usb_boot, spi::FrameFormat, usb::UsbBus, Clock, Sio, Timer},
};
use defmt::*;
use defmt_rtt as _;

use embedded_hal::{spi::MODE_3, timer::CountDown};
use fugit::{ExtU32, RateExtU32};
use panic_probe as _;
use usb_device::{
    class_prelude::UsbBusAllocator,
    prelude::{UsbDeviceBuilder, UsbVidPid},
};
// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use rp_pico as bsp;
// use sparkfun_pro_micro_rp2040 as bsp;

use bsp::hal::{clocks::init_clocks_and_plls, pac, watchdog::Watchdog};

use usbd_human_interface_device::{
    interface::{InBytes8, InterfaceBuilder, OutBytes8, ReportSingle},
    usb_class::UsbHidClassBuilder,
};

pub mod downstream;
pub mod mlx90363;
pub mod negicon_event;
pub mod spi_upstream;
pub mod upstream;

use upstream::Upstream;

use crate::{negicon_event::NegiconEvent, spi_upstream::SPIUpstream};

const USB_HID_DESCRIPTOR: [u8; 38] = [
    0x05, 0x01, // USAGE_PAGE (Generic Desktop)
    0x09, 0x00, // USAGE (Undefined)
    0xa1, 0x01, // COLLECTION (Application)
    0x09, 0x01, //   USAGE (Pointer)
    0xa1, 0x00, //   COLLECTION (Physical)
    // Input report
    0x09, 0x02, //     USAGE (Undefined)
    0x15, 0x00, //     LOGICAL_MINIMUM (0)
    0x26, 0xFF, 0x00, //     LOGICAL_MAXIMUM (255)
    0x75, 0x08, //     REPORT_SIZE (8) - 8 bits
    0x95, 0x08, //     REPORT_COUNT (8) - 8 fields
    0x81, 0x02, //     INPUT (Data,Var,Abs)
    // Output report
    0x09, 0x03, //     USAGE (Undefined)
    0x15, 0x00, //     LOGICAL_MINIMUM (0)
    0x26, 0xFF, 0x00, //     LOGICAL_MAXIMUM (255)
    0x75, 0x08, //     REPORT_SIZE (8)
    0x95, 0x08, //     REPORT_COUNT (8)
    0x91, 0x02, //     OUTPUT (Data,Var,Abs)
    0xc0, //   END_COLLECTION
    0xc0, // END_COLLECTION
];

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let _core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();
    //let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let timer = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

    let usb_bus = UsbBusAllocator::new(UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    let mut hid = UsbHidClassBuilder::new()
        .add_device(
            InterfaceBuilder::<InBytes8, OutBytes8, ReportSingle>::new(&USB_HID_DESCRIPTOR)
                .unwrap()
                .description("Negicon v3")
                .idle_default(500.millis())
                .unwrap()
                .in_endpoint(10.millis())
                .unwrap()
                .with_out_endpoint(100.millis())
                .unwrap()
                .build(),
        )
        .build(&usb_bus);

    let mut tick_timer = timer.count_down();
    tick_timer.start(1000.millis());

    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x1209, 0x3939))
        .manufacturer("LeekLabs International")
        .product("Negicon v3")
        .serial_number("3939")
        .build();

    let mut i = 0u8;

    let mut buffer: [u8; 8] = [0u8; 8];

    let _spi_sclk = pins.gpio10.into_function::<gpio::FunctionSpi>();
    let _spi_mosi = pins.gpio11.into_function::<gpio::FunctionSpi>();
    let _spi_miso = pins.gpio12.into_function::<gpio::FunctionSpi>();
    let mut _spi_cs = pins
        .gpio13
        .into_push_pull_output_in_state(gpio::PinState::High);

    let spi1 = bsp::hal::Spi::new(pac.SPI1, (_spi_mosi, _spi_miso, _spi_sclk))
        .init_slave(&mut pac.RESETS, FrameFormat::MotorolaSpi(MODE_3));
    let mut spi_upstream = SPIUpstream::new(spi1);

    loop {
        usb_dev.poll(&mut [&mut hid]);

        match hid.device().read_report(&mut buffer) {
            Ok(_) => {
                if buffer[0] == 0x39u8 {
                    reset_to_usb_boot(0, 0);
                }
            }
            Err(_) => {}
        }

        if let Ok(_) = tick_timer.wait() {
            let event = NegiconEvent::new(39u16, 1042i16, 0u8, i);
            let upstreams: [&mut dyn Upstream; 2] = [hid.device(), &mut spi_upstream];
            for up in upstreams {
                match up.send_event(&event) {
                    Ok(_) => {}
                    Err(e) => error!("Failed to send event: {:?}", e),
                }
            }
            i = i.wrapping_add(1);
            if i == 255 {
                //    reset_to_usb_boot(0, 0);
            }
            tick_timer.start(10.millis());
        };

        /*
        detect USB connection
        detect upstream SPI
        for CSpin {
          if initialized {
            poll(){
                ok => forward upstream
                error => initialized = false
            }
          }else{
            initialized = detect()
          }
        }
        */
    }
}

// End of file
