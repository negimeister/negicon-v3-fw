//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]
extern crate alloc;
use defmt::{debug, error, info, warn};
use defmt_rtt as _;

use embedded_alloc::Heap;
use embedded_hal::{digital::v2::PinState, spi::MODE_1, timer::CountDown};
use fugit::{ExtU32, RateExtU32};
use panic_probe as _;
use usb_device::{
    class_prelude::UsbBusAllocator,
    prelude::{UsbDeviceBuilder, UsbVidPid},
};
// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use rp2040_hal as hal;
// use sparkfun_pro_micro_rp2040 as bsp;

use hal::{
    clocks::init_clocks_and_plls,
    clocks::Clock,
    entry,
    gpio::{FunctionSpi, Pins},
    pac,
    rom_data::reset_to_usb_boot,
    spi::FrameFormat,
    usb::UsbBus,
    watchdog::Watchdog,
    Sio, Timer,
};

use usbd_human_interface_device::{
    interface::{InBytes8, InterfaceBuilder, OutBytes8, ReportSingle},
    usb_class::UsbHidClassBuilder,
};

pub mod downstream;
pub mod negicon_event;
pub mod upstream;
use upstream::upstream::Upstream;

use crate::{
    downstream::spi_downstream::SpiDownstream, negicon_event::NegiconEvent,
    upstream::spi::SPIUpstream,
};

#[global_allocator]
static HEAP: Heap = Heap::empty();

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
#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;
#[entry]
fn main() -> ! {
    info!("Program start");
    {
        use core::mem::MaybeUninit;
        const HEAP_SIZE: usize = 1024;
        static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
        unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE) }
    }
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
    let mut delay = cortex_m::delay::Delay::new(_core.SYST, clocks.system_clock.freq().to_Hz());

    let pins = Pins::new(
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

    let _i = 0u8;

    let mut buffer: [u8; 8] = [0u8; 8];
    let _spi_sclk = pins.gpio10.into_function::<FunctionSpi>();
    let _spi_mosi = pins.gpio11.into_function::<FunctionSpi>();
    let _spi_miso = pins.gpio12.into_function::<FunctionSpi>();
    let mut _spi_cs = pins.gpio13.into_push_pull_output_in_state(PinState::High);

    let spi1 = hal::Spi::new(pac.SPI1, (_spi_mosi, _spi_miso, _spi_sclk))
        .init_slave(&mut pac.RESETS, FrameFormat::MotorolaSpi(MODE_1));

    let _spi0_sclk = pins.gpio18.into_function::<FunctionSpi>();
    let _spi0_mosi = pins.gpio19.into_function::<FunctionSpi>();
    let _spi0_miso = pins.gpio20.into_function::<FunctionSpi>();
    let mut spi0 = hal::Spi::new(pac.SPI0, (_spi0_mosi, _spi0_miso, _spi0_sclk)).init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        2_500_000u32.Hz(),
        &embedded_hal::spi::MODE_1,
    );

    let _spi_upstream = SPIUpstream::new(spi1);

    let mut cs0 = pins.gpio0.into_push_pull_output_in_state(PinState::High);
    let mut cs1 = pins.gpio1.into_push_pull_output_in_state(PinState::High);
    let mut cs2 = pins.gpio2.into_push_pull_output_in_state(PinState::High);
    let mut cs3 = pins.gpio3.into_push_pull_output_in_state(PinState::High);
    let mut cs4 = pins.gpio4.into_push_pull_output_in_state(PinState::High);
    let mut cs5 = pins.gpio5.into_push_pull_output_in_state(PinState::High);
    let mut cs6 = pins.gpio6.into_push_pull_output_in_state(PinState::High);
    let mut cs7 = pins.gpio7.into_push_pull_output_in_state(PinState::High);
    let mut cs8 = pins.gpio8.into_push_pull_output_in_state(PinState::High);
    let mut cs9 = pins.gpio9.into_push_pull_output_in_state(PinState::High);
    let mut cs10 = pins.gpio14.into_push_pull_output_in_state(PinState::High);
    let mut cs11 = pins.gpio15.into_push_pull_output_in_state(PinState::High);
    let mut cs12 = pins.gpio16.into_push_pull_output_in_state(PinState::High);
    let mut cs13 = pins.gpio17.into_push_pull_output_in_state(PinState::High);
    let mut cs14 = pins.gpio21.into_push_pull_output_in_state(PinState::High);
    let mut cs15 = pins.gpio22.into_push_pull_output_in_state(PinState::High);
    let mut cs16 = pins.gpio23.into_push_pull_output_in_state(PinState::High);
    let mut cs17 = pins.gpio24.into_push_pull_output_in_state(PinState::High);
    let mut cs18 = pins.gpio25.into_push_pull_output_in_state(PinState::High);
    let mut cs19 = pins.gpio26.into_push_pull_output_in_state(PinState::High);
    let mut cs20 = pins.gpio27.into_push_pull_output_in_state(PinState::High);

    let mut downstreams = [
        SpiDownstream::new(&mut cs0),
        SpiDownstream::new(&mut cs1),
        SpiDownstream::new(&mut cs2),
        SpiDownstream::new(&mut cs3),
        SpiDownstream::new(&mut cs4),
        SpiDownstream::new(&mut cs5),
        SpiDownstream::new(&mut cs6),
        SpiDownstream::new(&mut cs7),
        SpiDownstream::new(&mut cs8),
        SpiDownstream::new(&mut cs9),
        SpiDownstream::new(&mut cs10),
        SpiDownstream::new(&mut cs11),
        SpiDownstream::new(&mut cs12),
        SpiDownstream::new(&mut cs13),
        SpiDownstream::new(&mut cs14),
        SpiDownstream::new(&mut cs15),
        SpiDownstream::new(&mut cs16),
        SpiDownstream::new(&mut cs17),
        SpiDownstream::new(&mut cs18),
        SpiDownstream::new(&mut cs19),
        SpiDownstream::new(&mut cs20),
    ];

    loop {
        usb_dev.poll(&mut [&mut hid]);
        match tick_timer.wait() {
            Ok(_) => {
                tick_timer.start(5.millis());

                match hid.device().read_report(&mut buffer) {
                    Ok(_) => {
                        if buffer.len() == 8 {
                            let event = NegiconEvent::deserialize(buffer);
                            match event.event_type {
                                negicon_event::NegiconEventType::Input => {
                                    error!("Input event received from upstream")
                                }
                                negicon_event::NegiconEventType::Output => {
                                    warn!("Output events are not implemented yet")
                                }
                                negicon_event::NegiconEventType::MemWrite => downstreams
                                    [event.id as usize]
                                    .write_memory(&event, &mut spi0, &mut delay),
                                negicon_event::NegiconEventType::Reboot => reset_to_usb_boot(0, 0),
                            }
                        }
                    }
                    Err(_) => {}
                }

                for ds in downstreams.iter_mut() {
                    match ds.poll(&mut delay, &mut spi0) {
                        Ok(res) => {
                            res.map(|event| {
                                let upstreams: [&mut dyn Upstream; 1] = [hid.device()];
                                for up in upstreams {
                                    match up.send_event(&event) {
                                        Ok(_) => {}
                                        Err(e) => match e {
                                            upstream::upstream::UpstreamError::SpiError => {
                                                error!("SPI error")
                                            }
                                            upstream::upstream::UpstreamError::UsbError(e) => {
                                                match e {
                                                    usb_device::UsbError::WouldBlock => {
                                                        //TODO enqueue event
                                                        //error!("USB would block")
                                                    }
                                                    _ => error!("USB error {}", e),
                                                }
                                            }
                                        },
                                    }
                                }
                            });
                        }
                        Err(e) => {
                            //debug!("Error while polling: {:?}", e);
                        }
                    };
                }
            }
            Err(_) => {}
        }
    }
}

// End of file
