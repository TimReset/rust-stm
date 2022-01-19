// src/main.rs

// std and main are not available for bare metal software
#![no_std]
#![no_main]

use panic_halt as _;
use core::borrow::Borrow;
use core::mem::MaybeUninit;
use cortex_m::peripheral::{DWT, SYST};
use cortex_m_rt::entry;
use cortex_m_semihosting::{hprint, hprintln};
// The runtime
use embedded_hal::digital::v2::OutputPin;
use stm32::NVIC;
// the `set_high/low`function
use stm32f1xx_hal::{delay::Delay, pac, prelude::*, stm32, stm32::interrupt};
use stm32f1xx_hal::gpio::{Edge, ExtiPin, Floating, Input, Output, PullUp, PushPull};
// STM32F1 specific functions
use stm32f1xx_hal::timer::{Event, Timer}; // When a panic occurs, stop the microcontroller

// This marks the entrypoint of our application. The cortex_m_rt creates some
// startup code before this, but we don't need to worry about this
#[entry]
fn main() -> ! {
    /*  // Get handles to the hardware objects. These functions can only be called
      // once, so that the borrowchecker can ensure you don't reconfigure
      // something by accident.
      let p = stm32::Peripherals::take().unwrap();
      let dp = pac::Peripherals::take().unwrap();

      let mut cp = cortex_m::Peripherals::take().unwrap();
      cp.DWT.enable_cycle_counter();
      // cp.SYST.set_reload(72_000_000);
      // cp.SYST.clear_current();
      // cp.SYST.enable_counter();
      // GPIO pins on the STM32F1 must be driven by the APB2 peripheral clock.
      // This must be enabled first. The HAL provides some abstractions for
      // us: First get a handle to the RCC peripheral:
      let mut rcc = dp.RCC.constrain();

      // Now we have access to the RCC's registers. The GPIOC can be enabled in
      // RCC_APB2ENR (Prog. Ref. Manual 8.3.7), therefore we must pass this
      // register to the `split` function.
      let mut gpioc = dp.GPIOC.split();
      let mut afio = dp.AFIO.constrain();
      let mut pin_pc0 = gpioc.pc0.into_pull_down_input(&mut gpioc.crl);
      pin_pc0.make_interrupt_source(&mut afio);
      pin_pc0.enable_interrupt(&dp.EXTI);
      pin_pc0.trigger_on_edge(&dp.EXTI, stm32f1xx_hal::gpio::Edge::RisingFalling);
      unsafe {
          NVIC::unmask(stm32::interrupt::EXTI0);
      }

      // This gives us an exclusive handle to the GPIOC peripheral. To get the
      // handle to a single pin, we need to configure the pin first. Pin C13
      // is usually connected to the Bluepills onboard LED.
      let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);

      // Now we need a delay object. The delay is of course depending on the clock
      // frequency of the microcontroller, so we need to fix the frequency
      // first. The system frequency is set via the FLASH_ACR register, so we
      // need to get a handle to the FLASH peripheral first:
      let mut flash = dp.FLASH.constrain();
      // Now we can set the controllers frequency to 8 MHz:
      let clocks = rcc.cfgr.sysclk(8.mhz()).freeze(&mut flash.acr);
      // The `clocks` handle ensures that the clocks are now configured and gives
      // the `Delay::new` function access to the configured frequency. With
      // this information it can later calculate how many cycles it has to
      // wait. The function also consumes the System Timer peripheral, so that no
      // other function can access it. Otherwise the timer could be reset during a
      // delay.
      let mut delay = Delay::new(cp.SYST, clocks);

      // Now, enjoy the lightshow!
      loop { //4_294_967_296  761_243_118
          let dtwV = DWT::cycle_count();
          hprintln!("Test from STM32 {}", dtwV);
          // hprint!("{}", cp.SYST.cvr.read());
          led.set_high();
          delay.delay_ms(1_000_u16);
          led.set_low();
          delay.delay_ms(1_000_u16);
          // hprint!("{}", cp.SYST.cvr.read());
      }*/
    // initialization phase
    let p = pac::Peripherals::take().unwrap();
    let mut cp = cortex_m::peripheral::Peripherals::take().unwrap();
    cp.DWT.enable_cycle_counter();
    let mut afio = p.AFIO.constrain();
    {
        // the scope ensures that the int_pin reference is dropped before the first ISR can be executed.

        let mut gpioa = p.GPIOA.split();
        let mut gpioc = p.GPIOC.split();
        // let pin_pa12 = gpioa.pa12.into_pull_up_input(&mut gpioa.crh);

        let led = unsafe { &mut *LED.as_mut_ptr() };
        *led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);
        let int_pin = unsafe { &mut *INT_PIN.as_mut_ptr() };
        *int_pin = gpioa.pa7.into_floating_input(&mut gpioa.crl);
        int_pin.make_interrupt_source(&mut afio);
        int_pin.trigger_on_edge(&p.EXTI, Edge::RisingFalling);
        int_pin.enable_interrupt(&p.EXTI);
    } // initialization ends here
    {
        let mut gpiob = p.GPIOB.split();
        let pin_pb12 = unsafe { &mut *INT_PIN_PB12.as_mut_ptr() };
        *pin_pb12 = gpiob.pb12.into_pull_up_input(&mut gpiob.crh);
        pin_pb12.make_interrupt_source(&mut afio);
        pin_pb12.trigger_on_edge(&p.EXTI, Edge::RisingFalling);
        pin_pb12.enable_interrupt(&p.EXTI);
    }
    unsafe {
        pac::NVIC::unmask(pac::Interrupt::EXTI9_5);
        pac::NVIC::unmask(pac::Interrupt::EXTI15_10);
    }

    loop {}
}

/*#[interrupt]
fn EXTI0() {
    cortex_m::interrupt::free(|cs| {
        let dtwV = DWT::cycle_count();
        hprintln!("From interrupt, {}", dtwV);
    });
}*/

static mut LED: MaybeUninit<stm32f1xx_hal::gpio::gpioc::PC13<Output<PushPull>>> =
    MaybeUninit::uninit();
static mut INT_PIN: MaybeUninit<stm32f1xx_hal::gpio::gpioa::PA7<Input<Floating>>> =
    MaybeUninit::uninit();

static mut INT_PIN_PB12: MaybeUninit<stm32f1xx_hal::gpio::gpiob::PB12<Input<PullUp>>> =
    MaybeUninit::uninit();

#[interrupt]
fn EXTI9_5() {
    let led = unsafe { &mut *LED.as_mut_ptr() };
    let int_pin = unsafe { &mut *INT_PIN.as_mut_ptr() };

    if int_pin.check_interrupt() {
        led.toggle();

        // if we don't clear this bit, the ISR would trigger indefinitely
        int_pin.clear_interrupt_pending_bit();
    }
}

#[interrupt]
fn EXTI15_10() {
    // let led = unsafe { &mut *LED.as_mut_ptr() };
    let int_pin = unsafe { &mut *INT_PIN_PB12.as_mut_ptr() };

    if int_pin.check_interrupt() {
        let dtwV = DWT::cycle_count();
        hprintln!("From interrupt, {}", dtwV);
        // if we don't clear this bit, the ISR would trigger indefinitely
        int_pin.clear_interrupt_pending_bit();
    }
}
