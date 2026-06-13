//! Opt-in deep-sleep (STOP mode) suspend for the wired keyboard.
//!
//! Enabled by the `stop_suspend` cargo feature. While the USB host is
//! suspended, the matrix scan loop calls `stop_nap` in place of an ordinary
//! timer delay: the MCU enters STOP mode (every clock except the LSI-driven
//! RTC gated, SRAM and peripheral registers retained) and an RTC wakeup timer
//! brings it back about once per `arm_rtc_wakeup` period to trickle-scan.
//! This drops suspend current from the milliamp range (CPU in WFI sleep) to
//! the low-microamp range.
//!
//! On STM32F4 the embassy low-power executor does not restore the clock tree
//! after STOP, so this module restores it by hand. STOP wakes the core on the
//! 16 MHz HSI; `stop_nap` then re-enables the HSE and PLL and switches the
//! system clock back to the PLL (restoring the 48 MHz USB clock with it)
//! before returning. The PLL and HSE *configuration* registers are retained
//! across STOP, so only the enable-and-switch sequence is replayed. TIM5 (the
//! embassy time driver) is deliberately left untouched: it is merely unclocked
//! during STOP and resumes counting on wake, so monotonic time is preserved
//! (it simply does not advance while stopped).
//!
//! # Warning
//!
//! This path drives PWR, RCC, RTC, and EXTI directly and cannot be exercised
//! in CI. A mistake can leave the keyboard unresponsive until it is re-flashed
//! over DFU, which is why it is gated behind an off-by-default feature. The
//! host-initiated resume latency is one wakeup period (the suspended USB
//! peripheral cannot interrupt the core to wake it earlier).

use cortex_m::{Peripherals, asm, interrupt::free};
use embassy_stm32::pac::{
    EXTI,
    PWR,
    RCC,
    RTC,
    pwr::vals::Pdds,
    rcc::vals::{Rtcsel, Sw},
    rtc::vals::Wucksel,
};
use embassy_time::Duration;

/// EXTI line the RTC wakeup timer is wired to on STM32F401.
const RTC_WAKEUP_EXTI_LINE: usize = 22;

/// First RTC write-protection unlock key, written to `RTC.WPR`.
const WPR_KEY_1: u8 = 0xCA;
/// Second RTC write-protection unlock key, written to `RTC.WPR`.
const WPR_KEY_2: u8 = 0x53;
/// Any other value re-locks the RTC configuration registers.
const WPR_LOCK: u8 = 0xFF;

/// LSI is nominally 32 kHz; the wakeup clock is RTC/16 (see
/// [`Wucksel::Div16`]), so the timer ticks at 2 kHz, i.e. two ticks per
/// millisecond.
const WAKEUP_TICKS_PER_MS: u64 = 2;

/// Configure the LSI, RTC, and RTC wakeup timer to fire every `interval`, and
/// route it to the core's STOP wakeup logic through EXTI.
///
/// Call once before the suspend loop. `interval` sets the trickle-scan cadence
/// and therefore the worst-case host-initiated resume latency.
pub fn arm_rtc_wakeup(interval: Duration) {
    // Auto-reload value: the wakeup timer counts `wut + 1` cycles.
    let ticks = interval.as_millis().saturating_mul(WAKEUP_TICKS_PER_MS);
    let wut = u16::try_from(ticks.saturating_sub(1)).unwrap_or(u16::MAX);

    // Bring up the low-speed internal oscillator that clocks the RTC in STOP.
    RCC.csr().modify(|w| w.set_lsion(true));
    while !RCC.csr().read().lsirdy() {
        asm::nop();
    }

    // Unlock the backup domain, select LSI as the RTC clock, and enable it.
    PWR.cr1().modify(|w| w.set_dbp(true));
    RCC.bdcr().modify(|w| {
        w.set_rtcsel(Rtcsel::Lsi);
        w.set_rtcen(true);
    });

    // Program the wakeup timer behind the RTC write-protection keys.
    unlock_rtc();
    RTC.cr().modify(|w| w.set_wute(false));
    while !RTC.isr().read().wutwf() {
        asm::nop();
    }
    RTC.wutr().write(|w| w.set_wut(wut));
    RTC.cr().modify(|w| w.set_wucksel(Wucksel::Div16));
    RTC.isr().modify(|w| w.set_wutf(false));
    RTC.cr().modify(|w| {
        w.set_wutie(true);
        w.set_wute(true);
    });
    lock_rtc();

    // Route the wakeup to EXTI line 22 as an event (rising edge), so `WFE`
    // wakes the core from STOP without vectoring to an interrupt handler.
    EXTI.rtsr(0).modify(|w| w.set_line(RTC_WAKEUP_EXTI_LINE, true));
    EXTI.emr(0).modify(|w| w.set_line(RTC_WAKEUP_EXTI_LINE, true));
}

/// Enter STOP mode until the next RTC wakeup, then restore the 84 MHz clock
/// tree before returning.
///
/// Runs with interrupts masked so the wakeup event resumes execution inline
/// (rather than vectoring) and no interrupt can run while the core is on the
/// HSI mid-restore. `arm_rtc_wakeup` must have been called first.
pub fn stop_nap() {
    free(|_| {
        // Clear the previous period's wakeup flags so the timer arms cleanly.
        RTC.isr().modify(|w| w.set_wutf(false));
        EXTI.pr(0).write(|w| w.set_line(RTC_WAKEUP_EXTI_LINE, true));

        // Select STOP (not Standby) with the regulator in low-power mode.
        PWR.cr1().modify(|w| {
            w.set_pdds(Pdds::StopMode);
            w.set_lpds(true);
            w.set_cwuf(true);
        });

        // SAFETY: single-threaded; we own the core peripherals for this scope.
        let mut scb = unsafe { Peripherals::steal() }.SCB;
        scb.set_sleepdeep();

        // Sleep on the wakeup event. `sev; wfe; wfe` first clears any stale
        // event latched by a prior exception return, so the second `wfe`
        // actually sleeps instead of falling through.
        asm::dsb();
        asm::sev();
        asm::wfe();
        asm::wfe();
        asm::isb();

        // Back on the HSI: leave deep sleep and rebuild the clock tree before
        // interrupts are unmasked.
        scb.clear_sleepdeep();
        restore_clocks();
    });
}

/// Re-enable the HSE and PLL and switch the system clock back to the PLL.
///
/// The PLL/HSE configuration registers survive STOP, so only the enable and
/// switch steps are replayed; this matches the boot configuration in
/// `stm32_config` (84 MHz SYSCLK, 48 MHz USB clock from PLL-Q).
fn restore_clocks() {
    RCC.cr().modify(|w| w.set_hseon(true));
    while !RCC.cr().read().hserdy() {
        asm::nop();
    }
    RCC.cr().modify(|w| w.set_pllon(true));
    while !RCC.cr().read().pllrdy() {
        asm::nop();
    }
    RCC.cfgr().modify(|w| w.set_sw(Sw::Pll1P));
    while RCC.cfgr().read().sws() != Sw::Pll1P {
        asm::nop();
    }
}

/// Write the RTC write-protection unlock sequence.
fn unlock_rtc() {
    RTC.wpr().write(|w| w.set_key(WPR_KEY_1));
    RTC.wpr().write(|w| w.set_key(WPR_KEY_2));
}

/// Re-lock the RTC configuration registers.
fn lock_rtc() { RTC.wpr().write(|w| w.set_key(WPR_LOCK)); }
