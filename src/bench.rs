//! Cycle-accurate performance benchmarking using the Cortex-M4 DWT counter.
//!
//! Only the `bench` feature activates real measurements. Without it every
//! call compiles away to nothing under LTO.
//!
//! # Usage
//!
//! ```rust
//! let t = bench::cycles();
//! // ... work to measure ...
//! if let Some(avg) = acc.record(bench::elapsed(t)) {
//!     debug!("avg={} cycles", avg);
//! }
//! ```

use cortex_m::peripheral::DWT;

/// Number of samples accumulated before reporting an average.
const REPORT_INTERVAL: u32 = 4096;

/// Initialises the DWT cycle counter.
///
/// No-op when the `bench` feature is disabled.
pub(crate) fn init() {
    #[cfg(feature = "bench")]
    {
        // Safety: DWT and DCB are unused by Embassy.
        let mut cp = unsafe { cortex_m::Peripherals::steal() };
        cp.DCB.enable_trace();
        cp.DWT.enable_cycle_counter();
    }
}

/// Returns the current DWT cycle counter value.
///
/// Returns zero when the `bench` feature is disabled.
#[inline(always)]
pub(crate) fn cycles() -> u32 {
    #[cfg(feature = "bench")]
    return DWT::cycle_count();
    #[cfg(not(feature = "bench"))]
    0
}

/// Returns cycles elapsed since `start`, wrapping on overflow.
///
/// Returns zero when the `bench` feature is disabled.
#[inline(always)]
pub(crate) fn elapsed(start: u32) -> u32 { cycles().wrapping_sub(start) }

/// Rolling average accumulator for cycle measurements.
///
/// Zero-size type when `bench` is disabled; all methods compile away under LTO.
pub(crate) struct Accumulator {
    #[cfg(feature = "bench")]
    sum:   u64,
    #[cfg(feature = "bench")]
    count: u32,
}

impl Accumulator {
    /// Create a new empty accumulator.
    pub(crate) const fn new() -> Self {
        Self {
            #[cfg(feature = "bench")]
            sum:                             0,
            #[cfg(feature = "bench")]
            count:                           0,
        }
    }

    /// Record one cycle measurement.
    ///
    /// Returns `Some(average)` every [`REPORT_INTERVAL`] samples.
    /// Always returns `None` when `bench` is disabled.
    pub(crate) fn record(&mut self, _cycles: u32) -> Option<u32> {
        #[cfg(not(feature = "bench"))]
        return None;

        #[cfg(feature = "bench")]
        {
            self.sum = self.sum.saturating_add(u64::from(_cycles));
            self.count = self.count.saturating_add(1);

            if self.count < REPORT_INTERVAL {
                return None;
            }

            let avg = u32::try_from(self.sum.checked_div(u64::from(self.count)).unwrap_or(0)).unwrap_or(u32::MAX);

            self.sum = 0;
            self.count = 0;
            Some(avg)
        }
    }
}
