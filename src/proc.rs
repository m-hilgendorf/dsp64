trait Clamped {
    fn clamped(self, min: Self, max: Self) -> Self;
}

impl Clamped for usize {
    fn clamped(self, min: Self, max: Self) -> Self {
        self.min(max).max(min)
    }
}

/// Delay processors
pub mod delay {
    use super::Clamped;
    use crate::{Process, Signal};

    /// A fixed length delay line
    #[derive(Clone, Debug)]
    pub struct DelayLine<S>
    where
        S: Signal + Copy + Sized,
    {
        write: usize,
        read: usize,
        mem: Vec<S>,
    }

    impl<S> DelayLine<S>
    where
        S: Signal + Copy + Sized,
    {
        /// Create a delay line with a maximum capacity. This is the only method that will allocate
        /// besides Clone::clone
        pub fn new(capacity: usize) -> Self {
            Self {
                write: 0,
                read: 0,
                mem: Vec::with_capacity(capacity),
            }
        }

        /// change the delay length
        pub fn with_delay(mut self, len: usize, init: S) -> Self {
            let len = len.clamped(0, self.mem.capacity());
            self.mem.resize_with(len, || init);
            self.read = (self.write) % self.mem.len();
            self
        }

        /// increment the counters
        fn inc(self) -> Self {
            let (write, read) = (
                (self.read + 1) % self.mem.len(),
                (self.write + 1) % self.mem.len(),
            );
            Self {
                read,
                write,
                mem: self.mem,
            }
        }

        /// read the delay line at a fixed offset. If the offset is beyond the length of the delay
        /// line, it is clamped to the maximum length.
        pub fn read(&self, offset: usize) -> <Self as Process>::Output {
            let idx = (self.read + offset.clamped(0, self.mem.len())) % self.mem.len();
            unsafe { *self.mem.get_unchecked(idx) }
        }
    }

    impl<S> Process for DelayLine<S>
    where
        S: Signal + Copy + Sized,
    {
        type Input = S;
        type Output = S;

        fn evolve(mut self, i: S) -> Self {
            unsafe {
                *self.mem.get_unchecked_mut(self.write) = i;
            }
            self.inc()
        }

        fn observe(&self) -> Self::Output {
            self.read(0)
        }
    }
}

/// Filter objects
pub mod filter {
    use crate::{Process, Signal};
    use std::f64::MAX;

    /// A first order averaging filter.
    #[derive(Copy, Clone, Debug)]
    pub struct Averager {
        time: f64,
        average: f64,
    }

    impl Default for Averager {
        fn default() -> Self {
            Self {
                time: 100.0,
                average: 0.0,
            }
        }
    }

    impl Averager {
        pub fn new(time_samples: f64) -> Self {
            Self::default().with_time(time_samples)
        }

        pub fn with_time(self, time_samples: f64) -> Self {
            Self {
                time: time_samples.clamp_to(1.0, MAX),
                average: self.average,
            }
        }
    }

    impl Process for Averager {
        type Input = f64;
        type Output = f64;

        fn evolve(self, input: f64) -> Self {
            let coeff = 1.0 / self.time;
            Self {
                time: self.time,
                average: (1.0 - coeff) * self.average + coeff * input,
            }
        }

        fn observe(&self) -> f64 {
            self.average
        }
    }
}
