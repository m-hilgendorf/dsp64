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

    /// This function is a little magical, tl;dr it's a bilinear transform. 
    #[allow(non_snake_case)]
    fn transform_quadratic(Q: [f64; 3]) -> [f64; 3] {
        let T = [1.0, 1.0 / 4.0, 1.0 / 16.0];
        let X = [[1.0, 1.0, 1.0], [-2.0, 0.0, 2.0], [1.0, -1.0, 1.0]];
        let TQ = [T[0] * Q[0], T[1] * Q[1], T[2] * Q[2]];

        let mut R = [0.0; 3];
        for r in 0..3 {
            for c in 0..3 {
                R[r] += X[r][c] * TQ[c];
            }
        }
        R
    }

    /// create a second order filter from an analog biquadratic prototype, 
    /// then transform to Z-domain. 
    /// 
    /// The cutoff frequency should be "normalized" to the domain [0, 1] where 
    /// 1 is equal to the Nyquist frequency. 
    fn make_2nd_order_filter(
        gain: [f64; 3],
        normalized_frequency: f64,
        q_factor: f64,
    ) -> ([f64; 3], [f64; 3]) {
        normalized_frequency.clamp_to(0.0,1.0);
        let prewarped = 4.0 * (normalized_frequency * 0.5 * std::f64::consts::PI).tan();
        let prototype = [1.0, prewarped / q_factor, prewarped * prewarped];
        let numerator = [
            prototype[0] * gain[0],
            prototype[1] * gain[1],
            prototype[2] * gain[2],
        ];
        let (mut num, mut den) = (
            transform_quadratic(numerator),
            transform_quadratic(prototype),
        );
        let scale = 1.0 / den[0];
        for (a, b) in num.iter_mut().zip(den.iter_mut()) {
            *a *= scale;
            *b *= scale;
        }
        (num, den)
    }

    /// A second order filter stage. Currenlty only supports Lowpass, Bandpass, and Highpass 
    /// filter shapes. 
    #[derive(Default, Debug, Copy, Clone)]
    pub struct Biquad {
        num: [f64; 3],
        den: [f64; 3],
        z: [f64; 3],
    }

    impl Biquad {
        /// Zero the filter state. 
        pub fn reset(mut self) -> Self {
            self.z = [0.0; 3];
            self
        }

        /// Transform into a lowpass filter of given cutoff and q factor. Cutoff must 
        /// be normalized where 1.0 represents the nyquist frequency. 
        pub fn into_lowpass(self, cutoff: f64, q_factor: f64) -> Self {
            let (num, den) = make_2nd_order_filter([0.0, 0.0, 1.0], cutoff, q_factor);
            Self {
                num,
                den,
                z: self.z,
            }
        }
        
        /// Transform into a lowpass filter of given cutoff and q factor. Cutoff must 
        /// be normalized where 1.0 represents the nyquist frequency. 
        pub fn into_highpass(self, cutoff: f64, q_factor: f64) -> Self {
            let (num, den) = make_2nd_order_filter([1.0, 0.0, 0.0], cutoff, q_factor);
            Self {
                num,
                den,
                z: self.z,
            }
        }

        /// Transform into a lowpass filter of given cutoff and q factor. Cutoff must 
        /// be normalized where 1.0 represents the nyquist frequency. 
        pub fn into_bandpass(self, cutoff: f64, q_factor: f64) -> Self {
            let (num, den) = make_2nd_order_filter([0.0, 1.0, 0.0], cutoff, q_factor);
            Self {
                num,
                den,
                z: self.z,
            }
        }
    }

    impl Process for Biquad {
        type Input = f64;
        type Output = f64;

        fn evolve(self, x: f64) -> Self {
            let mut zn = [0.0; 3];
            zn[2] = self.z[1];
            zn[1] = self.z[0];
            zn[0] = x - self.den[1] * zn[1] - self.den[2] * zn[2];
            Self {
                num: self.num,
                den: self.den,
                z: zn,
            }
        }

        fn observe(&self) -> f64 {
            self.num
                .iter()
                .zip(self.z.iter())
                .fold(0.0, |acc, (a, z)| acc + *a * *z)
        }
    }
}

#[cfg(test)]
mod tests {
    /// Not a real test, this is a sanity check that I used to compare against results in matlab. 
    /// I do not recommend actually running this 
    #[test]
    fn biquad_ir() {
        use crate::proc::filter::Biquad;
        use crate::Process;
        let h = [
            [
                2.008337e-02,
                7.151723e-02,
                1.188426e-01,
                1.396477e-01,
                1.417727e-01,
                1.317465e-01,
                1.147325e-01,
                9.460373e-02,
                7.409425e-02,
                5.498821e-02,
                3.831713e-02,
                2.454696e-02,
                1.374350e-02,
                5.710621e-03,
                9.996700e-05,
                -3.506465e-03,
                -5.537770e-03,
                -6.395682e-03,
                -6.432118e-03,
                -5.938772e-03,
                -5.145281e-03,
                -4.223037e-03,
                -3.292303e-03,
                -2.430893e-03,
                -1.683144e-03,
                -1.068362e-03,
                -5.882449e-04,
                -2.330655e-04,
                1.345240e-05,
                1.704763e-04,
                2.574889e-04,
                2.926096e-04,
            ],
            [
                6.745527e-02,
                2.120106e-01,
                2.819336e-01,
                2.347263e-01,
                1.519050e-01,
                7.672900e-02,
                2.499314e-02,
                -3.107177e-03,
                -1.386865e-02,
                -1.456895e-02,
                -1.092703e-02,
                -6.475291e-03,
                -2.890438e-03,
                -6.307033e-04,
                4.722957e-04,
                8.001801e-04,
                7.196258e-04,
                4.922027e-04,
                2.655154e-04,
                1.002968e-04,
                5.032162e-06,
                -3.565103e-05,
                -4.282572e-05,
                -3.423216e-05,
                -2.144816e-05,
                -1.038374e-05,
                -3.014581e-06,
                8.408194e-07,
                2.205464e-06,
                2.173711e-06,
                1.574090e-06,
                9.018428e-07,
            ],
            [
                1.311064e-01,
                3.602529e-01,
                3.648105e-01,
                1.747351e-01,
                3.135817e-02,
                -2.411621e-02,
                -2.657000e-02,
                -1.330397e-02,
                -2.715812e-03,
                1.590684e-03,
                1.928781e-03,
                1.009314e-03,
                2.297108e-04,
                -1.029750e-04,
                -1.395343e-04,
                -7.631090e-05,
                -1.908115e-05,
                6.504293e-06,
                1.005801e-05,
                5.750708e-06,
                1.562376e-06,
                -3.971010e-07,
                -7.222498e-07,
                -4.319938e-07,
                -1.264331e-07,
                2.304987e-08,
                5.165341e-08,
                3.235135e-08,
                1.013116e-08,
                -1.230551e-09,
                -3.678045e-09,
                -2.415428e-09,
            ],
            [
                2.065721e-01,
                4.894782e-01,
                3.469976e-01,
                3.237760e-02,
                -5.598318e-02,
                -2.702736e-02,
                9.750367e-04,
                5.652684e-03,
                1.897894e-03,
                -4.055606e-04,
                -5.215032e-04,
                -1.132946e-04,
                6.025308e-05,
                4.445002e-05,
                4.627001e-06,
                -6.994209e-06,
                -3.490591e-06,
                7.970703e-08,
                7.129665e-07,
                2.478528e-07,
                -4.802167e-08,
                -6.627879e-08,
                -1.508843e-08,
                7.402840e-09,
                5.690104e-09,
                6.530566e-10,
                -8.728894e-10,
                -4.504353e-10,
                4.477290e-12,
                8.985679e-11,
                3.232782e-11,
                -5.649357e-12,
            ],
            [
                2.928932e-01,
                5.857864e-01,
                2.426407e-01,
                -1.005051e-01,
                -4.163056e-02,
                1.724394e-02,
                7.142675e-03,
                -2.958593e-03,
                -1.225489e-03,
                5.076143e-04,
                2.102607e-04,
                -8.709284e-05,
                -3.607504e-05,
                1.494277e-05,
                6.189498e-06,
                -2.563774e-06,
                -1.061950e-06,
                4.398741e-07,
                1.822018e-07,
                -7.547046e-08,
                -3.126089e-08,
                1.294868e-08,
                5.363520e-09,
                -2.221643e-09,
                -9.202346e-10,
                3.811736e-10,
                1.578873e-10,
                -6.539906e-11,
                -2.708918e-11,
                1.122070e-11,
                4.647768e-12,
                -1.925169e-12,
            ],
            [
                3.913358e-01,
                6.380623e-01,
                7.892460e-02,
                -1.541074e-01,
                4.149223e-02,
                1.484414e-02,
                -1.361015e-02,
                2.122606e-03,
                1.880719e-03,
                -1.110617e-03,
                4.212896e-05,
                2.019084e-04,
                -8.286021e-05,
                -8.917730e-06,
                1.952068e-05,
                -5.467193e-06,
                -1.802178e-06,
                1.736516e-06,
                -2.887956e-07,
                -2.333193e-07,
                1.427686e-07,
                -7.069315e-09,
                -2.534403e-08,
                1.074959e-08,
                9.904891e-10,
                -2.470952e-09,
                7.191312e-10,
                2.181126e-10,
                -2.214158e-10,
                3.910931e-11,
                2.890473e-11,
                -1.833931e-11,
            ],
            [
                5.050010e-01,
                6.323678e-01,
                -1.053456e-01,
                -9.336368e-02,
                9.849298e-02,
                -4.823700e-02,
                9.259844e-03,
                6.206420e-03,
                -7.161762e-03,
                3.666008e-03,
                -7.918623e-04,
                -4.057960e-04,
                5.190066e-04,
                -2.776438e-04,
                6.633767e-05,
                2.597219e-05,
                -3.747983e-05,
                2.095699e-05,
                -5.468843e-06,
                -1.615265e-06,
                2.696578e-06,
                -1.576773e-06,
                4.450448e-07,
                9.642148e-08,
                -1.932508e-07,
                1.182635e-07,
                -3.583040e-08,
                -5.399501e-09,
                1.379126e-08,
                -8.843128e-09,
                2.858609e-09,
                2.695945e-10,
            ],
            [
                6.389455e-01,
                5.475888e-01,
                -2.506955e-01,
                6.049455e-02,
                3.434341e-02,
                -6.422610e-02,
                5.923216e-02,
                -4.118857e-02,
                2.262660e-02,
                -8.859057e-03,
                7.854319e-04,
                2.759299e-03,
                -3.478053e-03,
                2.836304e-03,
                -1.806094e-03,
                8.934994e-04,
                -2.756940e-04,
                -5.372515e-05,
                1.752137e-04,
                -1.780880e-04,
                1.312226e-04,
                -7.646990e-05,
                3.323469e-05,
                -6.419703e-06,
                -6.381736e-06,
                9.944264e-06,
                -8.731709e-06,
                5.875165e-06,
                -3.110736e-06,
                1.130233e-06,
                -7.717270e-09,
                -4.577412e-07,
            ],
            [
                8.005924e-01,
                3.514456e-01,
                -2.614817e-01,
                1.827775e-01,
                -1.176173e-01,
                6.637805e-02,
                -2.818332e-02,
                1.423016e-03,
                1.585406e-02,
                -2.566113e-02,
                2.988947e-02,
                -3.020019e-02,
                2.797339e-02,
                -2.429802e-02,
                1.998888e-02,
                -1.561943e-02,
                1.156231e-02,
                -8.031431e-03,
                5.121704e-03,
                -2.844102e-03,
                1.154881e-03,
                2.127817e-05,
                -7.739006e-04,
                1.194426e-03,
                -1.368178e-03,
                1.369704e-03,
                -1.260650e-03,
                1.089435e-03,
                -8.921081e-04,
                6.938861e-04,
                -5.110139e-04,
                3.526769e-04,
            ],
        ];
        let q = 2.0f64.sqrt() / 2.0;
        let mut dirac = [0.0; 32];
        dirac[0] = 1.0;
        for (f, h) in (&[0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9])
            .iter()
            .zip(h.iter())
        {
            let mut filter = Biquad::default().into_lowpass(*f, q);
            println!("{:?}", filter);
            let output = dirac
                .iter()
                .map(|x| {
                    filter = filter.evolve(*x);
                    filter.observe()
                })
                .collect::<Vec<f64>>();
            println!("{:?}", output)
        }
    }
}
