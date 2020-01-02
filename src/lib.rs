//! # DSP64 : double precision signal processsing
//!
pub mod proc;

/// Common methods for signal types. Currently implemented for `f64` and fixed sized arrays of `f64`,
/// and tuples of `f64`.
pub trait Signal
where
    Self: Sized,
{
    fn clamp_to(self, min: f64, max: f64) -> Self;
    fn clamp(self) -> Self;
    fn polyval(self, p: &[f64]) -> Self;
}

impl Signal for f64 {
    /// clamp to a fixed range.
    fn clamp_to(self, min: f64, max: f64) -> Self {
        self.max(min).min(max)
    }

    /// Clamp to unity gain (-1, 1)
    fn clamp(self) -> Self {
        self.clamp_to(-1.0, 1.0)
    }

    /// Evaluate a polynomial. Uses Horners method and some iterator tricks. Probably a faster
    /// way to do it, but it's safe and cool.
    fn polyval(self, p: &[f64]) -> Self {
        match p.split_first() {
            Some((p0, p)) => p.iter().rev().fold(0.0, |acc, coeff| (acc + *coeff) * self) + *p0,
            None => self,
        }
    }
}

impl<'a, S> Signal for &'a mut [S]
where
    S: Signal + Copy,
{
    fn clamp_to(self, min: f64, max: f64) -> Self {
        for x in self.iter_mut() {
            *x = x.clamp_to(min, max);
        }
        self
    }

    fn clamp(self) -> Self {
        for x in self.iter_mut() {
            *x = Signal::clamp(*x);
        }
        self
    }

    fn polyval(self, p: &[f64]) -> Self {
        for x in self.iter_mut() {
            *x = x.polyval(p);
        }
        self
    }
}

impl<A, B> Signal for (A, B)
where
    A: Signal,
    B: Signal,
{
    fn clamp_to(self, min: f64, max: f64) -> Self {
        (self.0.clamp_to(min, max), self.1.clamp_to(min, max))
    }

    fn clamp(self) -> Self {
        (self.0.clamp(), self.1.clamp())
    }

    fn polyval(self, p: &[f64]) -> Self {
        (self.0.polyval(p), self.1.polyval(p))
    }
}

/// An abstraction for processes. Based on the principle of state-space, but doesn't apply solely
/// to linear processes.
pub trait Process
where
    Self: Sized,
{
    /// The input type, could be one or more signal values.
    type Input: Signal + Copy;

    /// The output type, could be one or more signal values.
    type Output: Signal + Copy;

    /// Compute the next state of the process. Consumes the process itself.
    fn evolve(self, s: Self::Input) -> Self;

    /// Make an observation of the process (render the output).
    fn observe(&self) -> Self::Output;

    /// Like `zip` applied to iterators, this fuses two processes to create a new process that
    /// evaluates them both in parallel.
    fn zip<P>(self, p: P) -> PZip<Self, P>
    where
        P: Process<Input = Self::Input>,
    {
        PZip { a: self, b: p }
    }

    /// Like `chain` applied to iterators, this fuses two processes in series (evaluate one process
    /// before the other). Makes the output of the first process hidden.
    fn chain<P>(self, p: P) -> PChain<Self, P>
    where
        P: Process<Input = Self::Output>,
    {
        PChain {
            first: self,
            second: p,
        }
    }

    /// A process combinator that allows you to read the previous output of the process, the intended
    /// purpose is to allow for feedback when constructing chains/zips of processes.
    fn feedback(self, init: Self::Output) -> Feedback<Self> {
        Feedback { p: self, fb: init }
    }
}

#[derive(Copy, Clone, Debug)]
pub struct PZip<A, B>
where
    A: Process,
    B: Process<Input = A::Input>,
{
    a: A,
    b: B,
}

#[derive(Copy, Clone, Debug)]
pub struct PChain<A, B>
where
    A: Process,
    B: Process<Input = A::Output>,
{
    pub first: A,
    pub second: B,
}

#[derive(Copy, Clone, Debug)]
pub struct Feedback<P: Process> {
    fb: P::Output,
    p: P,
}

impl<P> Process for Feedback<P>
where
    P: Process,
{
    type Input = P::Input;
    type Output = (P::Output, P::Output);

    fn evolve(self, i: Self::Input) -> Self {
        let fb = self.p.observe();
        let p = self.p.evolve(i);
        Self { fb, p }
    }

    fn observe(&self) -> Self::Output {
        (self.p.observe(), self.fb)
    }
}

impl<A, B> Process for PZip<A, B>
where
    A: Process,
    B: Process<Input = A::Input>,
{
    type Input = A::Input;
    type Output = (A::Output, B::Output);

    fn evolve(self, i: Self::Input) -> Self {
        Self {
            a: self.a.evolve(i),
            b: self.b.evolve(i),
        }
    }

    fn observe(&self) -> Self::Output {
        (self.a.observe(), self.b.observe())
    }
}

impl<A, B> Process for PChain<A, B>
where
    A: Process,
    B: Process<Input = A::Output>,
{
    type Input = A::Input;
    type Output = B::Output;

    fn evolve(self, i: Self::Input) -> Self {
        let (a, b) = (self.first, self.second);
        let next_a = a.evolve(i);
        let i = next_a.observe();
        let next_b = b.evolve(i);

        Self {
            first: next_a,
            second: next_b,
        }
    }

    fn observe(&self) -> Self::Output {
        self.second.observe()
    }
}

macro_rules! impl_n {
    ($n:literal) => {
        impl<S> Signal for [S; $n]
        where
            S: Signal + Copy,
        {
            fn clamp_to(mut self, min: f64, max: f64) -> Self {
                for x in self.iter_mut() {
                    *x = x.clamp_to(min, max);
                }
                self
            }

            fn clamp(mut self) -> Self {
                for x in self.iter_mut() {
                    *x = Signal::clamp(*x);
                }
                self
            }

            fn polyval(mut self, p: &[f64]) -> Self {
                for x in self.iter_mut() {
                    *x = x.polyval(p);
                }
                self
            }
        }
    };
}

impl_n! {0}
impl_n! {1}
impl_n! {2}
impl_n! {3}
impl_n! {4}
impl_n! {5}
impl_n! {6}
impl_n! {7}
impl_n! {8}
impl_n! {9}
impl_n! {10}
impl_n! {11}
impl_n! {12}
impl_n! {13}
impl_n! {14}
impl_n! {15}
impl_n! {16}
impl_n! {17}
impl_n! {18}
impl_n! {19}
impl_n! {20}
impl_n! {21}
impl_n! {22}
impl_n! {23}
impl_n! {24}
impl_n! {25}
impl_n! {26}
impl_n! {27}
impl_n! {28}
impl_n! {29}
impl_n! {30}
impl_n! {31}
impl_n! {32}
