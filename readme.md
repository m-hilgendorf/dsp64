# DSP64: Opinionated DSP 

This crate was developed to explore various ideas and patterns for writing functional DSP (as in functional programming) leveraging Rust's type system and borrow checking semantics. I'm interesting in building useful and interesting abstractions on top of a fundamental state-space model of the universe. 

## Example: Measuring an impulse response

```rust
use dsp64::filter::Biquad; 
use dsp64::siggen::KronikerDelta;
use dsp64::Observer; 

let impulse_resonse: Vec<f64> =
    (0..32)
        .zip(Observer::new(
            KronikerDelta::default(), 
            Biquad::default()
                .into_lowpass(0.1, 4.0)))
        .map(|(_,h)| h.1)
        .collect();

println("{:?}", impulse_response);
```

