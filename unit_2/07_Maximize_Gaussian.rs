use std::num::{Float};

fn gaussian<F: Float>(mean: F, covar2: F, x: F) -> F {
    let two_pi: F = Float::two_pi();
    ((x - mean).powi(2) / -(covar2 + covar2)).exp() / (two_pi * covar2).sqrt()
}

fn main() {
    let x: f32 = 10.0;
    println!("{}", gaussian(10.0, 4.0, x))
}
