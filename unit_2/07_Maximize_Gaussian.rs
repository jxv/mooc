fn gaussian<F: Float + FromPrimitive>(mean: F, covar2: F, x: F) -> F {
    let two: F = FromPrimitive::from_f32(2.0).unwrap();
    let a: F = two * Float::pi() * covar2;
    let b: F = (x - mean).powi(2) / (-two * covar2);
    b.exp() / a.sqrt()
}

fn main() {
    let x: f32 = 10.0;
    println!("{}", gaussian(10.0, 4.0, x))
}
