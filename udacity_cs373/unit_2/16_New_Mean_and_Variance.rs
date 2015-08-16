fn update<F: Float + FromPrimitive>(mean1: F, var1: F, mean2: F, var2: F) -> (F, F) {
    let one: F = FromPrimitive::from_f32(1.0).unwrap();
    let new_mean: F = (one / (var1 + var2)) * (var2 * mean1 + var1 * mean2);
    let new_var: F = one / ((one / var1) + (one / var2));
    (new_mean, new_var)
}

fn main() {
    println!("{}", update(10.0, 8.0, 13.0, 2.0 as f32))
}
