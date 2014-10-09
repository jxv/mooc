#[allow(dead_code)]
fn update<F: Float + FromPrimitive>(mean1: F, var1: F, mean2: F, var2: F) -> (F, F) {
    let one: F = FromPrimitive::from_f32(1.0).unwrap();
    let new_mean: F = (one / (var1 + var2)) * (var2 * mean1 + var1 * mean2);
    let new_var: F = one / ((one / var1) + (one / var2));
    (new_mean, new_var)
}

fn predict<F: Float + FromPrimitive>(mean1: F, var1: F, mean2: F, var2: F) -> (F, F) {
    (mean1 + mean2, var1 + var2)
}

fn main() {
    println!("{}", predict(10.0, 4.0, 12.0, 4.0 as f32))
}
