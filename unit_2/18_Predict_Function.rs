#[allow(dead_code)]
fn update<F: Float + FromPrimitive>(m1: F, v1: F, m2: F, v2: F) -> (F, F) {
    let one: F = FromPrimitive::from_f32(1.0).unwrap();
    let new_mean: F = (one / (v1 + v2)) * (v2 * m1 + v1 * m2);
    let new_var: F = one / ((one / v1) + (one / v2));
    (new_mean, new_var)
}

fn predict<F: Float + FromPrimitive>(m1: F, v1: F, m2: F, v2: F) -> (F, F) {
    (m1 + m2, v1 + v2)
}

fn main() {
    println!("{}", predict(10.0, 4.0, 12.0, 4.0 as f32))
}
