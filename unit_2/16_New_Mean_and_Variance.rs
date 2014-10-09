fn update<F: Float + FromPrimitive>(m1: F, v1: F, m2: F, v2: F) -> (F, F) {
    let one: F = FromPrimitive::from_f32(1.0).unwrap();
    let new_mean: F = (one / (v1 + v2)) * (v2 * m1 + v1 * m2);
    let new_var: F = one / ((one / v1) + (one / v2));
    (new_mean, new_var)
}

fn main() {
    println!("{}", update(10.0, 8.0, 13.0, 2.0 as f32))
}
