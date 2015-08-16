fn update<F: Float + FromPrimitive>(mean1: F, var1: F, mean2: F, var2: F) -> (F, F) {
    let one: F = FromPrimitive::from_f32(1.0).unwrap();
    let new_mean: F = (one / (var1 + var2)) * (var2 * mean1 + var1 * mean2);
    let new_var: F = one / ((one / var1) + (one / var2));
    (new_mean, new_var)
}

fn predict<F: Float + FromPrimitive>(mean1: F, var1: F, mean2: F, var2: F) -> (F, F) {
    (mean1 + mean2, var1 + var2)
}

static MEASUREMENTS: [f32, ..5] = [5.0, 6.0, 7.0, 9.0, 10.0];
static MOTION: [f32, ..5] = [1.0, 1.0, 2.0, 1.0, 1.0];
static MEASUREMENT_SIG: f32 = 4.0;
static MOTION_SIG: f32 = 2.0;

fn main() {
    let (mut mu, mut sig): (f32, f32) = (0.0, 10000.0);
    for i in range(0, MEASUREMENTS.len()) {
        let (m, s) = update(mu, sig, MEASUREMENTS[i], MEASUREMENT_SIG);
        mu = m;
        sig = s;

        let (m, s) = predict(mu, sig, MOTION[i], MOTION_SIG);
        mu = m;
        sig = s;
    }
    println!("{}", (mu, sig))
}
