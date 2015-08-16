use std::iter::AdditiveIterator;

fn main() {
    let mut p: Vec<f32> = vec![0.2, 0.2, 0.2, 0.2, 0.2];
    let p_hit: f32 = 0.6;
    let p_miss: f32 = 0.2;

    let hits: Vec<uint> = vec![1,2];
    for i in range(0u, p.len()) {
        *p.get_mut(i) *= if hits.contains(&i) { p_hit } else { p_miss };
    }

    println!("{}", p.into_iter().sum());
}
