use std::iter::AdditiveIterator;

#[deriving(Show,PartialEq)]
enum Color {
    Green,
    Red,
}

static WORLD: [Color, ..5] = [Green, Red, Red, Green, Green];
static P_HIT: f32 = 0.6;
static P_MISS: f32 = 0.2;

fn sense(p: &Vec<f32>, z: Color) -> Vec<f32> {
    let mut q = p.clone();
    for i in range(0u, q.len()) {
        *q.get_mut(i) *= if z == WORLD[i] { P_HIT } else { P_MISS };
    }
    let sum = q.iter().map(|&x| x).sum();
    for i in range(0u, q.len()) {
        *q.get_mut(i) /= sum;
    }
    q
}

fn main() {
    let p: Vec<f32> = vec![0.2, 0.2, 0.2, 0.2, 0.2];
    let z: Color = Red;
    println!("{}", sense(&p, z));
}
