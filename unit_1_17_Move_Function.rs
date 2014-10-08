use std::iter::AdditiveIterator;

#[deriving(Show,PartialEq)]
enum Color {
    Green,
    Red,
}

#[allow(dead_code)]
static WORLD: [Color, ..5] = [Green, Red, Red, Green, Green];
#[allow(dead_code)]
static P_HIT: f32 = 0.6;
#[allow(dead_code)]
static P_MISS: f32 = 0.2;

#[allow(dead_code)]
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

fn travel(p: &Vec<f32>, u: int) -> Vec<f32> { // named as 'travel' because 'move' is a keyword
    let mut q = Vec::new();
    for i in range(0, p.len()) {
        let j = i as int - u;
        let mut k = j % p.len() as int;
        while k < 0 {
            k += p.len() as int;
        }
        q.push(p[k as uint]);
    }
    q
}

fn main() {
    let p: Vec<f32> = vec![0.0, 1.0, 0.0, 0.0, 0.0];
    println!("{}", travel(&p, 1));
}
