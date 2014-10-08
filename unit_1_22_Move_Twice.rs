use std::iter::AdditiveIterator;

#[deriving(Show,PartialEq)]
enum Color {
    Green,
    Red,
}

#[allow(dead_code)]
static WORLD: [Color, ..5] = [Green, Red, Red, Green, Green];
#[allow(dead_code)]
static MEASUREMENTS: [Color, ..2] = [Red, Green];
#[allow(dead_code)]
static P_HIT: f32 = 0.6;
#[allow(dead_code)]
static P_MISS: f32 = 0.2;

static P_UNDERSHOOT: f32 = 0.1;
static P_EXACT: f32 = 0.8;
static P_OVERSHOOT: f32 = 0.1;

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
        let shift = |x: int| {
            let mut k = x % p.len() as int;
            while k < 0 {
                k += p.len() as int;
            }
            k
        };
        let under = p[shift(j+1) as uint] * P_UNDERSHOOT;
        let exact = p[shift(j) as uint] * P_EXACT;
        let over  = p[shift(j-1) as uint] * P_OVERSHOOT;
        q.push(under + exact + over);
    }
    q
}

fn main() {
    let mut p: Vec<f32> = vec![0.0, 1.0, 0.0, 0.0, 0.0];
    p = travel(&p, 1);
    p = travel(&p, 1);
    println!("{}", p);
}
