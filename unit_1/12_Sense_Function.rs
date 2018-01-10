#[derive(Debug,PartialEq)]
enum Color {
    Green,
    Red,
}

static WORLD: [Color;5] = [Color::Green, Color::Red, Color::Red, Color::Green, Color::Green];
static P_HIT: f32 = 0.6;
static P_MISS: f32 = 0.2;

fn sense(p: &Vec<f32>, z: Color) -> Vec<f32> {
    let mut q = p.clone();
    for i in 0..q.len() {
        q[i] *= if z == WORLD[i] { P_HIT } else { P_MISS };
    }
    q
}

fn main() {
    let p: Vec<f32> = vec![0.2, 0.2, 0.2, 0.2, 0.2];
    let z: Color = Color::Red;
    println!("{:?}", sense(&p, z));
}
