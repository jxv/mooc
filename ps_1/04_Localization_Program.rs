#[derive(Debug,PartialEq,Copy,Clone)]
enum Color {
    Green,
    Red,
}

static COLORS: [[Color;5];4] = [
    [Color::Red, Color::Green, Color::Green, Color::Red, Color::Red],
    [Color::Red, Color::Red, Color::Green, Color::Red, Color::Red],
    [Color::Red, Color::Red, Color::Green, Color::Green, Color::Red],
    [Color::Red, Color::Red, Color::Red, Color::Red, Color::Red]
];

static MEASUREMENTS: [Color;5] =
    [Color::Green, Color::Green, Color::Green, Color::Green, Color::Green];

static MOTIONS: [(i32,i32);5] = [(0,0), (0,1), (1,0), (1,0), (0,1)];

static SENSOR_RIGHT: f32 = 0.7;
static P_MOVE: f32 = 0.8;

fn sense(p: &Vec<Vec<f32>>, z: Color) -> Vec<Vec<f32>> {
    let mut q = p.clone();
    let mut sum = 0.0;

    // Apply probability from sensor data
    for i in 0..p.len() {
        for j in 0..p[i].len() {
            let s = p[i][j] * if z == COLORS[i][j] { SENSOR_RIGHT } else { 1.0 - SENSOR_RIGHT };
            q[i][j] = s;
            sum += s;
        }
    }

    // Normalize
    for i in 0..p.len() {
        for j in 0..p[i].len() {
            q[i][j] /= sum;
        }
    }
    q
}

fn modulo(n: i32, m: i32) -> i32 {
    let mut k = n % m;
    if k < 0 { k += m };
    k
}

// Originally named 'move' but 'move' is a reserved word
fn travel(p: &Vec<Vec<f32>>, (y,x): (i32,i32)) -> Vec<Vec<f32>> {
    let mut q = p.clone();
    let mut sum = 0.0;

    // Apply travel probabilties
    for i in 0..p.len() {
        for j in 0..p[i].len() {
            let r = modulo(i as i32 - y, p.len() as i32) as usize;
            let c = modulo(j as i32 - x, p[i].len() as i32) as usize;
            let s = P_MOVE * p[r][c] + (1.0 - P_MOVE) * p[i][j];
            q[i][j] = s;
            sum += s;
        }
    }

    // Normalize
    for i in 0..p.len() {
        for j in 0..p[i].len() {
            q[i][j] /= sum;
        }
    }
    q
}

fn main() {
    // Uniform distribution
    let rows = 4;
    let cols = 5;
    let u = 1.0 / (cols * rows) as f32;
    let mut p = vec![vec![u; cols]; rows];

    for i in 0..MOTIONS.len() {
        p = travel(&p, MOTIONS[i]);
        p = sense(&p, MEASUREMENTS[i]);
    }
    println!("{:?}", p);
}
