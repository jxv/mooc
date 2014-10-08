#[deriving(Show,PartialEq)]
enum Color {
    Green,
    Red,
}

static COLORS: [[Color, ..5], ..4] = [[Red, Green, Green, Red, Red],
                                      [Red, Red, Green, Red, Red],
                                      [Red, Red, Green, Green, Red],
                                      [Red, Red, Red, Red, Red]];
static MEASUREMENTS: [Color, ..5] = [Green, Green, Green, Green, Green];
static MOTIONS: [(int,int), ..5] = [(0,0), (0,1), (1,0), (1,0), (0,1)];

static SENSOR_RIGHT: f32 = 0.7;
static P_MOVE: f32 = 0.8;

fn sense(p: &Vec<Vec<f32>>, z: Color) -> Vec<Vec<f32>> {
    let mut q = p.clone();
    let mut sum = 0.0;

    // apply probability from sensor data
    for i in range(0, p.len()) {
        for j in range(0, p[i].len()) {
            let s = p[i][j] * if z == COLORS[i][j] { SENSOR_RIGHT } else { 1.0 - SENSOR_RIGHT };
            *(*q.get_mut(i)).get_mut(j) = s;
            sum += s;
        }
    }

    // normalize
    for i in range(0, p.len()) {
        for j in range(0, p[i].len()) {
            *q.get_mut(i).get_mut(j) /= sum;
        }
    }
    q
}

fn modulo(n: int, m: int) -> int {
    let mut k = n % m;
    if k < 0 { k += m };
    k
}

fn travel(p: &Vec<Vec<f32>>, (y,x): (int,int)) -> Vec<Vec<f32>> { // named as 'travel' because 'move' is a keyword
    let mut q = p.clone();
    let mut sum = 0.0;

    // apply travel probabilties
    for i in range(0, p.len()) {
        for j in range(0, p[i].len()) {
            let r = modulo(i as int - y, p.len() as int) as uint;
            let c = modulo(j as int - x, p[i].len() as int) as uint;
            let s = P_MOVE * p[r][c] + (1.0 - P_MOVE) * p[i][j];
            *(*q.get_mut(i)).get_mut(j) = s;
            sum += s;
        }
    }

    // normalize
    for i in range(0, p.len()) {
        for j in range(0, p[i].len()) {
            *(*q.get_mut(i)).get_mut(j) /= sum;
        }
    }
    q
}

fn main() {
    // uniform distribution
    let rows = 4;
    let cols = 5;
    let u = 1.0 / (cols * rows) as f32;
    let mut p = Vec::from_elem(rows, Vec::from_elem(cols, u));

    for i in range(0, MOTIONS.len()) {
        p = travel(&p, MOTIONS[i]);
        p = sense(&p, MEASUREMENTS[i]);
    }
    println!("{}", p);
}
