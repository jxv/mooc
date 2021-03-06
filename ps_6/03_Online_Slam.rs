#![feature(tuple_indexing)]
#![allow(unused_variable, dead_code, unused_imports)]
use std::iter::{range_step_inclusive, AdditiveIterator};
use std::rand::{task_rng, random};


static NUM_LANDMARKS: uint = 5;
static N: uint = 20;
static WORLD_SIZE: f32 = 100.0;
static MEASUREMENT_RANGE: f32 = 50.0;
static MOTION_NOISE: f32 = 2.0;
static MEASUREMENT_NOISE: f32 = 2.0;
static DISTANCE: f32 = 20.0;


#[deriving(Show,Clone,PartialEq)]
struct Matrix {
    value: Vec<Vec<f32>>,
    dimx: uint,
    dimy: uint,
}


#[deriving(Show,Clone)]
struct Robot {
    x: f32,
    y: f32,
    world_size: f32,
    measurement_range: f32,
    motion_noise: f32,
    measurement_noise: f32,
    landmarks: Vec<Vec<f32>>,
    num_landmarks: uint,
}

#[deriving(Show,Clone)]
struct Step {
    measurements: Vec<(uint,Vec<f32>)>,
    motion: Vec<f32>,
}

impl Matrix {
    fn new(value: Vec<Vec<f32>>) -> Matrix {
        assert!(value.len() > 0);
        assert!(value[0].len() > 0);
        for i in range(1, value.len()) {
            assert!(value[0].len() == value[i].len());
        }

        Matrix {
            value: value.clone(),
            dimx: value.len(),
            dimy: value[0].len(),
        }
    }

    fn zero(dimx: uint, dimy: uint) -> Matrix {
        assert!(dimx > 0);
        assert!(dimy > 0);
        Matrix {
            value: Vec::from_elem(dimx, Vec::from_elem(dimy, 0.0)),
            dimx: dimx,
            dimy: dimy,
        }
    }

#[allow(dead_code)]
    fn identity(dim: uint) -> Matrix {
        assert!(dim > 0);
        Matrix {
            value: Vec::from_fn(dim, |col| {
                Vec::from_fn(dim, |row| {
                    if row == col { 1.0 } else { 0.0 }
                })
            }),
            dimx: dim,
            dimy: dim,
        }
    }

    fn add(&self, other: &Matrix) -> Matrix {
        assert!(self.dimx == other.dimx);
        assert!(self.dimy == other.dimy);
        let mut res = Matrix::zero(self.dimx, self.dimy);
        for i in range(0, self.dimx) {
            for j in range(0, self.dimy) {
                *res.value.get_mut(i).get_mut(j) = self.value[i][j] + other.value[i][j];
            }
        }
        res
    }
    
#[allow(dead_code)]
    fn sub(&self, other: &Matrix) -> Matrix {
        assert!(self.dimx == other.dimx);
        assert!(self.dimy == other.dimy);
        let mut res = Matrix::zero(self.dimx, self.dimy);
        for i in range(0, self.dimx) {
            for j in range(0, self.dimy) {
                *res.value.get_mut(i).get_mut(j) = self.value[i][j] - other.value[i][j];
            }
        }
        res
    }

    fn mul(&self, other: &Matrix) -> Matrix {
        assert!(self.dimy == other.dimx);
        let mut res = Matrix::zero(self.dimx, other.dimy);
        for i in range(0, self.dimx) {
            for j in range(0, other.dimy) {
                for k in range(0, self.dimy) {
                    *res.value.get_mut(i).get_mut(j) += self.value[i][k] * other.value[k][j];
                }
            }
        }
        res
    }

#[allow(dead_code)]
    fn transpose(&self) -> Matrix {
        let mut res = Matrix::zero(self.dimy, self.dimx);
        for i in range(0, self.dimx) {
            for j in range(0, self.dimy) {
                *res.value.get_mut(j).get_mut(i) = self.value[i][j];
            }
        }
        res
    }

    fn cholesky(&self, ztol: f32) -> Matrix {
        let mut res = Matrix::zero(self.dimx, self.dimx);
        for i in range(0, self.dimx) {

            let s = Vec::from_fn(i, |k| {
                res.value[k][i].powi(2)
            }).iter().map(|&x| x).sum();

            let d = self.value[i][i] - s;
            *res.value.get_mut(i).get_mut(i) = if d.abs() < ztol {
                0.0
            } else { 
                if d < 0.0 {
                    fail!("matrix not positive-definite");
                }
                d.sqrt()
            };

            for j in range(i+1, self.dimx) {
                let mut s: f32 = Vec::from_fn(i, |k| {
                    res.value[k][i] * res.value[k][j]
                }).iter().map(|&x| x).sum();
                if s.abs() < ztol {
                    s = 0.0;
                }

                *res.value.get_mut(i).get_mut(j) = (self.value[i][j] - s) / res.value[i][i];
            }
        }
        res
    }

    fn cholesky_inverse(&self) -> Matrix {
        let mut res = Matrix::zero(self.dimx, self.dimx);

        // backward step for inverse
        for j_ in range_step_inclusive(self.dimx as int -1, 0, -1i) {
            let j = j_ as uint;
            // tjj
            let tjj = self.value[j][j];

            // s
            let mut tmp = Vec::new();
            for k in range(j+1, self.dimx) {
                tmp.push(self.value[j][k] * res.value[j][k]);
            }
            let s: f32 = tmp.iter().map(|&x|x).sum();

            // value[j][i] = 1 / tjj^2 - s / ttj
            *res.value.get_mut(j).get_mut(j) = (1.0 / tjj.powi(2)) - (s / tjj);

            for i_ in range_step_inclusive(j_-1, 0, -1i) {
                let i = i_ as uint;

                let mut tmp = Vec::new();
                for k in range(i+1, self.dimx) {
                    tmp.push(self.value[i][k] * res.value[k][j]);
                }
                let v = -tmp.iter().map(|&x|x).sum() / self.value[i][i];

                *res.value.get_mut(i).get_mut(j) = v;
                *res.value.get_mut(j).get_mut(i) = v;
            }
        }
        res
    }
    
    fn inverse(&self) -> Matrix {
        let aux = self.cholesky(0.00001);
        aux.cholesky_inverse()
    }

    fn take(&self, list1: &Vec<uint>, list2: &Vec<uint>) -> Matrix {
        let list2 = if list2.len() == 0 { list1.clone() } else { list2.clone() };

        if list1.len() > self.dimx || list2.len() > self.dimy {
           fail!("list invalid in take()");
        }

        let mut res = Matrix::zero(list1.len(), list2.len());
        for i in range(0u, list1.len()) {
            for j in range(0u, list2.len()) {
                *res.value.get_mut(i).get_mut(j) = self.value [list1[i]] [list2[j]];
            }
        }
        res
    }

    fn expand(&self, dimx: uint, dimy: uint, v1: &Vec<uint>, v2: &Vec<uint>) -> Matrix {
        let mut res = Matrix::zero(dimx, dimy);
        for i in range(0u, v1.len()) {
            for j in range(0u, v2.len()) {
                *res.value.get_mut(v1[i]).get_mut(v1[j]) = self.value[i][j];
            }
        }
        res
    }

    fn print(&self, prefix: String) {
        for row in self.value.iter() {
            println!("{}{}", prefix, row);
        }
        println!("");
    }

}


impl Robot {
    
    fn new() -> Robot {
        Robot::new_extra(100.0, 30.0, 1.0, 1.0)
    }

    fn new_extra(world_size: f32, measurement_range: f32,
            motion_noise: f32, measurement_noise: f32) -> Robot {
        Robot {
            measurement_noise: 0.0,
            world_size: world_size,
            measurement_range: measurement_range,
            x: world_size / 2.0,
            y: world_size / 2.0,
            motion_noise: motion_noise,
            landmarks: Vec::new(),
            num_landmarks: 0,
        }
    }

    fn rand(&self) -> f32 {
        random::<f32>() * 2.0 - 1.0
    }

    fn make_landmarks(&mut self, num_landmarks: uint) {
        self.landmarks = Vec::with_capacity(num_landmarks);
        for _ in range(0u, num_landmarks) {
            self.landmarks.push(vec![(random::<f32>() * self.world_size).round(),
                                     (random::<f32>() * self.world_size).round()]);
        }
        self.num_landmarks = num_landmarks;
    }

    fn travel(&mut self, dx: f32, dy: f32) -> bool {
        let x = self.x + dx + self.rand() * self.motion_noise;
        let y = self.y + dy + self.rand() * self.motion_noise;
        if x < 0.0 || x > self.world_size || y < 0.0 || y > self.world_size {
            false
        } else {
            self.x = x;
            self.y = y;
            true
        }
    }

    fn sense(&self) -> Vec<(uint, Vec<f32>)> {
        let mut z = Vec::new();
        for i in range(0u, self.num_landmarks) {
            let dx = self.landmarks[i][0] - self.x + self.rand() * self.measurement_noise;
            let dy = self.landmarks[i][1] - self.y + self.rand() * self.measurement_noise;
            if self.measurement_noise < 0.0 || dx.abs() + dy.abs() <= self.measurement_range {
                z.push((i, vec![dx, dy]));
            }
        }
        z
    }
    
    fn print(&self) {
        print!("[x={:.5f} y={:.5f}]", self.x, self.y);
    }
}


fn make_data(n: uint, num_landmarks: uint, world_size: f32, measurement_range: f32,
        motion_noise: f32, measurement_noise: f32, distance: f32) -> Vec<Step> {
    let mut complete = false;
    let mut data = Vec::new();
    let mut r = Robot::new();
    while !complete {
        r = Robot::new_extra(world_size, measurement_range, motion_noise, measurement_noise);
        r.make_landmarks(num_landmarks);
        let mut seen = Vec::from_elem(num_landmarks, false);

        let mut orientation = random::<f32>() * Float::two_pi();
        let mut dx = orientation.cos() * distance;
        let mut dy = orientation.sin() * distance;

        for k in range(0u, n - 1) {
            let z = r.sense();
            for i in range(0u, z.len()) {
                *seen.get_mut(z[i].0) = true;
            }

            while !r.travel(dx, dy) {
                orientation = random::<f32>() * Float::two_pi();
                dx = orientation.cos() * distance;
                dy = orientation.sin() * distance;
            }
            let step = Step { measurements: z, motion: vec![dx, dy] };
            data.push(step);
        }
        complete = true;
        for &s in seen.iter() { complete = complete && s };
    }
    println!("Landmarks: {}", r.landmarks);
    r.print();
    println!("");
    data
}


fn print_result(n: uint, num_landmarks: uint, result: &Matrix) {
    println!("\nestimated pose(s):");
    for i in range(0u, n) {
        println!("    [{:.3f}, {:.3f}]",
                result.value[2 * i][0], result.value[2 * i + 1][0]);
    }
    println!("\nestimated landmarks:");
    for i in range(0u, num_landmarks) {
        println!("    [{:.3f}, {:.3f}]",
                result.value[2 * (n + i)][0], result.value[2 * (n + i) + 1][0]);
    }
}


fn main() {
    let data = make_data(N, NUM_LANDMARKS, WORLD_SIZE, MEASUREMENT_RANGE, MOTION_NOISE,
            MEASUREMENT_NOISE, DISTANCE);
    let result_offline = slam(&data, N, NUM_LANDMARKS, MOTION_NOISE, MEASUREMENT_NOISE);
    let result_online = online_slam(&data, N, NUM_LANDMARKS, MOTION_NOISE, MEASUREMENT_NOISE);
    print_result(N, NUM_LANDMARKS, &result_offline);
    print_result(1, NUM_LANDMARKS, &result_online);
}


fn slam(data: &Vec<Step>, n: uint, num_landmarks: uint,
        motion_noise: f32, measurement_noise: f32) -> Matrix { 
    let dim = (n + num_landmarks) * 2;

    let mut omega = Matrix::zero(dim,dim);
    *omega.value.get_mut(0).get_mut(0) = 1.0;
    *omega.value.get_mut(1).get_mut(1) = 1.0;

    let mut xi = Matrix::zero(dim, 1);
    *xi.value.get_mut(0).get_mut(0) = WORLD_SIZE / 2.0;
    *xi.value.get_mut(1).get_mut(0) = WORLD_SIZE / 2.0;

    for k in range(0u, data.len()) {
        let p = k * 2;
        let measurements = &data[k].measurements;
        let motion = &data[k].motion;
        // update info mat/vec on measurement
        for i in range(0u, measurements.len()) {
            let m = 2 * (n + measurements[i].0);
            for b in range(0u, 2u) {
                *omega.value.get_mut(p+b).get_mut(p+b) +=  1.0 / measurement_noise;
                *omega.value.get_mut(m+b).get_mut(m+b) +=  1.0 / measurement_noise;
                *omega.value.get_mut(p+b).get_mut(m+b) += -1.0 / measurement_noise;
                *omega.value.get_mut(m+b).get_mut(p+b) += -1.0 / measurement_noise;
                *xi.value.get_mut(p+b).get_mut(0) += -measurements[i].1[b] / measurement_noise;
                *xi.value.get_mut(m+b).get_mut(0) +=  measurements[i].1[b] / measurement_noise;
            }
        }
        // update info mat/vec on motion
        for b in range(0u, 4u) {
            *omega.value.get_mut(p+b).get_mut(p+b) += 1.0 / motion_noise;
        }
        for b in range(0u, 2u) {
            *omega.value.get_mut(p+b  ).get_mut(p+b+2) += -1.0 / motion_noise;
            *omega.value.get_mut(p+b+2).get_mut(p+b  ) += -1.0 / motion_noise;
            *xi.value.get_mut(p+b  ).get_mut(0) += -motion[b] / motion_noise;
            *xi.value.get_mut(p+b+2).get_mut(0) +=  motion[b] / motion_noise;
        }
    }

    let mu = omega.inverse().mul(&xi);
    mu
}


fn online_slam(data: &Vec<Step>, n: uint, num_landmarks: uint,
        motion_noise: f32, measurement_noise: f32) -> Matrix { 
    let dim = (1 + num_landmarks) * 2;

    let mut omega = Matrix::zero(dim,dim);
    *omega.value.get_mut(0).get_mut(0) = 1.0;
    *omega.value.get_mut(1).get_mut(1) = 1.0;

    let mut xi = Matrix::zero(dim, 1);
    *xi.value.get_mut(0).get_mut(0) = WORLD_SIZE / 2.0;
    *xi.value.get_mut(1).get_mut(0) = WORLD_SIZE / 2.0;

    for k in range(0u, data.len()) {
        let measurements = &data[k].measurements;
        let motion = &data[k].motion;

        for i in range(0, measurements.len()) {
            // idx of lm coord in mat/vec
            let m = 2 * (1 + measurements[i].0);

            // update the info mat/vec on measurement
            for b in range(0u, 2u) {
                *omega.value.get_mut(b).get_mut(b)     += 1.0 / measurement_noise;
                *omega.value.get_mut(m+b).get_mut(m+b) += 1.0 / measurement_noise;
                *omega.value.get_mut(b).get_mut(m+b)   -= 1.0 / measurement_noise;
                *omega.value.get_mut(m+b).get_mut(b)   -= 1.0 / measurement_noise;
                *xi.value.get_mut(b).get_mut(0)   -= measurements[i].1[b] / measurement_noise;
                *xi.value.get_mut(m+b).get_mut(0) += measurements[i].1[b] / measurement_noise;
            }
        }

        // expand the info mat/vec by one new pose
        let mut list = vec![0, 1];
        for i in range(4, dim+2) {
            list.push(i);
        }
        omega = omega.expand(dim + 2, dim + 2, &list, &list);
        xi = xi.expand(dim + 2, 1, &list, &vec![0]);

        // update the info mat/vec on motion
        for b in range(0u, 4u) {
            *omega.value.get_mut(b).get_mut(b) += 1.0 / motion_noise;
        }
        for b in range(0u, 2u) {
            *omega.value.get_mut(b).get_mut(b+2) -= 1.0 / motion_noise;
            *omega.value.get_mut(b+2).get_mut(b) -= 1.0 / motion_noise;
            *xi.value.get_mut(b).get_mut(0)   -= motion[b] / motion_noise;
            *xi.value.get_mut(b+2).get_mut(0) += motion[b] / motion_noise;
        }

        // rm previous pose
        let list = Vec::from_fn(omega.value.len() - 2, |u: uint| 2 + u);
        let a = omega.take(&vec![0, 1], &list);
        let b = omega.take(&vec![0, 1], &vec![]);
        let c = xi.take(&vec![0, 1], &vec![0]);

        omega = omega.take(&list, &vec![]).sub(&a.transpose().mul(&b.inverse()).mul(&a));
        xi = xi.take(&list, &vec![0]).sub(&a.transpose().mul(&b.inverse()).mul(&c));
    }

    let mu = omega.inverse().mul(&xi);
    mu
}
