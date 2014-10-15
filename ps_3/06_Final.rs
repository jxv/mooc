extern crate core;

use std::rand::{random, task_rng};
use std::rand::distributions::{Normal, IndependentSample};
use std::num::{Float, FloatMath, FromPrimitive, abs};

// static MAX_STEERING_ANGLE: f32 = core::f32::const::FRAC_PI_4;
static LENGTH: f32 = 20.0;
static BEARING_NOISE: f32= 0.1;
static STEERING_NOISE: f32 = 0.1;
static DISTANCE_NOISE: f32 = 5.0;

static TOLERANCE_XY: f32 = 15.0;
static TOLERANCE_ORIENTATION: f32 = 0.25;

static LANDMARKS: [(f32,f32), ..4] = [(0.0, 100.0), (0.0, 0.0), (100.0, 0.0), (100.0, 100.0)];
static WORLD_SIZE: f32 = 100.0;


#[deriving(Show,Clone)]
struct Robot {
    x: f32,
    y: f32,
    orientation: f32,
    bearing_noise: f32,
    steering_noise: f32,
    distance_noise: f32,
    length: f32,
}

fn modulo(n: f32, m: f32) -> f32 {
    let mut k = n % m;
    while k < 0.0 { k += m };
    k
}

fn gaussian<F: Float>(mean: F, covar2: F, x: F) -> F {
    let two_pi: F = Float::two_pi();
    ((x - mean).powi(2) / -(covar2 + covar2)).exp() / (two_pi * covar2).sqrt()
}

fn gauss(mean: f32, covar2: f32) -> f32 {
    let cvt_covar2: f64 = FromPrimitive::from_f32(covar2).unwrap();
    let cvt_mean: f64 = FromPrimitive::from_f32(mean).unwrap();
    FromPrimitive::from_f64(Normal::new(cvt_mean, cvt_covar2).ind_sample(&mut task_rng())).unwrap()
}

impl Robot {
    fn new() -> Robot {
        Robot {
            x: random::<f32>() * WORLD_SIZE,
            y: random::<f32>() * WORLD_SIZE,
            orientation: random::<f32>() * Float::two_pi(),
            bearing_noise: 0.0,
            steering_noise: 0.0,
            distance_noise: 0.0,
            length: LENGTH,
        }
    }

    fn set(&mut self, new_x: f32, new_y: f32, new_orientation: f32) {
        assert!(new_orientation >= 0.0 && new_orientation < Float::two_pi());
        self.x = new_x;
        self.y = new_y;
        self.orientation = new_orientation;
    }

    fn set_noise(&mut self, new_b_noise: f32, new_s_noise: f32, new_d_noise: f32) {
        self.bearing_noise = new_b_noise;
        self.steering_noise = new_s_noise;
        self.distance_noise = new_d_noise;
    }

    fn measurement_prob(&self, measurements: &Vec<f32>) -> f32 {
        let predicted_measurements: Vec<f32> = self.sense_without_noise();
        let mut error: f32 = 1.0;
        for i in range(0,measurements.len()) {
            let mut error_bearing: f32 = abs(measurements[i] - predicted_measurements[i]);
            error_bearing = modulo(error_bearing + Float::pi(), Float::two_pi()) - Float::pi();
            error *= gaussian(error_bearing, self.bearing_noise, 0.0);
        }
        error
    }

    fn sense(&self) -> Vec<f32> {
        let mut z: Vec<f32> = Vec::with_capacity(LANDMARKS.len());
        for &(y,x) in LANDMARKS.iter() {
            // noise
            let n = gauss(0.0, self.bearing_noise);
            // sensed bearing
            let bearing = modulo((y - self.y).atan2(x - self.x) - self.orientation + n, Float::two_pi());
            z.push(bearing);
        }
        z
    }
    
    fn sense_without_noise(&self) -> Vec<f32> {
        let mut z: Vec<f32> = Vec::with_capacity(LANDMARKS.len());
        for &(y,x) in LANDMARKS.iter() {
            // sensed bearing
            let bearing = modulo((y - self.y).atan2(x - self.x) - self.orientation, Float::two_pi());
            z.push(bearing);
        }
        z
    }
    
    fn travel(&self, (alpha, d) : (f32, f32)) -> Robot {
        let alpha = alpha + gauss(0.0, self.steering_noise);
        let d = d + gauss(0.0, self.distance_noise);
        let beta = (d / self.length) * alpha.tan();
        let mut x = self.x;
        let mut y = self.y;
        let mut theta = self.orientation;

        if beta.abs() > 0.0001 {
            let r = d / beta;
            let cx = x - theta.sin() * r;
            let cy = y + theta.cos() * r;
            x = cx + (theta + beta).sin() * r;
            y = cy - (theta + beta).cos() * r;
            theta = modulo((theta + beta), Float::two_pi());

        } else {
            x = x + d * theta.cos();
            y = y + d * theta.sin();
        }

        let mut res = Robot::new();
        res.set(x, y, theta);
        res.set_noise(self.bearing_noise, self.steering_noise, self.distance_noise);
        res
    }
}


fn main() {
    println!("\n=test_case_1=");
    test_case_1();
    println!("\n=test_case_2=");
    test_case_2();
}


fn test_case_1() {
    let motions: Vec<(f32,f32)> = Vec::from_elem(8, (2.0_f32 * Float::pi() / 10.0_f32, 20.0));
    let measurements: Vec<Vec<f32>> = vec![
        vec![4.746936, 3.859782, 3.045217, 2.045506],
        vec![3.510067, 2.916300, 2.146394, 1.598332],
        vec![2.972469, 2.407489, 1.588474, 1.611094],
        vec![1.906178, 1.193329, 0.619356, 0.807930],
        vec![1.352825, 0.662233, 0.144927, 0.799090],
        vec![0.856150, 0.214590, 5.651497, 1.062401],
        vec![0.194460, 5.660382, 4.761072, 2.471682],
        vec![5.717342, 4.736780, 3.909599, 2.342536]];

    println!("Actual:          (93.476, 75.186, 5.2664)");
    println!("Particle filter: {}", particle_filter(&motions, &measurements, 500));
}


fn test_case_2() {
    let number_of_iterations = 6;
    let motions: Vec<(f32,f32)> = Vec::from_elem(number_of_iterations, (2.0_f32 * Float::pi() / 20.0_32, 12.0));
    let (final_robot, measurements) = generate_ground_truth(&motions);
    let estimated_position = particle_filter(&motions, &measurements, 500);

    println!("Ground truth:    ({}, {}, {})", final_robot.x, final_robot.y, final_robot.orientation);
    println!("Particle filter: {}", estimated_position);
    println!("Code check:      {}", check_output(&final_robot, estimated_position));
}


fn get_position(ps: &Vec<Robot>) -> (f32, f32, f32) {
    let mut x = 0.0;
    let mut y = 0.0;
    let mut orientation = 0.0;

    for &p in ps.iter() {
        x += p.x;
        y += p.y;

        // Because angles are cyclic, the orientation needs adjustment.
        orientation  += modulo((p.orientation - ps[0].orientation + Float::pi()), Float::two_pi())
                        + ps[0].orientation
                        - Float::pi();
    }

    let len = ps.len() as f32;
    (x / len, y / len, orientation / len)
}


fn generate_ground_truth(motions: &Vec<(f32,f32)>) -> (Robot, Vec<Vec<f32>>) {
    let mut myrobot = Robot::new();
    myrobot.set_noise(BEARING_NOISE, STEERING_NOISE, DISTANCE_NOISE);

    let mut z = Vec::with_capacity(motions.len());
    for &m in motions.iter() {
        myrobot = myrobot.travel(m);
        z.push(myrobot.sense());
    };

    (myrobot, z)
}

#[allow(dead_code)]
fn print_measurements(zs: &Vec<Vec<f32>>) {
    println!("measurments = ");
    for z in zs.iter() {
        println!("{}", z);
    }
}


fn check_output(final_robot: &Robot, (est_x, est_y, est_or): (f32, f32, f32)) -> bool {
    let error_x = abs(final_robot.x - est_x);
    let error_y = abs(final_robot.y - est_y);
    let mut error_orientation = abs(final_robot.orientation - est_or);
    error_orientation = modulo(error_orientation + Float::pi(), Float::two_pi()) - Float::pi();
    let correct = error_x < TOLERANCE_XY && error_y < TOLERANCE_XY && error_orientation < TOLERANCE_ORIENTATION;
    correct
}


fn particle_filter(motions: &Vec<(f32, f32)>, measurements: &Vec<Vec<f32>>, n: uint) -> (f32, f32, f32) {
    let mut ps = Vec::from_fn(n, |_| {
        let mut r = Robot::new();
        r.set_noise(BEARING_NOISE, STEERING_NOISE, DISTANCE_NOISE);
        r
    });

    for t in range(0, motions.len()) {
        // motion update (prediction)
        ps = ps.map_in_place(|p| p.travel(motions[t]) );

        // measurement update
        let mut ws = Vec::with_capacity(ps.len());
        for p in ps.iter() {
            ws.push(p.measurement_prob(&measurements[t]));
        }

        // resampling
        let mut p3: Vec<Robot> = Vec::with_capacity(n);
        let mut index = random::<uint>() % n;
        let mut beta = 0.0;
        let mut mw = 0.0;
        for &w in ws.iter() { if w > mw { mw = w; } } 
        for _ in range(0, n) {
            beta += random::<f32>() * 2.0 * mw;
            while beta > ws[index] {
                beta -= ws[index];
                index = (index + n + 1) % n;
            }
            p3.push(ps[index].clone());
        }
        ps = p3;
    }

    get_position(&ps)
}
