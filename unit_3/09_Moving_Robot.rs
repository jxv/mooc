use std::rand::{random, task_rng};
use std::rand::distributions::{Normal, IndependentSample};
use std::num::{Float, FloatMath};
use std::num::{FromPrimitive};

#[deriving(Show,Clone)]
struct Robot {
    world_size: f32,
    x: f32,
    y: f32,
    orientation: f32,
    forward_noise: f32,
    turn_noise: f32,
    sense_noise: f32,
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
    fn new(world_size: f32) -> Robot {
        Robot {
            world_size: world_size,
            x: random::<f32>() * world_size,
            y: random::<f32>() * world_size,
            orientation: random::<f32>() * Float::two_pi(),
            forward_noise: 0.0,
            turn_noise: 0.0,
            sense_noise: 0.0,
        }
    }

    fn set(&mut self, new_x: f32, new_y: f32, new_orientation: f32) {
        assert!(new_x >= 0.0 && new_x < self.world_size);
        assert!(new_y >= 0.0 && new_y < self.world_size);
        assert!(new_orientation >= 0.0 && new_orientation < Float::two_pi());
        self.x = new_x;
        self.y = new_y;
        self.orientation = new_orientation;
    }

    fn set_noise(&mut self, new_f_noise: f32, new_t_noise: f32, new_s_noise: f32) {
        self.forward_noise = new_f_noise;
        self.turn_noise = new_t_noise;
        self.sense_noise = new_s_noise;
    }

    fn sense(&self, landmarks: &Vec<(f32,f32)>) -> Vec<f32> {
        let mut z: Vec<f32> = Vec::with_capacity(landmarks.len());
        for &(x,y) in landmarks.iter() {
            // sensed dist
            let mut dist = ((self.x - x).powi(2) + (self.y - y).powi(2)).sqrt();
            // add error
            dist += gauss(0.0, self.sense_noise);
            z.push(dist);
        }
        z
    }

    fn travel(&self, turn: f32, forward: f32) -> Robot {
        assert!(forward >= 0.0);

        let mut orientation = self.orientation + turn + gauss(0.0, self.sense_noise);
        orientation = (orientation + Float::two_pi()) % Float::two_pi();

        let dist = forward + gauss(0.0, self.forward_noise);
        let mut x = self.x + orientation.cos() * dist;
        let mut y = self.y + orientation.sin() * dist;
        x = x % self.world_size;
        y = y % self.world_size;

        let mut res = Robot::new(self.world_size);
        res.set(x, y, orientation);
        res.set_noise(self.forward_noise, self.turn_noise, self.sense_noise);
        res
    }

    #[allow(dead_code)]
    fn measurement_prob(&self, landmarks: &Vec<(f32,f32)>, measurement: &Vec<f32>) -> f32 {
        assert!(landmarks.len() == measurement.len());
        let mut prob: f32 = 1.0;
        for i in range(0,landmarks.len()) {
            let (x,y) = landmarks[i];
            let dist = ((self.x - x).powi(2) + (self.y - y).powi(2)).sqrt();
            prob = prob * gaussian(dist, self.sense_noise, measurement[i]);
        }
        prob
    }
}

#[allow(dead_code)]
fn eval(r: &Robot, ps: &Vec<Robot>) -> f32 {
    let mut sum: f32 = 0.0;
    for p in ps.iter() {
        assert!(r.world_size == p.world_size);
        let dx = (p.x - r.x + (r.world_size / 2.0)) % r.world_size - (r.world_size / 2.0);
        let dy = (p.y - r.y + (r.world_size / 2.0)) % r.world_size - (r.world_size / 2.0);
        let err = (dx * dx + dy * dy).sqrt();
        sum += err;
    }
    sum / (ps.len() as f32)
}

fn main() {
    let landmarks: Vec<(f32,f32)> = vec![(20.0, 20.0),
                                         (80.0, 80.0),
                                         (20.0, 80.0),
                                         (80.0, 20.0)];
    let world_size: f32 = 100.0;

    let mut myrobot = Robot::new(world_size);

    let frac_pi_2: f32 = Float::frac_pi_2();
    myrobot.set(30.0, 50.0, frac_pi_2);
    
    myrobot = myrobot.travel(-frac_pi_2, 15.0);
    println!("{}", myrobot.sense(&landmarks));
    
    myrobot = myrobot.travel(-frac_pi_2, 10.0);
    println!("{}", myrobot.sense(&landmarks));
}
