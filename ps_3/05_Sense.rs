// use std::iter::AdditiveIterator;
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
    length: f32,
}

fn modulo(n: f32, m: f32) -> f32 {
    let mut k = n % m;
    while k < 0.0 { k += m };
    k
}

fn gauss(mean: f32, covar2: f32) -> f32 {
    let cvt_covar2: f64 = FromPrimitive::from_f32(covar2).unwrap();
    let cvt_mean: f64 = FromPrimitive::from_f32(mean).unwrap();
    FromPrimitive::from_f64(Normal::new(cvt_mean, cvt_covar2).ind_sample(&mut task_rng())).unwrap()
}

impl Robot {
    fn new(world_size: f32, length: f32) -> Robot {
        Robot {
            world_size: world_size,
            x: random::<f32>() * world_size,
            y: random::<f32>() * world_size,
            orientation: random::<f32>() * Float::two_pi(),
            forward_noise: 0.0,
            turn_noise: 0.0,
            sense_noise: 0.0,
            length: length,
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
        for &(y,x) in landmarks.iter() {
            // sensed dist
            let mut dist = modulo((y - self.y).atan2(x - self.x) - self.orientation, Float::two_pi());
            // add error
            dist += gauss(0.0, self.sense_noise);
            z.push(dist);
        }
        z
    }
}

fn main() {
    let landmarks: Vec<(f32,f32)> = vec![(0.0, 100.0),
                                         (0.0, 0.0),
                                         (100.0, 0.0),
                                         (100.0, 100.0)];


    let world_size: f32 = 100.0;
    let length: f32 = 20.0;
    let bearing_noise = 0.0;
    let steering_noise = 0.0;
    let distance_noise = 0.0;

    let mut myrobot = Robot::new(world_size, length);
    myrobot.set(30.0, 20.0, 0.0);
    myrobot.set_noise(bearing_noise, steering_noise, distance_noise);

    println!("{}\n{}", myrobot, myrobot.sense(&landmarks));
}
