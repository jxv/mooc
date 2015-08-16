use std::rand::{task_rng};
use std::rand::distributions::{Normal, IndependentSample};
use std::num::{Float, FloatMath};
use std::num::{FromPrimitive};


struct Robot {
    x: f32,
    y: f32,
    orientation: f32,
    length: f32,
    steering_noise: f32,
    distance_noise: f32,
    steering_drift: f32
}


impl Robot {

    fn new() -> Robot {
        Robot::new_with_extra(20.0)
    }


    fn new_with_extra(length: f32) -> Robot {
        Robot {
            x: 0.0,
            y: 0.0,
            orientation: 0.0,
            length: length,
            steering_noise: 0.0,
            distance_noise: 0.0,
            steering_drift: 0.0
        }
    }


    fn set(&mut self, new_x: f32, new_y: f32, new_orientation: f32) {
        self.x = new_x;
        self.y = new_y;
        self.orientation = modulo(new_orientation, Float::two_pi());
    }


#[allow(dead_code)]
    fn set_noise(&mut self, new_s_noise: f32, new_d_noise: f32) {
        self.steering_noise = new_s_noise;
        self.distance_noise = new_d_noise;
    }


    fn travel(&self, steering: f32, distance: f32) -> Robot {
        self.travel_with_extra(steering, distance, 0.001, Float::frac_pi_4())
    }


    fn travel_with_extra(&self, steering: f32, distance: f32, tolerance: f32, max_steering_angle: f32) -> Robot {
        let mut steering = if steering > max_steering_angle { max_steering_angle } else { steering };
        steering = if steering < -max_steering_angle { -max_steering_angle } else { steering };
        let distance = if distance < 0.0 { 0.0 } else { distance };

        // make a new copy
        let mut res = Robot::new();
        res.length         = self.length;
        res.steering_noise = self.steering_noise;
        res.distance_noise = self.distance_noise;
        res.steering_drift = self.steering_drift;

        // apply noise
        let mut steering2 = gauss(steering, self.steering_noise);
        let distance2 = gauss(distance, self.distance_noise);

        // apply steering drift
        steering2 += self.steering_drift;

        let turn = steering2.tan() * distance2 / res.length;

        if turn.abs() < tolerance {
            
            // straight line
            res.x = self.x + (distance2 * self.orientation.cos());
            res.y = self.y + (distance2 * self.orientation.sin());
            res.orientation = modulo(self.orientation + turn, Float::two_pi());
        } else {

            // appoximate bicycle model for motion
            let radius = distance2 / turn;
            let cx = self.x - (self.orientation.sin() * radius);
            let cy = self.y + (self.orientation.cos() * radius);
            res.orientation = modulo(self.orientation + turn, Float::two_pi());
            res.x = cx + (res.orientation.sin() * radius);
            res.y = cy - (res.orientation.cos() * radius);
            
        }

        res
    }


    fn print(&self) {
        print!("[x={:.5f} y={:.5f} orient={:.5f}]", self.x, self.y, self.orientation);
    }

}


fn run(param1: f32, param2: f32) {
    let mut myrobot = Robot::new();
    myrobot.set(0.0, 1.0, 0.0);
    let speed = 1.0;
    let n = 100;
    
    let tau_p = param1;
    let tau_d = param2;
    let mut cte = myrobot.y;

    for _ in range(0u,n) {
        let o_cte = myrobot.y - cte; 
        cte = myrobot.y;

        let steering = (-tau_p * cte) + (-tau_d * o_cte);
        myrobot = myrobot.travel(steering, speed);
        myrobot.print();
        println!(" {:.15f}", steering);
    }
}


fn main() {
    run(0.2, 3.0)
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
