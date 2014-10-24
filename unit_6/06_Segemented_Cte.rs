#![feature(tuple_indexing)]

use std::rand::{task_rng, random};
use std::rand::distributions::{Normal, IndependentSample};
use std::num::{Float, FloatMath};
use std::num::{FromPrimitive};


#[deriving(Show,PartialEq,Clone)]
enum Dir {
    Up,
    Left,
    Down,
    Right,
}


#[deriving(Show,Clone)]
type Grid = [[Space,..6],..5];

static DIRS: [Dir, ..4] = [Up, Left, Down, Right];
static GRID: Grid = [
    [O,X,O,O,O,O],
    [O,X,O,X,X,O],
    [O,X,O,X,O,O],
    [O,O,O,X,O,X],
    [O,X,O,X,O,O]];

static INIT: (uint,uint) = (0,0);
static GOAL: (uint,uint) = (4,5);

static STEERING_NOISE: f32 = 1.0;
static DISTANCE_NOISE: f32 = 0.03;
static MEASUREMENT_NOISE: f32 = 0.3;

static WEIGHT_DATA: f32 = 0.1;
static WEIGHT_SMOOTH: f32 = 0.2;
static P_GAIN: f32 = 2.0;
static D_GAIN: f32 = 6.0;


fn forward(grid: &Grid, (x,y): (uint, uint), dir: Dir) -> Option<(uint, uint)> {
    let (x2, y2) = match dir {
        Up    => (x, y - 1),
        Left  => (x - 1, y),
        Down  => (x, y + 1),
        Right => (x + 1, y),
    };
    if x2 < grid.len() && y2 < grid[0].len() { Some((x2,y2)) } else { None }
}

#[allow(dead_code)]
fn from_dir(dir: Dir) -> uint {
    match dir { Up => 0, Left => 1, Down => 2, Right => 3 }
}

#[allow(dead_code)]
fn opp_dir(dir: Dir) -> Dir {
    match dir { Up => Down, Left => Right, Down => Up, Right => Left }
}

#[deriving(Show,PartialEq,Clone)]
enum Space {
    O,
    X,
}


struct Plan {
    cost: uint,
    grid: Grid,
    init: (uint,uint),
    goal: (uint,uint),
    heuristic: Vec<Vec<uint>>,
    path: Vec<Vec<uint>>,
    spath: Vec<Vec<f32>>,
}


#[deriving(Show,Clone)]
struct Robot {
    x: f32,
    y: f32,
    orientation: f32,
    length: f32,
    steering_noise: f32,
    distance_noise: f32,
    measurement_noise: f32,
    num_collisions: uint,
    num_steps: uint,
}


#[deriving(Show,Clone)]
struct Particles {
    n: uint,
    steering_noise: f32,
    distance_noise: f32,
    measurement_noise: f32,
    data: Vec<Robot>,
}


#[deriving(Show,Clone)]
impl Plan {
    
    fn new(grid: &Grid, init: (uint,uint), goal: (uint,uint)) -> Plan {
        Plan::new_with_extra(grid, init, goal, 1)
    }


    fn new_with_extra(grid: &Grid, init: (uint,uint), goal: (uint,uint), cost: uint) -> Plan {
        Plan {
            cost: cost,
            grid:[[grid[0][0], grid[0][1], grid[0][2], grid[0][3], grid[0][4], grid[0][5]],
                  [grid[1][0], grid[1][1], grid[1][2], grid[1][3], grid[1][4], grid[1][5]],
                  [grid[2][0], grid[2][1], grid[2][2], grid[2][3], grid[2][4], grid[2][5]],
                  [grid[3][0], grid[3][1], grid[3][2], grid[3][3], grid[3][4], grid[3][5]],
                  [grid[4][0], grid[4][1], grid[4][2], grid[4][3], grid[4][4], grid[4][5]]],
            init: init,
            goal: goal,
            heuristic: Plan::make_heuristic(grid, goal, cost),
            path: Vec::new(),
            spath: Vec::new(),
        }
    }


    fn make_heuristic(grid: &Grid, goal: (uint,uint), /*cost*/ _: uint) -> Vec<Vec<uint>> {
        Vec::from_fn(grid.len(), |i: uint| {
            Vec::from_fn(grid[i].len(), |j: uint| {
                (if i < goal.0 { goal.0 - i } else { i - goal.0 }) +
                (if j < goal.1 { goal.1 - j } else { j - goal.1 })
            })
        })
    }


    fn astar(&mut self) {

        if self.heuristic.len() == 0 {
            fail!("Heuristic must be defined to run astar");
        }

        let mut closed =
                Vec::from_elem(self.grid.len(), Vec::from_elem(self.grid[0].len(), false));
        let mut action: Vec<Vec<Option<Dir>>> =
                Vec::from_elem(self.grid.len(), Vec::from_elem(self.grid[0].len(), None));

        *closed.get_mut(self.init.0 as uint).get_mut(self.init.1 as uint) = true;

        let x = self.init.0;
        let y = self.init.1;
        let h = self.heuristic[x as uint][y as uint];
        let g = 0;
        let f = g + h;
        let mut open = vec![(f, g, h, x, y)];

        let mut found = false;
        let mut resign = false;
        let mut count: int = 0;

        while !found && !resign {
            open.sort_by(|a,b| b.cmp(a));
            let (_,g,_,x,y) = match open.pop() {
                None => {
                       resign = true;
                       println!("##### Search terminated without success, {}", count);
                       continue;
                    },
                Some(n) => n
            };

            if x == self.goal.0 && y == self.goal.1 {
                found = true;
                continue;
            }

            for &d in DIRS.iter() {
                let (x2,y2) = match forward(&self.grid, (x,y), d) {
                    None => continue,
                    Some(xy) => xy
                };

                if !closed[x2][y2] && self.grid[x2][y2] == O {
                    let g2 = g + self.cost;
                    let h2 = self.heuristic[x2][y2];
                    let f2 = g2 + h2;
                    open.push((f2, g2, h2, x2, y2));
                    *closed.get_mut(x2).get_mut(y2) = true;
                    *action.get_mut(x2).get_mut(y2) = Some(d);
                }
            }
            count += 1;
        }
        

        let mut invpath = Vec::new();
        let mut x = self.goal.0;
        let mut y = self.goal.1;
        invpath.push([x,y]);
        while x != self.init.0 || y != self.init.1 {
            let d = opp_dir(action[x][y].unwrap());
            let (x2,y2) = match d {
                Up => (x, y - 1),
                Down => (x, y + 1),
                Left => (x - 1, y),
                Right => (x + 1, y)
            };
            x = x2;
            y = y2;
            invpath.push([x,y]);
        }
        self.path = Vec::with_capacity(invpath.len());
        for i in range(0u,invpath.len()) {
            let e = invpath[invpath.len() - 1 - i];
            self.path.push(vec![e[0],e[1]]);
        }
    }


#[allow(dead_code)]
    fn smooth(&mut self) {
        self.smooth_extra(0.1, 0.1, 0.000001)
    }


    fn smooth_extra(&mut self, weight_data: f32, weight_smooth: f32, tolerance: f32) {
        if self.path.len() == 0 {
            fail!("run astar before smoothing path");
        }

        self.spath = Vec::from_fn(self.path.len(), |i: uint| {
            Vec::from_fn(self.path[i].len(), |j: uint| {
                self.path[i][j] as f32
            })
        });
        
        let mut change = tolerance;
        while change >= tolerance {
            change = 0.0;

            for i in range(1u, self.path.len() - 1) {
                for j in range(0u, self.path[0].len()) {
                    let aux = self.spath[i][j];

                    *self.spath.get_mut(i).get_mut(j)
                            += weight_data * ( self.spath[i-1][j]
                                             + self.spath[i+1][j]
                                             - 2.0 * self.path[i][j] as f32 );

                    *self.spath.get_mut(i).get_mut(j)
                            += weight_smooth * ( self.spath[i-1][j]
                                               + self.spath[i+1][j]
                                               - 2.0 * self.spath[i][j] );

                    if i >= 2 {
                        *self.spath.get_mut(i).get_mut(j)
                                += 0.5 * weight_smooth * ( 2.0 * self.spath[i-1][j]
                                                         - self.spath[i-2][j]
                                                         - self.spath[i][j] );
                    }

                    if i <= self.path.len() - 3 {
                        *self.spath.get_mut(i).get_mut(j)
                                += 0.5 * weight_smooth * ( 2.0 * self.spath[i+1][j]
                                                         - self.spath[i+2][j]
                                                         - self.spath[i][j] );
                    }

                    change += (aux - self.spath[i][j]).abs();
                }
            }
        }
    }

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
            measurement_noise: 0.0,
            num_collisions: 0,
            num_steps: 0,
        }
    }


    fn set(&mut self, new_x: f32, new_y: f32, new_orientation: f32) {
        self.x = new_x;
        self.y = new_y;
        self.orientation = modulo(new_orientation, Float::two_pi());
    }


    fn set_noise(&mut self, new_s_noise: f32, new_d_noise: f32, new_m_noise: f32) {
        self.steering_noise = new_s_noise;
        self.distance_noise = new_d_noise;
        self.measurement_noise = new_m_noise;
    }


    //--


    fn check_collision(&mut self, grid: &Grid) -> bool {
        for i in range(0u, grid.len()) {
            for j in range(0u, grid[i].len()) {
                if grid[i][j] == X {
                    let dist = ((self.x - i as f32).powi(2) + (self.y - j  as f32).powi(2)).sqrt();
                    if dist < 0.5 {
                        self.num_collisions += 1;
                        return false;
                    }
                }
            }
        }
        true
    }


    fn check_goal(&self, goal: (uint,uint)) -> bool {
        self.check_goal_extra(goal, 1.0)
    }

    
    fn check_goal_extra(&self, goal: (uint,uint), threshold: f32) -> bool {
        let dist = ((goal.0 as f32 - self.x).powi(2) + (goal.1 as f32 - self.y).powi(2)).sqrt();
        dist < threshold
    }


    //--


    fn travel(&self, grid: &Grid, steering: f32, distance: f32) -> Robot {
        self.travel_extra(grid, steering, distance, 0.001, Float::frac_pi_4())
    }


    fn travel_extra(&self, _: &Grid, steering: f32, distance: f32, tolerance: f32,
            max_steering_angle: f32) -> Robot {
        let steering =
            if steering > max_steering_angle {
                max_steering_angle
            } else if steering < -max_steering_angle {
                -max_steering_angle
            } else {
                steering
            };

        let distance = if distance < 0.0 { 0.0 } else { distance };

        // make a new copy
        let mut res = Robot::new();
        res.length            = self.length;
        res.steering_noise    = self.steering_noise;
        res.distance_noise    = self.distance_noise;
        res.measurement_noise = self.measurement_noise;
        res.num_collisions    = self.num_collisions;
        res.num_steps         = self.num_steps + 1;

        // apply noise
        let steering2 = gauss(steering, self.steering_noise);
        let distance2 = gauss(distance, self.distance_noise);

        // execute motion
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


    fn sense(&self) -> (f32, f32) {
        (gauss(self.x, self.measurement_noise), gauss(self.y, self.measurement_noise))
    }

   
#[allow(dead_code)]
    fn measurement_prob(&self, measurement: (f32, f32)) -> f32 {
        let error_x = measurement.0 - self.x;
        let error_y = measurement.1 - self.y;

        let mut error: f32 = (-(error_x.powi(2)) / self.measurement_noise.powi(2) / 2.0).exp()
                      / (self.measurement_noise.powi(2) * Float::two_pi()).sqrt();
        error *= (-(error_y.powi(2)) / self.measurement_noise.powi(2) / 2.0).exp()
                      / (self.measurement_noise.powi(2) * Float::two_pi()).sqrt();
        error
    }
    
    
    fn print(&self) {
        print!("[x={:.5f} y={:.5f} orient={:.5f}]", self.x, self.y, self.orientation);
    }
   
}

/*
    fn cte(&self, radius: f32) -> f32 {
        let mut cte = 0.0;
        let x = self.x;
        let y = self.y;

        if x <= radius || x >= radius * 3.0 {
            let mid_x = if x < radius { radius } else { 3.0 * radius };
            let mid_y = radius;
            cte = ((mid_x - x).powi(2) + (mid_y - y).powi(2)).sqrt() - radius;
        }

        if x >= radius && x <= radius * 3.0 && y >= radius {
            cte = -(radius * 2.0 - y);
        }

        if x >= radius && x <= radius * 3.0 && y <= radius {
            cte = -y;
        }

        cte
    }
*/


impl Particles {
    fn new(x: f32, y: f32, theta: f32, steering_noise: f32, distance_noise: f32,
            measurement_noise: f32) -> Particles {
        Particles::new_extra(x, y, theta, steering_noise, distance_noise, measurement_noise, 100)
    }

    
    fn new_extra(x: f32, y: f32, theta: f32, steering_noise: f32, distance_noise: f32,
            measurement_noise: f32, n: uint) -> Particles {
        Particles {
            n: n,
            steering_noise: steering_noise,
            distance_noise: distance_noise,
            measurement_noise: measurement_noise,
            data: Vec::from_elem(n, {
                    let mut r = Robot::new();
                    r.set(x, y, theta);
                    r.set_noise(steering_noise, distance_noise, measurement_noise);
                    r
                }),
        }
    }


    fn get_position(&self) -> (f32, f32, f32) {
        let mut x = 0.0;
        let mut y = 0.0;
        let mut orientation = 0.0;

        for i in range(0u, self.n) {
            x += self.data[i].x;
            y += self.data[i].y;

            orientation += modulo(self.data[i].orientation - self.data[0].orientation
                                                           + Float::pi(),
                                  Float::two_pi())
                          + self.data[0].orientation
                          - Float::pi();

        }

        (x / self.n as f32, y / self.n as f32, orientation / self.n as f32)
    }


    fn travel(&mut self, grid: &Grid, steer: f32, speed: f32) {
        self.data = Vec::from_fn(self.data.len(), |i: uint| {
            self.data[i].travel(grid, steer, speed)
        });
    }


    fn sense(&mut self, z: (f32, f32)) {
        let ws = Vec::from_fn(self.data.len(), |i: uint| {
                self.data[i].measurement_prob(z)
            });

        let mut p3 = Vec::with_capacity(self.n);
        let mut idx = random::<uint>() %  self.n;
        let mut beta = 0.0;
        let mut mw = 0.0;
        for &w in ws.iter() { if w > mw { mw = w; } }

        for _ in range(0u, self.n) {
            beta += random::<f32>() * 2.0 * mw;
            while beta > ws[idx] {
                beta -= ws[idx];
                idx = (idx + self.n + 1) % self.n;
            }
            p3.push(self.data[idx].clone());
        }

        self.data = p3;
    }

}


fn run(grid: &Grid, goal: (uint,uint), spath: &Vec<Vec<f32>>, params: (f32,f32))
        -> (bool, uint, uint) {
    run_extra(grid, goal, spath, params, false, 0.1, 1000)
}


fn run_extra(grid: &Grid, goal: (uint,uint), spath: &Vec<Vec<f32>>, params: (f32,f32),
        print_flag: bool, speed: f32, timeout: uint) -> (bool, uint, uint) {
    let mut myrobot = Robot::new();
    myrobot.set(0.0, 0.0, 0.0);
    myrobot.set_noise(STEERING_NOISE, DISTANCE_NOISE, MEASUREMENT_NOISE);
    let mut filter = Particles::new(myrobot.x, myrobot.y, myrobot.orientation,
            STEERING_NOISE, DISTANCE_NOISE, MEASUREMENT_NOISE); 

    //

    let mut cte_p = 0.0;
    let mut err = 0.0;
    let mut n = 0;
    let mut idx = 0; // index into the path

    while !myrobot.check_goal(goal) && n < timeout {
        let mut cte_d = -cte_p;
        let estimate = filter.get_position();

        let dx: f32 = spath[idx+1][0] as f32 - spath[idx][0] as f32;
        let dy: f32 = spath[idx+1][1] as f32 - spath[idx][1] as f32;
        let drx: f32 = estimate.0 as f32 - spath[idx][0] as f32;
        let dry: f32 = estimate.1 as f32 - spath[idx][1] as f32;
        
        let u = (drx * dx + dry * dy) / (dx.powi(2) + dy.powi(2));
        cte_p = (dry * dx + drx * dy) / (dx.powi(2) + dy.powi(2));

        if u > 1.0 {
            idx += 1;
        }

        cte_d += cte_p;

        let steer = -params.0 * cte_p - params.1 * cte_d;
        myrobot = myrobot.travel(grid, steer, speed);
        filter.travel(grid, steer, speed);

        let z = myrobot.sense();
        filter.sense(z);

        if !myrobot.check_collision(grid) {
            println!("##### Collision ####");
        }

        err += cte_p.powi(2);
        n += 1;

        if print_flag {
            myrobot.print();
            println!("cte_p:{} idx:{} u:{} err:{}", cte_p, idx, u, err);
        }
    }
    
    //

    (myrobot.check_goal(goal), myrobot.num_collisions, myrobot.num_steps)
}


fn main() {
    let mut plan = Plan::new(&GRID, INIT, GOAL);
    plan.astar();
    println!("{}", plan.path);
    plan.smooth_extra(WEIGHT_DATA, WEIGHT_SMOOTH, 0.000001);
    println!("{}", run(&GRID, GOAL, &plan.spath, (P_GAIN, D_GAIN)));
}

#[allow(dead_code)]
fn twiddle() {
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
