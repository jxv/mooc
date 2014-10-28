use std::iter::{range_step_inclusive, AdditiveIterator};

#[deriving(Show,Clone,PartialEq)]
struct Matrix {
    value: Vec<Vec<f32>>,
    dimx: uint,
    dimy: uint,
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


fn main() {
    println!("{}", doit(-3.0, 5.0, 3.0, 10.0, 5.0, 2.0).value)
}


fn doit(initial_pos: f32, move1: f32, move2: f32, z0: f32, z1: f32, z2: f32) -> Matrix {
    let mut omega = Matrix::new(vec![
        vec![1.0, 0.0, 0.0],
        vec![0.0, 0.0, 0.0],
        vec![0.0, 0.0, 0.0]]);
    let mut xi = Matrix::new(vec![vec![initial_pos], vec![0.0], vec![0.0]]);

    omega = omega.add(&Matrix::new(vec![
        vec![ 1.0, -1.0,  0.0],
        vec![-1.0,  1.0,  0.0],
        vec![ 0.0,  0.0,  0.0]]));
    xi = xi.add(&Matrix::new(vec![vec![-move1], vec![move1], vec![0.0]]));
    
    omega = omega.add(&Matrix::new(vec![
        vec![ 0.0,  0.0,  0.0],
        vec![ 0.0,  1.0, -1.0],
        vec![ 0.0, -1.0,  1.0]]));
    xi = xi.add(&Matrix::new(vec![vec![0.0], vec![-move2], vec![move2]]));


    omega = omega.expand(4, 4, &vec![0, 1, 2], &vec![0, 1, 2]);
    xi = xi.expand(4, 1, &vec![0, 1, 2], &vec![0]);

    omega = omega.add(&Matrix::new(vec![
        vec![1.0, 0.0, 0.0, -1.0],
        vec![0.0, 0.0, 0.0, 0.0],
        vec![0.0, 0.0, 0.0, 0.0],
        vec![-1.0, 0.0, 0.0, 1.0]]));
    xi = xi.add(&Matrix::new(vec![vec![-z0], vec![0.0], vec![0.0], vec![z0]]));

    omega = omega.add(&Matrix::new(vec![
        vec![0.0, 0.0, 0.0, 0.0],
        vec![0.0, 1.0, 0.0, -1.0],
        vec![0.0, 0.0, 0.0, 0.0],
        vec![0.0, -1.0, 0.0, 1.0]]));
    xi = xi.add(&Matrix::new(vec![vec![0.0], vec![-z1], vec![0.0], vec![z1]]));

    omega = omega.add(&Matrix::new(vec![
        vec![0.0, 0.0, 0.0, 0.0],
        vec![0.0, 0.0, 0.0, 0.0],
        vec![0.0, 0.0, 1.0, -1.0],
        vec![0.0, 0.0, -1.0, 1.0]]));
    xi = xi.add(&Matrix::new(vec![vec![0.0], vec![0.0], vec![-z2], vec![z2]]));

    omega.print("omega: ".to_string());
    xi.print("xi: ".to_string());
    let mu = omega.inverse().mul(&xi);
    mu
}
