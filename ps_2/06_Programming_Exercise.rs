use std::iter::AdditiveIterator;

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
                res.value[k][i] * res.value[k][i]
            }).iter().map(|&x| x).sum();

            let d = self.value[i][i] - s;
            assert!(d.abs() < ztol || d > 0.0);
            *res.value.get_mut(i).get_mut(i) = if d.abs() < ztol { 0.0 } else { d.sqrt() };

            for j in range(i+1, self.dimx) {
                let mut s: f32 = Vec::from_fn(self.dimx, |k| {
                    self.value[k][i] * res.value[k][j]
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
        let mut res = Matrix::zero(self.dimy, self.dimx);
        for j_ in range(0, self.dimx) {
            let j = self.dimx - (j_ + 1);

            let tjj = self.value[j][j];

            let ofs = j + 1;
            let s: f32 = Vec::from_fn(self.dimx - ofs, |k| {
                self.value[j][k + ofs] * res.value[j][k + ofs]
            }).iter().map(|&x| x).sum();

            *res.value.get_mut(j).get_mut(j) = 1.0 / (tjj * tjj) - s / tjj;

            for i_ in range(0,j) {
                let i = j - (i_ + 1);

                let ofs = i + 1;
                let v = -Vec::from_fn(self.dimx - ofs, |k| {
                    res.value[i][k + ofs] * res.value[k + ofs][j]
                }).iter().map(|&x| x).sum() / self.value[i][i];
                
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
}


struct Cxt {
    u: Matrix, // extern motion
    f: Matrix, // next state fn
    h: Matrix, // measurement fn
    r: Matrix, // measurement uncertainty
    i: Matrix, // identity matrix
}


impl Cxt {
    fn new(dt: f32) -> Cxt {
        Cxt {
            u: Matrix::new(vec![vec![0.0],
                                vec![0.0],
                                vec![0.0],
                                vec![0.0]]),

            f: Matrix::new(vec![vec![1.0, 0.0, dt, 0.0],
                                vec![0.0, 1.0, 0.0, dt],
                                vec![0.0, 0.0, 1.0, 0.0],
                                vec![0.0, 0.0, 0.0, 1.0]]),

            h: Matrix::new(vec![vec![1.0, 0.0, 0.0, 0.0],
                                vec![0.0, 1.0, 0.0, 0.0]]),

            r: Matrix::new(vec![vec![0.1, 0.0],
                                vec![0.0, 0.1]]),

            i: Matrix::identity(4),
        }
    }
}


fn main() {
    let cxt = Cxt::new(0.1);
    
    let measurements: Vec<Vec<f32>> = vec![vec![5.0, 10.0],
                                           vec![6.0, 8.0],
                                           vec![7.0, 6.0],
                                           vec![8.0, 4.0],
                                           vec![9.0, 2.0],
                                           vec![10.0, 0.0]];


    let initial_xy: Vec<f32> = vec![4.0, 12.0];

    // initial state
    let x = Matrix::new(vec![vec![initial_xy[0]],
                             vec![initial_xy[1]],
                             vec![0.0],
                             vec![0.0]]);

    // initial uncertainty
    let p = Matrix::new(vec![vec![0.0, 0.0, 0.0, 0.0],
                             vec![0.0, 0.0, 0.0, 0.0],
                             vec![0.0, 0.0, 1000.0, 0.0],
                             vec![0.0, 0.0, 0.0, 1000.0]]); 

    kalman_filter(&cxt, &measurements, &x, &p);
}


fn kalman_filter(cxt: &Cxt, measurements: &Vec<Vec<f32>>, x: &Matrix, p: &Matrix) {
    let mut x = x.clone(); // pos and vel
    let mut p = p.clone(); // initial uncertainty

    let h_t = cxt.h.transpose();

    for n in range(0, measurements.len()) {
        // prediction
        x = cxt.f.mul(&x).add(&cxt.u);
        p = cxt.f.mul(&p).mul(&cxt.f.transpose());

        // measurement update
        let z = Matrix::new(vec![measurements[n].clone()]);  // use z for higher dimensional spaces

        let y = z.transpose().sub(&cxt.h.mul(&x));   // y = trans(z) - (h * x)
        let s = cxt.h.mul(&p).mul(&h_t).add(&cxt.r); // s = h * p * trans(h) + r
        let k = p.mul(&h_t).mul(&s.inverse());       // k = p * trans(h) * inv(s)
        x = x.add(&k.mul(&y));                       // x := x + (k * y)
        p = cxt.i.sub(&k.mul(&cxt.h)).mul(&p);       // p := (i - (k * h)) * p
    }

    // print
    println!("x= {}", x.value);
    println!("p= {}", p.value);
}
