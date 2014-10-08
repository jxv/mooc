fn main() {
    let n = 5;
    let p = Vec::from_elem(n, 1.0 / n as f32);
    println!("{}", p);
}
