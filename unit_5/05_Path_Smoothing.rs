fn main() {
    let path: Vec<Vec<f32>> = vec![
        vec![0.0, 0.0],
        vec![0.0, 1.0],
        vec![0.0, 2.0],
        vec![1.0, 2.0],
        vec![2.0, 2.0],
        vec![3.0, 2.0],
        vec![4.0, 2.0],
        vec![4.0, 3.0],
        vec![4.0, 4.0]];

    let new_path = smooth(&path, 0.5, 0.1, 0.00001);

    for i in range(0, path.len()) {
        println!("[({:.3f},{:.3f})] -> [({:.3f},{:.3f})]",
                path[i][0], path[i][1], new_path[i][0], new_path[i][1]);
    }
}


fn smooth(path: &Vec<Vec<f32>>, weight_data: f32, weight_smooth: f32, tolerance: f32) -> Vec<Vec<f32>> {
    let mut new_path = path.clone();
    let mut ppath = Vec::from_elem(path.len(), vec![0.0,0.0]);

    let mut quit = false;
    while !quit {
        quit = true;
        for i in range(0, path.len() - 2) {
            for j in range(0, path[0].len()) {
                let k = i + 1;
                *new_path.get_mut(k).get_mut(j) = new_path[k][j] 
                                                + weight_data * (path[k][j] - new_path[k][j]);
                
                *new_path.get_mut(k).get_mut(j) = new_path[k][j] 
                                                + weight_smooth * ( new_path[k+1][j]
                                                                  + new_path[k-1][j]
                                                                  - (2.0 * new_path[k][j]));
                if (ppath[k][j] - new_path[k][j]).abs() >= tolerance {
                    quit = false;
                }
                *ppath.get_mut(k).get_mut(j) = new_path[k][j];
            }
        }
    }
    
    new_path 
}
