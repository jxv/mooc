fn smooth(path: &Vec<Vec<f32>>, fix: &Vec<bool>) -> Vec<Vec<f32>> {
    smooth_with_extra(path, fix, 0.0, 0.1, 0.00001)
}


fn smooth_with_extra(path: &Vec<Vec<f32>>, fix: &Vec<bool>, weight_data: f32, weight_smooth: f32, tolerance: f32) -> Vec<Vec<f32>> {
    let mut new_path = path.clone();
    let mut quit = false;

    let gamma = weight_smooth / 2.0;
    let len = path.len();
    let max_idx = len - 1;

    let mut change = tolerance;
    while change >= tolerance {
        change = 0.0;
        for i in range(0u, path.len()) {
            if fix[i] {
                continue
            }

            let prev  = if i == 0 { max_idx } else { i - 1 };
            let next  = if i == max_idx { 0 } else { i + 1 };

            let prev2 = if i <= 1 { (max_idx + 1) - i } else { i - (1 + 1) };
            let next2 = if i >= max_idx - 1 { (i + 1) - max_idx } else { i + (1 + 1) };

            for j in range(0u, path[i].len()) {

                let aux = new_path[i][j];

                *new_path.get_mut(i).get_mut(j) += weight_data * (path[i][j] - new_path[i][j]);

                *new_path.get_mut(i).get_mut(j) += weight_smooth *
                        (new_path[prev][j] + new_path[next][j] - 2.0 * new_path[i][j]);

                *new_path.get_mut(i).get_mut(j) += gamma * (2.0 * new_path[prev][j] - new_path[prev2][j] - new_path[i][j]);
                *new_path.get_mut(i).get_mut(j) += gamma * (2.0 * new_path[next][j] - new_path[next2][j] - new_path[i][j]);

                change += (aux - new_path[i][j]).abs();
            }
        }
    }

    new_path
}


//


fn close_enough(user_answer: f32, true_answer: f32) -> bool {
    close_enough_with_extra(user_answer, true_answer, 0.001)
}


fn close_enough_with_extra(user_answer: f32, true_answer: f32, epsilon: f32) -> bool {
    !((user_answer - true_answer).abs() > epsilon)
}


fn solution_check(new_path: &Vec<Vec<f32>>, answer: &Vec<Vec<f32>>) -> bool {
    if new_path.len() != answer.len() {
        println!("Error: Your new_path is not the correct length");
        return false
    }

    if new_path[0].len() != answer[0].len() {
        println!("Error: Your entries do not contain an (x,y) coordinate pair.");
        return false
    }

    for i in range(0u, new_path.len()) {
        for j in range(0u, new_path[i].len()) {
            if !close_enough(new_path[i][j], answer[i][j]) {
                println!("Error, entry {} and possibly more are not correct.", (i,j));
                return false
            }
        }
    }
    
    println!("Test case is correct!");
    true
}


fn main() {
    let testpath1 = vec![
        vec![0.0, 0.0],
        vec![1.0, 0.0],
        vec![2.0, 0.0],
        vec![3.0, 0.0],
        vec![4.0, 0.0],
        vec![5.0, 0.0],
        vec![6.0, 0.0],
        vec![6.0, 1.0],
        vec![6.0, 2.0],
        vec![6.0, 3.0],
        vec![5.0, 3.0],
        vec![4.0, 3.0],
        vec![3.0, 3.0],
        vec![2.0, 3.0],
        vec![1.0, 3.0],
        vec![0.0, 3.0],
        vec![0.0, 2.0],
        vec![0.0, 1.0]];

    let fix1 = vec![true, false, false, false, false, false,
                    true, false, false,
                    true, false, false, false, false, false,
                    true, false, false];

    let respath1 = smooth(&testpath1, &fix1);
    
    for i in range(0u, testpath1.len()) {
        println!("{} -> {}", (testpath1[i][0], testpath1[i][1]), (respath1[i][0], respath1[i][1]));
    }
}
