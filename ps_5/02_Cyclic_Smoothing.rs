fn smooth(path: &Vec<Vec<f32>>) -> Vec<Vec<f32>> {
    smooth_with_extra(path, 0.1, 0.1, 0.00001)
}


fn smooth_with_extra(path: &Vec<Vec<f32>>, weight_data: f32, weight_smooth: f32, tolerance: f32) -> Vec<Vec<f32>> {
    let mut new_path = path.clone();
    let mut ppath = path.clone();

    let mut quit = false;
    while !quit {
        quit = true;

        for i in range(0u, path.len()) {
            let prev = if i == 0 { path.len() - 1 } else { i - 1 };
            let next = if i == path.len() - 1 { 0 } else { i + 1 };

            for j in range(0u, path[i].len()) {
                *new_path.get_mut(i).get_mut(j) = new_path[i][j]
                                                + weight_data * (path[i][j] - new_path[i][j]);
                
                *new_path.get_mut(i).get_mut(j) = new_path[i][j]
                                                + weight_smooth * ( new_path[next][j]
                                                                  + new_path[prev][j]
                                                                  - 2.0 * new_path[i][j]);

                if (ppath[i][j] - new_path[i][j]).abs() >= tolerance {
                    quit = false;
                }
                *ppath.get_mut(i).get_mut(j) = new_path[i][j];
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

   let answer1 = vec![
	   vec![0.5449300156668018, 0.47485226780102946],
       vec![1.2230705677535505, 0.2046277687200752],
       vec![2.079668890615267, 0.09810778721159963],
       vec![3.0000020176660755, 0.07007646364781912],
       vec![3.9203348821839112, 0.09810853832382399],
       vec![4.7769324511170455, 0.20462917195702085],
       vec![5.455071854686622, 0.4748541381544533],
       vec![5.697264197153936, 1.1249625336275617],
       vec![5.697263485026567, 1.8750401628534337],
       vec![5.455069810373743, 2.5251482916876378],
       vec![4.776929339068159, 2.795372759575895],
       vec![3.92033110541304, 2.9018927284871063],
       vec![2.999998066091118, 2.929924058932193],
       vec![2.0796652780381826, 2.90189200881968],
       vec![1.2230677654766597, 2.7953714133566603],
       vec![0.544928391271399, 2.5251464933327794],
	   vec![0.3027360471605494, 1.875038145804603],
	   vec![0.302736726373967, 1.1249605602741133]];

	solution_check(&smooth(&testpath1), &answer1);
}
