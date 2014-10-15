#![feature(tuple_indexing)]

#[deriving(Show,PartialEq,Clone)]
enum Space {
    O,
    X,
}

static GRID: [[Space, ..6], ..5] = [
    [O, X, O, O, O, O],
    [O, X, O, O, O, O],
    [O, X, O, O, O, O],
    [O, X, O, O, O, O],
    [O, O, O, O, X, O]];

static HEURISTIC: [[int, ..6], ..5] = [
    [9, 8, 7, 6, 5, 4],
    [8, 7, 6, 5, 4, 3],
    [7, 6, 5, 4, 3, 2],
    [6, 5, 4, 3, 2, 1],
    [5, 4, 3, 2, 1, 0]];

static INIT: (int, int) = (0, 0);
static GOAL: (int, int) = (5-1, 6-1);

static DELTA: [(int, int), ..4] = [
    (-1, 0),
    (0, -1),
    (1, 0),
    (0, 1)];

#[allow(dead_code)]
static DELTA_NAME: [char, ..4] = ['^', '<','v','>'];

static COST: int = 1;


fn main() {
    for row in search().iter() {
        println!("{}", row);
    }
}


fn search() -> Vec<Vec<int>> {
    let mut closed = Vec::from_elem(GRID.len(), Vec::from_elem(GRID[0].len(), false));
    let mut expand = Vec::from_elem(GRID.len(), Vec::from_elem(GRID[0].len(), -1i));
    let mut action = Vec::from_elem(GRID.len(), Vec::from_elem(GRID[0].len(), -1i));

    *closed.get_mut(INIT.0 as uint).get_mut(INIT.1 as uint) = true;

    let init_g = 0i;
    let init_h = HEURISTIC[INIT.0 as uint][INIT.1 as uint];
    let mut open = vec![(init_g + init_h, init_g, init_h, INIT.0, INIT.1)];

    let mut found = false;
    let mut resign = false;
    let mut count = 0i;

    while !found && !resign {
        if open.len() == 0 {
            resign = true;
        } else {
            open.sort_by(|a,b| b.cmp(a));
            println!("{}", open);
            let (_, g, _, y, x) = open.pop().unwrap();
            *expand.get_mut(y as uint).get_mut(x as uint) = count;
            count += 1;

            if y == GOAL.0 && x == GOAL.1 {
                found = true;
            } else {
                for i in range(0, DELTA.len()) {
                    let y2 = y + DELTA[i].0;
                    let x2 = x + DELTA[i].1;

                    let yu = y2 as uint;
                    let xu = x2 as uint;
                    if y2 >= 0 && yu < GRID.len() && x2 >= 0 && xu < GRID[0].len() {
                        if !closed[yu][xu] && GRID[yu][xu] == O {
                            let h2 = HEURISTIC[yu][xu];
                            let g2 = g + COST;
                            let f2 = g2 + h2;
                            open.push((f2, g2, h2, y2, x2));
                            *closed.get_mut(yu).get_mut(xu) = true;
                        }
                    }
                }
            }
        }
    }
    expand
}
