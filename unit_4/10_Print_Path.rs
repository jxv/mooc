#![feature(tuple_indexing)]

#[deriving(Show,PartialEq,Clone)]
enum Space {
    O,
    X,
}

static GRID: [[Space, ..6], ..5] = [
    [O, O, X, O, O, O],
    [O, O, X, O, O, O],
    [O, O, O, O, X, O],
    [O, O, X, X, X, O],
    [O, O, O, O, X, O]];

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


fn search() -> Vec<Vec<char>> {
    let mut closed = Vec::from_elem(GRID.len(), Vec::from_elem(GRID[0].len(), false));
    let mut map = Vec::from_elem(GRID.len(), Vec::from_elem(GRID[0].len(), (0,0,0)));
    let mut path = Vec::from_elem(GRID.len(), Vec::from_elem(GRID[0].len(), ' '));

    *closed.get_mut(INIT.0 as uint).get_mut(INIT.1 as uint) = true;

    let mut open = vec![(0i, INIT.0, INIT.1)];

    let mut found = false;
    let mut resign = false;

    while !found && !resign {
        if open.len() == 0 {
            resign = true;
        } else {
            open.sort();
            let (g, y, x) = open.pop().unwrap();
            if y == GOAL.0 && x == GOAL.1 {
                found = true;
            } else {
                for i in range(0,DELTA.len()) {
                    let y2 = y + DELTA[i].0;
                    let x2 = x + DELTA[i].1;

                    let yu = y2 as uint;
                    let xu = x2 as uint;
                    if y2 >= 0 && yu < GRID.len() && x2 >= 0 && xu < GRID[0].len() {
                        if !closed[yu][xu] && GRID[yu][xu] == O {
                            let g2 = g + COST;
                            open.push((g2, y2, x2));
                            *closed.get_mut(yu).get_mut(xu) = true;
                            *map.get_mut(yu).get_mut(xu) = (y, x, i);
                            if INIT.0 == y && INIT.1 == x {
                                *map.get_mut(y as uint).get_mut(x as uint) = (y, x, i);
                            }
                        }
                    }
                }
            }
        }
    }


    let mut cur = map[GOAL.0 as uint][GOAL.1 as uint];
    let mut p = vec![cur];
    while cur != map[INIT.0 as uint][INIT.1 as uint] {
        cur = map[cur.0 as uint][cur.1 as uint];
        p.push(cur);
    }
    for p in p.iter() {
        *path.get_mut(p.0 as uint).get_mut(p.1 as uint) = DELTA_NAME[p.2 as uint];
    }
    *path.get_mut(GOAL.0 as uint).get_mut(GOAL.1 as uint) = '*';

    path
}

