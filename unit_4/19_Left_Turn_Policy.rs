#![feature(tuple_indexing)]

#[deriving(Show,PartialEq,Clone)]
enum Space {
    O,
    X,
}

#[deriving(Show,PartialEq,Clone)]
enum Dir {
    Up,
    Left,
    Down,
    Right,
}

fn from_dir(dir: Dir) -> uint {
    match dir { Up => 0, Left => 1, Down => 2, Right => 3 }
}

#[deriving(Show,PartialEq,Clone)]
enum Action {
    LeftTurn,
    NoTurn,
    RightTurn,
}

/*
fn from_action(action: Action) -> uint {
    match action {  LeftTurn => 0, NoTurn => 1, RightTurn => 2 }
}
*/

static GRID: [[Space, ..6], ..5] = [
    [X, X, X, O, O, O],
    [X, X, X, O, X, O],
    [O, O, O, O, O, O],
    [X, X, X, O, X, X],
    [X, X, X, O, X, X]];

static GOAL: (uint, uint) = (2, 0);
static INIT: (uint, uint, Dir) = (4, 3, Up);
static DIRS: [Dir, ..4] = [Up, Left, Down, Right];
static ACTIONS: [Action, ..3] = [LeftTurn, NoTurn, RightTurn];

fn forward((y,x): (uint, uint), dir: Dir) -> Option<(uint, uint)> {
    let (y2, x2) = match dir {
        Up    => (y - 1, x),
        Left  => (y, x - 1),
        Down  => (y + 1, x),
        Right => (y, x + 1),
    };
    if y2 < GRID.len() && x2 < GRID[0].len() { Some((y2,x2)) } else { None }
}

fn action_name(action: Action) -> char {
    match action { LeftTurn => 'L', NoTurn => '#', RightTurn => 'R' }
}

fn cost(action: Action) -> uint {
    match action { LeftTurn => 20, NoTurn => 1, RightTurn => 2 }
}

fn apply_action_on_dir(dir: Dir, action: Action) -> Dir {
    match dir {
        Up    => match action { LeftTurn => Left,  NoTurn => Up,    RightTurn => Right, },
        Left  => match action { LeftTurn => Down,  NoTurn => Left,  RightTurn => Up,    },
        Down  => match action { LeftTurn => Right, NoTurn => Down,  RightTurn => Left,  },
        Right => match action { LeftTurn => Up,    NoTurn => Right, RightTurn => Down,  },
    }
}

fn propagate_dir_from_action(action: Action, dir: Dir) -> Dir {
    match action {
        LeftTurn  => match dir { Up => Right, Left => Up,   Down => Left,  Right => Down  },
        NoTurn    => match dir { Up => Up,    Left => Left, Down => Down,  Right => Right },
        RightTurn => match dir { Up => Left,  Left => Down, Down => Right, Right => Up    },
    }
}

fn main() {
    for row in optimum_policy_2d().iter() {
        println!("{}", row);
    }
}

fn optimum_policy_2d() -> Vec<Vec<char>> {
    let mut value = Vec::from_elem(4, Vec::from_elem(GRID.len(), Vec::from_elem(GRID[0].len(), 999u)));
    let mut policy = Vec::from_elem(4, Vec::from_elem(GRID.len(), Vec::from_elem(GRID[0].len(), ' ')));
    let mut policy_2d = Vec::from_elem(GRID.len(), Vec::from_elem(GRID[0].len(), ' '));

    let mut change = true;

    // get value fn and policy action

    while change {
        change = false;
        for y in range(0, GRID.len()) {
            for x in range(0, GRID[0].len()) {
                for &d in DIRS.iter() {
                    if GOAL.0 == y && GOAL.1 == x {
                        if value[from_dir(d)][y][x] > 0 {
                            change = true;
                            *value.get_mut(from_dir(d)).get_mut(y).get_mut(x) = 0;
                            *policy.get_mut(from_dir(d)).get_mut(y).get_mut(x) = '*';
                        }
                    } else if GRID[y][x] == O {
                        for &a in ACTIONS.iter() {
                            let d2 = apply_action_on_dir(d, a);
                            let (y2,x2) = match forward((y,x), d2) { None => continue, Some(yx) => yx };
                            let v2 = match GRID[y2][x2] {
                                X => continue,
                                O => value[from_dir(d2)][y2][x2] + cost(a)
                            };
                            if v2 < value[from_dir(d)][y][x] {
                                *value.get_mut(from_dir(d)).get_mut(y).get_mut(x) = v2;
                                *policy.get_mut(from_dir(d)).get_mut(y).get_mut(x) = action_name(a);
                                change = true;
                            }
                        }
                    }
                }
            }
        }
    }

    //

    let mut y = INIT.0;
    let mut x = INIT.1;
    let mut d = INIT.2;


    *policy_2d.get_mut(y).get_mut(x) = policy[from_dir(d)][y][x];
    while policy[from_dir(d)][y][x] != '*' {
        let d2 = match policy[from_dir(d)][y][x] {
            '#' => d,
            'R' => apply_action_on_dir(d, RightTurn),
            'L' => apply_action_on_dir(d, LeftTurn),
            _ => break,
        };
        let (y2, x2) = match forward((y,x), d2) { None => break, Some(yx) => yx };

        y = y2;
        x = x2;
        d = d2;

        *policy_2d.get_mut(y).get_mut(x) = policy[from_dir(d)][y][x];
    }


    policy_2d
}
