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

#[deriving(Show,PartialEq,Clone)]
enum Offset {
    LeftOffset,
    NoOffset,
    RightOffset,
}


static GRID: [[Space, ..4], ..4] = [
    [O, O, O, O],
    [O, O, O, O],
    [O, O, O, O],
    [O, X, X, O]];

static GOAL: (uint, uint) = (0, 3);
static DELTA: [Dir, ..4] = [Up, Left, Down, Right];
static OFFSETS: [Offset, ..3] = [NoOffset, RightOffset, LeftOffset];
static SUCCESS_PROB: f32 = 0.5;
static FAILURE_PROB: f32 = (1.0 - SUCCESS_PROB) / 2.0;

static COLLISION_COST: f32 = 100.0;
static COST_STEP: f32 = 1.0;

fn delta((y,x): (uint, uint), dir: Dir) -> Option<(uint, uint)> {
    let (y2, x2) = match dir {
        Up    => (y - 1, x),
        Left  => (y, x - 1),
        Down  => (y + 1, x),
        Right => (y, x + 1),
    };
    if y2 < GRID.len() && x2 < GRID[0].len() { Some((y2,x2)) } else { None }
}


fn delta_name(dir: Dir) -> char {
    match dir { Up => '^', Down => 'v', Left => '<', Right => '>' }
}

fn apply_offset_on_dir(dir: Dir, offset: Offset) -> Dir {
    match dir {
        Up    => match offset { LeftOffset => Left,  NoOffset => Up,    RightOffset => Right, },
        Left  => match offset { LeftOffset => Down,  NoOffset => Left,  RightOffset => Up,    },
        Down  => match offset { LeftOffset => Right, NoOffset => Down,  RightOffset => Left,  },
        Right => match offset { LeftOffset => Up,    NoOffset => Right, RightOffset => Down,  },
    }
}


fn main() {
    let (value, policy) = stochastic_value();
    
    for v in value.iter() {
        println!("{}", v);
    }
    
    for p in policy.iter() {
        println!("{}", p);
    }
}

fn stochastic_value() -> (Vec<Vec<f32>>, Vec<Vec<char>>) {
    let mut value = Vec::from_elem(GRID.len(), Vec::from_elem(GRID[0].len(), 1000.0));
    let mut policy = Vec::from_elem(GRID.len(), Vec::from_elem(GRID[0].len(), ' '));

    let mut change = true;
    while change {
        change = false;

        for y in range(0, GRID.len()) {
            for x in range(0, GRID[0].len()) {
                if GOAL.0 == y && GOAL.1 == x {
                    if value[y][x] > 0.0 {
                        *value.get_mut(y).get_mut(x) = 0.0;
                        *policy.get_mut(y).get_mut(x) = '*';
                        change = true;
                        continue
                    }
                }

                if GRID[y][x] != O {
                    continue
                }

                for &d in DELTA.iter() {
                    let mut v2 = COST_STEP;
                    for &o in OFFSETS.iter() {
                        let prob = match o { NoOffset => SUCCESS_PROB, _ => FAILURE_PROB };
                        v2 += prob * match delta((y,x), apply_offset_on_dir(d, o)) {
                            Some((y2,x2)) => if GRID[y2][x2] == O { value[y2][x2] } else { COLLISION_COST },
                            None => COLLISION_COST,
                        };
                    }

                    if v2 < value[y][x] {
                        change = true;
                        *value.get_mut(y).get_mut(x) = v2;
                        *policy.get_mut(y).get_mut(x) = delta_name(d);
                    }
                }
            }
        }
    }

    (value, policy)
}
