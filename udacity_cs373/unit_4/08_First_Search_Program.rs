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
    // expected: (4, 5, 11)
    println!("{}", search());
}


fn search() -> Option<(int, int, int)> {
    let mut open = Vec::new();
    let mut close = Vec::new();

    open.push((INIT.0, INIT.1, 0i));

    loop {
        let cell = match open.pop() {
            None => break,
            Some(cell) => cell,
        };
        close.push(cell);
        
        // Test for reached goal?
        if cell.0 == GOAL.0 && cell.1 == GOAL.1 {
            return Some(cell);
        }

        // Scan for neighbors.
        let mut neighbors = Vec::new();
        for d in DELTA.iter() {
            let n = (cell.0 + d.0, cell.1 + d.1, cell.2 + COST);
            // Within grid?
            if n.0 >= 0 && n.0 < GRID.len() as int && n.1 >= 0 && n.1 < GRID[0].len() as int {
                neighbors.push(n);
            }
        }

        // Is the neighboring space an open space?
        for &n in neighbors.iter() {
            if GRID[n.0 as uint][n.1 as uint] == X {
                // No! Add it to the close vec to ignore it!
                close.push(n);
            }
        }

        // Add all non-scanned neighbors to the open vec.
        // Ignore previously scanned neighbors.
        for &n in neighbors.iter() {
            if     !open.iter().any(|node|  { node.0 == n.0 && node.1 == n.1 })
                && !close.iter().any(|node| { node.0 == n.0 && node.1 == n.1 }) {
                open.push(n);
            }
        }
    }

    None
}
