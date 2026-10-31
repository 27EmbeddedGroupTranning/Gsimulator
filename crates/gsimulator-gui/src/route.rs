//! Orthogonal wire routing that keeps out of the parts' way.
//!
//! Wires are drawn the way an EDA tool draws them: only horizontal and vertical
//! segments, leaving a pin along its stub, and going around the symbol bodies
//! instead of through them. The router first tries the two elbow shapes and a
//! handful of Z shapes; only when the drawing is too crowded for those does it
//! fall back to a small A\* search over the "interesting" coordinates of the
//! board (the Hanan grid induced by the part edges and the two endpoints).
//!
//! Everything here works in board space and is unit tested without a window.

use egui::{Pos2, Rect, Vec2};

/// Which way a pin points out of its symbol.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Dir {
    Left,
    Right,
    Up,
    Down,
}

impl Dir {
    /// Unit vector pointing away from the body.
    pub fn vector(self) -> Vec2 {
        match self {
            Dir::Left => Vec2::new(-1.0, 0.0),
            Dir::Right => Vec2::new(1.0, 0.0),
            Dir::Up => Vec2::new(0.0, -1.0),
            Dir::Down => Vec2::new(0.0, 1.0),
        }
    }
}

/// Empty space kept between a wire and a part body.
pub const CLEARANCE: f32 = 9.0;
/// How far a wire leaves a pin straight along its stub before it may turn.
const PIN_RUN: f32 = 15.0;
/// Cost of one corner, in board units; keeps long straight runs preferred.
const BEND_COST: f32 = 40.0;

/// Routes a wire between two pin tips.
///
/// `start_side`/`goal_side` are the sides the pins sit on (if known): the wire
/// leaves and enters along the stub, like a real schematic. `obstacles` are the
/// symbol bodies to route around.
pub fn route(
    start: Pos2,
    start_side: Option<Dir>,
    goal: Pos2,
    goal_side: Option<Dir>,
    obstacles: &[Rect],
) -> Vec<Pos2> {
    let start_gate = start + start_side.map_or(Vec2::ZERO, |side| side.vector() * PIN_RUN);
    let goal_gate = goal + goal_side.map_or(Vec2::ZERO, |side| side.vector() * PIN_RUN);

    let mut points = vec![start];
    points.extend(route_between(start_gate, goal_gate, obstacles));
    points.push(goal);
    simplify(&points)
}

/// Routes an orthogonal polyline between two free points.
pub fn route_between(start: Pos2, goal: Pos2, obstacles: &[Rect]) -> Vec<Pos2> {
    if (start - goal).length() < 0.5 {
        return vec![start];
    }
    if let Some(points) = simple_path(start, goal, obstacles) {
        return simplify(&points);
    }
    if let Some(points) = search_path(start, goal, obstacles) {
        return simplify(&points);
    }
    // Nothing fits (a part pinned between two walls): keep the wire readable
    // instead of dropping the connection.
    simplify(&[
        start,
        Pos2::new((start.x + goal.x) * 0.5, start.y),
        Pos2::new((start.x + goal.x) * 0.5, goal.y),
        goal,
    ])
}

/// The elbow and Z shapes that cover almost every wire on a tidy drawing.
fn simple_path(start: Pos2, goal: Pos2, obstacles: &[Rect]) -> Option<Vec<Pos2>> {
    let (sx, sy) = (start.x, start.y);
    let (gx, gy) = (goal.x, goal.y);

    let mut candidates: Vec<Vec<Pos2>> = vec![
        vec![start, Pos2::new(gx, sy), goal],
        vec![start, Pos2::new(sx, gy), goal],
    ];

    // Z shapes: turn on a coordinate taken from the surrounding parts.
    for x in interesting_x(obstacles) {
        candidates.push(vec![start, Pos2::new(x, sy), Pos2::new(x, gy), goal]);
    }
    for y in interesting_y(obstacles) {
        candidates.push(vec![start, Pos2::new(sx, y), Pos2::new(gx, y), goal]);
    }

    let mut best: Option<(f32, Vec<Pos2>)> = None;
    for candidate in candidates {
        if !orthogonal(&candidate) || collides(&candidate, obstacles) {
            continue;
        }
        let length: f32 = candidate
            .windows(2)
            .map(|pair| (pair[1] - pair[0]).length())
            .sum();
        let bends = corner_count(&candidate) as f32;
        let cost = length + BEND_COST * bends;
        if best.as_ref().is_none_or(|(best_cost, _)| cost < *best_cost) {
            best = Some((cost, candidate));
        }
    }
    best.map(|(_, points)| points)
}

/// A\* over the Hanan grid induced by the part edges and the endpoints.
fn search_path(start: Pos2, goal: Pos2, obstacles: &[Rect]) -> Option<Vec<Pos2>> {
    let mut xs = vec![start.x, goal.x];
    let mut ys = vec![start.y, goal.y];
    for rect in obstacles {
        xs.extend([rect.left(), rect.right()]);
        ys.extend([rect.top(), rect.bottom()]);
    }
    xs.sort_by(f32::total_cmp);
    xs.dedup_by(|a, b| (*a - *b).abs() < 0.5);
    ys.sort_by(f32::total_cmp);
    ys.dedup_by(|a, b| (*a - *b).abs() < 0.5);

    let width = xs.len();
    let height = ys.len();
    let nx = |x: f32| xs.iter().position(|value| (*value - x).abs() < 0.5);
    let ny = |y: f32| ys.iter().position(|value| (*value - y).abs() < 0.5);
    let start_node = (nx(start.x)?, ny(start.y)?);
    let goal_node = (nx(goal.x)?, ny(goal.y)?);

    let node = |xi: usize, yi: usize| yi * width + xi;
    let blocked = |xi: usize, yi: usize| {
        let point = Pos2::new(xs[xi], ys[yi]);
        obstacles
            .iter()
            .any(|rect| rect.shrink(0.5).contains(point))
    };
    let blocked_edge = |a: (usize, usize), b: (usize, usize)| {
        let mid = Pos2::new((xs[a.0] + xs[b.0]) * 0.5, (ys[a.1] + ys[b.1]) * 0.5);
        obstacles.iter().any(|rect| rect.shrink(0.5).contains(mid))
    };

    let count = width * height;
    let mut best = vec![f32::INFINITY; count];
    let mut previous = vec![usize::MAX; count];
    let mut heap: std::collections::BinaryHeap<(std::cmp::Reverse<u64>, usize)> =
        std::collections::BinaryHeap::new();

    let start_index = node(start_node.0, start_node.1);
    let goal_index = node(goal_node.0, goal_node.1);
    best[start_index] = 0.0;
    heap.push((std::cmp::Reverse(0), start_index));

    let mut visited = vec![false; count];
    let mut came_from_dir: Vec<Option<Dir>> = vec![None; count];

    while let Some((std::cmp::Reverse(_), current)) = heap.pop() {
        if visited[current] {
            continue;
        }
        visited[current] = true;
        if current == goal_index {
            break;
        }
        let (xi, yi) = (current % width, current / width);
        let neighbours = [
            (xi.wrapping_sub(1), yi, Dir::Left),
            (xi + 1, yi, Dir::Right),
            (xi, yi.wrapping_sub(1), Dir::Up),
            (xi, yi + 1, Dir::Down),
        ];
        for (nx_i, ny_i, dir) in neighbours {
            if nx_i >= width || ny_i >= height {
                continue;
            }
            let next = node(nx_i, ny_i);
            if visited[next] {
                continue;
            }
            if blocked_edge((xi, yi), (nx_i, ny_i)) {
                continue;
            }
            let is_goal = next == goal_index;
            if !is_goal && blocked(nx_i, ny_i) {
                continue;
            }
            let step = (xs[nx_i] - xs[xi]).abs() + (ys[ny_i] - ys[yi]).abs();
            let turn = match came_from_dir[current] {
                Some(previous) if previous != dir => BEND_COST,
                _ => 0.0,
            };
            let cost = best[current] + step + turn;
            if cost < best[next] {
                best[next] = cost;
                previous[next] = current;
                came_from_dir[next] = Some(dir);
                // Heuristic: Manhattan distance, which never overestimates.
                let heuristic =
                    (xs[goal_node.0] - xs[nx_i]).abs() + (ys[goal_node.1] - ys[ny_i]).abs();
                heap.push((std::cmp::Reverse(((cost + heuristic) * 10.0) as u64), next));
            }
        }
    }

    if best[goal_index].is_infinite() {
        return None;
    }

    let mut path = vec![goal_index];
    let mut current = goal_index;
    while current != start_index {
        let before = previous[current];
        if before == usize::MAX {
            return None;
        }
        path.push(before);
        current = before;
    }
    path.reverse();
    let mut points: Vec<Pos2> = path
        .into_iter()
        .map(|index| Pos2::new(xs[index % width], ys[index / width]))
        .collect();
    // Drop the collinear middle points the grid search produces.
    points.dedup_by(|a, b| (*a - *b).length() < 0.3);
    Some(points)
}

/// Coordinates of the part edges, used for the Z shapes.
fn interesting_x(obstacles: &[Rect]) -> Vec<f32> {
    let mut values: Vec<f32> = obstacles
        .iter()
        .flat_map(|rect| [rect.left(), rect.right()])
        .collect();
    values.sort_by(f32::total_cmp);
    values.dedup_by(|a, b| (*a - *b).abs() < 0.5);
    values
}

fn interesting_y(obstacles: &[Rect]) -> Vec<f32> {
    let mut values: Vec<f32> = obstacles
        .iter()
        .flat_map(|rect| [rect.top(), rect.bottom()])
        .collect();
    values.sort_by(f32::total_cmp);
    values.dedup_by(|a, b| (*a - *b).abs() < 0.5);
    values
}

/// True when every segment is horizontal or vertical.
pub fn orthogonal(points: &[Pos2]) -> bool {
    points
        .windows(2)
        .all(|pair| (pair[0].x - pair[1].x).abs() < 0.01 || (pair[0].y - pair[1].y).abs() < 0.01)
}

/// True when any segment of the polyline runs through one of the obstacles.
pub fn collides(points: &[Pos2], obstacles: &[Rect]) -> bool {
    points.windows(2).any(|pair| {
        let segment = Rect::from_two_pos(pair[0], pair[1]);
        obstacles
            .iter()
            .any(|rect| rect.shrink(0.5).intersects(segment))
    })
}

/// Number of corners of a polyline.
pub fn corner_count(points: &[Pos2]) -> usize {
    points
        .windows(3)
        .filter(|triple| !collinear(triple))
        .count()
}

fn collinear(triple: &[Pos2]) -> bool {
    let a = triple[0];
    let b = triple[1];
    let c = triple[2];
    ((a.x - b.x).abs() < 0.01 && (b.x - c.x).abs() < 0.01)
        || ((a.y - b.y).abs() < 0.01 && (b.y - c.y).abs() < 0.01)
}

/// Drops duplicate and collinear points so the polyline is minimal.
pub fn simplify(points: &[Pos2]) -> Vec<Pos2> {
    let mut out: Vec<Pos2> = Vec::with_capacity(points.len());
    for point in points {
        if out
            .last()
            .is_some_and(|last| (*last - *point).length() < 0.3)
        {
            continue;
        }
        out.push(*point);
    }
    let mut index = 1;
    while index + 1 < out.len() {
        if collinear(&out[index - 1..=index + 1]) {
            out.remove(index);
        } else {
            index += 1;
        }
    }
    out
}

/// Length of a polyline in board units.
pub fn length(points: &[Pos2]) -> f32 {
    points
        .windows(2)
        .map(|pair| (pair[1] - pair[0]).length())
        .sum()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn rect(x: f32, y: f32, w: f32, h: f32) -> Rect {
        Rect::from_min_size(Pos2::new(x, y), Vec2::new(w, h))
    }

    #[test]
    fn a_clear_run_is_two_segments() {
        let points = route_between(Pos2::new(0.0, 0.0), Pos2::new(100.0, 60.0), &[]);
        assert!(orthogonal(&points));
        assert_eq!(corner_count(&points), 1);
        assert_eq!(points.first(), Some(&Pos2::new(0.0, 0.0)));
        assert_eq!(points.last(), Some(&Pos2::new(100.0, 60.0)));
    }

    #[test]
    fn a_part_between_the_pins_is_walked_around() {
        // A wall in the middle, taller than the two endpoints.
        let wall = rect(40.0, -40.0, 40.0, 200.0);
        let points = route_between(Pos2::new(0.0, 50.0), Pos2::new(120.0, 50.0), &[wall]);
        assert!(orthogonal(&points), "{points:?}");
        assert!(!collides(&points, &[wall]), "{points:?}");
        assert!(
            points.iter().any(|point| point.y < 0.0 || point.y > 160.0),
            "the wire has to leave the corridor: {points:?}"
        );
    }

    #[test]
    fn a_short_part_can_be_walked_over_or_under() {
        let block = rect(40.0, 20.0, 40.0, 40.0);
        let points = route_between(Pos2::new(0.0, 40.0), Pos2::new(120.0, 40.0), &[block]);
        assert!(!collides(&points, &[block]), "{points:?}");
        assert!(orthogonal(&points));
    }

    #[test]
    fn the_wire_leaves_the_pin_along_its_stub() {
        let points = route(
            Pos2::new(100.0, 100.0),
            Some(Dir::Left),
            Pos2::new(100.0, 300.0),
            Some(Dir::Left),
            &[],
        );
        assert_eq!(points.first(), Some(&Pos2::new(100.0, 100.0)));
        assert_eq!(points.last(), Some(&Pos2::new(100.0, 300.0)));
        let first = points[1];
        assert!(
            first.x < 100.0,
            "the wire must leave to the left: {points:?}"
        );
        let last = points[points.len() - 2];
        assert!(last.x < 100.0, "and come back from the left: {points:?}");
    }

    #[test]
    fn simplify_removes_collinear_points() {
        let points = vec![
            Pos2::new(0.0, 0.0),
            Pos2::new(10.0, 0.0),
            Pos2::new(20.0, 0.0),
            Pos2::new(20.0, 10.0),
        ];
        assert_eq!(
            simplify(&points),
            vec![
                Pos2::new(0.0, 0.0),
                Pos2::new(20.0, 0.0),
                Pos2::new(20.0, 10.0)
            ]
        );
    }

    #[test]
    fn a_boxed_in_pin_still_gets_a_wire() {
        // Surround the target with parts: no clean path exists, the router must
        // still return something the painter can draw.
        let obstacles = [
            rect(60.0, 70.0, 20.0, 20.0),
            rect(20.0, 40.0, 20.0, 60.0),
            rect(60.0, 40.0, 20.0, 20.0),
            rect(60.0, 100.0, 20.0, 20.0),
        ];
        let points = route_between(Pos2::new(0.0, 70.0), Pos2::new(100.0, 70.0), &obstacles);
        assert!(points.len() >= 2);
        assert_eq!(points.first(), Some(&Pos2::new(0.0, 70.0)));
        assert_eq!(points.last(), Some(&Pos2::new(100.0, 70.0)));
    }
}
