use crate::models::Repeater;
use crate::physics::link_cost;
use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap};

#[derive(Debug, Clone, PartialEq)]
struct State {
    cost: f64,
    node_idx: usize,
}

impl Eq for State {}

impl PartialOrd for State {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        // Reverse because BinaryHeap is a max-heap, we want min-cost
        other.cost.partial_cmp(&self.cost)
    }
}

impl Ord for State {
    fn cmp(&self, other: &Self) -> Ordering {
        self.partial_cmp(other).unwrap_or(Ordering::Equal)
    }
}

/// Finds the lowest cost path between start_node and end_node indices.
/// Returns a vector of node indices representing the path.
pub fn find_path(nodes: &[Repeater], start_idx: usize, end_idx: usize) -> Option<Vec<usize>> {
    let mut dist: HashMap<usize, f64> = HashMap::new();
    let mut prev: HashMap<usize, usize> = HashMap::new();
    let mut heap = BinaryHeap::new();

    dist.insert(start_idx, 0.0);
    heap.push(State {
        cost: 0.0,
        node_idx: start_idx,
    });

    while let Some(State { cost, node_idx }) = heap.pop() {
        if node_idx == end_idx {
            // Reconstruct path
            let mut path = Vec::new();
            let mut current = end_idx;
            path.push(current);
            while let Some(&p) = prev.get(&current) {
                current = p;
                path.push(current);
            }
            path.reverse();
            return Some(path);
        }

        if cost > *dist.get(&node_idx).unwrap_or(&f64::INFINITY) {
            continue;
        }

        // Explore neighbors
        for (i, neighbor) in nodes.iter().enumerate() {
            if i == node_idx {
                continue;
            }

            let current_node = &nodes[node_idx];
            let edge_cost = link_cost(
                current_node.lat,
                current_node.lon,
                neighbor.lat,
                neighbor.lon,
            );

            if edge_cost.is_infinite() || edge_cost > 500.0 {
                continue; // Unreachable
            }

            let next_cost = cost + edge_cost;

            if next_cost < *dist.get(&i).unwrap_or(&f64::INFINITY) {
                dist.insert(i, next_cost);
                prev.insert(i, node_idx);
                heap.push(State {
                    cost: next_cost,
                    node_idx: i,
                });
            }
        }
    }

    None
}
