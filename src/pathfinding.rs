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
/// This uses Dijkstra's algorithm to explore the graph of repeaters.
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_dijkstra_simple_path() {
        // Create a simple line: Node 0 -> Node 1 -> Node 2
        // Distances approx 10km (0.1 deg lat is ~11km)
        let nodes = vec![
            Repeater {
                id: "000001".to_string(),
                name: "A".to_string(),
                lat: 0.0,
                lon: 0.0,
            },
            Repeater {
                id: "000002".to_string(),
                name: "B".to_string(),
                lat: 0.1,
                lon: 0.0,
            },
            Repeater {
                id: "000003".to_string(),
                name: "C".to_string(),
                lat: 0.2,
                lon: 0.0,
            },
        ];

        let path = find_path(&nodes, 0, 2).expect("Path should exist");
        assert_eq!(path, vec![0, 1, 2]);
    }

    #[test]
    fn test_dijkstra_no_path() {
        // Two nodes far apart
        let nodes = vec![
            Repeater {
                id: "000001".to_string(),
                name: "A".to_string(),
                lat: 0.0,
                lon: 0.0,
            },
            Repeater {
                id: "000002".to_string(),
                name: "B".to_string(),
                lat: 10.0, // > 1000km away
                lon: 0.0,
            },
        ];

        let path = find_path(&nodes, 0, 1);
        assert!(path.is_none());
    }

    #[test]
    fn test_dijkstra_shortest_path() {
        // Diamond shape:
        //      1 (Low Cost)
        //    /   \
        // S(0)    E(3)
        //    \   /
        //      2 (High Cost - Further away)

        // 0.1 deg ~ 11km.
        let nodes = vec![
            Repeater {
                id: "00".into(),
                name: "S".into(),
                lat: 0.0,
                lon: 0.0,
            },
            Repeater {
                id: "01".into(),
                name: "1".into(),
                lat: 0.05,
                lon: 0.05,
            }, // Path 1: ~15km total
            Repeater {
                id: "02".into(),
                name: "2".into(),
                lat: 0.5,
                lon: 0.5,
            }, // Path 2: ~150km total
            Repeater {
                id: "03".into(),
                name: "E".into(),
                lat: 0.1,
                lon: 0.1,
            },
        ];

        // Path via 1: 0->1->3. Distances are small. Cost is low.
        // Path via 2: 0->2->3. Distances are large. Cost is high.

        let path = find_path(&nodes, 0, 3).expect("Path should exist");
        assert_eq!(path, vec![0, 1, 3]);
    }
}
