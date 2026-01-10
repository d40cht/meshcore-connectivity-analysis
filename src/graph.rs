use crate::models::{PathNode, Repeater};
use crate::physics::link_cost;
use crate::terrain::TerrainMap;
use anyhow::{Result, anyhow};
use rstar::{AABB, PointDistance, RTree, RTreeObject};
use std::collections::HashMap;

// Constants
const MAX_LINK_RANGE_KM: f64 = 150.0;

/// Cost for staying in the Unknown state (Unknown -> Unknown).
/// This is the "base" penalty for missing information.
const COST_TRANSITION_UNKNOWN_TO_UNKNOWN: f64 = 8.0;

/// Cost for recovering from the Unknown state to a Known node (Unknown -> Known).
/// Slightly cheaper than staying Unknown, to encourage the path to "snap back"
/// to the known graph as soon as a valid node is observed.
const COST_TRANSITION_UNKNOWN_TO_KNOWN: f64 = 8.0 - 1.0e-6;

/// Cost for entering the Unknown state from a Known node (Known -> Unknown).
/// Slightly cheaper than recovery, to favor "trusting the source" (Known start)
/// in disconnected scenarios.
const COST_TRANSITION_KNOWN_TO_UNKNOWN: f64 = 8.0 - 2.0e-6;

/// Initial cost for Known nodes at the start of the path (Step 0).
/// A negative cost acts as a bonus, ensuring we overwhelmingly prefer starting
/// with a Known node over an Unknown wildcard if both are options.
const COST_START_KNOWN: f64 = -0.1;

/// A wrapper around a repeater index that can be stored in the R-Tree.
#[derive(Debug, Clone, Copy, PartialEq)]
struct SpatialNode {
    index: usize,
    lat: f64,
    lon: f64,
}

impl RTreeObject for SpatialNode {
    type Envelope = AABB<[f64; 2]>;

    fn envelope(&self) -> Self::Envelope {
        AABB::from_point([self.lon, self.lat])
    }
}

impl PointDistance for SpatialNode {
    fn distance_2(&self, point: &[f64; 2]) -> f64 {
        let d_lon = self.lon - point[0];
        let d_lat = self.lat - point[1];
        // Euclidean distance squared in degrees (approximate, for sorting/querying)
        // Physics calculations will use Haversine.
        d_lon * d_lon + d_lat * d_lat
    }
}

pub struct NetworkGraph {
    nodes: Vec<Repeater>,
    /// Adjacency list: nodes[i] -> list of (neighbor_index, cost)
    adjacency: Vec<Vec<(usize, f64)>>,
    /// Lookup: prefix (0-255) -> list of node indices
    nodes_by_prefix: Vec<Vec<usize>>,
}

impl NetworkGraph {
    /// Creates a new NetworkGraph.
    /// * Builds the R-Tree.
    /// * Pre-calculates the sparse adjacency matrix by finding neighbors within MAX_LINK_RANGE_KM.
    pub fn new(nodes: Vec<Repeater>, terrain: Option<&TerrainMap>) -> Self {
        let mut rtree_nodes = Vec::with_capacity(nodes.len());
        let mut nodes_by_prefix = vec![Vec::new(); 256];

        for (i, node) in nodes.iter().enumerate() {
            rtree_nodes.push(SpatialNode {
                index: i,
                lat: node.lat,
                lon: node.lon,
            });
            nodes_by_prefix[node.prefix() as usize].push(i);
        }

        let rtree = RTree::bulk_load(rtree_nodes);

        let mut adjacency: Vec<Vec<(usize, f64)>> = vec![Vec::new(); nodes.len()];

        for (i, node) in nodes.iter().enumerate() {
            let search_radius_deg = (MAX_LINK_RANGE_KM / 111.0) * 1.2;
            let lat_min = node.lat - search_radius_deg;
            let lat_max = node.lat + search_radius_deg;
            let lon_min = node.lon - search_radius_deg;
            let lon_max = node.lon + search_radius_deg;

            let envelope = AABB::from_corners([lon_min, lat_min], [lon_max, lat_max]);

            for neighbor in rtree.locate_in_envelope(&envelope) {
                let j = neighbor.index;
                if i == j {
                    continue;
                }
                let neighbor_node = &nodes[j];
                let cost = link_cost(
                    node.lat,
                    node.lon,
                    neighbor_node.lat,
                    neighbor_node.lon,
                    terrain,
                );
                if cost.is_finite() && cost < 1000.0 {
                    adjacency[i].push((j, cost));
                }
            }
        }

        NetworkGraph {
            nodes,
            adjacency,
            nodes_by_prefix,
        }
    }

    /// Decodes a path using the sparse graph and dynamic trellis expansion.
    /// Uses HashMaps to track only reachable states at each step.
    pub fn decode_path(&self, observations: &[u8]) -> Result<Vec<PathNode>> {
        if observations.is_empty() {
            return Ok(Vec::new());
        }

        let t_steps = observations.len();
        let unknown_state_idx = self.nodes.len(); // Special index for Unknown state

        // Trellis: [step] -> { state_idx -> (cost, prev_state_idx) }
        // prev_state_idx is stored to allow backtracking.
        // We only need the current 'cost' map to compute next step,
        // but we need the FULL history of backpointers to reconstruct the path.
        // So:
        // `current_costs`: HashMap<usize, f64> (Active states and their costs)
        // `backpointers`: Vec<HashMap<usize, usize>> (History of transitions)

        let mut backpointers: Vec<HashMap<usize, usize>> = Vec::with_capacity(t_steps);
        // Step 0 initialization (no backpointers)
        backpointers.push(HashMap::new());

        let mut current_costs: HashMap<usize, f64> = HashMap::new();

        let first_obs = observations[0];

        // Initialize Known nodes matching first prefix
        for &node_idx in &self.nodes_by_prefix[first_obs as usize] {
            current_costs.insert(node_idx, COST_START_KNOWN);
        }

        // Initialize Unknown state
        current_costs.insert(unknown_state_idx, 0.0);

        // Forward Pass
        for t in 1..t_steps {
            let obs = observations[t];
            let mut next_costs: HashMap<usize, f64> = HashMap::new();
            let mut step_backpointers: HashMap<usize, usize> = HashMap::new();

            // Iterate over all active states from previous step
            for (&prev_idx, &prev_cost) in &current_costs {
                // Case 1: Previous state was Known
                if prev_idx < unknown_state_idx {
                    // 1a. Transition: Known -> Known
                    // Use sparse adjacency list to find reachable neighbors
                    if let Some(neighbors) = self.adjacency.get(prev_idx) {
                        for &(neighbor_idx, link_c) in neighbors {
                            // Filter: Neighbor must match observation prefix
                            if self.nodes[neighbor_idx].prefix() == obs {
                                let total_c = prev_cost + link_c;

                                // Update if this path is cheaper
                                let entry = next_costs.entry(neighbor_idx).or_insert(f64::INFINITY);
                                if total_c < *entry {
                                    *entry = total_c;
                                    step_backpointers.insert(neighbor_idx, prev_idx);
                                }
                            }
                        }
                    }

                    // 1b. Transition: Known -> Unknown
                    let unknown_cost = prev_cost + COST_TRANSITION_KNOWN_TO_UNKNOWN;
                    let entry = next_costs.entry(unknown_state_idx).or_insert(f64::INFINITY);
                    if unknown_cost < *entry {
                        *entry = unknown_cost;
                        step_backpointers.insert(unknown_state_idx, prev_idx);
                    }
                } else {
                    // Case 2: Previous state was Unknown

                    // 2a. Transition: Unknown -> Known
                    // Try to snap to ANY Known node matching the prefix
                    for &curr_idx in &self.nodes_by_prefix[obs as usize] {
                        let total_c = prev_cost + COST_TRANSITION_UNKNOWN_TO_KNOWN;
                        let entry = next_costs.entry(curr_idx).or_insert(f64::INFINITY);
                        if total_c < *entry {
                            *entry = total_c;
                            step_backpointers.insert(curr_idx, prev_idx);
                        }
                    }

                    // 2b. Transition: Unknown -> Unknown
                    let unknown_unknown_cost = prev_cost + COST_TRANSITION_UNKNOWN_TO_UNKNOWN;
                    let entry = next_costs.entry(unknown_state_idx).or_insert(f64::INFINITY);
                    if unknown_unknown_cost < *entry {
                        *entry = unknown_unknown_cost;
                        step_backpointers.insert(unknown_state_idx, prev_idx);
                    }
                }
            }

            if next_costs.is_empty() {
                return Err(anyhow!("Viterbi stuck at step {}: no reachable states", t));
            }

            current_costs = next_costs;
            backpointers.push(step_backpointers);
        }

        // Termination: Find best final state
        let mut best_final_cost = f64::INFINITY;
        let mut best_final_state = None;

        // To ensure deterministic tie-breaking (e.g. favoring Known over Unknown, or lower indices),
        // we can sort the keys. But simpler: iterate and use strict < or <= logic.
        // HashMap iteration order is random. We MUST iterate in a deterministic order for consistent results.
        // Let's collect keys and sort them.

        let mut final_states: Vec<usize> = current_costs.keys().cloned().collect();
        final_states.sort_unstable(); // Deterministic order: 0, 1, 2, ..., Unknown(MAX)

        for &state_idx in &final_states {
            let cost = current_costs[&state_idx];
            if cost < best_final_cost {
                best_final_cost = cost;
                best_final_state = Some(state_idx);
            }
        }

        // Backtracking
        if let Some(mut curr_idx) = best_final_state {
            let mut path = Vec::new();
            let last_t = t_steps - 1;

            let to_path_node = |idx: usize, step_idx: usize| -> PathNode {
                if idx < unknown_state_idx {
                    PathNode::Known(idx)
                } else {
                    PathNode::Unknown(observations[step_idx])
                }
            };

            path.push(to_path_node(curr_idx, last_t));

            for t in (1..t_steps).rev() {
                if let Some(&prev_idx) = backpointers[t].get(&curr_idx) {
                    path.push(to_path_node(prev_idx, t - 1));
                    curr_idx = prev_idx;
                } else {
                    return Err(anyhow!("Broken path during backtracking at step {}", t));
                }
            }
            path.reverse();
            Ok(path)
        } else {
            Err(anyhow!("No valid path found (final state unreachable)"))
        }
    }
}
