use crate::models::{PathNode, Repeater};
use crate::physics::link_cost;
use crate::terrain::TerrainMap;
use anyhow::{Result, anyhow};
use rstar::{AABB, PointDistance, RTree, RTreeObject};

// Constants
const MAX_LINK_RANGE_KM: f64 = 150.0;
const UNKNOWN_LINK_COST: f64 = 8.0;

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

        let mut adjacency = vec![Vec::new(); nodes.len()];

        // Pre-calculate edges
        for (i, node) in nodes.iter().enumerate() {
            // 1 degree lat is ~111km.
            // Be conservative: search a bit wider than the exact KM limit.
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

                // Add edge if cost is feasible (not infinite)
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

    /// Decodes a path using the sparse graph.
    pub fn decode_path(&self, observations: &[u8]) -> Result<Vec<PathNode>> {
        if observations.is_empty() {
            return Ok(Vec::new());
        }

        let t_steps = observations.len();
        // State representation:
        // We cannot use a simple matrix [t][state] because 'state' maps to node index,
        // and we have ~10k nodes.
        // However, standard Viterbi usually does exactly that. 10k * 10 steps = 100k floats.
        // That is actually very small (400KB).
        // So a dense matrix is fine and faster than hashmaps.

        let num_nodes = self.nodes.len();
        let unknown_state_idx = num_nodes;
        let total_states = num_nodes + 1;

        // Current costs: maps state_idx -> cost
        // We only store the *current* and *previous* step costs to save memory,
        // but we need backpointers for the whole history.

        // Backpointers: [step][current_state_idx] -> prev_state_idx
        // Using Option<usize> to indicate reachability.
        // Since we have a sparse graph, many states will be unreachable (Cost = Infinity).
        // Storing a dense backpointer matrix is fine (10k * len * 8 bytes).
        // For 100 hops, that's 8MB. Acceptable.

        let mut backpointers = vec![vec![None; total_states]; t_steps];

        // Initialize Step 0
        let mut prev_costs = vec![f64::INFINITY; total_states];

        let first_obs = observations[0];

        // Initialize Known nodes matching the first prefix
        // Give a small bonus (-0.1) to prefer starting with a Known node over Unknown.
        for &node_idx in &self.nodes_by_prefix[first_obs as usize] {
            prev_costs[node_idx] = -0.1;
        }

        // Initialize Unknown state
        prev_costs[unknown_state_idx] = 0.0;

        // Forward Pass
        for t in 1..t_steps {
            let obs = observations[t];
            let mut curr_costs = vec![f64::INFINITY; total_states];
            let mut any_reachable = false;

            // To optimize: Iterate only over states that were reachable in prev_costs
            // But iterating 10k floats is extremely fast in CPU cache.
            // The bottleneck is the inner loop (checking neighbors).
            // Let's iterate all potential PREVIOUS states that have finite cost.

            // Optimization: First, identify active previous states to avoid looping all 10k if only 5 are active.
            // But 'active' list management has overhead.
            // Let's try simple loop first, optimizing inner logic.

            for prev_idx in 0..total_states {
                let prev_cost = prev_costs[prev_idx];
                if prev_cost.is_infinite() {
                    continue;
                }

                // 1. Transition: Known(prev) -> Known(curr)
                if prev_idx < num_nodes {
                    // Iterate ONLY neighbors from the sparse graph
                    for &(neighbor_idx, link_c) in &self.adjacency[prev_idx] {
                        // Check emission (does neighbor match obs?)
                        if self.nodes[neighbor_idx].prefix() == obs {
                            let total_c = prev_cost + link_c;
                            if total_c < curr_costs[neighbor_idx] {
                                curr_costs[neighbor_idx] = total_c;
                                backpointers[t][neighbor_idx] = Some(prev_idx);
                                any_reachable = true;
                            }
                        }
                    }

                    // 2. Transition: Known(prev) -> Unknown
                    // Cost is UNKNOWN_LINK_COST - 2*epsilon.
                    // Prefer entering unknown over anything else involving unknowns.
                    let unknown_cost = prev_cost + UNKNOWN_LINK_COST - 2.0e-6;
                    if unknown_cost < curr_costs[unknown_state_idx] {
                        curr_costs[unknown_state_idx] = unknown_cost;
                        backpointers[t][unknown_state_idx] = Some(prev_idx);
                        any_reachable = true;
                    }
                } else {
                    // Previous was Unknown
                    // 3. Transition: Unknown -> Known(curr)
                    // We allow transition to ANY node matching the observation.
                    // Cost is UNKNOWN_LINK_COST - epsilon.
                    // Prefer exiting unknown over staying in unknown.
                    for &curr_idx in &self.nodes_by_prefix[obs as usize] {
                        let total_c = prev_cost + UNKNOWN_LINK_COST - 1.0e-6;
                        if total_c < curr_costs[curr_idx] {
                            curr_costs[curr_idx] = total_c;
                            backpointers[t][curr_idx] = Some(prev_idx);
                            any_reachable = true;
                        }
                    }

                    // 4. Transition: Unknown -> Unknown
                    let unknown_unknown_cost = prev_cost + UNKNOWN_LINK_COST;
                    if unknown_unknown_cost < curr_costs[unknown_state_idx] {
                        curr_costs[unknown_state_idx] = unknown_unknown_cost;
                        backpointers[t][unknown_state_idx] = Some(prev_idx);
                        any_reachable = true;
                    }
                }
            }

            if !any_reachable {
                return Err(anyhow!("Viterbi stuck at step {}: no reachable states", t));
            }

            prev_costs = curr_costs;
        }

        // Backtracking
        let last_t = t_steps - 1;
        let mut best_final_cost = f64::INFINITY;
        let mut best_final_state = None;

        for i in 0..total_states {
            if prev_costs[i] < best_final_cost {
                best_final_cost = prev_costs[i];
                best_final_state = Some(i);
            }
        }

        if let Some(mut curr_idx) = best_final_state {
            let mut path = Vec::new();

            // Helper to convert state idx to PathNode
            let to_path_node = |idx: usize, step_idx: usize| -> PathNode {
                if idx < num_nodes {
                    PathNode::Known(idx)
                } else {
                    PathNode::Unknown(observations[step_idx])
                }
            };

            path.push(to_path_node(curr_idx, last_t));

            for t in (1..t_steps).rev() {
                if let Some(prev_idx) = backpointers[t][curr_idx] {
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
