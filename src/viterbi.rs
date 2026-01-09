use crate::models::Repeater;
use crate::physics::link_cost;
use anyhow::{Result, anyhow};
use serde::{Deserialize, Serialize};

/// Represents a node in the reconstructed path.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum PathNode {
    /// A known repeater from the database (index into the nodes list).
    Known(usize),
    /// An unknown repeater, identified only by its 1-byte prefix.
    Unknown(u8),
}

#[derive(Debug, Clone)]
struct TrellisNode {
    cost: f64,
    prev_node_idx: Option<usize>,
}

const UNKNOWN_LINK_COST: f64 = 8.0;

/// Runs the Viterbi algorithm to reconstruct the path of nodes.
///
/// * `nodes`: The full list of known repeaters.
/// * `observations`: The sequence of 1-byte prefixes (u8) observed in the packet header.
///
/// Returns the most likely sequence of PathNodes.
pub fn decode_path(nodes: &[Repeater], observations: &[u8]) -> Result<Vec<PathNode>> {
    if observations.is_empty() {
        return Ok(Vec::new());
    }

    let t_steps = observations.len();
    // States: 0..nodes.len() are Known nodes.
    // State nodes.len() is the "Unknown" state (wildcard for the current observation).
    let num_states = nodes.len() + 1;
    let unknown_state_idx = nodes.len();

    // Trellis matrix: trellis[time_step][state_idx]
    let mut trellis = vec![
        vec![
            TrellisNode {
                cost: f64::INFINITY,
                prev_node_idx: None
            };
            num_states
        ];
        t_steps
    ];

    // Initialization (Step 0)
    let first_obs = observations[0];
    for (i, node) in nodes.iter().enumerate() {
        if node.prefix() == first_obs {
            trellis[0][i].cost = 0.0;
        }
    }
    // Also initialize Unknown state if needed
    // If we start with an unknown node, we assume cost 0?
    // Or should we prefer known nodes?
    // Let's verify if any known node matched. If not, Unknown is the only choice.
    // Even if one matches, Unknown is a valid state (maybe the known one is geographically impossible later).
    // Let's give Unknown a small penalty at start to prefer Known if available?
    // Or just 0.0. Let's stick to 0.0 for now, relying on later link costs to disambiguate.
    // Actually, if we start at Unknown, we have no location. That's fine.
    trellis[0][unknown_state_idx].cost = 0.0;

    // Forward Pass
    for t in 1..t_steps {
        let obs = observations[t];
        let mut any_reachable = false;

        for curr_state in 0..num_states {
            // Emission Check
            if curr_state < unknown_state_idx {
                // Known node: must match observation
                if nodes[curr_state].prefix() != obs {
                    continue;
                }
            } else {
                // Unknown node: implicitly matches observation
            }

            let mut best_cost = f64::INFINITY;
            let mut best_prev = None;

            for prev_state in 0..num_states {
                let prev_cost = trellis[t - 1][prev_state].cost;
                if prev_cost.is_infinite() {
                    continue;
                }

                let trans_cost = if curr_state < unknown_state_idx && prev_state < unknown_state_idx
                {
                    // Known -> Known
                    let node_prev = &nodes[prev_state];
                    let node_curr = &nodes[curr_state];
                    link_cost(node_prev.lat, node_prev.lon, node_curr.lat, node_curr.lon)
                } else {
                    // Involves Unknown: Fixed penalty
                    UNKNOWN_LINK_COST
                };

                if trans_cost.is_infinite() {
                    continue;
                }

                let total_cost = prev_cost + trans_cost;
                if total_cost < best_cost {
                    best_cost = total_cost;
                    best_prev = Some(prev_state);
                }
            }

            if best_cost.is_finite() {
                trellis[t][curr_state].cost = best_cost;
                trellis[t][curr_state].prev_node_idx = best_prev;
                any_reachable = true;
            }
        }

        if !any_reachable {
            return Err(anyhow!("Viterbi stuck at step {}: no reachable states", t));
        }
    }

    // Termination
    let last_t = t_steps - 1;
    let mut best_final_cost = f64::INFINITY;
    let mut best_final_state = None;

    for i in 0..num_states {
        if trellis[last_t][i].cost < best_final_cost {
            best_final_cost = trellis[last_t][i].cost;
            best_final_state = Some(i);
        }
    }

    // Backtracking
    if let Some(mut curr_idx) = best_final_state {
        let mut path = Vec::new();
        // Decode last state
        if curr_idx < unknown_state_idx {
            path.push(PathNode::Known(curr_idx));
        } else {
            path.push(PathNode::Unknown(observations[last_t]));
        }

        for t in (1..t_steps).rev() {
            if let Some(prev_idx) = trellis[t][curr_idx].prev_node_idx {
                if prev_idx < unknown_state_idx {
                    path.push(PathNode::Known(prev_idx));
                } else {
                    path.push(PathNode::Unknown(observations[t - 1]));
                }
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

#[cfg(test)]
mod tests {
    use super::*;

    // Helper to create a dummy node
    fn create_node(id: &str, lat: f64, lon: f64) -> Repeater {
        Repeater {
            id: id.to_string(),
            name: "TestNode".to_string(),
            lat,
            lon,
        }
    }

    #[test]
    fn test_unknown_repeater_recovery() {
        // Node A: (0.0, 0.0) prefix A0
        // Node B: Missing, prefix B0
        // Node C: (0.5, 0.0) prefix C0 (~55km from A)

        let node_a = create_node("A00000", 0.0, 0.0);
        let node_c = create_node("C00000", 0.5, 0.0);
        let nodes = vec![node_a, node_c];

        // Observations: A0, B0, C0
        let obs = vec![0xA0, 0xB0, 0xC0];

        let result = decode_path(&nodes, &obs).expect("Viterbi failed");

        assert_eq!(result.len(), 3);
        assert_eq!(result[0], PathNode::Known(0)); // Node A
        assert_eq!(result[1], PathNode::Unknown(0xB0)); // Unknown
        assert_eq!(result[2], PathNode::Known(1)); // Node C
    }

    #[test]
    fn test_consecutive_unknowns() {
        // A -> Unknown -> Unknown -> C
        let node_a = create_node("A00000", 0.0, 0.0);
        let node_c = create_node("C00000", 1.0, 0.0); // 111km away
        let nodes = vec![node_a, node_c];

        let obs = vec![0xA0, 0xB0, 0xB1, 0xC0];

        let result = decode_path(&nodes, &obs).expect("Viterbi failed");

        assert_eq!(result.len(), 4);
        assert_eq!(result[0], PathNode::Known(0));
        assert_eq!(result[1], PathNode::Unknown(0xB0));
        assert_eq!(result[2], PathNode::Unknown(0xB1));
        assert_eq!(result[3], PathNode::Known(1));
    }
}
