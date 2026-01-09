use crate::models::Repeater;
use crate::physics::link_cost;

#[derive(Debug, Clone)]
struct TrellisNode {
    cost: f64,
    prev_node_idx: Option<usize>,
}

/// Runs the Viterbi algorithm to reconstruct the path of nodes.
///
/// * `nodes`: The full list of known repeaters.
/// * `observations`: The sequence of 1-byte prefixes (u8) observed in the packet header.
///
/// Returns the most likely sequence of node indices.
pub fn decode_path(nodes: &[Repeater], observations: &[u8]) -> Vec<usize> {
    if observations.is_empty() {
        return Vec::new();
    }

    let t_steps = observations.len();
    let num_states = nodes.len();

    // trellis[t][state_idx]
    // We only need to store the current column and the previous column's backpointers?
    // Actually, to reconstruct, we need backpointers for all steps.
    // Let's store a matrix of costs and backpointers.
    // Initialize with Infinity
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
        // Emission Probability:
        // P(obs | state) = 1 if match, 0 if no match.
        // Cost = -ln(P).
        // If match: cost 0. If no match: cost INFINITY.
        if node.prefix() == first_obs {
            // For the start, we assume uniform prior or just 0 cost if it matches
            trellis[0][i].cost = 0.0;
        }
    }

    // Recursion
    for t in 1..t_steps {
        let obs = observations[t];
        let mut any_reachable = false;

        for curr_state in 0..num_states {
            // Emission check first (optimization)
            if nodes[curr_state].prefix() != obs {
                continue;
            }

            let mut best_cost = f64::INFINITY;
            let mut best_prev = None;

            for prev_state in 0..num_states {
                let prev_cost = trellis[t - 1][prev_state].cost;
                if prev_cost.is_infinite() {
                    continue;
                }

                // Transition Cost
                let node_prev = &nodes[prev_state];
                let node_curr = &nodes[curr_state];
                let trans_cost =
                    link_cost(node_prev.lat, node_prev.lon, node_curr.lat, node_curr.lon);

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
            // If we get stuck (no path explains this observation), we might have a gap.
            // For this specific task, we assume the path is valid.
            // But good to log or handle?
            // println!("Warning: Viterbi stuck at step {}", t);
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
    let mut path = Vec::new();
    if let Some(mut curr_idx) = best_final_state {
        path.push(curr_idx);
        for t in (1..t_steps).rev() {
            if let Some(prev_idx) = trellis[t][curr_idx].prev_node_idx {
                path.push(prev_idx);
                curr_idx = prev_idx;
            } else {
                // Should not happen if path is valid
                break;
            }
        }
        path.reverse();
    }

    path
}
