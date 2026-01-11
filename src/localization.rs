use crate::models::{PathNode, Repeater};
use crate::physics::haversine_distance;
use serde::Serialize;
use std::collections::HashMap;

const CLUSTER_THRESHOLD_KM: f64 = 50.0;

#[derive(Debug, Serialize, Clone, PartialEq)]
pub struct InferredRepeater {
    pub prefix: String,
    pub lat: f64,
    pub lon: f64,
    pub observation_count: usize,
}

#[derive(Debug)]
struct Observation {
    lat: f64,
    lon: f64,
}

/// Identifies unknown repeaters by finding K->U->K triplets in paths,
/// calculating midpoints, and clustering them.
pub fn localize_unknowns(
    paths: &[Vec<PathNode>],
    known_nodes: &[Repeater],
) -> Vec<InferredRepeater> {
    let mut observations_by_prefix: HashMap<u8, Vec<Observation>> = HashMap::new();

    // 1. Extract Midpoints from K->U->K triplets
    for path in paths {
        if path.len() < 3 {
            continue;
        }

        for window in path.windows(3) {
            if let [PathNode::Known(k1_idx), PathNode::Unknown(u_prefix), PathNode::Known(k2_idx)] =
                window
            {
                let k1 = &known_nodes[*k1_idx];
                let k2 = &known_nodes[*k2_idx];

                // Simple midpoint calculation (flat earth approximation is sufficient for local midpoints)
                // or just average lat/lon.
                let mid_lat = (k1.lat + k2.lat) / 2.0;
                let mid_lon = (k1.lon + k2.lon) / 2.0;

                observations_by_prefix
                    .entry(*u_prefix)
                    .or_default()
                    .push(Observation {
                        lat: mid_lat,
                        lon: mid_lon,
                    });
            }
        }
    }

    let mut results = Vec::new();

    // 2. Cluster observations for each prefix
    for (prefix, obs_list) in observations_by_prefix {
        let clusters = cluster_points(&obs_list, CLUSTER_THRESHOLD_KM);

        for cluster in clusters {
            if cluster.is_empty() {
                continue;
            }

            // Calculate centroid
            let count = cluster.len();
            let sum_lat: f64 = cluster.iter().map(|p| p.lat).sum();
            let sum_lon: f64 = cluster.iter().map(|p| p.lon).sum();

            results.push(InferredRepeater {
                prefix: format!("{:02x}", prefix),
                lat: sum_lat / count as f64,
                lon: sum_lon / count as f64,
                observation_count: count,
            });
        }
    }

    // Sort for deterministic output
    results.sort_by(|a, b| {
        a.prefix.cmp(&b.prefix)
            .then(a.lat.partial_cmp(&b.lat).unwrap_or(std::cmp::Ordering::Equal))
    });

    results
}

/// Simple connected-components clustering based on distance.
// TODO: Upgrade to DBSCAN or similar density-based clustering for better outlier rejection.
fn cluster_points(points: &[Observation], threshold_km: f64) -> Vec<Vec<&Observation>> {
    let mut visited = vec![false; points.len()];
    let mut clusters = Vec::new();

    for i in 0..points.len() {
        if visited[i] {
            continue;
        }

        let mut current_cluster = Vec::new();
        let mut stack = vec![i];
        visited[i] = true;

        while let Some(curr_idx) = stack.pop() {
            let p_curr = &points[curr_idx];
            current_cluster.push(p_curr);

            for j in 0..points.len() {
                if !visited[j] {
                    let p_next = &points[j];
                    let dist = haversine_distance(p_curr.lat, p_curr.lon, p_next.lat, p_next.lon);
                    if dist <= threshold_km {
                        visited[j] = true;
                        stack.push(j);
                    }
                }
            }
        }
        clusters.push(current_cluster);
    }

    clusters
}
