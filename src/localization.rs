use crate::models::{PathNode, Repeater};
use crate::physics::haversine_distance;
use serde::Serialize;
use std::collections::HashMap;

const DBSCAN_EPSILON_KM: f64 = 50.0;
const DBSCAN_MIN_POINTS: usize = 1;

/// Represents an inferred unknown repeater location.
///
/// **Note:** The `lat` and `lon` fields are currently calculated as the centroid
/// of all `LinkMidpoint`s in the cluster. This is a first-order approximation
/// and may not be highly accurate, especially for geometries where the
/// repeater is not near the path midpoint.
#[derive(Debug, Serialize, Clone, PartialEq)]
pub struct InferredRepeater {
    pub prefix: String,
    pub lat: f64,
    pub lon: f64,
    pub observation_count: usize,
}

/// Represents the geometric midpoint between two Known nodes in a
/// `Known -> Unknown -> Known` path sequence.
///
/// This serves as a rough proxy for the location of the Unknown node.
#[derive(Debug, Clone)]
struct LinkMidpoint {
    lat: f64,
    lon: f64,
}

/// Identifies unknown repeaters by finding K->U->K triplets in paths,
/// calculating midpoints, and clustering them.
pub fn localize_unknowns(
    paths: &[Vec<PathNode>],
    known_nodes: &[Repeater],
) -> Vec<InferredRepeater> {
    let mut observations_by_prefix: HashMap<u8, Vec<LinkMidpoint>> = HashMap::new();

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
                    .push(LinkMidpoint {
                        lat: mid_lat,
                        lon: mid_lon,
                    });
            }
        }
    }

    let mut results = Vec::new();

    // 2. Cluster observations for each prefix
    for (prefix, obs_list) in observations_by_prefix {
        let clusters = dbscan(&obs_list, DBSCAN_EPSILON_KM, DBSCAN_MIN_POINTS);

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

#[derive(Clone, Copy, PartialEq)]
enum PointStatus {
    Unvisited,
    Visited,
    Noise, // Not used if min_points=1, but good for completeness
}

/// DBSCAN Clustering Implementation
fn dbscan(points: &[LinkMidpoint], epsilon: f64, min_points: usize) -> Vec<Vec<&LinkMidpoint>> {
    let mut status = vec![PointStatus::Unvisited; points.len()];
    let mut clusters = Vec::new();

    for i in 0..points.len() {
        if status[i] != PointStatus::Unvisited {
            continue;
        }

        status[i] = PointStatus::Visited;
        let neighbors = region_query(points, i, epsilon);

        if neighbors.len() < min_points {
            status[i] = PointStatus::Noise;
        } else {
            let mut current_cluster = Vec::new();
            expand_cluster(
                points,
                &mut status,
                &mut current_cluster,
                i,
                neighbors,
                epsilon,
                min_points,
            );
            clusters.push(current_cluster);
        }
    }

    clusters
}

fn expand_cluster<'a>(
    points: &'a [LinkMidpoint],
    status: &mut [PointStatus],
    cluster: &mut Vec<&'a LinkMidpoint>,
    seed_idx: usize,
    mut seeds: Vec<usize>,
    epsilon: f64,
    min_points: usize,
) {
    cluster.push(&points[seed_idx]);

    // Note: In a standard DBSCAN, we iterate through seeds.
    // Since we are modifying seeds (pushing to it), we use a while loop/index approach.
    let mut i = 0;
    while i < seeds.len() {
        let curr_idx = seeds[i];
        i += 1;

        if curr_idx == seed_idx {
            continue; // Already added seed
        }

        match status[curr_idx] {
            PointStatus::Noise => {
                // Change noise to border point
                status[curr_idx] = PointStatus::Visited;
                cluster.push(&points[curr_idx]);
            }
            PointStatus::Unvisited => {
                status[curr_idx] = PointStatus::Visited;
                cluster.push(&points[curr_idx]);
                let neighbors = region_query(points, curr_idx, epsilon);
                if neighbors.len() >= min_points {
                    // Extend the cluster
                    for n in neighbors {
                        if !seeds.contains(&n) { // Avoid dupes in processing queue
                             seeds.push(n);
                        }
                    }
                }
            }
            PointStatus::Visited => {
                // Already processed, do nothing (assumed already in a cluster or noise)
                // However, standard DBSCAN might add it if it was noise.
                // Our implementation handles noise above.
            }
        }
    }
}

fn region_query(points: &[LinkMidpoint], center_idx: usize, epsilon: f64) -> Vec<usize> {
    let p_center = &points[center_idx];
    let mut neighbors = Vec::new();
    for (i, p) in points.iter().enumerate() {
        // Distance to self is 0, so it's included
        let dist = haversine_distance(p_center.lat, p_center.lon, p.lat, p.lon);
        if dist <= epsilon {
            neighbors.push(i);
        }
    }
    neighbors
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_dbscan_clustering_basic() {
        // Cluster 1: (0,0), (0, 0.1)
        // Cluster 2: (10, 10)
        let points = vec![
            LinkMidpoint { lat: 0.0, lon: 0.0 },
            LinkMidpoint { lat: 0.0, lon: 0.1 }, // ~11km away
            LinkMidpoint { lat: 10.0, lon: 10.0 }, // far away
        ];

        let epsilon = 20.0;
        let min_points = 1;

        let clusters = dbscan(&points, epsilon, min_points);
        assert_eq!(clusters.len(), 2);
    }

    #[test]
    fn test_dbscan_noise_filtering() {
        // With min_points = 2, isolated points should be noise
        // P1, P2 are close. P3 is isolated.
        let points = vec![
            LinkMidpoint { lat: 0.0, lon: 0.0 },
            LinkMidpoint { lat: 0.0, lon: 0.0001 }, // very close
            LinkMidpoint { lat: 10.0, lon: 10.0 }, // far away
        ];

        let epsilon = 1.0;
        let min_points = 2;

        let clusters = dbscan(&points, epsilon, min_points);

        // Should find 1 cluster (P1, P2)
        // P3 should be ignored (noise)
        assert_eq!(clusters.len(), 1);
        assert_eq!(clusters[0].len(), 2);
    }
}
