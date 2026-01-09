use app::models::Repeater;
use app::pathfinding::find_path;
use app::physics;
use app::test_utils::generate_dummy_nodes;
use app::viterbi::decode_path;

fn find_node_idx(nodes: &[Repeater], id: &str) -> Option<usize> {
    nodes.iter().position(|n| n.id == id)
}

fn find_closest(nodes: &[Repeater], target_idx: usize) -> usize {
    let t = &nodes[target_idx];
    let mut best_dist = f64::INFINITY;
    let mut best_idx = 0;

    for (i, n) in nodes.iter().enumerate() {
        if i == target_idx {
            continue;
        }
        let d = physics::haversine_distance(t.lat, t.lon, n.lat, n.lon);
        if d < best_dist {
            best_dist = d;
            best_idx = i;
        }
    }
    best_idx
}

fn verify_path_reconstruction(nodes: &[Repeater], ground_truth_indices: &[usize]) {
    // 1. Convert to Prefixes
    let prefixes: Vec<u8> = ground_truth_indices
        .iter()
        .map(|&idx| nodes[idx].prefix())
        .collect();

    // 2. Run Viterbi
    let reconstructed_indices =
        decode_path(nodes, &prefixes).expect("Viterbi failed to decode path");

    // 3. Verify
    assert_eq!(
        reconstructed_indices, ground_truth_indices,
        "Viterbi reconstructed path does not match ground truth"
    );
}

#[test]
fn test_general_connectivity() {
    let nodes = generate_dummy_nodes();
    // Test 1: General Connectivity
    let start_node = 0;
    let end_node = 15;

    if let Some(path) = find_path(&nodes, start_node, end_node) {
        verify_path_reconstruction(&nodes, &path);
    } else {
        panic!("No path found for general connectivity test");
    }
}

#[test]
fn test_local_clash_resolution() {
    let nodes = generate_dummy_nodes();
    let clash_local_a_idx = find_node_idx(&nodes, "AA1111").expect("Node AA1111 not found");
    let clash_local_b_idx = find_node_idx(&nodes, "AA2222").expect("Node AA2222 not found");

    // Path to AA1111
    let neighbor_idx = find_closest(&nodes, clash_local_a_idx);
    if let Some(path) = find_path(&nodes, neighbor_idx, clash_local_a_idx) {
        verify_path_reconstruction(&nodes, &path);
    } else {
        panic!("No path found to local clash A");
    }

    // Path to AA2222
    let neighbor_b_idx = find_closest(&nodes, clash_local_b_idx);
    if let Some(path) = find_path(&nodes, neighbor_b_idx, clash_local_b_idx) {
        verify_path_reconstruction(&nodes, &path);
    } else {
        panic!("No path found to local clash B");
    }
}

#[test]
fn test_global_clash_resolution() {
    let nodes = generate_dummy_nodes();
    let clash_global_c_idx = find_node_idx(&nodes, "BB1111").expect("Node BB1111 not found");
    let clash_global_d_idx = find_node_idx(&nodes, "BB2222").expect("Node BB2222 not found");

    // Path involving BB1111 (Center)
    let neighbor_c_idx = find_closest(&nodes, clash_global_c_idx);
    if let Some(path) = find_path(&nodes, neighbor_c_idx, clash_global_c_idx) {
        verify_path_reconstruction(&nodes, &path);
    } else {
        panic!("No path found to global clash C");
    }

    // Path involving BB2222 (North)
    let neighbor_d_idx = find_closest(&nodes, clash_global_d_idx);
    if let Some(path) = find_path(&nodes, neighbor_d_idx, clash_global_d_idx) {
        verify_path_reconstruction(&nodes, &path);
    } else {
        panic!("No path found to global clash D");
    }
}

#[test]
fn test_complex_multipath() {
    // Manually construct a specific topology to ensure a path of at least 4 nodes.
    // We create a chain: S -> A -> B -> C -> E
    // Step size 0.4 degrees longitude at 51.0 latitude is approx 28km.
    // 28km has low Earth Bulge (~16m vs 40m threshold) -> Low cost.
    // 56km (skipping a node) has high Earth Bulge (~60m) -> High cost.
    // This forces the pathfinder to visit every node in the chain.

    let center_lat = 51.0;
    let center_lon = 0.0;

    let mut nodes = Vec::new();

    // Start Node (Index 0)
    nodes.push(Repeater {
        id: "S00000".to_string(),
        name: "Start".to_string(),
        lat: center_lat,
        lon: center_lon,
    });

    // Node A (Index 1)
    nodes.push(Repeater {
        id: "A00000".to_string(),
        name: "Node_A".to_string(),
        lat: center_lat,
        lon: center_lon + 0.4,
    });

    // Node B (Index 2)
    nodes.push(Repeater {
        id: "B00000".to_string(),
        name: "Node_B".to_string(),
        lat: center_lat,
        lon: center_lon + 0.8,
    });

    // Node C (Index 3)
    nodes.push(Repeater {
        id: "C00000".to_string(),
        name: "Node_C".to_string(),
        lat: center_lat,
        lon: center_lon + 1.2,
    });

    // End Node (Index 4)
    nodes.push(Repeater {
        id: "E00000".to_string(),
        name: "End".to_string(),
        lat: center_lat,
        lon: center_lon + 1.6,
    });

    let start_idx = 0;
    let end_idx = 4;

    if let Some(path) = find_path(&nodes, start_idx, end_idx) {
        // Expected path: [0, 1, 2, 3, 4]
        assert!(
            path.len() >= 4,
            "Path should be at least 4 nodes long, found length {}",
            path.len()
        );
        assert_eq!(
            path,
            vec![0, 1, 2, 3, 4],
            "Path did not follow the expected chain"
        );

        verify_path_reconstruction(&nodes, &path);
    } else {
        panic!("No path found for complex multipath test");
    }
}
