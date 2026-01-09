use app::models::Repeater;
use app::pathfinding::find_path;
use app::physics;
use app::test_utils::generate_dummy_nodes;
use app::viterbi::{PathNode, decode_path};

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
    let reconstructed_path = decode_path(nodes, &prefixes).expect("Viterbi failed to decode path");

    // 3. Verify
    // Convert reconstructed path (Vec<PathNode>) to indices (Vec<usize>) for comparison
    // If any node is Unknown, this specific verification fails (as it expects full knowledge)
    let reconstructed_indices: Vec<usize> = reconstructed_path
        .iter()
        .map(|node| match node {
            PathNode::Known(idx) => *idx,
            PathNode::Unknown(prefix) => panic!(
                "Unexpected Unknown node with prefix {:02x} in fully known scenario",
                prefix
            ),
        })
        .collect();

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
    // Manually construct a "bushy" topology.
    // Start (S) and End (E) are separated by distance.
    // A grid of intermediate nodes exists, providing a low-cost but multi-hop path.
    // A "Shortcut" node exists, providing a fewer-hop but higher-cost path (due to physics/distance).
    // We expect the pathfinder to choose the grid path (length >= 5) over the shortcut (length 3).

    let center_lat = 51.0;
    let center_lon = 0.0;

    let mut nodes = Vec::new();

    // 0. Start Node
    nodes.push(Repeater {
        id: "000000".to_string(),
        name: "Start".to_string(),
        lat: center_lat,
        lon: center_lon,
    });

    // Grid Layers (Indices 1..9)
    // 3 Layers at lon +0.2, +0.4, +0.6
    // 3 Rows at lat -0.1, 0.0, +0.1
    for i in 1..=3 {
        let lon_offset = i as f64 * 0.2;
        for j in -1..=1 {
            let lat_offset = j as f64 * 0.1;
            // Generate valid Hex IDs with unique prefixes to ensure Viterbi decoding works.
            // Format: "{Layer}{Row}0000" -> e.g., "100000", "110000", "120000"
            // i is 1,2,3. j is -1,0,1 -> map j to 0,1,2 for Hex digit.
            let row_digit = j + 1;
            let id = format!("{}{:x}0000", i, row_digit);

            nodes.push(Repeater {
                id,
                name: format!("Grid_{}_{}", i, j),
                lat: center_lat + lat_offset,
                lon: center_lon + lon_offset,
            });
        }
    }

    // 10. Shortcut Node (High lat offset, centered lon)
    // Positioned at (51.3, 0.4).
    // Dist S->Shortcut: sqrt(0.3^2 + 0.4^2) ~ 50km
    // Dist Shortcut->E: similar
    // This creates a 3-node path: S -> Shortcut -> E
    nodes.push(Repeater {
        id: "990000".to_string(),
        name: "Shortcut".to_string(),
        lat: center_lat + 0.3,
        lon: center_lon + 0.4,
    });

    // 11. End Node (at 0.8 lon)
    nodes.push(Repeater {
        id: "EE0000".to_string(),
        name: "End".to_string(),
        lat: center_lat,
        lon: center_lon + 0.8,
    });
    let end_idx = nodes.len() - 1;

    let start_idx = 0;

    if let Some(path) = find_path(&nodes, start_idx, end_idx) {
        println!("Found path: {:?}", path);

        // Define the Expected Path IDs (Straight line through the center)
        // Start -> Grid_1_0 (110000) -> Grid_2_0 (210000) -> Grid_3_0 (310000) -> End
        let expected_ids = vec!["000000", "110000", "210000", "310000", "EE0000"];

        let expected_indices: Vec<usize> = expected_ids
            .iter()
            .map(|id| find_node_idx(&nodes, id).unwrap_or_else(|| panic!("Node {} not found", id)))
            .collect();

        assert_eq!(
            path, expected_indices,
            "Path found {:?} does not match expected specific path {:?}",
            path, expected_indices
        );

        // Verify Viterbi reconstruction
        verify_path_reconstruction(&nodes, &path);
    } else {
        panic!("No path found for complex multipath test");
    }
}
