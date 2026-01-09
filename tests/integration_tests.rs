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
    let reconstructed_indices = decode_path(nodes, &prefixes);

    // 3. Verify
    assert_eq!(
        reconstructed_indices, ground_truth_indices,
        "Viterbi failed to reconstruct path"
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
