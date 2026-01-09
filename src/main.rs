use app::models::Repeater;
use app::pathfinding::find_path;
use app::viterbi::decode_path;
use std::error::Error;
use std::fs::File;
use std::path::Path;

fn main() -> Result<(), Box<dyn Error>> {
    println!("Loading repeaters...");
    let nodes = load_repeaters("repeaters.csv")?;
    println!("Loaded {} nodes.", nodes.len());

    // Find indices of special nodes
    let clash_local_a_idx = find_node_idx(&nodes, "AA1111").expect("Node AA1111 not found");
    let clash_local_b_idx = find_node_idx(&nodes, "AA2222").expect("Node AA2222 not found");
    let clash_global_c_idx = find_node_idx(&nodes, "BB1111").expect("Node BB1111 not found");
    let clash_global_d_idx = find_node_idx(&nodes, "BB2222").expect("Node BB2222 not found");

    // Case 1: Path traversing through Local Clash Area
    // We want a path that naturally wants to go through A or B.
    // Let's pick a node South of A and a node North of A.
    // We will simulate a path between random nodes but filter for ones that include our targets
    // Actually, let's just force a path between our clash nodes and neighbors.
    // Or better, let's pick two distant nodes and see if the path naturally crosses the center.

    // Let's explicitly construct a test case.
    // We'll calculate a path between Node_0 (near center) and some other node.
    let start_node = 0;
    // Pick a node far away?
    let end_node = 15; // Random guess

    println!("\n--- Test 1: General Connectivity ---");
    run_test(&nodes, start_node, end_node);

    println!("\n--- Test 2: Local Clash Resolution (0xAA) ---");
    // Path: neighbor -> AA1111 -> neighbor
    // Find a neighbor close to AA1111
    let neighbor_idx = find_closest(&nodes, clash_local_a_idx);
    // Force path: neighbor -> AA1111 -> neighbor (loop?) No.
    // Let's ask Dijkstra for a path.
    if let Some(path) = find_path(&nodes, neighbor_idx, clash_local_a_idx) {
        println!("Ground truth path to Local Clash A found.");
        verify_path_reconstruction(&nodes, &path);
    }

    // Now try to go to AA2222
    let neighbor_b_idx = find_closest(&nodes, clash_local_b_idx);
    if let Some(path) = find_path(&nodes, neighbor_b_idx, clash_local_b_idx) {
        println!("Ground truth path to Local Clash B found.");
        verify_path_reconstruction(&nodes, &path);
    }

    println!("\n--- Test 3: Global Clash Resolution (0xBB) ---");
    // BB1111 is near center. BB2222 is far north.
    // Let's do a path involving BB1111
    let neighbor_c_idx = find_closest(&nodes, clash_global_c_idx);
    if let Some(path) = find_path(&nodes, neighbor_c_idx, clash_global_c_idx) {
        println!("Ground truth path to Global Clash C (Center) found.");
        verify_path_reconstruction(&nodes, &path);
    }

    // Path involving BB2222
    let neighbor_d_idx = find_closest(&nodes, clash_global_d_idx);
    if let Some(path) = find_path(&nodes, neighbor_d_idx, clash_global_d_idx) {
        println!("Ground truth path to Global Clash D (Far North) found.");
        verify_path_reconstruction(&nodes, &path);
    }

    Ok(())
}

fn load_repeaters<P: AsRef<Path>>(path: P) -> Result<Vec<Repeater>, Box<dyn Error>> {
    let file = File::open(path)?;
    let mut rdr = csv::Reader::from_reader(file);
    let mut nodes = Vec::new();
    for result in rdr.deserialize() {
        let record: Repeater = result?;
        nodes.push(record);
    }
    Ok(nodes)
}

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
        let d = app::physics::haversine_distance(t.lat, t.lon, n.lat, n.lon);
        if d < best_dist {
            best_dist = d;
            best_idx = i;
        }
    }
    best_idx
}

fn run_test(nodes: &[Repeater], start: usize, end: usize) {
    match find_path(nodes, start, end) {
        Some(path) => {
            verify_path_reconstruction(nodes, &path);
        }
        None => println!("No path found between {} and {}", start, end),
    }
}

fn verify_path_reconstruction(nodes: &[Repeater], ground_truth_indices: &[usize]) {
    // 1. Convert to Prefixes
    let prefixes: Vec<u8> = ground_truth_indices
        .iter()
        .map(|&idx| nodes[idx].prefix())
        .collect();

    print!("Path IDs: ");
    for &idx in ground_truth_indices {
        print!("{} -> ", nodes[idx].id);
    }
    println!("END");

    print!("Prefixes: ");
    for &p in &prefixes {
        print!("{:02X} -> ", p);
    }
    println!("END");

    // 2. Run Viterbi
    let reconstructed_indices = decode_path(nodes, &prefixes);

    // 3. Verify
    if reconstructed_indices == ground_truth_indices {
        println!("✅ SUCCESS: Viterbi correctly reconstructed the path.");
    } else {
        println!("❌ FAILURE: Viterbi failed.");
        print!("Reconstructed: ");
        for &idx in &reconstructed_indices {
            print!("{} -> ", nodes[idx].id);
        }
        println!("END");
    }
}
