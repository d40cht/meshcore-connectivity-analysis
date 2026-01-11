use app::localization::localize_unknowns;
use app::models::{PathNode, Repeater};

fn make_repeater(id: &str, lat: f64, lon: f64) -> Repeater {
    Repeater {
        id: id.to_string(),
        name: "Test".to_string(),
        lat,
        lon,
    }
}

#[test]
fn test_localize_simple_midpoint() {
    // Scenario 1: Simple case
    // K1 (0,0) -> U(prefix AA) -> K2 (0, 2)
    // Expected midpoint: (0, 1)

    let k1 = make_repeater("0x1111", 0.0, 0.0);
    let k2 = make_repeater("0x2222", 0.0, 2.0); // Approx 222km away at equator
    let known_nodes = vec![k1, k2];

    // Path: K1 -> U(AA) -> K2
    let path = vec![
        PathNode::Known(0),
        PathNode::Unknown(0xAA),
        PathNode::Known(1),
    ];

    let paths = vec![path];

    let results = localize_unknowns(&paths, &known_nodes);

    assert_eq!(results.len(), 1);
    let res = &results[0];
    assert_eq!(res.prefix, "aa");
    assert!((res.lat - 0.0).abs() < 1e-6);
    assert!((res.lon - 1.0).abs() < 1e-6);
    assert_eq!(res.observation_count, 1);
}

#[test]
fn test_localize_multiple_observations_cluster() {
    // Scenario: Two paths implying same location
    // Path 1: K1(0,0) -> U(BB) -> K2(0,2)  => Mid (0,1)
    // Path 2: K3(1,1) -> U(BB) -> K4(-1,1) => Mid (0,1)

    let k1 = make_repeater("0x1111", 0.0, 0.0);
    let k2 = make_repeater("0x2222", 0.0, 2.0);
    let k3 = make_repeater("0x3333", 1.0, 1.0);
    let k4 = make_repeater("0x4444", -1.0, 1.0);
    let known_nodes = vec![k1, k2, k3, k4];

    let path1 = vec![
        PathNode::Known(0),
        PathNode::Unknown(0xBB),
        PathNode::Known(1),
    ];
    let path2 = vec![
        PathNode::Known(2),
        PathNode::Unknown(0xBB),
        PathNode::Known(3),
    ];

    let paths = vec![path1, path2];

    let results = localize_unknowns(&paths, &known_nodes);

    assert_eq!(results.len(), 1);
    let res = &results[0];
    assert_eq!(res.prefix, "bb");
    assert!((res.lat - 0.0).abs() < 1e-6);
    assert!((res.lon - 1.0).abs() < 1e-6);
    assert_eq!(res.observation_count, 2);
}

#[test]
fn test_localize_split_clusters() {
    // Scenario 2: Split Clusters
    // Prefix CC used in two far away places.
    // Cluster 1: Near (0,0)
    //    K1(-0.1, 0) -> U -> K2(0.1, 0) => Mid (0,0)
    // Cluster 2: Near (10, 10) (approx 1000km away)
    //    K3(9.9, 10) -> U -> K4(10.1, 10) => Mid (10, 10)

    let k1 = make_repeater("0x11", -0.1, 0.0);
    let k2 = make_repeater("0x22", 0.1, 0.0);
    let k3 = make_repeater("0x33", 9.9, 10.0);
    let k4 = make_repeater("0x44", 10.1, 10.0);

    let known_nodes = vec![k1, k2, k3, k4];

    let path1 = vec![
        PathNode::Known(0),
        PathNode::Unknown(0xCC),
        PathNode::Known(1),
    ];
    let path2 = vec![
        PathNode::Known(2),
        PathNode::Unknown(0xCC),
        PathNode::Known(3),
    ];

    let paths = vec![path1, path2];

    let results = localize_unknowns(&paths, &known_nodes);

    assert_eq!(results.len(), 2);

    // Results are sorted by prefix then lat
    // (0,0) comes before (10,10)
    let r1 = &results[0];
    let r2 = &results[1];

    assert_eq!(r1.prefix, "cc");
    assert!((r1.lat - 0.0).abs() < 1e-6);
    assert!((r1.lon - 0.0).abs() < 1e-6);

    assert_eq!(r2.prefix, "cc");
    assert!((r2.lat - 10.0).abs() < 1e-6);
    assert!((r2.lon - 10.0).abs() < 1e-6);
}

#[test]
fn test_localize_ignore_non_triplets() {
    // Path: K1 -> U -> U -> K2
    // Should be ignored by Phase 1 as per requirements (only K->U->K)

    let k1 = make_repeater("0x11", 0.0, 0.0);
    let k2 = make_repeater("0x22", 0.0, 2.0);
    let known_nodes = vec![k1, k2];

    let path = vec![
        PathNode::Known(0),
        PathNode::Unknown(0xDD),
        PathNode::Unknown(0xEE), // Chain
        PathNode::Known(1),
    ];

    let paths = vec![path];

    let results = localize_unknowns(&paths, &known_nodes);
    assert!(results.is_empty());
}
