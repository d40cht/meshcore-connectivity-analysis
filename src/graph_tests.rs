#[cfg(test)]
mod tests {
    use crate::graph::NetworkGraph;
    use crate::models::{PathNode, Repeater};

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
    fn test_winding_path_reconstruction() {
        // Scenario: A -> B -> C -> D -> E
        // Direct path from A to E is blocked/too far.
        // A direct neighbor search from A would NOT find E.
        // But the Viterbi should follow the chain of neighbors.

        // Coordinates: 0.4 degrees is ~44km.
        // Physics model: 44km is well within range and bulge is small.
        // A (0,0)
        // B (0, 0.4)
        // C (0, 0.8)
        // D (0, 1.2)
        // E (0, 1.6)

        let nodes = vec![
            create_node("A00000", 0.0, 0.0), // Index 0
            create_node("B00000", 0.0, 0.4), // Index 1
            create_node("C00000", 0.0, 0.8), // Index 2
            create_node("D00000", 0.0, 1.2), // Index 3
            create_node("E00000", 0.0, 1.6), // Index 4
        ];

        // We observe the full sequence
        let obs = vec![0xA0, 0xB0, 0xC0, 0xD0, 0xE0];

        let graph = NetworkGraph::new(nodes, None);
        let result = graph.decode_path(&obs).expect("Viterbi failed");

        assert_eq!(result.len(), 5);
        assert_eq!(result[0], PathNode::Known(0));
        assert_eq!(result[1], PathNode::Known(1));
        assert_eq!(result[2], PathNode::Known(2));
        assert_eq!(result[3], PathNode::Known(3));
        assert_eq!(result[4], PathNode::Known(4));
    }

    #[test]
    fn test_disconnected_components() {
        // A (0,0) and B (0,10) are disconnected (>1000km).
        // Obs: [A0, B0].
        // Should transition via Unknown because Known->Known is impossible.

        let nodes = vec![
            create_node("A00000", 0.0, 0.0),
            create_node("B00000", 0.0, 10.0),
        ];

        let obs = vec![0xA0, 0xB0];
        let graph = NetworkGraph::new(nodes, None);

        let result = graph.decode_path(&obs).expect("Viterbi failed");

        // We expect A -> Unknown(B0).
        if result[0] != PathNode::Known(0) {
            println!("Result: {:?}", result);
        }

        assert_eq!(
            result[0],
            PathNode::Known(0),
            "Should start with Known node A"
        );
        assert_eq!(
            result[1],
            PathNode::Unknown(0xB0),
            "Should fallback to Unknown for B"
        );
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

        let graph = NetworkGraph::new(nodes, None);
        let result = graph.decode_path(&obs).expect("Viterbi failed");

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

        let graph = NetworkGraph::new(nodes, None);
        let result = graph.decode_path(&obs).expect("Viterbi failed");

        assert_eq!(result.len(), 4);
        assert_eq!(result[0], PathNode::Known(0));
        assert_eq!(result[1], PathNode::Unknown(0xB0));
        assert_eq!(result[2], PathNode::Unknown(0xB1));
        assert_eq!(result[3], PathNode::Known(1));
    }

    #[test]
    fn test_confounding_prefix_mismatch() {
        // Scenario: A -> (B_Unknown) -> C
        // There is a Node B_bad at a perfect location, but it has the wrong prefix.
        // We should select Unknown(B) instead of Known(B_bad).

        let node_a = create_node("A00000", 0.0, 0.0);
        let node_c = create_node("C00000", 0.2, 0.0);
        // Node "B_bad": Perfect middle spot (0.1, 0.0) but prefix D0 (mismatch for B0)
        let node_b_bad = create_node("D00000", 0.1, 0.0);

        let nodes = vec![node_a, node_c, node_b_bad];

        // Observations: A0, B0, C0
        let obs = vec![0xA0, 0xB0, 0xC0];

        let graph = NetworkGraph::new(nodes, None);
        let result = graph.decode_path(&obs).expect("Viterbi failed");

        // Should pick A -> Unknown(B0) -> C
        // Not A -> D0 -> C (impossible due to emission)
        assert_eq!(result.len(), 3);
        assert_eq!(result[0], PathNode::Known(0)); // A
        assert_eq!(result[1], PathNode::Unknown(0xB0)); // Unknown B0
        assert_eq!(result[2], PathNode::Known(1)); // C
    }

    #[test]
    fn test_confounding_distance_cost() {
        // Scenario: A -> (B_Unknown) -> C
        // Node B_far exists with correct prefix (B0), but is way too far.
        // Cost(A->B_far) should be > UNKNOWN_LINK_COST (8.0).
        // Viterbi should prefer Unknown(B0) over Known(B_far).

        let node_a = create_node("A00000", 0.0, 0.0);
        let node_c = create_node("C00000", 0.2, 0.0); // ~22km from A
        // Node B_far: Correct prefix B0, but at (2.0, 0.0) -> ~222km away
        // link_cost for 222km is likely INFINITY or very high (>8.0).
        let node_b_far = create_node("B00000", 2.0, 0.0);

        let nodes = vec![node_a, node_c, node_b_far];

        let obs = vec![0xA0, 0xB0, 0xC0];

        let graph = NetworkGraph::new(nodes, None);
        let result = graph.decode_path(&obs).expect("Viterbi failed");

        assert_eq!(result.len(), 3);
        assert_eq!(result[0], PathNode::Known(0));
        assert_eq!(result[1], PathNode::Unknown(0xB0)); // Should prefer Unknown due to lower cost
        assert_eq!(result[2], PathNode::Known(1));
    }
}
