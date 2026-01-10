#[cfg(test)]
mod tests {
    use crate::graph::NetworkGraph;
    use crate::models::{PathNode, Repeater};
    use crate::terrain::{TerrainMap, TerrainTile};

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
        let nodes = vec![
            create_node("A00000", 0.0, 0.0), // Index 0
            create_node("B00000", 0.0, 0.4), // Index 1
            create_node("C00000", 0.0, 0.8), // Index 2
            create_node("D00000", 0.0, 1.2), // Index 3
            create_node("E00000", 0.0, 1.6), // Index 4
        ];

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
    fn test_winding_path_with_terrain_blockage() {
        // Map: 100x100km. Range approx -0.45 to +0.45 deg.
        // We manually construct a tile for this test.

        let width_km: f64 = 100.0;
        let height_km: f64 = 100.0;
        let resolution_m: f64 = 30.0;
        let center_lat: f64 = 0.0;
        let center_lon: f64 = 0.0;

        // Bounds calc logic copied from TerrainMap::calc_bounds to match dimensions
        let km_per_deg_lat = 111.0;
        let km_per_deg_lon = 111.0 * center_lat.to_radians().cos();

        let height_deg = height_km / km_per_deg_lat;
        let width_deg = width_km / km_per_deg_lon;

        let min_lat = center_lat - height_deg / 2.0;
        let max_lat = center_lat + height_deg / 2.0;
        let min_lon = center_lon - width_deg / 2.0;
        let max_lon = center_lon + width_deg / 2.0;

        let rows = (height_km * 1000.0 / resolution_m).ceil() as usize;
        let cols = (width_km * 1000.0 / resolution_m).ceil() as usize;

        let mut data = vec![0.0; rows * cols];

        // Add wall at lon=0.05.
        // Range = 0.9. (-0.45 to 0.45).
        // 0.05 is at fraction (0.05 - (-0.45))/0.9 = 0.5/0.9 = 0.555.
        let wall_col = (cols as f64 * 0.555) as usize;
        for r in 0..rows {
            data[r * cols + wall_col] = 2000.0; // Huge wall
        }

        // Gap at "top" (North, lat > 0.1).
        // 0.1 is at fraction (0.1 + 0.45)/0.9 = 0.55/0.9 = 0.61.
        let gap_start_row = (rows as f64 * 0.61) as usize;
        for r in gap_start_row..rows {
             data[r * cols + wall_col] = 0.0;
        }

        let tile = TerrainTile {
            min_lat,
            min_lon,
            max_lat,
            max_lon,
            width: cols,
            height: rows,
            data,
        };

        let map = TerrainMap::new(vec![tile]);

        // Nodes
        let a = create_node("A00000", 0.0, -0.05);
        let d = create_node("D00000", 0.0, 0.15);
        let b = create_node("B00000", 0.15, 0.0);
        let c = create_node("C00000", 0.15, 0.1);

        let nodes = vec![a.clone(), b.clone(), c.clone(), d.clone()];
        // Indices: A=0, B=1, C=2, D=3

        let obs = vec![0xA0, 0xB0, 0xC0, 0xD0];

        let graph = NetworkGraph::new(nodes, Some(&map));
        let result = graph.decode_path(&obs).expect("Viterbi failed");

        assert_eq!(result.len(), 4);
        assert_eq!(result[0], PathNode::Known(0)); // A
        assert_eq!(result[1], PathNode::Known(1)); // B
        assert_eq!(result[2], PathNode::Known(2)); // C
        assert_eq!(result[3], PathNode::Known(3)); // D
    }

    #[test]
    fn test_disconnected_components() {
        let nodes = vec![
            create_node("A00000", 0.0, 0.0),
            create_node("B00000", 0.0, 10.0),
        ];

        let obs = vec![0xA0, 0xB0];
        let graph = NetworkGraph::new(nodes, None);

        let result = graph.decode_path(&obs).expect("Viterbi failed");

        assert_eq!(result[0], PathNode::Known(0), "Should start with Known node A");
        assert_eq!(result[1], PathNode::Unknown(0xB0), "Should fallback to Unknown for B");
    }

    #[test]
    fn test_unknown_repeater_recovery() {
        let node_a = create_node("A00000", 0.0, 0.0);
        let node_c = create_node("C00000", 0.5, 0.0);
        let nodes = vec![node_a, node_c];

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
        let node_a = create_node("A00000", 0.0, 0.0);
        let node_c = create_node("C00000", 1.0, 0.0);
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
        let node_a = create_node("A00000", 0.0, 0.0);
        let node_c = create_node("C00000", 0.2, 0.0);
        let node_b_bad = create_node("D00000", 0.1, 0.0);

        let nodes = vec![node_a, node_c, node_b_bad];

        let obs = vec![0xA0, 0xB0, 0xC0];

        let graph = NetworkGraph::new(nodes, None);
        let result = graph.decode_path(&obs).expect("Viterbi failed");

        assert_eq!(result.len(), 3);
        assert_eq!(result[0], PathNode::Known(0)); // A
        assert_eq!(result[1], PathNode::Unknown(0xB0)); // Unknown B0
        assert_eq!(result[2], PathNode::Known(1)); // C
    }

    #[test]
    fn test_confounding_distance_cost() {
        let node_a = create_node("A00000", 0.0, 0.0);
        let node_c = create_node("C00000", 0.2, 0.0);
        let node_b_far = create_node("B00000", 2.0, 0.0);

        let nodes = vec![node_a, node_c, node_b_far];

        let obs = vec![0xA0, 0xB0, 0xC0];

        let graph = NetworkGraph::new(nodes, None);
        let result = graph.decode_path(&obs).expect("Viterbi failed");

        assert_eq!(result.len(), 3);
        assert_eq!(result[0], PathNode::Known(0));
        assert_eq!(result[1], PathNode::Unknown(0xB0));
        assert_eq!(result[2], PathNode::Known(1));
    }
}
