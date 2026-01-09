use app::models::Repeater;
use csv::Writer;
use rand::rngs::StdRng;
use rand::{Rng, SeedableRng};
use std::fs::File;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Center point (e.g., somewhere in UK)
    let center_lat = 51.5074;
    let center_lon = -0.1278;

    let mut nodes = Vec::new();
    let mut rng = StdRng::seed_from_u64(42);

    // 1. Generate a mesh of random nodes
    for i in 0..100 {
        let lat_offset = rng.random_range(-0.5..0.5); // +/- 50km approx
        let lon_offset = rng.random_range(-0.8..0.8); // +/- 50km approx

        // Random Hex ID
        let id_val = rng.random_range(0..0xFFFFFF);
        let id_str = format!("{:06X}", id_val);

        nodes.push(Repeater {
            id: id_str,
            name: format!("Node_{}", i),
            lat: center_lat + lat_offset,
            lon: center_lon + lon_offset,
        });
    }

    // 2. Inject Local Clash
    // Two nodes close to each other (~30km) with SAME prefix
    // Prefix 0xAA
    // Node A
    nodes.push(Repeater {
        id: "AA1111".to_string(),
        name: "Clash_Local_A".to_string(),
        lat: center_lat + 0.1, // ~11km North
        lon: center_lon,
    });
    // Node B
    nodes.push(Repeater {
        id: "AA2222".to_string(),
        name: "Clash_Local_B".to_string(),
        lat: center_lat + 0.15, // ~16km North (5km apart) - wait, plan said ~30km apart.
        // 0.1 deg lat is ~11km. 0.3 deg is ~33km.
        lon: center_lon + 0.3, // ~20km East
    });

    // 3. Inject Global Clash
    // Two nodes FAR apart (>200km) with SAME prefix
    // Prefix 0xBB
    // Node C (Near Center)
    nodes.push(Repeater {
        id: "BB1111".to_string(),
        name: "Clash_Global_C".to_string(),
        lat: center_lat,
        lon: center_lon,
    });
    // Node D (Far away)
    // 2 degrees lat is ~222km
    nodes.push(Repeater {
        id: "BB2222".to_string(),
        name: "Clash_Global_D".to_string(),
        lat: center_lat + 2.5,
        lon: center_lon + 2.0,
    });

    // Write to CSV
    let file = File::create("repeaters.csv")?;
    let mut wtr = Writer::from_writer(file);

    for node in nodes {
        wtr.serialize(node)?;
    }

    wtr.flush()?;
    println!("Generated repeaters.csv with {} nodes.", 104);
    Ok(())
}
