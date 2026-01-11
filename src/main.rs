use std::env;
use std::fs::File;
use std::error::Error;
use app::models::{Repeater, PathNode};
use app::graph::NetworkGraph;
use app::localization;
use serde::{Deserialize, Serialize};

#[derive(Debug, Deserialize)]
struct PacketInput {
    timestamp: String,
    start_lat: f64,
    start_lon: f64,
    end_lat: f64,
    end_lon: f64,
    repeater_prefixes: String,
}

#[derive(Debug, Serialize)]
struct PathOutput {
    timestamp: String,
    start_lat: f64,
    start_lon: f64,
    end_lat: f64,
    end_lon: f64,
    path: Vec<String>,
}

fn main() -> Result<(), Box<dyn Error>> {
    let args: Vec<String> = env::args().collect();
    if args.len() != 5 {
        eprintln!("Usage: {} <repeaters_csv> <packets_csv> <output_yaml> <inferred_unknowns_json>", args[0]);
        std::process::exit(1);
    }

    let repeaters_path = &args[1];
    let packets_path = &args[2];
    let output_path = &args[3];
    let inferred_json_path = &args[4];

    // Read Repeaters
    // Example: ID,Name,Lat,Lon
    // Example: 0x1234,RepeaterA,34.05,-118.25
    let mut repeater_reader = csv::Reader::from_path(repeaters_path)?;
    let mut repeaters: Vec<Repeater> = Vec::new();
    for result in repeater_reader.deserialize() {
        let record: Repeater = result?;
        repeaters.push(record);
    }

    // Clone for lookup since graph takes ownership
    let lookup_nodes = repeaters.clone();

    // Initialize Graph
    // Terrain is None for now as per instructions
    let graph = NetworkGraph::new(repeaters, None);

    // Read Packets
    // Example: timestamp,start_lat,start_lon,end_lat,end_lon,repeater_prefixes
    // Example: 2023-10-27T10:00:00Z,34.05,-118.25,34.10,-118.30,12:a4:b6
    let mut packet_reader = csv::Reader::from_path(packets_path)?;
    let mut outputs: Vec<PathOutput> = Vec::new();
    let mut all_decoded_paths: Vec<Vec<PathNode>> = Vec::new();

    for result in packet_reader.deserialize() {
        let packet: PacketInput = result?;

        // Parse prefixes (assume colon-separated hex strings)
        let prefixes_vec: Vec<u8> = packet.repeater_prefixes
            .split(':')
            .map(|s| s.trim())
            .filter(|s| !s.is_empty())
            .map(|s| {
                let clean = s.trim_start_matches("0x");
                u8::from_str_radix(clean, 16).unwrap_or(0)
            })
            .collect();

        match graph.decode_path(&prefixes_vec) {
            Ok(path_nodes) => {
                let path_strings: Vec<String> = path_nodes.iter().map(|node| {
                    match node {
                        PathNode::Known(idx) => lookup_nodes[*idx].id.clone(),
                        PathNode::Unknown(prefix) => format!("{:02x}", prefix),
                    }
                }).collect();

                outputs.push(PathOutput {
                    timestamp: packet.timestamp,
                    start_lat: packet.start_lat,
                    start_lon: packet.start_lon,
                    end_lat: packet.end_lat,
                    end_lon: packet.end_lon,
                    path: path_strings,
                });

                all_decoded_paths.push(path_nodes);
            },
            Err(e) => {
                eprintln!("Failed to decode path for packet at {}: {}", packet.timestamp, e);
            }
        }
    }

    // Write Output
    let f = File::create(output_path)?;
    serde_yaml::to_writer(f, &outputs)?;

    // Step 4: Localize Unknowns
    let inferred_unknowns = localization::localize_unknowns(&all_decoded_paths, &lookup_nodes);

    // Write Inferred Unknowns to JSON
    let json_file = File::create(inferred_json_path)?;
    serde_json::to_writer_pretty(json_file, &inferred_unknowns)?;

    Ok(())
}
