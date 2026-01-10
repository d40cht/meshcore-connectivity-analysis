use std::env;
use std::fs::File;
use std::error::Error;
use app::models::{Repeater, PathNode};
use app::graph::NetworkGraph;
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
    if args.len() != 4 {
        eprintln!("Usage: {} <repeaters_csv> <packets_csv> <output_yaml>", args[0]);
        std::process::exit(1);
    }

    let repeaters_path = &args[1];
    let packets_path = &args[2];
    let output_path = &args[3];

    // Read Repeaters
    let mut rdr = csv::Reader::from_path(repeaters_path)?;
    let mut repeaters: Vec<Repeater> = Vec::new();
    for result in rdr.deserialize() {
        let record: Repeater = result?;
        repeaters.push(record);
    }

    // Clone for lookup since graph takes ownership
    let lookup_nodes = repeaters.clone();

    // Initialize Graph
    // Terrain is None for now as per instructions
    let graph = NetworkGraph::new(repeaters, None);

    // Read Packets
    let mut p_rdr = csv::Reader::from_path(packets_path)?;
    let mut outputs: Vec<PathOutput> = Vec::new();

    for result in p_rdr.deserialize() {
        let packet: PacketInput = result?;

        // Parse prefixes (assume comma-separated hex strings)
        let prefixes_vec: Vec<u8> = packet.repeater_prefixes
            .split(',')
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
            },
            Err(e) => {
                eprintln!("Failed to decode path for packet at {}: {}", packet.timestamp, e);
            }
        }
    }

    // Write Output
    let f = File::create(output_path)?;
    serde_yaml::to_writer(f, &outputs)?;

    Ok(())
}
