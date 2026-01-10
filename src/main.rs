use std::env;
use std::fs::File;
use std::error::Error;
use std::path::Path;
use app::models::{Repeater, PathNode};
use app::graph::NetworkGraph;
use app::terrain::{TerrainMap, TerrainTile};
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

    // Parse arguments
    // Expected: [program] [pos1] [pos2] [pos3] [--terrain file1 --terrain file2 ...]
    // Or mixed.
    // We need 3 positional args: repeaters_csv, packets_csv, output_yaml.
    // And optional list of terrain files.

    let mut positional_args = Vec::new();
    let mut terrain_files = Vec::new();

    let mut i = 1;
    while i < args.len() {
        if args[i] == "--terrain" {
            if i + 1 < args.len() {
                terrain_files.push(args[i + 1].clone());
                i += 2;
            } else {
                eprintln!("Error: --terrain flag requires a file path");
                std::process::exit(1);
            }
        } else {
            positional_args.push(args[i].clone());
            i += 1;
        }
    }

    if positional_args.len() != 3 {
        eprintln!("Usage: {} <repeaters_csv> <packets_csv> <output_yaml> [--terrain <geotiff_path>]...", args[0]);
        std::process::exit(1);
    }

    let repeaters_path = &positional_args[0];
    let packets_path = &positional_args[1];
    let output_path = &positional_args[2];

    // Load Terrain
    let mut terrain_tiles = Vec::new();
    for path_str in terrain_files {
        let path = Path::new(&path_str);
        match TerrainTile::from_geotiff(path) {
            Ok(tile) => {
                println!("Loaded terrain tile from {}", path_str);
                terrain_tiles.push(tile);
            },
            Err(e) => {
                eprintln!("Failed to load terrain tile {}: {}", path_str, e);
                // Depending on requirements, maybe exit? User said "return an error...".
                // We'll treat loading failure as fatal for that file, and if requested, we should probably fail fully?
                // For now, let's warn and continue, or fail?
                // "return an error so as not to get silent infeasible paths" -> this was about missing data during lookup.
                // If we explicitly asked for a file and it fails, we should probably stop.
                std::process::exit(1);
            }
        }
    }

    let terrain_map = if !terrain_tiles.is_empty() {
        Some(TerrainMap::new(terrain_tiles))
    } else {
        None
    };

    // Read Repeaters
    let mut repeater_reader = csv::Reader::from_path(repeaters_path)?;
    let mut repeaters: Vec<Repeater> = Vec::new();
    for result in repeater_reader.deserialize() {
        let record: Repeater = result?;
        repeaters.push(record);
    }

    // Clone for lookup since graph takes ownership
    let lookup_nodes = repeaters.clone();

    // Initialize Graph
    let graph = NetworkGraph::new(repeaters, terrain_map.as_ref());

    // Read Packets
    let mut packet_reader = csv::Reader::from_path(packets_path)?;
    let mut outputs: Vec<PathOutput> = Vec::new();

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
