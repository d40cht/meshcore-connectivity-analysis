use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Repeater {
    #[serde(rename = "ID")]
    pub id: String,
    #[serde(rename = "Name")]
    pub name: String,
    #[serde(rename = "Lat")]
    pub lat: f64,
    #[serde(rename = "Lon")]
    pub lon: f64,
}

impl Repeater {
    pub fn prefix(&self) -> u8 {
        let clean_id = self.id.trim_start_matches("0x");
        u8::from_str_radix(&clean_id[0..2], 16).unwrap_or(0)
    }
}

/// Represents a node in the reconstructed path.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum PathNode {
    /// A known repeater from the database (index into the nodes list).
    Known(usize),
    /// An unknown repeater, identified only by its 1-byte prefix.
    Unknown(u8),
}
