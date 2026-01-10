use rand::rngs::StdRng;
use rand::{Rng, SeedableRng};
use std::fs::File;
use std::io::BufReader;
use std::path::Path;
use anyhow::{Result, anyhow, Context};
use tiff::decoder::{Decoder, DecodingResult};
use tiff::tags::Tag;
use tiff::decoder::ifd::Value;

pub struct TerrainTile {
    pub min_lat: f64,
    pub min_lon: f64,
    pub max_lat: f64,
    pub max_lon: f64,
    pub width: usize,
    pub height: usize,
    pub data: Vec<f64>, // Row-major elevation data in meters
}

impl TerrainTile {
    pub fn contains(&self, lat: f64, lon: f64) -> bool {
        lat >= self.min_lat && lat <= self.max_lat && lon >= self.min_lon && lon <= self.max_lon
    }

    pub fn get_elevation(&self, lat: f64, lon: f64) -> f64 {
        if !self.contains(lat, lon) {
            return 0.0;
        }

        let lat_norm = (lat - self.min_lat) / (self.max_lat - self.min_lat);
        let lon_norm = (lon - self.min_lon) / (self.max_lon - self.min_lon);

        let r_float = lat_norm * (self.height - 1) as f64;
        let c_float = lon_norm * (self.width - 1) as f64;

        let r0 = r_float.floor() as usize;
        let c0 = c_float.floor() as usize;
        let r1 = (r0 + 1).min(self.height - 1);
        let c1 = (c0 + 1).min(self.width - 1);

        let dr = r_float - r0 as f64;
        let dc = c_float - c0 as f64;

        let h00 = self.data[r0 * self.width + c0];
        let h01 = self.data[r0 * self.width + c1];
        let h10 = self.data[r1 * self.width + c0];
        let h11 = self.data[r1 * self.width + c1];

        // Bilinear interpolation
        let h0 = h00 * (1.0 - dc) + h01 * dc;
        let h1 = h10 * (1.0 - dc) + h11 * dc;

        h0 * (1.0 - dr) + h1 * dr
    }

    pub fn from_geotiff(path: &Path) -> Result<Self> {
        let file = File::open(path).context("Failed to open GeoTIFF file")?;
        let reader = BufReader::new(file);
        let mut decoder = Decoder::new(reader).context("Failed to create TIFF decoder")?;

        let (width, height) = decoder.dimensions().context("Failed to read dimensions")?;

        // Read Tags
        // ModelPixelScaleTag = 33550
        // ModelTiepointTag = 33922
        // GeoKeyDirectoryTag = 34735 (Not strictly needed if we assume standard WGS84 mapping for SRTM)

        let pixel_scale_tag = Tag::Unknown(33550);
        let tiepoint_tag = Tag::Unknown(33922);

        let pixel_scale_val = decoder.get_tag(pixel_scale_tag).context("Missing ModelPixelScaleTag")?;
        let tiepoint_val = decoder.get_tag(tiepoint_tag).context("Missing ModelTiepointTag")?;

        let scale = match pixel_scale_val {
            Value::List(v) => {
                 match v.first() {
                     Some(Value::Double(_)) => {
                         // Double (f64)
                        let mut scales = Vec::new();
                        for item in v {
                             if let Value::Double(f) = item {
                                 scales.push(f);
                             }
                        }
                        if scales.len() < 2 { return Err(anyhow!("Invalid ModelPixelScaleTag length")); }
                        (scales[0], scales[1])
                     },
                     Some(Value::Float(_)) => {
                         // Float (f32)
                        let mut scales = Vec::new();
                        for item in v {
                             if let Value::Float(f) = item {
                                 scales.push(f as f64);
                             }
                        }
                        if scales.len() < 2 { return Err(anyhow!("Invalid ModelPixelScaleTag length")); }
                        (scales[0], scales[1])
                     },
                     // Some TIFFs might use rational? Less common for this tag.
                     _ => return Err(anyhow!("Invalid ModelPixelScaleTag format (expected List of Double or Float)")),
                 }
            },
            _ => return Err(anyhow!("Invalid ModelPixelScaleTag format (expected List)")),
        };

        let tiepoint = match tiepoint_val {
             Value::List(v) => {
                 let mut points = Vec::new();
                 for item in v {
                     if let Value::Double(f) = item {
                         points.push(f);
                     } else if let Value::Float(f) = item {
                         points.push(f as f64);
                     }
                 }
                 // Expect 6 doubles: I, J, K, X, Y, Z. Usually (0,0,0) -> (Lon, Lat, 0)
                 if points.len() < 6 { return Err(anyhow!("Invalid ModelTiepointTag length")); }
                 (points[3], points[4]) // X (Lon), Y (Lat)
             },
             _ => return Err(anyhow!("Invalid ModelTiepointTag format")),
        };

        // Parse Data
        // SRTM is usually i16 (signed 16-bit)
        let decoding_result = decoder.read_image().context("Failed to read image data")?;

        let data: Vec<f64> = match decoding_result {
            DecodingResult::I16(raw) => raw.into_iter().map(|x| x as f64).collect(),
            DecodingResult::U16(raw) => raw.into_iter().map(|x| x as f64).collect(),
            DecodingResult::F32(raw) => raw.into_iter().map(|x| x as f64).collect(),
            DecodingResult::F64(raw) => raw,
            _ => return Err(anyhow!("Unsupported data format in GeoTIFF (expected 16-bit int or float)")),
        };

        if data.len() != (width * height) as usize {
            return Err(anyhow!("Data length mismatch: expected {} * {} = {}, got {}", width, height, width*height, data.len()));
        }

        // Coordinate Mapping
        let min_lon = tiepoint.0;
        let max_lat = tiepoint.1;

        let scale_x = scale.0;
        let scale_y = scale.1;

        // Bounds
        let max_lon = min_lon + (width as f64 * scale_x);
        let min_lat = max_lat - (height as f64 * scale_y);

        // Flip data rows to match Bottom-Up internal logic
        // Input: Row 0 = Top.
        // Target: Row 0 = Bottom.

        let mut flipped_data = vec![0.0; data.len()];
        for r in 0..height as usize {
            let target_r = (height as usize - 1) - r;
            let src_start = r * width as usize;
            let dst_start = target_r * width as usize;
            flipped_data[dst_start..dst_start + width as usize].copy_from_slice(&data[src_start..src_start + width as usize]);
        }

        Ok(TerrainTile {
            min_lat,
            min_lon,
            max_lat,
            max_lon,
            width: width as usize,
            height: height as usize,
            data: flipped_data,
        })
    }
}

pub struct TerrainMap {
    tiles: Vec<TerrainTile>,
}

impl TerrainMap {
    pub fn new(tiles: Vec<TerrainTile>) -> Self {
        TerrainMap { tiles }
    }

    /// Creates a new TerrainMap filled with generated pink noise (Single Tile).
    pub fn new_random(
        center_lat: f64,
        center_lon: f64,
        width_km: f64,
        height_km: f64,
        resolution_m: f64,
    ) -> Self {
        let (min_lat, max_lat, min_lon, max_lon, _res_deg_lat, rows, cols) =
            Self::calc_bounds(center_lat, center_lon, width_km, height_km, resolution_m);

        let mut rng = StdRng::seed_from_u64(12345);
        let mut data = vec![0.0; rows * cols];
        let octaves = 6;
        let persistence = 0.5;
        let amplitude = 150.0;
        let base_level = 50.0;
        let random_phases: Vec<(f64, f64)> = (0..octaves * 4)
            .map(|_| (rng.random_range(0.0..100.0), rng.random_range(0.0..100.0)))
            .collect();

        for r in 0..rows {
            for c in 0..cols {
                let y = r as f64 / rows as f64;
                let x = c as f64 / cols as f64;
                let mut elevation = base_level;
                let mut amp = amplitude;
                let mut freq = 4.0;
                for i in 0..octaves {
                    let phase_x = random_phases[i].0;
                    let phase_y = random_phases[i].1;
                    elevation += amp * ((x * freq + phase_x).sin() * (y * freq + phase_y).cos());
                    amp *= persistence;
                    freq *= 2.0;
                }
                if elevation < 0.0 { elevation = 0.0; }
                data[r * cols + c] = elevation;
            }
        }

        TerrainMap {
            tiles: vec![TerrainTile {
                min_lat,
                min_lon,
                max_lat,
                max_lon,
                width: cols,
                height: rows,
                data,
            }]
        }
    }

    /// Creates a new flat TerrainMap (all elevation 0.0) (Single Tile).
    pub fn new_flat(
        center_lat: f64,
        center_lon: f64,
        width_km: f64,
        height_km: f64,
        resolution_m: f64,
    ) -> Self {
        let (min_lat, max_lat, min_lon, max_lon, _res_deg_lat, rows, cols) =
            Self::calc_bounds(center_lat, center_lon, width_km, height_km, resolution_m);

        let data = vec![0.0; rows * cols];

        TerrainMap {
             tiles: vec![TerrainTile {
                min_lat,
                min_lon,
                max_lat,
                max_lon,
                width: cols,
                height: rows,
                data,
            }]
        }
    }

    fn calc_bounds(
        center_lat: f64,
        center_lon: f64,
        width_km: f64,
        height_km: f64,
        resolution_m: f64,
    ) -> (f64, f64, f64, f64, f64, usize, usize) {
        let km_per_deg_lat = 111.0;
        let km_per_deg_lon = 111.0 * center_lat.to_radians().cos();
        let height_deg = height_km / km_per_deg_lat;
        let width_deg = width_km / km_per_deg_lon;
        let min_lat = center_lat - height_deg / 2.0;
        let max_lat = center_lat + height_deg / 2.0;
        let min_lon = center_lon - width_deg / 2.0;
        let max_lon = center_lon + width_deg / 2.0;
        let res_deg_lat = (resolution_m / 1000.0) / km_per_deg_lat;
        let rows = (height_km * 1000.0 / resolution_m).ceil() as usize;
        let cols = (width_km * 1000.0 / resolution_m).ceil() as usize;
        (min_lat, max_lat, min_lon, max_lon, res_deg_lat, rows, cols)
    }

    /// Gets the elevation at a specific latitude and longitude.
    /// Returns Some(elevation) if covered by a tile, None otherwise.
    pub fn get_elevation(&self, lat: f64, lon: f64) -> Option<f64> {
        for tile in &self.tiles {
            if tile.contains(lat, lon) {
                return Some(tile.get_elevation(lat, lon));
            }
        }
        None
    }

    /// Checks if there is Line of Sight (LOS) between two points.
    /// Returns Ok(true) if CLEAR.
    /// Returns Ok(false) if BLOCKED.
    /// Returns Err if data is missing for any part of the path.
    pub fn check_line_of_sight(
        &self,
        lat1: f64,
        lon1: f64,
        h1_m: f64,
        lat2: f64,
        lon2: f64,
        h2_m: f64,
    ) -> Result<bool> {
        let dist_km = crate::physics::haversine_distance(lat1, lon1, lat2, lon2);
        if dist_km == 0.0 {
            return Ok(true);
        }

        let steps = (dist_km * 1000.0 / 30.0).ceil() as usize;
        if steps < 2 {
            return Ok(true);
        }

        let start_elev = self.get_elevation(lat1, lon1).ok_or_else(|| anyhow!("Missing terrain data at start"))?;
        let end_elev = self.get_elevation(lat2, lon2).ok_or_else(|| anyhow!("Missing terrain data at end"))?;

        let start_total_h = start_elev + h1_m;
        let end_total_h = end_elev + h2_m;

        for i in 1..steps {
            let t = i as f64 / steps as f64;
            let lat = lat1 + (lat2 - lat1) * t;
            let lon = lon1 + (lon2 - lon1) * t;

            let ray_h = start_total_h * (1.0 - t) + end_total_h * t;

            let dist_from_start_km = dist_km * t;
            let dist_from_end_km = dist_km * (1.0 - t);
            let r_meters = 6371.0 * 1000.0;
            let d1_m = dist_from_start_km * 1000.0;
            let d2_m = dist_from_end_km * 1000.0;
            let curvature_m = (d1_m * d2_m) / (2.0 * r_meters);

            let terrain_h = self.get_elevation(lat, lon).ok_or_else(|| anyhow!("Missing terrain data along path"))?;

            if ray_h < (terrain_h + curvature_m) {
                return Ok(false); // Blocked
            }
        }

        Ok(true)
    }
}
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_terrain_tile_get_elevation() {
        // Create a mock tile: 10x10 grid.
        // Bounds: Lat 0.0 to 1.0, Lon 0.0 to 1.0.
        // Data: Gradient. Elevation = Lat + Lon.

        let width = 10;
        let height = 10;
        let min_lat = 0.0;
        let max_lat = 1.0;
        let min_lon = 0.0;
        let max_lon = 1.0;

        let mut data = vec![0.0; width * height];
        for r in 0..height {
            for c in 0..width {
                // Internal data storage is now Bottom-Up (r=0 is min_lat)?
                // Wait, in `from_geotiff` I flipped it so r=0 is min_lat.
                // But `get_elevation` math:
                // lat_norm = (lat - min) / (max - min). 0 at min, 1 at max.
                // r_float = lat_norm * (height-1).
                // So r=0 is min_lat.

                // Let's populate data such that Elevation = r + c;
                data[r * width + c] = (r + c) as f64;
            }
        }

        let tile = TerrainTile {
            min_lat,
            min_lon,
            max_lat,
            max_lon,
            width,
            height,
            data,
        };

        // Test corners
        // MinLat, MinLon -> r=0, c=0 -> Elev 0
        assert!((tile.get_elevation(0.0, 0.0) - 0.0).abs() < 1e-6);

        // MaxLat, MaxLon -> r=9, c=9 -> Elev 18
        assert!((tile.get_elevation(1.0, 1.0) - 18.0).abs() < 1e-6);

        // Center -> r=4.5, c=4.5 -> Bilinear interp should yield 9.0
        assert!((tile.get_elevation(0.5, 0.5) - 9.0).abs() < 1e-6);
    }

    #[test]
    fn test_terrain_map_multiple_tiles() {
        // Tile 1: (0,0) to (1,1)
        let t1 = TerrainTile {
            min_lat: 0.0, max_lat: 1.0, min_lon: 0.0, max_lon: 1.0,
            width: 2, height: 2,
            data: vec![10.0, 10.0, 10.0, 10.0], // Flat 10m
        };

        // Tile 2: (1,1) to (2,2)
        let t2 = TerrainTile {
            min_lat: 1.0, max_lat: 2.0, min_lon: 1.0, max_lon: 2.0,
            width: 2, height: 2,
            data: vec![20.0, 20.0, 20.0, 20.0], // Flat 20m
        };

        let map = TerrainMap::new(vec![t1, t2]);

        // Query Tile 1
        assert_eq!(map.get_elevation(0.5, 0.5), Some(10.0));
        // Query Tile 2
        assert_eq!(map.get_elevation(1.5, 1.5), Some(20.0));
        // Query Gap
        assert_eq!(map.get_elevation(0.5, 1.5), None);
    }
}
