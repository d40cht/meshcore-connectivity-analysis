use rand::rngs::StdRng;
use rand::{Rng, SeedableRng};

pub struct TerrainMap {
    pub min_lat: f64,
    pub min_lon: f64,
    pub max_lat: f64,
    pub max_lon: f64,
    pub resolution_deg: f64, // approximate degrees per cell
    pub width: usize,
    pub height: usize,
    pub data: Vec<f64>, // Row-major elevation data in meters
}

impl TerrainMap {
    /// Creates a new TerrainMap filled with generated pink noise.
    ///
    /// # Arguments
    /// * `center_lat`, `center_lon`: The center of the map.
    /// * `width_km`, `height_km`: Total size of the area in km.
    /// * `resolution_m`: Approximate resolution of each grid cell in meters (e.g., 30.0).
    pub fn new_random(
        center_lat: f64,
        center_lon: f64,
        width_km: f64,
        height_km: f64,
        resolution_m: f64,
    ) -> Self {
        let km_per_deg_lat = 111.0;
        let km_per_deg_lon = 111.0 * center_lat.to_radians().cos();

        let height_deg = height_km / km_per_deg_lat;
        let width_deg = width_km / km_per_deg_lon;

        let min_lat = center_lat - height_deg / 2.0;
        let max_lat = center_lat + height_deg / 2.0;
        let min_lon = center_lon - width_deg / 2.0;
        let max_lon = center_lon + width_deg / 2.0;

        let res_deg_lat = (resolution_m / 1000.0) / km_per_deg_lat;

        // We use a square grid in terms of samples, but derived from meters
        let rows = (height_km * 1000.0 / resolution_m).ceil() as usize;
        let cols = (width_km * 1000.0 / resolution_m).ceil() as usize;

        let mut rng = StdRng::seed_from_u64(12345);
        let mut data = vec![0.0; rows * cols];

        // Generate Pink Noise (Fractal Brownian Motion)
        // We sum multiple octaves of noise.
        let octaves = 6;
        let persistence = 0.5;
        let amplitude = 150.0; // Base max height variance (meters) - Reduced to ~150m to avoid >50% blockage

        // Base ground level (e.g. 50m above sea level)
        let base_level = 50.0;

        // To keep it simple without external noise crates, we'll just synthesize
        // some "smooth" noise by combining sine waves with random phases.
        // This is not strictly "pink noise" in the DSP sense but gives topographic hills.

        let random_phases: Vec<(f64, f64)> = (0..octaves * 4)
            .map(|_| (rng.random_range(0.0..100.0), rng.random_range(0.0..100.0)))
            .collect();

        for r in 0..rows {
            for c in 0..cols {
                let y = r as f64 / rows as f64;
                let x = c as f64 / cols as f64;

                let mut elevation = base_level;
                let mut amp = amplitude;
                let mut freq = 4.0; // Start with some hills

                for i in 0..octaves {
                    let phase_x = random_phases[i].0;
                    let phase_y = random_phases[i].1;

                    // Simple superposition of sine waves
                    elevation += amp * ((x * freq + phase_x).sin() * (y * freq + phase_y).cos());

                    amp *= persistence;
                    freq *= 2.0;
                }

                // Clamp to non-negative
                if elevation < 0.0 {
                    elevation = 0.0;
                }

                data[r * cols + c] = elevation;
            }
        }

        TerrainMap {
            min_lat,
            min_lon,
            max_lat,
            max_lon,
            resolution_deg: res_deg_lat,
            width: cols,
            height: rows,
            data,
        }
    }

    /// Gets the elevation at a specific latitude and longitude using bilinear interpolation.
    pub fn get_elevation(&self, lat: f64, lon: f64) -> f64 {
        if lat < self.min_lat || lat > self.max_lat || lon < self.min_lon || lon > self.max_lon {
            return 0.0; // Out of bounds, assume sea level
        }

        // Map lat/lon to grid coordinates (float)
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

    /// Checks if there is Line of Sight (LOS) between two points.
    /// Returns true if the path is CLEAR (no obstruction).
    /// Returns false if BLOCKED.
    ///
    /// * `h1_m`, `h2_m`: Antenna heights in meters (added to terrain elevation).
    pub fn check_line_of_sight(
        &self,
        lat1: f64,
        lon1: f64,
        h1_m: f64,
        lat2: f64,
        lon2: f64,
        h2_m: f64,
    ) -> bool {
        // TODO: Add Fresnel zone calculation for more realistic physics.
        // Currently implements simple geometric line-of-sight.

        let dist_km = crate::physics::haversine_distance(lat1, lon1, lat2, lon2);
        if dist_km == 0.0 {
            return true;
        }

        let steps = (dist_km * 1000.0 / 30.0).ceil() as usize; // Sample every ~30m
        if steps < 2 {
            return true;
        }

        let start_elev = self.get_elevation(lat1, lon1);
        let end_elev = self.get_elevation(lat2, lon2);

        let start_total_h = start_elev + h1_m;
        let end_total_h = end_elev + h2_m;

        for i in 1..steps {
            let t = i as f64 / steps as f64;
            let lat = lat1 + (lat2 - lat1) * t;
            let lon = lon1 + (lon2 - lon1) * t;

            // Simple linear interpolation of the ray height (flat earth approximation for short segments,
            // but we should ideally account for earth curvature if segments are long).
            // Since we are already doing Earth Bulge calculation in the cost function,
            // we should ideally combine them.
            // However, usually LOS checks include earth curvature by adjusting the terrain height relative to the chord
            // or adjusting the ray.
            // For simplicity here: Ray is a straight line in 3D cartesian space?
            // No, we operate in lat/lon/alt.

            // "Correct" approach for long distance:
            // The ray height drops relative to the surface due to earth curvature.
            // Drop d = dist_from_start * dist_from_end / (2 * R)
            // But physics.rs calculates bulge separately.
            // If we want this function to be pure "is there a mountain", we should just check geometric LOS
            // assuming "up" is parallel.
            // BUT, a mountain 50km away might block LOS because of curvature + height.
            // Let's include curvature in the "ray height" check.

            // Ray height at fraction t (linear interpolation)
            let ray_h = start_total_h * (1.0 - t) + end_total_h * t;

            // Earth bulge at this point (relative to the straight line chord between surface points)
            // Note: physics::earth_bulge calculates the "hump" height.
            // Effectively, the terrain "rises" by the bulge amount relative to the chord.
            // OR the ray "sinks".
            // Let's model it as: Is (Terrain + Bulge) > Ray?
            // Wait, usually Bulge is calculated for the midpoint.
            // Correct logic:
            //   Effective Terrain Height = Actual Terrain Elevation + Earth Curvature Drop
            //   We compare this to the Line between (Start+H1) and (End+H2).
            //   Curvature Drop h = d1 * d2 / (2 * R). (Approx).
            //   Let's reuse `physics::earth_bulge` but applied to the segments.
            //   Actually `physics::earth_bulge` is h = d^2/8R (midpoint).
            //   General formula for point at distance x from start in link of length D:
            //   h_curvature = x * (D - x) / (2 * R)
            //   We check: Ray_Height(t) < Terrain_Height(t) + h_curvature?
            //   Wait, Ray_Height is altitude above sea level (approx).
            //   We should compare:
            //     Height_of_Ray_Above_Sea_Level vs Terrain_Height_Above_Sea_Level + Earth_Bulge_Correction?
            //   No.
            //   Let's visualize. Earth is curved. Ray is straight.
            //   At midpoint, the earth surface is "higher" relative to the chord connecting the endpoints' sea-levels.
            //   So yes, we ADD the bulge to the terrain elevation to check against the straight ray.

            let dist_from_start_km = dist_km * t;
            let dist_from_end_km = dist_km * (1.0 - t);

            // Earth radius 6371km.
            // h = (d1 * d2) / (2 * R)
            // We need consistent units. Meters.
            let r_meters = 6371.0 * 1000.0;
            let d1_m = dist_from_start_km * 1000.0;
            let d2_m = dist_from_end_km * 1000.0;
            let curvature_m = (d1_m * d2_m) / (2.0 * r_meters);

            let terrain_h = self.get_elevation(lat, lon);

            // Check
            // We assume the ray is a straight line between the two antenna tips.
            // The "ground" effectively rises up by `curvature_m` relative to that chord.
            if ray_h < (terrain_h + curvature_m) {
                return false; // Blocked
            }
        }

        true
    }
}
