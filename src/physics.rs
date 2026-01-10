const EARTH_RADIUS_KM: f64 = 6371.0;

/// Calculates the Haversine distance between two points in km.
pub fn haversine_distance(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f64 {
    let lat1_rad = lat1.to_radians();
    let lon1_rad = lon1.to_radians();
    let lat2_rad = lat2.to_radians();
    let lon2_rad = lon2.to_radians();

    let dlat = lat2_rad - lat1_rad;
    let dlon = lon2_rad - lon1_rad;

    let a =
        (dlat / 2.0).sin().powi(2) + lat1_rad.cos() * lat2_rad.cos() * (dlon / 2.0).sin().powi(2);
    let c = 2.0 * a.sqrt().atan2((1.0 - a).sqrt());

    EARTH_RADIUS_KM * c
}

/// Calculates the "Earth Bulge" in meters.
/// Formula: h = d^2 / (8 * R)
pub fn earth_bulge(distance_km: f64) -> f64 {
    if distance_km <= 0.0 {
        return 0.0;
    }
    // Convert R to meters for result in meters
    let r_meters = EARTH_RADIUS_KM * 1000.0;
    let d_meters = distance_km * 1000.0;

    (d_meters * d_meters) / (8.0 * r_meters)
}

/// Calculates the cost (Negative Log Probability) of a link.
///
/// We model the probability as a sigmoid function of distance and "feasibility".
///
/// P(link) = P_dist * P_obs
/// Cost = -ln(P(link))
///
/// For this simulation:
/// - Max realistic range: ~100km
/// - Antenna height assumption: ~30m. If bulge > 30m, probability drops sharply.
pub fn link_cost(
    lat1: f64,
    lon1: f64,
    lat2: f64,
    lon2: f64,
    terrain: Option<&crate::terrain::TerrainMap>,
) -> f64 {
    let dist_km = haversine_distance(lat1, lon1, lat2, lon2);

    // Terrain Check
    if let Some(map) = terrain {
        // Assume 30m antenna height for both
        if !map.check_line_of_sight(lat1, lon1, 30.0, lat2, lon2, 30.0) {
            // Blocked by terrain!
            // Add a massive penalty. e.g. +30.0 in log-space (e^-30 is tiny)
            // Existing max cost is around 1000.0 (from 1e-10).
            // Let's return a very high cost that isn't INFINITY but effectively rules it out
            // unless there are NO other options.
            // But if it's blocked, it's physically impossible in this model.
            // Let's add 50.0 to the calculated cost or just return High.
            return 2000.0;
        }
    }

    // Hard cutoff for performance/reality
    if dist_km > 150.0 {
        return f64::INFINITY;
    }

    let bulge_m = earth_bulge(dist_km);

    // Sigmoid for distance decay
    // Center at 50km, steepness 0.1
    let dist_prob = 1.0 / (1.0 + (0.15 * (dist_km - 60.0)).exp());

    // Sigmoid for bulge/horizon
    // If bulge > 30m (approx horizon for low towers), prob drops
    // Center at 30m, steepness 0.5
    let bulge_prob = 1.0 / (1.0 + (0.5 * (bulge_m - 40.0)).exp());

    let combined_prob = dist_prob * bulge_prob;

    // Avoid log(0)
    if combined_prob < 1e-10 {
        return 1000.0; // Very high cost
    }

    -combined_prob.ln()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_haversine() {
        // Dist between London (51.5074, -0.1278) and Paris (48.8566, 2.3522) is ~344km
        let d = haversine_distance(51.5074, -0.1278, 48.8566, 2.3522);
        assert!((d - 344.0).abs() < 5.0);
    }

    #[test]
    fn test_earth_bulge() {
        // For 100km, bulge should be ~196m
        // h = (100000^2) / (8 * 6371000) = 10000000000 / 50968000 = 196.2
        let b = earth_bulge(100.0);
        assert!((b - 196.0).abs() < 1.0);
    }

    #[test]
    fn test_link_cost() {
        // Short distance -> Low cost
        let c_short = link_cost(51.50, 0.0, 51.51, 0.0, None); // ~1km
        // Long distance -> High cost
        let c_long = link_cost(51.50, 0.0, 52.50, 0.0, None); // ~111km

        assert!(c_short < c_long);

        // Very long -> Infinity (or very high)
        let c_very_long = link_cost(51.50, 0.0, 55.00, 0.0, None); // ~300km
        assert!(c_very_long == f64::INFINITY || c_very_long > 500.0);
    }
}
