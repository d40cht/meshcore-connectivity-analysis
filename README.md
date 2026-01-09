# Meshcore Connectivity Mapper (Viterbi-Based)

## 1. Overview: What is Meshcore?
**Meshcore** is a parallel LoRa-based networking protocol designed for long-range, low-bandwidth communication. Unlike standard flood-mesh networks, Meshcore utilizes a hybrid routing strategy:
* **Static Dedicated Repeaters:** The backbone consists of fixed nodes that handle high-volume traffic.
* **Hybrid Routing:** Routes are discovered via flood broadcast, but subsequent packets are **source-routed** to save bandwidth.
* **Massive Constraint (The 64-Byte Header):** To minimize overhead, the routing path is stored in a dedicated 64-byte field. Crucially, each hop is represented only by the **first byte (prefix)** of the Repeater Node ID.

### The Problem: ID Aliasing & The Birthday Problem
Because nodes are represented by a single byte (0-255), collisions are inevitable as the network scales. In a country-wide deployment, dozens of repeaters may share the same `0xAF` prefix. Simply looking up an ID is insufficient to determine the path; we must use **spatial and electromagnetic plausibility** to de-alias the route.

---

## 2. Using Viterbi for Probabilistic Mapping
To resolve the true path of a packet, we treat the routing header as a sequence of observations in a **Hidden Markov Model (HMM)**.

* **States:** All known physical repeaters in the database.
* **Observations:** The 1-byte prefixes found in the packet header.
* **Emission Probability:** Binary. A repeater "emits" its 1-byte prefix with $P=1$ if it matches, and $0$ otherwise.
* **Transition Probability:** Calculated based on the physical likelihood of a LoRa link between Node A and Node B, using distance, Earth curvature (the "bulge"), and eventually terrain (SRTM).

By finding the **Viterbi Path** (the sequence of nodes that minimizes total path cost), we can reliably identify which physical hardware facilitated a message, even when ID collisions are present. Over many messages, the "ghost" paths (incorrect ID matches) wash out as noise, while true physical infrastructure emerges as high-confidence edges.

---

## 3. Five-Step Implementation Plan

### Step 1: Baseline Analysis (Geometric/Horizon)
**Goal:** Build the core Viterbi engine using only coordinates and basic physics.
* **Inputs:** 1. `repeaters.csv` (ID, Name, Lat, Lon).
    2. `packets.csv` (Start Lat/Lon, End Lat/Lon, 64-byte Header).
* **Logic:** Implement a "Horizon-Dropoff" model. 
    * Calculate distance $d$ between candidates.
    * Prune links where $d > 100km$.
    * Calculate "Earth Bulge" height: $h \approx d^2 / 8R$. If the bulge exceeds a theoretical antenna height (e.g., 20m), apply a heavy penalty.
* **Output:** A list of the most likely physical nodes for each packet based on geometry alone.

### Step 2: Shadow Network Discovery (Ghost Nodes)
**Goal:** Identify repeaters that exist in the real world but aren't in the known database.
* **Logic:** For every hop in the Viterbi trellis, inject a "Ghost" candidate for that specific prefix.
* **Costing:** Give Ghost nodes a "Fixed Uncertainty Penalty" (e.g., equivalent to a -122dBm link).
* **Selection:** If the Viterbi algorithm chooses a Ghost over all known repeaters (because the known ones are topographically blocked or too far), log this as a "Hidden Node Hit."
* **Aggregation:** Collect all Ghost hits. If a specific prefix consistently appears between two known nodes across multiple messages, we have discovered a likely physical repeater.

### Step 3.1: Topographic Refinement (Random data)
**Goal:** Use terrain data to penalize "impossible" links through mountains or ridges.
* **Test logic** for a given simulation region, generate plausible test pink noise to simulate terrain height at 10m resolution. Don't make it so high that it blocks too much transmission (run some checks with and without the elevation to make sure we're only losing max 50% of links).
* **Algorithm:** For each edge in the adjacency matrix, sample the elevation at points along the path (Bresenham-style line). 
* **Penalty:** If any terrain point obstructs the line of sight (or the 1st Fresnel zone), increase the edge cost significantly (e.g., a +30dB path loss penalty).
* **Refinement:** Re-run the Viterbi analysis on the packet set with these terrain-aware costs.

### Step 3.2: Topographic Refinement (SRTM Integration)
**Goal:** Use real terrain data to penalize "impossible" links through mountains or ridges.
* **Logic:** Integrate SRTM (30m) `.hgt` tile parsing.
* **Setup** a main function that loads node locations and messages from CSV files, and terrain from SRTM, pre-calculates edge costs/probabilities for each node (sparse, within a plausible range only) and runs the viterbi for each message to dump a full reconstructed path to disk for each message in a CSV file. 

### Step 3.3: Efficiency
**Goal**: come up with a method/heuristic that means we do not have to calculate the RSSI between every node in the graph. E.g. find some quick way to generate the conservative coverage polygon for a given node such that we can (with a spatial index) only pull in the plausibly reachable adjacent nodes. And then calcualte and maintain a sparse edge graph.

### Step 4: Clustering & Promotion (Map Generation)
**Goal:** Finalize the connectivity map and estimate missing node locations.
* **Logic:** Use **DBSCAN clustering** on the estimated midpoints of Ghost node observations.
* **Triangulation:** For each cluster, find the Lat/Lon that minimizes path loss (Maximum Likelihood Estimation) for all packets that traversed that Ghost.
* **Final Output:** Generate a **GraphML/GeoJSON map** of the network, showing:
    1. Confirmed links between known nodes (weighted by frequency).
    2. Estimated locations of "Discovered" repeaters.
    3. Confidence scores for each edge based on total message volume.
