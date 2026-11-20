# SubT Models Downloader & CoppeliaSim SDF Localizer

A Python utility for downloading Open Robotics / Gazebo Fuel simulation models referenced in procedural cave worlds and localizing their SDF descriptors for direct import into [CoppeliaSim](https://www.coppeliarobotics.com/).

---

## Background & Problem

Procedural SDF simulation worlds often reference remote 3D assets via Open Robotics Fuel URLs:
```xml
<include>
  <uri>https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/Cave Starting Area Type B</uri>
</include>
```
The CoppeliaSim SDF importer does not resolve remote HTTP URLs dynamically. It requires local absolute filesystem paths to directories containing `model.sdf` and associated mesh geometry (`meshes/*.dae`, `.obj`, or `.stl`).

[`download_fuel_models.py`](download_fuel_models.py) automates this pipeline:
1. **Parses dependencies recursively**: Scans both the world scenario SDF and nested `<include><uri>` elements inside downloaded models' `model.sdf` files (e.g. composite "Lights" models, detector plugins, and shared texture packages like `Cave Straight Type A`).
2. **Downloads and unpacks** assets to `~/subt_models/<model_name>/` using an idempotent multi-tier engine.
3. **Localizes and Converts to Composite `<model>`**: Rewrites remote Fuel URIs to absolute local paths, and converts the top-level `<world>` wrapper into a composite `<model>` with `<static>true</static>` (stripping simulator-only `<physics>`, `<scene>`, and `<atmosphere>` tags). This directly satisfies CoppeliaSim's `simExtSDF` importer, which only imports `<model>` elements.

---

## Prerequisites

- **Python 3.10+**
- **uv** (recommended runner; installs without external dependencies)

---

## Usage

### 1. Process All Scenarios (Default Behavior)
Running the script with no arguments automatically discovers and processes all scenarios (`*/*.sdf`) in the repository, downloading any missing models and generating localized `*_local.sdf` copies:
```bash
uv run python3 download_fuel_models.py
```

### 2. Process Specific Scenario(s)
You can optionally restrict execution to one or more specific scenarios:
```bash
uv run python3 download_fuel_models.py "Cave Simple 03/simple_cave_03.sdf"
```
Or multiple scenarios:
```bash
uv run python3 download_fuel_models.py "Cave Simple 01/simple_cave_01.sdf" "Cave Simple 02/simple_cave_02.sdf"
```

### 3. Custom Output Directory
By default, models are extracted to `~/subt_models`. You can override this location:
```bash
uv run python3 download_fuel_models.py --models-dir "/path/to/custom_models"
```

---

## Features

- **Idempotency**: Checks if `~/subt_models/<model_name>` already contains a valid `model.sdf`. Existing assets are skipped immediately with zero redundant network requests.
- **Resilient Multi-Tier Fetch Engine**:
  1. *Tier 1*: Direct ZIP archive from `fuel.gazebosim.org`.
  2. *Tier 2*: Direct ZIP archive from `fuel.ignitionrobotics.org`.
  3. *Tier 3 (Automated Fallback)*: When Fuel returns server errors (e.g. `503 Service Unavailable` or `500 Unable to save resource`), the script queries Fuel's `/tip/files` REST endpoint to recursively download individual mesh and configuration files directly.
- **Lossless SDF Rewriting**: Generates `<scenario>_local.sdf` without modifying the original `.sdf` file, maintaining original XML formatting and generator comments intact.
- **Nested Model & Texture Localization**: Automatically rewrites remote Fuel URLs inside downloaded `~/subt_models/<model>/model.sdf` files (including nested `<include><uri>` references in composite models like `* Lights Type B` and texture map links to `Cave Straight Type A`), preventing CoppeliaSim Error Code 14.
- **Submesh Consolidation**: Merges fragmented `<submesh>` visual elements across all cave models into single unified visual blocks referencing the underlying `.dae` mesh geometry, bypassing CoppeliaSim's lack of submesh loading support.
- **Validation & Reporting**: Verifies `model.sdf` and mesh presence, distinguishing between mesh-based tiles and pure geometric/spatial anchors (such as `Fiducial`).

---

## CoppeliaSim Import Guide

1. Open **CoppeliaSim**.
2. Navigate to **Modules** > **Geometry / Mesh** > **Import SDF...** (or your preferred SDF import plugin).
3. Select the generated `*_local.sdf` file (e.g., `Cave Simple 03/simple_cave_03_local.sdf`).
4. CoppeliaSim will load the scene and resolve all meshes directly from your local `~/subt_models/` folder.

---

## Sources & References

- [Gazebo Fuel Tools API Documentation](https://gazebosim.org/docs/all/fuel_tools)
- [Python `xml.etree.ElementTree` Documentation](https://docs.python.org/3/library/xml.etree.elementtree.html)
- [DARPA Subterranean (SubT) Challenge Simulation](https://www.darpa.mil/program/subterranean-challenge)
- [Open Robotics SubT Tech Repo Collection](https://subtchallenge.gazebosim.org/openrobotics/fuel/collections/SubT%20Tech%20Repo)
  - [SubT World Builder](https://subtchallenge.gazebosim.org/world-builder)

---

## License & Attribution

- The scripts and documentation in this repository ([`download_fuel_models.py`](download_fuel_models.py), [`README.md`](README.md)) are licensed under the [MIT License](LICENSE).
- The DARPA SubT Challenge cave world files (`Cave Qualification/`, `Cave Simple 01/`–`03/`) originate from Open Robotics / OSRF ([osrf/subt](https://github.com/osrf/subt)) and remain subject to their respective upstream Open Robotics licenses (Apache-2.0 / CC-BY).
