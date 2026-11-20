#!/usr/bin/env python3
"""
download_fuel_models.py

Parses SDF simulation world files for Open Robotics / Gazebo Fuel model dependencies,
downloads and extracts missing models locally into ~/subt_models/<model_name>,
recursively resolves nested dependencies inside downloaded model.sdf files,
localizes both the scenario SDF and all downloaded model.sdf files for CoppeliaSim.

Sources:
- Open Robotics Gazebo Fuel REST API: https://gazebosim.org/docs/all/fuel_tools
- Python XML ElementTree: https://docs.python.org/3/library/xml.etree.elementtree.html
"""

import argparse
import io
import json
import os
import re
import sys
import time
import urllib.error
import urllib.parse
import urllib.request
import xml.etree.ElementTree as ET
import zipfile
from dataclasses import dataclass, field
from pathlib import Path

# Matches file-level references: .../models/<model>/tip/files/<path>
FUEL_FILE_REGEX = re.compile(
    r"https?://fuel\.(?:ignitionrobotics|gazebosim)\.org/\d+\.\d+/(?P<owner>[^/<>]+)/models/(?P<model>[^/<>]+)/tip/files/(?P<relpath>[^<>\s\"']+)"
)

# Matches model-level references: .../models/<model>
FUEL_URI_REGEX = re.compile(
    r"https?://fuel\.(?:ignitionrobotics|gazebosim)\.org/\d+\.\d+/(?P<owner>[^/<>]+)/models/(?P<model>[^/<>]+)/?"
)

USER_AGENT = "Mozilla/5.0 (X11; Linux x86_64; rv:128.0) Gecko/20100101 Firefox/128.0"


@dataclass
class ModelDependency:
    name: str
    owner: str
    original_uri: str


@dataclass
class ScenarioResult:
    sdf_path: Path
    local_sdf_path: Path | None = None
    models_found: list[ModelDependency] = field(default_factory=list)
    downloaded: list[str] = field(default_factory=list)
    cached: list[str] = field(default_factory=list)
    failed: list[tuple[str, str]] = field(default_factory=list)
    validated: list[str] = field(default_factory=list)
    missing_mesh_warnings: list[str] = field(default_factory=list)
    localized_models_count: int = 0


def find_fuel_dependencies_in_text(content: str) -> list[ModelDependency]:
    """Finds all unique Fuel model dependencies in XML content (world or model.sdf)."""
    dependencies: dict[str, ModelDependency] = {}

    for match in FUEL_FILE_REGEX.finditer(content):
        model_name = match.group("model")
        owner = match.group("owner")
        if model_name not in dependencies:
            dependencies[model_name] = ModelDependency(
                name=model_name,
                owner=owner,
                original_uri=match.group(0),
            )

    for match in FUEL_URI_REGEX.finditer(content):
        model_name = match.group("model")
        owner = match.group("owner")
        if model_name not in dependencies:
            dependencies[model_name] = ModelDependency(
                name=model_name,
                owner=owner,
                original_uri=match.group(0),
            )

    return list(dependencies.values())


def parse_fuel_dependencies(sdf_path: Path) -> list[ModelDependency]:
    """Parses an SDF file to discover all direct Fuel model dependencies."""
    content = sdf_path.read_text(encoding="utf-8")
    return find_fuel_dependencies_in_text(content)


def is_model_cached(target_dir: Path) -> bool:
    """Idempotency check: returns True if directory exists, contains files, and has model.sdf."""
    if not target_dir.is_dir():
        return False
    try:
        has_files = any(target_dir.iterdir())
        return has_files and (target_dir / "model.sdf").exists()
    except OSError:
        return False


def _flatten_file_tree(nodes: list[dict], base_path: str = "") -> list[str]:
    """Recursively flattens the Fuel file_tree JSON structure into relative file paths."""
    files: list[str] = []
    for node in nodes:
        name = node.get("name", "")
        rel_path = f"{base_path}/{name}" if base_path else name
        children = node.get("children")
        if children:
            files.extend(_flatten_file_tree(children, rel_path))
        else:
            files.append(rel_path)
    return files


def _download_via_file_tree(
    dep: ModelDependency,
    target_dir: Path,
    timeout: int = 60,
) -> tuple[bool, str]:
    """
    Fallback mechanism: Queries Fuel's /tip/files REST API to discover and download
    all model files individually when ZIP archive generation fails or times out.
    """
    encoded_owner = urllib.parse.quote(dep.owner)
    encoded_model = urllib.parse.quote(dep.name)
    tree_url = (
        f"https://fuel.ignitionrobotics.org/1.0/{encoded_owner}/models/{encoded_model}/tip/files"
    )

    req = urllib.request.Request(tree_url, headers={"User-Agent": USER_AGENT, "Accept": "*/*"})
    try:
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            data = json.loads(resp.read().decode("utf-8"))
    except Exception as e:
        return False, f"Failed to retrieve file tree: {e}"

    file_list = _flatten_file_tree(data.get("file_tree", []))
    if not file_list:
        return False, "File tree empty"

    resolved_target = target_dir.resolve()
    for rel_path in file_list:
        dest_file = (target_dir / rel_path).resolve()
        if not str(dest_file).startswith(str(resolved_target)):
            return False, f"Unsafe file path detected: {rel_path}"

        dest_file.parent.mkdir(parents=True, exist_ok=True)
        file_url = (
            f"https://fuel.ignitionrobotics.org/1.0/{encoded_owner}/models/{encoded_model}/tip/files/{rel_path}"
        )
        file_req = urllib.request.Request(file_url, headers={"User-Agent": USER_AGENT})

        downloaded = False
        last_err = ""
        for attempt in range(3):
            try:
                with urllib.request.urlopen(file_req, timeout=timeout) as f_resp:
                    dest_file.write_bytes(f_resp.read())
                downloaded = True
                break
            except Exception as e:
                last_err = str(e)
                time.sleep(1.0)

        if not downloaded:
            return False, f"Failed to download {rel_path}: {last_err}"

    return True, "downloaded"


def download_and_extract_model(
    dep: ModelDependency,
    models_dir: Path,
    timeout: int = 60,
) -> tuple[bool, str]:
    """
    Downloads model from Gazebo/Ignition Fuel and extracts it directly into models_dir / dep.name.
    Tries ZIP endpoints first, then falls back to individual file tree downloads.
    Returns (success, message).
    """
    target_dir = models_dir / dep.name
    if is_model_cached(target_dir):
        return True, "already_cached"

    target_dir.mkdir(parents=True, exist_ok=True)

    encoded_owner = urllib.parse.quote(dep.owner)
    encoded_model = urllib.parse.quote(dep.name)

    # 1. Try ZIP from Gazebo Fuel and Ignition Fuel
    zip_urls = [
        f"https://fuel.gazebosim.org/1.0/{encoded_owner}/models/{encoded_model}.zip",
        f"https://fuel.ignitionrobotics.org/1.0/{encoded_owner}/models/{encoded_model}.zip",
    ]

    for zip_url in zip_urls:
        req = urllib.request.Request(
            zip_url,
            headers={"User-Agent": USER_AGENT, "Accept": "*/*"},
        )
        try:
            with urllib.request.urlopen(req, timeout=timeout) as resp:
                data = resp.read()

            if data[:4] == b"PK\x03\x04":
                with zipfile.ZipFile(io.BytesIO(data)) as zf:
                    resolved_target = target_dir.resolve()
                    for member in zf.infolist():
                        member_path = (target_dir / member.filename).resolve()
                        if not str(member_path).startswith(str(resolved_target)):
                            raise RuntimeError(f"Unsafe path detected in zip: {member.filename}")
                    zf.extractall(target_dir)
                return True, "downloaded"
        except Exception:
            continue

    # 2. Fallback to /tip/files individual downloads
    success, msg = _download_via_file_tree(dep, target_dir, timeout=timeout)
    if success:
        return True, "downloaded"

    if target_dir.is_dir() and not any(target_dir.iterdir()):
        try:
            target_dir.rmdir()
        except OSError:
            pass

    return False, msg


def validate_extracted_model(target_dir: Path) -> tuple[bool, bool, str]:
    """
    Validates model directory contents.
    Returns: (is_valid, has_meshes, message)
    """
    model_sdf = target_dir / "model.sdf"
    if not model_sdf.exists() or model_sdf.stat().st_size == 0:
        return False, False, f"Missing or empty model.sdf in {target_dir}"

    meshes_dir = target_dir / "meshes"
    has_meshes = False
    if meshes_dir.is_dir():
        mesh_files = [
            f for f in meshes_dir.rglob("*")
            if f.suffix.lower() in {".dae", ".obj", ".stl"}
        ]
        has_meshes = len(mesh_files) > 0

    return True, has_meshes, "OK"


def localize_sdf_content(content: str, models_dir: Path) -> str:
    """
    Replaces remote Fuel URIs (both models and files/textures) with absolute local paths.
    """
    abs_models_dir = str(models_dir.resolve())

    # 1. Replace file-level URLs (e.g., textures in materials)
    def _replace_file(match: re.Match) -> str:
        model = match.group("model")
        relpath = match.group("relpath")
        return f"{abs_models_dir}/{model}/{relpath}"

    content = FUEL_FILE_REGEX.sub(_replace_file, content)

    # 2. Replace model-level URLs (e.g., <include><uri>...)
    def _replace_model(match: re.Match) -> str:
        model = match.group("model")
        return f"{abs_models_dir}/{model}"

    content = FUEL_URI_REGEX.sub(_replace_model, content)
    return content


def localize_downloaded_models(models_dir: Path) -> int:
    """
    Scans all downloaded model.sdf files in models_dir and rewrites:
    1. Embedded remote Fuel URIs (nested includes or textures) with absolute local paths.
    2. Relative mesh and material URIs (<uri>meshes/... -> <uri>/abs/path/meshes/...)
       to prevent CoppeliaSim from resolving meshes relative to the world file directory.
    3. model:// URIs to absolute filesystem paths.
    Returns number of models modified.
    """
    model_map: dict[str, Path] = {}
    for d in models_dir.iterdir():
        if d.is_dir():
            model_map[d.name.lower()] = d
            model_map[d.name.lower().replace(" ", "_")] = d
            model_map[d.name.lower().replace("_", " ")] = d
            model_map[d.name] = d

    # Handle known Fuel model typos (e.g. 'clibming rope' in Climbing Rope/model.sdf)
    model_map["clibming rope"] = models_dir / "Climbing Rope"
    model_map["clibming_rope"] = models_dir / "Climbing Rope"

    updated = 0
    for model_dir in sorted(models_dir.iterdir()):
        model_sdf = model_dir / "model.sdf"
        if not model_sdf.is_file():
            continue

        content = model_sdf.read_text(encoding="utf-8")
        original_content = content
        abs_model_dir = str(model_dir.resolve())

        # 1. Localize Fuel remote URLs (both /models/ and /tip/files/)
        content = localize_sdf_content(content, models_dir)

        # 2. Localize relative mesh and material URIs
        content = re.sub(
            r"<uri>(?!/|https?://|model://)(meshes/[^<]+)</uri>",
            rf"<uri>{abs_model_dir}/\1</uri>",
            content,
        )
        content = re.sub(
            r"<uri>(?!/|https?://|model://)(materials/[^<]+)</uri>",
            rf"<uri>{abs_model_dir}/\1</uri>",
            content,
        )

        # 3. Localize model://<model>/<path> protocols
        def _replace_model_proto(match: re.Match) -> str:
            m_name = match.group("model").strip()
            rel = match.group("relpath").strip()
            tag = match.group("tag")
            target = model_map.get(m_name.lower()) or model_map.get(m_name)
            if target and target.is_dir():
                return f"<{tag}>{target.resolve()}/{rel}</{tag}>"
            return match.group(0)

        content = re.sub(
            r"<(?P<tag>uri|[a-z_]+_map)>model://(?P<model>[^/]+)/(?P<relpath>[^<]+)</(?P=tag)>",
            _replace_model_proto,
            content,
        )

        # 4. Sanitize model for CoppeliaSim: consolidate submeshes, remove unsupported lights,
        # and ensure empty links have marker shapes (fixes 'could not set parameter').
        content = _sanitize_model_for_coppelia(content)

        if content != original_content:
            model_sdf.write_text(content, encoding="utf-8")
            updated += 1

    return updated


def _sanitize_model_for_coppelia(content: str) -> str:
    """
    Sanitizes model SDF content for CoppeliaSim's simExtSDF importer:
    1. Consolidates multiple <visual> elements referencing the same .dae with <submesh>
       into a single <visual> without <submesh> (fixes 'submesh loading is not supported').
    2. Strips <light> elements and removes empty light-only links (fixes 'could not set parameter').
    3. Adds a small marker box to empty links without visual or collision geometry (e.g. Fiducial),
       preventing property setter failures on non-existent shape handles.
    """
    try:
        parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
        tree = ET.fromstring(content, parser=parser)
        modified = False

        # 1. Consolidate submeshes
        for link in tree.findall(".//link"):
            visuals = link.findall("visual")
            submesh_visuals = [v for v in visuals if v.find(".//submesh") is not None]
            if submesh_visuals:
                first_uri_elem = submesh_visuals[0].find(".//mesh/uri")
                if first_uri_elem is not None and first_uri_elem.text:
                    first_uri = first_uri_elem.text.strip()
                    for v in submesh_visuals:
                        link.remove(v)
                    new_v = ET.Element("visual", {"name": "visual"})
                    geom = ET.SubElement(new_v, "geometry")
                    mesh = ET.SubElement(geom, "mesh")
                    uri = ET.SubElement(mesh, "uri")
                    uri.text = first_uri
                    link.append(new_v)
                    modified = True

        # 2. Strip lights & 3. Ensure non-empty links
        for model in tree.findall(".//model"):
            for link in list(model.findall("link")):
                lights = link.findall("light")
                if lights:
                    for l in lights:
                        link.remove(l)
                    modified = True

                has_geom = bool(link.findall("visual") or link.findall("collision"))
                if not has_geom:
                    if "light" in link.attrib.get("name", "").lower():
                        model.remove(link)
                        modified = True
                    else:
                        col = ET.SubElement(link, "collision", {"name": "marker_collision"})
                        geom1 = ET.SubElement(col, "geometry")
                        box1 = ET.SubElement(geom1, "box")
                        size1 = ET.SubElement(box1, "size")
                        size1.text = "0.1 0.1 0.1"

                        vis = ET.SubElement(link, "visual", {"name": "marker_visual"})
                        geom2 = ET.SubElement(vis, "geometry")
                        box2 = ET.SubElement(geom2, "box")
                        size2 = ET.SubElement(box2, "size")
                        size2.text = "0.1 0.1 0.1"
                        modified = True

        if modified:
            ET.indent(tree, space="  ")
            res = ET.tostring(tree, encoding="unicode")
            if not res.startswith("<?xml"):
                res = '<?xml version="1.0"?>\n' + res
            return res
    except Exception:
        pass

    return content


def generate_local_sdf(sdf_path: Path, models_dir: Path, as_model: bool = True) -> Path:
    """
    Generates <scenario>_local.sdf by replacing Fuel remote URLs with absolute local paths.
    Leaves original SDF intact and preserves original XML comments and formatting.
    When as_model=True (default), converts <world> into a composite <model> compatible with
    CoppeliaSim's simExtSDF importer, which does not support importing <world> elements.
    """
    content = sdf_path.read_text(encoding="utf-8")
    new_content = localize_sdf_content(content, models_dir)

    if as_model and "<world" in new_content:
        # Convert <world name="..."> into <model name="..."> with <static>true</static>
        new_content = re.sub(
            r'<world\s+name="([^"]+)">',
            r'<model name="\1">\n    <static>true</static>',
            new_content,
        )
        new_content = re.sub(r"</world>", r"</model>", new_content)
        # Strip world-only simulation tags not valid inside <model>
        new_content = re.sub(r"\s*<physics[^>]*>.*?</physics>", "", new_content, flags=re.DOTALL)
        new_content = re.sub(r"\s*<scene[^>]*>.*?</scene>", "", new_content, flags=re.DOTALL)
        new_content = re.sub(r"\s*<atmosphere[^>]*>.*?</atmosphere>", "", new_content, flags=re.DOTALL)

    dest_path = sdf_path.parent / f"{sdf_path.stem}_local.sdf"
    dest_path.write_text(new_content, encoding="utf-8")
    return dest_path


def process_scenario(sdf_path: Path, models_dir: Path) -> ScenarioResult:
    """Processes a single scenario SDF file, resolving recursive dependencies."""
    result = ScenarioResult(sdf_path=sdf_path)

    if not sdf_path.exists():
        result.failed.append((sdf_path.name, "File not found"))
        return result

    # 1. Recursive Dependency Discovery & Download
    queue = parse_fuel_dependencies(sdf_path)
    visited: set[str] = set()

    while queue:
        dep = queue.pop(0)
        if dep.name in visited:
            continue
        visited.add(dep.name)
        result.models_found.append(dep)

        success, status = download_and_extract_model(dep, models_dir)
        if success:
            if status == "already_cached":
                result.cached.append(dep.name)
            else:
                result.downloaded.append(dep.name)

            # Inspect downloaded model.sdf for nested dependencies
            model_sdf = models_dir / dep.name / "model.sdf"
            if model_sdf.exists():
                model_text = model_sdf.read_text(encoding="utf-8")
                nested_deps = find_fuel_dependencies_in_text(model_text)
                for ndep in nested_deps:
                    if ndep.name not in visited:
                        queue.append(ndep)
        else:
            result.failed.append((dep.name, status))

    # 2. Localize all downloaded model.sdf files (rewriting nested includes & textures)
    result.localized_models_count = localize_downloaded_models(models_dir)

    # 3. Validation
    for dep in result.models_found:
        target_dir = models_dir / dep.name
        if target_dir.is_dir():
            valid, has_meshes, msg = validate_extracted_model(target_dir)
            if valid:
                result.validated.append(dep.name)
                if not has_meshes:
                    result.missing_mesh_warnings.append(dep.name)
            else:
                result.failed.append((dep.name, msg))

    # 4. Generate Local Scenario SDF
    result.local_sdf_path = generate_local_sdf(sdf_path, models_dir)
    return result


def print_summary(result: ScenarioResult, models_dir: Path) -> None:
    """Prints a clear summary for the processed scenario."""
    print("=" * 70)
    print(f"Scenario: {result.sdf_path}")
    print(f"Models Directory: {models_dir.resolve()}")
    print("-" * 70)
    print(f"Total Unique Models Found : {len(result.models_found)}")
    print(f"Already Cached (Skipped)  : {len(result.cached)}")
    print(f"Downloaded Successfully   : {len(result.downloaded)}")
    print(f"Validated Successfully    : {len(result.validated)}")
    print(f"Nested Models Localized   : {result.localized_models_count}")
    print(f"Failed Downloads/Models   : {len(result.failed)}")

    if result.missing_mesh_warnings:
        print(f"\nNote: {len(result.missing_mesh_warnings)} model(s) have no mesh directory (geometric/spatial links):")
        for m in sorted(result.missing_mesh_warnings):
            print(f"  - {m}")

    if result.failed:
        print("\nFailures:")
        for name, err in result.failed:
            print(f"  - {name}: {err}")

    if result.local_sdf_path:
        print(f"\nGenerated Local SDF File  : {result.local_sdf_path.resolve()}")
    print("=" * 70)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Download Open Robotics Fuel models and localize SDF files for CoppeliaSim."
    )
    parser.add_argument(
        "sdf_files",
        nargs="*",
        help="Path(s) to SDF world files. If omitted, all scenarios (*/*.sdf) in the repository are processed.",
    )
    parser.add_argument(
        "--all",
        action="store_true",
        help="Explicitly process all scenarios in subdirectories (default when no files are specified).",
    )
    parser.add_argument(
        "--models-dir",
        default="~/subt_models",
        help="Directory to store extracted models (default: ~/subt_models).",
    )

    args = parser.parse_args()
    models_dir = Path(args.models_dir).expanduser()
    models_dir.mkdir(parents=True, exist_ok=True)

    if args.sdf_files and not args.all:
        sdf_files = [Path(p) for p in args.sdf_files]
    else:
        # Default behavior: process all scenarios in subdirectories
        sdf_files = sorted(Path(".").glob("*/*.sdf"))
        sdf_files = [p for p in sdf_files if not p.stem.endswith("_local")]
        if not sdf_files:
            print("Error: No *.sdf files found in subdirectories.", file=sys.stderr)
            return 1

    total_failures = 0
    for sdf_path in sdf_files:
        print(f"\nProcessing {sdf_path}...")
        res = process_scenario(sdf_path, models_dir)
        print_summary(res, models_dir)
        if res.failed:
            total_failures += len(res.failed)

    return 0 if total_failures == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
