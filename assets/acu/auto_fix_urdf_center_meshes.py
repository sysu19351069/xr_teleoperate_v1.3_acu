#!/usr/bin/env python3
"""
Auto-fix URDF visual origins by centering meshes.
- Backup assets/acu/acu.urdf -> acu.urdf.bak
- For each <visual><geometry><mesh filename="..."/>, compute mesh centroid (using assets/acu/meshes/*)
  and set visual/origin xyz to the negative centroid so the mesh is centered at the link frame.

Dependencies: trimesh, lxml, numpy
Install: pip install trimesh lxml numpy

Run from repository root:
  python3 assets/acu/auto_fix_urdf_center_meshes.py
"""
import os
import shutil
import sys
from lxml import etree
import trimesh
import numpy as np

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
URDF_PATH = os.path.join(REPO_ROOT, 'assets', 'acu', 'acu.urdf')
MESH_DIR = os.path.join(REPO_ROOT, 'assets', 'acu', 'meshes')
BACKUP_PATH = URDF_PATH + '.bak'

if not os.path.exists(URDF_PATH):
    print(f"URDF not found: {URDF_PATH}")
    sys.exit(1)

print(f"Backing up {URDF_PATH} -> {BACKUP_PATH}")
shutil.copy2(URDF_PATH, BACKUP_PATH)

parser = etree.XMLParser(remove_blank_text=True)
tree = etree.parse(URDF_PATH, parser)
root = tree.getroot()

# helper to resolve mesh filename to local file in MESH_DIR
def resolve_mesh_file(fn):
    if fn is None:
        return None
    # strip package:// or file:// if present
    base = fn
    if fn.startswith('package://'):
        base = os.path.basename(fn)
    if fn.startswith('file://'):
        base = fn.replace('file://','')
    # try direct join
    candidates = [
        os.path.join(MESH_DIR, base),
        os.path.join(MESH_DIR, base.replace('\\','/')),
        os.path.join(os.path.dirname(URDF_PATH), base),
        os.path.join(os.path.dirname(URDF_PATH), 'meshes', base),
    ]
    for c in candidates:
        if os.path.exists(c):
            return c
    return None

patched = 0
for visual in root.findall('.//visual'):
    geom = visual.find('geometry')
    if geom is None:
        continue
    mesh = geom.find('mesh')
    if mesh is None:
        continue
    # filename might be attribute or child element
    fn = mesh.get('filename')
    if fn is None:
        fn_elem = mesh.find('filename')
        if fn_elem is not None:
            fn = fn_elem.text
    if not fn:
        continue
    mesh_path = resolve_mesh_file(fn)
    if mesh_path is None:
        print(f"WARN: cannot resolve mesh file for '{fn}', skipping visual")
        continue
    try:
        m = trimesh.load(mesh_path, force='mesh')
    except Exception as e:
        print(f"WARN: failed to load mesh {mesh_path}: {e}")
        continue
    centroid = np.array(m.centroid, dtype=float)
    # if centroid is close to zero, no need to patch
    if np.linalg.norm(centroid) < 1e-6:
        # ensure rpy attribute exists
        origin = visual.find('origin')
        if origin is None:
            origin = etree.SubElement(visual, 'origin')
            origin.set('xyz', '0 0 0')
            origin.set('rpy', '0 0 0')
        patched += 0
        continue
    # set origin to negative centroid (round to 6 decimals)
    nx, ny, nz = (-centroid).round(6)
    origin = visual.find('origin')
    if origin is None:
        origin = etree.SubElement(visual, 'origin')
    origin.set('xyz', f"{nx} {ny} {nz}")
    if origin.get('rpy') is None:
        origin.set('rpy', '0 0 0')
    print(f"Patched visual origin for mesh '{os.path.basename(mesh_path)}' -> xyz='{nx} {ny} {nz}' (centroid was {centroid})")
    patched += 1

if patched > 0:
    tree.write(URDF_PATH, pretty_print=True, xml_declaration=True, encoding='utf-8')
    print(f"Patched {patched} visuals in {URDF_PATH}; original backed up at {BACKUP_PATH}")
else:
    print("No visuals needed patching (all centroids near zero). Restoring backup.")
    # restore backup
    shutil.copy2(BACKUP_PATH, URDF_PATH)

print('Done.')
