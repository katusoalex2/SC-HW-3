#!/usr/bin/env python3
"""Preliminary level-flight AVL eigenmodes at a prescribed angle of attack.

The input file is the aircraft parameter file next to this script. The tail
aerodynamic centers are assumed to lie at quarter mean aerodynamic chord,
7 m behind a nose located at X=-3.651 m. Wing and tail leading edges are
assumed unswept; both tails are assumed to have zero vertical offset.
"""

import math
import re
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path


HERE = Path(__file__).resolve().parent
NUMBER = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[EeDd][-+]?\d+)?"


def read_parameters(path):
    text = path.read_text(encoding="utf-8-sig").replace(",", "")
    fields = {
        "mass": r"^Max Takeoff Weight.*?\s(" + NUMBER + r")\s*kg",
        "area": r"^Wing area\s+(" + NUMBER + r")",
        "span": r"^Wingspan.*?\s(" + NUMBER + r")\s*m",
        "mac": r"^Mean Aerodynamic Chord.*?\s(" + NUMBER + r")\s*m",
        "root": r"^Root Chord.*?\s(" + NUMBER + r")\s*m",
        "tip": r"^Tip Chord.*?\s(" + NUMBER + r")\s*m",
        "density": r"^Air Density.*?\s(" + NUMBER + r")\s*kg",
        "cd0": r"^Zero-Lift Drag.*?\s(" + NUMBER + r")",
        "dihedral": r"^Dihedral Angle.*?\s(" + NUMBER + r")\s*degrees",
        "ixx": r"\bIxx\s*=\s*(" + NUMBER + r")",
        "iyy": r"\bIyy\s*=\s*(" + NUMBER + r")",
        "izz": r"\bIzz\s*=\s*(" + NUMBER + r")",
        "ixz": r"\bIxz\s*=\s*(" + NUMBER + r")",
    }
    horizontal = re.search(r"Horizontal Tail:(.*?)Vertical Tail:", text, re.S)
    vertical = re.search(r"Vertical Tail:(.*?)Inertia Tensor:", text, re.S)
    if not horizontal or not vertical:
        raise ValueError("Missing horizontal or vertical tail section")
    values = {}
    for key, pattern in fields.items():
        match = re.search(pattern, text, re.I | re.M)
        if not match:
            raise ValueError(f"Missing or invalid parameter: {key}")
        values[key] = float(match.group(1).replace("D", "E"))
    for prefix, section in (("h", horizontal.group(1)), ("v", vertical.group(1))):
        for key, pattern in {
            "span": r"^.*(?:Tail Span|tail span).*?\s(" + NUMBER + r")",
            "root": r"^root\s+chord\s+(" + NUMBER + r")",
            "tip": r"^tip\s+cho+rd\s+(" + NUMBER + r")",
        }.items():
            match = re.search(pattern, section, re.I | re.M)
            if not match:
                raise ValueError(f"Missing or invalid {prefix} tail {key}")
            values[prefix + key] = float(match.group(1))
    if any(value <= 0 for key, value in values.items() if key != "ixz"):
        raise ValueError("Aircraft dimensions, mass, density and inertias must be positive")
    return values


def mac(root, tip):
    return 2.0 / 3.0 * (root * root + root * tip + tip * tip) / (root + tip)


def geometry(p):
    tail_ac = -3.651 + 7.0
    h_le = tail_ac - 0.25 * mac(p["hroot"], p["htip"])
    v_le = tail_ac - 0.25 * mac(p["vroot"], p["vtip"])
    semispan = p["span"] / 2
    z_tip = semispan * math.tan(math.radians(p["dihedral"]))
    mid = (p["root"] + p["tip"]) / 2
    return f"""UAM preliminary modes
0.0
0 0 0.0
{p['area']} {p['mac']} {p['span']}
0.4866 0 -0.053
{p['cd0']}

SURFACE
Wing
12 1.0 20 1.0
YDUPLICATE
0.0
SECTION
0 0 0 {p['root']} 0
AFILE
naca23012.dat
SECTION
0 {semispan / 2} {z_tip / 2} {mid} 0
AFILE
naca23012.dat
CONTROL
aileron 1.0 0.8 0 0 0 -1
SECTION
0 {semispan} {z_tip} {p['tip']} 0
AFILE
naca23012.dat
CONTROL
aileron 1.0 0.8 0 0 0 -1

SURFACE
Horizontal_tail
10 1.0 12 1.0
YDUPLICATE
0.0
SECTION
{h_le} 0 0 {p['hroot']} 0
CONTROL
elevator 1.0 0.7 0 0 0 1
SECTION
{h_le} {p['hspan'] / 2} 0 {p['htip']} 0
CONTROL
elevator 1.0 0.7 0 0 0 1

SURFACE
Vertical_tail
8 1.0 10 1.0
SECTION
{v_le} 0 0 {p['vroot']} 0
CONTROL
rudder 1.0 0.7 0 0 0 1
SECTION
{v_le} 0 {p['vspan']} {p['vtip']} 0
CONTROL
rudder 1.0 0.7 0 0 0 1
"""


def mass_file(p):
    return f"""Lunit = 1.0 m
Munit = 1.0 kg
Tunit = 1.0 s
g = 9.81
rho = {p['density']}
{p['mass']} 0.4866 0 -0.053 {p['ixx']} {p['iyy']} {p['izz']} 0 {p['ixz']} 0
"""


def value(output, label):
    matches = re.findall(r"(?<!\w)" + re.escape(label) + r"\s*=\s*(" + NUMBER + r")", output)
    if not matches:
        raise ValueError(f"AVL output missing {label}")
    return float(matches[-1].replace("D", "E"))


def identify_modes(output):
    pattern = (r"(?ms)^\s*mode\s+(\d+):\s*(" + NUMBER + r")\s+(" + NUMBER +
               r")\s*\n(.*?)(?=^\s*mode\s+\d+:|^\s*Run-case parameters|\Z)")
    modes = []
    for index, real, imaginary, vector in re.findall(pattern, output):
        rates = {}
        for name in ("p", "q", "r"):
            match = re.search(r"\b" + name + r"\s*:\s*(" + NUMBER + r")\s+(" + NUMBER + r")", vector)
            if not match:
                raise ValueError(f"Mode {index} missing eigenvector component {name}")
            rates[name] = abs(complex(float(match.group(1)), float(match.group(2))))
        q, lateral = rates["q"], rates["p"] + rates["r"]
        if q > 20 * lateral and q > 1e-6:
            family = "longitudinal"
        elif lateral > 20 * q and lateral > 1e-6:
            family = "lateral"
        else:
            family = "coupled"
        modes.append({"index": int(index), "root": complex(float(real), float(imaginary)),
                      "family": family, "name": "unclassified"})
    if len(modes) != 8:
        raise ValueError(f"Expected eight rigid-body eigenvalues; found {len(modes)}")

    longitudinal = [m for m in modes if m["family"] == "longitudinal" and m["root"].imag > 1e-5]
    lateral = [m for m in modes if m["family"] == "lateral" and m["root"].imag > 1e-5]
    real_lateral = [m for m in modes if m["family"] == "lateral" and abs(m["root"].imag) <= 1e-5]
    if len(longitudinal) == 2 and len(lateral) == 1 and len(real_lateral) == 2:
        longitudinal.sort(key=lambda mode: abs(mode["root"]))
        real_lateral.sort(key=lambda mode: abs(mode["root"]))
        for mode, name in ((longitudinal[0], "Phugoid"), (longitudinal[1], "Short period"),
                           (lateral[0], "Dutch roll"), (real_lateral[0], "Spiral"),
                           (real_lateral[1], "Roll")):
            mode["name"] = name
            if mode["root"].imag:
                partners = [other for other in modes if other is not mode and
                            abs(other["root"] - mode["root"].conjugate()) < 1e-4]
                if len(partners) == 1:
                    partners[0]["name"] = name
                else:
                    mode["name"] = "unclassified"
    return modes


def run():
    params = read_parameters(HERE / "our_reference_aircraft_parameters.txt")
    exe = HERE / "avl.exe"
    if not exe.is_file():
        raise FileNotFoundError(f"AVL executable not found: {exe}")
    airfoil = HERE / "naca23012.dat"
    if not airfoil.is_file():
        raise FileNotFoundError(f"Airfoil file not found: {airfoil}")
    commands = [
        "load model.avl", "mass model.mass", "mset 1",
        "plop", "g", "", "oper",
        "a a 2.5", "d2 pm 0", "x", "c1", "", "x",
        "c1", "", "x", "st", "", "", "mode", "n", "", "quit",
    ]
    with tempfile.TemporaryDirectory(prefix="avl_modes_") as folder:
        work = Path(folder)
        (work / "model.avl").write_text(geometry(params), encoding="ascii")
        (work / "model.mass").write_text(mass_file(params), encoding="ascii")
        shutil.copyfile(airfoil, work / "naca23012.dat")
        result = subprocess.run(
            [str(exe)], input="\n".join(commands) + "\n", cwd=work,
            capture_output=True, text=True, timeout=60, errors="replace"
        )
    output = result.stdout + result.stderr
    try:
        if result.returncode:
            raise RuntimeError(f"AVL exited with code {result.returncode}")
        if re.search(r"read error|not computed|not recognized|zero velocity", output, re.I):
            raise RuntimeError("AVL reported an error")
        alpha, cl, cm = (value(output, name) for name in ("Alpha", "CLtot", "Cmtot"))
        elevator = value(output, "elevator")
        velocities = re.findall(r"\bV\s+velocity\s*=\s*(" + NUMBER + r")", output)
        if not velocities:
            raise ValueError("AVL did not report a level-flight velocity")
        speed = float(velocities[-1].replace("D", "E"))
        if abs(alpha - 2.5) > 0.001 or abs(cm) > 0.001 or speed <= 0 or cl <= 0:
            raise ValueError("AVL did not produce the requested trimmed level-flight condition")
        lift_ratio = 0.5 * params["density"] * speed * speed * params["area"] * cl / (params["mass"] * 9.81)
        if abs(lift_ratio - 1) > 0.02:
            raise ValueError(f"Level-flight lift/weight check failed: {lift_ratio:.3f}")
        modes = identify_modes(output)
        print("PRELIMINARY LEVEL-FLIGHT ANALYSIS")
        print(f"Alpha: {alpha:.3f} deg  Speed: {speed:.3f} m/s  Elevator: {elevator:.3f} deg")
        print(f"CL: {cl:.5f}  Cm: {cm:.5f}  Lift/weight: {lift_ratio:.4f}")
        print("Eigenvalues (1/s); mode names are preliminary identifications:")
        for mode in modes:
            root = mode["root"]
            print(f"  {mode['index']:2d} {mode['name']:13s} {root.real:+.6g} {root.imag:+.6g}i")
        print("Assumptions: unswept, zero tail-height offsets, tail AC at quarter MAC.")
        wing_area = params["span"] * (params["root"] + params["tip"]) / 2
        print(f"Wing geometric area: {wing_area:.3f} m^2; reference area: {params['area']:.3f} m^2.")
        print("AVL is a preliminary model; it does not include propulsion or a full drag model.")
    except (ValueError, RuntimeError):
        print("AVL output follows for debugging:\n" + output, file=sys.stderr)
        raise


if __name__ == "__main__":
    try:
        run()
    except (OSError, ValueError, RuntimeError, subprocess.TimeoutExpired) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        sys.exit(1)
