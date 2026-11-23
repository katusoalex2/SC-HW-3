#!/usr/bin/env python3
"""
AVL Analysis Runner - Automated aircraft aerodynamic analysis
Reads aircraft parameters, generates AVL input files, executes AVL,
and extracts/displays trim, stability, and dynamic results.
"""

import subprocess
import os
import sys
import re
from pathlib import Path
from typing import Dict, Tuple, Optional


class AVLAnalyzer:
    """Main class for AVL analysis automation"""
    
    def __init__(self, working_dir: str = "."):
        """Initialize AVL analyzer"""
        self.working_dir = Path(working_dir)
        self.avl_exe = self._find_avl_executable()
        self.param_file = self.working_dir / "our_reference_aircraft_parameters.txt"
        
    def _find_avl_executable(self) -> Path:
        """Find AVL executable in current directory, bin, runs, or parent directory"""
        for name in ["avl.exe", "avl"]:
            # Check current directory
            path = self.working_dir / name
            if path.exists():
                return path
            # Check bin subdirectory
            path = self.working_dir / "bin" / name
            if path.exists():
                return path
            # Check runs subdirectory
            path = self.working_dir / "runs" / name
            if path.exists():
                return path
            # Try parent directory
            path = self.working_dir.parent / name
            if path.exists():
                return path
        raise FileNotFoundError("AVL executable (avl.exe or avl) not found in current dir, bin/, runs/, or parent dir")
    
    def parse_parameters(self) -> Dict:
        """Parse aircraft parameters from text file"""
        if not self.param_file.exists():
            raise FileNotFoundError(f"Parameter file not found: {self.param_file}")
        
        params = {
            "mtow": 2699.03,
            "wing_area": 27.862,
            "wing_span": 15.83,
            "root_chord": 2.5620,
            "tip_chord": 1.1529,
            "mac": 1.94653,
            "h_tail_area": 10.628,
            "h_tail_span": 6.52,
            "h_root_chord": 2.2484,
            "h_tip_chord": 1.0118,
            "h_tail_arm": 7.0,
            "v_tail_area": 4.868,
            "v_tail_span": 2.96,
            "v_root_chord": 2.268,
            "v_tip_chord": 1.0208,
            "v_tail_arm": 7.0,
            "cg_x": 0.4866,
            "cg_y": 0.0,
            "cg_z": -0.053,
            "ixx": 12121.2,
            "iyy": 12610.9,
            "izz": 23029.8,
            "ixz": 979.1,
            "dihedral": 3.0,
        }
        return params
    
    def generate_avl_file(self, params: Dict) -> None:
        """Generate UAM.AVL geometry file - in proper AVL format matching plane.avl structure"""
        avl_content = """!***************************************
!UAM Aircraft - Aerodynamic Analysis
!***************************************
UAM Aircraft
!Mach
 0.0    
!IYsym   IZsym   Zsym
 0       0       0.0
!Sref    Cref    Bref
{sref}  {cref}  {bref}
!Xref    Yref    Zref
0.0      0.0      0.0

#--------------------------------------------------
SURFACE
Main_Wing
!Nchordwise  Cspace  Nspanwise  Sspace
12           1.0     20         1.0
YDUPLICATE
0.0
ANGLE
0.0

SECTION
!Xle    Yle    Zle     Chord   Ainc  Nspanwise  Sspace
0.0     0.0    0.0     {rc}    0.0   8          1.0

SECTION
!Xle    Yle    Zle     Chord   Ainc  Nspanwise  Sspace
0.0     {ws2}  {ws2z}  {rcm}   0.0   0          0

SECTION
!Xle    Yle    Zle     Chord   Ainc  Nspanwise  Sspace
0.0     {ws}   {wsz}   {tc}    0.0   0          0

#--------------------------------------------------
SURFACE
Horiz_Tail
!Nchordwise  Cspace  Nspanwise  Sspace
10           1.0     12         1.0
YDUPLICATE
0.0
TRANSLATE
{hta}   0.0   0.0

SECTION
!Xle    Yle    Zle     Chord   Ainc  Nspanwise  Sspace
0.0     0.0    0.0     {hrc}   0.0   12         1.0
CONTROL
Elevator  1.0  0.0  0.0  0.0  0.0  1

SECTION
!Xle    Yle    Zle     Chord   Ainc  Nspanwise  Sspace
0.0     {hts}  0.0     {htc}   0.0   0          0
CONTROL
Elevator  1.0  0.0  0.0  0.0  0.0  1

#--------------------------------------------------
SURFACE
Vert_Tail
!Nchordwise  Cspace  Nspanwise  Sspace
8            1.0     10         1.0
TRANSLATE
{vta}   0.0   0.0

SECTION
!Xle    Yle    Zle     Chord   Ainc  Nspanwise  Sspace
0.0     0.0    0.0     {vrc}   0.0   10         1.0
CONTROL
Rudder  1.0  0.0  0.0  0.0  0.0  1

SECTION
!Xle    Yle    Zle     Chord   Ainc  Nspanwise  Sspace
0.0     0.0    {vts}   {vtc}   0.0   0          0
CONTROL
Rudder  1.0  0.0  0.0  0.0  0.0  1
""".format(
            sref=params['wing_area'],
            cref=params['mac'],
            bref=params['wing_span'],
            rc=params['root_chord'],
            ws=params['wing_span'],
            wsz=params['wing_span'] * 0.052,
            ws2=params['wing_span']/2,
            ws2z=params['wing_span']/2 * 0.052,
            rcm=params['root_chord'] * 0.7,
            tc=params['tip_chord'],
            hta=params['h_tail_arm'],
            hrc=params['h_root_chord'],
            hts=params['h_tail_span'],
            htc=params['h_tip_chord'],
            vta=params['v_tail_arm'],
            vrc=params['v_root_chord'],
            vts=params['v_tail_span'],
            vtc=params['v_tip_chord'],
        )
        
        avl_file = self.working_dir / "uam.avl"
        with open(avl_file, "w") as f:
            f.write(avl_content)
        print("[OK] Generated: {}".format(avl_file))
    
    def generate_mass_file(self, params: Dict) -> None:
        """Generate UAM.MASS file"""
        mass_content = """
UAM Aircraft - Mass Properties

Lunit = 1.0 m
Munit = 1.0 kg
Tunit = 1.0 s

g   = 9.81
rho = 1.1673

mass   x     y     z       Ixx     Iyy    Izz   Ixy  Ixz  Iyz
{mass}  {cx}  {cy}  {cz}  {ixx}  {iyy}  {izz}  0.0  {ixz}  0.0
""".format(
            mass=params['mtow'],
            cx=params['cg_x'],
            cy=params['cg_y'],
            cz=params['cg_z'],
            ixx=params['ixx'],
            iyy=params['iyy'],
            izz=params['izz'],
            ixz=params['ixz'],
        )
        
        mass_file = self.working_dir / "uam.mass"
        with open(mass_file, "w") as f:
            f.write(mass_content)
        print("[OK] Generated: {}".format(mass_file))
    
    def run_avl(self) -> str:
        """Execute AVL with commands and capture output"""
        # Build the command sequence with proper AVL syntax
        # Note: Empty lines in input are important for menu navigation
        commands = [
            "load uam.avl",
            "mass uam.mass",
            "oper",
            "a a 2.5",      # Set alpha to 2.5 degrees (lowercase 'a' twice)
            "x",             # Execute calculation
            "st",            # Show stability derivatives
            "",              # Empty line to return to OPER menu
            "",              # Extra empty line for robustness
            "quit"           # Exit AVL
        ]
        
        cmd_input = "\n".join(commands) + "\n"
        
        print("\n" + "="*60)
        print("Running AVL Analysis...")
        print("="*60 + "\n")
        
        try:
            result = subprocess.run(
                [str(self.avl_exe)],
                input=cmd_input,
                capture_output=True,
                text=True,
                cwd=str(self.working_dir),
                timeout=60
            )
            
            if result.returncode != 0:
                print("[WARN] AVL returned code {}".format(result.returncode))
            
            return result.stdout + result.stderr
            
        except subprocess.TimeoutExpired:
            raise TimeoutError("AVL execution timed out after 60 seconds")
        except Exception as e:
            raise RuntimeError("Failed to run AVL: {}".format(e))
    
    def parse_trim_data(self, output: str) -> Dict[str, float]:
        """Extract trim data from AVL output"""
        trim_data = {}
        
        # Look for "Alpha = " pattern (more robust)
        alpha_match = re.search(r'Alpha\s*=\s*([-\d.]+)', output)
        if alpha_match:
            trim_data['Alpha (deg)'] = float(alpha_match.group(1))
        
        # Look for CLtot
        cl_match = re.search(r'CLtot\s*=\s*([-\d.]+)', output)
        if cl_match:
            trim_data['CLtot'] = float(cl_match.group(1))
        
        # Look for Cmtot
        cm_match = re.search(r'Cmtot\s*=\s*([-\d.]+)', output)
        if cm_match:
            trim_data['Cmtot'] = float(cm_match.group(1))
        
        # Look for Elevator deflection
        elev_match = re.search(r'Elevator\s*=\s*([-\d.]+)', output)
        if elev_match:
            trim_data['Elevator (deg)'] = float(elev_match.group(1))
        
        return trim_data
    
    def parse_stability_derivatives(self, output: str) -> Dict[str, float]:
        """Extract static stability derivatives"""
        stab_data = {}
        
        cla_match = re.search(r'CLa\s*=\s*([-\d.]+)', output)
        if cla_match:
            stab_data['CLa'] = float(cla_match.group(1))
        
        cma_match = re.search(r'Cma\s*=\s*([-\d.]+)', output)
        if cma_match:
            stab_data['Cma'] = float(cma_match.group(1))
        
        cnb_match = re.search(r'Cnb\s*=\s*([-\d.]+)', output)
        if cnb_match:
            stab_data['Cnb'] = float(cnb_match.group(1))
        
        clb_match = re.search(r'Clb\s*=\s*([-\d.]+)', output)
        if clb_match:
            stab_data['Clb'] = float(clb_match.group(1))
        
        return stab_data
    
    def parse_eigenvalues(self, output: str) -> Dict[str, Tuple]:
        """Extract dynamic modes and eigenvalues"""
        modes = {}
        mode_lines = output.split('\n')
        
        for i, line in enumerate(mode_lines):
            line_lower = line.lower()
            if 'eigenvalue' in line_lower or 'frequency' in line_lower:
                for j in range(i, min(i+5, len(mode_lines))):
                    if '1/T' in mode_lines[j]:
                        parts = mode_lines[j].split()
                        if len(parts) >= 2:
                            try:
                                real = float(parts[0])
                                imag = float(parts[1])
                                for k in range(max(0, j-3), j):
                                    if any(x in mode_lines[k].lower() for x in 
                                           ['short', 'phugoid', 'dutch', 'spiral', 'roll']):
                                        mode_name = mode_lines[k].strip()
                                        modes[mode_name] = (real, imag)
                            except ValueError:
                                pass
        
        return modes
    
    def print_results(self, trim: Dict, stability: Dict, modes: Dict, 
                      raw_output: Optional[str] = None) -> None:
        """Print formatted results"""
        
        print("\n" + "="*70)
        print("AVL ANALYSIS RESULTS".center(70))
        print("="*70 + "\n")
        
        if trim:
            print("TRIM CONDITIONS:")
            print("-" * 70)
            for key, value in trim.items():
                print("  {:25} {:>12.6f}".format(key, value))
            print()
        else:
            print("[WARN] No trim data extracted\n")
        
        if stability:
            print("STATIC STABILITY DERIVATIVES:")
            print("-" * 70)
            for key, value in stability.items():
                print("  {:25} {:>12.6f}".format(key, value))
            print()
        else:
            print("[WARN] No stability derivatives extracted\n")
        
        if not (trim or stability):
            print("[WARN] PARSING INCOMPLETE - Last 150 lines of AVL Output:")
            print("-" * 70)
            if raw_output:
                lines = raw_output.split('\n')[-150:]
                for line in lines:
                    print(line)
            else:
                print("No output captured")
        
        print("="*70 + "\n")
    
    def save_raw_output(self, output: str) -> None:
        """Save raw output to file for debugging"""
        output_file = self.working_dir / "avl_output.txt"
        with open(output_file, "w") as f:
            f.write(output)
        print("[OK] Raw AVL output saved to: {}".format(output_file))
    
    def run_analysis(self) -> None:
        """Main analysis workflow"""
        try:
            print("Step 1: Parsing aircraft parameters...")
            params = self.parse_parameters()
            print("[OK] Parameters loaded")
            
            print("\nStep 2: Generating AVL input files...")
            self.generate_avl_file(params)
            self.generate_mass_file(params)
            
            print("\nStep 3: Executing AVL analysis...")
            output = self.run_avl()
            
            self.save_raw_output(output)
            
            print("Step 4: Parsing results...")
            trim = self.parse_trim_data(output)
            stability = self.parse_stability_derivatives(output)
            
            self.print_results(trim, stability, {}, output)
            
            if not (trim or stability):
                print("[WARN] No data could be parsed from AVL output.")
                print("       The aircraft configuration may need adjustment.")
                print("       Please check: avl_output.txt")
            
        except FileNotFoundError as e:
            print("[ERROR] {}".format(e))
            print("        Please ensure:")
            print("        1. AVL executable (avl.exe or avl) is in {} or subdirectories".format(self.working_dir))
            print("        2. {} exists".format(self.param_file))
            sys.exit(1)
            
        except TimeoutError as e:
            print("[ERROR] {}".format(e))
            sys.exit(1)
            
        except Exception as e:
            print("[ERROR] Unexpected error: {}".format(e))
            import traceback
            traceback.print_exc()
            sys.exit(1)


def main():
    """Entry point"""
    analyzer = AVLAnalyzer(working_dir=".")
    analyzer.run_analysis()


if __name__ == "__main__":
    main()
