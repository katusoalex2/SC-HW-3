import numpy as np

m = 2699.03
L = 8.0
b = 15.83
Rx = 0.28
Ry = 0.36
Rz = 0.44
 ###Refering to section 16.8 from  Raymer book##
Ixx_raymer = m * ((b * Rx) / 2)**2
Iyy_raymer = m * ((L * Ry) / 2)**2
Izz_raymer = m * (((b + L) * Rz) / 2)**2

print(f"Raymer: Ixx={Ixx_raymer:.1f}, Iyy={Iyy_raymer:.1f}, Izz={Izz_raymer:.1f}")

# Component buildup (typical 2700kg eVTOL / Lift+Cruise UAM)
# Origin at CG (x forward, y right, z down)
components = [
    # name, mass (kg), x (m), y (m), z (m), Ixx_loc, Iyy_loc, Izz_loc
    ("Fuselage & Cabin", 374.41, 0.2, 0.0, 0.1, 250, 2200, 2200),
    ("Battery +fuel +tank (Belly)", 350.0, 0.0, 0.0, 0.45, 120, 400, 450),
    ("Payload / 4 Pax + Pilot", 450.0, 0.3, 0.0, 0.05, 80, 250, 250),
    ("Main Wing", 327.61, -0.1, 0.0, -0.55, 4200, 80, 4250),
    ("Horizontal Tail", 53.0, -7.1, 0.0, -0.6, 150, 10, 160),
    ("Vertical Tail", 40.6, -7.1, 0.0, -1.8, 30, 30, 5),
    ("Cruise Propulsion (Aft)", 163.17, -4.0, 0.0, -0.2, 20, 30, 30),
    ("Lift Booms & 8 Lift Rotors (Left)", 269.695, -0.1, -3.5, -0.45, 50, 900, 900),
    ("Lift Booms & 8 Lift Rotors (Right)", 269.695, -0.1, 3.5, -0.45, 50, 900, 900),
    ("Avionics, Landing Gear & Misc", 399.99, 0.5, 0.0, 0.5, 30, 150, 150)
]

total_m = sum(c[1] for c in components)
# adjust fuselage mass slightly to match exact 2699.03
diff = m - total_m
components[0] = ("Fuselage & Cabin", 374.41 + diff, 0.2, 0.0, 0.1, 250, 2200, 2200)

# Re-center CG exactly to (0,0,0)
cg_x = sum(c[1]*c[2] for c in components) / m
cg_y = sum(c[1]*c[3] for c in components) / m
cg_z = sum(c[1]*c[4] for c in components) / m

Ixx = sum(c[5] + c[1]*((c[3]-cg_y)**2 + (c[4]-cg_z)**2) for c in components)
Iyy = sum(c[6] + c[1]*((c[2]-cg_x)**2 + (c[4]-cg_z)**2) for c in components)
Izz = sum(c[7] + c[1]*((c[2]-cg_x)**2 + (c[3]-cg_y)**2) for c in components)
Ixz = sum(c[1]*(c[2]-cg_x)*(c[4]-cg_z) for c in components)

print(f"Buildup: Total Mass={sum(c[1] for c in components):.2f}, CG=({cg_x:.3f}, {cg_y:.3f}, {cg_z:.3f})")
print(f"Buildup: Ixx={Ixx:.1f}, Iyy={Iyy:.1f}, Izz={Izz:.1f}, Ixz={Ixz:.1f}")
