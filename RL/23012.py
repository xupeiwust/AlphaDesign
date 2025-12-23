import numpy as np
from stl import mesh

def generate_naca_5digit_23012(n_points=150):
    """
    Generate NACA 23012 airfoil coordinates
    2: Design lift coefficient = 0.3 (for normal, so inverted gives -0.3 base)
    30: Max camber at 15% chord (30/2 = 15)
    12: 12% thickness
    """
    # Cosine spacing for better LE/TE resolution
    beta = np.linspace(0, np.pi, n_points)
    x = (1 - np.cos(beta)) / 2

    # NACA 5-digit camber line parameters
    # For 230xx series: m = 0.2025, k1 = 361.4
    m = 0.2025  # Max camber
    p = 0.15    # Position (15% chord)

    # Mean line for 230 series
    yc = np.zeros_like(x)
    dyc_dx = np.zeros_like(x)

    for i, xi in enumerate(x):
        if xi <= p:
            yc[i] = (m/6) * (xi**3 - 3*m*xi**2 + m**2*(3-m)*xi)
            dyc_dx[i] = (m/6) * (3*xi**2 - 6*m*xi + m**2*(3-m))
        else:
            yc[i] = (m*m**2/6) * (1 - xi)
            dyc_dx[i] = -(m*m**2/6)

    # Thickness distribution (12%)
    t = 0.12
    yt = 5 * t * (
        0.2969*np.sqrt(x) -
        0.1260*x -
        0.3516*x**2 +
        0.2843*x**3 -
        0.1015*x**4
    )

    # Calculate surface coordinates
    theta = np.arctan(dyc_dx)

    # Upper surface
    xu = x - yt * np.sin(theta)
    yu = yc + yt * np.cos(theta)

    # Lower surface
    xl = x + yt * np.sin(theta)
    yl = yc - yt * np.cos(theta)

    # Combine into single array (upper backwards, then lower)
    x_coords = np.concatenate([xu[::-1], xl[1:]])
    y_coords = np.concatenate([yu[::-1], yl[1:]])

    return np.column_stack([x_coords, y_coords])

def rotate_2d(coords, angle_deg):
    """Rotate 2D coordinates by angle (degrees)"""
    angle_rad = np.radians(angle_deg)
    cos_a = np.cos(angle_rad)
    sin_a = np.sin(angle_rad)
    rotation_matrix = np.array([[cos_a, -sin_a], [sin_a, cos_a]])
    return coords @ rotation_matrix.T

def create_3d_airfoil(coords_2d, chord_m, span_m):
    """Create 3D wing from 2D airfoil coordinates"""
    # Scale by chord
    coords_scaled = coords_2d * chord_m

    n_points = len(coords_scaled)
    vertices = []

    # Front profile (z=0)
    for point in coords_scaled:
        vertices.append([point[0], point[1], 0])

    # Back profile (z=span)
    for point in coords_scaled:
        vertices.append([point[0], point[1], span_m])

    vertices = np.array(vertices)

    # Create triangular faces
    faces = []

    # Side surface (quad split into 2 triangles)
    for i in range(n_points - 1):
        # Triangle 1
        faces.append([i, i + 1, i + n_points])
        # Triangle 2
        faces.append([i + 1, i + n_points + 1, i + n_points])

    # Front endcap
    for i in range(1, n_points - 1):
        faces.append([0, i, i + 1])

    # Back endcap
    for i in range(1, n_points - 1):
        faces.append([n_points, n_points + i + 1, n_points + i])

    return vertices, np.array(faces)

def save_stl(vertices, faces, filename):
    """Save mesh to STL file"""
    wing_mesh = mesh.Mesh(np.zeros(faces.shape[0], dtype=mesh.Mesh.dtype))
    for i, face in enumerate(faces):
        for j in range(3):
            wing_mesh.vectors[i][j] = vertices[face[j], :]
    wing_mesh.save(filename)
    return wing_mesh

# ============================================================================
# MAIN GENERATION
# ============================================================================

print("=" * 80)
print("NACA 23012 AIRFOIL GENERATOR - F1 WING MATCH")
print("=" * 80)

# Generate base NACA 23012 profile
print("\n🔧 Generating NACA 23012 coordinates...")
naca23012_coords = generate_naca_5digit_23012(n_points=150)

print(f"✅ Generated {len(naca23012_coords)} coordinate points")
print(f"\nFirst 5 points (leading edge):")
print(naca23012_coords[:5])

# Rotate by -15 degrees for inverted operation (downforce)
angle_of_attack = -15  # degrees (inverted for downforce)
print(f"\n🔄 Rotating airfoil by {angle_of_attack}° for downforce operation...")
naca23012_rotated = rotate_2d(naca23012_coords, angle_of_attack)

# Scale to F1 wing dimensions
# To match total area of 1.863 m², we'll use:
chord = 1.0   # 1.0m chord
span = 1.863  # 1.863m span → Area = 1.863 m²

print(f"\n📐 Wing dimensions:")
print(f"   Chord: {chord} m")
print(f"   Span: {span} m")
print(f"   Reference area: {chord * span:.3f} m²")
print(f"   Angle of attack: {angle_of_attack}° (inverted)")

# Create 3D geometry
print(f"\n🏗️  Creating 3D wing geometry...")
vertices, faces = create_3d_airfoil(naca23012_rotated, chord, span)

# Save STL
filename = 'naca23012_inverted.stl'
save_stl(vertices, faces, filename)

print(f"\n✅ STL file saved: {filename}")
print(f"   Vertices: {len(vertices):,}")
print(f"   Faces: {len(faces):,}")

# Calculate geometric properties
max_y = np.max(naca23012_coords[:, 1])
min_y = np.min(naca23012_coords[:, 1])
camber = (max_y + abs(min_y)) / 2

print(f"\n📊 AIRFOIL PROPERTIES:")
print(f"   Series: NACA 5-digit (230xx)")
print(f"   Camber: ~{camber*100:.1f}%")
print(f"   Max camber position: 15% chord")
print(f"   Thickness: 12%")
print(f"   Design: High-lift configuration")

print("\n" + "=" * 80)
print("✅ NACA 23012 GENERATION COMPLETE!")
print("=" * 80)
