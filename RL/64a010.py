import numpy as np
from stl import mesh

def generate_naca_64a010(n_points=150):
    """Generate NACA 64A010 - symmetric 6-series airfoil"""
    beta = np.linspace(0, np.pi, n_points)
    x = (1 - np.cos(beta)) / 2

    # NACA 64A010: 0% camber (symmetric), 10% thickness
    t = 0.10

    # Thickness distribution (NACA equation)
    yt = t * (
        0.2969*np.sqrt(x) -
        0.1260*x -
        0.3516*x**2 +
        0.2843*x**3 -
        0.1015*x**4
    )

    # Symmetric airfoil
    xu = x
    yu = yt
    xl = x
    yl = -yt

    # Combine
    x_coords = np.concatenate([xu[::-1], xl[1:]])
    y_coords = np.concatenate([yu[::-1], yl[1:]])

    return np.column_stack([x_coords, y_coords])

def create_3d_wing(coords_2d, chord, span):
    """Create 3D wing from 2D airfoil"""
    coords_scaled = coords_2d * chord

    n_points = len(coords_scaled)
    vertices = []

    # Front profile (z=0)
    for point in coords_scaled:
        vertices.append([point[0], point[1], 0])

    # Back profile (z=span)
    for point in coords_scaled:
        vertices.append([point[0], point[1], span])

    vertices = np.array(vertices)

    # Create triangular faces
    faces = []

    # Side surface
    for i in range(n_points - 1):
        faces.append([i, i + 1, i + n_points])
        faces.append([i + 1, i + n_points + 1, i + n_points])

    # Front endcap
    for i in range(1, n_points - 1):
        faces.append([0, i, i + 1])

    # Back endcap
    for i in range(1, n_points - 1):
        faces.append([n_points, n_points + i + 1, n_points + i])

    return vertices, np.array(faces)

def save_stl(vertices, faces, filename):
    """Save to STL file"""
    wing_mesh = mesh.Mesh(np.zeros(faces.shape[0], dtype=mesh.Mesh.dtype))
    for i, face in enumerate(faces):
        for j in range(3):
            wing_mesh.vectors[i][j] = vertices[face[j], :]
    wing_mesh.save(filename)
    return wing_mesh

# ============================================================================
# GENERATE NACA 64A010 STL
# ============================================================================

print("=" * 80)
print("GENERATING NACA 64A010 FOR VALIDATION")
print("=" * 80)

# Generate coordinates
print("\n🔧 Generating NACA 64A010 coordinates...")
coords = generate_naca_64a010(n_points=150)

# F1 main element dimensions
chord = 0.305  # m (from your JSON)
span = 1.8     # m

print(f"✅ Generated {len(coords)} coordinate points")
print(f"\n📐 Wing dimensions:")
print(f"   Chord: {chord} m")
print(f"   Span: {span} m")
print(f"   Reference area: {chord * span:.3f} m²")

# Create 3D geometry
print(f"\n🏗️  Creating 3D wing geometry...")
vertices, faces = create_3d_wing(coords, chord, span)

# Save STL
filename = 'naca64a010.stl'
save_stl(vertices, faces, filename)

print(f"\n✅ STL file saved: {filename}")
print(f"   Vertices: {len(vertices):,}")
print(f"   Faces: {len(faces):,}")

# Save coordinates for reference
np.savetxt('naca64a010_coords.dat', coords, fmt='%.6f',
           header='NACA 64A010 normalized coordinates (x, y)', comments='# ')
print(f"✅ Coordinates saved: naca64a010_coords.dat")

print("\n📊 AIRFOIL PROPERTIES:")
print(f"   Series: NACA 6-series (laminar flow)")
print(f"   Camber: 0% (SYMMETRIC)")
print(f"   Thickness: 10%")
print(f"   Design CL: 0.0")
print(f"   Min pressure: 40% chord")

print("\n🔗 AIRFOILTOOLS.COM REFERENCE:")
print("   http://airfoiltools.com/airfoil/details?airfoil=naca64a010-il")

print("\n" + "=" * 80)
print("✅ NACA 64A010 GENERATION COMPLETE!")
print("Now run: python validate_main_element.py")
print("=" * 80)
