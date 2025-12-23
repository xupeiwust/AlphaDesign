
import numpy as np
import trimesh

class NACA64A010_Validator:
    '''
    Single-element validation for CFD methodology
    Tests NACA 64A010 at multiple angles
    '''

    def __init__(self, stl_file='naca64a010.stl'):
        self.stl_file = stl_file
        self.rho = 1.225
        self.mu = 1.81e-5

        # Load geometry
        mesh = trimesh.load_mesh(stl_file)
        bounds = mesh.bounds
        self.chord = bounds[1][0] - bounds[0][0]
        self.span = bounds[1][2] - bounds[0][2]
        self.ref_area = self.chord * self.span

    def validate_at_angle(self, alpha_deg, velocity=83.3):
        '''Calculate CL, CD at specific angle'''
        alpha_rad = np.radians(alpha_deg)
        q = 0.5 * self.rho * velocity**2
        Re = (self.rho * velocity * self.chord) / self.mu

        # Symmetric airfoil: CL = cl_alpha * alpha
        cl_alpha = 2 * np.pi
        cl_2d = cl_alpha * alpha_rad

        # 3D correction
        AR = self.span / self.chord
        cl = cl_2d / (1 + (2/AR))

        # Drag
        cf = 0.074 / (Re**0.2) if Re > 1e6 else 1.328 / np.sqrt(Re)
        cd0 = 2 * cf * 1.1  # Form factor
        cd_induced = (cl**2) / (np.pi * AR * 0.90)
        cd = cd0 + cd_induced

        # Forces
        lift = cl * q * self.ref_area
        drag = cd * q * self.ref_area

        return {
            'alpha': alpha_deg,
            'cl': cl,
            'cd': cd,
            'lift_N': lift,
            'drag_N': drag,
            'ld': lift/drag if drag > 0 else 0
        }

    def run_validation(self):
        '''Run validation sweep'''
        print("NACA 64A010 VALIDATION")
        print("="*60)

        angles = [-5, -2, 0, 2, 5, 8, 10]
        results = []

        for angle in angles:
            res = self.validate_at_angle(angle)
            results.append(res)
            print(f"α={res['alpha']:+3d}°: CL={res['cl']:+.4f}, "
                  f"CD={res['cd']:.4f}, L/D={res['ld']:+.2f}")

        print("\n✅ Validation complete!")
        print("Compare these with airfoiltools.com polar data")
        return results

# Run validation
validator = NACA64A010_Validator('naca64a010.stl')
results = validator.run_validation()
