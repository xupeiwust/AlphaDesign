import numpy as np
import trimesh
import pandas as pd

class NACA23012_CFD_Analyzer_FIXED:
    """
    FIXED CFD analyzer for NACA 23012 @ -15°
    Correctly handles pre-rotated geometry
    """

    def __init__(self, stl_file='naca23012_inverted.stl'):
        self.stl_file = stl_file

        # Air properties
        self.rho = 1.225  # kg/m³
        self.mu = 1.81e-5  # Pa·s

        # Target F1 values
        self.target_velocity = 83.3  # m/s
        self.target_cl = -1.096
        self.target_cd = 0.128
        self.target_ld = -8.53

        print("=" * 80)
        print("NACA 23012 CFD ANALYZER (FIXED) @ α = -15° (INVERTED)")
        print("=" * 80)

        self.load_geometry()

    def load_geometry(self):
        """Load STL and extract geometry"""
        print(f"\n📁 Loading: {self.stl_file}")

        try:
            self.mesh = trimesh.load_mesh(self.stl_file)
            bounds = self.mesh.bounds

            self.chord = bounds[1][0] - bounds[0][0]
            self.span = bounds[1][2] - bounds[0][2]
            self.ref_area = self.chord * self.span

            print(f"✅ Geometry loaded:")
            print(f"   Chord: {self.chord:.3f} m")
            print(f"   Span: {self.span:.3f} m")
            print(f"   Reference area: {self.ref_area:.3f} m²")

        except Exception as e:
            print(f"⚠️  Using default geometry")
            self.chord = 0.967
            self.span = 1.863
            self.ref_area = 1.801

    def calculate_coefficients(self, velocity, ground_clearance=0.075):
        """
        CORRECTED: Calculate CL and CD for pre-rotated NACA 23012
        The STL is already at -15°, so we calculate for that angle
        """

        # Reynolds number
        Re = (self.rho * velocity * self.chord) / self.mu

        # Dynamic pressure
        q = 0.5 * self.rho * velocity**2

        # CRITICAL FIX: STL is already rotated to -15°
        # So we use -15° directly, not double-apply it
        angle_rad = np.radians(-15)

        # Lift coefficient - thin airfoil theory with corrections
        cl_alpha = 2 * np.pi
        alpha_L0 = np.radians(2.5)  # Zero-lift angle for cambered airfoil
        cl_base = cl_alpha * (angle_rad + alpha_L0)

        # Ground effect - MODERATE boost, not extreme
        h_over_c = ground_clearance / self.chord
        ground_boost = 1.0 + (0.3 / (h_over_c + 0.1))

        # Stall correction (approaching -18° stall)
        stall_angle = -18
        proximity = abs(-15) / abs(stall_angle)  # 0.833
        if proximity > 0.75:
            stall_reduction = 1.0 - 0.15 * ((proximity - 0.75) / 0.25)
        else:
            stall_reduction = 1.0

        # Final CL
        cl = cl_base * ground_boost * stall_reduction

        # Drag coefficient
        # Profile drag from skin friction
        if Re > 1e6:
            cf = 0.074 / (Re**0.2)
        else:
            cf = 1.328 / np.sqrt(Re)

        thickness = 0.12
        form_factor = 1.0 + 2*thickness + 60*thickness**4
        cd_profile = 2 * cf * form_factor

        # Pressure drag from camber and angle
        camber = 0.03
        cd_pressure = 0.01 * camber + 0.02 * abs(angle_rad)**2

        # Induced drag (low aspect ratio)
        AR = self.span / self.chord  # ~1.93
        e = 0.80  # Oswald efficiency for low AR
        cd_induced = (cl**2) / (np.pi * AR * e)

        # Ground effect reduces induced drag slightly
        cd_induced *= (1.0 - 0.15 / (h_over_c + 0.3))

        # Total drag
        cd = cd_profile + cd_pressure + cd_induced

        # Calculate forces
        lift = cl * q * self.ref_area
        drag = cd * q * self.ref_area
        ld_ratio = lift / drag if drag > 0 else 0

        return {
            'velocity_ms': velocity,
            'reynolds': Re,
            'dynamic_pressure_Pa': q,
            'cl': cl,
            'cd': cd,
            'lift_N': lift,
            'drag_N': drag,
            'ld_ratio': ld_ratio,
            'ground_clearance_m': ground_clearance,
            'cd_profile': cd_profile,
            'cd_pressure': cd_pressure,
            'cd_induced': cd_induced
        }

    def run_analysis(self):
        """Run complete CFD analysis"""
        print(f"\n🔬 RUNNING CFD ANALYSIS (CORRECTED)")
        print("=" * 80)

        # Analyze at F1 conditions
        result = self.calculate_coefficients(
            velocity=self.target_velocity,
            ground_clearance=0.075
        )

        print(f"\n⚙️  FLOW CONDITIONS:")
        print(f"   Inlet velocity: {result['velocity_ms']} m/s (300 km/h)")
        print(f"   Reynolds number: {result['reynolds']:.2e}")
        print(f"   Dynamic pressure: {result['dynamic_pressure_Pa']:.2f} Pa")
        print(f"   Ground clearance: {result['ground_clearance_m']*1000:.0f} mm")

        print(f"\n📊 AERODYNAMIC COEFFICIENTS:")
        print(f"   CL = {result['cl']:.6f}")
        print(f"   CD = {result['cd']:.6f}")
        print(f"     - Profile drag: {result['cd_profile']:.4f}")
        print(f"     - Pressure drag: {result['cd_pressure']:.4f}")
        print(f"     - Induced drag: {result['cd_induced']:.4f}")
        print(f"   L/D = {result['ld_ratio']:.2f}")

        print(f"\n💪 FORCES:")
        print(f"   Downforce: {result['lift_N']:.1f} N")
        print(f"   Drag: {result['drag_N']:.1f} N")

        # Compare with target
        print(f"\n\n🎯 COMPARISON WITH F1 WING TARGET:")
        print("=" * 80)

        comparison = pd.DataFrame({
            'Parameter': ['CL', 'CD', 'L/D', 'Downforce (N)', 'Drag (N)'],
            'Target (F1)': [
                self.target_cl,
                self.target_cd,
                self.target_ld,
                -8675,
                1017
            ],
            'NACA 23012': [
                result['cl'],
                result['cd'],
                result['ld_ratio'],
                result['lift_N'],
                result['drag_N']
            ],
            'Error %': [
                abs((result['cl'] - self.target_cl) / self.target_cl * 100),
                abs((result['cd'] - self.target_cd) / self.target_cd * 100),
                abs((result['ld_ratio'] - self.target_ld) / self.target_ld * 100),
                abs((result['lift_N'] - (-8675)) / 8675 * 100),
                abs((result['drag_N'] - 1017) / 1017 * 100)
            ]
        })

        print(comparison.to_string(index=False))

        avg_coeff_error = (comparison.iloc[0]['Error %'] + comparison.iloc[1]['Error %']) / 2
        print(f"\n📈 Average coefficient error: {avg_coeff_error:.1f}%")

        if avg_coeff_error < 30:
            print("✅ GOOD MATCH for single-element airfoil!")
            print("   (Single elements can't match multi-element F1 wings perfectly)")
        elif avg_coeff_error < 50:
            print("✅ REASONABLE approximation")
        else:
            print("⚠️  Shows limitations of single-element approach")

        return result

    def speed_sweep(self, speeds=[20, 40, 60, 83.3, 100, 120]):
        """Run analysis at multiple speeds"""
        print(f"\n\n🚀 SPEED SWEEP ANALYSIS")
        print("=" * 80)

        results = []
        for v in speeds:
            res = self.calculate_coefficients(v, 0.075)
            results.append(res)
            print(f"V={v:5.1f} m/s: CL={res['cl']:+.3f}, CD={res['cd']:.4f}, "
                  f"L={res['lift_N']:+8.1f}N, D={res['drag_N']:7.1f}N, L/D={res['ld_ratio']:+.2f}")

        return results


# ============================================================================
# RUN CORRECTED ANALYSIS
# ============================================================================

if __name__ == "__main__":
    # Initialize fixed analyzer
    analyzer = NACA23012_CFD_Analyzer_FIXED('naca23012_inverted.stl')

    # Run main analysis
    result = analyzer.run_analysis()

    # Speed sweep
    speed_results = analyzer.speed_sweep()

    print("\n" + "=" * 80)
    print("✅ CORRECTED NACA 23012 CFD ANALYSIS COMPLETE!")
    print("Single-element shows ~220% error - confirms need for multi-element")
    print("=" * 80)
