import numpy as np
import trimesh
import pandas as pd
from datetime import datetime
import os

class MultiElementF1WingAnalyzer:
    """
    Complete multi-element F1 wing CFD analyzer
    Matches the exact F1 wing configuration
    """

    def __init__(self, element_files):
        """
        Args:
            element_files: List of STL file paths [main, flap1, flap2, flap3, flap4]
        """
        self.element_files = element_files
        self.elements = []

        # Air properties
        self.air_density = 1.225  # kg/m³
        self.air_viscosity = 1.81e-5  # Pa·s

        # F1 target conditions
        self.target_velocity = 83.3  # m/s (300 km/h)
        self.target_cl = -1.096
        self.target_cd = 0.128
        self.target_ld = -8.53

        # Element configurations (from JSON)
        self.element_configs = [
            {'name': 'Main', 'area_fraction': 0.295, 'angle': 0, 'camber': 0.0},
            {'name': 'Flap 1', 'area_fraction': 0.237, 'angle': 8, 'camber': 0.142},
            {'name': 'Flap 2', 'area_fraction': 0.188, 'angle': 11, 'camber': 0.118},
            {'name': 'Flap 3', 'area_fraction': 0.159, 'angle': 14, 'camber': 0.092},
            {'name': 'Flap 4', 'area_fraction': 0.121, 'angle': 17, 'camber': 0.068}
        ]

        # Slot effect parameters
        self.slot_effect_multiplier = 1.35  # Circulation boost from slot
        self.boundary_layer_control = 0.92  # Reduction in separation

        print("🏎️  MULTI-ELEMENT F1 WING CFD ANALYZER")
        print("=" * 70)

        self.load_elements()

    def load_elements(self):
        """Load all wing elements"""
        print("\n📁 LOADING WING ELEMENTS")
        print("-" * 70)

        total_area = 0
        for i, filepath in enumerate(self.element_files):
            try:
                mesh = trimesh.load_mesh(filepath)
                bounds = mesh.bounds

                # Calculate element area
                x_range = bounds[1][0] - bounds[0][0]
                z_range = bounds[1][2] - bounds[0][2]
                area = x_range * z_range

                element_data = {
                    'mesh': mesh,
                    'bounds': bounds,
                    'area': area,
                    'config': self.element_configs[i]
                }

                self.elements.append(element_data)
                total_area += area

                print(f"✅ {self.element_configs[i]['name']}: {area:.3f} m²")

            except Exception as e:
                print(f"❌ Error loading {filepath}: {e}")

        self.total_area = total_area
        print(f"\n📏 Total reference area: {total_area:.3f} m²")

    def calculate_reynolds_number(self, velocity, chord):
        """Calculate Reynolds number"""
        return (self.air_density * velocity * chord) / self.air_viscosity

    def calculate_element_forces(self, element_idx, velocity, ground_clearance=0.075):
        """Calculate aerodynamic forces for single element"""
        element = self.elements[element_idx]
        config = element['config']
        area = element['area']

        # Dynamic pressure
        q = 0.5 * self.air_density * velocity**2

        # Effective angle (element angle + wing attitude)
        alpha_eff = np.radians(config['angle'])

        # Base lift coefficient (thin airfoil theory with camber)
        cl_alpha = 2 * np.pi
        alpha_0 = -2 * config['camber']  # Zero-lift angle
        cl_base = cl_alpha * (alpha_eff - alpha_0)

        # Multi-element effects
        if element_idx > 0:
            # Slot effect: upstream element energizes boundary layer
            slot_boost = self.slot_effect_multiplier
            cl_base *= slot_boost

            # Circulation coupling from upstream elements
            upstream_boost = 1.0 + 0.15 * element_idx
            cl_base *= upstream_boost

        # Ground effect (inverse of height)
        ground_factor = 1.0 + (0.1 / (ground_clearance + 0.01))
        cl_element = cl_base * ground_factor

        # Drag calculation
        # Profile drag
        cd_profile = 0.006 + 0.01 * (config['camber'] / 0.15)

        # Induced drag
        aspect_ratio = 5.9  # span/chord approximation
        cd_induced = (cl_element**2) / (np.pi * aspect_ratio * 0.85)

        # Multi-element interference drag
        if element_idx > 0:
            cd_interference = 0.008 * element_idx
        else:
            cd_interference = 0

        cd_element = cd_profile + cd_induced + cd_interference

        # Forces
        lift = cl_element * q * area
        drag = cd_element * q * area

        return {
            'cl': cl_element,
            'cd': cd_element,
            'lift_N': lift,
            'drag_N': drag,
            'area': area
        }

    def analyze_complete_wing(self, velocity=None, ground_clearance=0.075):
        """Analyze complete multi-element wing"""
        if velocity is None:
            velocity = self.target_velocity

        print(f"\n🔬 ANALYZING COMPLETE WING")
        print(f"   Velocity: {velocity:.1f} m/s ({velocity*3.6:.0f} km/h)")
        print(f"   Ground clearance: {ground_clearance*1000:.0f} mm")
        print("-" * 70)

        total_lift = 0
        total_drag = 0
        element_results = []

        for i, element in enumerate(self.elements):
            result = self.calculate_element_forces(i, velocity, ground_clearance)
            element_results.append(result)

            total_lift += result['lift_N']
            total_drag += result['drag_N']

            print(f"{element['config']['name']:8s}: CL={result['cl']:+.3f}, CD={result['cd']:.4f}, "
                  f"L={result['lift_N']:+7.1f}N, D={result['drag_N']:6.1f}N")

        # Overall coefficients
        q = 0.5 * self.air_density * velocity**2
        cl_total = total_lift / (q * self.total_area)
        cd_total = total_drag / (q * self.total_area)
        ld_ratio = cl_total / cd_total if cd_total > 0 else 0

        print(f"\n{'TOTAL':8s}: CL={cl_total:+.3f}, CD={cd_total:.4f}, "
              f"L={total_lift:+7.1f}N, D={total_drag:6.1f}N, L/D={ld_ratio:+.2f}")

        return {
            'velocity_ms': velocity,
            'ground_clearance_m': ground_clearance,
            'total_lift_N': total_lift,
            'total_drag_N': total_drag,
            'cl_total': cl_total,
            'cd_total': cd_total,
            'ld_ratio': ld_ratio,
            'element_results': element_results
        }

    def match_target_values(self):
        """Attempt to match target F1 wing values"""
        print("\n\n🎯 MATCHING TARGET F1 WING VALUES")
        print("=" * 70)
        print(f"Target: CL={self.target_cl:.3f}, CD={self.target_cd:.3f}, L/D={self.target_ld:.2f}")

        result = self.analyze_complete_wing(
            velocity=self.target_velocity,
            ground_clearance=0.075
        )

        # Compare with targets
        cl_error = abs(result['cl_total'] - self.target_cl)
        cd_error = abs(result['cd_total'] - self.target_cd)
        ld_error = abs(result['ld_ratio'] - self.target_ld)

        print("\n📊 COMPARISON WITH TARGET")
        print("-" * 70)
        comparison_df = pd.DataFrame({
            'Parameter': ['CL', 'CD', 'L/D', 'Downforce (N)', 'Drag (N)'],
            'Target': [self.target_cl, self.target_cd, self.target_ld, -8675, 1017],
            'Calculated': [
                result['cl_total'],
                result['cd_total'],
                result['ld_ratio'],
                result['total_lift_N'],
                result['total_drag_N']
            ],
            'Error %': [
                cl_error / abs(self.target_cl) * 100,
                cd_error / self.target_cd * 100,
                ld_error / abs(self.target_ld) * 100,
                abs(result['total_lift_N'] - (-8675)) / 8675 * 100,
                abs(result['total_drag_N'] - 1017) / 1017 * 100
            ]
        })

        print(comparison_df.to_string(index=False))

        # Verdict
        avg_error = (cl_error / abs(self.target_cl) + cd_error / self.target_cd) / 2 * 100
        print(f"\n📈 Average coefficient error: {avg_error:.1f}%")

        if avg_error < 10:
            print("✅ EXCELLENT MATCH!")
        elif avg_error < 20:
            print("✅ GOOD MATCH")
        elif avg_error < 30:
            print("⚠️  ACCEPTABLE MATCH")
        else:
            print("❌ NEEDS TUNING")

        return result

    def export_report(self, output_folder="cfd_reports"):
        """Export analysis report"""
        if not os.path.exists(output_folder):
            os.makedirs(output_folder)

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"{output_folder}/F1_MultiElement_Report_{timestamp}.md"

        result = self.match_target_values()

        with open(filename, 'w') as f:
            f.write("# F1 Multi-Element Wing CFD Analysis Report\n\n")
            f.write(f"**Date:** {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n\n")
            f.write("## Configuration\n")
            f.write(f"- Elements: 5 (NACA 64A010 main + 4 flaps)\n")
            f.write(f"- Total Area: {self.total_area:.3f} m²\n")
            f.write(f"- Velocity: {self.target_velocity:.1f} m/s\n\n")
            f.write("## Results\n")
            f.write(f"- **CL:** {result['cl_total']:.3f} (target: {self.target_cl:.3f})\n")
            f.write(f"- **CD:** {result['cd_total']:.3f} (target: {self.target_cd:.3f})\n")
            f.write(f"- **L/D:** {result['ld_ratio']:.2f} (target: {self.target_ld:.2f})\n")

        print(f"\n💾 Report saved: {filename}")
        return filename


# Main execution
if __name__ == "__main__":
    element_files = [
        'f1_main_element.stl',
        'f1_flap_1.stl',
        'f1_flap_2.stl',
        'f1_flap_3.stl',
        'f1_flap_4.stl'
    ]

    analyzer = MultiElementF1WingAnalyzer(element_files)
    result = analyzer.match_target_values()
    analyzer.export_report()

    print("\n✅ Multi-element F1 wing CFD analysis complete!")
