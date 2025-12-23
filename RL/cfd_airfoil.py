import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from scipy import interpolate, optimize
from scipy.spatial import distance
import trimesh
import json
import os
from datetime import datetime
import warnings
warnings.filterwarnings('ignore')

class AirfoilCFDAnalyzer:
    """
    Simplified CFD analysis for single airfoil profiles (NACA 4415)
    Based on enhanced F1 wing analyzer but adapted for 2D airfoil analysis
    """

    def __init__(self, stl_file_path):
        """
        Initialize airfoil CFD analyzer

        Args:
            stl_file_path: Path to NACA 4415 STL file
        """
        self.stl_file_path = stl_file_path
        self.mesh = None

        # Air properties (standard conditions)
        self.air_density = 1.225  # kg/m³
        self.air_viscosity = 1.81e-5  # Pa·s
        self.kinematic_viscosity = 1.5e-5  # m²/s

        # Test conditions
        self.test_speeds = [20, 30, 40, 50, 60, 70]  # m/s
        self.test_angles = [-5, -2, 0, 2, 5, 8, 10, 12, 15]  # degrees

        # Airfoil properties (NACA 4415)
        self.max_camber = 0.04  # 4% camber
        self.max_camber_position = 0.4  # 40% chord
        self.max_thickness = 0.15  # 15% thickness

        print("✈️  NACA 4415 AIRFOIL CFD ANALYSIS SYSTEM")
        print("=" * 60)
        print(f"📁 Loading STL file: {stl_file_path}")

        # Load STL
        self.load_stl_file()
        self.extract_airfoil_geometry()

    def load_stl_file(self):
        """Load STL file using trimesh"""
        try:
            self.mesh = trimesh.load_mesh(self.stl_file_path)
            print(f"✅ STL file loaded successfully")
            print(f"   - Vertices: {len(self.mesh.vertices):,}")
            print(f"   - Faces: {len(self.mesh.faces):,}")

            self.mesh_bounds = self.mesh.bounds
            self.mesh_center = self.mesh.centroid

        except Exception as e:
            print(f"❌ Error loading STL file: {e}")
            raise

    def extract_airfoil_geometry(self):
        """Extract airfoil geometric parameters"""
        print("\n🔍 EXTRACTING AIRFOIL GEOMETRY")
        print("-" * 50)

        vertices = self.mesh.vertices

        # Get dimensions
        x_range = self.mesh_bounds[1][0] - self.mesh_bounds[0][0]  # Chord
        y_range = self.mesh_bounds[1][1] - self.mesh_bounds[0][1]  # Span
        z_range = self.mesh_bounds[1][2] - self.mesh_bounds[0][2]  # Thickness

        self.chord_length = x_range
        self.span = y_range
        self.max_thickness_actual = z_range

        print(f"📐 Airfoil Dimensions:")
        print(f"   - Chord length: {self.chord_length*1000:.1f} mm")
        print(f"   - Span: {self.span*1000:.1f} mm")
        print(f"   - Max thickness: {self.max_thickness_actual*1000:.1f} mm")
        print(f"   - Thickness ratio: {self.max_thickness_actual/self.chord_length:.3f}")

        # Reference area for force calculations
        self.reference_area = self.chord_length * self.span
        print(f"📏 Reference Area: {self.reference_area:.6f} m²")

    def calculate_reynolds_number(self, velocity_ms):
        """Calculate Reynolds number"""
        return (self.air_density * velocity_ms * self.chord_length) / self.air_viscosity

    def calculate_dynamic_pressure(self, velocity_ms):
        """Calculate dynamic pressure"""
        return 0.5 * self.air_density * velocity_ms**2

    def calculate_lift_coefficient(self, angle_of_attack_deg):
        """
        Calculate lift coefficient for NACA 4415
        Uses thin airfoil theory with corrections
        """
        alpha_rad = np.radians(angle_of_attack_deg)

        # Lift slope (2D airfoil)
        cl_alpha = 2 * np.pi

        # Zero-lift angle (function of camber)
        alpha_0 = -2 * self.max_camber  # radians

        # Linear region
        cl_linear = cl_alpha * (alpha_rad - alpha_0)

        # Stall model for NACA 4415
        stall_angle = 16.0  # degrees (typical for NACA 4415)

        if abs(angle_of_attack_deg) > stall_angle:
            # Post-stall
            stall_progress = (abs(angle_of_attack_deg) - stall_angle) / 10.0
            stall_factor = np.exp(-stall_progress)
            cl_stalled = cl_linear * stall_factor
            return cl_stalled

        # Pre-stall nonlinearity
        if abs(angle_of_attack_deg) > stall_angle * 0.7:
            nonlinear_factor = 1 - 0.1 * ((abs(angle_of_attack_deg) / stall_angle) - 0.7)**2
            cl_linear *= nonlinear_factor

        return cl_linear

    def calculate_drag_coefficient(self, angle_of_attack_deg, reynolds_number):
        """
        Calculate drag coefficient for NACA 4415
        """
        alpha_rad = np.radians(angle_of_attack_deg)

        # Profile drag (based on NACA 4415 characteristics)
        cd_0 = 0.006  # Zero-lift drag coefficient

        # Reynolds number correction
        if reynolds_number > 1e6:
            re_factor = (reynolds_number / 1e6) ** (-0.15)
        else:
            re_factor = 1.2

        cd_profile = cd_0 * re_factor

        # Angle of attack contribution
        cd_alpha = 0.01 * (alpha_rad**2)

        # Induced drag (for 2D, this represents tip effects)
        cl = self.calculate_lift_coefficient(angle_of_attack_deg)
        aspect_ratio = self.span / self.chord_length
        cd_induced = (cl**2) / (np.pi * aspect_ratio * 0.9)  # e = 0.9

        # Stall drag penalty
        stall_angle = 16.0
        if abs(angle_of_attack_deg) > stall_angle:
            stall_factor = 1 + 3 * ((abs(angle_of_attack_deg) - stall_angle) / 10)**2
            cd_profile *= stall_factor

        return cd_profile + cd_alpha + cd_induced

    def analyze_at_condition(self, velocity_ms, angle_of_attack_deg):
        """
        Perform CFD analysis at specific condition
        """
        # Calculate flow parameters
        reynolds = self.calculate_reynolds_number(velocity_ms)
        dynamic_pressure = self.calculate_dynamic_pressure(velocity_ms)

        # Calculate coefficients
        cl = self.calculate_lift_coefficient(angle_of_attack_deg)
        cd = self.calculate_drag_coefficient(angle_of_attack_deg, reynolds)

        # Calculate forces
        lift = cl * dynamic_pressure * self.reference_area
        drag = cd * dynamic_pressure * self.reference_area

        # Lift-to-drag ratio
        ld_ratio = lift / drag if drag > 0 else 0

        # Pressure coefficient at leading edge (stagnation)
        cp_stagnation = 1.0

        # Flow quality assessment
        stall_angle = 16.0
        stall_margin = stall_angle - abs(angle_of_attack_deg)

        if stall_margin > 5:
            flow_quality = "Excellent - Fully attached"
        elif stall_margin > 2:
            flow_quality = "Good - Attached"
        elif stall_margin > 0:
            flow_quality = "Marginal - Near stall"
        else:
            flow_quality = "Stalled"

        return {
            'velocity_ms': velocity_ms,
            'angle_deg': angle_of_attack_deg,
            'reynolds_number': reynolds,
            'dynamic_pressure_Pa': dynamic_pressure,
            'lift_coefficient': cl,
            'drag_coefficient': cd,
            'lift_N': lift,
            'drag_N': drag,
            'ld_ratio': ld_ratio,
            'stall_margin_deg': stall_margin,
            'flow_quality': flow_quality
        }

    def run_comprehensive_analysis(self):
        """
        Run comprehensive CFD analysis
        """
        print("\n🔍 STARTING COMPREHENSIVE CFD ANALYSIS")
        print("=" * 60)

        results = {
            'speed_sweep': [],
            'angle_sweep': [],
            'polar_data': []
        }

        # Speed sweep at 0° angle of attack
        print("\n📊 Speed Sweep Analysis (α = 0°)...")
        for speed in self.test_speeds:
            result = self.analyze_at_condition(speed, 0)
            results['speed_sweep'].append(result)
            print(f"   {speed} m/s: L={result['lift_N']:.2f}N, D={result['drag_N']:.3f}N, L/D={result['ld_ratio']:.1f}")

        # Angle sweep at 50 m/s
        print("\n📐 Angle of Attack Sweep (V = 50 m/s)...")
        for angle in self.test_angles:
            result = self.analyze_at_condition(50, angle)
            results['angle_sweep'].append(result)
            print(f"   α={angle:+3.0f}°: CL={result['lift_coefficient']:.3f}, CD={result['drag_coefficient']:.4f}, L/D={result['ld_ratio']:.1f}")

        # Generate polar data (full sweep)
        print("\n🌀 Generating Airfoil Polar Data...")
        for speed in [30, 50, 70]:
            for angle in np.linspace(-5, 15, 21):
                result = self.analyze_at_condition(speed, angle)
                results['polar_data'].append(result)

        self.analysis_results = results
        print("\n✅ Comprehensive analysis complete!")

        return results

    def export_results_to_markdown(self, output_folder="cfd_reports"):
        """
        Export CFD results to markdown report
        """
        if not os.path.exists(output_folder):
            os.makedirs(output_folder)

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"{output_folder}/NACA4415_CFD_Report_{timestamp}.md"

        if not hasattr(self, 'analysis_results'):
            print("⚠️  No analysis data available")
            return None

        results = self.analysis_results
        report = []

        # Header
        report.append("# NACA 4415 Airfoil CFD Analysis Report\n")
        report.append(f"**Date:** {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
        report.append(f"**Airfoil:** NACA 4415\n")
        report.append("---\n")

        # Geometry
        report.append("## 📐 Airfoil Geometry\n")
        report.append(f"- **Chord Length:** {self.chord_length*1000:.1f} mm\n")
        report.append(f"- **Span:** {self.span*1000:.1f} mm\n")
        report.append(f"- **Max Thickness:** {self.max_thickness*100:.1f}%\n")
        report.append(f"- **Max Camber:** {self.max_camber*100:.1f}%\n")
        report.append(f"- **Reference Area:** {self.reference_area:.6f} m²\n")
        report.append("---\n")

        # Speed Sweep
        report.append("## 🚀 Speed Sweep Results (α = 0°)\n")
        report.append("| Speed (m/s) | Reynolds | CL | CD | L/D | Lift (N) | Drag (N) |\n")
        report.append("|-------------|----------|----|----|-----|----------|----------|\n")
        for data in results['speed_sweep']:
            report.append(f"| {data['velocity_ms']:.0f} | {data['reynolds_number']:.2e} | "
                         f"{data['lift_coefficient']:.3f} | {data['drag_coefficient']:.4f} | "
                         f"{data['ld_ratio']:.1f} | {data['lift_N']:.2f} | {data['drag_N']:.3f} |\n")
        report.append("---\n")

        # Angle Sweep
        report.append("## 📐 Angle of Attack Sweep (V = 50 m/s)\n")
        report.append("| Angle (°) | CL | CD | L/D | Stall Margin (°) | Flow Quality |\n")
        report.append("|-----------|----|----|-----|------------------|-------------|\n")
        for data in results['angle_sweep']:
            report.append(f"| {data['angle_deg']:+3.0f} | {data['lift_coefficient']:.3f} | "
                         f"{data['drag_coefficient']:.4f} | {data['ld_ratio']:.1f} | "
                         f"{data['stall_margin_deg']:.1f} | {data['flow_quality']} |\n")
        report.append("---\n")

        # Performance Summary
        report.append("## 📈 Performance Summary\n")
        angle_data = results['angle_sweep']
        max_ld_idx = np.argmax([d['ld_ratio'] for d in angle_data])
        max_cl_idx = np.argmax([d['lift_coefficient'] for d in angle_data])

        report.append(f"- **Maximum L/D:** {angle_data[max_ld_idx]['ld_ratio']:.1f} at α = {angle_data[max_ld_idx]['angle_deg']:.0f}°\n")
        report.append(f"- **Maximum CL:** {angle_data[max_cl_idx]['lift_coefficient']:.3f} at α = {angle_data[max_cl_idx]['angle_deg']:.0f}°\n")
        report.append(f"- **Stall Angle:** ~16°\n")
        report.append("---\n")

        report.append("*Report generated by NACA 4415 CFD Analysis System*\n")

        # Write file
        with open(filename, 'w') as f:
            f.writelines(report)

        print(f"✅ Report saved to: {filename}")
        return filename

    def plot_results(self):
        """Generate plots of analysis results"""
        if not hasattr(self, 'analysis_results'):
            print("⚠️  Run analysis first")
            return

        results = self.analysis_results

        fig, axes = plt.subplots(2, 2, figsize=(12, 10))

        # CL vs alpha
        angle_data = results['angle_sweep']
        angles = [d['angle_deg'] for d in angle_data]
        cls = [d['lift_coefficient'] for d in angle_data]
        axes[0, 0].plot(angles, cls, 'b-o', linewidth=2)
        axes[0, 0].set_xlabel('Angle of Attack (°)')
        axes[0, 0].set_ylabel('Lift Coefficient (CL)')
        axes[0, 0].set_title('CL vs Angle of Attack')
        axes[0, 0].grid(True)

        # CD vs alpha
        cds = [d['drag_coefficient'] for d in angle_data]
        axes[0, 1].plot(angles, cds, 'r-o', linewidth=2)
        axes[0, 1].set_xlabel('Angle of Attack (°)')
        axes[0, 1].set_ylabel('Drag Coefficient (CD)')
        axes[0, 1].set_title('CD vs Angle of Attack')
        axes[0, 1].grid(True)

        # Drag polar (CL vs CD)
        axes[1, 0].plot(cds, cls, 'g-o', linewidth=2)
        axes[1, 0].set_xlabel('Drag Coefficient (CD)')
        axes[1, 0].set_ylabel('Lift Coefficient (CL)')
        axes[1, 0].set_title('Drag Polar')
        axes[1, 0].grid(True)

        # L/D vs alpha
        ld_ratios = [d['ld_ratio'] for d in angle_data]
        axes[1, 1].plot(angles, ld_ratios, 'm-o', linewidth=2)
        axes[1, 1].set_xlabel('Angle of Attack (°)')
        axes[1, 1].set_ylabel('L/D Ratio')
        axes[1, 1].set_title('Efficiency vs Angle of Attack')
        axes[1, 1].grid(True)

        plt.tight_layout()
        plt.savefig('naca4415_analysis.png', dpi=300)
        print("✅ Plots saved to: naca4415_analysis.png")
        plt.show()


# Main execution
if __name__ == "__main__":
    print("✈️  NACA 4415 AIRFOIL CFD ANALYSIS")
    print("=" * 60)

    # Initialize analyzer
    analyzer = AirfoilCFDAnalyzer('naca4415.stl')

    # Run comprehensive analysis
    results = analyzer.run_comprehensive_analysis()

    # Export report
    report_file = analyzer.export_results_to_markdown()

    # Generate plots
    analyzer.plot_results()

    print("\n✅ NACA 4415 CFD Analysis Complete!")
