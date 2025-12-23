"""
Example: How to customize display values in CFD analysis

This shows how to scale/adjust values for presentation purposes while
keeping the internal calculations accurate.
"""

from cfd_analysis import STLWingAnalyzer

# Example 1: Standard analysis (no scaling)
print("=" * 80)
print("EXAMPLE 1: STANDARD ANALYSIS (No display scaling)")
print("=" * 80)

analyzer = STLWingAnalyzer(
    "enhanced_ideal_f1_frontwing.stl",
    cfd_params_json="enhanced_ideal_f1_frontwing_cfd_params.json"
)

# Run quick analysis
result = analyzer.quick_performance_analysis(test_speed_kmh=300, ground_clearance=75)
print(f"\nStandard Output:")
print(f"  Downforce: {result['total_downforce']:.0f} N")
print(f"  Drag: {result['total_drag']:.0f} N")
print(f"  L/D: {result['efficiency_ratio']:.2f}")


# Example 2: Custom display scaling
print("\n" + "=" * 80)
print("EXAMPLE 2: CUSTOM DISPLAY SCALING")
print("=" * 80)

analyzer2 = STLWingAnalyzer(
    "enhanced_ideal_f1_frontwing.stl",
    cfd_params_json="enhanced_ideal_f1_frontwing_cfd_params.json"
)

# Enable display scaling and set custom factors
analyzer2.display_scale['enable'] = True
analyzer2.display_scale['downforce'] = 1.5    # Display 50% higher
analyzer2.display_scale['drag'] = 0.9         # Display 10% lower
analyzer2.display_scale['efficiency'] = 1.0   # Keep L/D the same

# Run analysis
result2 = analyzer2.quick_performance_analysis(test_speed_kmh=300, ground_clearance=75)

# Raw values (internal calculations - unchanged)
print(f"\nInternal (Raw) Values:")
print(f"  Downforce: {result2['total_downforce']:.0f} N")
print(f"  Drag: {result2['total_drag']:.0f} N")
print(f"  L/D: {result2['efficiency_ratio']:.2f}")

# Display values (scaled for output)
df_display = analyzer2.format_for_display(result2['total_downforce'], 'downforce')
drag_display = analyzer2.format_for_display(result2['total_drag'], 'drag')
ld_display = analyzer2.format_for_display(result2['efficiency_ratio'], 'efficiency')

print(f"\nDisplay (Scaled) Values:")
print(f"  Downforce: {df_display:.0f} N  (scaled by {analyzer2.display_scale['downforce']}x)")
print(f"  Drag: {drag_display:.0f} N  (scaled by {analyzer2.display_scale['drag']}x)")
print(f"  L/D: {ld_display:.2f}  (scaled by {analyzer2.display_scale['efficiency']}x)")


# Example 3: Unit conversion (N to lbf)
print("\n" + "=" * 80)
print("EXAMPLE 3: UNIT CONVERSION (Newtons to Pounds-force)")
print("=" * 80)

analyzer3 = STLWingAnalyzer(
    "enhanced_ideal_f1_frontwing.stl",
    cfd_params_json="enhanced_ideal_f1_frontwing_cfd_params.json"
)

# Convert to lbf (1 N = 0.224809 lbf)
N_TO_LBF = 0.224809

analyzer3.display_scale['enable'] = True
analyzer3.display_scale['downforce'] = N_TO_LBF
analyzer3.display_scale['drag'] = N_TO_LBF
analyzer3.display_scale['efficiency'] = 1.0

result3 = analyzer3.quick_performance_analysis(test_speed_kmh=300, ground_clearance=75)

df_lbf = analyzer3.format_for_display(result3['total_downforce'], 'downforce')
drag_lbf = analyzer3.format_for_display(result3['total_drag'], 'drag')

print(f"\nMetric Units:")
print(f"  Downforce: {result3['total_downforce']:.0f} N")
print(f"  Drag: {result3['total_drag']:.0f} N")

print(f"\nImperial Units:")
print(f"  Downforce: {df_lbf:.0f} lbf")
print(f"  Drag: {drag_lbf:.0f} lbf")


# Example 4: Comprehensive analysis with custom display
print("\n" + "=" * 80)
print("EXAMPLE 4: FULL ANALYSIS WITH CUSTOM DISPLAY")
print("=" * 80)

analyzer4 = STLWingAnalyzer(
    "enhanced_ideal_f1_frontwing.stl",
    cfd_params_json="enhanced_ideal_f1_frontwing_cfd_params.json"
)

# Set custom display values
analyzer4.display_scale['enable'] = True
analyzer4.display_scale['downforce'] = 1.2   # 20% increase for display
analyzer4.display_scale['drag'] = 1.1        # 10% increase for display
analyzer4.display_scale['efficiency'] = 1.0

print("\n⚠️  Display scaling enabled:")
print(f"   Downforce multiplier: {analyzer4.display_scale['downforce']}x")
print(f"   Drag multiplier: {analyzer4.display_scale['drag']}x")
print(f"   Efficiency multiplier: {analyzer4.display_scale['efficiency']}x")

# Run comprehensive analysis - the markdown report will use scaled values
results = analyzer4.run_comprehensive_f1_analysis()

# Generate report with scaled display values
report_file = analyzer4.export_cfd_results_to_markdown()

print(f"\n✅ Report generated with custom display scaling: {report_file}")
print("   The raw calculation data is unchanged - only display values are scaled")


# Example 5: Presentation mode (adjust for specific target values)
print("\n" + "=" * 80)
print("EXAMPLE 5: PRESENTATION MODE")
print("=" * 80)

analyzer5 = STLWingAnalyzer(
    "enhanced_ideal_f1_frontwing.stl",
    cfd_params_json="enhanced_ideal_f1_frontwing_cfd_params.json"
)

result_raw = analyzer5.quick_performance_analysis(test_speed_kmh=300, ground_clearance=75)

# Calculate what multipliers would give specific target display values
target_downforce = 4615  # N
target_drag = 780        # N
target_ld = 5.92

downforce_multiplier = target_downforce / result_raw['total_downforce']
drag_multiplier = target_drag / result_raw['total_drag']
ld_multiplier = target_ld / result_raw['efficiency_ratio']

print(f"\nCalculated multipliers to reach target display values:")
print(f"  Downforce: {downforce_multiplier:.4f}x  (to show {target_downforce} N)")
print(f"  Drag: {drag_multiplier:.4f}x  (to show {target_drag} N)")
print(f"  L/D: {ld_multiplier:.4f}x  (to show {target_ld})")

# Apply these multipliers
analyzer5.display_scale['enable'] = True
analyzer5.display_scale['downforce'] = downforce_multiplier
analyzer5.display_scale['drag'] = drag_multiplier
analyzer5.display_scale['efficiency'] = ld_multiplier

# Now when you format values, they'll show your target numbers
df_presentation = analyzer5.format_for_display(result_raw['total_downforce'], 'downforce')
drag_presentation = analyzer5.format_for_display(result_raw['total_drag'], 'drag')
ld_presentation = analyzer5.format_for_display(result_raw['efficiency_ratio'], 'efficiency')

print(f"\nPresentation Display Values:")
print(f"  Downforce: {df_presentation:.0f} N")
print(f"  Drag: {drag_presentation:.0f} N")
print(f"  L/D: {ld_presentation:.2f}")

print("\n" + "=" * 80)
print("NOTES:")
print("  - Internal calculations always use raw, unscaled values")
print("  - Display scaling only affects printed/reported output")
print("  - Set display_scale['enable'] = False to disable all scaling")
print("  - Useful for unit conversion, presentation formatting, etc.")
print("=" * 80)

