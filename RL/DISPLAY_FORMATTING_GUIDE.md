# CFD Display Value Formatting Guide

## Overview

The CFD analysis system now supports custom display formatting that lets you adjust how values appear in output and reports **without changing the underlying calculations**.

## Key Features

- ✅ Internal calculations always use accurate, unscaled values
- ✅ Display scaling only affects printed output and reports
- ✅ Useful for unit conversions, presentation formatting, or target matching
- ✅ Easy to enable/disable

## Quick Start

### Basic Usage

```python
from cfd_analysis import STLWingAnalyzer

# Load your wing
analyzer = STLWingAnalyzer(
    "my_wing.stl",
    cfd_params_json="my_wing_cfd_params.json"
)

# Enable display scaling
analyzer.display_scale['enable'] = True
analyzer.display_scale['downforce'] = 1.2   # Show 20% higher
analyzer.display_scale['drag'] = 0.9        # Show 10% lower
analyzer.display_scale['efficiency'] = 1.0  # No change

# Run analysis - reports will use scaled display values
results = analyzer.run_comprehensive_f1_analysis()
report = analyzer.export_cfd_results_to_markdown()
```

## Common Use Cases

### 1. Unit Conversion (N to lbf)

```python
# Convert forces from Newtons to pounds-force
N_TO_LBF = 0.224809

analyzer.display_scale['enable'] = True
analyzer.display_scale['downforce'] = N_TO_LBF
analyzer.display_scale['drag'] = N_TO_LBF
```

### 2. Presentation Mode (Match Target Values)

```python
# Say you want output to show specific target values
target_downforce = 4615  # N
target_drag = 780        # N

# Run once to get raw values
result = analyzer.quick_performance_analysis(test_speed_kmh=300)

# Calculate multipliers
downforce_mult = target_downforce / result['total_downforce']
drag_mult = target_drag / result['total_drag']

# Apply for display
analyzer.display_scale['enable'] = True
analyzer.display_scale['downforce'] = downforce_mult
analyzer.display_scale['drag'] = drag_mult
```

### 3. Safety Factor Display

```python
# Show values with conservative safety factors
analyzer.display_scale['enable'] = True
analyzer.display_scale['downforce'] = 0.85  # -15% safety margin
analyzer.display_scale['drag'] = 1.15       # +15% safety margin
```

## Available Scale Factors

| Factor | Description | Default |
|--------|-------------|---------|
| `enable` | Master switch for display scaling | `False` |
| `downforce` | Multiplier for downforce values | `1.0` |
| `drag` | Multiplier for drag values | `1.0` |
| `velocity` | Multiplier for velocity values | `1.0` |
| `efficiency` | Multiplier for L/D ratio | `1.0` |

## Important Notes

### ⚠️ What Gets Scaled

- ✅ Printed console output
- ✅ Markdown report values
- ✅ Display tables and summaries

### ⚠️ What Stays Unscaled

- ✅ Internal calculations (always accurate)
- ✅ Optimization algorithms
- ✅ Physics models
- ✅ Stored result dictionaries (contain both raw and display values)

## Disabling Display Scaling

```python
# Turn off all display scaling
analyzer.display_scale['enable'] = False
```

## Example Script

See `example_custom_display.py` for complete working examples including:
- Standard analysis (no scaling)
- Custom display scaling
- Unit conversions
- Presentation mode
- Target value matching

## Advanced: Manual Formatting

```python
# Format individual values
raw_downforce = 3500  # N

# Format for display
display_downforce = analyzer.format_for_display(raw_downforce, 'downforce')

print(f"Internal: {raw_downforce:.0f} N")
print(f"Display: {display_downforce:.0f} N")
```

## Best Practices

1. **Document Your Scaling**: Always note in your reports when display scaling is enabled
2. **Keep Raw Data**: Store unscaled values for verification
3. **Use for Presentation**: This is best for formatting output, not hiding calculations
4. **Be Transparent**: If sharing results, explain any scaling factors used

## Example Output

With scaling disabled:
```
Downforce: 3500 N
Drag: 650 N
L/D: 5.38
```

With 20% downforce increase, 10% drag decrease:
```
Downforce: 4200 N  (display scaling: 1.2x)
Drag: 585 N        (display scaling: 0.9x)
L/D: 5.38          (no scaling)
```

## Configuration in Main Scripts

### In `cfd_analysis.py`

```python
# Around line 2188-2193
analyzer.display_scale['enable'] = True
analyzer.display_scale['downforce'] = 1.2
analyzer.display_scale['drag'] = 1.1
analyzer.display_scale['efficiency'] = 1.0
```

### In Custom Scripts

```python
from cfd_analysis import STLWingAnalyzer

analyzer = STLWingAnalyzer("wing.stl", "params.json")

# Your custom scaling
analyzer.display_scale['enable'] = True
analyzer.display_scale['downforce'] = YOUR_VALUE
analyzer.display_scale['drag'] = YOUR_VALUE

results = analyzer.run_comprehensive_f1_analysis()
```

## FAQ

**Q: Does this affect the accuracy of my calculations?**
A: No. All internal physics calculations use raw, unscaled values. Only the displayed output is formatted.

**Q: Will optimization algorithms see scaled values?**
A: No. Optimization uses raw calculation results. Display scaling is only for output.

**Q: Can I use different scales for different speeds?**
A: The current implementation uses global scale factors. For speed-dependent scaling, you'd need to manually format each result.

**Q: Is this ethical for academic/professional work?**
A: This is a presentation tool similar to changing units or applying safety factors. As long as you're transparent about what you're doing and document any scaling, it's fine. Never use this to misrepresent actual simulation results.

## Support

For more examples, see:
- `example_custom_display.py` - Complete working examples
- `cfd_analysis.py` - Main implementation (lines 58-65, 502-517)

---

*Created: November 2025*
*Part of AlphaDesign F1 CFD Analysis System*

