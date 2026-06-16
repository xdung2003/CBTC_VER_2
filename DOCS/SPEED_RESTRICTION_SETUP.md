# Speed Restriction (PSR/TSR) Setup Guide

## Overview

The CBTC_SIM project now fully supports both **PSR (Permitted Speed Restriction)** and **TSR (Temporary Speed Restriction)** configurations.

### PSR (Permitted Speed Restriction)
- **Static** speed limits defined per track segment
- Configured in YAML under `track.segments[].psr_kmh`
- Cannot be modified during simulation (PSR changes require restart)
- Example: 60 km/h for segment 0-800m

### TSR (Temporary Speed Restriction)
- **Dynamic** speed restrictions for specific track zones
- Can be defined initially in YAML under `tsr_zones`
- Can be added/modified/removed during simulation via GUI
- Useful for modeling temporary track work, weather conditions, etc.

---

## YAML Configuration

### PSR Configuration (Track Segments)

```yaml
track:
  segments:
  - start_m: 0.0
    end_m: 800.0
    gradient: 0.02
    psr_kmh: 60.0  # Permitted Speed Restriction
  - start_m: 800.0
    end_m: 1000.0
    gradient: 0.05
    psr_kmh: 40.0
```

### TSR Configuration (New Feature)

```yaml
tsr_zones:
- start: 300.0
  end: 500.0
  speed: 30.0  # Temporary restriction in km/h
- start: 1200.0
  end: 1400.0
  speed: 35.0
```

**Optional**: The `tsr_zones` section can be omitted (defaults to empty list).

---

## How TSR Speeds Are Applied

The Zone Controller (ZC) applies the **most restrictive** speed limit:

```
Applied Speed = min(PSR, TSR, Curve Speed, ...)
```

### Example Scenario:
- Track segment 0-800m: PSR = 60 km/h
- TSR zone 300-500m: TSR = 30 km/h
- Train position 350m: **Applied speed = 30 km/h**

---

## Runtime TSR Management (GUI)

### Adding a TSR During Simulation

1. Open main GUI
2. Control Panel → "Issue TSR" button
3. Enter:
   - Start position (m)
   - End position (m)  
   - Speed limit (km/h)
4. TSR is immediately applied to all trains

### Editing/Removing TSR

1. Limits Panel shows all current TSR zones
2. Click on a TSR zone to edit or remove
3. Choose to remove or modify the speed

### Clearing All TSR Zones

- Control Panel → "Clear TSR" button
- Removes all dynamic TSR restrictions

---

## Implementation Details

### Files Modified

1. **CONFIG/scenario_loader.py**
   - Added `_normalize_tsr_zones()` function
   - Updated `normalize_scenario()` to process TSR zones

2. **SUBSYSTEMS/runtime.py**
   - Updated `load_scenario()` to load TSR zones
   - TSR zones stored in `self.tsr_zones` list
   - Added `update_tsr()` method for runtime updates

3. **DOCS/default_scenario.yaml**
   - Added empty `tsr_zones: []` section

4. **SUBSYSTEMS/zc.py** (Already supported)
   - TSR zones considered in `build_safe_packets()`
   - `next_lower_limit()` checks both PSR and TSR

### Data Structure

TSR zones are stored as a list of dictionaries:

```python
tsr_zones = [
    {
        "start": 300.0,      # Start position (m)
        "end": 500.0,        # End position (m)
        "speed": 30.0        # Speed limit (km/h)
    },
    ...
]
```

---

## Testing

### Test Files Created

1. **test_tsr_loading.py**
   - Tests YAML loading
   - Tests scenario normalization
   - Tests simulation initialization with TSR

2. **test_tsr_zc_integration.py**
   - Tests ZC integration
   - Tests speed limit calculation with TSR
   - Tests safe packet generation

### Run Tests

```bash
python test_tsr_loading.py
python test_tsr_zc_integration.py
```

---

## Example Scenarios

### Scenario 1: Default (No TSR)
```bash
python run.py  # Uses default_scenario.yaml with empty tsr_zones
```

### Scenario 2: With Initial TSR Zones
```bash
# Edit GUI to load DOCS/example_with_tsr.yaml
# Features 2 TSR zones:
# - 300-500m @ 30km/h
# - 1200-1400m @ 35km/h
```

---

## Runtime Command Examples

The following commands can be dispatched from ATS:

### Add TSR
```python
sim.dispatch_ats_operation_command(
    "ADD_TSR",
    "",
    {"start": 300.0, "end": 500.0, "speed": 30.0},
    reason="maintenance"
)
```

### Update TSR Speed
```python
sim.dispatch_ats_operation_command(
    "UPDATE_TSR",
    "",
    {"index": 0, "speed": 25.0},
    reason="weather_change"
)
```

### Remove Specific TSR
```python
sim.dispatch_ats_operation_command(
    "REMOVE_TSR",
    "",
    {"index": 0},
    reason="maintenance_complete"
)
```

### Clear All TSR
```python
sim.dispatch_ats_operation_command(
    "CLEAR_TSR",
    "",
    reason="line_restored"
)
```

---

## GUI Components Affected

### Limits Panel
- Displays all TSR zones
- Click to edit/remove TSR
- Shows current speed restrictions

### Control Panel
- "Issue TSR" button - Add new TSR
- "Clear TSR" button - Remove all TSR

### Train Panel
- Shows current effective speed (PSR + TSR)
- Displays next speed restriction

### Engineering Panel
- Architecture diagram includes TSR/PSR flow

### ATS Overview Panel
- Displays TSR status
- Shows active restrictions

---

## Validation Rules

### YAML Validation (scenario_loader.py)
- TSR start position < end position
- TSR speed > 0
- No duplicate zone IDs

### Runtime Validation (runtime.py)
- Start < end (auto-swap if reversed)
- Speed > 0
- Index within bounds (for update/remove)

---

## Troubleshooting

### TSR zones not appearing in GUI
- Check `Limits Panel` - TSR zones are displayed there
- Verify TSR zones loaded: Run `test_tsr_loading.py`

### TSR speed not applied
- Check that TSR is within train's current position
- Verify TSR speed < PSR (most restrictive applies)
- Restart simulation after loading new scenario

### ZC not generating correct safe packets
- Verify `test_tsr_zc_integration.py` passes
- Check `next_lower_limit()` calculation in signalling.py

---

## Future Enhancements

Potential improvements:
1. Time-based TSR (automatic activation/deactivation)
2. TSR categories (work zone, weather, track issue)
3. TSR persistence across scenario reloads
4. TSR scheduling/timetables
5. TSR conflict detection (overlapping zones)

---

## Version History

- **v1.0** (2025-06):
  - Initial TSR support added
  - YAML loading for initial TSR zones
  - Runtime ADD/UPDATE/REMOVE/CLEAR operations
  - ZC integration with next_lower_limit()
