#!/usr/bin/env python3
"""Test script for TSR zone loading from YAML."""

from CONFIG.scenario_loader import load_scenario

print("=" * 60)
print("TSR Zone Loading Test")
print("=" * 60)

# Test 1: Default scenario (no TSR)
print("\n[Test 1] Loading default_scenario.yaml...")
try:
    scenario = load_scenario('DOCS/default_scenario.yaml')
    print("✓ Scenario loaded successfully")
    print(f"  - Track segments: {len(scenario['track_profile'])}")
    print(f"  - TSR zones: {len(scenario['tsr_zones'])}")
except Exception as e:
    print(f"✗ Failed: {e}")

# Test 2: Example with TSR
print("\n[Test 2] Loading example_with_tsr.yaml...")
try:
    scenario = load_scenario('DOCS/example_with_tsr.yaml')
    print("✓ Scenario loaded successfully")
    print(f"  - Track segments: {len(scenario['track_profile'])}")
    print(f"  - TSR zones: {len(scenario['tsr_zones'])}")
    for idx, z in enumerate(scenario['tsr_zones'], 1):
        start = z['start']
        end = z['end']
        speed = z['speed']
        print(f"    {idx}. TSR: {start:.0f}-{end:.0f}m @ {speed:.0f}km/h")
except Exception as e:
    print(f"✗ Failed: {e}")

# Test 3: Simulate creating a simulation with TSR zones
print("\n[Test 3] Creating Simulation with TSR zones...")
try:
    from SUBSYSTEMS.runtime import Simulation
    
    scenario = load_scenario('DOCS/example_with_tsr.yaml')
    sim = Simulation(scenario)
    
    print("✓ Simulation created successfully")
    print(f"  - TSR zones in simulation: {len(sim.tsr_zones)}")
    for idx, z in enumerate(sim.tsr_zones, 1):
        start = z['start']
        end = z['end']
        speed = z['speed']
        print(f"    {idx}. TSR: {start:.0f}-{end:.0f}m @ {speed:.0f}km/h")
except Exception as e:
    print(f"✗ Failed: {e}")
    import traceback
    traceback.print_exc()

print("\n" + "=" * 60)
print("Test Complete")
print("=" * 60)
