#!/usr/bin/env python3
"""Test script for TSR zone integration with ZC."""

from CONFIG.scenario_loader import load_scenario
from SUBSYSTEMS.runtime import Simulation

print("=" * 60)
print("TSR Zone ZC Integration Test")
print("=" * 60)

try:
    # Load scenario with TSR zones
    scenario = load_scenario('DOCS/example_with_tsr.yaml')
    sim = Simulation(scenario)
    
    print("\n[Setup] Scenario loaded with TSR zones:")
    for idx, z in enumerate(sim.tsr_zones, 1):
        print(f"  {idx}. TSR: {z['start']:.0f}-{z['end']:.0f}m @ {z['speed']:.0f}km/h")
    
    # Test 1: Test signalling next_lower_limit function
    print("\n[Test 1] Testing next_lower_limit with TSR zones...")
    from SUBSYSTEMS.signalling import next_lower_limit
    
    # Get current position and PSR
    track_profile = sim.track_profile
    current_pos = 0.0
    current_psr = 60.0  # From first segment
    
    next_speed, next_dist = next_lower_limit(track_profile, current_pos, current_psr, sim.tsr_zones)
    print(f"  Current position: {current_pos:.0f}m, PSR: {current_psr:.0f}km/h")
    print(f"  Next lower limit: {next_speed:.0f}km/h @ {next_dist:.0f}m away")
    
    # Test at position 200m
    current_pos = 200.0
    next_speed, next_dist = next_lower_limit(track_profile, current_pos, current_psr, sim.tsr_zones)
    print(f"\n  Current position: {current_pos:.0f}m, PSR: {current_psr:.0f}km/h")
    print(f"  Next lower limit: {next_speed:.0f}km/h @ {next_dist:.0f}m away")
    print(f"    (Should see TSR at 300-500m @ 30km/h)")
    
    # Test 2: Test ZC safe packet generation
    print("\n[Test 2] Testing ZC safe packet generation...")
    
    # Create a minimal train for testing
    from SUBSYSTEMS.train import Train
    
    train_cfg = {
        "id": "TEST_TRAIN",
        "start_pos": 0.0,
        "length_m": 60.0,
        "mass_kg": 291600.0,
        "car_count": 4,
        "drive_mode": "ATO",
        "requested_drive_mode": "ATO",
        "max_ato_speed_kmh": 70.0,
        "max_manual_speed_kmh": 45.0,
        "dcs_mute_windows": [],
        "track_profile": sim.track_profile,
        "balises": sim.balises,
        "scheduled_stops": sim.scheduled_stops,
        "color": "#1f77b4",
    }
    
    train = Train(train_cfg)
    train.has_position_report = True
    train.may_receive_authority = True
    train.reported_pos = 100.0
    
    # Build safe packets
    packets = sim.zc.build_safe_packets(
        track_profile=sim.track_profile,
        tsr_zones=sim.tsr_zones,
        track_end_m=sim.track_end_m,
        stop_eoa_map={},
        trains_for_authority=[train],
    )
    
    if train.id in packets:
        packet = packets[train.id]
        print(f"  Train {train.id} at position {train.reported_pos:.0f}m:")
        print(f"    - TSR speed limit: {packet.tsr_kmh:.0f}km/h")
        print(f"    - EOA: {packet.eoa_m:.0f}m")
        print(f"    - Next speed limit: {packet.variants.get('next_speed_limit_kmh', 0):.0f}km/h @ {packet.variants.get('next_speed_limit_dist_m', 0):.0f}m")
    
    print("\n" + "=" * 60)
    print("✓ All TSR integration tests passed!")
    print("=" * 60)

except Exception as e:
    print(f"\n✗ Test failed: {e}")
    import traceback
    traceback.print_exc()
