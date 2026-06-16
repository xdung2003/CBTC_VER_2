#!/usr/bin/env python3
"""
Test script để xác minh các scenario chạy nhiều tàu mới
- multi_train_slippery_conditions.yaml
- extreme_conditions_test.yaml
"""

import sys
import os
from pathlib import Path

# Thêm project root vào path
project_root = Path(__file__).parent
sys.path.insert(0, str(project_root))

from CONFIG.scenario_loader import load_scenario, normalize_scenario
import yaml


def test_slippery_conditions_scenario():
    """Kiểm tra scenario điều kiện trơn"""
    print("\n" + "="*70)
    print("TEST: Multi-Train Slippery Conditions Scenario")
    print("="*70)
    
    scenario_path = project_root / "DOCS" / "multi_train_slippery_conditions.yaml"
    
    try:
        scenario_dict = load_scenario(str(scenario_path))
        
        print(f"✅ Scenario loaded successfully: {scenario_dict.get('name')}")
        print(f"   Description: {scenario_dict.get('description', 'N/A')[:60]}...")
        
        # Kiểm tra số tàu
        source_trains = scenario_dict.get('source_trains', [])
        total_trains = sum(st.get('total_trains', 0) for st in source_trains)
        print(f"\n📊 Thông tin tàu:")
        print(f"   Total trains: {total_trains}")
        print(f"   Source: {source_trains[0]['name'] if source_trains else 'N/A'}")
        print(f"   Depot position: {source_trains[0]['start_m']}m" if source_trains else "N/A")
        
        # Kiểm tra headway
        headway = scenario_dict.get('headway', {})
        print(f"\n📏 Thông tin headway:")
        print(f"   Mode: {headway.get('mode', 'N/A')}")
        print(f"   Target: {headway.get('target_headway_s', 'N/A')}s")
        
        # Kiểm tra TSR zones
        tsr_zones = scenario_dict.get('tsr_zones', [])
        print(f"\n⚠️  TSR Zones ({len(tsr_zones)} zones):")
        for i, zone in enumerate(tsr_zones, 1):
            print(f"   Zone {i}: {zone['start_m']:.0f}-{zone['end_m']:.0f}m @ {zone['speed_kmh']}km/h")
            print(f"      Reason: {zone.get('reason', 'N/A')}")
        
        # Kiểm tra track segments
        track = scenario_dict.get('track', {})
        segments = track.get('segments', [])
        print(f"\n🛤️  Track Segments ({len(segments)} segments):")
        for i, seg in enumerate(segments, 1):
            gradient_pct = seg.get('gradient', 0) * 100
            print(f"   Segment {i}: {seg['start_m']:.0f}-{seg['end_m']:.0f}m")
            print(f"      Gradient: {gradient_pct:+.1f}% | PSR: {seg['psr_kmh']}km/h")
        
        # Kiểm tra stations
        stops = scenario_dict.get('scheduled_stops', [])
        print(f"\n🏢 Scheduled Stops ({len(stops)} stations):")
        for stop in stops:
            print(f"   {stop['name']}: {stop['pos_m']:.0f}m (capacity: {stop['capacity']})")
        
        print("\n✅ Scenario structure is valid!")
        return True
        
    except Exception as e:
        print(f"❌ Error loading scenario: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_extreme_conditions_scenario():
    """Kiểm tra scenario điều kiện cực đoan"""
    print("\n" + "="*70)
    print("TEST: Extreme Conditions Multi-Train Scenario")
    print("="*70)
    
    scenario_path = project_root / "DOCS" / "extreme_conditions_test.yaml"
    
    try:
        scenario_dict = load_scenario(str(scenario_path))
        
        print(f"✅ Scenario loaded successfully: {scenario_dict.get('name')}")
        print(f"   Description: {scenario_dict.get('description', 'N/A')[:80]}...")
        
        # Kiểm tra khối lượng tàu
        train_defaults = scenario_dict.get('train_defaults', {})
        mass_kg = train_defaults.get('mass_kg', 0)
        print(f"\n📊 Thông tin tàu:")
        print(f"   Mass: {mass_kg:,.0f} kg (crush load)")
        print(f"   Length: {train_defaults.get('length_m')}m")
        print(f"   Drive mode: {train_defaults.get('drive_mode')}")
        print(f"   Max ATO speed: {train_defaults.get('max_ato_speed_kmh')}km/h")
        
        source_trains = scenario_dict.get('source_trains', [])
        total_trains = sum(st.get('total_trains', 0) for st in source_trains)
        print(f"   Total trains: {total_trains}")
        
        # Kiểm tra headway
        headway = scenario_dict.get('headway', {})
        print(f"\n📏 Headway tighter:")
        print(f"   Target: {headway.get('target_headway_s', 'N/A')}s (very close)")
        
        # Kiểm tra TSR zones - nhiều và giới hạn chặt
        tsr_zones = scenario_dict.get('tsr_zones', [])
        print(f"\n⚠️  TSR Zones - Rất nhiều giới hạn ({len(tsr_zones)} zones):")
        for i, zone in enumerate(tsr_zones, 1):
            print(f"   Zone {i}: {zone['start_m']:.0f}-{zone['end_m']:.0f}m @ {zone['speed_kmh']}km/h")
        
        # Kiểm tra line conditions
        line_cond = scenario_dict.get('line_conditions', [])
        print(f"\n🌧️  Line Conditions - Extreme ({len(line_cond)} zones):")
        for cond in line_cond:
            r = cond.get('range', {})
            fc = cond.get('friction_coefficient', 'N/A')
            weather = cond.get('weather', 'N/A')
            print(f"   {r['start_m']:.0f}-{r['end_m']:.0f}m: {weather} | Friction: {fc}")
        
        # Tìm friction coefficient thấp nhất
        if line_cond:
            min_friction = min(c.get('friction_coefficient', 1.0) for c in line_cond)
            print(f"\n   ⚠️  Min friction coefficient: {min_friction:.2f} (very slippery!)")
        
        print("\n✅ Scenario structure is valid!")
        return True
        
    except Exception as e:
        print(f"❌ Error loading scenario: {e}")
        import traceback
        traceback.print_exc()
        return False


def compare_scenarios():
    """So sánh các scenario"""
    print("\n" + "="*70)
    print("COMPARISON: Slippery vs Extreme Conditions")
    print("="*70)
    
    # Load both scenarios
    slippery_path = project_root / "DOCS" / "multi_train_slippery_conditions.yaml"
    extreme_path = project_root / "DOCS" / "extreme_conditions_test.yaml"
    
    slippery = load_scenario(str(slippery_path))
    extreme = load_scenario(str(extreme_path))
    
    comparison_data = [
        ("Scenario Name", slippery['name'], extreme['name']),
        ("Total Trains", 
         sum(t.get('total_trains', 0) for t in slippery.get('source_trains', [])),
         sum(t.get('total_trains', 0) for t in extreme.get('source_trains', []))),
        ("Headway (s)", 
         slippery['headway']['target_headway_s'],
         extreme['headway']['target_headway_s']),
        ("Train Mass (kg)", 
         slippery['train_defaults']['mass_kg'],
         extreme['train_defaults']['mass_kg']),
        ("TSR Zones Count",
         len(slippery.get('tsr_zones', [])),
         len(extreme.get('tsr_zones', []))),
        ("Max Gradient",
         f"+5% / -6%",
         f"+5% / -7%"),
        ("Min Friction Coeff",
         "0.30",
         "0.25"),
    ]
    
    print("\n{:<25} {:<30} {:<30}".format("Parameter", "Slippery", "Extreme"))
    print("-" * 85)
    for param, val1, val2 in comparison_data:
        print("{:<25} {:<30} {:<30}".format(param, str(val1), str(val2)))
    
    print("\n📊 Analysis:")
    print("   ✓ Slippery: Đa dạng điều kiện, phù hợp test chạy normal")
    print("   ✓ Extreme: Kiểm tra giới hạn hệ thống, test an toàn tối ưu")


def main():
    """Main test runner"""
    print("\n" + "="*70)
    print("CBTC_SIM: Multi-Train Scenario Validation Test Suite")
    print("="*70)
    
    results = []
    
    # Test slippery scenario
    results.append(("Slippery Conditions", test_slippery_conditions_scenario()))
    
    # Test extreme scenario
    results.append(("Extreme Conditions", test_extreme_conditions_scenario()))
    
    # Compare
    try:
        compare_scenarios()
    except Exception as e:
        print(f"\n⚠️  Comparison failed: {e}")
    
    # Summary
    print("\n" + "="*70)
    print("TEST SUMMARY")
    print("="*70)
    
    all_passed = True
    for test_name, result in results:
        status = "✅ PASS" if result else "❌ FAIL"
        print(f"{status}: {test_name}")
        all_passed = all_passed and result
    
    if all_passed:
        print("\n🎉 All tests passed! Scenarios are ready to use.")
        print("\nTo run these scenarios:")
        print("   1. Start the GUI: python GUI/main_gui.py")
        print("   2. Load scenario: File → Open → multi_train_slippery_conditions.yaml")
        print("   3. Or: File → Open → extreme_conditions_test.yaml")
    else:
        print("\n❌ Some tests failed. Please check the errors above.")
    
    return 0 if all_passed else 1


if __name__ == "__main__":
    sys.exit(main())
