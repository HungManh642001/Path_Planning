#!/usr/bin/env python
"""
Interactive Path Planning GUI Launcher
Simply run: python launch_gui.py
"""

import sys
import os

# Add current directory to path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

def main():
    """Launch the interactive GUI"""
    print("="*60)
    print("🚀 Missile Path Planning - Interactive Scenario Builder")
    print("="*60)
    print("\n📋 Loading GUI modules...")
    
    try:
        from gui_scenario_builder import ScenarioBuilder
        import tkinter as tk
        
        print("✅ All modules loaded successfully")
        print("\n📝 Controls:")
        print("  1. Select launch and target points by clicking buttons")
        print("  2. Draw islands (polygons) and SAM sites (circles)")
        print("  3. Click 'Run Path Planning' to execute algorithm")
        print("  4. View results in new window")
        print("\n🌍 Map size: 500km × 500km")
        print("\n" + "="*60 + "\n")
        
        root = tk.Tk()
        app = ScenarioBuilder(root)
        root.mainloop()
        
    except ImportError as e:
        print(f"\n❌ Import Error: {e}")
        print("\nMake sure these packages are installed:")
        print("  pip install tkinter matplotlib numpy scipy shapely")
        sys.exit(1)
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
