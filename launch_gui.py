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
    print("Missile Path Planning - Interactive Planner")
    try:
        import tkinter as tk
        from gui.app import PlannerApp
    except ImportError as e:
        print(f"Import Error: {e}")
        sys.exit(1)
    root = tk.Tk()
    PlannerApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
