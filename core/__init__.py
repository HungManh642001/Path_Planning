"""Planning pipeline: scenario generation, preprocessing, search and validation.

The modules form a strict one-directional pipeline. ``config`` holds the tunable
constants, ``types`` the shared type vocabulary, and nothing here imports from
``render`` or ``gui``.
"""
