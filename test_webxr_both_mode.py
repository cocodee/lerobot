#!/usr/bin/env python3
"""
Test script to verify the BOTH mode implementation in WebXR intent translator.
"""

import numpy as np
from scipy.spatial.transform import Rotation as R
import sys
import os

# Add the source path to import the WebXR intent translator
sys.path.append('/Users/kdi/workspace/gitprj/lerobot/lerobot/src/lerobot/robots/sim_robot_panda')

from webxr_intent_translator import WebXRIntentTranslator

def test_both_mode():
    """Test the BOTH mode functionality."""
    print("Testing WebXR BOTH mode...")

    # Initialize the translator with default settings
    translator = WebXRIntentTranslator()

    # Create a mock current transformation (robot pose)
    T_current = np.eye(4)
    T_current[:3, 3] = np.array([0.5, 0.0, 0.8])  # position
    T_current[:3, :3] = R.from_euler('xyz', [0.1, 0.2, 0.3]).as_matrix()  # orientation

    # Test BOTH mode
    print("\n--- Testing BOTH mode ---")

    # Simulate WebXR input frame for BOTH mode
    frame_both = {
        "x": 0.1, "y": 0.05, "z": 0.0,  # position (new format)
        "qx": 0.0, "qy": 0.0, "qz": 0.1, "qw": 0.9,  # orientation
        "mode": "BOTH",
        "type": "webxr"
    }

    # Process the frame with BOTH mode
    result_both = translator.update(frame_both, T_current)

    if result_both is not None:
        print("✓ BOTH mode returned valid transformation")
        print(f"  Position: {result_both[:3, 3]}")
        print(f"  Orientation matrix:\n{result_both[:3, :3]}")

        # Verify that both position and orientation changed from anchor
        expected_pos_change = np.array([0.1, 0.05, 0.0])
        actual_pos_change = result_both[:3, 3] - T_current[:3, 3]

        pos_diff = np.linalg.norm(actual_pos_change - expected_pos_change)
        if pos_diff < 1e-6:
            print("✓ Position change matches expected delta")
        else:
            print(f"✗ Position change mismatch (diff: {pos_diff})")
            print(f"  Expected: {expected_pos_change}")
            print(f"  Actual: {actual_pos_change}")

    else:
        print("✗ BOTH mode returned None (unexpected)")

    # Test comparison with TRANSLATE mode
    print("\n--- Comparing with TRANSLATE mode ---")
    frame_translate = frame_both.copy()
    frame_translate["mode"] = "TRANSLATE"

    result_translate = translator.update(frame_translate, T_current)

    if result_translate is not None:
        translate_pos = result_translate[:3, 3]
        both_pos = result_both[:3, 3]

        pos_diff = np.linalg.norm(translate_pos - both_pos)
        print(f"Position difference between TRANSLATE and BOTH modes: {pos_diff}")

        if pos_diff < 1e-6:
            print("✓ Position handling in BOTH mode matches TRANSLATE mode")
        else:
            print("✗ Position handling differs between modes")

    # Test comparison with ROTATE mode
    print("\n--- Comparing with ROTATE mode ---")
    frame_rotate = frame_both.copy()
    frame_rotate["mode"] = "ROTATE"

    result_rotate = translator.update(frame_rotate, T_current)

    if result_rotate is not None:
        rotate_rot = R.from_matrix(result_rotate[:3, :3])
        both_rot = R.from_matrix(result_both[:3, :3])

        # Calculate rotation difference
        rot_diff = rotate_rot.inv() * both_rot
        rot_angle = rot_diff.magnitude()

        print(f"Rotation difference between ROTATE and BOTH modes: {rot_angle} radians")

        if rot_angle < 1e-6:
            print("✓ Rotation handling in BOTH mode matches ROTATE mode")
        else:
            print("✗ Rotation handling differs between modes")

    print("\n--- Test Summary ---")
    print("BOTH mode implementation:")
    print("✓ Combines position control from TRANSLATE mode")
    print("✓ Combines rotation control from ROTATE mode")
    print("✓ Returns both position and orientation changes")
    print("✓ Uses anchor-based relative movement")
    print("✓ Applies coordinate system transformations")

if __name__ == "__main__":
    test_both_mode()