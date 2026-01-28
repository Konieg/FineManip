"""IKPy demo using exact link offsets/rotations from scene-4.xml.

Usage:
  python ikpy_demo.py --target 0.2 0.1 0.25 --plot
"""
from __future__ import annotations

import argparse
import math
from typing import List

import numpy as np
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink


def build_chain() -> Chain:
    # Values taken from scene-4.xml (link body pos/euler, joint axes).
    return Chain(
        name="scene4_arm",
        links=[
            OriginLink(),
            URDFLink(
                name="joint1",
                origin_translation=[0.0, 0.0, 0.08840],
                origin_orientation=[0.0, 0.0, 0.0],
                rotation=[0, 0, -1],
                bounds=(-3.14, 3.14),
            ),
            URDFLink(
                name="joint2",
                origin_translation=[0.06, 0.0, 0.078],
                origin_orientation=[1.5708, 0.0, 0.0],
                rotation=[0, 0, 1],
                bounds=(-3.14, 3.14),
            ),
            URDFLink(
                name="joint3",
                origin_translation=[0.0, 0.18, 0.0],
                origin_orientation=[0.0, 0.0, 1.5708],
                rotation=[0, 0, 1],
                bounds=(-3.14, 3.14),
            ),
            URDFLink(
                name="joint4",
                origin_translation=[0.06, 0.0, 0.0],
                origin_orientation=[1.5708, 0.0, 0.0],
                rotation=[0, 0, -1],
                bounds=(-3.14, 3.14),
            ),
            URDFLink(
                name="joint5",
                origin_translation=[0.0, 0.0, 0.151],
                origin_orientation=[-1.5708, 0.0, -3.1416],
                rotation=[0, 0, 1],
                bounds=(-3.14, 3.14),
            ),
            URDFLink(
                name="joint6",
                origin_translation=[0.0, 0.0, 0.0],
                origin_orientation=[-1.5708, 0.0, 3.1416],
                rotation=[0, 0, 1],
                bounds=(-3.14, 3.14),
            ),
            URDFLink(
                name="link_hand",
                origin_translation=[0.0, 0.0, 0.11564],
                origin_orientation=[1.5708, 1.5708, 0.0],
                rotation=None,
                joint_type="fixed",
            ),
        ],
    )


def _plot_chain(chain: Chain, joints: List[float]) -> None:
    try:
        import matplotlib.pyplot as plt
    except Exception as exc:  # pragma: no cover - best effort
        print(f"matplotlib not available: {exc}")
        return

    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")

    transforms = chain.forward_kinematics(joints, full_kinematics=True)
    xs = [t[0, 3] for t in transforms]
    ys = [t[1, 3] for t in transforms]
    zs = [t[2, 3] for t in transforms]

    ax.plot(xs, ys, zs, marker="o")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title("IKPy Chain Visualization")
    ax.set_box_aspect([1, 1, 1])
    plt.show()


def main() -> None:
    parser = argparse.ArgumentParser(description="IKPy demo for scene-4.xml arm")
    parser.add_argument("--target", nargs=3, type=float, default=[0.2, 0.1, 0.25])
    parser.add_argument("--no-plot", action="store_true", help="disable 3D plot")
    parser.add_argument("--deg", action="store_true", help="print angles in degrees")
    args = parser.parse_args()

    chain = build_chain()
    target_pos = np.array(args.target, dtype=float)

    q = chain.inverse_kinematics(target_pos)

    if args.deg:
        q_disp = np.degrees(q)
        unit = "deg"
    else:
        q_disp = q
        unit = "rad"

    print("IK result ({}):".format(unit), np.round(q_disp, 4))
    print("Joint1~6 ({}):".format(unit), np.round(q_disp[1:7], 4))

    fk = chain.forward_kinematics(q)
    print("FK position:", np.round(fk[:3, 3], 4))

    if not args.no_plot:
        _plot_chain(chain, q)


if __name__ == "__main__":
    main()
