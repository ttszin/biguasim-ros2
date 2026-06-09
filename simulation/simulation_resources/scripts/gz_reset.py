"""
Resets the Gazebo simulation world or teleports a specific drone model.

Usage:
    # Reset the entire world (position, physics, simulation time):
    python3 gz_reset.py --world

    # Teleport a specific drone model (Gazebo model only, does NOT reset PX4 EKF):
    python3 gz_reset.py --model x500_0 --x 0 --y 0 --z 0.2
"""
import os
import argparse
import gz.transport13
from gz.msgs10.world_control_pb2 import WorldControl
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.boolean_pb2 import Boolean as GzBoolean


def reset_world(gz_node, world_name):
    req = WorldControl()
    req.reset.all = True

    result, _ = gz_node.request(
        f"/world/{world_name}/control",
        req,
        WorldControl,
        GzBoolean,
        2000,
    )
    if result:
        print(f"World '{world_name}' reset successfully.")
    else:
        print("World reset failed — is Gazebo running?")


def set_pose(gz_node, world_name, model_name, x, y, z):
    req = Pose()
    req.name = model_name
    req.position.x = x
    req.position.y = y
    req.position.z = z
    req.orientation.w = 1.0
    req.orientation.x = 0.0
    req.orientation.y = 0.0
    req.orientation.z = 0.0

    result, _ = gz_node.request(
        f"/world/{world_name}/set_pose",
        req,
        Pose,
        GzBoolean,
        2000,
    )
    if result:
        print(f"Model '{model_name}' moved to ({x}, {y}, {z}).")
    else:
        print("set_pose failed — check model name and world name.")


def main():
    parser = argparse.ArgumentParser(description="Reset Gazebo world or teleport a model")
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--world", action="store_true", help="Reset entire world (position + physics + time)")
    group.add_argument("--model", type=str, metavar="MODEL_NAME", help="Teleport this model (e.g. x500_0)")
    parser.add_argument("--x", type=float, default=0.0)
    parser.add_argument("--y", type=float, default=0.0)
    parser.add_argument("--z", type=float, default=0.2)
    args = parser.parse_args()

    world_name = os.environ.get("WORLD", "default")
    gz_node = gz.transport13.Node()

    if args.world:
        reset_world(gz_node, world_name)
    else:
        set_pose(gz_node, world_name, args.model, args.x, args.y, args.z)


if __name__ == "__main__":
    main()
