# MuJoCo model resources

This directory contains the shared MJCF world scene and generated MuJoCo compatible robot descriptions used by the RoboPlan MuJoCo examples.

The canonical robot descriptions are present in the `*_robot_model` directories. `prepare_urdf.py` derives the files under `mujoco/<robot>/` by expanding Xacro, converting unsupported meshes, preserving materials, and normalizing geometry for MuJoCo.

The examples load the same prepared URDF into both RoboPlan and MuJoCo so their kinematic representations remain aligned.