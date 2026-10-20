"""The data files (URDF files, meshes and textures) of the tiny differentiable
simulator, installed by setup.py as the pytinydiffsim_data package."""

import os


def get_data_path():
  return os.path.dirname(os.path.abspath(__file__))
