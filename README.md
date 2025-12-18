# ICP Point Cloud Registration

This project demonstrates **3D point cloud registration** using the **Iterative Closest Point (ICP)** algorithm in Python with Open3D.

## Features
- Load and visualize 3D meshes (`.obj` format).
- Apply manual transformations (rotation + translation).
- Perform ICP registration to align point clouds.
- Animate the alignment with color-coded visualization:
  - Red: source (transformed)
  - Blue: target (original)
  - Green: final aligned

## Requirements
- Python 3.8+
- Open3D (`pip install open3d`)
- NumPy (`pip install numpy`)

## Usage
1. Place your `.obj` mesh in the folder.
2. Run the script:
```bash
python 3Dannimation.py

