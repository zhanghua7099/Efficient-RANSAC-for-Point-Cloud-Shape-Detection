# Efficient-RANSAC-for-Point-Cloud-Shape-Detection
Please note that this software is not my own work, this repository is just to try a Linux build of the original software.

As you can see in the [ReadMe.txt](ReadMe.txt) this software is written by Ruwen Schnabel and Roland Wahl:

    Copyright 2009 Ruwen Schnabel (schnabel@cs.uni-bonn.de),
                   Roland Wahl (wahl@cs.uni-bonn.de).

    This software may be used for research purposes only.


I downloaded it from https://cg.cs.uni-bonn.de/aigaion2root/attachments/Software%20v1.1.zip and it accompains the following paper:

Ruwen Schnabel, Roland Wahl, and Reinhard Klein
"Efficient RANSAC for Point-Cloud Shape Detection"
*In: Computer Graphics Forum (June 2007), 26:2(214-226)*
http://cg.cs.uni-bonn.de/en/publications/paper-details/schnabel-2007-efficient/


使用这个库的其他项目
https://github.com/LiangliangNan/Easy3D/tree/main/3rd_party/ransac
https://github.com/ihmcrobotics/ihmc-open-robotics-software/tree/develop/ihmc-sensor-processing/csrc/ransac_schnabel

## Python bindings

This fork also provides optional Python bindings built with
[nanobind](https://github.com/wjakob/nanobind). Python 3.8 builds require
`nanobind>=2.0,<2.10`; newer nanobind releases require Python 3.9 or newer.

For a browser-assisted install page, open [docs/install.html](docs/install.html)
and copy the one-command installer.

```bash
python -m pip install .
```

or build the extension with CMake:

```bash
cmake -S . -B build-python -DPC_RANSAC_BUILD_PYTHON=ON
cmake --build build-python
```

Minimal usage:

```python
import numpy as np
import pc_ransac

points = np.asarray(point_array, dtype=np.float32)  # shape: (N, 3)

options = pc_ransac.DetectOptions()
options.min_support = 200
options.epsilon = -1.0          # auto: 0.01 * point-cloud scale
options.bitmap_epsilon = -1.0   # auto: 0.02 * point-cloud scale
options.detect_tori = False

result = pc_ransac.detect(points, options)

print(result.remaining)
for shape in result.shapes:
    print(shape.kind, shape.support, shape.description)
    print(shape.indices[:10])
```

`detect()` accepts `float32` or `float64` NumPy arrays with shape `(N, 3)`.
Optional `normals=` can provide a matching `(N, 3)` normal array; otherwise
normals are estimated in C++ when `options.calculate_normals` is true. Optional
`bbox=` can be either `[xmin, xmax, ymin, ymax, zmin, zmax]` or
`[[xmin, ymin, zmin], [xmax, ymax, zmax]]`.
