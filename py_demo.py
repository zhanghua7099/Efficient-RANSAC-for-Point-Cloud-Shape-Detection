import math

import numpy as np
import open3d as o3d
import pc_ransac


K_PI = math.pi
K_NOISE_SIGMA = 0.01
K_OUTLIER_COUNT = 800

# 对应 C++:
# pc.setBBox(Vec3f(-11.0f, -8.0f, -3.0f), Vec3f(11.0f, 8.0f, 6.0f));
BBOX_MIN = np.asarray([-11.0, -8.0, -3.0], dtype=np.float32)
BBOX_MAX = np.asarray([11.0, 8.0, 6.0], dtype=np.float32)
BBOX = [BBOX_MIN.tolist(), BBOX_MAX.tolist()]


def gaussian(rng, sigma):
    if sigma <= 0.0:
        return 0.0
    return float(rng.normal(0.0, sigma))


def uniform(rng, lo, hi):
    return float(rng.uniform(lo, hi))


def add_point(points, labels, rng, x, y, z, label):
    nx = x + gaussian(rng, K_NOISE_SIGMA)
    ny = y + gaussian(rng, K_NOISE_SIGMA)
    nz = z + gaussian(rng, K_NOISE_SIGMA)

    points.append([nx, ny, nz])
    labels.append(label)


def add_plane(points, labels, rng):
    n = 40
    min_coord = -5.0
    max_coord = 5.0

    for ix in range(n + 1):
        x = min_coord + (max_coord - min_coord) * ix / n

        for iy in range(n + 1):
            y = min_coord + (max_coord - min_coord) * iy / n
            add_point(points, labels, rng, x, y, -2.0, "plane")


def add_sphere(points, labels, rng):
    cx, cy, cz = -7.0, 3.0, 2.5
    r = 1.5

    n_theta = 80
    n_phi = 40

    for ip in range(1, n_phi):
        phi = K_PI * ip / n_phi
        sin_phi = math.sin(phi)
        cos_phi = math.cos(phi)

        for it in range(n_theta):
            theta = 2.0 * K_PI * it / n_theta

            x = cx + r * sin_phi * math.cos(theta)
            y = cy + r * sin_phi * math.sin(theta)
            z = cz + r * cos_phi

            add_point(points, labels, rng, x, y, z, "sphere")


def add_cylinder(points, labels, rng):
    cx, cy = 7.0, 3.0
    r = 1.2
    z0, z1 = 0.3, 4.3

    n_theta = 80
    n_z = 60

    for iz in range(n_z + 1):
        z = z0 + (z1 - z0) * iz / n_z

        for it in range(n_theta):
            theta = 2.0 * K_PI * it / n_theta

            x = cx + r * math.cos(theta)
            y = cy + r * math.sin(theta)

            add_point(points, labels, rng, x, y, z, "cylinder")


def add_cone(points, labels, rng):
    cx, cy = -7.0, -4.0
    base_z = 0.5
    apex_z = 4.5
    base_radius = 1.5
    height = apex_z - base_z

    n_theta = 80
    n_h = 60

    for ih in range(n_h + 1):
        t = 0.92 * ih / n_h
        z = base_z + height * t
        r = base_radius * (1.0 - t)

        for it in range(n_theta):
            theta = 2.0 * K_PI * it / n_theta

            x = cx + r * math.cos(theta)
            y = cy + r * math.sin(theta)

            add_point(points, labels, rng, x, y, z, "cone")


def add_outliers(points, labels, rng):
    for _ in range(K_OUTLIER_COUNT):
        add_point(
            points,
            labels,
            rng,
            uniform(rng, -10.0, 10.0),
            uniform(rng, -7.0, 7.0),
            uniform(rng, -2.5, 5.5),
            "outlier",
        )


def build_synthetic_point_cloud(seed=42):
    rng = np.random.default_rng(seed)

    points = []
    labels = []

    add_plane(points, labels, rng)
    add_sphere(points, labels, rng)
    add_cylinder(points, labels, rng)
    add_cone(points, labels, rng)
    add_outliers(points, labels, rng)

    # binding 要求 C-contiguous float32 或 float64，显式保证一下
    points = np.asarray(points, dtype=np.float32)
    points = np.ascontiguousarray(points)

    return points, labels


def point_cloud_scale_from_bbox(bbox_min, bbox_max):
    # 通常 PointCloud::getScale() 对应 bbox 对角线长度
    return float(np.linalg.norm(bbox_max - bbox_min))


def make_detect_options_like_cpp():
    scale = point_cloud_scale_from_bbox(BBOX_MIN, BBOX_MAX)

    options = pc_ransac.DetectOptions()

    # 对应 C++:
    # ransacOptions.m_epsilon = 0.0025f * pc.getScale();
    # ransacOptions.m_bitmapEpsilon = 0.01f * pc.getScale();
    options.epsilon = 0.0025 * scale
    options.bitmap_epsilon = 0.01 * scale

    # 对应 C++:
    # ransacOptions.m_normalThresh = 0.85f;
    # ransacOptions.m_minSupport = 200;
    # ransacOptions.m_probability = 0.001f;
    options.normal_thresh = 0.85
    options.min_support = 200
    options.probability = 0.001

    # 对应 binding:
    # ransac_options.m_fitting =
    #     options.fitting ? LS_FITTING : NO_FITTING;
    options.fitting = True

    # 对应 C++:
    # pc.calcNormals(0.55f, 20, 100);
    #
    # 注意：
    # binding 里如果 normals=None 且 calculate_normals=True，
    # 会在 detect() 内部调用 pc.calcNormals(...)
    options.calculate_normals = True
    options.normal_radius = 0.55
    options.normal_k = 20
    options.normal_max_tries = 100

    # primitive constructors
    options.detect_planes = True
    options.detect_spheres = True
    options.detect_cylinders = True
    options.detect_cones = True
    options.detect_tori = False

    return options


def make_open3d_point_cloud(points, color):
    points = np.asarray(points, dtype=np.float64)

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    colors = np.tile(np.asarray(color, dtype=np.float64), (len(points), 1))
    pcd.colors = o3d.utility.Vector3dVector(colors)

    return pcd


def visualize_detected_result(result):
    """
    直接使用 binding 返回的:
      - shape.points
      - result.remaining_points

    不再用原始 points[shape.indices] 反查。
    """

    color_table = {
        "plane": [1.0, 0.2, 0.2],
        "sphere": [0.2, 0.8, 0.2],
        "cylinder": [0.2, 0.4, 1.0],
        "cone": [1.0, 0.8, 0.2],
        "torus": [0.8, 0.2, 1.0],
        "unknown": [1.0, 1.0, 1.0],
    }

    geometries = []

    for shape_index, shape in enumerate(result.shapes):
        shape_points = np.asarray(shape.points, dtype=np.float32)

        if len(shape_points) == 0:
            continue

        color = color_table.get(shape.kind, color_table["unknown"])
        pcd = make_open3d_point_cloud(shape_points, color)

        geometries.append(pcd)

        print(
            f"visualize shape {shape_index}"
            f" | kind = {shape.kind}"
            f" | support = {shape.support}"
            f" | identifier = {shape.identifier}"
            f" | color = {color}"
        )

    remaining_points = np.asarray(result.remaining_points, dtype=np.float32)

    if len(remaining_points) > 0:
        remaining_pcd = make_open3d_point_cloud(
            remaining_points,
            [0.45, 0.45, 0.45],
        )
        geometries.append(remaining_pcd)

        print(f"visualize remaining points | count = {len(remaining_points)}")

    axis = o3d.geometry.TriangleMesh.create_coordinate_frame(
        size=2.0,
        origin=[0.0, 0.0, 0.0],
    )
    geometries.append(axis)

    o3d.visualization.draw_geometries(
        geometries,
        window_name="pc_ransac detected primitives",
        width=1280,
        height=800,
        point_show_normal=False,
    )


def visualize_original_labels(points, labels):
    color_table = {
        "plane": [1.0, 0.2, 0.2],
        "sphere": [0.2, 0.8, 0.2],
        "cylinder": [0.2, 0.4, 1.0],
        "cone": [1.0, 0.8, 0.2],
        "outlier": [0.45, 0.45, 0.45],
    }

    colors = np.asarray(
        [color_table.get(label, [1.0, 1.0, 1.0]) for label in labels],
        dtype=np.float64,
    )

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points.astype(np.float64))
    pcd.colors = o3d.utility.Vector3dVector(colors)

    axis = o3d.geometry.TriangleMesh.create_coordinate_frame(
        size=2.0,
        origin=[0.0, 0.0, 0.0],
    )

    o3d.visualization.draw_geometries(
        [pcd, axis],
        window_name="original synthetic point cloud labels",
        width=1280,
        height=800,
        point_show_normal=False,
    )


def print_detection_result(result):
    print()
    print("Detection result")
    print(f"Remaining unassigned point count: {result.remaining}")
    print(f"Remaining indices count: {len(result.remaining_indices)}")
    print(f"Remaining points count: {len(result.remaining_points)}")
    print(f"Detected shape count: {len(result.shapes)}")

    found_count = {}

    for i, shape in enumerate(result.shapes):
        found_count[shape.kind] = found_count.get(shape.kind, 0) + 1

        print(
            f"shape {i}"
            f" | kind = {shape.kind}"
            f" | support = {shape.support}"
            f" | identifier = {shape.identifier}"
        )
        print(f"  description: {shape.description}")
        print(f"  parameters: {shape.parameters}")
        print(f"  first indices: {list(shape.indices[:10])}")

    print()
    print("Summary")

    for name in ["plane", "sphere", "cylinder", "cone", "torus", "unknown"]:
        n = found_count.get(name, 0)
        if n > 0:
            print(f"  {name}: {n}")

    expected = ["plane", "sphere", "cylinder", "cone"]
    ok = all(found_count.get(name, 0) >= 1 for name in expected)

    if ok:
        print()
        print("PASS: all expected primitive types were detected at least once.")
    else:
        print()
        print("WARN: at least one expected primitive type was not detected.")
        print(
            "Try tuning: epsilon, bitmap_epsilon, normal_thresh, "
            "min_support, normal_radius, or normal_k."
        )


def detect_with_bbox(points, options):
    """
    binding 的 C++ 签名是:
        detect(points, options, normals, bbox)

    nanobind 通常支持关键字 bbox。
    如果你的构建不支持关键字调用，可以改成：
        pc_ransac.detect(points, options, None, BBOX)
    """

    return pc_ransac.detect(
        points,
        options,
        normals=None,
        bbox=BBOX,
    )


def main():
    points, labels = build_synthetic_point_cloud(seed=42)

    scale = point_cloud_scale_from_bbox(BBOX_MIN, BBOX_MAX)

    print()
    print(f"Total points: {len(points)}")
    print(f"Point cloud scale from explicit bbox: {scale}")
    print(f"BBox min: {BBOX_MIN.tolist()}")
    print(f"BBox max: {BBOX_MAX.tolist()}")

    options = make_detect_options_like_cpp()

    print()
    print("DetectOptions")
    print(f"  epsilon: {options.epsilon}")
    print(f"  bitmap_epsilon: {options.bitmap_epsilon}")
    print(f"  normal_thresh: {options.normal_thresh}")
    print(f"  min_support: {options.min_support}")
    print(f"  probability: {options.probability}")
    print(f"  fitting: {options.fitting}")
    print(f"  calculate_normals: {options.calculate_normals}")
    print(f"  normal_radius: {options.normal_radius}")
    print(f"  normal_k: {options.normal_k}")
    print(f"  normal_max_tries: {options.normal_max_tries}")
    print(f"  detect_planes: {options.detect_planes}")
    print(f"  detect_spheres: {options.detect_spheres}")
    print(f"  detect_cylinders: {options.detect_cylinders}")
    print(f"  detect_cones: {options.detect_cones}")
    print(f"  detect_tori: {options.detect_tori}")

    result = detect_with_bbox(points, options)

    print_detection_result(result)

    # 可选：先看原始合成数据的真实标签
    # visualize_original_labels(points, labels)

    # 看 RANSAC 检测结果
    visualize_detected_result(result)


if __name__ == "__main__":
    main()