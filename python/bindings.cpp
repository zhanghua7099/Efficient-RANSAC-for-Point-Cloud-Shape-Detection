#include <ConePrimitiveShapeConstructor.h>
#include <CylinderPrimitiveShapeConstructor.h>
#include <PlanePrimitiveShapeConstructor.h>
#include <PointCloud.h>
#include <PrimitiveShape.h>
#include <RansacShapeDetector.h>
#include <SpherePrimitiveShapeConstructor.h>
#include <TorusPrimitiveShapeConstructor.h>

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/array.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace nb = nanobind;

namespace
{
using Array2D = nb::ndarray<nb::numpy, nb::ndim<2>, nb::c_contig>;

struct DetectOptions
{
    float epsilon = -1.0f;
    float bitmap_epsilon = -1.0f;
    float normal_thresh = 0.9f;
    unsigned int min_support = 100;
    float probability = 0.001f;
    bool fitting = true;
    bool calculate_normals = true;
    float normal_radius = -1.0f;
    unsigned int normal_k = 20;
    unsigned int normal_max_tries = 100;
    bool detect_planes = true;
    bool detect_spheres = true;
    bool detect_cylinders = true;
    bool detect_cones = true;
    bool detect_tori = false;
};

struct DetectedShape
{
    std::string kind;
    std::string description;
    std::size_t identifier = 0;
    std::size_t support = 0;
    std::vector<float> parameters;
    std::vector<std::size_t> indices;
    std::vector<std::array<float, 3>> points;
};

struct DetectionResult
{
    std::size_t remaining = 0;
    std::vector<std::size_t> remaining_indices;
    std::vector<std::array<float, 3>> remaining_points;
    std::vector<DetectedShape> shapes;
};

std::string to_lower(std::string s)
{
    std::transform(
        s.begin(),
        s.end(),
        s.begin(),
        [](unsigned char c)
        {
            return static_cast<char>(std::tolower(c));
        });

    return s;
}

std::string primitive_kind_from_description(const std::string& description)
{
    const std::string d = to_lower(description);

    if (d.find("plane") != std::string::npos)
        return "plane";
    if (d.find("sphere") != std::string::npos)
        return "sphere";
    if (d.find("cylinder") != std::string::npos)
        return "cylinder";
    if (d.find("cone") != std::string::npos)
        return "cone";
    if (d.find("torus") != std::string::npos)
        return "torus";

    return "unknown";
}

Vec3f vec3_from_array(const std::array<float, 3>& v)
{
    return Vec3f(v[0], v[1], v[2]);
}

std::array<float, 3> array_from_vec3(const Vec3f& v)
{
    return {v[0], v[1], v[2]};
}

void validate_xyz_array(const Array2D& array, const char* name)
{
    if (!array.is_valid())
        throw std::invalid_argument(std::string(name) + " must be an array");

    if (array.shape(1) != 3)
        throw std::invalid_argument(std::string(name) + " must have shape (N, 3)");

    const nb::dlpack::dtype dt = array.dtype();
    if (dt.code != static_cast<uint8_t>(nb::dlpack::dtype_code::Float) ||
        (dt.bits != 32 && dt.bits != 64) ||
        dt.lanes != 1)
    {
        throw std::invalid_argument(std::string(name) + " must contain float32 or float64 values");
    }
}

std::array<float, 3> read_xyz_row(const Array2D& array, std::size_t row)
{
    if (array.dtype().bits == 32)
    {
        const auto* data = static_cast<const float*>(array.data());
        return {data[row * 3 + 0], data[row * 3 + 1], data[row * 3 + 2]};
    }

    const auto* data = static_cast<const double*>(array.data());
    return {
        static_cast<float>(data[row * 3 + 0]),
        static_cast<float>(data[row * 3 + 1]),
        static_cast<float>(data[row * 3 + 2])};
}

void update_bounds(
    const std::array<float, 3>& p,
    std::array<float, 3>* min_bound,
    std::array<float, 3>* max_bound)
{
    for (std::size_t i = 0; i < 3; ++i)
    {
        (*min_bound)[i] = std::min((*min_bound)[i], p[i]);
        (*max_bound)[i] = std::max((*max_bound)[i], p[i]);
    }
}

void apply_bbox_object(
    const nb::object& bbox,
    std::array<float, 3>* min_bound,
    std::array<float, 3>* max_bound)
{
    if (bbox.is_none())
        return;

    nb::sequence seq = nb::cast<nb::sequence>(bbox);

    if (nb::len(seq) == 6)
    {
        *min_bound = {
            nb::cast<float>(seq[0]),
            nb::cast<float>(seq[2]),
            nb::cast<float>(seq[4])};
        *max_bound = {
            nb::cast<float>(seq[1]),
            nb::cast<float>(seq[3]),
            nb::cast<float>(seq[5])};
        return;
    }

    if (nb::len(seq) == 2)
    {
        nb::sequence lo = nb::cast<nb::sequence>(seq[0]);
        nb::sequence hi = nb::cast<nb::sequence>(seq[1]);
        if (nb::len(lo) != 3 || nb::len(hi) != 3)
            throw std::invalid_argument("bbox must be [xmin, xmax, ymin, ymax, zmin, zmax] or [[xmin, ymin, zmin], [xmax, ymax, zmax]]");

        *min_bound = {nb::cast<float>(lo[0]), nb::cast<float>(lo[1]), nb::cast<float>(lo[2])};
        *max_bound = {nb::cast<float>(hi[0]), nb::cast<float>(hi[1]), nb::cast<float>(hi[2])};
        return;
    }

    throw std::invalid_argument("bbox must be [xmin, xmax, ymin, ymax, zmin, zmax] or [[xmin, ymin, zmin], [xmax, ymax, zmax]]");
}

void add_constructors(RansacShapeDetector* detector, const DetectOptions& options)
{
    if (options.detect_planes)
        detector->Add(std::make_unique<PlanePrimitiveShapeConstructor>());
    if (options.detect_spheres)
        detector->Add(std::make_unique<SpherePrimitiveShapeConstructor>());
    if (options.detect_cylinders)
        detector->Add(std::make_unique<CylinderPrimitiveShapeConstructor>());
    if (options.detect_cones)
        detector->Add(std::make_unique<ConePrimitiveShapeConstructor>());
    if (options.detect_tori)
        detector->Add(std::make_unique<TorusPrimitiveShapeConstructor>());
}

DetectionResult detect(const Array2D& points, const DetectOptions& options, nb::object normals, nb::object bbox)
{
    validate_xyz_array(points, "points");

    const std::size_t point_count = points.shape(0);
    if (point_count == 0)
        return DetectionResult();

    PointCloud pc;
    pc.reserve(point_count);

    std::array<float, 3> min_bound = {
        std::numeric_limits<float>::max(),
        std::numeric_limits<float>::max(),
        std::numeric_limits<float>::max()};
    std::array<float, 3> max_bound = {
        -std::numeric_limits<float>::max(),
        -std::numeric_limits<float>::max(),
        -std::numeric_limits<float>::max()};

    bool has_normals = !normals.is_none();
    Array2D normal_array;
    if (has_normals)
    {
        normal_array = nb::cast<Array2D>(normals);
        validate_xyz_array(normal_array, "normals");
        if (normal_array.shape(0) != point_count)
            throw std::invalid_argument("normals must have the same number of rows as points");
    }

    for (std::size_t i = 0; i < point_count; ++i)
    {
        const std::array<float, 3> p = read_xyz_row(points, i);
        update_bounds(p, &min_bound, &max_bound);

        Point point(vec3_from_array(p));
        point.index = i;

        if (has_normals)
            point.normal = vec3_from_array(read_xyz_row(normal_array, i));

        pc.push_back(point);
    }

    apply_bbox_object(bbox, &min_bound, &max_bound);
    pc.setBBox(vec3_from_array(min_bound), vec3_from_array(max_bound));

    if (!has_normals && options.calculate_normals && point_count > 0)
    {
        const float radius =
            options.normal_radius > 0.0f ? options.normal_radius : 0.02f * pc.getScale();
        pc.calcNormals(radius, options.normal_k, options.normal_max_tries);
    }

    RansacShapeDetector::Options ransac_options;
    ransac_options.m_epsilon =
        options.epsilon > 0.0f ? options.epsilon : 0.01f * pc.getScale();
    ransac_options.m_bitmapEpsilon =
        options.bitmap_epsilon > 0.0f ? options.bitmap_epsilon : 0.02f * pc.getScale();
    ransac_options.m_normalThresh = options.normal_thresh;
    ransac_options.m_minSupport = options.min_support;
    ransac_options.m_probability = options.probability;
    ransac_options.m_fitting = options.fitting
        ? RansacShapeDetector::Options::LS_FITTING
        : RansacShapeDetector::Options::NO_FITTING;

    RansacShapeDetector detector(ransac_options);
    add_constructors(&detector, options);

    MiscLib::Vector<std::pair<MiscLib::RefCountPtr<PrimitiveShape>, std::size_t>> shapes;
    const std::size_t remaining = detector.Detect(pc, 0, pc.size(), &shapes);

    DetectionResult result;
    result.remaining = remaining;
    result.remaining_indices.reserve(remaining);
    result.remaining_points.reserve(remaining);

    for (std::size_t i = 0; i < remaining; ++i)
    {
        result.remaining_indices.push_back(pc[i].index);
        result.remaining_points.push_back(array_from_vec3(pc[i].pos));
    }

    std::size_t begin_of_current_shape = pc.size();
    result.shapes.reserve(shapes.size());

    for (std::size_t shape_index = 0; shape_index < shapes.size(); ++shape_index)
    {
        const std::size_t support = shapes[shape_index].second;
        begin_of_current_shape -= support;

        PrimitiveShape* shape = shapes[shape_index].first.Ptr();

        DetectedShape py_shape;
        py_shape.support = support;
        py_shape.identifier = shape->Identifier();
        shape->Description(&py_shape.description);
        py_shape.kind = primitive_kind_from_description(py_shape.description);

        const std::size_t parameter_count = shape->SerializedFloatSize();
        py_shape.parameters.resize(parameter_count);
        if (parameter_count > 0)
            shape->Serialize(py_shape.parameters.data());

        py_shape.indices.reserve(support);
        py_shape.points.reserve(support);

        const std::size_t begin = begin_of_current_shape;
        const std::size_t end = begin_of_current_shape + support;
        for (std::size_t idx = begin; idx < end; ++idx)
        {
            py_shape.indices.push_back(pc[idx].index);
            py_shape.points.push_back(array_from_vec3(pc[idx].pos));
        }

        result.shapes.push_back(std::move(py_shape));
    }

    return result;
}

} // namespace

NB_MODULE(pc_ransac, m)
{
    m.doc() = "Python bindings for Efficient RANSAC point-cloud shape detection.";

    nb::class_<DetectOptions>(m, "DetectOptions")
        .def(nb::init<>())
        .def_rw("epsilon", &DetectOptions::epsilon)
        .def_rw("bitmap_epsilon", &DetectOptions::bitmap_epsilon)
        .def_rw("normal_thresh", &DetectOptions::normal_thresh)
        .def_rw("min_support", &DetectOptions::min_support)
        .def_rw("probability", &DetectOptions::probability)
        .def_rw("fitting", &DetectOptions::fitting)
        .def_rw("calculate_normals", &DetectOptions::calculate_normals)
        .def_rw("normal_radius", &DetectOptions::normal_radius)
        .def_rw("normal_k", &DetectOptions::normal_k)
        .def_rw("normal_max_tries", &DetectOptions::normal_max_tries)
        .def_rw("detect_planes", &DetectOptions::detect_planes)
        .def_rw("detect_spheres", &DetectOptions::detect_spheres)
        .def_rw("detect_cylinders", &DetectOptions::detect_cylinders)
        .def_rw("detect_cones", &DetectOptions::detect_cones)
        .def_rw("detect_tori", &DetectOptions::detect_tori);

    nb::class_<DetectedShape>(m, "DetectedShape")
        .def_ro("kind", &DetectedShape::kind)
        .def_ro("description", &DetectedShape::description)
        .def_ro("identifier", &DetectedShape::identifier)
        .def_ro("support", &DetectedShape::support)
        .def_ro("parameters", &DetectedShape::parameters)
        .def_ro("indices", &DetectedShape::indices)
        .def_ro("points", &DetectedShape::points);

    nb::class_<DetectionResult>(m, "DetectionResult")
        .def_ro("remaining", &DetectionResult::remaining)
        .def_ro("remaining_indices", &DetectionResult::remaining_indices)
        .def_ro("remaining_points", &DetectionResult::remaining_points)
        .def_ro("shapes", &DetectionResult::shapes);

    m.def(
        "detect",
        &detect,
        nb::arg("points"),
        nb::arg("options") = DetectOptions(),
        nb::arg("normals") = nb::none(),
        nb::arg("bbox") = nb::none(),
        "Detect primitive shapes in an Nx3 float32/float64 point array.");
}
