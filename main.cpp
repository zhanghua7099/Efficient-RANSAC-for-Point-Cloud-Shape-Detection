#include <PointCloud.h>
#include <RansacShapeDetector.h>
#include <PlanePrimitiveShapeConstructor.h>
#include <CylinderPrimitiveShapeConstructor.h>
#include <SpherePrimitiveShapeConstructor.h>
#include <ConePrimitiveShapeConstructor.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <random>
#include <sstream>
#include <string>
#include <vector>

namespace
{
constexpr float kPi = 3.14159265358979323846f;

constexpr float kNoiseSigma = 0.01f;
constexpr int kOutlierCount = 800;

struct LabeledPoint
{
    float x;
    float y;
    float z;
    std::string label;
};

float gaussian(std::mt19937& rng, float sigma)
{
    if (sigma <= 0.0f)
        return 0.0f;

    std::normal_distribution<float> dist(0.0f, sigma);
    return dist(rng);
}

float uniform(std::mt19937& rng, float lo, float hi)
{
    std::uniform_real_distribution<float> dist(lo, hi);
    return dist(rng);
}

void addPoint(
    PointCloud& pc,
    std::vector<LabeledPoint>& exportedPoints,
    std::mt19937& rng,
    float x,
    float y,
    float z,
    const std::string& label)
{
    const float nx = x + gaussian(rng, kNoiseSigma);
    const float ny = y + gaussian(rng, kNoiseSigma);
    const float nz = z + gaussian(rng, kNoiseSigma);

    pc.push_back(Point(Vec3f(nx, ny, nz)));
    exportedPoints.push_back({nx, ny, nz, label});
}

void addPlane(PointCloud& pc, std::vector<LabeledPoint>& exportedPoints, std::mt19937& rng)
{
    const int n = 40;
    const float minCoord = -5.0f;
    const float maxCoord = 5.0f;

    for (int ix = 0; ix <= n; ++ix)
    {
        const float x = minCoord + (maxCoord - minCoord) * float(ix) / float(n);

        for (int iy = 0; iy <= n; ++iy)
        {
            const float y = minCoord + (maxCoord - minCoord) * float(iy) / float(n);
            addPoint(pc, exportedPoints, rng, x, y, -2.0f, "plane");
        }
    }
}

void addSphere(PointCloud& pc, std::vector<LabeledPoint>& exportedPoints, std::mt19937& rng)
{
    const float cx = -7.0f;
    const float cy = 3.0f;
    const float cz = 2.5f;
    const float r = 1.5f;

    const int nTheta = 80;
    const int nPhi = 40;

    for (int ip = 1; ip < nPhi; ++ip)
    {
        const float phi = kPi * float(ip) / float(nPhi);
        const float sinPhi = std::sin(phi);
        const float cosPhi = std::cos(phi);

        for (int it = 0; it < nTheta; ++it)
        {
            const float theta = 2.0f * kPi * float(it) / float(nTheta);

            const float x = cx + r * sinPhi * std::cos(theta);
            const float y = cy + r * sinPhi * std::sin(theta);
            const float z = cz + r * cosPhi;

            addPoint(pc, exportedPoints, rng, x, y, z, "sphere");
        }
    }
}

void addCylinder(PointCloud& pc, std::vector<LabeledPoint>& exportedPoints, std::mt19937& rng)
{
    const float cx = 7.0f;
    const float cy = 3.0f;
    const float r = 1.2f;
    const float z0 = 0.3f;
    const float z1 = 4.3f;

    const int nTheta = 80;
    const int nZ = 60;

    for (int iz = 0; iz <= nZ; ++iz)
    {
        const float z = z0 + (z1 - z0) * float(iz) / float(nZ);

        for (int it = 0; it < nTheta; ++it)
        {
            const float theta = 2.0f * kPi * float(it) / float(nTheta);

            const float x = cx + r * std::cos(theta);
            const float y = cy + r * std::sin(theta);

            addPoint(pc, exportedPoints, rng, x, y, z, "cylinder");
        }
    }
}

void addCone(PointCloud& pc, std::vector<LabeledPoint>& exportedPoints, std::mt19937& rng)
{
    const float cx = -7.0f;
    const float cy = -4.0f;
    const float baseZ = 0.5f;
    const float apexZ = 4.5f;
    const float baseRadius = 1.5f;
    const float height = apexZ - baseZ;

    const int nTheta = 80;
    const int nH = 60;

    for (int ih = 0; ih <= nH; ++ih)
    {
        const float t = 0.92f * float(ih) / float(nH);
        const float z = baseZ + height * t;
        const float r = baseRadius * (1.0f - t);

        for (int it = 0; it < nTheta; ++it)
        {
            const float theta = 2.0f * kPi * float(it) / float(nTheta);

            const float x = cx + r * std::cos(theta);
            const float y = cy + r * std::sin(theta);

            addPoint(pc, exportedPoints, rng, x, y, z, "cone");
        }
    }
}

void addOutliers(PointCloud& pc, std::vector<LabeledPoint>& exportedPoints, std::mt19937& rng)
{
    for (int i = 0; i < kOutlierCount; ++i)
    {
        addPoint(
            pc,
            exportedPoints,
            rng,
            uniform(rng, -10.0f, 10.0f),
            uniform(rng, -7.0f, 7.0f),
            uniform(rng, -2.5f, 5.5f),
            "outlier");
    }
}

void writeOriginalPointCloudTxt(
    const std::string& filename,
    const std::vector<LabeledPoint>& points)
{
    std::ofstream out(filename.c_str());

    if (!out)
    {
        std::cerr << "Failed to open output file: " << filename << std::endl;
        return;
    }

    out << "# x y z label\n";

    for (const auto& p : points)
    {
        out << p.x << " " << p.y << " " << p.z << " " << p.label << "\n";
    }

    std::cout << "Exported original point cloud: " << filename << std::endl;
}

std::string toLower(std::string s)
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

std::string primitiveKindFromDescription(const std::string& desc)
{
    const std::string d = toLower(desc);

    if (d.find("plane") != std::string::npos)
        return "plane";

    if (d.find("sphere") != std::string::npos)
        return "sphere";

    if (d.find("cylinder") != std::string::npos)
        return "cylinder";

    if (d.find("cone") != std::string::npos)
        return "cone";

    return "unknown";
}

std::string makeShapeInlierFilename(size_t shapeIndex, const std::string& kind)
{
    std::ostringstream oss;

    oss << "detected_shape_"
        << std::setw(2)
        << std::setfill('0')
        << shapeIndex
        << "_"
        << kind
        << "_inliers.txt";

    return oss.str();
}

void writeOnePoint(std::ofstream& out, const Point& p)
{
    out << p.pos[0] << " " << p.pos[1] << " " << p.pos[2];
}

void exportDetectedInliers(
    const PointCloud& pc,
    const MiscLib::Vector<std::pair<MiscLib::RefCountPtr<PrimitiveShape>, size_t>>& shapes)
{
    std::ofstream allOut("detected_inliers_all.txt");

    if (!allOut)
    {
        std::cerr << "Failed to open detected_inliers_all.txt" << std::endl;
        return;
    }

    allOut << "# x y z shape_index shape_kind shape_support\n";

    size_t beginOfCurrentShape = pc.size();

    for (size_t shapeIndex = 0; shapeIndex < shapes.size(); ++shapeIndex)
    {
        const size_t support = shapes[shapeIndex].second;
        beginOfCurrentShape -= support;

        std::string desc;
        shapes[shapeIndex].first->Description(&desc);

        const std::string kind = primitiveKindFromDescription(desc);
        const std::string filename = makeShapeInlierFilename(shapeIndex, kind);

        std::ofstream shapeOut(filename.c_str());

        if (!shapeOut)
        {
            std::cerr << "Failed to open " << filename << std::endl;
            continue;
        }

        shapeOut << "# x y z shape_index shape_kind shape_support\n";

        const size_t begin = beginOfCurrentShape;
        const size_t end = beginOfCurrentShape + support;

        for (size_t idx = begin; idx < end; ++idx)
        {
            const Point& p = pc[idx];

            writeOnePoint(shapeOut, p);
            shapeOut << " " << shapeIndex << " " << kind << " " << support << "\n";

            writeOnePoint(allOut, p);
            allOut << " " << shapeIndex << " " << kind << " " << support << "\n";
        }

        std::cout
            << "Exported inliers of shape "
            << shapeIndex
            << " to "
            << filename
            << " | support = "
            << support
            << " | kind = "
            << kind
            << std::endl;
    }

    std::cout << "Exported all detected inliers to detected_inliers_all.txt" << std::endl;
}

void exportRemainingUnassignedPoints(const PointCloud& pc, size_t remaining)
{
    std::ofstream out("remaining_unassigned_points.txt");

    if (!out)
    {
        std::cerr << "Failed to open remaining_unassigned_points.txt" << std::endl;
        return;
    }

    out << "# x y z\n";

    for (size_t idx = 0; idx < remaining; ++idx)
    {
        const Point& p = pc[idx];
        writeOnePoint(out, p);
        out << "\n";
    }

    std::cout
        << "Exported remaining unassigned points to remaining_unassigned_points.txt"
        << " | count = "
        << remaining
        << std::endl;
}

} // namespace

int main(int argc, char** argv)
{
    const std::string originalPointCloudTxt =
        argc > 1 ? argv[1] : "synthetic_primitives_pointcloud.txt";
    (void)originalPointCloudTxt;

    PointCloud pc;
    std::vector<LabeledPoint> exportedPoints;

    std::mt19937 rng(42);

    addPlane(pc, exportedPoints, rng);
    addSphere(pc, exportedPoints, rng);
    addCylinder(pc, exportedPoints, rng);
    addCone(pc, exportedPoints, rng);
    addOutliers(pc, exportedPoints, rng);

    // writeOriginalPointCloudTxt(originalPointCloudTxt, exportedPoints);

    pc.setBBox(Vec3f(-11.0f, -8.0f, -3.0f), Vec3f(11.0f, 8.0f, 6.0f));

    pc.calcNormals(0.55f, 20, 100);

    std::cout << "\nTotal points: " << pc.size() << std::endl;
    std::cout << "Point cloud scale: " << pc.getScale() << std::endl;

    RansacShapeDetector::Options ransacOptions;

    ransacOptions.m_epsilon = 0.0025f * pc.getScale();
    ransacOptions.m_bitmapEpsilon = 0.01f * pc.getScale();
    ransacOptions.m_normalThresh = 0.85f;
    ransacOptions.m_minSupport = 200;
    ransacOptions.m_probability = 0.001f;

    RansacShapeDetector detector(ransacOptions);

    detector.Add(std::make_unique<PlanePrimitiveShapeConstructor>());
    detector.Add(std::make_unique<SpherePrimitiveShapeConstructor>());
    detector.Add(std::make_unique<CylinderPrimitiveShapeConstructor>());
    detector.Add(std::make_unique<ConePrimitiveShapeConstructor>());

    MiscLib::Vector<std::pair<MiscLib::RefCountPtr<PrimitiveShape>, size_t>> shapes;

    const size_t remaining = detector.Detect(pc, 0, pc.size(), &shapes);

    std::cout << "\nDetection result" << std::endl;
    std::cout << "Remaining unassigned points: " << remaining << std::endl;
    std::cout << "Detected shape count: " << shapes.size() << std::endl;

    // std::map<std::string, int> foundCount;

    // size_t beginOfCurrentShape = pc.size();

    // for (size_t i = 0; i < shapes.size(); ++i)
    // {
    //     const size_t support = shapes[i].second;
    //     beginOfCurrentShape -= support;

    //     std::string desc;
    //     shapes[i].first->Description(&desc);

    //     const std::string kind = primitiveKindFromDescription(desc);
    //     ++foundCount[kind];

    //     std::cout
    //         << "shape " << i
    //         << " | support = " << support
    //         << " | point range after Detect = ["
    //         << beginOfCurrentShape
    //         << ", "
    //         << beginOfCurrentShape + support
    //         << ")"
    //         << " | kind = " << kind
    //         << " | desc = " << desc
    //         << std::endl;
    // }

    // exportDetectedInliers(pc, shapes);
    // exportRemainingUnassignedPoints(pc, remaining);

    // std::cout << "\nSummary" << std::endl;

    // const char* expected[] = {"plane", "sphere", "cylinder", "cone"};
    // bool ok = true;

    // for (const char* name : expected)
    // {
    //     const int n = foundCount[name];
    //     std::cout << "  " << name << ": " << n << std::endl;

    //     if (n < 1)
    //         ok = false;
    // }

    // if (ok)
    // {
    //     std::cout << "\nPASS: all expected primitive types were detected at least once." << std::endl;
    //     return 0;
    // }

    // std::cout << "\nWARN: at least one expected primitive type was not detected." << std::endl;
    // std::cout << "Try tuning: m_epsilon, m_bitmapEpsilon, m_normalThresh, m_minSupport, or calcNormals radius." << std::endl;

    return 0;
}
