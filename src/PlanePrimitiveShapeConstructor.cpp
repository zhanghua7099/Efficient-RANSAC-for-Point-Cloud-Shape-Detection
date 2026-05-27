#include "PlanePrimitiveShapeConstructor.h"
#include "PlanePrimitiveShape.h"
#include "ScoreComputer.h"
#include <GfxTL/NullClass.h>
#include <memory>

size_t PlanePrimitiveShapeConstructor::Identifier() const
{
	return 0;
}

unsigned int PlanePrimitiveShapeConstructor::RequiredSamples() const
{
	return 3;
}

PrimitiveShape *PlanePrimitiveShapeConstructor::Construct(
	const MiscLib::Vector< Vec3f > &points, const MiscLib::Vector< Vec3f > &) const
{
	return std::make_unique< PlanePrimitiveShape >(points[0], points[1], points[2]).release();
}

PrimitiveShape *PlanePrimitiveShapeConstructor::Construct(
	const MiscLib::Vector< Vec3f > &samples) const
{
	Plane plane;
	if(!plane.Init(samples))
		return NULL;
	return std::make_unique< PlanePrimitiveShape >(plane).release();
}

PrimitiveShape *PlanePrimitiveShapeConstructor::Deserialize(std::istream *i,
	bool binary) const
{
	Plane plane;
	plane.Init(binary, i);
	return std::make_unique< PlanePrimitiveShape >(plane).release();
}

size_t PlanePrimitiveShapeConstructor::SerializedSize() const
{
	return Plane::SerializedSize();
}
