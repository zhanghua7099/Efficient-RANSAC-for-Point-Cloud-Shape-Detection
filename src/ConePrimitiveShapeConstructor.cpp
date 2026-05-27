#include "ConePrimitiveShapeConstructor.h"
#include "ConePrimitiveShape.h"
#include "Cone.h"
#include "ScoreComputer.h"
#include <GfxTL/NullClass.h>
#include <memory>

size_t ConePrimitiveShapeConstructor::Identifier() const
{
	return 3;
}

unsigned int ConePrimitiveShapeConstructor::RequiredSamples() const
{
	return 3;
}

PrimitiveShape *ConePrimitiveShapeConstructor::Construct(
	const MiscLib::Vector< Vec3f > &points,
	const MiscLib::Vector< Vec3f > &normals) const
{
	Cone cone;
	if(!cone.Init(points[0], points[1], points[2], normals[0], normals[1],
		normals[2]))
		return NULL;
	if(cone.Angle() > 1.4835298641951801403851371532153)
		// do not allow cones with an opening angle of more than 85 degrees
		return NULL;
	return std::make_unique< ConePrimitiveShape >(cone).release();
}

PrimitiveShape *ConePrimitiveShapeConstructor::Construct(
	const MiscLib::Vector< Vec3f > &samples) const
{
	Cone cone;
	if(!cone.Init(samples))
		return NULL;
	return std::make_unique< ConePrimitiveShape >(cone).release();
}

PrimitiveShape *ConePrimitiveShapeConstructor::Deserialize(std::istream *i,
	bool binary) const
{
	Cone cone;
	cone.Init(binary, i);
	return std::make_unique< ConePrimitiveShape >(cone).release();
}

size_t ConePrimitiveShapeConstructor::SerializedSize() const
{
	return Cone::SerializedSize();
}
