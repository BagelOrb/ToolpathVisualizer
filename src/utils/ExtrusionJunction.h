//Copyright (c) 2019 Ultimaker B.V.


#ifndef UTILS_EXTRUSION_JUNCTION_H
#define UTILS_EXTRUSION_JUNCTION_H

#include "IntPoint.h"
#include "Point3.h"

namespace visualizer
{

struct ExtrusionJunction
{
    Point p;
    coord_t w;
    coord_t perimeter_index;
    ExtrusionJunction(Point p, coord_t w, coord_t perimeter_index)
    : p(p), w(w), perimeter_index(perimeter_index) {}
    ExtrusionJunction(Point p, coord_t w)
    : p(p), w(w), perimeter_index(-1) {}
    bool operator==(const ExtrusionJunction& other) const
    {
        return p == other.p
            && w == other.w
            && perimeter_index == other.perimeter_index;
    }
    Point3 toPoint3() const
	{
		return Point3(p.X, p.Y, w);
	}
};




} // namespace visualizer
#endif // UTILS_EXTRUSION_JUNCTION_H
