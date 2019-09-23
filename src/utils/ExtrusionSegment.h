//Copyright (c) 2019 Ultimaker B.V.


#ifndef UTILS_EXTRUSION_SEGMENT_H
#define UTILS_EXTRUSION_SEGMENT_H

#include <utility>

#include "utils/IntPoint.h"
#include "utils/polygon.h"
#include "utils/polygonUtils.h"
#include "utils/ExtrusionJunction.h"

namespace visualizer
{

/*!
 * extrusion bead
 * suports varying width
 */
class ExtrusionSegment
{
    static constexpr float a_step = 15 / 180.0 * M_PI;
public:
    ExtrusionJunction from;
    ExtrusionJunction to;

    bool is_odd; //!< Whether this is a polyline segment rather than a polygonal segment

    ExtrusionSegment(ExtrusionJunction from, ExtrusionJunction to, bool is_odd)
    : from(from)
    , to(to)
    , is_odd(is_odd)
    {}

    /*!
     * \param reduced Whether to remove the circle from the to-location because it will be included in the next extrusion move
     */
    Polygons toPolygons(bool reduced);
};




} // namespace visualizer
#endif // UTILS_EXTRUSION_SEGMENT_H
