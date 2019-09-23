//Copyright (c) 2019 Ultimaker B.V.


#ifndef UTILS_EXTRUSION_LINE_H
#define UTILS_EXTRUSION_LINE_H

#include "ExtrusionJunction.h"

namespace visualizer
{

struct ExtrusionLine
{
    coord_t inset_idx;
    std::list<ExtrusionJunction> junctions;
    ExtrusionLine(coord_t inset_idx)
    : inset_idx(inset_idx)
    {}
    coord_t computeLength()
    {
        if (junctions.size() <= 1) return 0;
        coord_t len = 0;
        ExtrusionJunction prev = junctions.front();
        for (ExtrusionJunction& next : junctions)
        {
            len += vSize(next.p - prev.p);
            prev = next;
        }
        return len;
    }
};


} // namespace visualizer
#endif // UTILS_EXTRUSION_LINE_H
