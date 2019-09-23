//Copyright (c) 2019 Ultimaker B.V.


#ifndef TEST_GEOMETRY_SPIKY_H
#define TEST_GEOMETRY_SPIKY_H

#include <utility>

#include "../utils/IntPoint.h"
#include "../utils/logoutput.h"
#include "../utils/polygonUtils.h"
#include "../utils/ExtrusionSegment.h"
#include "../utils/polygon.h"

namespace visualizer
{

class Spiky
{
public:
    static Polygons oneSpike(coord_t gap = 300, coord_t size = 3000)
    {
        coord_t hole = 10;
        Polygons ret;
        PolygonRef poly = ret.newPoly();
        poly.emplace_back(0,0);
        poly.emplace_back(size,0);
        poly.emplace_back(size,size);
        poly.emplace_back(size / 2 + hole,size);
        poly.emplace_back(size / 2, gap);
        poly.emplace_back(size / 2 - hole,size);
        poly.emplace_back(0,size);
        return ret;
    }
    static Polygons oneSpikeOneCorner(coord_t gap = 300, coord_t size = 3000, coord_t hole = 100)
    {
        Polygons ret;
        PolygonRef poly = ret.newPoly();
        poly.emplace_back(size / 2 - gap,size / 4);
        poly.emplace_back(size,size / 4);
        poly.emplace_back(size,size - hole);
        poly.emplace_back(size / 2 + gap * .5 * sqrt(2.0), size / 2 + gap * .5 * sqrt(2.0));
        poly.emplace_back(size,size);
        poly.emplace_back(0,size);
        poly.emplace_back(size / 2,size / 2);
        return ret;
    }
    static Polygons twoSpikes(coord_t gap = 300, coord_t size = 3000, coord_t hole = 100)
    {
        Polygons ret;
        PolygonRef poly = ret.newPoly();
        poly.emplace_back(0,0);
        poly.emplace_back(size / 2 - hole,0);
        poly.emplace_back(size / 2,size/2 - gap / 2);
        poly.emplace_back(size / 2 + hole,0);
        poly.emplace_back(size,0);
        poly.emplace_back(size,size);
        poly.emplace_back(size / 2 + hole,size);
        poly.emplace_back(size / 2,size/2 + gap / 2);
        poly.emplace_back(size / 2 - hole,size);
        poly.emplace_back(0,size);
        return ret;
    }
    static Polygons twoSpikesDiamond(coord_t gap = 300, coord_t size = 3000, coord_t hole = 100)
    {
        Polygons ret;
        PolygonRef poly = ret.newPoly();
        poly.emplace_back(size / 2 - hole,0);
        poly.emplace_back(size / 2,size/2 - gap / 2);
        poly.emplace_back(size / 2 + hole,0);
        poly.emplace_back(size,size / 2);
        poly.emplace_back(size / 2 + hole,size);
        poly.emplace_back(size / 2,size/2 + gap / 2);
        poly.emplace_back(size / 2 - hole,size);
        poly.emplace_back(0,size / 2);
        return ret;
    }
    static Polygons fourSpikes(coord_t gap1 = 500, coord_t gap2 = 1200, coord_t size = 3000)
    {
        coord_t hole = 10;
        Polygons ret;
        PolygonRef poly = ret.newPoly();
        poly.emplace_back(0,0);
        poly.emplace_back(size / 2 - hole,0);
        poly.emplace_back(size / 2,size/2 - gap1 / 2);
        poly.emplace_back(size / 2 + hole,0);
        poly.emplace_back(size,0);
        poly.emplace_back(size,size / 2 - hole);
        poly.emplace_back(size/2 + gap2 / 2, size / 2);
        poly.emplace_back(size,size / 2 + hole);
        poly.emplace_back(size,size);
        poly.emplace_back(size / 2 + hole,size);
        poly.emplace_back(size / 2,size/2 + gap1 / 2);
        poly.emplace_back(size / 2 - hole,size);
        poly.emplace_back(0,size);
        poly.emplace_back(0,size / 2 + hole);
        poly.emplace_back(size/2 - gap2 / 2, size / 2);
        poly.emplace_back(0,size / 2 - hole);
        return ret;
    }
    
    static Polygons doubleOutSpike(coord_t hole = 600, coord_t recede_dist = 50, coord_t size = 3000)
    {
        Polygons ret;
        PolygonRef poly = ret.newPoly();
        poly.emplace_back(0,0);
        poly.emplace_back(size,0);
        poly.emplace_back(size,size);
        poly.emplace_back(size / 2 + hole / 2,size);
        poly.emplace_back(size / 2 + hole / 4, size * 2);
        poly.emplace_back(size / 2, size + recede_dist);
        poly.emplace_back(size / 2 - hole / 4, size * 2);
        poly.emplace_back(size / 2 - hole / 2,size);
        poly.emplace_back(0,size);
        return ret;
    }
};

} // namespace visualizer
#endif // TEST_GEOMETRY_SPIKY_H
