//Copyright (c) 2019 Ultimaker B.V.


#include "PathReader.h"

#include <cassert>

namespace visualizer
{


int PathReader::read(std::vector<std::list<ExtrusionLine>> & result_polygons_per_index, std::vector<std::list<ExtrusionLine>> & result_polylines_per_index)
{
    std::string line;
	
	int size, inset_nr;
	double x, y, r, dx, dy;
	
	ExtrusionLine * last_polyline = nullptr;
	
    while (getline(file, line))
    {
		inset_nr = 0;
        if (std::sscanf(line.c_str(), "closed %d %d", &size, &inset_nr) >= 1)
        {
			if (inset_nr >= result_polygons_per_index.size()) {
				result_polygons_per_index.resize(inset_nr + 1);
			}
			result_polygons_per_index[inset_nr].emplace_back(inset_nr);
			last_polyline = & result_polygons_per_index[inset_nr].back();
        }
        else if (std::sscanf(line.c_str(), "%lf %lf %lf %lf %lf", &x, &y, &r, &dx, &dy) >= 2)
		{
			assert(last_polyline);
			last_polyline->junctions.emplace_back(Point(MM2INT(x), MM2INT(y)), MM2INT(2.0 * r), last_polyline->inset_idx);
		}
    }
    return 0;
}

} // namespace visualizer
