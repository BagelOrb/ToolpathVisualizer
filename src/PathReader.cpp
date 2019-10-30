//Copyright (c) 2019 Ultimaker B.V.


#include "PathReader.h"

#include <cassert>

#include "utils/linearAlg2D.h"

namespace visualizer
{


int PathReader::read(std::vector<std::list<ExtrusionLine>> & result_polygons_per_index, std::vector<std::list<ExtrusionLine>> & result_polylines_per_index)
{
    std::string line;
	
	int size, inset_nr;
	double x, y, r, dx, dy;
	
	
	std::vector<std::list<std::list<ExtrusionJunction>>> lines_per_index;
	std::list<ExtrusionJunction> * last_polyline = nullptr;
	
    while (getline(file, line))
    {
		inset_nr = 0;
        if (std::sscanf(line.c_str(), "closed %d %d", &size, &inset_nr) >= 1)
        {
			if (inset_nr >= lines_per_index.size()) {
				lines_per_index.resize(inset_nr + 1);
			}
			lines_per_index[inset_nr].emplace_back();
			last_polyline = & lines_per_index[inset_nr].back();
        }
        else if (std::sscanf(line.c_str(), "%lf %lf %lf %lf %lf", &x, &y, &r, &dx, &dy) >= 2)
		{
			assert(last_polyline);
			last_polyline->emplace_back(Point(MM2INT(x), MM2INT(y)), MM2INT(2.0 * r));
		}
    }
    
    result_polygons_per_index.resize(lines_per_index.size());
    for ( size_t index = 0; index < lines_per_index.size(); index++ )
	{
		for ( std::list<ExtrusionJunction> & line : lines_per_index[index] )
		{
			simplify(line);
			
			result_polygons_per_index[index].emplace_back(-1);
			for (ExtrusionJunction j : line)
				result_polygons_per_index[index].back().junctions.emplace_back(j);
		}
	}
	
	for (auto result_polygons : result_polygons_per_index)
    {
        std::remove_if(result_polygons.begin(), result_polygons.end(), [](const ExtrusionLine& line) { return line.junctions.empty(); } );
    }
	
    return 0;
}

void PathReader::simplify(std::list<ExtrusionJunction>& line) const
{
	using it = std::list<ExtrusionJunction>::iterator;
	it prev = --(--line.end());
	it here = --line.end();
	for ( it next = line.begin(); next != line.end(); ++next)
	{
		if (shouldRemove(*prev, *here, *next))
		{
			line.erase(here);
			here = next;
		}
		else
		{
			prev = here;
			here = next;
		}
	}
	
	// TODO: how to deal with the first vertex again?
	// It seems like this algorithm already automatically takes care of it...
}

bool PathReader::shouldRemove(ExtrusionJunction & prev, ExtrusionJunction & here, ExtrusionJunction & next) const
{
	if ((next.toPoint3() - prev.toPoint3()).vSize2() > min_length * min_length) return false;
	if (LinearAlg2D::getDist2FromLineSegment(prev.p, here.p, next.p) > max_deviation * max_deviation) return false;
	Point a = prev.p;
	Point b = here.p;
	Point c = next.p;
	if (a == b || b == c) return true;
	Point ba = a - b;
	Point bc = c - b;
	double cos_angle = INT2MM2(dot(ba, bc)) / vSizeMM(ba) / vSizeMM(bc);
	assert(std::isfinite(cos_angle));
	if (cos_angle > cos(max_reduce_angle)) return false;
	return true;
}

} // namespace visualizer
