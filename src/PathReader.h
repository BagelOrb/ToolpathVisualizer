//Copyright (c) 2019 Ultimaker B.V.


#ifndef PATH_READER_H
#define PATH_READER_H


#include <fstream>
#include <regex>

#include "utils/ExtrusionLine.h"

namespace visualizer
{

/*!
 * Read variable width paths
 */
class PathReader
{
public:
	PathReader()
	{}
	
	int open(std::string filename) {
        try
        {
            file.open(filename);
        }
        catch (const std::exception& e)
        {
            std::printf("Couldn't open file '%s' for reading svg.\n", filename.c_str());
            std::exit(-1);
            return -1;
        }
        return 0;
	}
	/*!
	 * \return error code
	 */
	int read(std::vector<std::list<ExtrusionLine>> & result_polygons_per_index, std::vector<std::list<ExtrusionLine>> & result_polylines_per_index);
protected:
	std::ifstream file;
	coord_t simplify_min_length_3D = MM2INT(0.3); // 3D length over x,y,R
	static constexpr coord_t max_deviation = MM2INT(0.025);
	static constexpr float max_reduce_angle = 130.0/180.0 * M_PI;

	void simplify(std::list<ExtrusionJunction>& line, bool closed) const;
	bool shouldRemove(ExtrusionJunction & before, ExtrusionJunction & here, ExtrusionJunction & next) const;
};




} // namespace visualizer
#endif // PATH_READER_H
