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
};




} // namespace visualizer
#endif // PATH_READER_H
