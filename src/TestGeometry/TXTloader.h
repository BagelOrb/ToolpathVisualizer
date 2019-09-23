//Copyright (c) 2019 Ultimaker B.V.


#ifndef TEST_GEOMETRY_TXT_LOADER_H
#define TEST_GEOMETRY_TXT_LOADER_H

#include <fstream>

#include "utils/polygon.h"
#include "utils/floatpoint.h"
#include "utils/logoutput.h"

namespace visualizer
{

class TXTloader
{
public:
    static Polygons load(std::string filename)
    {
                
        Polygons ret;
        
        std::ifstream file;
        try
        {
            file.open(filename);
        }
        catch (const std::exception& e)
        {
            logError("Couldn't open file '%s' for reading svg.\n", filename.c_str());
            std::cerr << std::flush;
            std::exit(-1);
            return ret;
        }
        if (!file.is_open())
        {
            logError("Couldn't open file '%s' for reading svg.\n", filename.c_str());
            std::exit(-1);
            return ret;
        }
        
        std::string line;

		double min_w, max_w;
		int size;
		double x, y;
		
		if (!getline(file, line)) std::cerr << "Premature EOF\n";
		if (sscanf(line.c_str(), "%lf", &min_w) < 1) std::cerr << "Something's wrong\n";
		if (!getline(file, line)) std::cerr << "Premature EOF\n";
		if (sscanf(line.c_str(), "%lf", &max_w) < 1) std::cerr << "Something's wrong\n";
				
        while (getline(file, line))
        {
            if (std::sscanf(line.c_str(), "%lf %lf", &x, &y) == 2)
            {
				ret.back().emplace_back(MM2INT(x), MM2INT(y));
            }
            else if (std::sscanf(line.c_str(), "%i", &size) == 1)
            {
				ret.emplace_back();
            }
        }
        return ret;
    }
};

} // namespace visualizer
#endif // TEST_GEOMETRY_TXT_LOADER_H
