//Copyright (c) 2019 Ultimaker B.V.


#ifndef TEST_GEOMETRY_SVG_LOADER_H
#define TEST_GEOMETRY_SVG_LOADER_H

#include <fstream>
#include <regex>

#include "utils/polygon.h"
#include "utils/floatpoint.h"
#include "utils/logoutput.h"

namespace visualizer
{

class SVGloader
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
        std::regex poly_regex("<(polygon)|(path) .*/>");

        std::regex points_regex(" points=\"([^\"]*)\"");
        std::smatch points_sm;

        std::regex d_regex("d=\"([^\"]*)\"");
        std::smatch d_sm;

        std::regex point_regex("([^ ]+)");
        std::smatch point_sm;

        Point3Matrix transform;
        std::regex transform_regex(" transform=\"matrix\\(([^\"]*)\\)\"");
        std::smatch transform_sm;
        
        std::string line;
        while (getline(file, line))
        {
            if (std::regex_search(line, transform_sm, transform_regex))
            {
                std::string transform_str = transform_sm[1].str();
               
                float matrix[6];
                if (transform_sm.size() <= 0) continue;
                if (std::sscanf(transform_str.c_str(), "%f,%f,%f,%f,%f,%f", &matrix[0], &matrix[1], &matrix[2], &matrix[3], &matrix[4], &matrix[5]) == 6)
                {
                    transform.matrix[0] = MM2INT(matrix[0]);
                    transform.matrix[1] = MM2INT(matrix[1]);
                    transform.matrix[3] = MM2INT(matrix[2]);
                    transform.matrix[4] = MM2INT(matrix[3]);
                    transform.matrix[6] = MM2INT(matrix[4]);
                    transform.matrix[7] = MM2INT(matrix[5]);
                }
                else
                {
                    std::cerr << "Couldn't parse '" << transform_str << "'\n";
                }
            }
            if (std::regex_search(line, points_sm, points_regex))
            {
                if (points_sm.size() < 2)
                {
                    assert(false);
                    continue; // didn't contain points
                }
                std::string points_str = points_sm[1].str();
                        
                std::transform(points_str.begin(), points_str.end(), points_str.begin(),
                    [](unsigned char c){ return std::tolower(c); });

                PolygonRef poly = ret.newPoly();
                PolygonPointer poly_p(poly);
                bool additive = false;
                bool painting = true;
                bool is_vertical = false;
                bool is_horizontal = false;
                Point current(0,0);
                while (std::regex_search (points_str, point_sm, point_regex))
                {
                    float x, y;
                    if (point_sm.size() <= 0) continue;
                    std::string point_str = point_sm[0].str();
                    if (point_str.compare("") == 0) break;
                    if (std::sscanf(point_str.c_str(), "%f,%f", &x, &y) == 2)
                    {
                        Point here(MM2INT(x), MM2INT(y));
                        current = additive? current + here : here;
                        poly_p->emplace_back(current);
                    }
                    else
                    {
                        std::cerr << "Couldn't parse '" << point_str << "'\n";
                    }
                    points_str = point_sm.suffix().str();
                }
                poly.applyMatrix(transform);
            }
        }
        return ret;
    }
};

} // namespace visualizer
#endif // TEST_GEOMETRY_SVG_LOADER_H
