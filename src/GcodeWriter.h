//Copyright (c) 2019 Ultimaker B.V.


#ifndef GCODER_H
#define GCODER_H

#include <fstream>

#include "utils/polygon.h"
#include "utils/AABB.h"
#include "utils/ExtrusionJunction.h"
#include "utils/ExtrusionLine.h"

namespace visualizer
{

/*!
 * Write gcode for a single layer variable line width print
 */
class GcodeWriter
{
    struct Path
    {
        std::vector<ExtrusionJunction> junctions;
        bool is_closed;
        Path(bool is_closed)
        : is_closed(is_closed)
        {}
    };
public:
    GcodeWriter(std::string filename, int type, coord_t layer_thickness = MM2INT(0.2), float print_speed = 600, float travel_speed = 3500, float extrusion_multiplier = 1.0, bool equalize_flow = true);
    ~GcodeWriter();
    static constexpr int type_UM3 = 1;
    void printBrim(AABB aabb, coord_t count, coord_t w = 400, coord_t dist = 600);
    /*!
     * 
     * \param ordered Whether to print all index 0 polylines before any index 1 polylines etc.
     */
    void print(std::vector<std::list<ExtrusionLine>> polygons_per_index, std::vector<std::list<ExtrusionLine>> polylines_per_index, AABB aabb, bool ordered = false, bool startup_and_reduction = true);
    void printOrdered(std::vector<std::list<ExtrusionLine>>& polygons_per_index, std::vector<std::list<ExtrusionLine>>& polylines_per_index, AABB aabb, bool startup_and_reduction);
    void printUnordered(std::vector<std::list<ExtrusionLine>>& polygons_per_index, std::vector<std::list<ExtrusionLine>>& polylines_per_index, AABB aabb, bool startup_and_reduction);
    void printPolygon(Path& polygon, int start_idx);
    void printPolyline(Path& poly, int start_idx);
    void reduce(Path& polyline, size_t start_point_idx, coord_t initial_width, coord_t traveled_dist);
    void move(Point p);
    void print(ExtrusionJunction from, ExtrusionJunction to);
    void extrude(float amount);
private:
    void printSingleExtrusionMove(ExtrusionJunction& from, ExtrusionJunction& to);
    std::ofstream file;
    int type;
    Point build_plate_middle = Point(MM2INT(100), MM2INT(100));
    float filament_diameter = 2.85;
    coord_t discretization_size = MM2INT(0.1);
    coord_t nozzle_size = MM2INT(0.4);

    coord_t layer_thickness;
    float print_speed;
    float travel_speed;
    float extrusion_multiplier;
	bool equalize_flow;
	float flow;

    Point reduction;

    Point cur_pos;
    bool is_unretracted;
    float last_E = 0;

    float getExtrusionFilamentMmPerMmMove(coord_t width) const;
	float getExtrusionFilamentMmPerCubicMm() const;
};




} // namespace visualizer
#endif // GCODER_H
