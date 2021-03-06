//Copyright (c) 2019 Ultimaker B.V.


#ifndef GCODER_H
#define GCODER_H

#include <fstream>

#include "utils/polygon.h"
#include "utils/AABB.h"
#include "utils/ExtrusionJunction.h"
#include "utils/ExtrusionLine.h"
#include "utils/types/Duration.h"

#include "timeEstimate.h"

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
    GcodeWriter(std::string filename, int type, bool dual_extrusion, coord_t layer_thickness, float print_speed, float travel_speed, float extrusion_multiplier, bool equalize_flow);
    ~GcodeWriter();
    static constexpr int type_UM3 = 1;
    static constexpr int type_UMS5 = 2;
    void printBrim(const Polygons& outline, coord_t count, coord_t w = 400, coord_t dist = 600);
	void printRaft(const Polygons& outline);
	
	std::vector<ExtrusionLine> generateLines(const Polygons& outline, coord_t line_width, coord_t spacing, float angle);
	std::vector<ExtrusionLine> generateLines(const Polygons& outline, coord_t line_width, coord_t spacing);
    /*!
     * 
     * \param ordered Whether to print all index 0 polylines before any index 1 polylines etc.
     */
    void print(const std::vector<std::list<ExtrusionLine>> polygons_per_index, const std::vector<std::list<ExtrusionLine>> polylines_per_index, bool ordered = false, bool startup_and_reduction = true);
    void printOrdered(const std::vector<std::list<ExtrusionLine>>& polygons_per_index, const std::vector<std::list<ExtrusionLine>>& polylines_per_index, bool startup_and_reduction);
    void printUnordered(const std::vector<std::list<ExtrusionLine>>& polygons_per_index, const std::vector<std::list<ExtrusionLine>>& polylines_per_index, bool startup_and_reduction);
	
	void printLinesByOptimizer(const std::vector<ExtrusionLine>& lines);
    void printPolygon(Path& polygon, int start_idx);
    void printPolyline(Path& poly, int start_idx);
    void reduce(Path& polyline, size_t start_point_idx, coord_t initial_width, coord_t traveled_dist);
	void setTranslation(Point p);
	void setNominalSpeed(float print_speed);
	void setFlowModifier(float flow_ratio) { extrusion_multiplier = flow_ratio; }
	void retract();
    void move(Point p);
    void print(ExtrusionJunction from, ExtrusionJunction to);
    void extrude(float amount);
	
	void switchExtruder(int extruder_nr);
	
	void setBackPressureCompensation(float bpc);
	void setTemp(int temp);
	template<class... Args>
	void comment(std::string format, Args... args)
	{
		char buffer[80]; // max line limit in firmware
	    sprintf(buffer, format.c_str(), args...);
		file << ";" << buffer << "\n";
	}
    TimeEstimateCalculator marlin_estimates;
	Duration total_naive_print_time = 0.0;
private:
    void printSingleExtrusionMove(ExtrusionJunction& from, ExtrusionJunction& to);
    std::ofstream file;
    int type;
    Point build_plate_middle;
    float filament_diameter = 2.85;
    coord_t discretization_size = MM2INT(0.2);
    coord_t nozzle_size = MM2INT(0.4);
	float retraction_distance = 6.5;
	Point extruder_offset[2] = {Point(0,0), Point(0,0)}; // set in constructor
	float temp = 0; // set in constructor
    
    
    coord_t layer_thickness;
    float print_speed;
    float travel_speed;
    float extrusion_multiplier;
	bool equalize_flow;
	float flow;

	float back_pressure_compensation;
	
    Point translation;

	int current_extruder;
    Point cur_pos;
	coord_t cur_z;
    bool is_retracted;
    float last_E = 0;

    float getExtrusionFilamentMmPerMmMove(coord_t width) const;
	float getExtrusionFilamentMmPerCubicMm() const;
};




} // namespace visualizer
#endif // GCODER_H
