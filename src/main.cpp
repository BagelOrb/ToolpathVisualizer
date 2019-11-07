
#include <cstdio>
#include <string.h>
#include <strings.h>
#include <sstream>
#include <stdio.h> // for file output
#include <fstream>
#include <iostream>
#include <algorithm> // random_shuffle

#include <boost/version.hpp>

#include <unordered_set>
#include <unordered_map>

#include "utils/logoutput.h"
#include "utils/polygon.h"
#include "utils/gettime.h"
#include "utils/SVG.h"

#include "utils/VoronoiUtils.h"
#include "GcodeWriter.h"
#include "PathReader.h"
#include "Statistics.h"

#include "TestGeometry/Pika.h"
#include "TestGeometry/Jin.h"
#include "TestGeometry/Moessen.h"
#include "TestGeometry/Prescribed.h"
#include "TestGeometry/Spiky.h"
#include "TestGeometry/SVGloader.h"
#include "TestGeometry/TXTloader.h"
#include "TestGeometry/Microstructure.h"

#include "TestGeometry/VariableWidthGcodeTester.h"

using visualizer::Point;

namespace visualizer
{

void convertSvg2SmoothPathPlanningFormat(const Polygons polys)
{
	std::cerr << "0.3\n";
	std::cerr << "1.0\n";
	for (ConstPolygonRef poly : polys)
	{
		std::cerr << poly.size() << '\n';
		for (Point p : poly)
			std::cerr << INT2MM(p.X) << " " << INT2MM(p.Y) << '\n';
	}
}

float nominal_print_speed = 30.0;
float travel_speed = 60.0;
float flow_modifier = 1.05;
coord_t layer_thickness = MM2INT(0.05);
float kappa = 0.25;//0.45;

// raft settings
float nominal_raft_speed = 50.0;

void squareGridTest(const std::vector<std::list<ExtrusionLine>> & result_polylines_per_index, const std::vector<std::list<ExtrusionLine>> & result_polygons_per_index, const Polygons & polys, const std::string output_prefix)
{
	AABB aabb(polys);
	Point aabb_size = aabb.max - aabb.min;

    std::ostringstream ss;
    ss << "visualization/" << output_prefix << ".gcode";
	GcodeWriter gcode(ss.str(), GcodeWriter::type_UM3, true, layer_thickness, nominal_raft_speed, travel_speed, flow_modifier, true);
	
	
	Point grid_shape(4,6);
	coord_t gap_dist = MM2INT(1);
	
	Polygons raft_outline; // = polys.offset(MM2INT(5.0), ClipperLib::jtRound);
	AABB raft_aabb = aabb;
	for ( int x = 0; x < grid_shape.X; x++ )
	for ( int y = 0; y < grid_shape.Y; y++ )
	{
		Point translation = Point(.5 * (2 * x - grid_shape.X) * (aabb_size.X + gap_dist), .5 * (2 * y - grid_shape.Y) * (aabb_size.Y + gap_dist));
		raft_aabb.include(aabb.min + translation);
		raft_aabb.include(aabb.max + translation);
	}
	raft_outline = raft_aabb.toPolygons().offset(MM2INT(2.0), ClipperLib::jtRound);
	gcode.printRaft(raft_outline);

	gcode.switchExtruder(0);
//     gcode.marlin_estimates.reset(); gcode.total_naive_print_time = 0;
	gcode.setNominalSpeed(nominal_print_speed);
	
	gcode.printBrim(raft_aabb.toPolygons(), 1);
	
	gcode.comment("TYPE:WALL-OUTER");
	float back_pressure_compensation = 0.0;
	for ( int x = 0; x < grid_shape.X; x++ )
	for ( int y = 0; y < grid_shape.Y; y++ )
	{
		Point translation = Point(.5 * (2 * x - grid_shape.X) * (aabb_size.X + gap_dist), .5 * (2 * y - grid_shape.Y) * (aabb_size.Y + gap_dist));
		gcode.setTranslation(translation);
		
		gcode.setBackPressureCompensation(back_pressure_compensation);
		gcode.comment("Back-pressure compensation: %f", back_pressure_compensation);
		
		
		gcode.comment("Pos:%i,%i", x, y);
		
// 		gcode.printBrim(aabb.toPolygons(), 1, MM2INT(0.4), MM2INT(0.6));
// 		gcode.retract();
		
		gcode.print(result_polygons_per_index, result_polylines_per_index, false);
		
		gcode.retract();
		back_pressure_compensation += 0.05;
	}
    
    std::cerr << "Print time: " << gcode.marlin_estimates.calculate() << "\n";
}

void raftedPrint(const std::vector<std::list<ExtrusionLine>> & result_polylines_per_index, const std::vector<std::list<ExtrusionLine>> & result_polygons_per_index, const Polygons & polys, const std::string output_prefix, bool brim = true)
{
	AABB aabb(polys);

    std::ostringstream ss;
    ss << "visualization/" << output_prefix << ".gcode";
	GcodeWriter gcode(ss.str(), GcodeWriter::type_UM3, true, layer_thickness, nominal_raft_speed, travel_speed, flow_modifier, true);
	
	Polygons raft_outline = polys.offset(MM2INT(6.0), ClipperLib::jtRound).offset(MM2INT(-3.0), ClipperLib::jtRound);
	gcode.printRaft(raft_outline);
	gcode.retract();

	gcode.switchExtruder(0);
//     gcode.marlin_estimates.reset(); gcode.total_naive_print_time = 0;
	gcode.setNominalSpeed(nominal_print_speed);
	gcode.setBackPressureCompensation(kappa);
	gcode.comment("kappa: %f", kappa);
	gcode.retract();
	
// 	gcode.move(aabb.min);
    if (brim)
    {
        gcode.printBrim(polys, 1, MM2INT(0.4), MM2INT(0.6));
        gcode.retract();
    }
	
	gcode.comment("TYPE:WALL-OUTER");
	gcode.print(result_polygons_per_index, result_polylines_per_index, false, false);
    
    std::cerr << "Print time: " << gcode.marlin_estimates.calculate() << "\n";
}

void print(const std::vector<std::list<ExtrusionLine>> & result_polylines_per_index, const std::vector<std::list<ExtrusionLine>> & result_polygons_per_index, const Polygons & polys, const std::string output_prefix)
{
	AABB aabb(polys);

    std::ostringstream ss;
    ss << "visualization/" << output_prefix << ".gcode";
	GcodeWriter gcode(ss.str(), GcodeWriter::type_UM3, true, layer_thickness, nominal_raft_speed, travel_speed, flow_modifier, true);
	
	gcode.switchExtruder(0);
	gcode.setNominalSpeed(nominal_print_speed);
	gcode.setBackPressureCompensation(kappa);
	gcode.comment("kappa: %f", kappa);
	gcode.retract();
	
// 	gcode.move(aabb.min);
// 	gcode.printBrim(polys, 1, MM2INT(0.4), MM2INT(0.6));
// 	gcode.retract();
	
	gcode.comment("TYPE:WALL-OUTER");
	gcode.print(result_polygons_per_index, result_polylines_per_index, false, false);
    
    std::cerr << "Print time: " << gcode.marlin_estimates.calculate() << "\n";
}

void varWidthTest(std::vector<std::list<ExtrusionLine>> & result_polylines_per_index, std::vector<std::list<ExtrusionLine>> & result_polygons_per_index, Polygons & polys)
{
	result_polygons_per_index.clear();
	result_polylines_per_index.clear();
	
	result_polylines_per_index.emplace_back();
	result_polylines_per_index.back().emplace_back();
	ExtrusionLine & line = result_polylines_per_index.back().back();
	
	coord_t minW = MM2INT(0.3);
	coord_t maxW = MM2INT(1.0);
	coord_t midW = (minW + maxW) / 2;
	coord_t nrml = MM2INT(0.4);
	
	coord_t gap = MM2INT(0.1);
	
	coord_t endL = MM2INT(5);
	
	std::list<std::vector<Point>> dist_and_widths_list;
	dist_and_widths_list.emplace_back(std::initializer_list<Point>({Point(0,nrml), Point(endL * 2 + MM2INT(30),nrml)}));
	dist_and_widths_list.emplace_back(std::initializer_list<Point>({Point(0,nrml), Point(endL,minW), Point(MM2INT(30),maxW), Point(endL,nrml)}));
	dist_and_widths_list.emplace_back(std::initializer_list<Point>({Point(0,nrml), Point(endL,minW), Point(MM2INT(10),maxW), Point(MM2INT(10),minW), Point(MM2INT(10),maxW), Point(endL,nrml)}));
	dist_and_widths_list.emplace_back(std::initializer_list<Point>({Point(0,nrml), Point(endL,minW), Point(MM2INT(5),maxW), Point(MM2INT(5),minW), Point(MM2INT(5),maxW), Point(MM2INT(5),minW), Point(MM2INT(5),maxW), Point(MM2INT(5),minW), Point(endL,nrml)}));
	dist_and_widths_list.emplace_back(std::initializer_list<Point>({Point(0,nrml), Point(endL,maxW), Point(MM2INT(5),minW), Point(MM2INT(5),maxW), Point(MM2INT(5),minW), Point(MM2INT(5),maxW), Point(MM2INT(5),minW), Point(MM2INT(5),maxW), Point(endL,nrml)}));
	dist_and_widths_list.emplace_back(std::initializer_list<Point>({Point(0,nrml), Point(endL,maxW), Point(MM2INT(10),minW), Point(MM2INT(10),maxW), Point(MM2INT(10),minW), Point(endL,nrml)}));
	dist_and_widths_list.emplace_back(std::initializer_list<Point>({Point(0,nrml), Point(endL,maxW), Point(MM2INT(30),minW), Point(endL,nrml)}));
	dist_and_widths_list.emplace_back(std::initializer_list<Point>({Point(0,nrml), Point(endL * 2 + MM2INT(30),nrml)}));

	coord_t n_lines = 4;
	Point current_pos(0, 0);
	for ( const std::vector<Point> & dist_and_widths : dist_and_widths_list)
	{
		line.junctions.emplace_back(current_pos, dist_and_widths.front().Y);
		for ( size_t idx = 0; idx < dist_and_widths.size() ; idx++ )
		{
			if (idx > 0)
				current_pos.X += dist_and_widths[idx].X;
			line.junctions.emplace_back(current_pos, dist_and_widths[idx].Y);
		}
		current_pos.Y += maxW + gap;
		line.junctions.emplace_back(current_pos, nrml);
		current_pos.X = 0;
		line.junctions.emplace_back(current_pos, nrml);
		current_pos.Y += maxW + MM2INT(1.0);
	}

	AABB aabb;
	for ( ExtrusionJunction & j : line.junctions)
		aabb.include(j.p);
	aabb.expand(maxW / 2);
	polys = aabb.toPolygons();
}

void widthLimitsTest(std::vector<std::list<ExtrusionLine>> & result_polylines_per_index, std::vector<std::list<ExtrusionLine>> & result_polygons_per_index, Polygons & polys)
{
	result_polygons_per_index.clear();
	result_polylines_per_index.clear();
	
	result_polylines_per_index.emplace_back();
	result_polylines_per_index.back().emplace_back();
	ExtrusionLine & line = result_polylines_per_index.back().back();
	
	coord_t minW = MM2INT(0.3);
	coord_t maxW = MM2INT(1.0);
	coord_t midW = (minW + maxW) / 2;
	coord_t nrml = MM2INT(0.4);
	
    coord_t sample_dist = MM2INT(0.4);
    
	coord_t gap = MM2INT(1.2);
	
	coord_t n_lines = 4;
    
    coord_t min_r = MM2INT(20.0);
    coord_t r = min_r;
    float max_a = 10.0 * 2*M_PI;
    
    float constant_cycles = 2.0 * 2*M_PI;
    
    Point prev(min_r, 0);
	for ( float a = 0; a < max_a; a += INT2MM(sample_dist) / INT2MM(r) )
	{
        Point current_pos(r * cos(a), r * sin(a));
        coord_t w = std::max(float(minW), maxW - (maxW - nrml) * std::max(0.0f, a - constant_cycles) / (max_a - 2*constant_cycles));
		line.junctions.emplace_back(current_pos, w);
        r = min_r + a / (2*M_PI) * gap;
        prev = current_pos;
	}

	Polygons polylines;
    PolygonRef polyline = polylines.newPoly();
	for ( ExtrusionJunction & j : line.junctions)
		polyline.add(j.p);
	polys = polylines.offsetPolyLine(gap).approxConvexHull(maxW - gap);
}

void test(std::string input_outline_filename, float input_outline_scaling, std::string output_prefix, std::string input_segment_file)
{
	bool is_svg = input_outline_filename.substr(input_outline_filename.find_last_of(".") + 1) == "svg";
    Polygons polys = is_svg? SVGloader::load(input_outline_filename) : TXTloader::load(input_outline_filename);
    polys.applyMatrix(PointMatrix::scale(input_outline_scaling));
    
    
    PathReader reader;
	if (reader.open(input_segment_file))
	{
		std::cerr << "Error opening " << input_segment_file << "!\n";
		std::exit(-1);
	}
    std::vector<std::list<ExtrusionLine>> result_polylines_per_index;
    std::vector<std::list<ExtrusionLine>> result_polygons_per_index;
	if (reader.read(result_polygons_per_index, result_polylines_per_index))
	{
		std::cerr << "Error reading " << input_segment_file << "!\n";
		std::exit(-1);
	}

// 	varWidthTest(result_polylines_per_index, result_polygons_per_index, polys);
// 	widthLimitsTest(result_polylines_per_index, result_polygons_per_index, polys);
// 	raftedPrint(result_polylines_per_index, result_polygons_per_index, polys, output_prefix, false);
// 	squareGridTest(result_polylines_per_index, result_polygons_per_index, polys, output_prefix);
	
//	raftedPrint(result_polylines_per_index, result_polygons_per_index, polys, output_prefix);
 	print(result_polylines_per_index, result_polygons_per_index, polys, output_prefix);

	std::cout << "Computing statistics...\n";
	Statistics stats("external", output_prefix, polys, -1.0);
	stats.analyse(result_polygons_per_index, result_polylines_per_index);
	stats.visualize(MM2INT(0.3), MM2INT(0.9));
	stats.saveResultsCSV();
}



} // namespace visualizer

int main(int argc, char *argv[])
{
    std::string input_outline_filename;
    std::string output_prefix;
    std::string input_segment_file;
    float input_outline_scaling = 1.0;
    if (argc >= 2) input_outline_filename = argv[1];
    if (argc >= 3) output_prefix = argv[2];
    if (argc >= 4) input_segment_file = argv[3];
    if (argc >= 5) input_outline_scaling = std::stof(argv[4]);
    long n = 1;
    for (int i = 0; i < n; i++)
    {
        visualizer::test(input_outline_filename, input_outline_scaling, output_prefix, input_segment_file);
//         if (++i % std::max(1l, n / 100) == 0)
//             std::cerr << (i / 100) << "%\n";
    }
    return 0;
}
