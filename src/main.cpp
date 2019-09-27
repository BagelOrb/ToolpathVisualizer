
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
coord_t layer_thickness = MM2INT(0.2);
float gamma = 0.2;

// raft settings
float nominal_raft_speed = 50.0;

void squareGridTest(const std::vector<std::list<ExtrusionLine>> & result_polylines_per_index, const std::vector<std::list<ExtrusionLine>> & result_polygons_per_index, const Polygons & polys, const std::string output_prefix)
{
	AABB aabb(polys);

    std::ostringstream ss;
    ss << "visualization/" << output_prefix << ".gcode";
	GcodeWriter gcode(ss.str(), GcodeWriter::type_UM3, true, layer_thickness, nominal_raft_speed, travel_speed, flow_modifier, true);
	
	
	coord_t sizer = 2;
	
	Polygons raft_outline; // = polys.offset(MM2INT(5.0), ClipperLib::jtRound);
	raft_outline.add(aabb.expanded(MM2INT(25) * sizer).toPolygon());
	raft_outline = raft_outline.offset(MM2INT(5.0), ClipperLib::jtRound);
	gcode.printRaft(raft_outline);

	gcode.switchExtruder(0);
	gcode.setNominalSpeed(nominal_print_speed);
	
	float gamma = 0.0;
	for ( int x = -sizer; x <= sizer; x++ )
	for ( int y = -sizer; y <= sizer; y++ )
	{
		Point translation = Point(x, y) * MM2INT(25);
		gcode.setTranslation(translation);
		
		gcode.setGamma(gamma);
		gcode.comment("Gamma:%f", gamma);
		
		
		gcode.comment("Pos:%i,%i", x, y);
		
		gcode.printBrim(aabb.toPolygons(), 1, MM2INT(0.4), MM2INT(0.6));
		
		gcode.printOrdered(result_polygons_per_index, result_polylines_per_index, false);
		
		gcode.retract();
		gamma += 0.01;
	}
}

void raftedPrint(const std::vector<std::list<ExtrusionLine>> & result_polylines_per_index, const std::vector<std::list<ExtrusionLine>> & result_polygons_per_index, const Polygons & polys, const std::string output_prefix)
{
	AABB aabb(polys);

    std::ostringstream ss;
    ss << "visualization/" << output_prefix << ".gcode";
	GcodeWriter gcode(ss.str(), GcodeWriter::type_UM3, true, layer_thickness, nominal_raft_speed, travel_speed, flow_modifier, true);
	
	Polygons raft_outline = polys.offset(MM2INT(6.0), ClipperLib::jtRound).offset(MM2INT(-3.0), ClipperLib::jtRound);
	gcode.printRaft(raft_outline);

	gcode.switchExtruder(0);
	gcode.setNominalSpeed(nominal_print_speed);
	gcode.setGamma(gamma);
	gcode.comment("gamma: %f", gamma);
	
	gcode.printBrim(polys, 1, MM2INT(0.4), MM2INT(0.6));
	
	gcode.print(result_polygons_per_index, result_polylines_per_index, false, false);
}

void varWidthTest(std::vector<std::list<ExtrusionLine>> & result_polylines_per_index, std::vector<std::list<ExtrusionLine>> & result_polygons_per_index, Polygons & polys)
{
	result_polygons_per_index.clear();
	result_polylines_per_index.clear();
	
	result_polylines_per_index.emplace_back();
	result_polylines_per_index.back().emplace_back();
	ExtrusionLine & line = result_polylines_per_index.back().back();
	
	coord_t min = MM2INT(0.3);
	coord_t max = MM2INT(1.0);
	coord_t mid = (min + max) / 2;
	
	coord_t gap = MM2INT(0.1);
	
	std::list<std::vector<Point>> dist_and_widths_list;
	dist_and_widths_list.emplace_back(std::initializer_list<Point>({Point(MM2INT(50),mid), Point(MM2INT(50),mid)}));
	dist_and_widths_list.emplace_back(std::initializer_list<Point>({Point(MM2INT(50),mid), Point(MM2INT(50),mid)}));
	dist_and_widths_list.emplace_back(std::initializer_list<Point>({Point(MM2INT(50),max), Point(MM2INT(50),min)}));
	dist_and_widths_list.emplace_back(std::initializer_list<Point>({Point(MM2INT(50),min), Point(MM2INT(50),max)}));
	dist_and_widths_list.emplace_back(std::initializer_list<Point>({Point(MM2INT(50),max), Point(MM2INT(50),min)}));
	dist_and_widths_list.emplace_back(std::initializer_list<Point>({Point(MM2INT(50),min), Point(MM2INT(50),max)}));
	dist_and_widths_list.emplace_back(std::initializer_list<Point>({Point(MM2INT(10),max), Point(MM2INT(10),min), Point(MM2INT(10),max), Point(MM2INT(10),min), Point(MM2INT(10),max), Point(MM2INT(10),min)}));
	dist_and_widths_list.emplace_back(std::initializer_list<Point>({Point(MM2INT(10),min), Point(MM2INT(10),max), Point(MM2INT(10),min), Point(MM2INT(10),max), Point(MM2INT(10),min), Point(MM2INT(10),max)}));
	dist_and_widths_list.emplace_back(std::initializer_list<Point>({Point(MM2INT(5),max), Point(MM2INT(5),min), Point(MM2INT(5),max), Point(MM2INT(5),min), Point(MM2INT(5),max), Point(MM2INT(5),min), Point(MM2INT(5),max), Point(MM2INT(5),min), Point(MM2INT(5),max), Point(MM2INT(5),min), Point(MM2INT(5),max)}));
	dist_and_widths_list.emplace_back(std::initializer_list<Point>({Point(MM2INT(5),min), Point(MM2INT(5),max), Point(MM2INT(5),min), Point(MM2INT(5),max), Point(MM2INT(5),min), Point(MM2INT(5),max), Point(MM2INT(5),min), Point(MM2INT(5),max), Point(MM2INT(5),min), Point(MM2INT(5),max), Point(MM2INT(5),min)}));
	
	for ( std::vector<Point> & dist_and_widths : dist_and_widths_list)
	{
		dist_and_widths.emplace_back(MM2INT(10), (min + max) / 2);
	}
		
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
		current_pos.Y += (max + min ) / 2 + gap;
// 		line.junctions.emplace_back(current_pos, dist_and_widths.back().Y);
		for ( coord_t idx = dist_and_widths.size() - 1 ; idx >= 0 ; idx-- )
		{
			if (idx < dist_and_widths.size() - 1)
				current_pos.X -= dist_and_widths[idx + 1].X;
			line.junctions.emplace_back(current_pos, max + min - dist_and_widths[idx].Y);
		}
		current_pos.Y += max + gap;
	}

	AABB aabb;
	for ( ExtrusionJunction & j : line.junctions)
		aabb.include(j.p);
	aabb.expand(max / 2);
	polys = aabb.toPolygons();
}

void test(std::string input_outline_filename, std::string output_prefix, std::string input_segment_file)
{
	bool is_svg = input_outline_filename.substr(input_outline_filename.find_last_of(".") + 1) == "svg";
    Polygons polys = is_svg? SVGloader::load(input_outline_filename) : TXTloader::load(input_outline_filename);
    
    
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

	varWidthTest(result_polylines_per_index, result_polygons_per_index, polys);
	
	AABB aabb(polys);
	{
		SVG svg("visualization/points.svg", aabb);
		svg.writeAreas(polys);
		for (auto& lines : result_polygons_per_index)
			for (auto& line : lines)
				for (auto& j : line.junctions)
					svg.writePoint(j.p, false, 0.25);
	}
	
	raftedPrint(result_polylines_per_index, result_polygons_per_index, polys, output_prefix);

	std::cout << "Computing statistics...\n";
	Statistics stats("external", output_prefix, polys, -1.0);
	stats.analyse(result_polygons_per_index, result_polylines_per_index);
	stats.visualize(MM2INT(0.3), MM2INT(1.0));
	stats.saveResultsCSV();
}



} // namespace visualizer

int main(int argc, char *argv[])
{
    std::string input_outline_filename;
    std::string output_prefix;
    std::string input_segment_file;
    if (argc >= 2) input_outline_filename = argv[1];
    if (argc >= 3) output_prefix = argv[2];
    if (argc >= 4) input_segment_file = argv[3];
    long n = 1;
    for (int i = 0; i < n; i++)
    {
        visualizer::test(input_outline_filename, output_prefix, input_segment_file);
//         if (++i % std::max(1l, n / 100) == 0)
//             std::cerr << (i / 100) << "%\n";
    }
    return 0;
}
