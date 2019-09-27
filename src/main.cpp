
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
	
	Polygons raft_outline = polys.offset(MM2INT(10.0), ClipperLib::jtRound).offset(MM2INT(-5.0), ClipperLib::jtRound);
	gcode.printRaft(raft_outline);

	gcode.switchExtruder(0);
	gcode.setNominalSpeed(nominal_print_speed);
	gcode.setGamma(gamma);
	gcode.comment("gamma: %f", gamma);
	
	gcode.printBrim(polys, 1, MM2INT(0.4), MM2INT(0.6));
	
	gcode.print(result_polygons_per_index, result_polylines_per_index, false, false);
}

void test(std::string input_outline_filename, std::string output_prefix, std::string input_segment_file)
{
	bool is_svg = input_outline_filename.substr(input_outline_filename.find_last_of(".") + 1) == "svg";
    Polygons polys = is_svg? SVGloader::load(input_outline_filename) : TXTloader::load(input_outline_filename);
	AABB aabb(polys);
    
    
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
