//Copyright (c) 2019 Ultimaker B.V.


#ifndef STATISTICS_H
#define STATISTICS_H

#include "utils/polygon.h"
#include "utils/ExtrusionJunction.h"
#include "utils/ExtrusionSegment.h"
#include "utils/ExtrusionLine.h"
namespace visualizer
{

/*!
 * Get statistics of the resulting toolpaths
 */
class Statistics
{
public:
    Statistics(std::string test_type, std::string output_prefix, Polygons& input, double print_time)
    : print_time(print_time)
    , test_type(test_type)
    , output_prefix(output_prefix)
    , input(input)
    {
        input = input.processEvenOdd();
        total_target_area = INT2MM2(input.area());
        total_target_area_length = INT2MM(input.polygonLength());
    }
    void analyse(std::vector<std::list<ExtrusionLine>>& polygons_per_index, std::vector<std::list<ExtrusionLine>>& polylines_per_index);
    void visualize(coord_t min_nozzle_size, coord_t max_nozzle_size, bool output_toolpaths = false, bool output_widths = true, bool include_legend = false, bool output_accuracy = true, bool exaggerate_widths = false, bool rounded_visualization = true);
    void saveResultsCSV();
    double print_time = -1;
    double overfill_area = -1;
    double double_overfill_area = -1;
    double total_underfill_area = -1;
    double total_target_area = -1;
    double total_target_area_length = -1;
    int closed_toolpaths = -1;
    int open_toolpaths = -1;
private:
    struct Segment
    {
        ExtrusionSegment s;
        bool is_full;
        Segment(ExtrusionSegment s, bool is_full)
        : s(s)
        , is_full(is_full)
        {}
        Polygons toPolygons()
        {
            return s.toPolygons(!is_full);
        }
    };
    std::string test_type;
    std::string output_prefix;
    const Polygons& input;

    std::vector<std::list<ExtrusionLine>>* polygons_per_index;
    std::vector<std::list<ExtrusionLine>>* polylines_per_index;
    std::vector<Segment> all_segments;
    Polygons area_covered;
    Polygons overlaps;
    Polygons underfills;
    Polygons overfills;
    Polygons double_overfills;
    Polygons paths;

    void generateAllSegments(std::vector<std::list<ExtrusionLine>>& polygons_per_index, std::vector<std::list<ExtrusionLine>>& polylines_per_index);

    std::vector<Segment> discretize(const Segment& ss, coord_t step_size);
};




} // namespace visualizer
#endif // STATISTICS_H
