//Copyright (c) 2019 Ultimaker B.V.


#include "Statistics.h"

#include <sstream>
#include <fstream>

#include "utils/logoutput.h"

namespace visualizer
{
    
extern coord_t preferred_bead_width;

void Statistics::analyse(std::vector<std::list<ExtrusionLine>>& polygons_per_index, std::vector<std::list<ExtrusionLine>>& polylines_per_index, bool perform_overfill_underfill_analysis)
{
    this->polygons_per_index = &polygons_per_index;
    this->polylines_per_index = &polylines_per_index;

    generateAllSegments(polygons_per_index, polylines_per_index);
    
    if (perform_overfill_underfill_analysis)
    {
        for (coord_t segment_idx = 0; segment_idx < all_segments.size(); segment_idx++)
        {
            Segment s = all_segments[segment_idx];
            Polygons covered = s.s.toPolygons(false);
            area_covered.add(covered);
            Polygons extruded = s.toPolygons();
            overlaps.add(extruded);
        }
        
        area_covered = area_covered.execute(ClipperLib::pftNonZero);

        overfills = overlaps;
        for (PolygonRef poly : area_covered)
        {
            PolygonRef new_poly = overfills.newPoly();
            for (coord_t point_idx = poly.size() - 1; point_idx >= 0; --point_idx)
            {
                new_poly.add(poly[point_idx]);
            }
        }
        overfills.add(area_covered.difference(input));

        double_overfills = overfills;
        for (PolygonRef poly : area_covered)
        {
            PolygonRef new_poly = double_overfills.newPoly();
            for (coord_t point_idx = poly.size() - 1; point_idx >= 0; --point_idx)
            {
                new_poly.add(poly[point_idx]);
            }
        }
        overfills = overfills.execute(ClipperLib::pftPositive);
        overfills = overfills.intersection(area_covered);
        overfills = overfills.offset(-5);
        overfills = overfills.offset(10);
        overfills = overfills.offset(-5);

        double_overfills = double_overfills.execute(ClipperLib::pftPositive);
        double_overfills = double_overfills.offset(-5);
        double_overfills = double_overfills.offset(10);
        double_overfills = double_overfills.offset(-5);

        overfill_area = INT2MM2(overfills.area());
        double_overfill_area = INT2MM2(double_overfills.area());
        double total_overfill_area = overfill_area + double_overfill_area;
    //     logAlways("Total overfill area: %f mm²\n", total_overfill_area);

        underfills = input.difference(area_covered);
        underfills = underfills.offset(5);
        underfills = underfills.offset(-10);
        underfills = underfills.offset(5);

        total_underfill_area = INT2MM2(underfills.area());
        
        outer_underfills = underfills;
        outer_underfills = outer_underfills.difference(input.offset(-preferred_bead_width, ClipperLib::jtRound));
        std::vector<PolygonsPart> parts = outer_underfills.splitIntoParts();
        outer_underfills.clear();
        Polygons inside_outline = input.offset(-50);
        for (PolygonsPart& part : parts)
        {
            Polygons outline_intersection = part.difference(inside_outline);
            if (!outline_intersection.empty())
            {
                outer_underfills.add(part);
            }
        }
        outer_underfill_area = INT2MM2(outer_underfills.area());
    }

//     logAlways("Total underfill area: %f mm²\n", total_underfill_area);
//     std::vector<PolygonsPart> underfill_areas = underfills.splitIntoParts();
//     logAlways("Average area: %f mm² over %d parts\n", total_underfill_area / underfill_areas.size(), underfill_areas.size());

//     logAlways("Total target area: %f mm²\n", total_target_area);

    // initialize paths
    for (Segment& segment : all_segments)
    {
        PolygonRef poly = paths.newPoly();
        poly.emplace_back(segment.s.from.p);
        poly.emplace_back(segment.s.to.p);
    }

    closed_toolpaths = 0;
    for (const auto& polys : polygons_per_index)
        closed_toolpaths += polys.size();
    open_toolpaths = 0;
    for (const auto& lines : polylines_per_index)
        open_toolpaths += lines.size();

    total_toolpath_length = 0;
    for (Segment& segment : all_segments)
    {
        total_toolpath_length += vSizeMM(segment.s.to.p - segment.s.from.p);
    }
    
    auto bin_corner = [this](Point prev, Point here, Point next)
    {
        Point v1 = next - here;
        Point v2 = prev - here;
        float dott = INT2MM2(dot(v1, v2));
        float v1_size = vSizeMM(v1);
        float v2_size = vSizeMM(v2);
        float p = std::min(1.0f, std::max(-1.0f, dott / v1_size / v2_size));
        float angle = acos(p) * 180 / M_PI;
        angle_bins[int(angle / angle_bin_size + 0.5)]++;
    };
    
    for (const auto& polys : polygons_per_index)
    {
        for (const auto& poly : polys)
        {
            Point prev = (--(--poly.junctions.end()))->p;
            Point here = poly.junctions.back().p;
            for (const ExtrusionJunction& j : poly.junctions)
            {
                Point next = j.p;
                if (next == here) continue;
                bin_corner(prev, here, next);
                prev = here;
                here = next;
            }
        }
    }
    
    for (const auto& lines : polygons_per_index)
    {
        for (const auto& line : lines )
        {
            auto it = line.junctions.begin();
            Point prev = it->p;
            ++it;
            Point here = it->p;
            ++it;
            for (; it != line.junctions.end(); ++it)
            {
                Point next = it->p;
                if (next == here) continue;
                bin_corner(prev, here, next);
                prev = here;
                here = next;
            }
        }
    }
    
    
    for (Segment& segment : all_segments)
    {
        coord_t a = std::min(segment.s.from.w, segment.s.to.w);
        coord_t b = std::max(segment.s.from.w, segment.s.to.w);
        float length = vSizeMM(segment.s.from.p - segment.s.to.p);
        float segment_length = length / INT2MM(b - a) * INT2MM(width_bin_size); 
        coord_t start_bin = a / width_bin_size;
        coord_t end_bin = b / width_bin_size;
        if (start_bin == end_bin)
        {
            if (start_bin >= 0 && start_bin < width_bins.size())
                width_bins[start_bin] += length;
        }
        else
        {
            coord_t start_bin_w = (start_bin + 1) * width_bin_size;
            if (start_bin >= 0 && start_bin < width_bins.size())
                width_bins[start_bin] += segment_length * (start_bin_w - a) / width_bin_size;
            for (coord_t bin = start_bin + 1; bin < end_bin; bin++)
                if (bin >= 0 && bin < width_bins.size())
                    width_bins[bin] += segment_length;
            coord_t end_bin_w = end_bin * width_bin_size;
            if (end_bin >= 0 && end_bin < width_bins.size())
                width_bins[end_bin] += segment_length * (b - end_bin_w) / width_bin_size;
        }
    }
    
    float total_binned_path_length = 0;
    for (float bin_length : width_bins)
        total_binned_path_length += bin_length;
    if (std::abs( total_binned_path_length - total_toolpath_length) / total_toolpath_length > 0.001)
        logWarning("Total binned length (%d) doesn't coincide with total path length (%d)!\n", total_binned_path_length, total_toolpath_length);
}

void Statistics::saveSegmentsCSV()
{
    if ( ! all_segments.empty())
    {
        std::ostringstream ss;
        ss << "visualization/" << output_prefix << "_" << test_type << "_segments.csv";
        std::ofstream csv(ss.str(), std::ofstream::out | std::ofstream::trunc);
        csv << "from_x,from_y,from_width,to_x,to_y,to_width,output_prefix,inset_index\n";
        for (const Segment& segment : all_segments)
        {
            if (segment.s.from.perimeter_index != segment.s.to.perimeter_index)
                std::cerr << "Inset index doesn't correspond!\n";
            csv << segment.s.from.p.X << "," << segment.s.from.p.Y << "," << segment.s.from.w << ","
                << segment.s.to.p.X << "," << segment.s.to.p.Y << "," << segment.s.to.w << ","
                << test_type << "," << output_prefix << ","
                << segment.s.from.perimeter_index << '\n';
        }
        csv.close();
    }
}
void Statistics::saveResultsCSV()
{
    {
        coord_t vert_count = 0;
        for (ConstPolygonRef poly : input)
            for (const Point& p : poly)
                vert_count++;
        std::ostringstream ss;
        ss << "visualization/" << output_prefix << "_" << test_type << "_results.csv";
        std::ofstream csv(ss.str(), std::ofstream::out | std::ofstream::trunc);
        csv << "print_time,overfill_area,double_overfill_area,total_underfill_area,outer_underfill_area,"
            << "total_target_area,total_target_area_length,vert_count,"
            << "test_type,output_prefix,"
            << "closed_toolpaths,open_toolpaths,total_toolpath_length\n";
        csv << print_time << "," << overfill_area << "," << double_overfill_area << "," << total_underfill_area << "," << outer_underfill_area << ","
            << total_target_area << "," << total_target_area_length << "," << vert_count << ","
            << test_type << "," << output_prefix << ","
            << closed_toolpaths << "," << open_toolpaths << "," << total_toolpath_length << '\n';
        csv.close();
    }
}
void Statistics::saveAnglesCSV()
{
    coord_t vert_count = 0;
    for (ConstPolygonRef poly : input)
        for (const Point& p : poly)
            vert_count++;
    std::ostringstream ss;
    ss << "visualization/" << output_prefix << "_" << test_type << "_angles.csv";

    std::ofstream csv(ss.str(), std::ofstream::out | std::ofstream::trunc);
    for (int bin_idx = 0; bin_idx < angle_bins.size(); bin_idx++)
        csv << (float(bin_idx) * angle_bin_size) << ",";
    csv << '\n';
    for (int bin_idx = 0; bin_idx < angle_bins.size(); bin_idx++)
        csv << angle_bins[bin_idx] << ",";
    csv << '\n';
    csv.close();
}
void Statistics::saveWidthsCSV()
{
    coord_t vert_count = 0;
    for (ConstPolygonRef poly : input)
        for (const Point& p : poly)
            vert_count++;
    std::ostringstream ss;
    ss << "visualization/" << output_prefix << "_" << test_type << "_widths.csv";

    std::ofstream csv(ss.str(), std::ofstream::out | std::ofstream::trunc);
    for (int bin_idx = 0; bin_idx < width_bins.size(); bin_idx++)
        csv << (float(bin_idx) * width_bin_size) << ",";
    csv << '\n';
    for (int bin_idx = 0; bin_idx < width_bins.size(); bin_idx++)
        csv << width_bins[bin_idx] << ",";
    csv << '\n';
    csv.close();
}

void Statistics::generateAllSegments(std::vector<std::list<ExtrusionLine>>& polygons_per_index, std::vector<std::list<ExtrusionLine>>& polylines_per_index)
{
    for (std::list<ExtrusionLine>& polygons : polygons_per_index)
    {
        for (ExtrusionLine& polygon : polygons)
        {
            auto last_it = --polygon.junctions.end();
            for (auto junction_it = polygon.junctions.begin(); junction_it != polygon.junctions.end(); ++junction_it)
            {
                ExtrusionJunction& junction = *junction_it;
                ExtrusionSegment segment(*last_it, junction, false);
                all_segments.emplace_back(segment, false);
                last_it = junction_it;
            }
        }
    }
    for (std::list<ExtrusionLine>& polylines : polylines_per_index)
    {
        for (ExtrusionLine& polyline : polylines)
        {
            auto last_it = polyline.junctions.begin();
            for (auto junction_it = ++polyline.junctions.begin(); junction_it != polyline.junctions.end(); ++junction_it)
            {
                ExtrusionJunction& junction = *junction_it;
                ExtrusionSegment segment(*last_it, junction, false);
                all_segments.emplace_back(segment, junction_it == --polyline.junctions.end());
                last_it = junction_it;
            }
        }
    }
}

void Statistics::visualize(coord_t min_nozzle_size, coord_t max_nozzle_size, bool output_toolpaths, bool output_widths, bool include_legend, bool visualize_accuracy, bool exaggerate_widths, bool rounded_visualization)
{
    AABB aabb(input);
	
	float rounded_viz_growth_factor = 1.4;

    if (output_toolpaths)
    {
        std::ostringstream ss;
        ss << "visualization/" << output_prefix << "_" << test_type << "_toolpaths.svg";
        SVG svg(ss.str(), aabb);
        svg.writeAreas(input, SVG::Color::GRAY, SVG::Color::NONE, 2);
        svg.nextLayer();
        bool alternate = true;
        for (PolygonRef poly : overlaps)
        {
            svg.writeAreas(poly, alternate? SVG::Color::BLUE : SVG::Color::MAGENTA, SVG::Color::NONE);
            alternate = !alternate;
        }
        svg.writePolylines(paths, SVG::Color::BLACK, 2);
    }

    if (visualize_accuracy)
    {
        std::ostringstream ss;
        ss << "visualization/" << output_prefix << "_" << test_type << "_accuracy.svg";
        SVG svg(ss.str(), aabb);
//         svg.writeAreas(input, SVG::Color::NONE, SVG::Color::BLACK, 3);
//         svg.nextLayer();
//         Polygons connecteds = PolygonUtils::connect(area_covered);
//         for (PolygonRef connected : connecteds)
//             svg.writeAreas(connected, SVG::Color::BLACK, SVG::Color::NONE);
        for (float w = .9; w > .25; w = 1.0 - (1.0 - w) * rounded_viz_growth_factor)
        {
            Polygons polys;
            for (coord_t segment_idx = 0; segment_idx < all_segments.size(); segment_idx++)
            {
                Segment s = all_segments[segment_idx];
                s.s.from.w *= w / .9;
                s.s.to.w *= w / .9;
                Polygons covered = s.s.toPolygons(false);
                polys.add(covered);
            }
            int c = 255 - 200 * (w - .25);
            SVG::ColorObject clr(c, c, c);
            polys = polys.execute(ClipperLib::pftNonZero);
            polys = PolygonUtils::connect(polys);
            for (PolygonRef connected : polys)
                svg.writeAreas(connected, clr, SVG::Color::NONE);
            if (!rounded_visualization) break;
        }
        svg.nextLayer();
        svg.writePolylines(paths, SVG::Color::BLACK, 2);
        svg.nextLayer();
        svg.writeAreas(underfills, SVG::ColorObject(0,128,255), SVG::Color::NONE);
        svg.nextLayer();
        svg.writeAreas(outer_underfills, SVG::ColorObject(0,255,128), SVG::Color::NONE);
        svg.nextLayer();
        svg.writeAreas(overfills, SVG::Color::ORANGE, SVG::Color::NONE);
        svg.nextLayer();
        svg.writeAreas(double_overfills, SVG::Color::ORANGE, SVG::Color::NONE);
        svg.nextLayer();
        svg.writePolygons(input);
    }

    if (output_widths)
    {
        std::ostringstream ss;
        ss << "visualization/" << output_prefix << "_" << test_type << "_widths.svg";
        SVG svg(ss.str(), aabb);
//         svg.writeAreas(input, SVG::Color::GRAY, SVG::Color::NONE, 2);

		coord_t normal_nozzle_size = preferred_bead_width;
        coord_t min_dev = normal_nozzle_size - min_nozzle_size;
        coord_t max_dev = max_nozzle_size - normal_nozzle_size;
        coord_t min_w = 30;

        // add legend
        std::vector<Segment> all_segments_plus = all_segments;
        if (include_legend)
        {
            auto to_string = [](float v)
            {
                std::ostringstream ss;
                ss << v;
                return ss.str();
            };
            AABB aabb(input);
            ExtrusionJunction legend_btm(Point(aabb.max.X + normal_nozzle_size + max_dev, aabb.max.Y), normal_nozzle_size - min_dev, 0);
            ExtrusionJunction legend_top(Point(aabb.max.X + normal_nozzle_size + max_dev, aabb.min.Y), normal_nozzle_size + max_dev, 0);
            ExtrusionJunction legend_mid(legend_btm.p + (legend_top.p - legend_btm.p) * min_dev / (max_dev + min_dev), normal_nozzle_size, 0);
            legend_btm.p += (legend_mid.p - legend_btm.p) / 4;
            legend_top.p += (legend_mid.p - legend_top.p) / 4;
            ExtrusionSegment legend_segment(legend_btm, legend_top, true);
            svg.writeAreas(legend_segment.toPolygons(false), SVG::ColorObject(200,200,200), SVG::Color::NONE); // real outline
            all_segments_plus.emplace_back(legend_segment, true); // colored
            Point legend_text_offset(normal_nozzle_size, 0);
            svg.writeText(legend_top.p + legend_text_offset, to_string(INT2MM(legend_top.w)));
            svg.writeText(legend_btm.p + legend_text_offset, to_string(INT2MM(legend_btm.w)));
            svg.writeText(legend_mid.p + legend_text_offset, to_string(INT2MM(legend_mid.w)));
            svg.writeLine(legend_top.p, legend_top.p + legend_text_offset);
            svg.writeLine(legend_btm.p, legend_btm.p + legend_text_offset);
            svg.writeLine(legend_mid.p, legend_mid.p + legend_text_offset);
        }


//         Point3 middle = rounded_visualization? Point3(255,255,255) : Point3(192,192,192);
        Point3 middle(255,255,255);
        Point3 wide(255,0,0);
        Point3 narrow(0,0,255);
        
//         Polygons connecteds = PolygonUtils::connect(area_covered);
//         for (PolygonRef connected : connecteds)
//             svg.writeAreas(connected, SVG::Color::BLACK, SVG::Color::NONE);
        
        for (float w = .9; w > .25; w = 1.0 - (1.0 - w) * rounded_viz_growth_factor)
        {
            int brightness = rounded_visualization? 255 - 200 * (w - .25) : 192;
            for (coord_t segment_idx = 0; segment_idx < all_segments_plus.size(); segment_idx++)
            {
                Segment ss = all_segments_plus[segment_idx];
//                 ss.s.from.w *= w;
//                 ss.s.to.w *= w;
                for (Segment s : discretize(ss, MM2INT(0.1)))
                {
                    coord_t avg_w = (s.s.from.w + s.s.to.w) / 2;
                    Point3 clr;
                    coord_t dev = (avg_w > normal_nozzle_size)? max_dev : min_dev;
                    float color_ratio = std::min(1.0, double(std::abs(avg_w - normal_nozzle_size)) / dev);
                    color_ratio = color_ratio * .5 + .5 * sqrt(color_ratio);
                    if (avg_w > normal_nozzle_size)
                    {
                        clr = wide * color_ratio + middle * (1.0 - color_ratio );
                    }
                    else
                    {
                        clr = narrow * color_ratio + middle * (1.0 - color_ratio );
                    }
                    clr = clr * brightness / 255;
                        
    //                 coord_t clr_max = std::max(clr.x, std::max(clr.y, clr.z));
    //                 clr = clr * 255 / clr_max;

    //                 clr.y = clr.y * (255 - 92 * clr.dot(green) / green.vSize() / 255) / 255;
                    if (exaggerate_widths)
                    {
                        s.s.from.w = std::max(min_w, min_w + (s.s.from.w - (normal_nozzle_size - dev)) * 5 / 4);
                        s.s.to.w = std::max(min_w, min_w + (s.s.to.w - (normal_nozzle_size - dev)) * 5 / 4);
                    }
//                     else
//                     {
//                         s.s.from.w *= 0.9;
//                         s.s.to.w *= 0.9;
//                     }
                    s.s.from.w *= w / .9;
                    s.s.to.w *= w / .9;
                    Polygons covered = s.toPolygons();
                    svg.writeAreas(covered, SVG::ColorObject(clr.x, clr.y, clr.z), SVG::Color::NONE);
                }
            }
            if (!rounded_visualization) break;
            svg.nextLayer();
        }
//         svg.nextLayer();
//         svg.writeAreas(underfills, SVG::ColorObject(255,0,255), SVG::Color::NONE);
//         svg.nextLayer();
//         svg.writeAreas(overfills, SVG::ColorObject(255,255,0), SVG::Color::NONE);
//         svg.writeAreas(double_overfills, SVG::ColorObject(255,0,0), SVG::Color::NONE);
    }
}

std::vector<Statistics::Segment> Statistics::discretize(const Segment& segment, coord_t step_size)
{
    ExtrusionSegment extrusion_segment = segment.s;
    Point a = extrusion_segment.from.p;
    Point b = extrusion_segment.to.p;
    Point ab = b - a;
    coord_t ab_length = vSize(ab);
    coord_t step_count = std::max(static_cast<coord_t>(1), (ab_length + step_size / 2) / step_size);
    std::vector<Segment> discretized;
    ExtrusionJunction from = extrusion_segment.from;
    for (coord_t step = 0; step < step_count; step++)
    {
        ExtrusionJunction mid(a + ab * (step + 1) / step_count, extrusion_segment.from.w + (extrusion_segment.to.w - extrusion_segment.from.w) * (step + 1) / step_count, extrusion_segment.from.perimeter_index);
        discretized.emplace_back(ExtrusionSegment(from, mid, segment.s.is_odd), false);
        from = mid;
    }
    discretized.back().is_full = segment.is_full;
    return discretized;
}

} // namespace visualizer
