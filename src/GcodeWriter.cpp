//Copyright (c) 2019 Ultimaker B.V.


#include "GcodeWriter.h"

#include <algorithm>
#include <iomanip>

#include "pathOrderOptimizer.h"
#include "OrderOptimizer.h"
#include "utils/logoutput.h"
#include "utils/ExtrusionSegment.h"

namespace visualizer
{
GcodeWriter::GcodeWriter(std::string filename, int type, bool dual_extrusion, coord_t layer_thickness, float print_speed, float travel_speed, float extrusion_multiplier, bool equalize_flow)
: file(filename.c_str())
, type(type)
, layer_thickness(layer_thickness)
, print_speed(print_speed)
, travel_speed(travel_speed)
, extrusion_multiplier(extrusion_multiplier)
, equalize_flow(equalize_flow)
, flow(print_speed * INT2MM(layer_thickness) * 0.4)
, gamma(0.0)
{
    assert(file.good());

	file << std::setprecision(5);
	file << std::fixed;
	
    file << ";START_OF_HEADER\n";
    file << ";HEADER_VERSION:0.1\n";
    file << ";FLAVOR:Griffin\n";
    file << ";GENERATOR.NAME:Cura_SteamEngine\n";
    file << ";GENERATOR.VERSION:4.2.1\n";
    file << ";GENERATOR.BUILD_DATE:2019-08-01\n";
    file << ";TARGET_MACHINE.NAME:Ultimaker 3\n";
    file << ";EXTRUDER_TRAIN.0.INITIAL_TEMPERATURE:210\n";
    file << ";EXTRUDER_TRAIN.0.MATERIAL.VOLUME_USED:109\n";
    file << ";EXTRUDER_TRAIN.0.MATERIAL.GUID:506c9f0d-e3aa-4bd4-b2d2-23e2425b1aa9\n";
    file << ";EXTRUDER_TRAIN.0.NOZZLE.DIAMETER:0.4\n";
    file << ";EXTRUDER_TRAIN.0.NOZZLE.NAME:AA 0.4\n";
	if (dual_extrusion)
	{
	    file << ";EXTRUDER_TRAIN.1.INITIAL_TEMPERATURE:210\n";
	    file << ";EXTRUDER_TRAIN.1.MATERIAL.VOLUME_USED:1150\n";
	    file << ";EXTRUDER_TRAIN.1.MATERIAL.GUID:44a029e6-e31b-4c9e-a12f-9282e29a92ff\n";
	    file << ";EXTRUDER_TRAIN.1.NOZZLE.DIAMETER:0.4\n";
	    file << ";EXTRUDER_TRAIN.1.NOZZLE.NAME:AA 0.4\n";
	}
    file << ";BUILD_PLATE.TYPE:glass\n";
    file << ";BUILD_PLATE.INITIAL_TEMPERATURE:60\n";
    file << ";BUILD_VOLUME.TEMPERATURE:28\n";
    file << ";PRINT.TIME:57\n";
    file << ";PRINT.GROUPS:1\n";
    file << ";PRINT.SIZE.MIN.X:9\n";
    file << ";PRINT.SIZE.MIN.Y:6\n";
    file << ";PRINT.SIZE.MIN.Z:0.27\n";
    file << ";PRINT.SIZE.MAX.X:126.29\n";
    file << ";PRINT.SIZE.MAX.Y:117.29\n";
    file << ";PRINT.SIZE.MAX.Z:2\n";
    file << ";END_OF_HEADER\n";
    file << ";Generated with libVisualizer\n";
    file << "T1\n"; current_extruder = 1;
    file << "M82 ;absolute extrusion mode\n";
    file << "\n";
    file << "G92 E0\n";
    file << "M109 S210\n";
//     file << "G0 F1500 X9 Y6 Z2\n"; // for T0
    file << "G0 F1500 X204 Y6 Z2\n";
    file << "G280\n";
    file << "G1 F1500 E-" << retraction_distance << "\n";
    file << ";LAYER_COUNT:1\n";
    file << ";LAYER:0\n";
    file << "M107\n";
    file << "M204 S625; set acceleration\n";
    file << "M205 X6 Y6; set jerk\n";
    file << "G0 F" << 60.0 * travel_speed << " X" << INT2MM(build_plate_middle.X / 2) << " Y" << INT2MM(build_plate_middle.Y / 2) << " Z" << INT2MM(layer_thickness) << " ; start location\n";
	cur_z = layer_thickness;
    is_retracted = true;
    file << "\n";
//     file << "M214 K1.0 ; linear advance\n";
//     file << "M83 ;relative extrusion mode\n";
    file << "\n";
    cur_pos = build_plate_middle;
	setTranslation(Point(0, 0));
}

GcodeWriter::~GcodeWriter()
{
	std::cout << "Total naive print time: " << (total_naive_print_time / 60.0) << " minutes.\n";


    file << "G0 F" << 60.0 * travel_speed << " X" << 20 << " Y" << 20 << " Z" << (INT2MM(layer_thickness) + 0.18) << " ; start location\n";
//     file << "M214 K0.0\n";
    file << "M107\n";
    file.close();
}

void GcodeWriter::setGamma(float gamma)
{
	this->gamma = gamma;
}

void GcodeWriter::setTemp(int temp)
{
    file << "M109 S" << temp << "\n";
}

void GcodeWriter::printBrim(const Polygons& outline, coord_t count, coord_t w, coord_t dist)
{
    std::vector<std::list<ExtrusionLine>> polygons_per_index;
    std::vector<std::list<ExtrusionLine>> polylines_per_index;
    polygons_per_index.resize(1);
    std::list<ExtrusionLine>& polygons = polygons_per_index[0];

    Polygons prev = outline;
    prev = prev.offset(dist - w);
    
    for (int i = 0; i < count; i++)
    {
        Polygons skuurt = prev.offset(w, ClipperLib::jtRound);
        for (PolygonRef poly : skuurt)
        {
            polygons.emplace_back(0);
            ExtrusionLine& polygon = polygons.back();
            for (Point p : poly)
            {
                polygon.junctions.emplace_back(p, w, 0);
            }
        }
        prev = skuurt;
    }
    print(polygons_per_index, polylines_per_index);
}

void GcodeWriter::printRaft(const Polygons& outline)
{
	coord_t spacing_0 = MM2INT(0.7);
	coord_t line_width_0 = MM2INT(0.6);
	coord_t layer_thickness_0 = MM2INT(0.3);
	cur_z = layer_thickness_0;
	file << "G0 Z" << INT2MM(cur_z) << '\n';
	printBrim(outline, 1, line_width_0, 0);
	Polygons inset_outline = outline.offset(0);//-line_width_0 / 2);
	std::vector<ExtrusionLine> lines = generateLines(inset_outline, line_width_0, spacing_0, 45.0);
	printLinesByOptimizer(lines);
	coord_t spacing_1 = MM2INT(0.3);
	coord_t line_width_1 = MM2INT(0.3);
	coord_t layer_thickness_1 = MM2INT(0.1);
	cur_z += layer_thickness_1;
	file << "G0 Z" << INT2MM(cur_z) << '\n';
	lines = generateLines(outline, line_width_1, spacing_1, -45.0);
	printLinesByOptimizer(lines);
	
	cur_z += layer_thickness;
	file << "G0 Z" << INT2MM(cur_z) << '\n';
}


std::vector<ExtrusionLine> GcodeWriter::generateLines(const Polygons& outline, coord_t line_width, coord_t spacing, float angle)
{
	Polygons outline_rotated = outline;
	outline_rotated.applyMatrix(PointMatrix(angle));
	std::vector<ExtrusionLine> lines = generateLines(outline_rotated, line_width, spacing);
	for (auto& line : lines)
	{
		for (auto& j : line.junctions)
		{
			j.p = PointMatrix(-angle).apply(j.p);
		}
	}
	return lines;
}

std::vector<ExtrusionLine> GcodeWriter::generateLines(const Polygons& outline, coord_t line_width, coord_t spacing)
{
	std::vector<ExtrusionLine> ret;
	
	
	std::unordered_map<coord_t, std::vector<coord_t>> x_to_ys;
	
	for (ConstPolygonRef poly : outline)
	{
		assert(poly.size() >= 3);
		Point before = poly[poly.size() - 2];
		Point prev = poly.back();
		for (Point p : poly)
		{
			if (p == prev) continue;
			if (p.X != prev.X)
			{
				coord_t dir =  (p.X > prev.X)? 1 : -1;
				coord_t sgn = (prev.X > 0)? 1 : (prev.X < 0)? -1 : 0;
				coord_t pos_dir = (sgn * p.X > sgn * prev.X)? 1 : 0;
				for (coord_t x = ((prev.X - sgn * pos_dir) / spacing + sgn * pos_dir) * spacing; x * dir < dir * p.X; x += dir * spacing)
				{
					if (x == prev.X)
					{
						if ((before.X > x) == (p.X > x)) continue;
					}
					assert (x * dir >= prev.X * dir);
					coord_t y = prev.Y + (x - prev.X) * (p.Y - prev.Y) / (p.X - prev.X);
					if (x_to_ys.count(x) == 0) x_to_ys.emplace(x, std::vector<coord_t>());
					x_to_ys[x].emplace_back(y);
				}
			}
			before = prev;
			prev = p;
		}
	}
	
	for (std::pair<coord_t, std::vector<coord_t>> x_and_ys : x_to_ys)
	{
		coord_t x = x_and_ys.first;
		std::vector<coord_t>& ys = x_and_ys.second;
		std::sort(ys.begin(), ys.end());
// 		assert(ys.size() % 2 == 0);
		bool start = true;
		for (coord_t y : ys)
		{
			if (start) ret.emplace_back(0);
			start = ! start;
			ret.back().junctions.emplace_back(Point(x, y), line_width, 0);
		}
	}
	return ret;
}

void GcodeWriter::setTranslation(Point p)
{
    this->translation = build_plate_middle + p - extruder_offset[current_extruder];
}

void GcodeWriter::setNominalSpeed(float print_speed)
{
	this->print_speed = print_speed;
    this->flow = print_speed * INT2MM(layer_thickness) * 0.4;
}

void GcodeWriter::print(const std::vector<std::list<ExtrusionLine>> polygons_per_index, const std::vector<std::list<ExtrusionLine>> polylines_per_index, bool ordered, bool startup_and_reduction)
{
    if (ordered)
    {
        printOrdered(polygons_per_index, polylines_per_index, startup_and_reduction);
    }
    else
    {
        printUnordered(polygons_per_index, polylines_per_index, startup_and_reduction);
    }
}

void GcodeWriter::printOrdered(const std::vector<std::list<ExtrusionLine>>& polygons_per_index, const std::vector<std::list<ExtrusionLine>>& polylines_per_index, bool startup_and_reduction)
{
    if (startup_and_reduction)
    {
        logError("not implemented yet\n");
        std::exit(-1);
    }

    std::vector<std::vector<ExtrusionLine>> polygons_per_index_vector;
    polygons_per_index_vector.resize(polygons_per_index.size());
    for (coord_t inset_idx = 0; inset_idx < polygons_per_index.size(); inset_idx++)
        polygons_per_index_vector[inset_idx].insert(polygons_per_index_vector[inset_idx].end(), polygons_per_index[inset_idx].begin(), polygons_per_index[inset_idx].end());

    std::vector<std::vector<ExtrusionLine>> polylines_per_index_vector;
    polylines_per_index_vector.resize(polylines_per_index.size());
    for (coord_t inset_idx = 0; inset_idx < polylines_per_index.size(); inset_idx++)
        polylines_per_index_vector[inset_idx].insert(polylines_per_index_vector[inset_idx].end(), polylines_per_index[inset_idx].begin(), polylines_per_index[inset_idx].end());

    for (int inset_idx = 0; inset_idx < std::max(polygons_per_index_vector.size(), polylines_per_index_vector.size()); inset_idx++)
    {
        if (inset_idx < polylines_per_index_vector.size())
        {
            printLinesByOptimizer(polylines_per_index_vector[inset_idx]);
        }
        if (inset_idx < polygons_per_index_vector.size())
        {
            PathOrderOptimizer order_optimizer(cur_pos);
            Polygons recreated;
            for (ExtrusionLine& polygon : polygons_per_index_vector[inset_idx])
            {
                PolygonRef recreated_poly = recreated.newPoly();
                for (ExtrusionJunction& j : polygon.junctions)
                {
                    recreated_poly.add(j.p);
                }
            }
            order_optimizer.addPolygons(recreated);
            order_optimizer.optimize();
            for (int poly_idx : order_optimizer.polyOrder)
            {
                ExtrusionLine& polygon = polygons_per_index_vector[inset_idx][poly_idx];
                int start_idx = order_optimizer.polyStart[poly_idx];
                assert(start_idx < polygon.junctions.size());
                auto first = polygon.junctions.begin();
                std::advance(first, start_idx);
                move(first->p);
                if (startup_and_reduction) extrude(getExtrusionFilamentMmPerMmMove(first->w) / 2 * first->w / 2 * M_PI);
                auto prev = first;
                
                auto second = first; second++;
                for (auto junction_it = second; ; ++junction_it)
                {
                    if (junction_it == polygon.junctions.end()) junction_it = polygon.junctions.begin();
                    ExtrusionJunction& junction = *junction_it;
                    print(*prev, junction);
                    prev = junction_it;
                    if (junction_it == first) break;
                }
            }
        }
    }
}

void GcodeWriter::printLinesByOptimizer(const std::vector<ExtrusionLine>& lines)
{
	LineOrderOptimizer order_optimizer(cur_pos);
    Polygons recreated;
    for (const ExtrusionLine& polyline : lines)
    {
        PolygonRef recreated_poly = recreated.newPoly();
        recreated_poly.add(polyline.junctions.front().p);
        recreated_poly.add(polyline.junctions.back().p);
    }
    order_optimizer.addPolygons(recreated);
    order_optimizer.optimize();
    for (int poly_idx : order_optimizer.polyOrder)
    {
        const ExtrusionLine& polyline = lines[poly_idx];
        int start_idx = order_optimizer.polyStart[poly_idx];
        assert(start_idx < polyline.junctions.size());
        if (start_idx == 0)
        {
            auto last = polyline.junctions.begin();
            move(last->p);
//             if (startup_and_reduction) extrude(getExtrusionFilamentMmPerMmMove(last->w) / 2 * last->w / 2 * M_PI);
            for (auto junction_it = ++polyline.junctions.begin(); junction_it != polyline.junctions.end(); ++junction_it)
            {
                const ExtrusionJunction& junction = *junction_it;
                print(*last, junction);
                last = junction_it;
            }
        }
        else
        {
            auto last = polyline.junctions.rbegin();
            move(last->p);
//             if (startup_and_reduction) extrude(getExtrusionFilamentMmPerMmMove(last->w) / 2 * last->w / 2 * M_PI);
            for (auto junction_it = ++polyline.junctions.rbegin(); junction_it != polyline.junctions.rend(); ++junction_it)
            {
                const ExtrusionJunction& junction = *junction_it;
                print(*last, junction);
                last = junction_it;
            }
        }
    }
}

void GcodeWriter::printUnordered(const std::vector<std::list<ExtrusionLine>>& polygons_per_index, const std::vector<std::list<ExtrusionLine>>& polylines_per_index, bool startup_and_reduction)
{
    std::vector<Path> paths;
    Polygons recreated;
    for (bool is_closed : {true, false})
    {
        auto polys_per_index = is_closed? polygons_per_index : polylines_per_index;
        for (auto polys : polys_per_index)
            for (auto poly : polys)
            {
                paths.emplace_back(is_closed);
                Path& path = paths.back();
                path.junctions.insert(path.junctions.begin(), poly.junctions.begin(), poly.junctions.end());

                recreated.emplace_back();
                PolygonRef recreated_poly = recreated.back();
                for (auto j : poly.junctions)
                    recreated_poly.add(j.p);
            }
    }
    OrderOptimizer order_optimizer(cur_pos);
    for (coord_t path_idx = 0; path_idx < paths.size(); path_idx++)
    {
        order_optimizer.addPoly(recreated[path_idx], paths[path_idx].is_closed);
    }
    order_optimizer.optimize();


    for (int poly_idx : order_optimizer.polyOrder)
    {
        Path& poly = paths[poly_idx];
        int start_idx = order_optimizer.polyStart[poly_idx];
        assert(start_idx < poly.junctions.size());
        if (poly.is_closed)
        {
            printPolygon(poly, start_idx);
        }
        else
        {
            printPolyline(poly, start_idx);
        }
    }
}

void GcodeWriter::printPolygon(GcodeWriter::Path& polygon, int start_idx)
{
    Path polyline(false);
    polyline.junctions.insert(polyline.junctions.end(), polygon.junctions.begin() + start_idx, polygon.junctions.end());
    polyline.junctions.insert(polyline.junctions.end(), polygon.junctions.begin(), polygon.junctions.begin() + (start_idx + 1) % polygon.junctions.size());
    
//     reduce(polyline, polyline.junctions.size() - 1, polyline.junctions.back().w, 0);
    if (polyline.junctions.empty()) return;

    printPolyline(polyline, 0);
}

void GcodeWriter::reduce(Path& polyline, size_t start_point_idx, coord_t initial_width, coord_t traveled_dist)
{
    if (start_point_idx == 0) polyline.junctions.clear();

    ExtrusionJunction& start_junction = polyline.junctions[start_point_idx];
    ExtrusionJunction& end_junction = polyline.junctions[start_point_idx - 1];
    Point a = start_junction.p;
    Point b = end_junction.p;
    Point ab = b - a;
    coord_t length = vSize(ab);
    
    
    coord_t total_reduction_length = initial_width + start_junction.w / 2;
    
    if (traveled_dist + length > total_reduction_length)
    {
        coord_t reduction_left = total_reduction_length - traveled_dist;
        Point mid = a + ab * std::max(static_cast<coord_t>(0), std::min(length, reduction_left)) / length;
        coord_t mid_w = start_junction.w + (end_junction.w - start_junction.w) * reduction_left / length;
        polyline.junctions.insert(polyline.junctions.begin() + start_point_idx, ExtrusionJunction(mid, mid_w, start_junction.perimeter_index));
    }
    else
    {
        reduce(polyline, start_point_idx - 1, initial_width, traveled_dist + length);
    }

    if (traveled_dist + length > total_reduction_length / 2 && traveled_dist < total_reduction_length / 2)
    {
        coord_t reduction_left = total_reduction_length / 2 - traveled_dist;
        Point mid = a + ab * std::max(static_cast<coord_t>(0), std::min(length, reduction_left)) / length;
        coord_t mid_w = 1;
        polyline.junctions.insert(polyline.junctions.begin() + start_point_idx, ExtrusionJunction(mid, mid_w, start_junction.perimeter_index));
    }

    polyline.junctions[start_point_idx].w = std::max(coord_t(1), traveled_dist - total_reduction_length / 2);
}

void GcodeWriter::printPolyline(GcodeWriter::Path& poly, int start_idx)
{
    {
        coord_t total_length = 0;
        Point prev = poly.junctions.front().p;
        for (size_t j_idx = 1; j_idx < poly.junctions.size(); j_idx++)
        {
            Point here = poly.junctions[j_idx].p;
            total_length += vSize(here - prev);
            prev = here;
        }
        if (total_length < nozzle_size)
        { // extrude a whole bead
            double factor = nozzle_size * nozzle_size / 4 * M_PI / double(total_length * nozzle_size);
            factor = std::max(1.0, factor);
            for (ExtrusionJunction& j : poly.junctions)
                j.w *= factor;
        }
    }
    
    ExtrusionJunction* prev = &poly.junctions[start_idx];
    move(prev->p);
//     extrude(getExtrusionFilamentMmPerMmMove(prev->w) / 2 * prev->w / 2 * M_PI);

    for (size_t pos = 1; pos < poly.junctions.size(); pos++)
    {
        size_t idx = (start_idx == 0)? pos : poly.junctions.size() - 1 - pos;
        ExtrusionJunction& here = poly.junctions[idx];
        print(*prev, here);
        prev = &here;
    }
}

void GcodeWriter::switchExtruder(int extruder_nr)
{
	
    this->translation = translation + extruder_offset[current_extruder];
	
	int old_extruder = current_extruder;
	file << "G92 E0\n";
	file << "T" << extruder_nr << '\n'; current_extruder = extruder_nr;
	
    this->translation = translation - extruder_offset[current_extruder];
	
	file << "G92 E0\n"; last_E = 0;
	file << "M109 S210\n";
	file << "M104 T" << old_extruder << " S0\n";
	file << "M106 S255\n";
	file << "M104 S205\n";
	file << "G1 F1500 E-6.5\n";
	file << "G1 F600 Z2.324\n";
	file << "G0 F" << 60.0 * travel_speed << " X9 Y6 Z2.324\n";
	file << "G0 F" << 60.0 * travel_speed << " X9 Y6 Z4\n";
	file << "G280\n";
	file << "G0 Z" << INT2MM(cur_z) << '\n';
	
}

void GcodeWriter::retract()
{
    file << "G92 E0\n"; last_E = 0;
    file << "G1 F1500 E-" << retraction_distance << "\n";
	
	is_retracted = true;
}

void GcodeWriter::move(Point p)
{
    file << "\n";
    p += translation;
	total_naive_print_time += vSizeMM(cur_pos - p) / travel_speed;
    switch(type)
    {
        case type_UM3:
        default:
			file << std::setprecision(3);
            file << "G0 F" << 60.0 * travel_speed << " X" << INT2MM(p.X) << " Y" << INT2MM(p.Y) << "\n";
            break;
    }
    cur_pos = p;
}

void GcodeWriter::print(ExtrusionJunction from, ExtrusionJunction to)
{
    if (is_retracted)
    {
        file << "G0 F1500 E0 ; unretract\n";
        is_retracted = false;
    }
    from.p += translation;
    to.p += translation;

    assert(from.p == cur_pos);
    bool discretize = std::abs(to.w - from.w) > 10;

    if (from.p == to.p)
    {
        return;
    }
    
    if (!discretize)
    {
        printSingleExtrusionMove(from, to);
    }
    else
    {
        Point vec = to.p - from.p;
        coord_t length = vSize(vec);
        coord_t segment_count = (length + discretization_size / 2) / discretization_size; // round to nearest
        ExtrusionJunction last = from;
        for (coord_t segment_idx = 0; segment_idx < segment_count; segment_idx++)
        {
            ExtrusionJunction here(from.p + vec * (segment_idx + 1) / segment_count, from.w + (to.w - from.w) * (segment_idx + 1) / segment_count, last.perimeter_index);
            printSingleExtrusionMove(last, here);
            last = here;
        }
    }
    
    cur_pos = to.p;
}

void GcodeWriter::printSingleExtrusionMove(ExtrusionJunction& from, ExtrusionJunction& to)
{
    coord_t w = (from.w + to.w) / 2;
    double print_speed = this->print_speed;
	double slippage_compensation_factor = 1.0;
	if (equalize_flow)
	{
		double back_pressure = INT2MM(w - 400) / 0.4 / INT2MM(layer_thickness);
		print_speed = (flow - gamma * back_pressure) / INT2MM(layer_thickness) / INT2MM(w);
		print_speed = std::max(1.0, print_speed);
// 		slippage_compensation_factor = 1.0 + back_pressure * 0.15 * (5.0/100.0); // twice the normal width increases feader wheel speed by 5% at a layer height of 0.15mm
	}
// 	float der = INT2MM(to.w - from.w) / vSizeMM(to.p - from.p) / 0.4;
// 	print_speed -= der * 10;
	total_naive_print_time += vSizeMM(to.p - from.p) / print_speed;
    switch(type)
    {
        case type_UM3:
        default:
            last_E += INT2MM2(ExtrusionSegment(from, to, false).getArea(true)) * INT2MM(layer_thickness) * getExtrusionFilamentMmPerCubicMm() * slippage_compensation_factor;
//             last_E += getExtrusionFilamentMmPerMmMove(w) * INT2MM(vSize(to.p - from.p));
			file << std::setprecision(3);
            file << "G1 F" << 60.0 * print_speed << " X" << INT2MM(to.p.X) << " Y" << INT2MM(to.p.Y);
			file << std::setprecision(5);
			file << " E" << last_E << "\n";
            break;
    }
}

void GcodeWriter::extrude(float amount)
{
    switch(type)
    {
        case type_UM3:
        default:
            last_E += amount;
			file << std::setprecision(5);
            file << "G1 E" << last_E << "\n";
            break;
    }
}

float GcodeWriter::getExtrusionFilamentMmPerMmMove(coord_t width) const
{
    float volume_per_mm_move = INT2MM(width) * INT2MM(layer_thickness);
    // v / m / (v / f) = f / m
    return volume_per_mm_move * getExtrusionFilamentMmPerCubicMm();
}


float GcodeWriter::getExtrusionFilamentMmPerCubicMm() const
{
    float filament_radius = 0.5 * filament_diameter;
    float volume_per_mm_filament = M_PI * filament_radius * filament_radius;
	return 1.0 / volume_per_mm_filament * extrusion_multiplier;
}

} // namespace visualizer
