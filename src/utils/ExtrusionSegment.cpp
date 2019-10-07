/** Copyright (C) 2019 Ultimaker */
#include "ExtrusionSegment.h"

#include "utils/logoutput.h"

namespace visualizer
{
    
Polygons ExtrusionSegment::toPolygons(bool reduced) const
{
    Polygons ret;
    Point vec = to.p - from.p;
    coord_t vec_length = vSize( vec );

    if (vec_length <= 0)
    {
        return ret;
    }

    PolygonRef poly = ret.newPoly();
    float delta_r = 0.5f * std::abs(from.w - to.w);
    float vec_length_fixed = std::max(delta_r, static_cast<float>(vec_length));
    float alpha = std::acos(delta_r / vec_length_fixed);
    if (to.w > from.w)
    {
        alpha = M_PI - alpha;
    }
    assert(alpha > - M_PI - 0.0001);
    assert(alpha < M_PI + 0.0001);
    
    float dir = std::atan(vec.Y / static_cast<float>(vec.X));
    if (vec.X < 0)
    {
        dir += M_PI;
    }
    
    {
        poly.emplace_back(from.p + Point(from.w / 2 * cos(alpha + dir), from.w / 2 * sin(alpha + dir)));
        float start_a = 2 * M_PI; while (start_a > alpha + dir) start_a -= a_step;
        start_a += a_step;
        float end_a = -2 * M_PI; while (end_a < 2 * M_PI - alpha + dir) end_a += a_step;
        for (float a = start_a; a <= end_a; a += a_step)
        {
            poly.emplace_back(from.p + Point(from.w / 2 * cos(a), from.w / 2 * sin(a)));
        }
        poly.emplace_back(from.p + Point(from.w / 2 * cos(2 * M_PI - alpha + dir), from.w / 2 * sin(2 * M_PI - alpha + dir)));
    }
    {
        poly.emplace_back(to.p + Point(to.w / 2 * cos(2 * M_PI - alpha + dir), to.w / 2 * sin(2 * M_PI - alpha + dir)));
        float start_a = 2 * M_PI; while (start_a > alpha + dir) start_a -= a_step;
        if (reduced)
        {
            start_a += a_step;
        }
        float end_a = -2 * M_PI; while (end_a < 2 * M_PI - alpha + dir) end_a += a_step;
        if (reduced)
        {
            end_a -= a_step;
        }
        else
        {
            end_a -= 2 * M_PI;
        }
        if (reduced)
        {
            for (float a = end_a; a >= start_a; a -= a_step)
            {
                poly.emplace_back(to.p + Point(to.w / 2 * cos(a), to.w / 2 * sin(a)));
            }
        }
        else
        {
            for (float a = end_a; a <= start_a; a += a_step)
            {
                poly.emplace_back(to.p + Point(to.w / 2 * cos(a), to.w / 2 * sin(a)));
            }
        }
        poly.emplace_back(to.p + Point(to.w / 2 * cos(alpha + dir), to.w / 2 * sin(alpha + dir)));
    }
    
    for (Point p : poly)
    {
        assert(p.X < 0x3FFFFFFFFFFFFFFFLL);
        assert(p.Y < 0x3FFFFFFFFFFFFFFFLL);
    }

    return ret;
}

double ExtrusionSegment::getArea(bool reduced) const
{
	coord_t r = from.w / 2;
	coord_t s = to.w / 2;
	coord_t d2 = vSize2(to.p - from.p);
	coord_t d = sqrt(d2);
	coord_t dr = s - r;
	coord_t l2 = d2 - dr * dr;
	coord_t l = sqrt(l2);
	coord_t trapezoid_area = l * (r + s) / 2;
	double a = asin(sqrt(INT2MM2(l2) / INT2MM2(d2)));
	double b = M_PI - a;
	if (to.w > from.w)
		std::swap(a, b);
	coord_t r2 = r * r;
	coord_t s2 = s * s;
	coord_t start_area = .5 * b * r2;
	coord_t end_area = reduced? - .5 * b * s2 : .5 * a * s2;
	
	coord_t total_area = (trapezoid_area + end_area + start_area) * 2;
	
// 	assert(double(std::abs(total_area - toPolygons(reduced).area())) / double(total_area) < 0.01);
	return total_area;
}

}//namespace cura
