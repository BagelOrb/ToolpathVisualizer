//Copyright (c) 2018 Ultimaker B.V.


#ifndef UTILS_COORD_T_H
#define UTILS_COORD_T_H


//Include Clipper to get the ClipperLib::IntPoint definition, which we reuse as Point definition.
#include <clipper.hpp>

namespace visualizer
{

using coord_t = ClipperLib::cInt;

#define INT2MM(n) (double(n) / 1000.0)
#define INT2MM2(n) (double(n) / 1000000.0)
#define MM2INT(n) (coord_t((n) * 1000 + 0.5 * (((n) > 0) - ((n) < 0))))
#define MM2_2INT(n) (coord_t((n) * 1000000 + 0.5 * (((n) > 0) - ((n) < 0))))
#define MM3_2INT(n) (coord_t((n) * 1000000000 + 0.5 * (((n) > 0) - ((n) < 0))))

#define INT2MICRON(n) ((n) / 1)
#define MICRON2INT(n) ((n) * 1)

} // namespace visualizer


#endif // UTILS_COORD_T_H
