//Copyright (c) 2019 Ultimaker B.V.


#ifndef TEST_GEOMETRY_JIN_H
#define TEST_GEOMETRY_JIN_H

#include <utility>

#include "../utils/IntPoint.h"
#include "../utils/logoutput.h"
#include "../utils/polygon.h"

namespace visualizer
{

Polygons generateJin()
{
    Polygons jin;
    PolygonRef jinn = jin.newPoly();
    float scaler = 14;
    jinn.emplace_back(scaler * 1239.9478,38.566027);
    jinn.emplace_back(jinn.back() + Point(scaler * 9.1429, scaler * 16.947077));
    jinn.emplace_back(jinn.back() + Point(scaler * 7.5947, scaler * 17.699747));
    jinn.emplace_back(jinn.back() + Point(scaler * 6.3727, scaler * 18.178184));
    jinn.emplace_back(jinn.back() + Point(scaler * 5.3771, scaler * 18.497175));
    jinn.emplace_back(jinn.back() + Point(scaler * 4.5414, scaler * 18.71988));
    jinn.emplace_back(jinn.back() + Point(scaler * 3.8216, scaler * 18.88187));
    jinn.emplace_back(jinn.back() + Point(scaler * 3.1853, scaler * 18.99895));
    jinn.emplace_back(jinn.back() + Point(scaler * 2.6114, scaler * 19.0875));
    jinn.emplace_back(jinn.back() + Point(scaler * 2.0825, scaler * 19.15146));
    jinn.emplace_back(jinn.back() + Point(scaler * 1.5864, scaler * 19.19885));
    jinn.emplace_back(jinn.back() + Point(scaler * 1.1125, scaler * 19.23186));
    jinn.emplace_back(jinn.back() + Point(scaler * 0.6515, scaler * 19.25403));
    jinn.emplace_back(jinn.back() + Point(scaler * 0.195, scaler * 19.26316));
    jinn.emplace_back(jinn.back() + Point(scaler * -0.265, scaler * 19.26319));
    jinn.emplace_back(jinn.back() + Point(scaler * -0.7377, scaler * 19.25093));
    jinn.emplace_back(jinn.back() + Point(scaler * -1.2329, scaler * 19.22485));
    jinn.emplace_back(jinn.back() + Point(scaler * -1.7632, scaler * 19.18368));
    jinn.emplace_back(jinn.back() + Point(scaler * -2.345, scaler * 19.12013));
    jinn.emplace_back(jinn.back() + Point(scaler * -3.0011, scaler * 19.0292));
    jinn.emplace_back(jinn.back() + Point(scaler * -3.765, scaler * 18.8919));
    jinn.emplace_back(jinn.back() + Point(scaler * -4.6895, scaler * 18.68222));
    jinn.emplace_back(jinn.back() + Point(scaler * -5.3825, scaler * 17.79596));
    jinn.emplace_back(jinn.back() + Point(scaler * -6.1722, scaler * 17.53803));
    jinn.emplace_back(jinn.back() + Point(scaler * -7.1652, scaler * 17.15515));
    jinn.emplace_back(jinn.back() + Point(scaler * -8.4171, scaler * 16.57458));
    jinn.emplace_back(jinn.back() + Point(scaler * -9.9694, scaler * 15.68454));
    jinn.emplace_back(jinn.back() + Point(scaler * -11.7976, scaler * 14.35399));
    jinn.emplace_back(jinn.back() + Point(scaler * -13.7266, scaler * 12.51708));
    jinn.emplace_back(jinn.back() + Point(scaler * -15.4342, scaler * 10.3397));
    jinn.emplace_back(jinn.back() + Point(scaler * -16.6597, scaler * 8.23787));
    jinn.emplace_back(jinn.back() + Point(scaler * -17.3801, scaler * 6.59921));
    jinn.emplace_back(jinn.back() + Point(scaler * -17.739, scaler * 5.56782));
    jinn.emplace_back(jinn.back() + Point(scaler * -17.8679, scaler * 5.45874));
    jinn.emplace_back(jinn.back() + Point(scaler * -17.7262, scaler * 5.89946));
    jinn.emplace_back(jinn.back() + Point(scaler * -17.5255, scaler * 6.47141));
    jinn.emplace_back(jinn.back() + Point(scaler * -17.2245, scaler * 7.22964));
    jinn.emplace_back(jinn.back() + Point(scaler * -16.7526, scaler * 8.25926));
    jinn.emplace_back(jinn.back() + Point(scaler * -15.9701, scaler * 9.67884));
    jinn.emplace_back(jinn.back() + Point(scaler * -14.6169, scaler * 11.60614));
    jinn.emplace_back(jinn.back() + Point(scaler * -12.3383, scaler * 13.98742));
    jinn.emplace_back(jinn.back() + Point(scaler * -9.0791, scaler * 16.28441));
    jinn.emplace_back(jinn.back() + Point(scaler * -9.09918, scaler * 17.69635));
    jinn.emplace_back(jinn.back() + Point(scaler * -12.44885, scaler * 15.48314));
    jinn.emplace_back(jinn.back() + Point(scaler * -16.39917, scaler * 11.14299));
    jinn.emplace_back(jinn.back() + Point(scaler * -19.11085, scaler * 5.36278));
    jinn.emplace_back(jinn.back() + Point(scaler * -19.87633, scaler * 0.93834));
    jinn.emplace_back(jinn.back() + Point(scaler * -19.88691, scaler * -1.09593));
    jinn.emplace_back(jinn.back() + Point(scaler * -19.88222, scaler * -1.19587));
    jinn.emplace_back(jinn.back() + Point(scaler * -19.1901, scaler * -0.65917));
    jinn.emplace_back(jinn.back() + Point(scaler * -19.19689, scaler * -0.41274));
    jinn.emplace_back(jinn.back() + Point(scaler * -19.19132, scaler * 0.4649));
    jinn.emplace_back(jinn.back() + Point(scaler * -18.98401, scaler * 2.71606));
    jinn.emplace_back(jinn.back() + Point(scaler * -17.95853, scaler * 6.66399));
    jinn.emplace_back(jinn.back() + Point(scaler * -16.17841, scaler * 10.29639));
    jinn.emplace_back(jinn.back() + Point(scaler * -14.84564, scaler * 12.16939));
    jinn.emplace_back(jinn.back() + Point(scaler * -11.88547, scaler * 11.03756));
    jinn.emplace_back(jinn.back() + Point(scaler * -13.9808, scaler * 7.78003));
    jinn.emplace_back(jinn.back() + Point(scaler * -16.16268, scaler * 0.65849));
    jinn.emplace_back(jinn.back() + Point(scaler * -16.1495, scaler * -1.49828));
    jinn.emplace_back(jinn.back() + Point(scaler * -16.08719, scaler * -2.06366));
    jinn.emplace_back(jinn.back() + Point(scaler * -19.09982, scaler * -2.55633));
    jinn.emplace_back(jinn.back() + Point(scaler * -18.99456, scaler * -3.23208));
    jinn.emplace_back(jinn.back() + Point(scaler * -18.70023, scaler * -4.62771));
    jinn.emplace_back(jinn.back() + Point(scaler * -18.14638, scaler * -6.45642));
    jinn.emplace_back(jinn.back() + Point(scaler * -17.27089, scaler * -8.524));
    jinn.emplace_back(jinn.back() + Point(scaler * -16.05868, scaler * -10.63026));
    jinn.emplace_back(jinn.back() + Point(scaler * -14.57678, scaler * -12.58676));
    jinn.emplace_back(jinn.back() + Point(scaler * -12.94746, scaler * -14.26053));
    jinn.emplace_back(jinn.back() + Point(scaler * -11.30097, scaler * -15.59921));
    jinn.emplace_back(jinn.back() + Point(scaler * -9.73812, scaler * -16.62196));
    jinn.emplace_back(jinn.back() + Point(scaler * -8.31284, scaler * -17.38059));
    jinn.emplace_back(jinn.back() + Point(scaler * -7.04349, scaler * -17.93507));
    jinn.emplace_back(jinn.back() + Point(scaler * -5.92543, scaler * -18.33486));
    jinn.emplace_back(jinn.back() + Point(scaler * -4.94596, scaler * -18.62422));
    jinn.emplace_back(jinn.back() + Point(scaler * -4.08683, scaler * -18.83051));
    jinn.emplace_back(jinn.back() + Point(scaler * -3.33188, scaler * -18.97981));
    jinn.emplace_back(jinn.back() + Point(scaler * -2.66502, scaler * -19.08557));
    jinn.emplace_back(jinn.back() + Point(scaler * -2.07293, scaler * -19.15823));
    jinn.emplace_back(jinn.back() + Point(scaler * -1.54473, scaler * -19.20902));
    jinn.emplace_back(jinn.back() + Point(scaler * -1.07073, scaler * -19.24033));
    jinn.emplace_back(jinn.back() + Point(scaler * -0.64338, scaler * -19.26033));
    jinn.emplace_back(jinn.back() + Point(scaler * -0.25609, scaler * -19.26936));
    jinn.emplace_back(jinn.back() + Point(scaler * -0.0549, scaler * -19.49343));
    jinn.emplace_back(jinn.back() + Point(scaler * -0.007, scaler * -19.49325));
    jinn.emplace_back(jinn.back() + Point(scaler * 0.0545, scaler * -19.49273));
    jinn.emplace_back(jinn.back() + Point(scaler * 0.13243, scaler * -19.49189));
    jinn.emplace_back(jinn.back() + Point(scaler * 0.23172, scaler * -19.49222));
    jinn.emplace_back(jinn.back() + Point(scaler * 0.35875, scaler * -19.49007));
    jinn.emplace_back(jinn.back() + Point(scaler * 0.52227, scaler * -19.4857));
    jinn.emplace_back(jinn.back() + Point(scaler * 0.73453, scaler * -19.47827));
    jinn.emplace_back(jinn.back() + Point(scaler * 1.01293, scaler * -19.4663));
    jinn.emplace_back(jinn.back() + Point(scaler * 1.38219, scaler * -19.44216));
    jinn.emplace_back(jinn.back() + Point(scaler * 1.87839, scaler * -19.40211));
    jinn.emplace_back(jinn.back() + Point(scaler * 2.55129, scaler * -19.32262));
    jinn.emplace_back(jinn.back() + Point(scaler * 3.46817, scaler * -19.17877));
    jinn.emplace_back(jinn.back() + Point(scaler * 4.70106, scaler * -18.91284));
    jinn.emplace_back(jinn.back() + Point(scaler * 6.28387, scaler * -18.44515));
    jinn.emplace_back(jinn.back() + Point(scaler * 8.12105, scaler * -17.7107));
    jinn.emplace_back(jinn.back() + Point(scaler * 9.93387, scaler * -16.76088));
    jinn.emplace_back(jinn.back() + Point(scaler * 11.40132, scaler * -15.803776));
    jinn.emplace_back(jinn.back() + Point(scaler * 12.38087, scaler * -15.05277));
    jinn.emplace_back(jinn.back() + Point(scaler * 12.93006, scaler * -14.585829));
    jinn.emplace_back(jinn.back() + Point(scaler * 13.17677, scaler * -14.364368));
    jinn.emplace_back(jinn.back() + Point(scaler * 13.23272, scaler * -14.313574));
    jinn.emplace_back(jinn.back() + Point(scaler * 13.63427, scaler * -13.786472));
    jinn.emplace_back(jinn.back() + Point(scaler * 14.49978, scaler * -12.872874));
    jinn.emplace_back(jinn.back() + Point(scaler * 15.23246, scaler * -11.9984642));
    jinn.emplace_back(jinn.back() + Point(scaler * 15.85469, scaler * -11.1647508));
    jinn.emplace_back(jinn.back() + Point(scaler * 16.38418, scaler * -10.369847));
    jinn.emplace_back(jinn.back() + Point(scaler * 16.84004, scaler * -9.61317));
    jinn.emplace_back(jinn.back() + Point(scaler * 17.2326, scaler * -8.889433));
    jinn.emplace_back(jinn.back() + Point(scaler * 17.57397, scaler * -8.195575));
    jinn.emplace_back(jinn.back() + Point(scaler * 17.87027, scaler * -7.526437));
    jinn.emplace_back(jinn.back() + Point(scaler * 18.12982, scaler * -6.878814));
    jinn.emplace_back(jinn.back() + Point(scaler * 18.35808, scaler * -6.248806));
    jinn.emplace_back(jinn.back() + Point(scaler * 18.55627, scaler * -5.631779));
    jinn.emplace_back(jinn.back() + Point(scaler * 18.72914, scaler * -5.024901));
    jinn.emplace_back(jinn.back() + Point(scaler * 18.88002, scaler * -4.425094));
    jinn.emplace_back(jinn.back() + Point(scaler * 19.01033, scaler * -3.829031));
    jinn.emplace_back(jinn.back() + Point(scaler * 19.1208, scaler * -3.233564));
    jinn.emplace_back(jinn.back() + Point(scaler * 19.21098, scaler * -2.635612));
    jinn.emplace_back(jinn.back() + Point(scaler * 19.28487, scaler * -2.032806));
    jinn.emplace_back(jinn.back() + Point(scaler * 19.3391, scaler * -1.421817));
    jinn.emplace_back(jinn.back() + Point(scaler * 19.37535, scaler * -0.799995));
    jinn.emplace_back(jinn.back() + Point(scaler * 19.39115, scaler * -0.164362));
    jinn.emplace_back(jinn.back() + Point(scaler * 18.87903, scaler * 0.69094));
    jinn.emplace_back(jinn.back() + Point(scaler * 18.81348, scaler * 1.71442));
    jinn.emplace_back(jinn.back() + Point(scaler * 18.7005, scaler * 2.678208));
    jinn.emplace_back(jinn.back() + Point(scaler * 18.5448, scaler * 3.598251));
    jinn.emplace_back(jinn.back() + Point(scaler * 18.351, scaler * 4.488661));
    jinn.emplace_back(jinn.back() + Point(scaler * 18.1148, scaler * 5.360164));
    jinn.emplace_back(jinn.back() + Point(scaler * 17.837, scaler * 6.223788));
    jinn.emplace_back(jinn.back() + Point(scaler * 17.5106, scaler * 7.087582));
    jinn.emplace_back(jinn.back() + Point(scaler * 17.1324, scaler * 7.960867));
    jinn.emplace_back(jinn.back() + Point(scaler * 16.69, scaler * 8.849109));
    jinn.emplace_back(jinn.back() + Point(scaler * 16.1738, scaler * 9.758569));
    jinn.emplace_back(jinn.back() + Point(scaler * 15.5714, scaler * 10.694243));
    jinn.emplace_back(jinn.back() + Point(scaler * 14.864, scaler * 11.656573));
    jinn.emplace_back(jinn.back() + Point(scaler * 14.0327, scaler * 12.6435098));
    jinn.emplace_back(jinn.back() + Point(scaler * 13.0583, scaler * 13.6491273));
    jinn.emplace_back(jinn.back() + Point(scaler * 11.9163, scaler * 14.6551449));
    jinn.emplace_back(jinn.back() + Point(scaler * 10.5909, scaler * 15.637161));
    Point b = jinn.back();
    PolygonRef jinn2 = jin.newPoly();
    jinn2.emplace_back(b + Point(scaler * 226.5281, scaler * 332.873643));
    jinn2.emplace_back(jinn2.back() + Point(scaler * -0.3085, scaler * -19.74316));
    jinn2.emplace_back(jinn2.back() + Point(scaler * -0.928, scaler * -19.72448));
    jinn2.emplace_back(jinn2.back() + Point(scaler * -1.5502, scaler * -19.68528));
    jinn2.emplace_back(jinn2.back() + Point(scaler * -2.174, scaler * -19.6261));
    jinn2.emplace_back(jinn2.back() + Point(scaler * -2.7984, scaler * -19.5473));
    jinn2.emplace_back(jinn2.back() + Point(scaler * -3.4221, scaler * -19.44743));
    jinn2.emplace_back(jinn2.back() + Point(scaler * -4.0439, scaler * -19.32683));
    jinn2.emplace_back(jinn2.back() + Point(scaler * -4.6633, scaler * -19.18716));
    jinn2.emplace_back(jinn2.back() + Point(scaler * -5.2787, scaler * -19.02678));
    jinn2.emplace_back(jinn2.back() + Point(scaler * -5.8897, scaler * -18.84703));
    jinn2.emplace_back(jinn2.back() + Point(scaler * -6.4951, scaler * -18.64754));
    jinn2.emplace_back(jinn2.back() + Point(scaler * -7.094, scaler * -18.42788));
    jinn2.emplace_back(jinn2.back() + Point(scaler * -7.6856, scaler * -18.18877));
    jinn2.emplace_back(jinn2.back() + Point(scaler * -8.2692, scaler * -17.93069));
    jinn2.emplace_back(jinn2.back() + Point(scaler * -8.8448, scaler * -17.65494));
    jinn2.emplace_back(jinn2.back() + Point(scaler * -9.4103, scaler * -17.359085));
    jinn2.emplace_back(jinn2.back() + Point(scaler * -9.9666, scaler * -17.046169));
    jinn2.emplace_back(jinn2.back() + Point(scaler * -10.5117, scaler * -16.71438));
    jinn2.emplace_back(jinn2.back() + Point(scaler * -11.0464, scaler * -16.365756));
    jinn2.emplace_back(jinn2.back() + Point(scaler * -11.5704, scaler * -16.000633));
    jinn2.emplace_back(jinn2.back() + Point(scaler * -12.0827, scaler * -15.618869));
    jinn2.emplace_back(jinn2.back() + Point(scaler * -12.5812, scaler * -15.218402));
    jinn2.emplace_back(jinn2.back() + Point(scaler * -13.0684, scaler * -14.803091));
    jinn2.emplace_back(jinn2.back() + Point(scaler * -13.5419, scaler * -14.371133));
    jinn2.emplace_back(jinn2.back() + Point(scaler * -14.0017, scaler * -13.923166));
    jinn2.emplace_back(jinn2.back() + Point(scaler * -14.4482, scaler * -13.460141));
    jinn2.emplace_back(jinn2.back() + Point(scaler * -14.879, scaler * -12.980882));
    jinn2.emplace_back(jinn2.back() + Point(scaler * -15.2964, scaler * -12.487693));
    jinn2.emplace_back(jinn2.back() + Point(scaler * -15.6978, scaler * -11.97927));
    jinn2.emplace_back(jinn2.back() + Point(scaler * -16.0827, scaler * -11.45616));
    jinn2.emplace_back(jinn2.back() + Point(scaler * -16.4521, scaler * -10.91959));
    jinn2.emplace_back(jinn2.back() + Point(scaler * -16.8036, scaler * -10.36886));
    jinn2.emplace_back(jinn2.back() + Point(scaler * -17.1391, scaler * -9.80589));
    jinn2.emplace_back(jinn2.back() + Point(scaler * -17.4554, scaler * -9.22969));
    jinn2.emplace_back(jinn2.back() + Point(scaler * -17.7537, scaler * -8.64184));
    jinn2.emplace_back(jinn2.back() + Point(scaler * -18.0344, scaler * -8.04332));
    jinn2.emplace_back(jinn2.back() + Point(scaler * -18.2934, scaler * -7.43329));
    jinn2.emplace_back(jinn2.back() + Point(scaler * -18.533, scaler * -6.81386));
    jinn2.emplace_back(jinn2.back() + Point(scaler * -18.7514, scaler * -6.18549));
    jinn2.emplace_back(jinn2.back() + Point(scaler * -18.95, scaler * -5.5497));
    jinn2.emplace_back(jinn2.back() + Point(scaler * -19.1257, scaler * -4.90678));
    jinn2.emplace_back(jinn2.back() + Point(scaler * -19.28151, scaler * -4.25876));
    jinn2.emplace_back(jinn2.back() + Point(scaler * -19.41346, scaler * -3.60604));
    jinn2.emplace_back(jinn2.back() + Point(scaler * -19.5237, scaler * -2.95042));
    jinn2.emplace_back(jinn2.back() + Point(scaler * -19.6116, scaler * -2.29317));
    jinn2.emplace_back(jinn2.back() + Point(scaler * -19.67734, scaler * -1.63568));
    jinn2.emplace_back(jinn2.back() + Point(scaler * -19.72244, scaler * -0.97943));
    jinn2.emplace_back(jinn2.back() + Point(scaler * -19.74392, scaler * -0.32566));
    jinn2.emplace_back(jinn2.back() + Point(scaler * -19.62678, scaler * 0.32769));
    jinn2.emplace_back(jinn2.back() + Point(scaler * -19.60619, scaler * 0.98313));
    jinn2.emplace_back(jinn2.back() + Point(scaler * -19.56078, scaler * 1.63759));
    jinn2.emplace_back(jinn2.back() + Point(scaler * -19.49561, scaler * 2.29035));
    jinn2.emplace_back(jinn2.back() + Point(scaler * -19.40826, scaler * 2.94036));
    jinn2.emplace_back(jinn2.back() + Point(scaler * -19.29899, scaler * 3.5867));
    jinn2.emplace_back(jinn2.back() + Point(scaler * -19.16942, scaler * 4.2288));
    jinn2.emplace_back(jinn2.back() + Point(scaler * -19.01759, scaler * 4.86542));
    jinn2.emplace_back(jinn2.back() + Point(scaler * -18.84523, scaler * 5.49609));
    jinn2.emplace_back(jinn2.back() + Point(scaler * -18.65206, scaler * 6.11996));
    jinn2.emplace_back(jinn2.back() + Point(scaler * -18.43884, scaler * 6.73644));
    jinn2.emplace_back(jinn2.back() + Point(scaler * -18.20489, scaler * 7.34454));
    jinn2.emplace_back(jinn2.back() + Point(scaler * -17.95161, scaler * 7.94392));
    jinn2.emplace_back(jinn2.back() + Point(scaler * -17.67818, scaler * 8.53347));
    jinn2.emplace_back(jinn2.back() + Point(scaler * -17.38692, scaler * 9.11347));
    jinn2.emplace_back(jinn2.back() + Point(scaler * -17.0765, scaler * 9.68249));
    jinn2.emplace_back(jinn2.back() + Point(scaler * -16.74746, scaler * 10.23993));
    jinn2.emplace_back(jinn2.back() + Point(scaler * -16.40073, scaler * 10.7854));
    jinn2.emplace_back(jinn2.back() + Point(scaler * -16.03793, scaler * 11.31921));
    jinn2.emplace_back(jinn2.back() + Point(scaler * -15.65818, scaler * 11.8399));
    jinn2.emplace_back(jinn2.back() + Point(scaler * -15.26093, scaler * 12.34594));
    jinn2.emplace_back(jinn2.back() + Point(scaler * -14.84996, scaler * 12.839362));
    jinn2.emplace_back(jinn2.back() + Point(scaler * -14.42306, scaler * 13.317338));
    jinn2.emplace_back(jinn2.back() + Point(scaler * -13.98106, scaler * 13.779292));
    jinn2.emplace_back(jinn2.back() + Point(scaler * -13.67908, scaler * 14.39264));
    jinn2.emplace_back(jinn2.back() + Point(scaler * -13.19911, scaler * 14.833649));
    jinn2.emplace_back(jinn2.back() + Point(scaler * -12.70602, scaler * 15.258791));
    jinn2.emplace_back(jinn2.back() + Point(scaler * -12.19921, scaler * 15.665999));
    jinn2.emplace_back(jinn2.back() + Point(scaler * -11.68097, scaler * 16.056866));
    jinn2.emplace_back(jinn2.back() + Point(scaler * -11.15091, scaler * 16.429702));
    jinn2.emplace_back(jinn2.back() + Point(scaler * -10.60977, scaler * 16.784211));
    jinn2.emplace_back(jinn2.back() + Point(scaler * -10.05851, scaler * 17.120612));
    jinn2.emplace_back(jinn2.back() + Point(scaler * -9.49714, scaler * 17.437598));
    jinn2.emplace_back(jinn2.back() + Point(scaler * -8.92674, scaler * 17.73564));
    jinn2.emplace_back(jinn2.back() + Point(scaler * -8.34811, scaler * 18.01501));
    jinn2.emplace_back(jinn2.back() + Point(scaler * -7.76171, scaler * 18.27546));
    jinn2.emplace_back(jinn2.back() + Point(scaler * -7.16808, scaler * 18.51676));
    jinn2.emplace_back(jinn2.back() + Point(scaler * -6.56784, scaler * 18.73916));
    jinn2.emplace_back(jinn2.back() + Point(scaler * -5.96063, scaler * 18.93969));
    jinn2.emplace_back(jinn2.back() + Point(scaler * -5.34853, scaler * 19.12277));
    jinn2.emplace_back(jinn2.back() + Point(scaler * -4.73072, scaler * 19.28371));
    jinn2.emplace_back(jinn2.back() + Point(scaler * -4.10892, scaler * 19.42666));
    jinn2.emplace_back(jinn2.back() + Point(scaler * -3.48295, scaler * 19.54825));
    jinn2.emplace_back(jinn2.back() + Point(scaler * -2.85392, scaler * 19.65103));
    jinn2.emplace_back(jinn2.back() + Point(scaler * -2.22193, scaler * 19.73114));
    jinn2.emplace_back(jinn2.back() + Point(scaler * -1.58825, scaler * 19.79283));
    jinn2.emplace_back(jinn2.back() + Point(scaler * -0.95326, scaler * 19.83397));
    jinn2.emplace_back(jinn2.back() + Point(scaler * -0.31775, scaler * 19.85473));
    jinn2.emplace_back(jinn2.back() + Point(scaler * 0.31772, scaler * 19.85399));
    jinn2.emplace_back(jinn2.back() + Point(scaler * 0.95316, scaler * 19.83268));
    jinn2.emplace_back(jinn2.back() + Point(scaler * 1.58822, scaler * 19.79313));
    jinn2.emplace_back(jinn2.back() + Point(scaler * 2.22191, scaler * 19.73142));
    jinn2.emplace_back(jinn2.back() + Point(scaler * 2.85374, scaler * 19.65028));
    jinn2.emplace_back(jinn2.back() + Point(scaler * 3.48286, scaler * 19.54815));
    jinn2.emplace_back(jinn2.back() + Point(scaler * 4.10884, scaler * 19.42662));
    jinn2.emplace_back(jinn2.back() + Point(scaler * 4.73061, scaler * 19.2836));
    jinn2.emplace_back(jinn2.back() + Point(scaler * 5.34854, scaler * 19.12312));
    jinn2.emplace_back(jinn2.back() + Point(scaler * 5.96058, scaler * 18.93975));
    jinn2.emplace_back(jinn2.back() + Point(scaler * 6.56767, scaler * 18.73889));
    jinn2.emplace_back(jinn2.back() + Point(scaler * 7.1678, scaler * 18.5163));
    jinn2.emplace_back(jinn2.back() + Point(scaler * 7.76189, scaler * 18.27608));
    jinn2.emplace_back(jinn2.back() + Point(scaler * 8.34811, scaler * 18.0152));
    jinn2.emplace_back(jinn2.back() + Point(scaler * 8.92748, scaler * 17.73719));
    jinn2.emplace_back(jinn2.back() + Point(scaler * 9.49715, scaler * 17.43766));
    jinn2.emplace_back(jinn2.back() + Point(scaler * 10.05792, scaler * 17.11967));
    jinn2.emplace_back(jinn2.back() + Point(scaler * 10.60986, scaler * 16.78443));
    jinn2.emplace_back(jinn2.back() + Point(scaler * 11.15063, scaler * 16.42938));
    jinn2.emplace_back(jinn2.back() + Point(scaler * 11.68112, scaler * 16.05716));
    jinn2.emplace_back(jinn2.back() + Point(scaler * 12.19924, scaler * 15.66611));
    jinn2.emplace_back(jinn2.back() + Point(scaler * 12.70571, scaler * 15.2585));
    jinn2.emplace_back(jinn2.back() + Point(scaler * 13.19956, scaler * 14.83421));
    jinn2.emplace_back(jinn2.back() + Point(scaler * 13.67974, scaler * 14.39336));
    jinn2.emplace_back(jinn2.back() + Point(scaler * 13.98173, scaler * 13.77992));
    jinn2.emplace_back(jinn2.back() + Point(scaler * 14.42247, scaler * 13.31678));
    jinn2.emplace_back(jinn2.back() + Point(scaler * 14.849, scaler * 12.83855));
    jinn2.emplace_back(jinn2.back() + Point(scaler * 15.26167, scaler * 12.34657));
    jinn2.emplace_back(jinn2.back() + Point(scaler * 15.65801, scaler * 11.83978));
    jinn2.emplace_back(jinn2.back() + Point(scaler * 16.03872, scaler * 11.31976));
    jinn2.emplace_back(jinn2.back() + Point(scaler * 16.40206, scaler * 10.78623));
    jinn2.emplace_back(jinn2.back() + Point(scaler * 16.74809, scaler * 10.24022));
    jinn2.emplace_back(jinn2.back() + Point(scaler * 17.07538, scaler * 9.68177));
    jinn2.emplace_back(jinn2.back() + Point(scaler * 17.38623, scaler * 9.11307));
    jinn2.emplace_back(jinn2.back() + Point(scaler * 17.67847, scaler * 8.53357));
    jinn2.emplace_back(jinn2.back() + Point(scaler * 17.95166, scaler * 7.94391));
    jinn2.emplace_back(jinn2.back() + Point(scaler * 18.20375, scaler * 7.34406));
    jinn2.emplace_back(jinn2.back() + Point(scaler * 18.43843, scaler * 6.73631));
    jinn2.emplace_back(jinn2.back() + Point(scaler * 18.65075, scaler * 6.11957));
    jinn2.emplace_back(jinn2.back() + Point(scaler * 18.84491, scaler * 5.49608));
    jinn2.emplace_back(jinn2.back() + Point(scaler * 19.01848, scaler * 4.86571));
    jinn2.emplace_back(jinn2.back() + Point(scaler * 19.16923, scaler * 4.2288));
    jinn2.emplace_back(jinn2.back() + Point(scaler * 19.29983, scaler * 3.5869));
    jinn2.emplace_back(jinn2.back() + Point(scaler * 19.40858, scaler * 2.94043));
    jinn2.emplace_back(jinn2.back() + Point(scaler * 19.49521, scaler * 2.29032));
    jinn2.emplace_back(jinn2.back() + Point(scaler * 19.56169, scaler * 1.63767));
    jinn2.emplace_back(jinn2.back() + Point(scaler * 19.60463, scaler * 0.98307));
    jinn2.emplace_back(jinn2.back() + Point(scaler * 19.62818, scaler * 0.32774));
    jinn2.emplace_back(jinn2.back() + Point(scaler * 19.74384, scaler * -0.32565));
    jinn2.emplace_back(jinn2.back() + Point(scaler * 19.72194, scaler * -0.9794));
    jinn2.emplace_back(jinn2.back() + Point(scaler * 19.67809, scaler * -1.63574));
    jinn2.emplace_back(jinn2.back() + Point(scaler * 19.61283, scaler * -2.29333));
    jinn2.emplace_back(jinn2.back() + Point(scaler * 19.52323, scaler * -2.9504));
    jinn2.emplace_back(jinn2.back() + Point(scaler * 19.41318, scaler * -3.60601));
    jinn2.emplace_back(jinn2.back() + Point(scaler * 19.28126, scaler * -4.25874));
    jinn2.emplace_back(jinn2.back() + Point(scaler * 19.1263, scaler * -4.90694));
    jinn2.emplace_back(jinn2.back() + Point(scaler * 18.9498, scaler * -5.54968));
    jinn2.emplace_back(jinn2.back() + Point(scaler * 18.7521, scaler * -6.18576));
    jinn2.emplace_back(jinn2.back() + Point(scaler * 18.533, scaler * -6.81391));
    jinn2.emplace_back(jinn2.back() + Point(scaler * 18.2935, scaler * -7.43341));
    jinn2.emplace_back(jinn2.back() + Point(scaler * 18.0336, scaler * -8.04299));
    jinn2.emplace_back(jinn2.back() + Point(scaler * 17.7536, scaler * -8.64184));
    jinn2.emplace_back(jinn2.back() + Point(scaler * 17.4566, scaler * -9.23037));
    jinn2.emplace_back(jinn2.back() + Point(scaler * 17.139, scaler * -9.80592));
    jinn2.emplace_back(jinn2.back() + Point(scaler * 16.8042, scaler * -10.3693));
    jinn2.emplace_back(jinn2.back() + Point(scaler * 16.4525, scaler * -10.92003));
    jinn2.emplace_back(jinn2.back() + Point(scaler * 16.0825, scaler * -11.45611));
    jinn2.emplace_back(jinn2.back() + Point(scaler * 15.6975, scaler * -11.9792));
    jinn2.emplace_back(jinn2.back() + Point(scaler * 15.2959, scaler * -12.4874));
    jinn2.emplace_back(jinn2.back() + Point(scaler * 14.8792, scaler * -12.98108));
    jinn2.emplace_back(jinn2.back() + Point(scaler * 14.448, scaler * -13.46013));
    jinn2.emplace_back(jinn2.back() + Point(scaler * 14.001, scaler * -13.92257));
    jinn2.emplace_back(jinn2.back() + Point(scaler * 13.5418, scaler * -14.37106));
    jinn2.emplace_back(jinn2.back() + Point(scaler * 13.0685, scaler * -14.80336));
    jinn2.emplace_back(jinn2.back() + Point(scaler * 12.5816, scaler * -15.21895));
    jinn2.emplace_back(jinn2.back() + Point(scaler * 12.0824, scaler * -15.61854));
    jinn2.emplace_back(jinn2.back() + Point(scaler * 11.5707, scaler * -16.00132));
    jinn2.emplace_back(jinn2.back() + Point(scaler * 11.0472, scaler * -16.36704));
    jinn2.emplace_back(jinn2.back() + Point(scaler * 10.5115, scaler * -16.71434));
    jinn2.emplace_back(jinn2.back() + Point(scaler * 9.9662, scaler * -17.04581));
    jinn2.emplace_back(jinn2.back() + Point(scaler * 9.4107, scaler * -17.35999));
    jinn2.emplace_back(jinn2.back() + Point(scaler * 8.8442, scaler * -17.65412));
    jinn2.emplace_back(jinn2.back() + Point(scaler * 8.2695, scaler * -17.9316));
    jinn2.emplace_back(jinn2.back() + Point(scaler * 7.6856, scaler * -18.1892));
    jinn2.emplace_back(jinn2.back() + Point(scaler * 7.0938, scaler * -18.42785));
    jinn2.emplace_back(jinn2.back() + Point(scaler * 6.4946, scaler * -18.64634));
    jinn2.emplace_back(jinn2.back() + Point(scaler * 5.8895, scaler * -18.84662));
    jinn2.emplace_back(jinn2.back() + Point(scaler * 5.2786, scaler * -19.02686));
    jinn2.emplace_back(jinn2.back() + Point(scaler * 4.6631, scaler * -19.18707));
    jinn2.emplace_back(jinn2.back() + Point(scaler * 4.0438, scaler * -19.32667));
    jinn2.emplace_back(jinn2.back() + Point(scaler * 3.422, scaler * -19.44756));
    jinn2.emplace_back(jinn2.back() + Point(scaler * 2.7982, scaler * -19.54634));
    jinn2.emplace_back(jinn2.back() + Point(scaler * 2.1739, scaler * -19.6249));
    jinn2.emplace_back(jinn2.back() + Point(scaler * 1.5502, scaler * -19.68535));
    jinn2.emplace_back(jinn2.back() + Point(scaler * 0.9279, scaler * -19.72424));
    jinn2.emplace_back(jinn2.back() + Point(scaler * 0.3085, scaler * -19.74263));

    return jin;
}

} // namespace visualizer
#endif // TEST_GEOMETRY_JIN_H
