/*
 * cmrMathDef.hpp
 *
 * Description: This file provide variable and math function definition,
 *
 * Author: Feijian.Ni
 * Data: 2020.2.13
 */

#ifndef CMRMATHDEF_HPP_
#define CMRMATHDEF_HPP_
#include "cmrMatrix.hpp"

namespace cmr {

//! robot control cycle time
const double g_controlCycleTime = 0.01;

//! define pi
const double g_cmrPI = 3.1415926;

//! unit convertion function
#define _DegToRad(deg) (deg * M_PI / 180.0);
#define _RadToDeg(rad) (rad * 180.0 / M_PI);

} // namespace cmr
#endif