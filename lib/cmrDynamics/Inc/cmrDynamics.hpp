/*
 * cmrDynamics.hpp
 *
 * Description: This file provide kinematic and dynamic
 * computation for robot
 *
 * Author: Feijian.Ni
 * Date: 2020.2.13
 */

#ifndef CMRDYNAMICS_HPP
#define CMRDYNAMICS_HPP

#include "cmrException.hpp"
#include "cmrRobotData.hpp"
#include "rbdl/rbdl.h"

namespace cmr{

class cmrDynamics
{
public:
  cmrDynamics();
  ~cmrDynamics();

  //! creat robot model with urdf file
  cmrErrorType createRobotModel(const cmrRobotData &robotData,
                                RigidBodyDynamics::Model *model);
};

} // namespace cmr

#endif