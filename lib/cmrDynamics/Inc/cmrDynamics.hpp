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

#include "rbdl/rbdl.h"

namespace cmr{

class cmrDynamics
{
public:
  cmrDynamics();
  ~cmrDynamics();

  //! creat robot model with urdf file
  bool createRobotModel(const char *urdfFilename,
                        RigidBodyDynamics::Model *model);
};

} // namespace cmr

#endif