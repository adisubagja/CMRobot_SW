/*
 * cmrDynamics.cpp
 * 2020.2.13
 * feijian.ni
 */

#include "cmrDynamics.hpp"

namespace cmr {

cmrDynamics::cmrDynamics(){};

cmrDynamics::~cmrDynamics(){};

//! creat robot model with urdf file
bool cmrDynamics::createRobotModel(const char *urdfFilename,
                                   RigidBodyDynamics::Model *model) {
  model = new RigidBodyDynamics::Model();
  return true;
}

} // namespace cmr