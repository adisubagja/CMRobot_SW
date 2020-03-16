/*
 * cmrRobotData.cpp
 *
 * Author: Feijian.Ni
 * Date: 2020.2.20
 *
 */

#include "cmrRobotData.hpp"

namespace cmr {

//! print link info
void cmrLinkData::printInfo() {
  std::cout << "linkName:" << m_linkName << std::endl;
  std::cout << "linkMass:" << m_linkMass << std::endl;
  std::cout << "COM_xyz:" << m_massCenterOrigin.pos.transpose() << std::endl;
  std::cout << "COM_rot:" << std::endl << m_massCenterOrigin.rot << std::endl;
  std::cout << "inertia:" << std::endl << m_inertia << std::endl;
  std::cout << "geometry_xyz:" << m_geometryOrigin.pos.transpose() << std::endl;
  std::cout << "geometry_rot:" << std::endl
            << m_geometryOrigin.rot << std::endl;
}

//! pirnt joint info
void cmrJointData::printInfo() {
  std::cout << "JointName:" << m_jointName << std::endl;
  std::cout << "JointType:" << m_jointType << std::endl;
  std::cout << "JointAxis:" << m_jointAxis.transpose() << std::endl;
  std::cout << "parentLink" << m_parentLink << std::endl;
  std::cout << "childLink:" << m_childLink << std::endl;
  std::cout << "jointOrigin_xyz:" << m_jointOrigin.pos.transpose() << std::endl;
  std::cout << "jointOrigin_rot:" << std::endl
            << m_jointOrigin.rot << std::endl;
  std::cout << "maxEffort:" << m_maxEffort << std::endl;
  std::cout << "maxVelocity:" << m_maxEffort << std::endl;
  std::cout << "maxPos:" << m_maxPos << std::endl;
  std::cout << "minPos:" << m_minPos << std::endl;
  std::cout << "damping:" << m_damping << std::endl;
  std::cout << "friction:" << m_friction << std::endl;
}

//! print robot info
void cmrRobotData::printInfo() {
  std::cout << "robotName:" << std::setw(12) << m_robotName << std::endl;
  std::cout << "robotDoFs:" << std::setw(12) << m_robotDoFs << std::endl;

  for (unsigned int i = 0; i < m_linksData.size(); i++) {
    m_linksData[i].printInfo();
  }

  for (unsigned int i = 0; i < m_jointsData.size(); i++) {
    m_jointsData[i].printInfo();
  }
}

//! get joint data with parent link name
const cmrJointData *
cmrRobotData::getChildJointData(const std::string &parentLinkName) const {
  const cmrJointData *jointDataPtr = nullptr;
  for (const auto &joint : m_jointsData) {
    if (parentLinkName == joint.m_parentLink) {
      jointDataPtr = &joint;
      break;
    }
  }
  return jointDataPtr;
}

//! get link data with child link name
const cmrLinkData *
cmrRobotData::getChildLinkData(const cmrJointData *jointDataPtr) const {
  const cmrLinkData *linkDataPtr = nullptr;
  for (auto &link : m_linksData) {
    if (jointDataPtr->m_childLink == link.m_linkName) {
      linkDataPtr = &link;
    }
  }

  return linkDataPtr;
}

} // namespace cmr