/*
 * cmrRobotData.hpp
 *
 * Description: This file provide data struct for robot define
 *
 * Author: Feijian.Ni
 * Date: 2020.2.20
 *
 */

#ifndef CMRROBOTDATA_HPP_
#define CMRROBOTDATA_HPP_

#include "baseStdLibs.hpp"
#include "cmrTransform.hpp"

namespace cmr {

//! link defined data
struct cmrLinkData {
  cmrLinkData() : m_linkMass(0) { m_inertia = cmrMatrix3d::Constant(1e-6); }

  //! link name
  std::string m_linkName;

  //! geometry file path
  std::string m_geometryFilePath;

  //! gemetry coordinate origin in link coordiante
  cmrTransform m_geometryOrigin;

  //! mass value[kg]
  double m_linkMass;

  //! mass center coordiante origin in link coordinate
  cmrTransform m_massCenterOrigin;

  //! link inertia in mass center coordinate
  cmrMatrix3d m_inertia;

  //! print link info
  void printInfo() {
    std::cout << "linkName:" << m_linkName << std::endl;
    std::cout << "linkMass:" << m_linkMass << std::endl;
    std::cout << "COM_xyz:" << m_massCenterOrigin.pos.transpose() << std::endl;
    std::cout << "COM_rot:" << std::endl << m_massCenterOrigin.rot << std::endl;
    std::cout << "inertia:" << std::endl << m_inertia << std::endl;
    std::cout << "geometry_xyz:" << m_geometryOrigin.pos.transpose()
              << std::endl;
    std::cout << "geometry_rot:" << std::endl
              << m_geometryOrigin.rot << std::endl;
  }
};

//! joint defined data
struct cmrJointData {
  cmrJointData()
      : m_jointDoFs(0), m_maxEffort(0), m_maxVelocity(0), m_damping(0),
        m_friction(0), m_maxPos(LONG_MAX), m_minPos(LONG_MIN) {
    m_jointAxis = cmrVector3d::Zero();
  }

  //! joint name
  std::string m_jointName;

  //! joint type
  std::string m_jointType;

  //! joint DoFs
  int m_jointDoFs;

  //! joint axis
  cmrVector3d m_jointAxis;

  //! parent link
  std::string m_parentLink;

  //! child link
  std::string m_childLink;

  //! joint origin in parent link coordinate
  cmrTransform m_jointOrigin;

  //! joint maximum effort[N, N.m]
  double m_maxEffort;

  //! joint maximum velocity[m/s, rad/s]
  double m_maxVelocity;

  //! joint maximum position[m, rad]
  double m_maxPos;

  //! joint minimum position[m, rad]
  double m_minPos;

  //! joint damping [N/m/s, N.m/rad/s]
  double m_damping;

  //! joint friction [N, N.m]
  double m_friction;

  //! print joint info
  void printInfo() {
    std::cout << "JointName:" << m_jointName << std::endl;
    std::cout << "JointType:" << m_jointType << std::endl;
    std::cout << "JointAxis:" << m_jointAxis.transpose() << std::endl;
    std::cout << "parentLink" << m_parentLink << std::endl;
    std::cout << "childLink:" << m_childLink << std::endl;
    std::cout << "jointOrigin_xyz:" << m_jointOrigin.pos.transpose()
              << std::endl;
    std::cout << "jointOrigin_rot:" << std::endl
              << m_jointOrigin.rot << std::endl;
    std::cout << "maxEffort:" << m_maxEffort << std::endl;
    std::cout << "maxVelocity:" << m_maxEffort << std::endl;
    std::cout << "maxPos:" << m_maxPos << std::endl;
    std::cout << "minPos:" << m_minPos << std::endl;
    std::cout << "damping:" << m_damping << std::endl;
    std::cout << "friction:" << m_friction << std::endl;
  }
};

//! robot defined data
struct cmrRobotData {
  cmrRobotData() : m_robotDoFs(0) {}

  //! init robot data
  inline void init() {
    m_linksData.clear();
    m_jointsData.clear();
    m_robotDoFs = 0;
  }

  //! robot name
  std::string m_robotName;

  //! robot dofs
  unsigned int m_robotDoFs;

  //! link data
  std::vector<cmrLinkData> m_linksData;

  //! joint data
  std::vector<cmrJointData> m_jointsData;

  //! print link info
  void printInfo() {
    std::cout << "robotName:" << std::setw(12) << m_robotName << std::endl;
    std::cout << "robotDoFs:" << std::setw(12) << m_robotDoFs << std::endl;

    for (unsigned int i = 0; i < m_linksData.size(); i++) {
      m_linksData[i].printInfo();
    }

    for (unsigned int i = 0; i < m_jointsData.size(); i++) {
      m_jointsData[i].printInfo();
    }
  }
};

} // namespace cmr

#endif