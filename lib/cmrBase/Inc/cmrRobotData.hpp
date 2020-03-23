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

//---------------------------Definition-----------------------------

//! robot joint type define
enum cmrJointType {
  CMR_JOINT_FIXED = 0,
  CMR_JOINT_CONTINUOUS,
  CMR_JOINT_REVOLUTE,
  CMR_JOINT_PRISMATIC,
  CMR_JOINT_FLOATING,
  CMR_JOINT_PLANAR,

  CMR_JOINTTYPE_NUM = CMR_JOINT_PLANAR - CMR_JOINT_FIXED + 1
};

//! robot joint type name define
const std::string g_cmrJointTypeName[6] = {
    "fixed", "continuous", "revolute", "prismatic", "floating", "planar"};

//! robot base link name
const std::string g_baseLinkName = "Base";

//! robot target control point link name
//! We take the target control point as a "link" without size and mass
const std::string g_tcpLinkName = "TcpLink";

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
  void printInfo();
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
  cmrJointType m_jointType;

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
  void printInfo();
};

//! robot control point  data
struct cmrTcpData {
  std::string m_parentLink;
  cmrVector3d m_posInParent;
  cmrMatrix3d m_rotInParent;
};

//! robot defined data
struct cmrRobotData {
  cmrRobotData() : m_robotDoFs(0) {}

  //! init robot data
  inline void init() {
    m_linksData.clear();
    m_jointsData.clear();
    m_robotDoFs = 0;
    m_gravity << 0, 0, -9.8;
  }

  // robot name
  std::string m_robotName;

  // robot dofs
  unsigned int m_robotDoFs;

  // gravity
  cmrVector3d m_gravity;

  // link data
  std::vector<cmrLinkData> m_linksData;

  // joint data
  std::vector<cmrJointData> m_jointsData;

  // control point data
  cmrTcpData m_tcpData;

  // print link info
  void printInfo();

  // get joint data with parent link name
  const cmrJointData *
  getChildJointData(const std::string &parentLinkName) const;

  // get joint child link data
  const cmrLinkData *getChildLinkData(const cmrJointData *jointDataPtr) const;
};

} // namespace cmr

#endif