
/*
 * cmrURDFParser.cpp
 *
 * Author: Feijian.Ni
 * Date: 2020.2.20
 */

#include "cmrURDFParser.hpp"
#include <fstream>
#include <sstream>
using namespace tinyxml2;
using std::string;
namespace cmr {

//---------------------------Class Definition------------------------:w

cmrURDFParser::cmrURDFParser() {}

cmrURDFParser::~cmrURDFParser() {}

//! parse URDF file
//! Inpute: string of URDF file name
//! Output: cmrRobotData Struct
cmrErrorType cmrURDFParser::parseURDF(const string &URDFFileName,
                                      cmrRobotData *robotData) {

  //! clear robot data
  robotData->m_linksData.clear();
  robotData->m_jointsData.clear();

  // load URDF file
  XMLDocument URDFFile;
  XMLError errorId = URDFFile.LoadFile(URDFFileName.c_str());
  if (errorId != XML_SUCCESS) {
    string msg = "Could not open URDF file " + URDFFileName;
    throw cmrException(msg);
  };
  // get robotroot element
  XMLElement *robotRootElement = URDFFile.RootElement();

  // get all the links data iteratively
  cmrLinkData linkData;
  XMLElement *curLinkElement;
  bool baseLinkDefined = false;
  while (curLinkElement = robotRootElement->FirstChildElement("link")) {
    parseLinkData(curLinkElement, linkData);
    robotData->m_linksData.push_back(linkData);
    robotRootElement->DeleteChild(curLinkElement);
    if (g_baseLinkName == linkData.m_linkName) {
      baseLinkDefined = true;
    }
  }
  if (!baseLinkDefined) {
    string msg = "No Base link is defined";
    throw cmrException(msg);
  }

  // get all the joints data iteratively
  cmrJointData jointData;
  XMLElement *curJointElement;
  while (curJointElement = robotRootElement->FirstChildElement("joint")) {
    parseJointData(curJointElement, jointData);
    robotData->m_jointsData.push_back(jointData);
    robotData->m_robotDoFs += jointData.m_jointDoFs;
    robotRootElement->DeleteChild(curJointElement);
  }

  return CMR_SUCCESS;
}

//! parse link Data
cmrErrorType cmrURDFParser::parseLinkData(const XMLElement *curLinkElement,
                                          cmrLinkData &linkData) {
  // get attribute of link name
  string name = curLinkElement->FirstAttribute()->Value();
  if (name.empty()) {
    string msg = "No link name is defined in URDF file";
    throw cmrException(msg);
  }
  linkData.m_linkName = name;

  // get geometry file and origin
  const XMLElement *visualElement = curLinkElement->FirstChildElement("visual");
  _CHECK_NULLPOINTER(visualElement, "No visual element in URDF file");

  // geometry file
  const XMLElement *meshElement =
      visualElement->FirstChildElement("geometry")->FirstChildElement();
  linkData.m_geometryFilePath = meshElement->FirstAttribute()->Value();

  // geometry origin
  const XMLElement *geometryOriginElement =
      visualElement->FirstChildElement("origin");
  std::istringstream geometry_xyz(
      geometryOriginElement->FindAttribute("xyz")->Value());
  geometry_xyz >> linkData.m_geometryOrigin.m_pos[0] >>
      linkData.m_geometryOrigin.m_pos[1] >> linkData.m_geometryOrigin.m_pos[2];
  std::istringstream geometry_rpy(
      geometryOriginElement->FindAttribute("rpy")->Value());
  double roll, pitch, yaw;
  geometry_rpy >> roll >> pitch >> yaw;
  linkData.m_geometryOrigin.setRotWihtRPY(roll, pitch, yaw);

  //! get inertial parameters
  const XMLElement *inertialElement =
      curLinkElement->FirstChildElement("inertial");
  _CHECK_NULLPOINTER(inertialElement, "No inertial element in URDF file");

  // mass center origin
  const XMLElement *massOriginElement =
      inertialElement->FirstChildElement("origin");
  std::istringstream mass_xyz(massOriginElement->FindAttribute("xyz")->Value());
  std::istringstream mass_rpy(massOriginElement->FindAttribute("rpy")->Value());
  mass_xyz >> linkData.m_massCenterOrigin.m_pos[0] >>
      linkData.m_massCenterOrigin.m_pos[1] >>
      linkData.m_massCenterOrigin.m_pos[2];
  mass_rpy >> roll >> pitch >> yaw;
  linkData.m_massCenterOrigin.setRotWihtRPY(roll, pitch, yaw);

  // mass
  const XMLElement *massElement = inertialElement->FirstChildElement("mass");
  std::istringstream mass(massElement->FirstAttribute()->Value());
  mass >> linkData.m_linkMass;

  // inertia
  const XMLElement *inertiaElement =
      inertialElement->FirstChildElement("inertia");
  double Ixx = std::stod(inertiaElement->FindAttribute("ixx")->Value());
  double Iyy = std::stod(inertiaElement->FindAttribute("iyy")->Value());
  double Izz = std::stod(inertiaElement->FindAttribute("izz")->Value());
  double Ixy = std::stod(inertiaElement->FindAttribute("ixy")->Value());
  double Ixz = std::stod(inertiaElement->FindAttribute("ixz")->Value());
  double Iyz = std::stod(inertiaElement->FindAttribute("iyz")->Value());
  linkData.m_inertia << Ixx, Ixy, Ixz, Ixy, Iyy, Iyz, Ixz, Iyz, Izz;

  return CMR_SUCCESS;
}
//! parse joint Data
cmrErrorType cmrURDFParser::parseJointData(const XMLElement *curJointElement,
                                           cmrJointData &jointData) {
  // get attribute of joint name
  string name = curJointElement->FindAttribute("name")->Value();
  if (name.empty()) {
    string msg = "No joint name or type is defined in URDF file";
    throw cmrException(msg);
  }
  jointData.m_jointName = name;

  // get joint type
  string type = curJointElement->FindAttribute("type")->Value();
  bool findJointType = false;
  for (int jointType = 0; jointType < CMR_JOINTTYPE_NUM; jointType++) {
    if (g_cmrJointTypeName[jointType] == type) {
      switch (static_cast<cmrJointType>(jointType)) {
      case CMR_JOINT_FIXED:
        jointData.m_jointDoFs = 0;
        break;
      case CMR_JOINT_CONTINUOUS:
      case CMR_JOINT_REVOLUTE:
      case CMR_JOINT_PRISMATIC:
        jointData.m_jointDoFs = 1;
        break;
      case CMR_JOINT_PLANAR:
        jointData.m_jointDoFs = 2;
        break;
      case CMR_JOINT_FLOATING:
        jointData.m_jointDoFs = 6;
        break;
      default:
        break;
      }

      findJointType = true;
      jointData.m_jointType = static_cast<cmrJointType>(jointType);
      break;
    }
  }
  if (!findJointType) {
    string msg = "Joint type of " + type + " is not supported now";
    throw cmrException(msg);
  }

  // joint axis
  const XMLElement *axisElement = curJointElement->FirstChildElement("axis");
  if (axisElement) {
    std::istringstream xyz(axisElement->FirstAttribute()->Value());
    xyz >> jointData.m_jointAxis[0] >> jointData.m_jointAxis[1] >>
        jointData.m_jointAxis[2];
  } else {
    string msg = "No joint axis is defined!";
    throw cmrException(msg);
  }

  // joint parent and child
  const XMLElement *parentElement =
      curJointElement->FirstChildElement("parent");
  const XMLElement *childElement = curJointElement->FirstChildElement("child");
  if (parentElement && childElement) {
    jointData.m_parentLink = parentElement->FirstAttribute()->Value();
    jointData.m_childLink = childElement->FirstAttribute()->Value();
  } else {
    string msg = "No parent or child link is defined!";
    throw cmrException(msg);
  }

  // joint origin
  const XMLElement *originElement =
      curJointElement->FirstChildElement("origin");
  _CHECK_NULLPOINTER(originElement, "No joint origin is defined!");

  std::istringstream xyz(originElement->FindAttribute("xyz")->Value());
  std::istringstream rpy(originElement->FindAttribute("rpy")->Value());
  xyz >> jointData.m_jointOrigin.m_pos[0] >> jointData.m_jointOrigin.m_pos[1] >>
      jointData.m_jointOrigin.m_pos[2];
  double roll, pitch, yaw;
  rpy >> roll >> pitch >> yaw;
  jointData.m_jointOrigin.setRotWihtRPY(roll, pitch, yaw);

  // joint limit
  const XMLElement *limitElement = curJointElement->FirstChildElement("limit");
  _CHECK_NULLPOINTER(limitElement, "No joint limit is defined!");
  jointData.m_maxEffort =
      std::stod(limitElement->FindAttribute("effort")->Value());
  jointData.m_maxVelocity =
      std::stod(limitElement->FindAttribute("velocity")->Value());
  jointData.m_minPos = std::stod(limitElement->FindAttribute("lower")->Value());
  jointData.m_maxPos = std::stod(limitElement->FindAttribute("upper")->Value());

  // joint dynamics
  const XMLElement *dynamicsElement =
      curJointElement->FirstChildElement("dynamics");

  if (dynamicsElement) {
    jointData.m_damping =
        std::stod(dynamicsElement->FindAttribute("damping")->Value());
    jointData.m_friction =
        std::stod(dynamicsElement->FindAttribute("friction")->Value());
  }

  return CMR_SUCCESS;
}

//! converte rpy string to rot matrix
void cmrURDFParser::rpyStrToRotMat(const std::string rpyStr,
                                   cmrMatrix3d &rotMat) {
  std::istringstream rpy(rpyStr);
  double roll, pitch, yaw;
  rpy >> roll >> pitch >> yaw;
  cmrVector3d eulerAngle(yaw, pitch, roll);

  cmrAngleAxis rollMat(eulerAngle(2), cmrVector3d::UnitX());
  cmrAngleAxis pitchMat(eulerAngle(1), cmrVector3d::UnitY());
  cmrAngleAxis yawMat(eulerAngle(0), cmrVector3d::UnitZ());

  rotMat = yawMat * pitchMat * rollMat;
}

} // namespace cmr