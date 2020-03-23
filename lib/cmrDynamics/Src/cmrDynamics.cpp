/*
 * cmrDynamics.cpp
 * 2020.2.13
 * feijian.ni
 */

#include "cmrDynamics.hpp"

using RigidBodyDynamics::Body;
using RigidBodyDynamics::Joint;
using RigidBodyDynamics::Math::Matrix3d;
using RigidBodyDynamics::Math::SpatialTransform;
using RigidBodyDynamics::Math::Vector3d;
using std::string;

namespace cmr {

cmrDynamics::cmrDynamics()
    : m_robotModel{nullptr}, m_robotDoFs{0}, m_tcpLinkId{0} {};

cmrDynamics::~cmrDynamics() { delete m_robotModel; };

//! creat robot model with urdf file
cmrErrorType cmrDynamics::createRobotModel(const cmrRobotData &robotData) {
  // creat robot model
  m_robotModel = new RigidBodyDynamics::Model();
  m_linkNameIdMap.clear();

  // set gravity
  m_robotModel->gravity = robotData.m_gravity;

  // set DoFs
  m_robotDoFs = robotData.m_robotDoFs;

  // add base link first
  unsigned int baseLinkId = 0;
  Body body_base;
  for (const auto &link : robotData.m_linksData) {
    if (g_baseLinkName == link.m_linkName) {
      Vector3d linkCOM = link.m_massCenterOrigin.m_pos;
      Matrix3d linkInertia = link.m_inertia;
      body_base = Body(link.m_linkMass, linkCOM, linkInertia);
    }
  }
  cmrMatrix3d rot = cmrMatrix3d::Identity();
  // fixed joint for base link
  Joint joint_base = Joint(RigidBodyDynamics::JointTypeFixed);
  baseLinkId = m_robotModel->AddBody(
      0, RigidBodyDynamics::Math::Xtrans(Vector3d(0, 0, 0)), joint_base,
      body_base, g_baseLinkName);

  // add following links and joints in sequence
  string parentLinkName = g_baseLinkName;
  unsigned int parentLinkId = baseLinkId;
  const cmrJointData *curAddJointData = nullptr;

  // if current link is connect to another joint
  while (curAddJointData = robotData.getChildJointData(parentLinkName)) {
    // get next link
    const cmrLinkData *curAddLinkData =
        robotData.getChildLinkData(curAddJointData);
    Vector3d linkCOM = curAddLinkData->m_massCenterOrigin.m_pos;
    Matrix3d linkInertia = curAddLinkData->m_inertia;
    Body curAddBody = Body(curAddLinkData->m_linkMass, linkCOM, linkInertia);
    std::string curAddLinkName = curAddLinkData->m_linkName;

    // get next joint
    RigidBodyDynamics::JointType curAddJointType =
        getChildJointType(curAddJointData->m_jointType);
    Vector3d curAddJointAxis = curAddJointData->m_jointAxis;
    Joint curAddJoint = Joint(curAddJointType, curAddJointAxis);

    // set joint transform to parent link
    // Info: Here the joint rot is transposed, to be compatible with RBDL
    // interface
    SpatialTransform curAddJointTrans;
    curAddJointTrans.E = curAddJointData->m_jointOrigin.m_rot.transpose();
    curAddJointTrans.r = curAddJointData->m_jointOrigin.m_pos;
    unsigned int curLinkId =
        m_robotModel->AddBody(parentLinkId, curAddJointTrans, curAddJoint,
                              curAddBody, curAddLinkName);

    // add link name and id to map
    m_linkNameIdMap.insert(std::unordered_map<string, unsigned int>::value_type(
        curAddLinkName, curLinkId));

    // update paretn link name and id
    parentLinkName = curAddLinkName;
    parentLinkId = curLinkId;
  }

  // add target control point to robot model
  unsigned int tcpParentLinkId = getLinkId(robotData.m_tcpData.m_parentLink);
  SpatialTransform tcpTrans;
  tcpTrans.E = robotData.m_tcpData.m_rotInParent;
  tcpTrans.r = robotData.m_tcpData.m_posInParent;
  Vector3d tcpCOM = Vector3d::Zero();
  Matrix3d tcpInertial = Matrix3d::Zero();
  Body tcpLink = Body(0.0, tcpCOM, tcpInertial);

  m_tcpLinkId = m_robotModel->AddBody(tcpParentLinkId, tcpTrans,
                                      Joint(RigidBodyDynamics::JointTypeFixed),
                                      tcpLink, g_tcpLinkName);

  // add tcp link id to map
  m_linkNameIdMap.insert(std::unordered_map<string, unsigned int>::value_type(
      g_tcpLinkName, m_tcpLinkId));

  return CMR_SUCCESS;
}

//! get link id
unsigned int cmrDynamics::getLinkId(std::string linkName) {
  auto mapIt = m_linkNameIdMap.find(linkName);

  if (m_linkNameIdMap.end() == mapIt) {
    throw cmrException("Can no find " + linkName);
  }
  return mapIt->second;
}

//! get rbdl joint type
RigidBodyDynamics::JointType
cmrDynamics::getChildJointType(cmrJointType jointType) {
  RigidBodyDynamics::JointType rbdlJointType =
      RigidBodyDynamics::JointTypeUndefined;

  // get rbdl joint type
  switch (jointType) {
  case CMR_JOINT_FIXED:
    rbdlJointType = RigidBodyDynamics::JointTypeFixed;
    break;
  case CMR_JOINT_CONTINUOUS:
  case CMR_JOINT_REVOLUTE:
    rbdlJointType = RigidBodyDynamics::JointTypeRevolute;
    break;
  case CMR_JOINT_PRISMATIC:
    rbdlJointType = RigidBodyDynamics::JointTypePrismatic;
    break;
  case CMR_JOINT_FLOATING:
    rbdlJointType = RigidBodyDynamics::JointTypeFloatingBase;
    break;
  default:
    throw cmrException("Undefined Joint Type!");
    break;
  }

  return rbdlJointType;
}

//! update robot model kinematic with joint position
//! user should make sure this function is called at the
//! start of each control loop.
cmrErrorType cmrDynamics::updataRobotKinematic(const cmrVectorXd &jointPos) {
  // check input vector size
  if (jointPos.size() != m_robotDoFs) {
    return CMR_SIZE_NOT_COMPATIBLE;
  }

  // check if robot model is created
  if (!m_robotModel) {
    throw cmrException("robot model is not initialized"); //! get tcp transform
    cmrTransform getTcpTransform(const cmrVectorXd &jointPos);
  }

  //! update kinematic
  RigidBodyDynamics::UpdateKinematicsCustom(*m_robotModel, &jointPos, nullptr,
                                            nullptr);

  return CMR_SUCCESS;
}

//! get point position in world coordinate
cmrVector3d cmrDynamics::getPointPosInWrd(const cmrVectorXd &jointPos,
                                          unsigned int linkId,
                                          const cmrVector3d &posInLink) {
  // check if robot model is created
  if (!m_robotModel) {
    throw cmrException("robot model is not initialized");
  }

  // get point pos
  return RigidBodyDynamics::CalcBodyToBaseCoordinates(*m_robotModel, jointPos,
                                                      linkId, posInLink, false);
}

//! get link orientation in world coordinate
cmrMatrix3d cmrDynamics::getLinkOriInWrd(const cmrVectorXd &jointPos,
                                         unsigned int linkId) {
  // check if robot model is created
  if (!m_robotModel) {
    throw cmrException("robot model is not initialized");
  }

  // get link oritentation
  return RigidBodyDynamics::CalcBodyWorldOrientation(*m_robotModel, jointPos,
                                                     linkId, false);
}

//! get point coordinate with point transform in specific link coordinate
cmrTransform cmrDynamics::getPointTransform(const cmrVectorXd &jointPos,
                                            unsigned int linkId,
                                            const cmrTransform &transInLink) {
  cmrTransform pointTrans;
  // get point pos
  pointTrans.m_pos = getPointPosInWrd(jointPos, linkId, transInLink.m_pos);

  // get point ori
  pointTrans.m_rot = getLinkOriInWrd(jointPos, linkId);

  return pointTrans;
}

//! get tcp transform
cmrTransform cmrDynamics::getTcpTransform(const cmrVectorXd &jointPos) {
  cmrTransform zeroTrans;
  return getPointTransform(jointPos, m_tcpLinkId, zeroTrans);
}

//! get point jacobian with point position in specific link coordinate
cmrMatrixXd cmrDynamics::getPointJacobian(const cmrVectorXd &jointPos,
                                          unsigned int linkId,
                                          const cmrVector3d &posInLink) {
  // check if robot model is created
  if (!m_robotModel) {
    throw cmrException("robot model is not initialized");
  }

  // get jacobian
  cmrMatrixXd jacobian = cmrMatrixXd::Zero(6, m_robotDoFs);
  RigidBodyDynamics::CalcPointJacobian6D(*m_robotModel, jointPos, linkId,
                                         posInLink, jacobian, false);

  // exchange linear and angular jacob
  cmrMatrixXd linearJaco = jacobian.block(0, 0, 3, m_robotDoFs);
  jacobian.block(0, 0, 3, m_robotDoFs) = jacobian.block(3, 0, 3, m_robotDoFs);
  jacobian.block(3, 0, 3, m_robotDoFs) = linearJaco;

  return jacobian;
}

//! compute joint space mass matrix
cmrErrorType cmrDynamics::computeJntSpaceMassMat(const cmrVectorXd jointPos) {
  // check input vector size
  if (jointPos.size() != m_robotDoFs) {
    return CMR_SIZE_NOT_COMPATIBLE;
  }

  // check if robot model is created
  if (!m_robotModel) {
    throw cmrException("robot model is not initialized");
  }

  // get mass matrix in joint space
  m_massMat = cmrMatrixXd::Zero(m_robotDoFs, m_robotDoFs);
  RigidBodyDynamics::CompositeRigidBodyAlgorithm(*m_robotModel, jointPos,
                                                 m_massMat, false);

  return CMR_SUCCESS;
}

//! compute coriolis force
cmrErrorType cmrDynamics::computeCoriolisForce(const cmrVectorXd jointPos,
                                               const cmrVectorXd &jointVel) {
  // check input vector size
  if (jointPos.size() != m_robotDoFs) {
    return CMR_SIZE_NOT_COMPATIBLE;
  }

  // check if robot model is created
  if (!m_robotModel) {
    throw cmrException("robot model is not initialized");
  }

  // compute coriolise foce
  m_coriolisForce = cmrVectorXd::Zero(m_robotDoFs);
  RigidBodyDynamics::NonlinearEffects(*m_robotModel, jointPos, jointVel,
                                      m_coriolisForce, nullptr);

  return CMR_SUCCESS;
}

} // namespace cmr