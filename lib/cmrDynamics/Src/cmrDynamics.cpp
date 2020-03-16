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

cmrDynamics::cmrDynamics(){};

cmrDynamics::~cmrDynamics(){};

//! creat robot model with urdf file
cmrErrorType cmrDynamics::createRobotModel(const cmrRobotData &robotData,
                                           RigidBodyDynamics::Model *model) {
  model = new RigidBodyDynamics::Model();

  //! set gravity
  model->gravity = robotData.m_gravity;

  //! add base link first
  unsigned int baseLinkId = 0;
  Body body_base;
  for (const auto &link : robotData.m_linksData) {
    if (g_baseLinkName == link.m_linkName) {
      Vector3d linkCOM = link.m_massCenterOrigin.pos;
      Matrix3d linkInertia = link.m_inertia;
      body_base = Body(link.m_linkMass, linkCOM, linkInertia);
    }
  }

  //! fixed joint for base link
  Joint joint_base = Joint(RigidBodyDynamics::JointTypeFixed);
  baseLinkId =
      model->AddBody(0, RigidBodyDynamics::Math::Xtrans(Vector3d(0, 0, 0)),
                     joint_base, body_base, g_baseLinkName);

  //! add following links and joints in sequence
  string parentLinkName = g_baseLinkName;
  unsigned int lastLinkId = baseLinkId;
  const cmrJointData *curAddJointData = nullptr;

  //! if current link is connect to another joint
  while (curAddJointData = robotData.getChildJointData(parentLinkName)) {
    //! get next link
    const cmrLinkData *curAddLinkData =
        robotData.getChildLinkData(curAddJointData);
    Vector3d linkCOM = curAddLinkData->m_massCenterOrigin.pos;
    Matrix3d linkInertia = curAddLinkData->m_inertia;
    Body curAddBody = Body(curAddLinkData->m_linkMass, linkCOM, linkInertia);

    //! get next joint
    RigidBodyDynamics::JointType curAddJointType =
        getChildJointType(curAddJointData->m_jointType);
    Vector3d curAddJointAxis = curAddJointData->m_jointAxis;
    Joint curAddJoint = Joint(curAddJointType, curAddJointAxis);

    //! set joint transform to parent link
    SpatialTransform curAddJointTrans;
    curAddJointTrans.E = curAddJointData->m_jointOrigin.rot;
    curAddJointTrans.r = curAddJointData->m_jointOrigin.pos;
    unsigned int curLinkId =
        model->AddBody(lastLinkId, curAddJointTrans, curAddJoint, curAddBody,
                       curAddLinkData->m_linkName);

    //! update link id
    lastLinkId = curLinkId;
  }

  return CMR_SUCCESS;
}

//! get rbdl joint type
RigidBodyDynamics::JointType
cmrDynamics::getChildJointType(cmrJointType jointType) {
  RigidBodyDynamics::JointType rbdlJointType =
      RigidBodyDynamics::JointTypeUndefined;

  //! get rbdl joint type
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

} // namespace cmr