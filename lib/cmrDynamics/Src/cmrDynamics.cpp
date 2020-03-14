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

  Joint joint_base = Joint(RigidBodyDynamics::JointTypeFixed);
  baseLinkId =
      model->AddBody(0, RigidBodyDynamics::Math::Xtrans(Vector3d(0, 0, 0)),
                     joint_base, body_base, g_baseLinkName);

  string parentLinkName = g_baseLinkName;
  unsigned int lastLinkId = baseLinkId;
  Body curAddBody;
  cmrJointData *curAddJointData = nullptr;
  while (curAddJointData =
             getChildJointData(parentLinkName, robotData.m_jointsData)) {

    cmrLinkData *curAddLinkData =
        robotData.m_linksData
            .begin() // getChildLinkData(parentLinkName, robotData.m_linksData);
        Vector3d linkCOM = curAddLinkData.m_massCenterOrigin.pos;
    Matrix3d linkInertia = curAddLinkData.m_inertia;
    curAddBody = Body(curAddLinkData.m_linkMass, linkCOM, linkInertia);

    RigidBodyDynamics::JointType curAddJointType =
        getChildJointType(curAddJointData.m_jointType);
    Vector3d curAddJointAxis = curAddJointData.m_jointAxis;
    Joint curAddJoint = Joint(curAddJointType, curAddJointAxis);

    SpatialTransform curAddJointTrans;
    curAddJointTrans.E = curAddJointData.m_jointOrigin.rot;
    curAddJointTrans.r = curAddJointData.m_jointOrigin.pos;
    unsigned int curLinkId =
        model->AddBody(lastLinkId, curAddJointTrans, curAddJoint, curAddBody,
                       curAddLinkData.m_linkName);
  }
  cmrLinkData curAddLinkData =
      getChildLinkData(parentLinkName, robotData.m_linksData);
  Vector3d linkCOM = curAddLinkData.m_massCenterOrigin.pos;
  Matrix3d linkInertia = curAddLinkData.m_inertia;
  curAddBody = Body(curAddLinkData.m_linkMass, linkCOM, linkInertia);

  RigidBodyDynamics::JointType curAddJointType =
      getChildJointType(curAddJointData.m_jointType);
  Vector3d curAddJointAxis = curAddJointData.m_jointAxis;
  Joint curAddJoint = Joint(curAddJointType, curAddJointAxis);

  SpatialTransform curAddJointTrans;
  curAddJointTrans.E = curAddJointData.m_jointOrigin.rot;
  curAddJointTrans.r = curAddJointData.m_jointOrigin.pos;
  unsigned int curLinkId =
      model->AddBody(lastLinkId, curAddJointTrans, curAddJoint, curAddBody,
                     curAddLinkData.m_linkName);

  Model *model = NULL;
  unsigned int body_a_id, body_b_id, body_c_id;
  Body body_a, body_b, body_c;
  Joint joint_a, joint_b, joint_c;
  model = new Model();
  model->gravity = Vector3d(0., -9.81, 0.);
  body_a = Body(1., Vector3d(0.5, 0., 0.0), Vector3d(1., 1., 1.));
  joint_a = Joint(JointTypeRevolute, Vector3d(0., 0., 1.));

  body_a_id = model->AddBody(0, Xtrans(Vector3d(0., 0., 0.)), joint_a, body_a);

  body_b = Body(1., Vector3d(0., 0.5, 0.), Vector3d(1., 1., 1.));
  joint_b = Joint(JointTypeRevolute, Vector3d(0., 0., 1.));

  body_b_id =
      model->AddBody(body_a_id, Xtrans(Vector3d(1., 0., 0.)), joint_b, body_b);

  body_c = Body(0., Vector3d(0.5, 0., 0.), Vector3d(1., 1., 1.));
  joint_c = Joint(JointTypeRevolute, Vector3d(0., 0., 1.));

  body_c_id =
      model->AddBody(body_b_id, Xtrans(Vector3d(0., 1., 0.)), joint_c, body_c);
  VectorNd Q = VectorNd::Zero(model->dof_count);
  VectorNd QDot = VectorNd::Zero(model->dof_count);
  VectorNd Tau = VectorNd::Zero(model->dof_count);
  VectorNd QDDot = VectorNd::Zero(model->dof_count);
  ForwardDynamics(*model, Q, QDot, Tau, QDDot);
  std::cout << QDDot.transpose() << std::endl;
  delete model;

  return CMR_SUCCESS;
}

} // namespace cmr