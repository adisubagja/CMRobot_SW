/*
 * cmrDynamics.hpp
 *
 * Description: This file provide kinematic and dynamic
 * computation for robot
 *
 * Author: Feijian.Ni
 * Date: 2020.2.13
 */

#ifndef CMRDYNAMICS_HPP_
#define CMRDYNAMICS_HPP_

#include "cmrException.hpp"
#include "cmrRobotCfgData.hpp"
#include "rbdl/rbdl.h"
#include <unordered_map>

namespace cmr{

class cmrDynamics
{
public:
  cmrDynamics();
  ~cmrDynamics();

  //! creat robot model with urdf file
  cmrErrorType createRobotModel(const cmrRobotCfgData &robotData);

  //! get robot DoFs
  inline unsigned int getRobotDoFs() { return m_robotDoFs; }

  //! get link id
  unsigned int getLinkId(std::string linkName);

  //! get tcp transform
  cmrTransform getTcpTransform(const cmrVectorXd &jointPos);

  //! get rbdl joint type
  RigidBodyDynamics::JointType getChildJointType(cmrJointType jointType);

  //! update robot model kinematic with joint position
  cmrErrorType updataRobotKinematic(const cmrVectorXd &jointPos);

  //! get point position in world coordinate with point position in specific
  //! link coordinate
  cmrVector3d getPointPosInWrd(const cmrVectorXd &jointPos, unsigned int linkId,
                               const cmrVector3d &posInLink);

  //! get link orientation in world coordinate
  cmrMatrix3d getLinkOriInWrd(const cmrVectorXd &jointPos, unsigned int linkId);

  //! get point coordinate in world coordinate with point transform in specific
  //! link coordinate
  cmrTransform getPointTransform(const cmrVectorXd &jointPos,
                                 unsigned int linkId,
                                 const cmrTransform &transInLink);

  //! get point jacobian with point position in specific link coordinate
  cmrMatrixXd getPointJacobian(const cmrVectorXd &jointPos, unsigned int linkId,
                               const cmrVector3d &posInLink);

  //! compute joint space mass matrix
  cmrErrorType computeJntSpaceMassMat(const cmrVectorXd jointPos);

  //! compute coriolis force
  cmrErrorType computeCoriolisForce(const cmrVectorXd jointPos,
                                    const cmrVectorXd &jointVel);

private:
  //! robot model
  RigidBodyDynamics::Model *m_robotModel;

  //! link name and id map
  std::unordered_map<std::string, unsigned int> m_linkNameIdMap;

  //! tcp link id
  unsigned int m_tcpLinkId;

  //! robot DoFs
  unsigned int m_robotDoFs;

  //! joint space mass matrix
  cmrMatrixXd m_massMat;

  //! coriolis force
  cmrVectorXd m_coriolisForce;
};

} // namespace cmr

#endif