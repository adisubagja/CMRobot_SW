/*
 *cmrDoubleSTraj.hpp
 *
 * this file generate trapzoidal trajectory in joint space
 *
 * Author: Feijian.ni
 * Date: 2020.7.4
 */

#ifndef _CMRTDOUBLESTRAJ_HPP
#define _CMRTDOUBLESTRAJ_HPP

#include "cmrException.hpp"
#include "cmrTrajDataDef.hpp"

namespace cmr {

//------------------Double S trajectory class----------------------------
class cmrDoubleSTraj {
public:
  cmrDoubleSTraj();
  ~cmrDoubleSTraj();

  cmrDoubleSTraj(const cmrDoubleSTraj&) = delete;
  cmrDoubleSTraj& operator=(const cmrDoubleSTraj&) =delete;

  //! init trajectory with each waypoint is defined
  cmrErrorType init(const std::vector<cmrTrajPointData> waypoints,
                    const cmrVectorXd &maxV, const cmrVectorXd &maxA,
                    const cmrVectorXd &maxJ);

  //! init trajectory only with position is defined
  cmrErrorType init(const std::vector<cmrVectorXd> waypointsPos,
                    const cmrVectorXd &maxV, const cmrVectorXd &maxA);

  //! calculate trajecotry point data
  bool calculate(double timePoint, cmrTrajPointData &trajPoint);

  //! check if trajectory is generated
  bool isTrajGenerated() const { return m_trajGenerated; }

  //! return trajectory duration
  double getTrajDuration() const { return m_trajDuration; }

private:
  //! generate velocity for waypoints
  void interPtsVelCfg(const std::vector<cmrVectorXd> &waypointsPos,
                      const cmrVectorXd &maxV, const cmrVectorXd &maxA,
                      std::vector<cmrVectorXd> &interPtsVel);

  //! trajectory segments
  std::vector<std::unique_ptr<cmrSegDoubleS>> m_trajSegs;

  //! flag if trajectory is generated successfully
  bool m_trajGenerated;

  //! total trajectory duration
  double m_trajDuration;

  //! trajecotry dimensions
  int m_trajDims;
};

//------------------Double S Class for single Segment---------------------
class cmrSegDoubleS {
public:
  cmrSegDoubleS();
  ~cmrSegDoubleS();

  //! init
  cmrErrorType init(const cmrTrajPointData &initPoint,
                    const cmrTrajPointData &finalPoint, const cmrVectorXd &maxV,
                    const cmrVectorXd &maxA, const cmrVectorXd &maxJ);

  //! calculate trajecotry point data
  bool calculate(double timePoint, cmrTrajPointData &trajPoint);

  //! check if trajectory is generated
  bool isTrajGenerated() const { return m_trajGenerated; }

  //! return trajectory duration
  double getTrajDuration() const { return m_trajDuration; }

private:
  //! constraints preprocess to make sure q0<q1
  void preprocess(const cmrTrajPointData &initPoint,
                  const cmrTrajPointData &endPoint);

  //! calculate duration of each trajectory phase, assuming maxV is reached
  void phaseDurCalc(cmrVectorXd &Taj, cmrVectorXd &Ta, cmrVectorXd &Tv,
                    cmrVectorXd &Tdj, cmrVectorXd &Td);

  //! multiple dimensins synchronization
  void multiDimsSync(const cmrVectorXd &Taj, const cmrVectorXd &Ta,
                     const cmrVectorXd &Tv, const cmrVectorXd &Tdj,
                     const cmrVectorXd &Td);

  //! trajecotry dimensions
  int m_trajDims;

  //! init position
  cmrVectorXd m_initPos;

  //! init velocity
  cmrVectorXd m_initVel;

  //! final position
  cmrVectorXd m_finalPos;

  //! final velocity
  cmrVectorXd m_finalVel;

  //! max velocity
  cmrVectorXd m_maxV;

  //! max acceleration
  cmrVectorXd m_maxA;

  //! max jerk
  cmrVectorXd m_maxJ;

  //! trajectory sign
  cmrVectorXd m_trajSign;

  //! limit jerk
  cmrVectorXd m_limitJ;

  //! limit acceleration
  cmrVectorXd m_limitAcc;

  //! limit deceleration
  cmrVectorXd m_limitDec;

  //! limit velocity
  cmrVectorXd m_limitVel;

  //! Duration with jerk as constant in acceletaion phase
  double m_Taj;

  //! Duration of acceletaion phase
  double m_Ta;

  //! Duration with jerk as constant in deceletaion phase
  double m_Tdj;

  //! Duration of deceletaion phase
  double m_Td;

  //! Duration of constance velocity phase
  double m_Tv;

  //! flag if trajecotry is generated
  bool m_trajGenerated;

  //! total trajectory Duration
  double m_trajDuration;
};

} // namespace cmr
#endif