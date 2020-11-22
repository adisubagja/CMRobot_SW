/*
 *cmrDoubleSTraj.cpp
 *
 * Author: Feijian.Ni
 * Date: 2020.9.6
 */

#include "cmrDoubleSTraj.hpp"
#include "cmrMathDef.hpp"
#include "cmrTimer.hpp"

namespace cmr {
//--------------------Variables Definition------------------------
//! max recursive times to scale m_maxA to generate trajectory
const int g_maxRecursiveTime = 5;

//! recursive step to scale m_maxA to generate trajectory
const double g_recursiveStep = 0.8;

//! default scaler of maxJ to maxA
const double g_maxJerkToAccScaler = 100;

//------------------Double S Trajectory Class----------------------------
//! construct/disconstruct funciton
cmrDoubleSTraj::cmrDoubleSTraj()
    : m_trajGenerated(false), m_trajDims(0), m_trajDuration(0.0) {}

//! init trajectory only with position is defined
cmrErrorType cmrDoubleSTraj::init(const std::vector<cmrVectorXd> waypointsPos,
                                  const cmrVectorXd &maxV,
                                  const cmrVectorXd &maxA) {
  // set max jerk
  cmrVectorXd maxJ = maxA * g_maxJerkToAccScaler;

  // trajectory dimension
  m_trajDims = waypointsPos.at(0).size();

  //! generate velocity for waypoints
  std::vector<cmrVectorXd> waypointsVel;
  interPtsVelCfg(waypointsPos, maxV, maxA, waypointsVel);

  //! configure trajectory points
  std::vector<cmrTrajPointData> waypoints;
  cmrTrajPointData pointData;
  for (unsigned int i = 0; i < waypointsPos.size(); i++) {
    pointData.m_pos = waypointsPos[i];
    pointData.m_vel = waypointsVel[i];
    waypoints.push_back(pointData);
  }

  // generate trajectory
  return init(waypoints, maxV, maxA, maxJ);
}

//! init trajectory with each waypoint is defined
cmrErrorType cmrDoubleSTraj::init(const std::vector<cmrTrajPointData> waypoints,
                                  const cmrVectorXd &maxV,
                                  const cmrVectorXd &maxA,
                                  const cmrVectorXd &maxJ) {
  m_trajGenerated = false;
  // check input waypoints number
  if (waypoints.size() < 2) {
    _ERROR_RETURN("Input waypoints number should be larger than 2");
  }

  // generate segments trajectory
  m_trajSegs.clear();
  m_trajDuration = 0.0;
  for (unsigned int i = 0; i < waypoints.size - 1; i++) {
    // check if trajectory exist
    std::unique_ptr<cmrSegDoubleS> segTraj(new cmrSegDoubleS());
    segTraj->init(waypoints[i], waypoints[i + 1], maxV, maxA, maxJ);
    if (false == segTraj->isTrajGenerated()) {
      m_trajSegs.clear();
      return CMR_SUCCESS;
    }
    m_trajSegs.push_back(segTraj);
    m_trajDuration += segTraj->getTrajDuration();
  }

  m_trajGenerated = false;
  return CMR_SUCCESS;
}

//! generate velocity for waypoints
void cmrDoubleSTraj::interPtsVelCfg(
    const std::vector<cmrVectorXd> &waypointsPos, const cmrVectorXd &maxV,
    const cmrVectorXd &maxA, std::vector<cmrVectorXd> &interPtsVel) {
  interPtsVel.clear();
  // set initial point
  interPtsVel.push_back(cmrVectorXd::Zero(m_trajDims));

  // set intermediate points
  for (unsigned int i = 1; i < waypointsPos.size() - 1; i++) {
    cmrVectorXd sign1 = (waypointsPos[i + 1] - waypointsPos[i]).cwiseSign();
    cmrVectorXd sign2 = (waypointsPos[1] - waypointsPos[i - 1]).cwiseSign();
    cmrVectorXd maxInterVel = maxV.array() * (sign1 + sign2).array() / 2;

    // check if the intermediate velocity is appropriate
    cmrVectorXd thres =
        (waypointsPos[i] - waypointsPos[i - 1]).cwiseAbs().array() *
        maxA.array();
    thres =
        thres - (maxInterVel.cwiseAbs2() - interPtsVel.back().cwiseAbs2()) / 2;
    cmrVectorXd interVel =
        (thres.array() > 0).select(maxInterVel, cmrVectorXd::Zero(m_trajDims));

    interPtsVel.push_back(interVel);
  }

  // set final point
  interPtsVel.push_back(cmrVectorXd::Zero(m_trajDims));
}

//! calculate trajecotry point data
bool cmrDoubleSTraj::calculate(double timePoint, cmrTrajPointData &trajPoint) {
  // check in which segment
  double durationShift = 0.0;
  int segIndex;
  for (segIndex = 0; segIndex < m_trajSegs.size(); segIndex++) {
    if (timePoint - durationShift <= m_trajSegs[segIndex]->getTrajDuration()) {
      break;
    }
    durationShift += m_trajSegs[segIndex]->getTrajDuration();
  }

  // calculate trajectory point
  return m_trajSegs[segIndex]->calculate(timePoint - durationShift, trajPoint);
}

//------------------Double S Class for single
//Segment----------------------------
//! construct/disconstruct funciton
cmrSegDoubleS::cmrSegDoubleS()
    : m_trajGenerated(false), m_Taj(0.0), m_Ta(0.0), m_Tv(0.0), m_Tdj(0.0),
      m_Td(0.0), m_trajDuration(0.0) {}

cmrSegDoubleS::~cmrSegDoubleS() {}

//! constraints preprocess to make sure m_initPos<m_finalPos
void cmrSegDoubleS::preprocess(const cmrTrajPointData &initPoint,
                               const cmrTrajPointData &finalPoint) {
  // init constraint
  m_initPos = initPoint.m_pos;
  m_initVel = initPoint.m_vel;
  m_finalPos = finalPoint.m_pos;
  m_finalVel = finalPoint.m_vel;

  // compute trajectory sign
  m_trajSign = (m_finalPos - m_initPos).cwiseSign().cast<double>();
  m_initPos = m_trajSign.array() * m_initPos.array();
  m_initVel = m_trajSign.array() * m_initVel.array();
  m_finalPos = m_trajSign.array() * m_finalPos.array();
  m_finalVel = m_trajSign.array() * m_finalVel.array();
}

//! calculate time period of each trajectory phase
void cmrSegDoubleS::phaseDurCalc(cmrVectorXd &Taj, cmrVectorXd &Ta,
                                 cmrVectorXd &Tv, cmrVectorXd &Tdj,
                                 cmrVectorXd &Td) {
  // pre-calculate each phase duration, assuming m_maxV is reached
  for (unsigned int i = 0; i < m_trajDims; i++) {
    // acceleration phase computation
    // m_maxA is not reached
    if (m_maxA(i) * m_maxA(i) > (m_maxV(i) - m_initVel(i)) * m_maxJ(i)) {
      Taj(i) = sqrt((m_maxV(i) - m_initVel(i)) / m_maxJ(i));
      Ta(i) = 4 * Taj(i);
      m_limitAcc(i) = Taj(i) * m_maxJ(i);
    }
    // m_maxA is reached
    else {
      Taj(i) = m_maxA(i) / m_maxJ(i);
      Ta(i) = Taj(i) + (m_maxV(i) - m_initVel(i)) / m_maxA(i);
      m_limitAcc(i) = m_maxA(i);
    }

    // deceleration phase computation
    // minA is not reached
    if (m_maxA(i) * m_maxA(i) > (m_maxV(i) - m_finalVel(i)) * m_maxJ(i)) {
      Tdj(i) = sqrt((m_maxV(i) - m_finalVel(i)) / m_maxJ(i));
      Td = 4 * Tdj;
      m_limitDec(i) = -Tdj(i) * m_maxJ(i);

    }
    // minA is reached
    else {
      Tdj(i) = m_maxA(i) / m_maxJ(i);
      Td(i) = Tdj(i) + (m_maxV(i) - m_finalVel(i)) / m_maxA(i);
      m_limitDec(i) = -m_maxA(i);
    }

    // constant velocity phase
    Tv(i) = (m_finalPos(i) - m_initPos(i)) / m_maxV(i) -
            Ta(i) * (1 + m_initVel(i) / m_maxV(i)) / 2 -
            Td(i) * (1 + m_finalVel(i) / m_maxV(i)) / 2;
  }

  // check whether m_maxV is reached and calculate final each phase time
  int curRecursiveTimes = 0;
  bool toGeneTraj = false;
  cmrVectorXd deltaPos = m_finalPos - m_initPos;

  for (unsigned int i = 0; i < m_trajDims; i++) {
    // m_maxV is reached
    if (Tv(i) > 0) {
      m_limitVel(i) = m_maxV(i);
    }
    // m_maxV is not reached, in this case m_maxA must be reached
    else {
      Tv(i) = 0;
      curRecursiveTimes = 0;
      toGeneTraj = false;
      double scaleMaxA = m_maxA(i);

      // scale m_maxA to guarantee traj could be generated
      while (curRecursiveTimes < g_maxRecursiveTime) {
        Taj(i) = scaleMaxA / m_maxJ(i);
        Tdj(i) = scaleMaxA / m_maxJ(i);
        double Delta =
            pow(scaleMaxA, 4) / (m_maxJ(i) * m_maxJ(i)) +
            2 * (m_initVel(i) * m_initVel(i) + m_finalVel(i) * m_finalVel(i)) +
            scaleMaxA *
                (4 * deltaPos(i) -
                 2 * scaleMaxA * (m_initVel(i) + m_finalVel(i)) / m_maxJ(i));
        Ta(i) = (scaleMaxA * scaleMaxA / m_maxJ(i) - 2 * m_initVel(i) +
                 sqrt(Delta)) /
                (2 * scaleMaxA);
        Td(i) = ((scaleMaxA * scaleMaxA) / m_maxJ(i) - 2 * m_finalVel(i) +
                 sqrt(Delta)) /
                (2 * scaleMaxA);

        if (Ta(i) < 0 && m_initVel(i) > m_finalVel(i)) {
          Taj(i) = 0;
          Ta(i) = 0;
          Td(i) = 2 * deltaPos(i) / (m_finalVel(i) + m_initVel(i));
          Tdj(i) = (m_maxJ(i) * deltaPos(i) -
                    sqrt(m_maxJ(i) * (m_maxJ(i) * deltaPos(i) * deltaPos(i) +
                                      pow((m_initVel(i) + m_finalVel(i)), 2) *
                                          (m_finalVel(i) - m_initVel(i))))) /
                   (m_maxJ(i) * (m_initVel(i) + m_finalVel(i)));
          toGeneTraj = true;
        } else if (Td(i) < 0 && m_finalVel(i) > m_initVel(i)) {
          Tdj(i) = 0;
          Td(i) = 0;
          Ta(i) = 2 * deltaPos(i) / (m_initVel(i) + m_finalVel(i));
          Taj(i) = (m_maxJ(i) * deltaPos(i) -
                    sqrt(m_maxJ(i) * (m_maxJ(i) * deltaPos(i) * deltaPos(i) -
                                      pow((m_initVel(i) + m_finalVel(i)), 2) *
                                          (m_finalVel(i) - m_initVel(i))))) /
                   (m_maxJ(i) * (m_initVel(i) + m_finalVel(i)));
          toGeneTraj = true;
        } else if (Ta(i) > 2 * Taj(i) && Td(i) > 2 * Tdj(i)) {
          toGeneTraj = true;
        }

        if (toGeneTraj) {
          m_limitAcc(i) = m_maxJ(i) * Taj(i);
          m_limitDec(i) = -m_maxJ(i) * Tdj(i);
          m_limitVel(i) = m_initVel(i) + (Ta(i) - Taj(i)) * m_limitAcc(i);
          break;
        }
        curRecursiveTimes++;
        scaleMaxA *= g_recursiveStep;
      }

      // failed to generate trajectory
      if (curRecursiveTimes >= g_maxRecursiveTime) {
        m_trajGenerated = false;
        break;
      }
    }
  }
}

//! multiple dimensins synchronization
void cmrSegDoubleS::multiDimsSync(const cmrVectorXd &Taj, const cmrVectorXd &Ta,
                                  const cmrVectorXd &Tv, const cmrVectorXd &Tdj,
                                  const cmrVectorXd &Td) {
  // get max trajectory period
  cmrVectorXd totalDuration = Ta + Tv + Td;
  int maxDurIndex;
  double maxDuration = totalDuration.maxCoeff(&maxDurIndex);
  m_Taj = Taj(maxDurIndex);
  m_Ta = Ta(maxDurIndex);
  m_Tv = Tv(maxDurIndex);
  m_Tdj = Tdj(maxDurIndex);
  m_Td = Td(maxDurIndex);

  // trajectory scaling
  cmrVectorXd trajScale = totalDuration / maxDuration;
  m_limitVel = m_limitVel.array() / trajScale.array();
  m_limitAcc = m_limitAcc.array() / (trajScale.array() * trajScale.array());
  m_limitDec = m_limitDec.array() / (trajScale.array() * trajScale.array());
  m_limitJ = m_maxJ.array() /
             (trajScale.array() * trajScale.array() * trajScale.array());
}

//! initialize trajectory
cmrErrorType cmrSegDoubleS::init(const cmrTrajPointData &initPoint,
                                 const cmrTrajPointData &finalPoint,
                                 const cmrVectorXd &maxV,
                                 const cmrVectorXd &maxA,
                                 const cmrVectorXd &maxJ) {
  // check size
  m_trajDims = initPoint.m_pos.size();
  if (finalPoint.m_pos.size() != m_trajDims ||
      initPoint.m_vel.size() != m_trajDims ||
      finalPoint.m_vel.size() != m_trajDims || m_maxV.size() != m_trajDims ||
      m_maxV.size() != m_trajDims || m_maxJ.size() != m_trajDims) {
    _ERROR_RETURN("position or velocity size not compatible");
  }

  // initialize member variables
  m_limitVel = cmrVectorXd::Zero(m_trajDims);
  m_limitDec = cmrVectorXd::Zero(m_trajDims);
  m_limitAcc = cmrVectorXd::Zero(m_trajDims);
  m_limitJ = cmrVectorXd::Zero(m_trajDims);
  m_trajGenerated = false;
  m_trajDuration = 0.0;
  m_Taj = 0.0;
  m_Ta = 0.0;
  m_Tv = 0.0;
  m_Tdj = 0.0;
  m_Td = 0.0;

  // check if trajecory exist
  if ((initPoint.m_pos - finalPoint.m_pos).norm() < g_smallNum) {
    m_trajGenerated = true;
    return CMR_SUCCESS;
  }

  cmrVectorXd thres =
      (initPoint.m_pos - finalPoint.m_pos).cwiseAbs().array() * maxA.array();
  thres =
      thres - (initPoint.m_vel.cwiseAbs2() - finalPoint.m_vel.cwiseAbs2()) / 2;
  if (thres.minCoeff() < 0) {
    return CMR_SUCCESS;
  }

  // preprocess trajectory constraints
  preprocess(initPoint, finalPoint);
  m_maxV = maxV.cwiseAbs();
  m_maxA = maxA.cwiseAbs();
  m_maxJ = maxJ.cwiseAbs();

  //! duration of each phase
  cmrVectorXd Taj = cmrVectorXd::Zero(m_trajDims);
  cmrVectorXd Ta = cmrVectorXd::Zero(m_trajDims);
  cmrVectorXd Tdj = cmrVectorXd::Zero(m_trajDims);
  cmrVectorXd Td = cmrVectorXd::Zero(m_trajDims);
  cmrVectorXd Tv = cmrVectorXd::Zero(m_trajDims);

  //! calculate duration of each trajectory phase, assuming m_maxV is reached
  phaseDurCalc(Taj, Ta, Tv, Tdj, Td);

  //! multiple dimensins synchronization
  multiDimsSync(Taj, Ta, Tv, Tdj, Td);

  m_trajGenerated = true;

  return CMR_SUCCESS;
}

//! calculate trajecotry point data
bool cmrSegDoubleS::calculate(double timePoint, cmrTrajPointData &trajPoint) {
  if (!m_trajGenerated) {
    return false;
  }

  // acceleration phase
  if (timePoint < 0.0) {
    trajPoint.m_pos = m_initPos;
    trajPoint.m_vel = m_initVel;
    trajPoint.m_acc = cmrVectorXd::Zero(m_trajDims)
  } else if (timePoint < m_Taj && timePoint >= 0) {
    trajPoint.m_pos =
        m_initPos + m_initVel * timePoint + m_limitJ * pow(timePoint, 3) / 6;
    trajPoint.m_vel = m_initVel + m_limitJ * pow(timePoint, 2) / 2;
    trajPoint.m_acc = m_limitJ * timePoint;
  } else if (timePoint >= m_Taj && timePoint < (m_Ta - m_Taj)) {
    trajPoint.m_pos =
        m_initPos + m_initVel * timePoint +
        m_limitAcc *
            (3 * pow(timePoint, 2) - 3 * m_Taj * timePoint + pow(m_Taj, 2)) / 6;
    trajPoint.m_vel = m_initVel + m_limitAcc * (timePoint - m_Taj / 2);
    trajPoint.m_acc = m_limitAcc;
  } else if (timePoint >= (m_Ta - m_Taj) && timePoint < m_Ta) {
    double t = m_Ta - timePoint;
    trajPoint.m_pos = m_initPos + (m_initVel + m_limitVel) * m_Ta / 2 -
                      m_limitVel * t + m_limitJ * pow(t, 3) / 6;
    trajPoint.m_vel = m_limitVel - m_limitJ * pow(t, 2) / 2;
    trajPoint.m_acc = m_limitJ * t;
  }
  // constant phase
  else if (timePoint >= m_Ta && timePoint < (m_Tv + m_Ta)) {
    trajPoint.m_pos = m_initPos + (m_initVel + m_limitVel) * m_Ta / 2 +
                      m_limitVel * (timePoint - m_Ta);
    trajPoint.m_vel = m_limitVel;
    trajPoint.m_acc = cmrVectorXd::Zero(m_trajDims);
  }
  // deceleration phase
  else if (timePoint >= (m_trajDuration - m_Td) &&
           timePoint < (m_trajDuration - m_Td + m_Tdj)) {
    double t = timePoint - m_trajDuration + m_Td;
    trajPoint.m_pos = m_finalPos - (m_limitVel + m_finalVel) * m_Td / 2 +
                      m_limitVel * t - m_limitJ * pow(t, 3) / 6;
    trajPoint.m_vel = m_limitVel - m_limitJ * pow(t, 2) / 2;
    trajPoint.m_acc = -m_limitJ * t;
  } else if (timePoint >= (m_trajDuration - m_Td + m_Tdj) &&
             timePoint < (m_trajDuration - m_Tdj)) {
    double t = timePoint - m_trajDuration + m_Td;
    trajPoint.m_pos =
        m_finalPos - (m_limitVel + m_finalVel) * m_Td / 2 + m_limitVel * t +
        m_limitDec * (3 * t * t - 3 * m_Tdj * t + m_Tdj * m_Tdj) / 6;
    trajPoint.m_vel = m_limitVel + m_limitDec * (t - m_Tdj / 2);
    trajPoint.m_acc = m_limitDec;
  } else if (timePoint >= (m_trajDuration - m_Tdj) &&
             timePoint < m_trajDuration) {
    double t = m_trajDuration - timePoint;
    trajPoint.m_pos = m_finalPos - m_finalVel * t - m_limitJ * pow(t, 3) / 6;
    trajPoint.m_vel = m_finalVel + m_limitJ * pow(t, 2) / 2;
    trajPoint.m_acc = -m_limitJ * t;
  } else {
    trajPoint.m_pos = m_finalPos;
    trajPoint.m_vel = m_finalVel;
    trajPoint.m_acc = cmrVectorXd::Zero(m_trajDims);
  }

  // add trajectory sign
  trajPoint.m_pos = trajPoint.m_pos.array() * m_trajSign.array();
  trajPoint.m_vel = trajPoint.m_vel.array() * m_trajSign.array();
  trajPoint.m_acc = trajPoint.m_acc.array() * m_trajSign.array();

  return true;
}

} // namespace cmr