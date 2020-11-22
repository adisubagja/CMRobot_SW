/*
 *cmrTrajDataDef.hpp
 *
 * this file define reletive data struct for trajectory generator
 *
 * Author: Feijian.ni
 * Date: 2020.9.4
 */

#ifndef _CMRTRAJDATADEF_HPP_
#define _CMRTRAJDATADEF_HPP_

#include "cmrMatrix.hpp"

namespace cmr {
//------------------trajectory point data----------------------------
struct cmrTrajPointData {
  //！  position
  cmrVectorXd m_pos;

  //！ velocity
  cmrVectorXd m_vel;

  //! acceleration
  cmrVectorXd m_acc;

  //! jerk
  cmrVectorXd m_jerk;
};

} // namespace cmr
#endif