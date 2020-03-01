/*
 * cmrTransform.hpp
 * 
 * Description: This file deine the coordinate transformation
 *              and its API
 * 
 * Author: Feijian.Ni
 * Date: 2020.2.23
 * 
*/
#ifndef CMRTRANSFORM_HPP_
#define CMRTRANSFORM_HPP_

#include "baseStdLibs.hpp"
#include "cmrMath.hpp"

namespace cmr{

class cmrTransform
{
public:

    cmrTransform()
    {
        pos=cmrVector3d::Zero();
        rot=cmrMatrix3d::Identity();
    }
    
    cmrVector3d pos;
    cmrMatrix3d rot;

    //! get transform rot from rpy euler angel
    void setRotWihtRPY(double roll,double pitch, double yaw)
    {
        rot=cmrAngleAxis(yaw, cmrVector3d::UnitZ())*
            cmrAngleAxis(pitch,cmrVector3d::UnitY())*
            cmrAngleAxis(roll,cmrVector3d::UnitX());
    }
};
}

#endif