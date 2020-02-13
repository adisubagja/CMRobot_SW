
#include "myHello.h"
#include "multiThreadTest.hpp"
#include "baseStdLibs.hpp"
#include "cmrMath.hpp"
#include <math.h>
int a =0;
using namespace cmr;
int main()
{
    cmrVector3d vec3;
    cmrMatrix3d mat3;
    cmrAngleAxis axis(M_PI/4,cmrVector3d(1,0,0));
    cmrQuat quat;

    // std::cout<<"axis is "<<axis.matrix()<<std::endl;
    // mat3=axis;
    // std::cout<<"mat is "<<mat3<<std::endl;
    // quat=axis;
    // std::cout<<"quat is "<<quat.coeffs()<<std::endl;
    // std::cout<<"quat is "<<quat.w()<<quat.x()<<quat.y()<<quat.z()<<std::endl;
    std::thread t1(addNum,std::ref(a));
    std::thread t2(minuNum,std::ref(a));
    t1.join();
    t2.join();
    // printA();
    // printB();
    printMyHello();
    return 0;
}