/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2007 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use
of this software. Permission is granted to anyone to use this software for any
purpose, including commercial applications, and to alter it and redistribute it
freely, subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim
that you wrote the original software. If you use this software in a product, an
acknowledgment in the product documentation would be appreciated but is not
required.
2. Altered source versions must be plainly marked as such, and must not be
misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

///-----includes_start-----
#include "btBulletDynamicsCommon.h"
#include "cmrMathDef.hpp"
#include "cmrTimer.hpp"
#include <chrono>
#include <iostream>
#include <pthread.h>
#include <stdio.h>

#include <GL/glu.h>
#include <GL/glut.h>
#include <math.h>
#include <mutex>

using namespace std::chrono;
using namespace cmr;
/// This is a Hello World program for running a basic Bullet physics simulation
float sopherePos1[3];
float sopherePos2[3];
std::mutex posLock;

const double g_unitLength = 150.0;

// bullt variable declare
btDiscreteDynamicsWorld *dynamicsWorld;
btDefaultCollisionConfiguration *collisionConfiguration;
btCollisionDispatcher *dispatcher;
btBroadphaseInterface *overlappingPairCache;
btSequentialImpulseConstraintSolver *solver;
btAlignedObjectArray<btCollisionShape *> collisionShapes;
void bulletInit() {
  ///-----initialization_start-----

  /// collision configuration contains default setup for memory, collision
  /// setup. Advanced users can create their own configuration.
  collisionConfiguration = new btDefaultCollisionConfiguration();

  /// use the default collision dispatcher. For parallel processing you can use
  /// a diffent dispatcher (see Extras/BulletMultiThreaded)
  dispatcher = new btCollisionDispatcher(collisionConfiguration);

  /// btDbvtBroadphase is a good general purpose broadphase. You can also try
  /// out btAxis3Sweep.
  overlappingPairCache = new btDbvtBroadphase();

  /// the default constraint solver. For parallel processing you can use a
  /// different solver (see Extras/BulletMultiThreaded)
  solver = new btSequentialImpulseConstraintSolver;

  dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, overlappingPairCache,
                                              solver, collisionConfiguration);

  dynamicsWorld->setGravity(btVector3(0, -10, 0));

  ///-----initialization_end-----
}

void bulletObjCreat() {
  {
    btCollisionShape *groundShape =
        new btBoxShape(btVector3(btScalar(50.), btScalar(50.), btScalar(50.)));

    collisionShapes.push_back(groundShape);

    btTransform groundTransform;
    groundTransform.setIdentity();
    groundTransform.setOrigin(btVector3(0, -56, 0));

    btScalar mass(0.);

    // rigidbody is dynamic if and only if mass is non zero, otherwise static
    bool isDynamic = (mass != 0.f);

    btVector3 localInertia(0, 0, 0);
    if (isDynamic)
      groundShape->calculateLocalInertia(mass, localInertia);

    // using motionstate is optional, it provides interpolation capabilities,
    // and only synchronizes 'active' objects
    btDefaultMotionState *myMotionState =
        new btDefaultMotionState(groundTransform);
    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState,
                                                    groundShape, localInertia);
    btRigidBody *body = new btRigidBody(rbInfo);

    // add the body to the dynamics world
    dynamicsWorld->addRigidBody(body);
  }

  {
    // create a dynamic rigidbody

    // btCollisionShape* colShape = new btBoxShape(btVector3(1,1,1));
    btCollisionShape *colShape = new btSphereShape(btScalar(3.));
    collisionShapes.push_back(colShape);

    /// Create Dynamic Objects
    btTransform startTransform;
    startTransform.setIdentity();

    btScalar mass(1.f);

    // rigidbody is dynamic if and only if mass is non zero, otherwise static
    bool isDynamic = (mass != 0.f);

    btVector3 localInertia(0, 0, 0);
    if (isDynamic)
      colShape->calculateLocalInertia(mass, localInertia);

    startTransform.setOrigin(btVector3(2, 10, 0));

    // using motionstate is recommended, it provides interpolation capabilities,
    // and only synchronizes 'active' objects
    btDefaultMotionState *myMotionState =
        new btDefaultMotionState(startTransform);
    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState,
                                                    colShape, localInertia);
    btRigidBody *body = new btRigidBody(rbInfo);

    dynamicsWorld->addRigidBody(body);
  }

  {
    // create a dynamic rigidbody

    // btCollisionShape* colShape = new btBoxShape(btVector3(1,1,1));
    btCollisionShape *colShape = new btSphereShape(btScalar(3.));
    collisionShapes.push_back(colShape);

    /// Create Dynamic Objects
    btTransform startTransform;
    startTransform.setIdentity();

    btScalar mass(1.f);

    // rigidbody is dynamic if and only if mass is non zero, otherwise static
    bool isDynamic = (mass != 0.f);

    btVector3 localInertia(0, 0, 0);
    if (isDynamic)
      colShape->calculateLocalInertia(mass, localInertia);

    startTransform.setOrigin(btVector3(0, 17, 0));

    // using motionstate is recommended, it provides interpolation capabilities,
    // and only synchronizes 'active' objects
    btDefaultMotionState *myMotionState =
        new btDefaultMotionState(startTransform);
    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState,
                                                    colShape, localInertia);
    btRigidBody *body = new btRigidBody(rbInfo);

    dynamicsWorld->addRigidBody(body);
  }
}

void bulletClean() {
  // remove the rigidbodies from the dynamics world and delete them
  for (int i = dynamicsWorld->getNumCollisionObjects() - 1; i >= 0; i--) {
    btCollisionObject *obj = dynamicsWorld->getCollisionObjectArray()[i];
    btRigidBody *body = btRigidBody::upcast(obj);
    if (body && body->getMotionState()) {
      delete body->getMotionState();
    }
    dynamicsWorld->removeCollisionObject(obj);
    delete obj;
  }

  // delete collision shapes
  for (int j = 0; j < collisionShapes.size(); j++) {
    btCollisionShape *shape = collisionShapes[j];
    collisionShapes[j] = 0;
    delete shape;
  }

  // delete dynamics world
  delete dynamicsWorld;

  // delete solver
  delete solver;

  // delete broadphase
  delete overlappingPairCache;

  // delete dispatcher
  delete dispatcher;

  delete collisionConfiguration;

  // next line is optional: it will be cleared by the destructor when the array
  // goes out of scope
  collisionShapes.clear();
}

// opengl variable declare
#define pi 3.1415926
bool mouseisdown = false;
bool loopr = false;
int mx, my;
int ry = 0;
int rx = 0;

void openglInit(void) {
  GLfloat mat_specular[] = {1.0, 1.0, 1.0, 1.0};
  GLfloat mat_shininess[] = {50.0};
  GLfloat light_position[] = {1.0, 1.0, 1.0, 0.0};
  glClearColor(0.0, 0.0, 0.0, 1.0);
  glShadeModel(GL_SMOOTH);
  glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);
  glMaterialfv(GL_FRONT, GL_SHININESS, mat_shininess);
  glLightfv(GL_LIGHT0, GL_POSITION, light_position);
  // glEnable (GL_LIGHTING);
  // glEnable (GL_LIGHT0);
}

void windowInit(int argc, char **argv) {
  /* GLUT环境初始化*/
  glutInit(&argc, argv);
  /* 显示模式初始化 */
  glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB | GLUT_DEPTH);
  /* 定义窗口大小 */
  glutInitWindowSize(300, 300);
  /* 定义窗口位置 */
  glutInitWindowPosition(100, 100);
  /* 显示窗口，窗口标题为执行函数名 */
  glutCreateWindow(argv[0]);
  /* 调用OpenGL初始化函数 */
  openglInit();
}

/* 定时器函数 */
void timer(int p) {
  ry -= 5;
  // marks the current window as needing to be redisplayed.
  glutPostRedisplay();
  if (loopr)
    // re-start the timer, after 200ms, timer would be called
    glutTimerFunc(200, timer, 0);
}

/* 鼠标状态检查函数 */
void mouse(int button, int state, int x, int y) {
  if (button == GLUT_LEFT_BUTTON) {
    if (state == GLUT_DOWN) {
      mouseisdown = true;
      loopr = false;
    } else
      mouseisdown = false;
  }

  if (button == GLUT_RIGHT_BUTTON)
    if (state == GLUT_DOWN) {
      loopr = true;
      glutTimerFunc(200, timer, 0);
    }
}

/* 鼠标运动检查函数 */
void motion(int x, int y) {
  if (mouseisdown == true) {
    ry += x - mx;
    rx += y - my;
    mx = x;
    my = y;
    glutPostRedisplay();
  }
}

/* 特殊按键监测函数 */
void special(int key, int x, int y) {
  switch (key) {
  case GLUT_KEY_LEFT:
    ry -= 5;
    glutPostRedisplay();
    break;
  case GLUT_KEY_RIGHT:
    ry += 5;
    glutPostRedisplay();
    break;
  case GLUT_KEY_UP:
    rx += 5;
    glutPostRedisplay();
    break;
  case GLUT_KEY_DOWN:
    rx -= 5;
    glutPostRedisplay();
    break;
  }
}

/*渲染函数，调用GLUT函数，绘制图像*/
void display() {
  glClear(GL_DEPTH_BUFFER_BIT);
  glClearDepth(10.0);
  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LESS);
  float red[3] = {1, 0, 0};
  float green[3] = {0, 1, 0};
  float blue[3] = {0, 0, 1};
  float yellow[3] = {1, 1, 0};

  glLoadIdentity();
  // glOrtho(-1.0, 1.0,-1.0,1.0,-10,10);
  // gluPerspective(90.0f, 1.0f, 1.0f, 2000.0f);
  glRotatef(ry, 0, 1, 0);
  glRotatef(rx, 1, 0, 0);
  glClearColor(1, 1, 1, 1);
  glClear(GL_COLOR_BUFFER_BIT);

  glPushMatrix();
  glTranslatef(0, -56.0 / g_unitLength, 0);
  glColor3fv(blue);
  glutSolidCube(100.0 / g_unitLength);
  glPopMatrix();

  glPushMatrix();
  glColor3fv(red);
  glTranslatef(sopherePos1[0], sopherePos1[1], sopherePos1[2]);
  glutSolidSphere(3.0 / g_unitLength, 20, 20);
  glPopMatrix();

  glPushMatrix();
  glColor3fv(green);
  glTranslatef(sopherePos2[0], sopherePos2[1], sopherePos2[2]);
  glutSolidSphere(3.0 / g_unitLength, 20, 20);
  glPopMatrix();
  // glFrustum(-1.5, 1.5,-1.5,1.5,-10,10);
  // glOrtho(-1.5, 1.5,-1.5,1.5,-10,10);
  glFlush();
  glutPostRedisplay();
}

void *openglRun(void *) {

  /* 注册OpenGL绘图函数 */
  glutDisplayFunc(display);
  /* 注册鼠标状态函数*/
  glutMouseFunc(mouse);
  /* 注册鼠标运动函数*/
  glutMotionFunc(motion);
  /* 注册特殊按键函数*/
  glutSpecialFunc(special);
  // /* 进入GLUT消息循环，开始执行程序 */
  glutMainLoop();
}

int main(int argc, char **argv) {
  bulletInit();
  bulletObjCreat();

  windowInit(argc, argv);

  pthread_t threadId;
  pthread_create(&threadId, NULL, openglRun, NULL);
  ///-----stepsimulation_start-----
  while (true) {
    high_resolution_clock::time_point loopstart = high_resolution_clock::now();
    dynamicsWorld->stepSimulation(200.0f / 1000.f, 100, 200.0 / 1000.0);

    // print positions of all objects
    for (int j = dynamicsWorld->getNumCollisionObjects() - 1; j > 0; j--) {
      btCollisionObject *obj = dynamicsWorld->getCollisionObjectArray()[j];
      btRigidBody *body = btRigidBody::upcast(obj);
      btTransform trans;
      if (body && body->getMotionState()) {
        // btTransform startTransform;
        // startTransform.setIdentity();
        // btVector3 pos(0,15,0);
        // startTransform.setOrigin(pos);
        // body->getMotionState()->setWorldTransform(startTransform);
        body->getMotionState()->getWorldTransform(trans);
        std::cout << "!!!" << trans.getOrigin().getY() << std::endl;
      } else {
        trans = obj->getWorldTransform();
      }
      std::cout << "~~~" << obj->getWorldTransform().getOrigin().getY()
                << std::endl;
      // printf("world pos object %d = %f,%f,%f\n", j,
      // float(trans.getOrigin().getX()), float(trans.getOrigin().getY()),
      // float(trans.getOrigin().getZ()));
      posLock.lock();
      if (j == 1) {
        sopherePos1[0] = trans.getOrigin().getX() / g_unitLength;
        sopherePos1[1] = trans.getOrigin().getY() / g_unitLength;
        sopherePos1[2] = trans.getOrigin().getZ() / g_unitLength;
      }
      if (j == 2) {
        sopherePos2[0] = trans.getOrigin().getX() / g_unitLength;
        sopherePos2[1] = trans.getOrigin().getY() / g_unitLength;
        sopherePos2[2] = trans.getOrigin().getZ() / g_unitLength;
      }
      posLock.unlock();
    }
    high_resolution_clock::time_point loopend = high_resolution_clock::now();
    double loopTime = duration_cast<milliseconds>(loopend - loopstart).count();
    double sleepTime = 1e3 * g_controlCycleTime - loopTime;
    if (sleepTime > 0) {
      cmrSleep(sleepTime);
    } else {
      std::cout << "looptime is " << loopTime << std::endl;
    }
  }

  bulletClean();
}
