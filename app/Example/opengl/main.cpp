#include <GL/glut.h>
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#include <iostream>
GLuint tex2D;
GLfloat angle;
unsigned char *texPtr;

bool mouseisdown=false;

bool loopr=false;

int mx,my;
int ry = 0;

int rx = 0;

/* 鼠标状态检查函数 */
void mouse(int button, int state, int x, int y)
{
    if(button == GLUT_LEFT_BUTTON)
     {
        if(state == GLUT_DOWN)
         {    mouseisdown=true; loopr=false;}
         else
              mouseisdown=false;
     }
}

/* 鼠标运动检查函数 */
void motion(int x, int y)
{
    if(mouseisdown==true)
    {
        ry+=x-mx;
         rx+=y-my;
         mx=x;
         my=y;
         glutPostRedisplay();
    }
}

// 初始化参数
void init() {
  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LESS);
  glClearColor(1, 1, 1, 0.0);
  glShadeModel(GL_SMOOTH);

  // load texture
  int imgWidth, imgHeight, channels;
  texPtr = stbi_load("./rabit.jpg", &imgWidth, &imgHeight, &channels, 0);
  std::cout << imgWidth << " " << imgHeight << " " << channels << std::endl;

  // 创建纹理
  glGenTextures(1, &tex2D);
  glBindTexture(GL_TEXTURE_2D, tex2D);

  // 纹理滤波参数设置
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);

  // 设置纹理数据
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, imgWidth, imgHeight, 0, GL_RGB,
               GL_UNSIGNED_BYTE, texPtr);
  angle = 0;
}

/** 绘制木箱 */
void DrawBox() {
  float blue[3] = {0, 0, 1};
  float red[3] = {1, 0, 0};
  glColor3fv(blue);
  glEnable(GL_TEXTURE_2D);

  /** 选择纹理 */
  glBindTexture(GL_TEXTURE_2D, tex2D);

  /** 开始绘制四边形 */
  glBegin(GL_QUADS);

  /// 前侧面
  glNormal3f(0.0f, 0.0f, 0.5f); /**指定法线指向观察者 */
  glTexCoord2f(0.0f, 0.0f);
  glVertex3f(-0.5f, -0.5f, 0.5f);
  glTexCoord2f(0.5f, 0.0f);
  glVertex3f(0.5f, -0.5f, 0.5f);
  glTexCoord2f(0.5f, 0.5f);
  glVertex3f(0.5f, 0.5f, 0.5f);
  glTexCoord2f(0.0f, 0.5f);
  glVertex3f(-0.5f, 0.5f, 0.5f);

  /// 后侧面
  glNormal3f(0.0f, 0.0f, -0.5f); /** 指定法线背向观察者 */
  glTexCoord2f(0.0f, 0.0f);
  glVertex3f(-0.5f, -0.5f, -0.5f);
  glTexCoord2f(0.7f, 0.0f);
  glVertex3f(-0.5f, 0.5f, -0.5f);
  glTexCoord2f(0.7f, 0.7f);
  glVertex3f(0.5f, 0.5f, -0.5f);
  glTexCoord2f(0.0f, 0.7f);
  glVertex3f(0.5f, -0.5f, -0.5f);

  /// 顶面
  glNormal3f(0.0f, 0.5f, 0.0f); /**指定法线向上 */
  glTexCoord2f(0.0f, 0.0f);
  glVertex3f(-0.5f, 0.5f, 0.5f);
  glTexCoord2f(0.5f, 0.0f);
  glVertex3f(0.5f, 0.5f, 0.5f);
  glTexCoord2f(0.5f, 0.5f);
  glVertex3f(0.5f, 0.5f, -0.5f);
  glTexCoord2f(0.0f, 0.5f);
  glVertex3f(-0.5f, 0.5f, -0.5f);

  /// 底面
  glNormal3f(0.0f, -0.5f, 0.0f); /** 指定法线朝下 */
  glTexCoord2f(0.0f, 0.0f);
  glVertex3f(-0.5f, -0.5f, 0.5f);
  glTexCoord2f(0.5f, 0.0f);
  glVertex3f(0.5f, -0.5f, 0.5f);
  glTexCoord2f(0.5f, 0.5f);
  glVertex3f(0.5f, -0.5f, -0.5f);
  glTexCoord2f(0.0f, 0.5f);
  glVertex3f(-0.5f, -0.5f, -0.5f);

  /// 右侧面
  glNormal3f(0.5f, 0.0f, 0.0f); /**指定法线朝右 */
  glTexCoord2f(0.0f, 0.0f);
  glVertex3f(0.5f, -0.5f, -0.5f);
  glTexCoord2f(0.5f, 0.0f);
  glVertex3f(0.5f, 0.5f, -0.5f);
  glTexCoord2f(0.5f, 0.5f);
  glVertex3f(0.5f, 0.5f, 0.5f);
  glTexCoord2f(0.0f, 0.5f);
  glVertex3f(0.5f, -0.5f, 0.5f);

  /// 左侧面
  glNormal3f(-0.5f, 0.0f, 0.0f); /**指定法线朝左 */
  glTexCoord2f(0.0f, 0.0f);
  glVertex3f(-0.5f, -0.5f, -0.5f);
  glTexCoord2f(0.5f, 0.0f);
  glVertex3f(-0.5f, 0.5f, -0.5f);
  glTexCoord2f(0.5f, 0.5f);
  glVertex3f(-0.5f, 0.5f, 0.5f);
  glTexCoord2f(0.0f, 0.5f);
  glVertex3f(-0.5f, -0.5f, 0.5f);

  glEnd();
  glDisable(GL_TEXTURE_2D);
}

// 绘图回调函数
void display() {
  // 清除之前帧数据
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glLoadIdentity();
  glRotatef(ry, 0, 1, 0);
  glRotatef(rx, 1, 0, 0);
  // glPushMatrix();
  glTranslatef(0.0f, 0.0f, -1.9f);
  DrawBox();
  // glPopMatrix();

  // 执行绘图命令
  glFlush();
  angle++;
  // glutPostRedisplay();
}

// 窗口大小变化回调函数
void reshape(int w, int h) {
  glViewport(0, 0, w, h);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  // gluPerspective(60.0, (GLfloat)w/(GLfloat)h, 0.1, 100000.0);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  glutPostRedisplay();
}

int main(int argc, const char *argv[]) {
  // 初始化显示模式
  glutInit(&argc, const_cast<char **>(argv));
  glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB | GLUT_DEPTH);

  // 初始化窗口
  glutInitWindowSize(500, 500);
  glutInitWindowPosition(100, 100);
  glutCreateWindow(argv[0]);

  init();
  glutReshapeFunc(reshape);
  glutDisplayFunc(display);
  glutMouseFunc(mouse);
  /* 注册鼠标运动函数*/
  glutMotionFunc(motion);

  // 开始主循环绘制
  glutMainLoop();
  return 0;
}