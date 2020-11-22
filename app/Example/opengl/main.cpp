#include <math.h>
/* 由于头文件glut.h中已经包含了头文件gl.h和glu.h，所以只需要include 此文件*/
#include <GL/glut.h>


#define pi 3.1415926

bool mouseisdown=false;

bool loopr=false;

int mx,my;

int ry=30;

int rx=30;

/* 初始化材料属性、光源属性、光照模型，打开深度缓冲区 */
void init ( void )
{
    GLfloat mat_specular [ ] = { 1.0, 1.0, 1.0, 1.0 };
    GLfloat mat_shininess [ ] = { 50.0 };
    GLfloat light_position [ ] = { 1.0, 1.0, 1.0, 0.0 };
    glClearColor ( 0.0, 0.0, 0.0, 1.0 );
    glShadeModel ( GL_SMOOTH );
    glMaterialfv ( GL_FRONT, GL_SPECULAR, mat_specular);
    glMaterialfv ( GL_FRONT, GL_SHININESS, mat_shininess);
    glLightfv ( GL_LIGHT0, GL_POSITION, light_position);
    glEnable (GL_LIGHTING);
    glEnable (GL_LIGHT0);
    glEnable (GL_DEPTH_TEST);
}

/* 定时器函数 */
void timer(int p)
{
     ry-=5;
    //marks the current window as needing to be redisplayed.
        glutPostRedisplay();                 
     if (loopr)
        // re-start the timer, after 200ms, timer would be called
         glutTimerFunc(200,timer,0);
}

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

     if (button== GLUT_RIGHT_BUTTON)
         if(state == GLUT_DOWN)
         {loopr=true; glutTimerFunc(200,timer,0);}
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

/* 特殊按键监测函数 */
void special(int key, int x, int y)
{
    switch(key)
    {
    case GLUT_KEY_LEFT:
        ry-=5;
        glutPostRedisplay();
        break;
    case GLUT_KEY_RIGHT:
       ry+=5;
        glutPostRedisplay();
        break;
    case GLUT_KEY_UP:
        rx+=5;
        glutPostRedisplay();
        break;
    case GLUT_KEY_DOWN:
        rx-=5;
        glutPostRedisplay();
        break;
    }
}

/*渲染函数，调用GLUT函数，绘制图像*/
void display()
{
    float red[3]={1,0,0};
    float green[3]={0,1,0};
    float blue[3]={0,0,1};
    float yellow[3]={1,1,0};   

    glClearColor(1,1,1,1);
    glClear(GL_COLOR_BUFFER_BIT);
    glLoadIdentity();
    glRotatef(ry,0,1,0);      
    glRotatef(rx,1,0,0);
    glColor3fv(blue);
    glutWireTeapot(0.5);
    glFlush();
}

int main(int argc, char** argv)

{
    /* GLUT环境初始化*/
    glutInit (&argc, argv);
    /* 显示模式初始化 */
    glutInitDisplayMode (GLUT_SINGLE | GLUT_RGB | GLUT_DEPTH);
    /* 定义窗口大小 */
    glutInitWindowSize (300, 300);
    /* 定义窗口位置 */
    glutInitWindowPosition (100, 100);
    /* 显示窗口，窗口标题为执行函数名 */
    glutCreateWindow ( argv [ 0 ] );
    /* 调用OpenGL初始化函数 */
    init();

    /* 注册OpenGL绘图函数 */
    glutDisplayFunc (display);     
    /* 注册鼠标状态函数*/
     glutMouseFunc(mouse);       
    /* 注册鼠标运动函数*/
     glutMotionFunc(motion);
    /* 注册特殊按键函数*/
     glutSpecialFunc(special);
    // /* 进入GLUT消息循环，开始执行程序 */
    glutMainLoop();      

    return 0;

}