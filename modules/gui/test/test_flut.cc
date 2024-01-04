#include <GL/glut.h>
#include <stdlib.h>

/* 初始化材料属性、光源属性、光照模型，打开深度缓冲区 */
void init(void)
{
    GLfloat mat_specular[] = {1.0, 1.0, 1.0, 1.0};
    GLfloat mat_shininess[] = {50.0};
    GLfloat light_position[] = {1.0, 1.0, 1.0, 0.0};
    glClearColor(0.0, 0.0, 0.0, 0.0);
    glShadeModel(GL_SMOOTH);
    glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);
    glMaterialfv(GL_FRONT, GL_SHININESS, mat_shininess);
    glLightfv(GL_LIGHT0, GL_POSITION, light_position);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_DEPTH_TEST);
}

/*调用GLUT函数，绘制一个球*/
void display(void)
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glutSolidSphere(1.0, 40, 50);
    glFlush();
}

int main(int argc, char** argv)
{
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
    init();
    /* 注册OpenGL绘图函数 */
    glutDisplayFunc(display);
    // /* 进入GLUT消息循环，开始执行程序 */
    glutMainLoop();
    return 0;
}