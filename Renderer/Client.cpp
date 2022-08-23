#include "pch.h"
#include "Client.h"
#include "Renderer.h"

void display() {

    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    // ������ �÷�
    glColor3f(1.0f, 0.0f, 1.0f);


    // �������� �׷��ִ� ��
    glBegin(GL_POLYGON);
    glVertex3f(-0.3f, -0.3f, 0.0f);
    glVertex3f(0.3f, -0.3f, 0.0f);
    glVertex3f(0.3f, 0.3f, 0.0f);
    glVertex3f(-0.3f, 0.3f, 0.0f);


    glEnd();
    glFinish();
}

int main(int argc, char** argv)

{
    glutInit(&argc, argv);
    glutCreateWindow("Hello OpenGL");

    glutDisplayFunc(display);
    glutMainLoop();
    return 0;
}