#include "pch.h"
#include "Client.h"
#include "Renderer.h"

double GWidth = 1080;
double GHeight = 860;

RigidBody GBody;

void initRigid()
{
    GBody.setAwake();
    GBody.setCanSleep(true);

    GBody.setPosition(Vector3(0.0f, 0.0f, 0.0f));
    GBody.setAcceleration(Vector3(0.5f, 0.0f, 0.0f));
}

void display()
{
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glClearDepth(1.0);
    glEnable(GL_DEPTH_TEST);
    glViewport(0, 0, (GLsizei)GWidth, (GLsizei)GHeight);


    // Æú¸®°ï ÄÃ·¯
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    glPushMatrix();
    glColor3f(1.0f, 0.0f, 1.0f);
    glTranslatef(0.0f, -100.0f, 0.0f);
    glScalef(50.0f, 50.0f, 50.0f);
    glBegin(GL_POLYGON);
    glVertex3d(-1.0, -1.0, -1.0);
    glVertex3d(-1.0, 1.0, -1.0);
    glVertex3d(1.0, 1.0, -1.0);
    glVertex3d(1.0, -1.0, -1.0);
    glEnd();
    glPopMatrix();

    glutSwapBuffers();
    glFlush();
}

void idle()
{
    // GBody.integrate()
    glutPostRedisplay();
}

int main(int argc, char** argv)
{
    glutInit(&argc, argv);
    glutInitWindowSize((int)GWidth, (int)GHeight);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
    glutCreateWindow("Dynamics3D");


    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(-GWidth / 2.0, GWidth / 2.0, -GHeight / 2.0, GHeight / 2.0, -1.0, 100.0);

    glutDisplayFunc(display);
    glutIdleFunc(idle);
    glutMainLoop();
    return 0;
}