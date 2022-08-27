#include "pch.h"
#include "Client.h"
#include "Renderer.h"

double GWidth = 1080;
double GHeight = 860;

RigidBody GBody;
GameTimer GGameTimer;

void initRigid()
{
    real mass = 8.0f;

    GBody.setAwake(true);
    GBody.setCanSleep(true);

    GBody.setPosition(Vector3(0.0f, 0.0f, 0.0f));
    GBody.setAcceleration(Vector3(0.0f, 0.0f, 0.0f));
    GBody.setDamping(0.95f, 0.80f);

    Vector3 halfSize(25.0f, 25.0f, 25.0f);
    Matrix3 inertiaTensor;
    inertiaTensor.setBlockInertiaTensor(halfSize, mass);
    GBody.setInertiaTensor(inertiaTensor);

    GBody.setMass(mass);
    GBody.setOrientation(Quaternion(1.0f, 0.0f, 0.0f, 0.0f));
    GBody.setRotation(Vector3(0.0f, 0.0f, 0.0f));
    GBody.setVelocity(Vector3(2.0f, 0.0f, 0.0f));


    Contacts contacts;
    contacts.calculateContactBasis();
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

    GLfloat matrix[16] = {};
    GBody.getOTransform(matrix);
    glScalef(50.0f, 50.0f, 50.0f);
    glMultMatrixf(matrix);

    glutSolidCube(1.0f);
    glPopMatrix();

    glutSwapBuffers();
    glFlush();
}

void idle()
{
    GGameTimer.Tick();
    real dt = GGameTimer.DeltaTime();

    GBody.integrate(dt);

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

    initRigid();
    GGameTimer.Reset();

    glutDisplayFunc(display);
    glutIdleFunc(idle);
    glutMainLoop();
    return 0;
}