/* * A tester to load mesh and skeleton into scene
 */

#include <GL/freeglut.h>
#include <iostream>
#include <fstream>
#include "../Pinocchio/mesh.h"
#include <fstream>
#include "../Pinocchio/vector.h"
#include "../Pinocchio/skeleton.h"
#include "../Pinocchio/utils.h"
#include "../Pinocchio/debugging.h"
#include "../Pinocchio/pinocchioApi.h"
#include "../Pinocchio/attachment.h"

#include <cstring>
#include "defMesh.h"
#include <Leap.h>
using namespace std;

static HumanSkeleton human;

//Leap Motion controller
Leap::Controller controller;
Leap::Frame frame;

//OpenGL framework
//
int width = 1024;
int height = 768;

//
///* Ortho (if used) */
double _left = 0.0;		/* ortho view volume params */
double _right = 0.0;
double _bottom = 0.0;
double _top = 0.0;
double _zNear = 0.1;
double _zFar = 10.0;
double fovy = 45.0;
double prev_z = 0;

//Model matrices
double _matrix[16];
double _matrixI[16];

/* Mouse Interface  */
int _mouseX = 0;		/* mouse control variables */
int _mouseY = 0;
bool _mouseLeft = false;
bool _mouseMiddle = false;
bool _mouseRight = false;

double _dragPosX = 0.0;
double _dragPosY = 0.0;
double _dragPosZ = 0.0;

double vlen(double x, double y, double z)
{
    return sqrt(x * x + y * y + z * z);
}

void invertMatrix(const GLdouble * m, GLdouble * out)
{

/* NB. OpenGL Matrices are COLUMN major. */
#define MAT(m,r,c) (m)[(c)*4+(r)]

/* Here's some shorthand converting standard (row,column) to index. */
#define m11 MAT(m,0,0)
#define m12 MAT(m,0,1)
#define m13 MAT(m,0,2)
#define m14 MAT(m,0,3)
#define m21 MAT(m,1,0)
#define m22 MAT(m,1,1)
#define m23 MAT(m,1,2)
#define m24 MAT(m,1,3)
#define m31 MAT(m,2,0)
#define m32 MAT(m,2,1)
#define m33 MAT(m,2,2)
#define m34 MAT(m,2,3)
#define m41 MAT(m,3,0)
#define m42 MAT(m,3,1)
#define m43 MAT(m,3,2)
#define m44 MAT(m,3,3)

    GLdouble det;
    GLdouble d12, d13, d23, d24, d34, d41;
    GLdouble tmp[16];		/* Allow out == in. */

    /* Inverse = adjoint / det. (See linear algebra texts.) */

    /* pre-compute 2x2 dets for last two rows when computing */
    /* cofactors of first two rows. */
    d12 = (m31 * m42 - m41 * m32);
    d13 = (m31 * m43 - m41 * m33);
    d23 = (m32 * m43 - m42 * m33);
    d24 = (m32 * m44 - m42 * m34);
    d34 = (m33 * m44 - m43 * m34);
    d41 = (m34 * m41 - m44 * m31);

    tmp[0] = (m22 * d34 - m23 * d24 + m24 * d23);
    tmp[1] = -(m21 * d34 + m23 * d41 + m24 * d13);
    tmp[2] = (m21 * d24 + m22 * d41 + m24 * d12);
    tmp[3] = -(m21 * d23 - m22 * d13 + m23 * d12);

    /* Compute determinant as early as possible using these cofactors. */
    det = m11 * tmp[0] + m12 * tmp[1] + m13 * tmp[2] + m14 * tmp[3];

    /* Run singularity test. */
    if (det == 0.0) {
	/* printf("invert_matrix: Warning: Singular matrix.\n"); */
/* 	  memcpy(out,_identity,16*sizeof(double)); */
    } else {
	GLdouble invDet = 1.0 / det;
	/* Compute rest of inverse. */
	tmp[0] *= invDet;
	tmp[1] *= invDet;
	tmp[2] *= invDet;
	tmp[3] *= invDet;

	tmp[4] = -(m12 * d34 - m13 * d24 + m14 * d23) * invDet;
	tmp[5] = (m11 * d34 + m13 * d41 + m14 * d13) * invDet;
	tmp[6] = -(m11 * d24 + m12 * d41 + m14 * d12) * invDet;
	tmp[7] = (m11 * d23 - m12 * d13 + m13 * d12) * invDet;

	/* Pre-compute 2x2 dets for first two rows when computing */
	/* cofactors of last two rows. */
	d12 = m11 * m22 - m21 * m12;
	d13 = m11 * m23 - m21 * m13;
	d23 = m12 * m23 - m22 * m13;
	d24 = m12 * m24 - m22 * m14;
	d34 = m13 * m24 - m23 * m14;
	d41 = m14 * m21 - m24 * m11;

	tmp[8] = (m42 * d34 - m43 * d24 + m44 * d23) * invDet;
	tmp[9] = -(m41 * d34 + m43 * d41 + m44 * d13) * invDet;
	tmp[10] = (m41 * d24 + m42 * d41 + m44 * d12) * invDet;
	tmp[11] = -(m41 * d23 - m42 * d13 + m43 * d12) * invDet;
	tmp[12] = -(m32 * d34 - m33 * d24 + m34 * d23) * invDet;
	tmp[13] = (m31 * d34 + m33 * d41 + m34 * d13) * invDet;
	tmp[14] = -(m31 * d24 + m32 * d41 + m34 * d12) * invDet;
	tmp[15] = (m31 * d23 - m32 * d13 + m33 * d12) * invDet;

	memcpy(out, tmp, 16 * sizeof(GLdouble));
    }

#undef m11
#undef m12
#undef m13
#undef m14
#undef m21
#undef m22
#undef m23
#undef m24
#undef m31
#undef m32
#undef m33
#undef m34
#undef m41
#undef m42
#undef m43
#undef m44
#undef MAT
}



void pos(double *px, double *py, double *pz, const int x, const int y,
	 const int *viewport)
{
    /*
       Use the ortho projection and viewport information
       to map from mouse co-ordinates back into world 
       co-ordinates
     */

    *px = (double) (x - viewport[0]) / (double) (viewport[2]);
    *py = (double) (y - viewport[1]) / (double) (viewport[3]);

    *px = _left + (*px) * (_right - _left);
    *py = _top + (*py) * (_bottom - _top);
    *pz = _zNear;
}

void getMatrix()
{
    glGetDoublev(GL_MODELVIEW_MATRIX, _matrix);
    invertMatrix(_matrix, _matrixI);
}



Mesh mesh;
DefMesh *myDefMesh;

double rotateY = 0.0;

void drawSkeleton(const vector<Vector3> v, Vector3 trans)
{
    glLineWidth(5);
    glColor3d(.5, 0, 0);

    const vector<int> &prev = human.fPrev();
    glBegin(GL_LINES);
    for(int j = 1; j < (int)prev.size(); ++j) {
        int k = prev[j];
        glVertex3d(v[j][0]+trans[0], v[j][1]+trans[1], v[j][2]+trans[2]);
        glVertex3d(v[k][0]+trans[0], v[k][1]+trans[1], v[k][2]+trans[2]);
    }
    glEnd();
    glLineWidth(0.1);

}
void drawMesh(const Mesh &m, bool flatShading, Vector3 trans)
{
    int i;
    Vector3 normal;

    glBegin(GL_TRIANGLES);
    for(i = 0; i < (int)m.edges.size(); ++i) {
        int v = m.edges[i].vertex;
        const Vector3 &p = m.vertices[v].pos;

        if(!flatShading) {
            normal = m.vertices[v].normal;
            glNormal3d(normal[0], normal[1], normal[2]);
        }
        else if(i % 3 == 0) {
            const Vector3 &p2 = m.vertices[m.edges[i + 1].vertex].pos;
            const Vector3 &p3 = m.vertices[m.edges[i + 2].vertex].pos;
        
            normal = ((p2 - p) % (p3 - p)).normalize();
            glNormal3d(normal[0], normal[1], normal[2]);
        }
    
        glVertex3d(p[0] + trans[0], p[1] + trans[1], p[2] + trans[2]);
    }
    glEnd();
}


void init()
{
    //OpenGL initialize functions goes here

    glutInitContextVersion(4, 2);
    glutInitContextProfile(GLUT_CORE_PROFILE);
    glutInitContextFlags(GLUT_DEBUG);

    std::cout<<"Vendor: "<<glGetString(GL_VENDOR)<<std::endl;
    std::cout<<"Version: "<<glGetString(GL_VERSION)<<std::endl;
    std::cout<<"GLSL: "<<glGetString(GL_SHADING_LANGUAGE_VERSION)<<std::endl;

      //Light values and coordinates
     GLfloat ambientLight[] = { 0.3f, 0.3f, 0.3f, 1.0f };
     GLfloat diffuseLight[] = { 0.7f, 0.7f, 0.7f, 1.0f };
     GLfloat lightPos[] = {20.0f, 20.0f, 50.0f};
     glEnable(GL_DEPTH_TEST);
     glFrontFace(GL_CCW);
     //glEnable(GL_CULL_FACE);
     glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
     // Hidden surface removal // Counterclockwise polygons face out // Do not calculate inside of jet // Enable lighting
     glEnable(GL_LIGHTING);
     // Set up and enable light 0
     glLightfv(GL_LIGHT0,GL_AMBIENT,ambientLight);
     glLightfv(GL_LIGHT0,GL_DIFFUSE,diffuseLight);
     glEnable(GL_LIGHT0);
     // Enable color tracking
     glEnable(GL_COLOR_MATERIAL);
     // Set material properties to follow glColor values
     glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);

     glClearColor(0.2f, 0.2f, 0.2f, 3.0f );
    
     //Rescale normals to unit length
     glEnable(GL_NORMALIZE);
     glLightfv(GL_LIGHT0,GL_POSITION,lightPos);

     glShadeModel(GL_FLAT);
     getMatrix(); //Init matrix

     //Fit the skeleton with mesh and calculate the weight
     //Mesh m("./armadillo.obj");
     mesh.normalizeBoundingBox();

     Skeleton given = HumanSkeleton();
     given.scale(0.7);
     PinocchioOutput o;
     o = autorig(given, mesh);

     if(o.embedding.size() == 0) {
        cout << "Error embedding" << endl;
        exit(0);
     }
     mesh.computeVertexNormals();

     myDefMesh = new DefMesh(mesh, given, o.embedding, *(o.attachment));   
}

void changeSize(int w, int h)
{
    //GLfloat aspectRatio;
    //if(h==0)
    //    h = 1;
    //glViewport(0, 0, w, h);
    //glMatrixMode(GL_PROJECTION);
    //glLoadIdentity();
    //aspectRatio = (GLfloat)w / (GLfloat)h;
    //gluPerspective(45.0f, aspectRatio, 1.0f, 900.0f);    //using perspective
    //
    //glMatrixMode(GL_MODELVIEW);
    //glLoadIdentity();

    glViewport(0, 0, w, h);


    _top = 1.0;
    _bottom = -1.0;
    _left = -(double) w / (double) h;
    _right = -_left;

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    /* glOrtho(_left,_right,_bottom,_top,_zNear,_zFar);  Ortho */
    gluPerspective(fovy, (double) w / (double) h, _zNear, _zFar);	/* PErspective for stereo */

    glMatrixMode(GL_MODELVIEW);
}

void timerFunction(int value)       
{
    //force it to refresh, no need of postRedisplay function any more
    if (controller.isConnected())
    {
        frame = controller.frame();
        const Leap::Hand hand = frame.hands()[0];
        const Leap::FingerList fingers = hand.fingers();
        //std::cout<<fingers.count()<<std::endl;
        if (fingers.count() <= 4)
            myDefMesh->dir.setCurDir(fingers);

    }
    //if (!frame.hands().isEmpty())
    //{
    //    const Leap::Hand hand = frame.hands()[0];
    //    const Leap::FingerList fingers = hand.fingers();

    //    if (!fingers.isEmpty() && fingers.count()==3)
    //    {
    //        glBegin(GL_LINES);
    //        for (int i=0; i<fingers.count(); i++){
    //            Leap::Vector dir = fingers[i].direction().normalized();
    //            cout<<dir<<std::endl;
    //            glVertex3d(0.0, 0.0, 0.0);
    //            glVertex3d(dir.x, dir.y, dir.z);

    //        }
    //        std::cout<<"=====\n";
    //        glEnd();

    //    }

    glutPostRedisplay();
    glutTimerFunc(10,timerFunction,1);
}

void handleKeyPress(int key, int x, int y)
{ 
    switch(key)
    {
                
    }
}

void mouseEvent(int button, int state, int x, int y)
{
    int viewport[4];

    _mouseX = x;
    _mouseY = y;

    if (state == GLUT_UP)
	switch (button) {
	case GLUT_LEFT_BUTTON:
	    _mouseLeft = false;
	    break;
	case GLUT_MIDDLE_BUTTON:
	    _mouseMiddle = false;
	    break;
	case GLUT_RIGHT_BUTTON:
	    _mouseRight = false;
	    break;
    } else
	switch (button) {
	case GLUT_LEFT_BUTTON:
	    _mouseLeft = true;
	    break;
	case GLUT_MIDDLE_BUTTON:
	    _mouseMiddle = true;
	    break;
	case GLUT_RIGHT_BUTTON:
	    _mouseRight = true;
	    break;
	}

    glGetIntegerv(GL_VIEWPORT, viewport);
    pos(&_dragPosX, &_dragPosY, &_dragPosZ, x, y, viewport);
}

void mouseMoveEvent(int x, int y)
{
    bool changed = false;

    const int dx = x - _mouseX;
    const int dy = y - _mouseY;

    int viewport[4];
    glGetIntegerv(GL_VIEWPORT, viewport);

    if (dx == 0 && dy == 0)
	return;

    if (_mouseMiddle || (_mouseLeft && _mouseRight)) {
	/* double s = exp((double)dy*0.01); */
	/* glScalef(s,s,s); */
	/* if(abs(prev_z) <= 1.0) */

	glLoadIdentity();
	glTranslatef(0, 0, dy * 0.01);
	glMultMatrixd(_matrix);



	changed = true;
    } else if (_mouseLeft) {
	double ax, ay, az;
	double bx, by, bz;
	double angle;

	ax = dy;
	ay = dx;
	az = 0.0;
	angle = vlen(ax, ay, az) / (double) (viewport[2] + 1) * 180.0;

	/* Use inverse matrix to determine local axis of rotation */

	bx = _matrixI[0] * ax + _matrixI[4] * ay + _matrixI[8] * az;
	by = _matrixI[1] * ax + _matrixI[5] * ay + _matrixI[9] * az;
	bz = _matrixI[2] * ax + _matrixI[6] * ay + _matrixI[10] * az;

	glRotatef(angle, bx, by, bz);

	changed = true;
    } else if (_mouseRight) {
	double px, py, pz;

	pos(&px, &py, &pz, x, y, viewport);

	glLoadIdentity();
	glTranslatef(px - _dragPosX, py - _dragPosY, pz - _dragPosZ);
	glMultMatrixd(_matrix);

	_dragPosX = px;
	_dragPosY = py;
	_dragPosZ = pz;

	changed = true;
    }

    _mouseX = x;
    _mouseY = y;

    if (changed) {
	getMatrix();
	glutPostRedisplay();
    }
}

void display()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();
    glMultMatrixd(_matrix);
    Vector3 vec(-myDefMesh->getSkel()[0]);
    myDefMesh->updateMesh();

    drawMesh(myDefMesh->getMesh(), false, vec);
    drawSkeleton(myDefMesh->getSkel(), vec);

    //Draw fingers
    glBegin(GL_LINES);
    for (int i=0; i<3; i++){
        Vector3 curDir = myDefMesh->dir.curDir[i];
        glVertex3d(0.0, 0.0, 0.0);
        glVertex3d(curDir[0], curDir[1], curDir[2]);
    }
    glEnd();

    glBegin(GL_LINES);
    glColor3d(0.0, 1.0, 0.0);
    for (int i=0; i<3; i++){
        Vector3 curDir = myDefMesh->dir.originDir[i];
        glVertex3d(0.0, 0.0, 0.0);
        glVertex3d(curDir[0], curDir[1], curDir[2]);
    }
    glEnd();


    glutSwapBuffers();
}
int main(int argc, char **argv)
{
    //Init Opengl
    std::string fileName = argv[1];
    mesh.loadObj(fileName);
    glutInit(&argc, argv);
    //Print contex info
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);	//double buffer
    glutInitWindowSize(width, height);
    glutInitWindowPosition(0, 0);
    glutCreateWindow("Leap Deform");
    glutDisplayFunc(display);
    glutReshapeFunc(changeSize);
    glutTimerFunc(10, timerFunction, 1);

    glutMouseFunc(mouseEvent);
    glutMotionFunc(mouseMoveEvent);
    glutSpecialFunc(handleKeyPress);
    
 
    init();
    glutMainLoop();
    std::cout<<"Running in two threads\n";
    delete myDefMesh;
    return 0;
}

