#ifdef WIN32
#include <FL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#endif

#include "Render_API.h"
#include "glut.h"
#include "glui.h" 
#include "CBmp.h"

void setColor(const int color)
{
    switch(color){
    case 0:
        glColor3f(1., .2, .2);
        break;
    case 1:
        glColor3f(.2, 1., .2);
        break;
    case 2:
        glColor3f(.2, .2, 1.);
        break;
    case 3:
        glColor4f(1., 1., 0.1, 0.9);
        break;
    case 4:
        glColor4f(1.0, 0.54, .1, 0.9);
        break;
    }
}

void setColor3d(const unsigned char red, const unsigned char green, const unsigned char blue)
{
    double fRed = static_cast<double>(red)/255.0;
    double fGreen = static_cast<double>(green)/255.0;
    double fBlue = static_cast<double>(blue)/255.0;
    glColor3f(fRed, fGreen, fBlue);
}
    
void setColor3f(const double red, const double green, const double blue)
{
    glColor3f(red, green, blue);
}

void setColor3fv(const Vector3d color)
{
    glColor3dv(color.val);
}

void setColor4f(const double red, const double green, const double blue, const double alpha)
{
    glEnable(GL_BLEND);
    glColor4f(red, green, blue, alpha);
    glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
}


void setTranslate(const Vector3d &transVec)
{
    glTranslatef(transVec.x, transVec.y, transVec.z);
}

void setRotation(const double rotAngle, const Vector3d &rotAxis)
{
    glRotatef(rotAngle, rotAxis.x, rotAxis.y, rotAxis.z);
}

void drawPoint(const Vector3d &pos, double size)
{
    glDisable(GL_LIGHTING);
    glPushMatrix();
    glPointSize(size);
    glBegin(GL_POINTS);
    glVertex3dv(pos.val);
    glEnd();
    glPopMatrix();
    glEnable(GL_LIGHTING);
}

void drawLine(const Vector3d &startPoint, const Vector3d &endPoint)
{
    glDisable(GL_LIGHTING);
	glLineWidth(0.1);
	glColor3f(0.8, 0.2, 0.3);
    glBegin(GL_LINES);
    glVertex3f(startPoint.x, startPoint.y, startPoint.z);		// start
    glVertex3f(endPoint.x, endPoint.y, endPoint.z);		        // end
    glEnd();
    glEnable(GL_LIGHTING);
}


void drawCylinder(const Vector3d &startPoint, const Vector3d &endPoint, double radius)
{
    //the same quadric can be re-used for drawing many cylinders
    GLUquadricObj *quadric = gluNewQuadric();
    gluQuadricNormals(quadric, GLU_SMOOTH);
    int subdivisions = 18;

    Vector3d cyl_vec;
    cyl_vec = endPoint - startPoint;
    double vx = cyl_vec.x;
    double vy = cyl_vec.y;
    double vz = cyl_vec.z;
    double ax,rx,ry,rz;
    double cyl_len = cyl_vec.Length();

    glPushMatrix();

    glTranslatef(startPoint.x, startPoint.y, startPoint.z);
    if (fabs(vz) < 0.0001){
        glRotatef(90, 0, 1, 0);
        ax = 57.2957795*-atan( vy / vx );
        if (vx < 0){
            ax = ax + 180;
        }
        rx = 1;
        ry = 0;
        rz = 0;
    }
    else{
        ax = 57.2957795*acos( vz/ cyl_len );
        if (vz < 0.0)
            ax = -ax;
        rx = -vy*vz;
        ry = vx*vz;
        rz = 0;
    }
    glRotatef(ax, rx, ry, rz);
    gluQuadricOrientation(quadric,GLU_OUTSIDE);

	gluQuadricDrawStyle(quadric, GLU_FILL);
	gluQuadricTexture(quadric, GL_TRUE);

	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, 16);

    gluCylinder(quadric, radius, radius, cyl_len, subdivisions, 1);

	glDisable(GL_TEXTURE_2D);
    
    glPopMatrix();

    gluDeleteQuadric(quadric);
}


//RCModified
/*
void loadTexture()
{
	//glEnable(GL_DEPTH_TEST);
	//glEnable(GL_LIGHTING);
	//glEnable(GL_LIGHT0);
	//glShadeModel(GL_SMOOTH);
	//glEnable(GL_COLOR_MATERIAL);
	//glDisable(GL_CULL_FACE);
	//glEnable(GL_TEXTURE_2D);

	CBmp TextureImage;
	TextureImage.load("Texture/wood.bmp");
	
	glEnable(GL_TEXTURE_2D);
	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
	glGenTextures(1, &myTextureId[0]);
	glBindTexture(GL_TEXTURE_2D, myTextureId[0]);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, TextureImage.w, TextureImage.h, 0, GL_RGB, GL_UNSIGNED_BYTE, TextureImage.rgb);
	glBindTexture(GL_TEXTURE_2D, 0);
	glDisable(GL_TEXTURE_2D);

}
*/

void drawBall(const Vector3d &ballPos, double radius, double angle, Vector3d axis)
{
	glPushMatrix();
	GLUquadricObj *sphere = gluNewQuadric();
	gluQuadricDrawStyle(sphere, GLU_FILL);
	gluQuadricTexture(sphere, GL_TRUE);
	gluQuadricNormals(sphere, GLU_SMOOTH);
	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, 15);
	glTranslatef(ballPos.x, ballPos.y, ballPos.z);    // ²yªº¦ì¸m

	setRotation(angle, axis);

	gluSphere(sphere, radius, 20, 20);
	gluDeleteQuadric(sphere);
	glDisable(GL_TEXTURE_2D);
	glPopMatrix();
}

void drawSolidBall(const Vector3d &ballPos, double radius)
{
    glPushMatrix();
    glTranslatef(ballPos.x, ballPos.y, ballPos.z);
	glutSolidSphere(radius, 50, 50);
    glPopMatrix();
}
