#include <stdio.h>
#include <gl/glut.h>
#include <fstream>
#include <iostream>
#include <math.h>

double z=0.0;
double u=391.0,v=265.0;
bool Change=false;
double h[9];
int count =0;
void homography(double &u, double &v)
{ 
	double w[3] = {u,v,1};
	u = ((h[0]*w[0])+(h[1]*w[1])+(h[2]*1))/((h[6]*w[0])+(h[7]*w[1])+(h[8]*1));
	v = ((h[3]*w[0])+(h[4]*w[1])+(h[5]*1))/((h[6]*w[0])+(h[7]*w[1])+(h[8]*1));
}

void display(void)
{
	GLfloat color[4] = {0.0, 0.8, 0.7, 1.0};//球の色指定
	glClearColor(1.0, 1.0, 1.0, 1.0); //背景の色指定
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);

	//glViewport(-640,0,1280,800);
	//glLoadIdentity();
	//gluPerspective( 151.927 , 1280/800 ,0.01 , 100);
	//gluLookAt(0.0, 0.0, 100.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);//視点
	glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, color);

	//u = 10.0 * count; v = 10.0*800.0/1280.0*count;
	u=391.0;
	v=265.0;
	homography(u,v);
	//std::cout <<u << std::endl;
	double x = (u/640.0-1.0)/*(1280.0/800.0)*/;
	double y = (v/400.0-1.0)*(-1.0);
	glLoadIdentity();
	glTranslated(x,y, z);
	count++;

	glutSolidSphere(0.1, 20, 20);//球の描画

	glDisable(GL_DEPTH_TEST);
	glDisable(GL_LIGHTING);
	glDisable(GL_LIGHT0);

	glutSwapBuffers();
}

void reshape(int w, int h)
{
	glMatrixMode(GL_PROJECTION);
	glViewport(0,0,w,h);
	glLoadIdentity();
	double a = atan(1.0/100.0) * 360.0 /(2.0*3.141592)*2.0;
	gluPerspective( a , 1280.0/800.0 ,0.01 , 200);
	gluLookAt(0.0, 0.0, 100.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);//視点
	glMatrixMode(GL_MODELVIEW);
}

void timer(int value) {
	display();
	glutTimerFunc(50, timer, 0);
}

void fullscreen(){
	int nMode = 0;
	DEVMODE devMode;
	HWND hWnd;
	hWnd = GetActiveWindow();
	if(Change){
		glClearColor( 1.0f, 1.0f, 0.0f, 1.0f );
		ChangeDisplaySettings( &devMode, CDS_FULLSCREEN );
		glutFullScreen();
	}else{
		glClearColor( 0.0f, 1.0f, 0.0f, 1.0f );
		ChangeDisplaySettings(NULL, 0);
		glutPositionWindow(100,100);
		glutReshapeWindow(1280,800);
	}
}

void key(unsigned char key , int x , int y) {
	if(key == ' '){//スペースキーでウインドウモードを切り替え
		if(Change == false){Change = true;}
		else{Change = false;}
		fullscreen();
	}
}

int main(int argc, char** argv)
{
	std::ifstream ifs("H_wp_inv.txt");
	ifs >> h[0] ; ifs >> h[1] ; ifs >> h[2] ;
	ifs >> h[3] ; ifs >> h[4] ;	ifs >> h[5] ;
	ifs >> h[6] ; ifs >> h[7] ;	ifs >> h[8] ;
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			std::cout << h[3*i+j] << " " << h[3*i+j+1] << " " << h[3*i+j+2] << std::endl;
		}
	}

	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
	glutInitWindowSize (1280, 800); 
	glutInitWindowPosition (1366, 0);
	glutCreateWindow ("GLsample");
	glutDisplayFunc(display); 
	glutReshapeFunc(reshape);
	glutTimerFunc(100, timer, 9);
	glutKeyboardFunc(key);
	glutMainLoop();
	return 0;
}