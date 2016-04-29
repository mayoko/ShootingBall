/**************************************--�d�l--***************************************
������
�}�[�J�uHiro�v�uSample1�v�uKanji�v��F�����ĕʂ̃I�u�W�F�N�g��\��������v���O����
�}�[�J�Ԃ̋����E�p�x�̍����R���\�[���Ō��邱�Ƃ��ł���B
*************************************************************************************/

#pragma warning(disable:4819)

#include "field.h"
#include "physSimu.h"
#include "Mat.h"
#include <windows.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <utility>
#include <map>
#include <chrono>
#include <iostream>
#include <cmath>
#include <fstream>

#define _USE_MATH_DEFINES	// math.h��M_PI���g������
#include <math.h>			// �p�x�v�Z�p

#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>

#include <AR/ar.h>
#include <AR/param.h>
#include <AR/video.h>
#include <AR/gsub.h>

// �O���[�o���ϐ�
/* �J�����\�� */
char *vconf_name  = "Data/WDM_camera_flipV.xml";	// �r�f�I�f�o�C�X�̐ݒ�t�@�C��
int  xsize;											// �E�B���h�E�T�C�Y
int  ysize;											// �E�B���h�E�T�C�Y
int  thresh = 70;									// 2�l����臒l
int  count = 0;										// �����t���[����
int startFlag = 0;                                  // �X�^�[�g�t���O
int winID[2];                                       // �E�B���h�E��ID
double gstartV;                                        // �������x
const int width = 1366;
const int height = 768;
Mat H_pc;
double H_pw[9];
bool Change = false;

// �t�B�[���h
Field gfield;

// �{�[��
physSimu gsimulator;

// ���Ԍv���J�n����
std::chrono::system_clock::time_point start;

/* �J�����p�����[�^ */
char *cparam_name = "Data/camera_para.dat";			// �J�����p�����[�^�t�@�C��
ARParam cparam;										// �J�����p�����[�^

/* �p�^�[���t�@�C�� */
/*! �p�^�[���t�@�C����Field�N���X�ɒ�`����Ă��鏇�Ԃɒ�`���邱�� !*/
#define MARK_NUM		5					// �g�p����}�[�J�[�̎��
//-----
#define MARK1_MARK_ID	1						// �}�[�J�[ID
#define MARK1_PATT_NAME	"Data\\patt.wall"		// �p�^�[���t�@�C����
#define MARK1_SIZE		60.0					// �p�^�[���̕��i40mm�j
//-----
#define MARK2_MARK_ID	2						// �}�[�J�[ID
#define MARK2_PATT_NAME	"Data\\patt.arrow "	// �p�^�[���t�@�C����
#define MARK2_SIZE		60.0					// �p�^�[���̕��i40mm�j
//-----
#define MARK3_MARK_ID	3						// �}�[�J�[ID
#define MARK3_PATT_NAME	"Data\\patt.start"		// �p�^�[���t�@�C����
#define MARK3_SIZE		60.0					// �p�^�[���̕��i40mm�j
//-----
#define MARK4_MARK_ID	4						// �}�[�J�[ID
#define MARK4_PATT_NAME	"Data\\patt.goal"		// �p�^�[���t�@�C����
#define MARK4_SIZE		60.0					// �p�^�[���̕��i40mm�j
//-----
#define MARK5_MARK_ID	5						// �}�[�J�[ID
#define MARK5_PATT_NAME	"Data\\patt.hole"		// �p�^�[���t�@�C����
#define MARK5_SIZE		60.0					// �p�^�[���̕��i40mm�j
//-----

typedef struct {
	char   *patt_name;			// �p�^�[���t�@�C��
	int    patt_id;				// �p�^�[����ID
	int    mark_id;				// �}�[�J�[ID
	int    visible;				// ���o�t���O
	double patt_width;			// �p�^�[���̃T�C�Y�i�P�ʁF�����j
	double patt_center[2];		// �p�^�[���̒��S���W
	double patt_trans[3][4];	// ���W�ϊ��s��
} MARK_T;
//-----
MARK_T   marker[MARK_NUM] = {
	{MARK1_PATT_NAME, -1, MARK1_MARK_ID, 0, MARK1_SIZE, {0.0, 0.0}},
	{MARK2_PATT_NAME, -1, MARK2_MARK_ID, 0, MARK2_SIZE, {0.0, 0.0}},
	{MARK3_PATT_NAME, -1, MARK3_MARK_ID, 0, MARK3_SIZE, {0.0, 0.0}},
	{MARK4_PATT_NAME, -1, MARK4_MARK_ID, 0, MARK4_SIZE, {0.0, 0.0}},
	{MARK5_PATT_NAME, -1, MARK5_MARK_ID, 0, MARK5_SIZE, {0.0, 0.0}}
};

// �v���g�^�C�v�錾
void Init(void);
void MainLoop(void);
void SetupLighting1(void);
void SetupLighting2(void);
void SetupMaterial1(void);
void SetupMaterial2(void);
void KeyEvent( unsigned char key, int x, int y );
void MouseEvent( int button, int state, int x, int y );
void Cleanup(void);
void DrawObject( int mark_id, double patt_trans[3][4] );
void homography(double& u, double& v);


//=======================================================
// main�֐�
//=======================================================
int main( int argc, char **argv )
{

	// GLUT�̏�����
	glutInit( &argc, argv );

	// AR�A�v���P�[�V�����̏�����
	Init();

	// �r�f�I�L���v�`���̊J�n
	arVideoCapStart();

	// ���C�����[�v�̊J�n
	argMainLoop( MouseEvent, KeyEvent, MainLoop );

	return 0;
}

void convert(double X, double Y) {
	Mat vec(3, 1);
	vec.mat[0][0] = X;
	vec.mat[1][0] = Y;
	vec.mat[2][0] = 1;
	vec = mul(H_pc, vec);
	glVertex2d(vec.mat[0][0], vec.mat[1][0]);
}

void homography(double &u, double &v)
{ 
	double w[3] = {u,v,1};
	u = ((H_pw[0]*w[0])+(H_pw[1]*w[1])+(H_pw[2]*1))/((H_pw[6]*w[0])+(H_pw[7]*w[1])+(H_pw[8]*1));
	v = ((H_pw[3]*w[0])+(H_pw[4]*w[1])+(H_pw[5]*1))/((H_pw[6]*w[0])+(H_pw[7]*w[1])+(H_pw[8]*1));
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
		glutReshapeWindow(1366,768);
	}
}

void display(void)
{
	//glViewport(0, 0, width, height);
	//GLfloat color[4] = {0.0, 0.8, 0.7, 1.0};//���̐F�w��
	//glClearColor(1.0, 1.0, 1.0, 1.0); //�w�i�̐F�w��
	//glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	//glEnable(GL_DEPTH_TEST);
	//glEnable(GL_LIGHTING);
	//glEnable(GL_LIGHT0);

	////glViewport(-640,0,1280,800);
	////glLoadIdentity();
	////gluPerspective( 151.927 , 1280/800 ,0.01 , 100);
	////gluLookAt(0.0, 0.0, 100.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);//���_
	//glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, color);

	////u = 10.0 * count; v = 10.0*800.0/1280.0*count;
	//double u = real(gsimulator.circle.p);
	//double v = imag(gsimulator.circle.p);

	//homography(u, v);
	//std::cout << u << " " << v << std::endl;
	//double x = (u/640.0-1.0)/*(1280.0/800.0)*/;
	//double y = (v/400.0-1.0)*(-1.0);
	//glLoadIdentity();
	//glTranslated(x, y, 0);

	//glutSolidSphere((gsimulator.circle.r/800.0), 20, 20);//���̕`��

	//glDisable(GL_DEPTH_TEST);
	//glDisable(GL_LIGHTING);
	//glDisable(GL_LIGHT0);

	//glutSwapBuffers();

	// ��������������������������������������������������������������������������
	glViewport(0, 0, width, height);
	//GLfloat color[4] = {0.0, 0.8, 0.7, 1.0};//���̐F�w��
	if (gsimulator.ballIsMoving) glClearColor(1.0, 1.0, 1.0, 0); // �w�i�F
	else if (gsimulator.ballIsClear) glClearColor(0, 1.0, 1.0, 0);
	else if (gsimulator.ballIsOver) glClearColor(0, 0, 0, 0);
	else glClearColor(1.0, 1.0, 1.0, 0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	//const double x = real(gsimulator.circle.p)/782*2-1.;
	//const double y = imag(gsimulator.circle.p)/530*2-1.;
	
	// world
	double x = real(gsimulator.circle.p);
	double y = imag(gsimulator.circle.p);
	// world -> projector �ɕϊ�
	homography(x, y);
	//x = x/640. - 1.;
	//y = y/400. - 1.;
	x=x/678. -1;
	y=y/384. -1;
	std::cout<<"x="<<x<<","<<"y="<<y<<std::endl;



	glDisable(GL_TEXTURE_2D);
	glLoadIdentity();
	glColor3d(1.0, 0, 0);
	glPointSize(15);
	if (gsimulator.ballIsMoving) {
		glBegin(GL_POLYGON);
		for (int i = 0; i < 32; i++) {
			//double X = gsimulator.circle.r/1280*2*std::cos(2*M_PI*i/32);
			//double Y = 1-(gsimulator.circle.r/800*2*std::sin(2*M_PI*i/32));
			double X = x+gsimulator.circle.r/1366*2*std::cos(2*M_PI*i/32);
			double Y = -(y+gsimulator.circle.r/768*2*std::sin(2*M_PI*i/32));
			//convert(X, Y);
			glVertex2d(X, Y);
		}
		glEnd();

	}
	//for (Field::Board board : gfield.boards) {
	//	switch(board.id) {
	//	case Field::Board::OBSTACLE:
	//		// ��
	//		glColor3d(0, 1.0, 0);
	//		break;
	//	case Field::Board::CHANGE_DIRECTION:
	//		// ��
	//		glColor3d(0, 0, 1.0);
	//		break;
	//	case Field::Board::START:
	//		// ���F
	//		glColor3d(1.0, 1.0, 0);
	//		break;
	//	case Field::Board::HOLE:
	//		// ��
	//		glColor3d(1.0, 0, 1.0);
	//		break;
	//	case Field::Board::GOAL:
	//		// ���F
	//		glColor3d(0, 1.0, 1.0);
	//		break;
	//	default:
	//		std::cout << "unko" << std::endl;
	//		break;
	//	}
	//	glBegin(GL_POLYGON);
	//	for (int i = 0; i < 4; i++) {
	//		//double X = real(board.position[i])/782*2-1;
	//		//double Y = -(imag(board.position[i])/530*2-1);
	//		double X = real(board.position[i]);
	//		double Y = (imag(board.position[i]));
	//		homography(X, Y);
	//		//convert(X, Y);
	//		glVertex2d(X/640. - 1., -(Y/400. - 1.));
	//	}
	//	glEnd();
	//}
	glFlush();
}



void reshape(int w, int h)
{
	glMatrixMode(GL_PROJECTION);
	glViewport(0,0,w,h);
	glLoadIdentity();
	double a = atan(1.0/100.0) * 360.0 /(2.0*3.141592)*2.0;
	//gluPerspective( a , 1280.0/800.0 ,0.01 , 200);
	gluLookAt(0.0, 0.0, 100.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);//���_
	glMatrixMode(GL_MODELVIEW);

	//glViewport(0, 0, (GLsizei)w, (GLsizei)h);
	//glMatrixMode(GL_PROJECTION);
	//glLoadIdentity();
	////gluPerspective(30.0, (double)w / (double)h, 1.0, 800.0);
	//glMatrixMode(GL_MODELVIEW);
	//glLoadIdentity();
}

//=======================================================
// �������֐�
//=======================================================
void Init(void)
{
	ARParam wparam;		// �J�����p�����[�^

	// �r�f�I�f�o�C�X�̐ݒ�
	if( arVideoOpen( vconf_name ) < 0 ){
		printf("�r�f�I�f�o�C�X�̃G���[\n");
		while(1);	
		exit(0);
	}

	// �E�B���h�E�T�C�Y�̎擾
	if( arVideoInqSize( &xsize, &ysize ) < 0 ) exit(0);
	printf("Image size (x,y) = (%d,$d)\n", xsize, ysize);

	// �J�����p�����[�^�̐ݒ�
	if( arParamLoad( cparam_name, 1, &wparam ) < 0 ){
		printf("�J�����p�����[�^�̓ǂݍ��݂Ɏ��s���܂���\n");
		while(1);
		exit(0);
	}

	// �J�����p�����[�^�̃T�C�Y����
	arParamChangeSize( &wparam, xsize, ysize, &cparam );
	// �J�����p�����[�^�̏�����
	arInitCparam( &cparam );
	printf("*** Camera Parameter ***\n");
	arParamDisp( &cparam );

	// �p�^�[���t�@�C���̃��[�h
	for( int i=0; i<Field::Board::EFFECT_NUM; i++ ){
		if( (marker[i].patt_id = arLoadPatt(marker[i].patt_name)) < 0){
			printf("�p�^�[���t�@�C���̓ǂݍ��݂Ɏ��s���܂���\n");
			printf("%s\n", marker[i].patt_name);
			exit(0);
		}
		gfield.trans.insert(std::pair<int, int>(marker[i].patt_id, i));
	}

	// gsub���C�u�����̏�����
	argInit( &cparam, 1.0, 0, 0, 0, 0 );

	// �E�B���h�E�^�C�g���̐ݒ�
	glutSetWindowTitle("ARTK_basic");
	//glutInitWindowPosition(1366, 0);
	winID[0] = glutGetWindow();
	glutInitDisplayMode(GLUT_RGBA| GLUT_DOUBLE|GLUT_DEPTH);
	glutInitWindowSize(width, height);
	winID[1] = glutCreateWindow("ojisan");
	glutSetWindow(winID[1]);
	glutInitWindowPosition (1366, 0);
	glutDisplayFunc(display);
	glutReshapeFunc(reshape);
	glutKeyboardFunc(KeyEvent);
	glutSetWindow(winID[0]);

	// H_pc�s��̏�����
	H_pc.resize(3, 3);
	std::ifstream ifs("Data/H_pc.txt");
	for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) ifs >> H_pc.mat[i][j];
	std::cout << "initialize H_pc mat" << std::endl;
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			std::cout << H_pc.mat[i][j] << " ";
		}
		std::cout << std::endl;
	}
	std::ifstream ifs2("Data/H_pw.txt");
	for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) ifs2 >> H_pw[3*i+j];
	std::cout << "initialize H_pw mat" << std::endl;
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			std::cout << H_pw[3*i+j] << " ";
		}
		std::cout << std::endl;
	}
}


//=======================================================
// ���C�����[�v�֐�
//=======================================================
void MainLoop(void)
{
	ARUint8          *image;			// �J�����L���v�`���摜
	ARMarkerInfo     *marker_info;		// �}�[�J���
	int              marker_num;		// ���o���ꂽ�}�[�J�̐�

	// �t�B�[���h�̏�����
	gfield.clear();
	// �J�����摜�̎擾
	if( (image = (ARUint8 *)arVideoGetImage()) == NULL ){
		arUtilSleep( 2 );
		return;
	}
	if( count == 0 ) arUtilTimerReset();
	count++;

	// �J�����摜�̕`��
	argDrawMode2D();
	argDispImage( image, 0, 0 );
	// �}�[�J�̌��o�ƔF��
	if( arDetectMarker( image, thresh, &marker_info, &marker_num ) < 0 ){
		Cleanup();
		exit(0);
	}
	gfield.receiveData(marker_num, marker_info);
	if (startFlag == 1) {
		std::cout << "simulate start" << std::endl;
		gsimulator.shootBall(gfield, gstartV);
		start = std::chrono::system_clock::now();
		startFlag = 2;
	}
	if (gsimulator.ballIsMoving) {
		double t;
		auto now = std::chrono::system_clock::now();
		auto elapsed = std::chrono::duration_cast< std::chrono::milliseconds >(now-start);
		t = elapsed.count() / 1000.0;
		std::cout << "now time is " << t << std::endl;
		gsimulator.simulate(gfield, t);
		gsimulator.print();
	}

	// ���̉摜�̃L���v�`���w��
	arVideoCapNext();

	// 3D�I�u�W�F�N�g��`�悷�邽�߂̏���
	argDrawMode3D();
	argDraw3dCamera( 0, 0 );
	glClearDepth(1.0);					// �f�v�X�o�b�t�@�̏����l
	glClear( GL_DEPTH_BUFFER_BIT );		// �f�v�X�o�b�t�@�̏�����

	//���̂܂܂ł͓��핡���}�[�J�[�ɕ`��ł��Ȃ��̂Œ���

	// �}�[�J�̈�v�x�̔�r
	//for( i=0; i<marker_num; i++ ){
	//	k = -1;
	//	for( j=0; j<marker_num; j++ ){
	//		if( marker[i].patt_id == marker_info[j].id ){
	//			if( k == -1 ) k = j;
	//			else if( marker_info[k].cf < marker_info[j].cf ) k = j;
	//		}
	//	}

	//	// �}�[�J�[��������Ȃ������Ƃ�
	//	if( k == -1 ){
	//		marker[i].visible = 0;
	//		continue;
	//	}

	//	// ���W�ϊ��s����擾
	//	if( marker[i].visible == 0 ) {
	//		// 1�t���[�����g���ă}�[�J�̈ʒu�E�p���i���W�ϊ��s��j�̌v�Z
	//		arGetTransMat( &marker_info[k], marker[i].patt_center, marker[i].patt_width, marker[i].patt_trans );
	//	} else {
	//		// �O�̃t���[�����g���ă}�[�J�̈ʒu�E�p���i���W�ϊ��s��j�̌v�Z
	//		arGetTransMatCont( &marker_info[k], marker[i].patt_trans, marker[i].patt_center, marker[i].patt_width, marker[i].patt_trans );
	//	}
	//		marker[i].visible = 1;

	//		// 3D�I�u�W�F�N�g�̕`��
	//		//DrawObject( marker[i].mark_id, marker[i].patt_trans );
	//	}
	// �o�b�t�@�̓��e����ʂɕ\��
	argSwapBuffers();
	glutSetWindow(winID[1]);
	glutPostRedisplay();
	glutSetWindow(winID[0]);
}


//=======================================================
// 3D�I�u�W�F�N�g�̕`����s���֐�
//=======================================================
void DrawObject( int mark_id, double patt_trans[3][4] )
{
	double gl_para[16];	// ARToolKit->OpenGL�ϊ��s��

	// �A�ʏ���
	glEnable( GL_DEPTH_TEST );			// �A�ʏ����E�L��
	glDepthFunc( GL_LEQUAL );			// �f�v�X�e�X�g

	// �ϊ��s��̓K�p
	argConvGlpara( patt_trans, gl_para );	// ARToolKit����OpenGL�̍s��ɕϊ�
	glMatrixMode( GL_MODELVIEW );			// �s��ϊ����[�h�E���f���r���[
	glLoadMatrixd( gl_para );				// �ǂݍ��ލs����w��

	switch( mark_id ){
	case MARK1_MARK_ID:
		// ���C�e�B���O
		SetupLighting1();			// ���C�g�̒�`
		glEnable( GL_LIGHTING );	// ���C�e�B���O�E�L��
		glEnable( GL_LIGHT0 );		// ���C�g0�E�I��
		// �I�u�W�F�N�g�̍ގ�
		SetupMaterial1();

		// 3D�I�u�W�F�N�g�̕`��
		glTranslatef( 0.0, 0.0, 25.0 );	// �}�[�J�̏�ɍڂ��邽�߂�Z�����i�}�[�J����j��25.0[mm]�ړ�
		glutSolidCube( 150.0 );			// �\���b�h�L���[�u��`��i1�ӂ̃T�C�Y50[mm]�j
		break;

	case MARK2_MARK_ID:
		// ���C�e�B���O
		SetupLighting2();			// ���C�g�̒�`
		glEnable( GL_LIGHTING );	// ���C�e�B���O�E�L��
		glEnable( GL_LIGHT0 );		// ���C�g0�E�I��
		// �I�u�W�F�N�g�̍ގ�
		SetupMaterial2();

		// 3D�I�u�W�F�N�g�̕`��
		glTranslatef( 0.0, 0.0, 25.0 );		// �}�[�J�̏�ɍڂ��邽�߂�Z�����i�}�[�J����j��25.0[mm]�ړ�
		glutSolidSphere( 50.0, 10, 10 );	// �\���b�h�X�t�B�A��`��i1�ӂ̃T�C�Y50[mm]�j
		break;

	case MARK3_MARK_ID:
		// ���C�e�B���O
		SetupLighting1();			// ���C�g�̒�`
		glEnable( GL_LIGHTING );	// ���C�e�B���O�E�L��
		glEnable( GL_LIGHT0 );		// ���C�g0�E�I��
		// �I�u�W�F�N�g�̍ގ�
		SetupMaterial2();

		// 3D�I�u�W�F�N�g�̕`��
		glTranslatef( 0.0, 0.0, 25.0 );	// �}�[�J�̏�ɍڂ��邽�߂�Z�����i�}�[�J����j��25.0[mm]�ړ�
		glRotated( 90, 1.0, 0.0, 0.0);	// �e�B�[�|�b�g���}�[�J��ɍڂ��邽�߂�90����]
		glutSolidTeapot( 50.0 );		// �\���b�h�e�B�[�|�b�g��`��i�T�C�Y50[mm]�j
		break;
	}


	// �I������
	glDisable( GL_LIGHTING );		// ���C�e�B���O�E����
	glDisable( GL_DEPTH_TEST );		// �f�v�X�e�X�g�E����
}


//=======================================================
// ���C�e�B���O
//=======================================================
void SetupLighting1(void)
{
	// ���C�g�̒�`
	GLfloat lt0_position[] = {100.0, -200.0, 200.0, 0.0};	// ���C�g0�̈ʒu
	GLfloat lt0_ambient[]  = {0.1, 0.1, 0.1, 1.0};			// �@�@�@�@ ����
	GLfloat lt0_diffuse[]  = {0.8, 0.8, 0.8, 1.0};			// �@�@�@�@ �g�U��

	// ���C�g�̐ݒ�
	glLightfv( GL_LIGHT0, GL_POSITION, lt0_position );
	glLightfv( GL_LIGHT0, GL_AMBIENT, lt0_ambient );
	glLightfv( GL_LIGHT0, GL_DIFFUSE, lt0_diffuse );
}

void SetupLighting2(void)
{
	// ���C�g�̒�`
	GLfloat lt0_position[] = {100.0, 200.0, 200.0, 0.0};	// ���C�g0�̈ʒu
	GLfloat lt0_ambient[]  = {0.2, 0.2, 0.2, 1.0};			// �@�@�@�@ ����
	GLfloat lt0_diffuse[]  = {0.8, 0.8, 0.8, 1.0};			// �@�@�@�@ �g�U��

	// ���C�g�̐ݒ�
	glLightfv( GL_LIGHT0, GL_POSITION, lt0_position );
	glLightfv( GL_LIGHT0, GL_AMBIENT, lt0_ambient );
	glLightfv( GL_LIGHT0, GL_DIFFUSE, lt0_diffuse );
}


//=======================================================
// �}�e���A���̐ݒ�
//=======================================================
void SetupMaterial1(void)
{
	// �I�u�W�F�N�g�̍ގ�
	GLfloat mat_ambient[] = {0.0, 1.0, 1.0, 1.0};	// �ގ��̊���
	GLfloat mat_specular[] = {0.0, 0.0, 1.0, 1.0};	// ���ʌ�
	GLfloat mat_shininess[] = {50.0};				// ���ʌW��

	// �}�e���A���̐ݒ�
	glMaterialfv(GL_FRONT, GL_AMBIENT, mat_ambient);
	glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);
	glMaterialfv(GL_FRONT, GL_SHININESS, mat_shininess);
}

void SetupMaterial2(void)
{
	// �I�u�W�F�N�g�̍ގ�
	GLfloat mat_ambient[] = {0.0, 0.0, 1.0, 1.0};	// �ގ��̊���
	GLfloat mat_specular[] = {0.0, 0.0, 1.0, 1.0};	// ���ʌ�
	GLfloat mat_shininess[] = {50.0};				// ���ʌW��

	// �}�e���A���̐ݒ�
	glMaterialfv(GL_FRONT, GL_AMBIENT, mat_ambient);
	glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);
	glMaterialfv(GL_FRONT, GL_SHININESS, mat_shininess);
}


//=======================================================
// �L�[�{�[�h���͏����֐�
//=======================================================
void KeyEvent( unsigned char key, int x, int y )
{
	//Enter�L�[����͂�����{�[���𔭎�
	if (48 <= key && key <= 57) {
		gstartV = (key - 48) * 30.;
	}
	if(key == 0x0D /*&& !gsimulator.ballIsMoving*/){
		startFlag = 1;
	}else if (key == 0x1b ){// ESC�L�[����͂�����A�v���P�[�V�����I��
		printf("*** %f (frame/sec)\n", (double)count/arUtilTimer());
		Cleanup();
		exit(0);
	}
	if(key == ' '){//�X�y�[�X�L�[�ŃE�C���h�E���[�h��؂�ւ�
		if(Change == false){Change = true;}
		else{Change = false;}
		fullscreen();
	}
}


//=======================================================
// �}�E�X���͏����֐�
//=======================================================
void MouseEvent( int button, int state, int x, int y )
{
	// ���͏�Ԃ�\��
	printf("�{�^���F%d ��ԁF%d ���W�F(x,y)=(%d,%d) \n", button, state, x, y );
}


//=======================================================
// �I�������֐�
//=======================================================
void Cleanup(void)
{
	arVideoCapStop();	// �r�f�I�L���v�`���̒�~
	arVideoClose();		// �r�f�I�f�o�C�X�̏I��
	argCleanup();		// ARToolKit�̏I������
}