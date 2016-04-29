/**************************************--仕様--***************************************
■説明
マーカ「Hiro」「Sample1」「Kanji」を認識して別のオブジェクトを表示させるプログラム
マーカ間の距離・角度の差をコンソールで見ることができる。
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

#define _USE_MATH_DEFINES	// math.hのM_PIを使うため
#include <math.h>			// 角度計算用

#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>

#include <AR/ar.h>
#include <AR/param.h>
#include <AR/video.h>
#include <AR/gsub.h>

// グローバル変数
/* カメラ構成 */
char *vconf_name  = "Data/WDM_camera_flipV.xml";	// ビデオデバイスの設定ファイル
int  xsize;											// ウィンドウサイズ
int  ysize;											// ウィンドウサイズ
int  thresh = 70;									// 2値化の閾値
int  count = 0;										// 処理フレーム数
int startFlag = 0;                                  // スタートフラグ
int winID[2];                                       // ウィンドウのID
double gstartV;                                        // 初期速度
const int width = 1366;
const int height = 768;
Mat H_pc;
double H_pw[9];
bool Change = false;

// フィールド
Field gfield;

// ボール
physSimu gsimulator;

// 時間計測開始時間
std::chrono::system_clock::time_point start;

/* カメラパラメータ */
char *cparam_name = "Data/camera_para.dat";			// カメラパラメータファイル
ARParam cparam;										// カメラパラメータ

/* パターンファイル */
/*! パターンファイルはFieldクラスに定義されている順番に定義すること !*/
#define MARK_NUM		5					// 使用するマーカーの種類
//-----
#define MARK1_MARK_ID	1						// マーカーID
#define MARK1_PATT_NAME	"Data\\patt.wall"		// パターンファイル名
#define MARK1_SIZE		60.0					// パターンの幅（40mm）
//-----
#define MARK2_MARK_ID	2						// マーカーID
#define MARK2_PATT_NAME	"Data\\patt.arrow "	// パターンファイル名
#define MARK2_SIZE		60.0					// パターンの幅（40mm）
//-----
#define MARK3_MARK_ID	3						// マーカーID
#define MARK3_PATT_NAME	"Data\\patt.start"		// パターンファイル名
#define MARK3_SIZE		60.0					// パターンの幅（40mm）
//-----
#define MARK4_MARK_ID	4						// マーカーID
#define MARK4_PATT_NAME	"Data\\patt.goal"		// パターンファイル名
#define MARK4_SIZE		60.0					// パターンの幅（40mm）
//-----
#define MARK5_MARK_ID	5						// マーカーID
#define MARK5_PATT_NAME	"Data\\patt.hole"		// パターンファイル名
#define MARK5_SIZE		60.0					// パターンの幅（40mm）
//-----

typedef struct {
	char   *patt_name;			// パターンファイル
	int    patt_id;				// パターンのID
	int    mark_id;				// マーカーID
	int    visible;				// 検出フラグ
	double patt_width;			// パターンのサイズ（単位：ｍｍ）
	double patt_center[2];		// パターンの中心座標
	double patt_trans[3][4];	// 座標変換行列
} MARK_T;
//-----
MARK_T   marker[MARK_NUM] = {
	{MARK1_PATT_NAME, -1, MARK1_MARK_ID, 0, MARK1_SIZE, {0.0, 0.0}},
	{MARK2_PATT_NAME, -1, MARK2_MARK_ID, 0, MARK2_SIZE, {0.0, 0.0}},
	{MARK3_PATT_NAME, -1, MARK3_MARK_ID, 0, MARK3_SIZE, {0.0, 0.0}},
	{MARK4_PATT_NAME, -1, MARK4_MARK_ID, 0, MARK4_SIZE, {0.0, 0.0}},
	{MARK5_PATT_NAME, -1, MARK5_MARK_ID, 0, MARK5_SIZE, {0.0, 0.0}}
};

// プロトタイプ宣言
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
// main関数
//=======================================================
int main( int argc, char **argv )
{

	// GLUTの初期化
	glutInit( &argc, argv );

	// ARアプリケーションの初期化
	Init();

	// ビデオキャプチャの開始
	arVideoCapStart();

	// メインループの開始
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
	//GLfloat color[4] = {0.0, 0.8, 0.7, 1.0};//球の色指定
	//glClearColor(1.0, 1.0, 1.0, 1.0); //背景の色指定
	//glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	//glEnable(GL_DEPTH_TEST);
	//glEnable(GL_LIGHTING);
	//glEnable(GL_LIGHT0);

	////glViewport(-640,0,1280,800);
	////glLoadIdentity();
	////gluPerspective( 151.927 , 1280/800 ,0.01 , 100);
	////gluLookAt(0.0, 0.0, 100.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);//視点
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

	//glutSolidSphere((gsimulator.circle.r/800.0), 20, 20);//球の描画

	//glDisable(GL_DEPTH_TEST);
	//glDisable(GL_LIGHTING);
	//glDisable(GL_LIGHT0);

	//glutSwapBuffers();

	// あああああああああああああああああああああああああああああああああああああ
	glViewport(0, 0, width, height);
	//GLfloat color[4] = {0.0, 0.8, 0.7, 1.0};//球の色指定
	if (gsimulator.ballIsMoving) glClearColor(1.0, 1.0, 1.0, 0); // 背景色
	else if (gsimulator.ballIsClear) glClearColor(0, 1.0, 1.0, 0);
	else if (gsimulator.ballIsOver) glClearColor(0, 0, 0, 0);
	else glClearColor(1.0, 1.0, 1.0, 0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	//const double x = real(gsimulator.circle.p)/782*2-1.;
	//const double y = imag(gsimulator.circle.p)/530*2-1.;
	
	// world
	double x = real(gsimulator.circle.p);
	double y = imag(gsimulator.circle.p);
	// world -> projector に変換
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
	//		// 緑
	//		glColor3d(0, 1.0, 0);
	//		break;
	//	case Field::Board::CHANGE_DIRECTION:
	//		// 青
	//		glColor3d(0, 0, 1.0);
	//		break;
	//	case Field::Board::START:
	//		// 黄色
	//		glColor3d(1.0, 1.0, 0);
	//		break;
	//	case Field::Board::HOLE:
	//		// 紫
	//		glColor3d(1.0, 0, 1.0);
	//		break;
	//	case Field::Board::GOAL:
	//		// 水色
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
	gluLookAt(0.0, 0.0, 100.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);//視点
	glMatrixMode(GL_MODELVIEW);

	//glViewport(0, 0, (GLsizei)w, (GLsizei)h);
	//glMatrixMode(GL_PROJECTION);
	//glLoadIdentity();
	////gluPerspective(30.0, (double)w / (double)h, 1.0, 800.0);
	//glMatrixMode(GL_MODELVIEW);
	//glLoadIdentity();
}

//=======================================================
// 初期化関数
//=======================================================
void Init(void)
{
	ARParam wparam;		// カメラパラメータ

	// ビデオデバイスの設定
	if( arVideoOpen( vconf_name ) < 0 ){
		printf("ビデオデバイスのエラー\n");
		while(1);	
		exit(0);
	}

	// ウィンドウサイズの取得
	if( arVideoInqSize( &xsize, &ysize ) < 0 ) exit(0);
	printf("Image size (x,y) = (%d,$d)\n", xsize, ysize);

	// カメラパラメータの設定
	if( arParamLoad( cparam_name, 1, &wparam ) < 0 ){
		printf("カメラパラメータの読み込みに失敗しました\n");
		while(1);
		exit(0);
	}

	// カメラパラメータのサイズ調整
	arParamChangeSize( &wparam, xsize, ysize, &cparam );
	// カメラパラメータの初期化
	arInitCparam( &cparam );
	printf("*** Camera Parameter ***\n");
	arParamDisp( &cparam );

	// パターンファイルのロード
	for( int i=0; i<Field::Board::EFFECT_NUM; i++ ){
		if( (marker[i].patt_id = arLoadPatt(marker[i].patt_name)) < 0){
			printf("パターンファイルの読み込みに失敗しました\n");
			printf("%s\n", marker[i].patt_name);
			exit(0);
		}
		gfield.trans.insert(std::pair<int, int>(marker[i].patt_id, i));
	}

	// gsubライブラリの初期化
	argInit( &cparam, 1.0, 0, 0, 0, 0 );

	// ウィンドウタイトルの設定
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

	// H_pc行列の初期化
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
// メインループ関数
//=======================================================
void MainLoop(void)
{
	ARUint8          *image;			// カメラキャプチャ画像
	ARMarkerInfo     *marker_info;		// マーカ情報
	int              marker_num;		// 検出されたマーカの数

	// フィールドの初期化
	gfield.clear();
	// カメラ画像の取得
	if( (image = (ARUint8 *)arVideoGetImage()) == NULL ){
		arUtilSleep( 2 );
		return;
	}
	if( count == 0 ) arUtilTimerReset();
	count++;

	// カメラ画像の描画
	argDrawMode2D();
	argDispImage( image, 0, 0 );
	// マーカの検出と認識
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

	// 次の画像のキャプチャ指示
	arVideoCapNext();

	// 3Dオブジェクトを描画するための準備
	argDrawMode3D();
	argDraw3dCamera( 0, 0 );
	glClearDepth(1.0);					// デプスバッファの消去値
	glClear( GL_DEPTH_BUFFER_BIT );		// デプスバッファの初期化

	//このままでは同種複数マーカーに描画できないので注意

	// マーカの一致度の比較
	//for( i=0; i<marker_num; i++ ){
	//	k = -1;
	//	for( j=0; j<marker_num; j++ ){
	//		if( marker[i].patt_id == marker_info[j].id ){
	//			if( k == -1 ) k = j;
	//			else if( marker_info[k].cf < marker_info[j].cf ) k = j;
	//		}
	//	}

	//	// マーカーが見つからなかったとき
	//	if( k == -1 ){
	//		marker[i].visible = 0;
	//		continue;
	//	}

	//	// 座標変換行列を取得
	//	if( marker[i].visible == 0 ) {
	//		// 1フレームを使ってマーカの位置・姿勢（座標変換行列）の計算
	//		arGetTransMat( &marker_info[k], marker[i].patt_center, marker[i].patt_width, marker[i].patt_trans );
	//	} else {
	//		// 前のフレームを使ってマーカの位置・姿勢（座標変換行列）の計算
	//		arGetTransMatCont( &marker_info[k], marker[i].patt_trans, marker[i].patt_center, marker[i].patt_width, marker[i].patt_trans );
	//	}
	//		marker[i].visible = 1;

	//		// 3Dオブジェクトの描画
	//		//DrawObject( marker[i].mark_id, marker[i].patt_trans );
	//	}
	// バッファの内容を画面に表示
	argSwapBuffers();
	glutSetWindow(winID[1]);
	glutPostRedisplay();
	glutSetWindow(winID[0]);
}


//=======================================================
// 3Dオブジェクトの描画を行う関数
//=======================================================
void DrawObject( int mark_id, double patt_trans[3][4] )
{
	double gl_para[16];	// ARToolKit->OpenGL変換行列

	// 陰面消去
	glEnable( GL_DEPTH_TEST );			// 陰面消去・有効
	glDepthFunc( GL_LEQUAL );			// デプステスト

	// 変換行列の適用
	argConvGlpara( patt_trans, gl_para );	// ARToolKitからOpenGLの行列に変換
	glMatrixMode( GL_MODELVIEW );			// 行列変換モード・モデルビュー
	glLoadMatrixd( gl_para );				// 読み込む行列を指定

	switch( mark_id ){
	case MARK1_MARK_ID:
		// ライティング
		SetupLighting1();			// ライトの定義
		glEnable( GL_LIGHTING );	// ライティング・有効
		glEnable( GL_LIGHT0 );		// ライト0・オン
		// オブジェクトの材質
		SetupMaterial1();

		// 3Dオブジェクトの描画
		glTranslatef( 0.0, 0.0, 25.0 );	// マーカの上に載せるためにZ方向（マーカ上方）に25.0[mm]移動
		glutSolidCube( 150.0 );			// ソリッドキューブを描画（1辺のサイズ50[mm]）
		break;

	case MARK2_MARK_ID:
		// ライティング
		SetupLighting2();			// ライトの定義
		glEnable( GL_LIGHTING );	// ライティング・有効
		glEnable( GL_LIGHT0 );		// ライト0・オン
		// オブジェクトの材質
		SetupMaterial2();

		// 3Dオブジェクトの描画
		glTranslatef( 0.0, 0.0, 25.0 );		// マーカの上に載せるためにZ方向（マーカ上方）に25.0[mm]移動
		glutSolidSphere( 50.0, 10, 10 );	// ソリッドスフィアを描画（1辺のサイズ50[mm]）
		break;

	case MARK3_MARK_ID:
		// ライティング
		SetupLighting1();			// ライトの定義
		glEnable( GL_LIGHTING );	// ライティング・有効
		glEnable( GL_LIGHT0 );		// ライト0・オン
		// オブジェクトの材質
		SetupMaterial2();

		// 3Dオブジェクトの描画
		glTranslatef( 0.0, 0.0, 25.0 );	// マーカの上に載せるためにZ方向（マーカ上方）に25.0[mm]移動
		glRotated( 90, 1.0, 0.0, 0.0);	// ティーポットをマーカ上に載せるために90°回転
		glutSolidTeapot( 50.0 );		// ソリッドティーポットを描画（サイズ50[mm]）
		break;
	}


	// 終了処理
	glDisable( GL_LIGHTING );		// ライティング・無効
	glDisable( GL_DEPTH_TEST );		// デプステスト・無効
}


//=======================================================
// ライティング
//=======================================================
void SetupLighting1(void)
{
	// ライトの定義
	GLfloat lt0_position[] = {100.0, -200.0, 200.0, 0.0};	// ライト0の位置
	GLfloat lt0_ambient[]  = {0.1, 0.1, 0.1, 1.0};			// 　　　　 環境光
	GLfloat lt0_diffuse[]  = {0.8, 0.8, 0.8, 1.0};			// 　　　　 拡散光

	// ライトの設定
	glLightfv( GL_LIGHT0, GL_POSITION, lt0_position );
	glLightfv( GL_LIGHT0, GL_AMBIENT, lt0_ambient );
	glLightfv( GL_LIGHT0, GL_DIFFUSE, lt0_diffuse );
}

void SetupLighting2(void)
{
	// ライトの定義
	GLfloat lt0_position[] = {100.0, 200.0, 200.0, 0.0};	// ライト0の位置
	GLfloat lt0_ambient[]  = {0.2, 0.2, 0.2, 1.0};			// 　　　　 環境光
	GLfloat lt0_diffuse[]  = {0.8, 0.8, 0.8, 1.0};			// 　　　　 拡散光

	// ライトの設定
	glLightfv( GL_LIGHT0, GL_POSITION, lt0_position );
	glLightfv( GL_LIGHT0, GL_AMBIENT, lt0_ambient );
	glLightfv( GL_LIGHT0, GL_DIFFUSE, lt0_diffuse );
}


//=======================================================
// マテリアルの設定
//=======================================================
void SetupMaterial1(void)
{
	// オブジェクトの材質
	GLfloat mat_ambient[] = {0.0, 1.0, 1.0, 1.0};	// 材質の環境光
	GLfloat mat_specular[] = {0.0, 0.0, 1.0, 1.0};	// 鏡面光
	GLfloat mat_shininess[] = {50.0};				// 鏡面係数

	// マテリアルの設定
	glMaterialfv(GL_FRONT, GL_AMBIENT, mat_ambient);
	glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);
	glMaterialfv(GL_FRONT, GL_SHININESS, mat_shininess);
}

void SetupMaterial2(void)
{
	// オブジェクトの材質
	GLfloat mat_ambient[] = {0.0, 0.0, 1.0, 1.0};	// 材質の環境光
	GLfloat mat_specular[] = {0.0, 0.0, 1.0, 1.0};	// 鏡面光
	GLfloat mat_shininess[] = {50.0};				// 鏡面係数

	// マテリアルの設定
	glMaterialfv(GL_FRONT, GL_AMBIENT, mat_ambient);
	glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);
	glMaterialfv(GL_FRONT, GL_SHININESS, mat_shininess);
}


//=======================================================
// キーボード入力処理関数
//=======================================================
void KeyEvent( unsigned char key, int x, int y )
{
	//Enterキーを入力したらボールを発射
	if (48 <= key && key <= 57) {
		gstartV = (key - 48) * 30.;
	}
	if(key == 0x0D /*&& !gsimulator.ballIsMoving*/){
		startFlag = 1;
	}else if (key == 0x1b ){// ESCキーを入力したらアプリケーション終了
		printf("*** %f (frame/sec)\n", (double)count/arUtilTimer());
		Cleanup();
		exit(0);
	}
	if(key == ' '){//スペースキーでウインドウモードを切り替え
		if(Change == false){Change = true;}
		else{Change = false;}
		fullscreen();
	}
}


//=======================================================
// マウス入力処理関数
//=======================================================
void MouseEvent( int button, int state, int x, int y )
{
	// 入力状態を表示
	printf("ボタン：%d 状態：%d 座標：(x,y)=(%d,%d) \n", button, state, x, y );
}


//=======================================================
// 終了処理関数
//=======================================================
void Cleanup(void)
{
	arVideoCapStop();	// ビデオキャプチャの停止
	arVideoClose();		// ビデオデバイスの終了
	argCleanup();		// ARToolKitの終了処理
}