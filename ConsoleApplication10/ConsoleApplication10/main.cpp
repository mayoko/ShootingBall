#include <opencv2\opencv.hpp>
#include <opencv2/highgui/highgui.hpp> 
#include <iostream>
#include <fstream>
#include <vector>

 #ifdef _DEBUG
 //Debugモードの場合
#pragma comment(lib,"opencv_core2411d.lib")
#pragma comment(lib,"opencv_calib3d2411d.lib")
 #pragma comment(lib,"opencv_imgproc2411d.lib")
 #pragma comment(lib,"opencv_highgui2411d.lib")
 #pragma comment(lib,"opencv_objdetect2411d.lib")
 #pragma comment(lib,"opencv_contrib2411d.lib")
 #pragma comment(lib,"opencv_features2d2411d.lib")
 #pragma comment(lib,"opencv_flann2411d.lib")
 #pragma comment(lib,"opencv_gpu2411d.lib")
 #pragma comment(lib,"opencv_legacy2411d.lib")
 #pragma comment(lib,"opencv_ts2411d.lib")
 #pragma comment(lib,"opencv_video2411d.lib")
 #else
 //Releaseモードの場合
#pragma comment(lib,"opencv_core2411.lib")
 #pragma comment(lib,"opencv_imgproc2411.lib")
 #pragma comment(lib,"opencv_highgui2411.lib")
 #pragma comment(lib,"opencv_objdetect2411.lib")
 #pragma comment(lib,"opencv_contrib2411.lib")
 #pragma comment(lib,"opencv_features2d2411.lib")
 #pragma comment(lib,"opencv_flann2411.lib")
 #pragma comment(lib,"opencv_gpu2411.lib")
 #pragma comment(lib,"opencv_legacy2411.lib")
 #pragma comment(lib,"opencv_ts2411.lib")
 #pragma comment(lib," opencv_video2411.lib")
 #endif

using namespace cv;
using namespace std;

void on_mouse( int e,int x, int y, int d, void* param)
{
	vector<Point>* markerPos = (vector<Point>*)(param);
	Point pt;
	if(e == CV_EVENT_LBUTTONDOWN){
		pt.x = x;                         //x座標はx
		pt.y = y;                         //y座標はy
		markerPos->push_back(pt);
		std::cout << *markerPos << std::endl;
			}
}


int new_main(int argc, const char* argv[])
{

//////////////////カメラからウィンドウに出力、マウスクリックで座標取得///////////////////////////////////////
  int camera_id = 0;
  cv::VideoCapture cap(CV_CAP_DSHOW + camera_id);
  if(!cap.isOpened())
    return -1;                                    
  cap.set(CV_CAP_PROP_FPS, 30.0);                  //フレームレート30.0
  namedWindow("window", cv::WINDOW_AUTOSIZE);  
  Mat frame;                                   
  vector <Point> markerPos; 
  cv::setMouseCallback("window",on_mouse,&markerPos);

  for(;;)
  {
    cap >> frame;	                                //カメラから1フレーム分の画像データを取得して変数frameに格納する
    if (frame.empty()) break;	                    // 画像データ取得に失敗したら
	//imwrite("image.png", frame);	          
    imshow("window", frame);	      
	if ((int)markerPos.size() > 3) break;
    if (cv::waitKey(30) >= 0) break;
  }
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
  
 
 
///////////////////////ホモグラフィー行列生成、出力///////////////////////////////////////////////////////////////////////
 int i;
 double cx[4],cy[4];
 Mat imageroi[4],gray_roi[4],mono[4],remono[4],H;
 Moments m[4];
 Point2d center[4];
 vector<Point2f> before,after;

 for(i=0;i<4 ;i++)
 {
	  imageroi[i]= frame(Rect(markerPos[i].x,markerPos[i].y,15,15)); 
	  //namedWindow("miniwindow"); cv::imshow("miniwindow",imageroi[0]);
	  cvtColor(imageroi[i], gray_roi[i],CV_RGB2GRAY);
	  threshold(gray_roi[i],mono[i],0,255,THRESH_BINARY | THRESH_OTSU);
	 //namedWindow("nitika"); imshow("nitika",mono[3]);
	  remono[i] = ~mono[i];	  
	  //namedWindow("reverse"); imshow("reverse",remono[0]);
	  m[i] = moments(remono[i],true);
	  cx[i] = (m[i].m10/m[i].m00);
	  cy[i] = (m[i].m01/m[i].m00);  
	  center[i].x = markerPos[i].x + cx[i];
	  center[i].y = markerPos[i].y + cy[i];  
	  circle(frame, center[i], 5, cv::Scalar(200,0,0),4,4);
	  cv::imwrite("image.png", frame);
  }
 before.push_back(center[0]);
 before.push_back(center[1]);
 before.push_back(center[2]);
 before.push_back(center[3]);

 //H_wcのとき
 
 after.push_back(cv::Point(0,0));
 after.push_back(cv::Point(0,530));
 after.push_back(cv::Point(782,530));
 after.push_back(cv::Point(782,0));
 
//

 //H_pcのとき
 /*
 after.push_back(cv::Point(100,100));
 after.push_back(cv::Point(100,700));
 after.push_back(cv::Point(1180,700));
 after.push_back(cv::Point(1180,100));
 */
 //
 
 H = cv::findHomography( before,after);
 cout <<H << std::endl;
 
 ofstream ofs( "H_wc.txt" );    //H_wc,H_pcで書き換え必要
	ofs << H.at<double>(0,0) <<"\t"<< H.at<double>(0,1) <<"\t"<< H.at<double>(0,2) << std::endl;
	ofs << H.at<double>(1,0) <<"\t"<< H.at<double>(1,1) <<"\t"<< H.at<double>(1,2) << std::endl;
	ofs << H.at<double>(2,0) <<"\t"<< H.at<double>(2,1) <<"\t"<< H.at<double>(2,2) << std::endl;
	
/////////////////////////////////////////////////////////////////////////////////////////////////////////


/////////////////中点を出力してH_wcをチェック///////////////////////////////////////////////////////////
 Point2d tyuuten[4];
 Mat mata = (Mat_<double>(3,4)<< 0,391,782,391, 265,530,265,0, 1,1,1,1);
 Mat matb = (Mat_<double>(3,4));
 
 matb = H.inv() * mata;
 for(i=0;i<4 ;i++)
 {
	tyuuten[i].x = matb.at<double>(0,i)/matb.at<double>(2,i) ;
	tyuuten[i].y = matb.at<double>(1,i)/matb.at<double>(2,i) ;
	circle(frame, tyuuten[i], 5, cv::Scalar(0,200,0),4,4);
 }
 imwrite("image.png", frame);
 
//////////////////////////////////////////////////////////////////////////////////////////////////


///////////Hを読み込んでH_pcのチェック画像生成///////////////////////////////////////////////////////////////////////////
  /*
	Mat H_wc(3,3,CV_64FC1);
	Mat H_pc(3,3,CV_64FC1);
	Mat H_pw(3,3,CV_64FC1);

	ifstream ifs( "H_wc.txt" );
	ifs >> H_wc.at<double>(0,0) ;
	ifs >> H_wc.at<double>(0,1) ;
	ifs >> H_wc.at<double>(0,2) ;
	ifs >> H_wc.at<double>(1,0) ;
	ifs >> H_wc.at<double>(1,1) ;
	ifs >> H_wc.at<double>(1,2) ;
	ifs >> H_wc.at<double>(2,0) ;
	ifs >> H_wc.at<double>(2,1) ;
	ifs >> H_wc.at<double>(2,2) ;
	cout <<"H_wc = "<<H_wc << std::endl;
	ifstream ifs2( "H_pc.txt" );
	ifs2 >> H_pc.at<double>(0,0) ;
	ifs2 >> H_pc.at<double>(0,1) ;
	ifs2 >> H_pc.at<double>(0,2) ;
	ifs2 >> H_pc.at<double>(1,0) ;
	ifs2 >> H_pc.at<double>(1,1) ;
	ifs2 >> H_pc.at<double>(1,2) ;
	ifs2 >> H_pc.at<double>(2,0) ;
	ifs2 >> H_pc.at<double>(2,1) ;
	ifs2 >> H_pc.at<double>(2,2) ;
	cout <<"H_pc = "<<H_pc << std::endl;
	
	H_pw = H_pc * H_wc.inv();
    ofstream ofs( "H_pw.txt");  
	ofs << H_pw.at<double>(0,0) <<"\t"<< H_pw.at<double>(0,1) <<"\t"<< H_pw.at<double>(0,2) << std::endl;
	ofs << H_pw.at<double>(1,0) <<"\t"<< H_pw.at<double>(1,1) <<"\t"<< H_pw.at<double>(1,2) << std::endl;
	ofs << H_pw.at<double>(2,0) <<"\t"<< H_pw.at<double>(2,1) <<"\t"<< H_pw.at<double>(2,2) << std::endl;
	cout <<"H_pw = "<<H_pw << std::endl;
	
	Mat mataa = (cv::Mat_<double>(3,4)<< 0,0,782,782,0,530,530,0,1,1,1,1);
	Mat matbb = (cv::Mat_<double>(3,4));
	matbb = H_pw * mataa;
	Point2d kado[4];
	int i;
	for(i=0;i<4 ;i++)
	{
		kado[i].x = matbb.at<double>(0,i)/matbb.at<double>(2,i) ;
		kado[i].y = matbb.at<double>(1,i)/matbb.at<double>(2,i) ;
	}
	Mat white_img(cv::Size(1280,800), CV_8UC3, cv::Scalar::all(255));
	line(white_img,kado[0],kado[2],Scalar(0,0,250), 3, 8);
	line(white_img,kado[1],kado[3],Scalar(0,0,250), 3, 8);
    imwrite("check.png", white_img);
	*/
  /////////////////////////////////////////////////////////////////////////////////
 
for(;;){ 
	if (cv::waitKey(30) >= 0) break;
}
  return 0;
}