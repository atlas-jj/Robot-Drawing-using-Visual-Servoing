///*
// * captureVideo.cpp
// *
// *  Created on: 2016年10月22日
// *      Author: junjin
// */
//#include <cv.h>
//#include <highgui.h>
//#include <iostream>
//#include <fstream>
//
//using namespace cv;
//using namespace std;
//
////Mat src;
//Mat src_gray;
//int thresh = 40;
//int max_thresh = 255;
//RNG rng(12345);
//
//int main( int argc, char** argv )
//{
//   VideoCapture capture(argv[1]);//argv[1]
//	if(!capture.isOpened())
//	   return 1;
//	double rate=capture.get(CV_CAP_PROP_FPS);
//	bool stop(false);
//	Mat frame;
//
//	//namedWindow("Canny Video");
//	int delay=1000/rate;
//
//	while(!stop)
//	{
//	   if(!capture.read(frame))
//		   break;
////	   Mat result;
////	   Canny(frame,result,100,200);
////	   threshold(result,result,128,255,THRESH_BINARY);
////	   imshow("Canny Video",result);
//
//	   /// Convert image to gray and blur it
//	     cvtColor( frame, src_gray, CV_BGR2GRAY );
//	     blur( src_gray, src_gray, Size(3,3) );
//
//	     /// Create Window
//	     char* source_window = "Source";
//	     namedWindow( source_window, CV_WINDOW_AUTOSIZE );
//
//	     Mat canny_output;
//	       vector<vector<Point> > contours;
//	       vector<Vec4i> hierarchy;
//
//	       /// Detect edges using canny
//	       //Canny( src_gray, canny_output, thresh, thresh*2, 3 );
//	       Canny( src_gray, canny_output, thresh, thresh*2, 3 );
//	       /// Find contours
//	       findContours( canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
//
//	       /// Draw contours
//	       Mat drawing = Mat::zeros( canny_output.size(), CV_8UC3 );
//
//	       drawContours(frame, contours, -1, (255,255,0), 1);
//
//	       /// Show in a window
//	       //namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
//	       imshow( source_window, frame );
//
//	       string PointOutPuts;
//	       	       for(int i=0;i<contours.size();i++)
//	       	       {
//	       	    	   stringstream ss0;
//	       	    	   ss0<<contours[i].size();
//	       	    	   string thisVectorPoints=ss0.str()+"\r\n";
//	       	    	   for(int j=0;j<contours[i].size();j++)
//	       	    	   {
//	                         Point ppTempt=contours[i][j];
//	                         stringstream ss;
//	                         ss<<ppTempt.x;
//	                         stringstream ss2;
//	                         ss2<<(0-ppTempt.y);
//	                         thisVectorPoints +=ss.str()+","+ss2.str()+"\r\n";
//	       	    	   }
//	       	    	   PointOutPuts +=(thisVectorPoints)+"\r\n\r\n";
//	       	       }
//	       	    ofstream myfile;
//	       	      myfile.open ("images/samples.txt");
//	       	      myfile << PointOutPuts;
//	       	      myfile.close();
//	   if(waitKey(delay)>=0)
//		   stop=true;
//	}
//	capture.release();
//}
