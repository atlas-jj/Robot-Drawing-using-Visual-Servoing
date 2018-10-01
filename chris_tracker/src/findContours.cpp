////#include <opencv2/opencv.hpp>
//#include <iostream>
//#include <vector>
//#include <cv.h>
//#include <highgui.h>
////
//using namespace cv;
//using namespace std;
//
////#define minwidth 30
//cv::Size2f a;
////const decltype(a.width) minwidth = 30;  //if you dont know the type of width
//#define minheight  30
//#define maxwidth   100
//#define maxheight  100
//
//
//
//double threshold1 = 20;
//double threshold2 = 40;
//int apertureSize = 3;//aperture size for the Sobel() operator.
//int point_num_min = 100; //least point in a contour
//
//int main(int argc, char** argv)
//{
//    if(argc < 2)
//        return -1;
//    cv::Mat image1 = cv::imread(argv[1]);
//    cv::namedWindow("canny_edge1",1);
//    cv::cvtColor(image1, image1, CV_BGR2GRAY);
//    cv::GaussianBlur(image1, image1, cv::Size(7,7), 1.5, 1.5);
//    cv::Canny(image1,image1,threshold1,threshold2,apertureSize, false);
//    cv::imshow("canny_edge1",image1);
//    std::vector<std::vector<cv::Point> > contours1;
//    std::vector<cv::RotatedRect> contours01;
//
//    cv::findContours( image1, contours1 ,CV_RETR_LIST,CV_CHAIN_APPROX_NONE,cvPoint(0,0));
//
//    cv::Mat ellipse_contour ;
//
//    cv::Mat allcontours(image1.size(),CV_8U,cv::Scalar(255));
//    ellipse_contour.create(image1.size(),image1.type());          //two types of initiation
//
//    contours01.reserve(contours1.size());
//
//    for( std::size_t k = 0; k < contours1.size(); k++ )
//    {
//
//        int count = contours1[k].size(); // This is point's number in the contour
//        if( count > point_num_min )   //  the least points to form a contour
//            contours01.push_back(cv::fitEllipse( contours1[k]) );
//    }
//    //cv::drawContours(allcontours, contours1,-1, CV_RGB(0,0,0), 1, cv::LINE_8,cv::noArray(),contours1.size(),cvPoint(0,0));
//
//
//    cv::Size2f min;
//    cv::Size2f max;
//    min.width = 200;
//    min.height = minheight;
//    max.width  = maxwidth ;
//    max.height = maxheight;
//    cv::Size2f ellipse_size;
//    for(  a : contours01)
//    {
//    	ellipse_size = a.size;
//        if(
//                ellipse_size.width < max.width && ellipse_size.height < max.height
//          &&    ellipse_size.width > min.width && ellipse_size.height > min.height
//                )
//        {
//            cv::ellipse(ellipse_contour,a,CV_RGB(255,255,255));
//            std::cout << "size = "  << a.size   << " , "
//                      << "center =" << a.center << std::endl;
//        }
//
//    }
//    cv::namedWindow("drawcontour",1);
//    cv::namedWindow("ellipsecontour",1);
//    cv::imshow("drawcontour",allcontours);
//    cv::imshow("ellipsecontour",ellipse_contour);
//    cv::waitKey(0);
//    return 0;
//}
