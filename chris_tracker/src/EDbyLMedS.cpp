//#include "stdafx.h"
//#include "stdio.h"
//#include "stdlib.h"
//#include "conio.h"
//#include "iostream"
//#include "math.h"
//#include "time.h"
//using namespace std;
//
//#include "cv.h"
//#include "highgui.h"
//#include "cxcore.h"
//#include "cvaux.h"
//using namespace cv;
//
//const int Bubblelength = 10000;		//���鳤��(��ֹ�������)
//const double pi = 3.141592653;		//����pi
//const double Threshold = 0.8;		//ͶƱ��ֵ(�жϴ˴��Ƿ������Բ)
//const double smallestAxis = 15;		//�����Բ��С��ֵ(ֻ�ܼ���������smallestAxis���ص���Բ)
//
//int obtainMedian( Mat Input, int Size );
//void obtain5parameters( CvMat *P, int Psize, double &a, double &b, double &c, double &d, double &e );
//void votingFilter( CvMat *InMat, Mat ImageMat, int MatSize, CvSize ImageSize, Mat &bestOutput, double T, double sAxis );
//void findRealEllipses( Mat BestM, int BestMsize, double Th, Mat &RealP, int &RealNum );
//
//void main() {
//	long begin,end;																//�����㷨����ʱ��
//	begin = clock();															//�㷨��ʼʱ��
//	int columnnum = 0;
//	int contourlength,NumofrealEllipses;										//�������ȣ���ʵ��Բ����
//	char *fileName = "5.jpg";												//�ļ���
//	CvPoint center;																//��ԲԲ������
//	CvSize axis;																//���̰���
//	CvSeq* contours;
//	IplImage *originalImage = cvLoadImage( fileName, CV_LOAD_IMAGE_COLOR );		//����ɫͼƬ����
//	IplImage *ImageRGB = cvCreateImage(cvGetSize(originalImage), 8, 1 );
//	IplImage *SmoothyImage = cvCreateImage(cvGetSize(originalImage), 8, 1 );
//	IplImage *BImage = cvCreateImage(cvGetSize(originalImage), 8, 1 );			//����8λ1ͨ��ͼ�����ڴ�Ŷ�ֵͼƬ
//	CvSize Imagesize = cvGetSize( originalImage );
//	Mat BImageMat = Mat::zeros( Imagesize.height, Imagesize.width, CV_64FC1 );	//��Ŷ�ֵ����ͼƬ���ؾ���
//	CvMemStorage *storage = cvCreateMemStorage(0);								//�����ڴ�洢���������洢���ص�����
//	cvConvertImage( originalImage, ImageRGB, 0 );								//�������ͼƬת��Ϊ8λ��ͨ��ͼ�����ImageRGB
//	cvSmooth( ImageRGB, SmoothyImage, CV_GAUSSIAN, 3, 3, 0 );					//ƽ���������SmoothyImage
//	cvCanny( SmoothyImage, BImage, 100, 255, 3);								//ȡ��������ö�ֵͼ��
//	for ( int i = 0; i < Imagesize.height; i++ ) {
//		for ( int j = 0; j < Imagesize.width; j ++) {
//			BImageMat.at<double>(i,j) = cvGet2D(BImage,i,j).val[0];
//		}
//	}//����ͼ�����ؾ���
//	cvNamedWindow( "Ellipse", 1 );
//	cvShowImage( "Ellipse", BImage );				//��ʾ��ֵ����ͼƬ
//	cvWaitKey(20);
//	int numberOfContours = cvFindContours( BImage,
//		storage,
//		&contours,
//		sizeof(CvContour),
//		CV_RETR_LIST,
//		CV_CHAIN_APPROX_SIMPLE,
//		cvPoint(0,0)
//	);//��ȡ����
//	CvMat *fiveparmeters = cvCreateMat( 5, numberOfContours, CV_64FC1 );	//��Բ��������
//	for ( ; contours!=0; contours = contours -> h_next ) {					//��ÿ���������ɨ�������������
//		double A,B,C,D,E,p,q,semimajor,semiminor,theta,k;					//������Բ������һЩ���ȱ���
//		contourlength = contours -> total;									//��ȡÿ�������ĳ���(ÿ�������������������)
//		if( contourlength >10) {											//��������С��10������
//			CvMat *MatPinXY = cvCreateMat( 2, contourlength, CV_64FC1 );
//			for( int i = 0 ; i < contourlength; i++ ) {
//				CvPoint *points = (CvPoint*)cvGetSeqElem(contours, i);
//				cvmSet( MatPinXY, 0, i, points -> y);
//				cvmSet( MatPinXY, 1, i, points -> x);
//			}//����������д��MatPinXY(ע��xy����λ�ã�ת��Ϊ��׼����ϵ���������)
//			obtain5parameters( MatPinXY, contourlength, A, B, C, D, E );	//ÿ���������һ����Բ��������
//		}
//		else { A = 0; B = 0; C = 0; D = 0; E = 0; }						//���ڲ�����Ҫ�������������Բ������������Ϊ��
//		if( (B*B) < (4*A*C) ) {												//��������������ԲԼ���ж�
//			p = ((B*D-2*A*E)/(4*A*C-B*B));
//			q = ((B*E-2*C*D)/(4*A*C-B*B));									//���Բ������(�Ѿ�ת��Ϊͼ������ϵ)
//			k = A*q*q+B*q*p+C*p*p-1;										//���ɱ���
//			if ( k*(A+C-sqrt(B*B+pow(A-C,2))) > 0 && p > 0 && q > 0) {		//�ٴ�ɸѡ
//				if( A < C) {												//��ʱ���᳤�ȴ��ڸ���
//					semimajor = (sqrt((2*k)/(A+C-sqrt(B*B+pow((A-C),2)))));
//					semiminor = (sqrt((2*k)/(A+C+sqrt(B*B+pow((A-C),2)))));
//				}
//				else {
//					semimajor = (sqrt((2*k)/(A+C+sqrt(B*B+pow((A-C),2)))));
//					semiminor = (sqrt((2*k)/(A+C-sqrt(B*B+pow((A-C),2)))));
//				}
//				if( (A-C)==0 ) {											//ƫ�Ǵ�ֱ�ж�
//					theta = 0.25*pi;
//				}
//				else {
//					theta = 0.5*atan(B/(A-C));
//				}
//				if( p < originalImage->width && q < originalImage->height) {	//Բ��������ͼƬ��(�ٴ�ɸѡ)
//					cvmSet( fiveparmeters, 0, columnnum, p);
//					cvmSet( fiveparmeters, 1, columnnum, q);
//					cvmSet( fiveparmeters, 2, columnnum, semimajor);
//					cvmSet( fiveparmeters, 3, columnnum, semiminor);
//					cvmSet( fiveparmeters, 4, columnnum, theta);
//					columnnum = columnnum + 1;
//				}//�������Ҫ��Ĳ���������������������������columnnum
//				else {}
//			}
//			else{}
//		}
//		else{}
//	}
//	Mat best5Parameters = Mat::zeros( 5, columnnum, CV_64FC1 );		//������Ѳ�������(��������ȡ��ֵ)
//	Mat realEllipses = Mat::zeros( 5, columnnum, CV_64FC1 );		//������ʵ��Բ��������(��������ȡ��ֵ)
//	votingFilter( fiveparmeters, BImageMat, columnnum, Imagesize, best5Parameters, Threshold, smallestAxis );//ͶƱ�˲��������Ѳ���
//	findRealEllipses( best5Parameters, columnnum, smallestAxis, realEllipses, NumofrealEllipses );
//	end = clock();													//�㷨����ʱ��
//	cout << "�˴μ����ʱ:"<< end-begin << "ms" << endl;			//��ӡ�㷨��ʱ
//	cout << endl;
//	cout << "����⵽" << NumofrealEllipses << "����Բ" << endl;
//	cout << endl;
//	for( int i =0; i < NumofrealEllipses; i++ ) {
//		center.x = (int)realEllipses.at<double>(0,i);
//		center.y = (int)realEllipses.at<double>(1,i);
//		axis.width = (int)realEllipses.at<double>(2,i);
//		axis.height = (int)realEllipses.at<double>(3,i);
//		cvEllipse( originalImage, center, axis, 90-180*realEllipses.at<double>(4,i)/pi, 0, 360, CV_RGB(255, 0, 0), 1, CV_AA, 0);
//		cout << "��" << i+1 << "����Բ����:" << endl;
//		cout << "Բ������:(" << center.x << "," << center.y << ")" << endl;
//		cout << "���̰���:" << axis.width << "  " << axis.height << endl;
//		cout << "ƫ��" << 90-180*realEllipses.at<double>(4,i)/pi << "��" << endl;
//		cout << endl;
//	}
//
//	cvNamedWindow( "Final", 1 );
//	cvShowImage( "Final", originalImage );
//
//	cvWaitKey(0);
//	cvReleaseMemStorage( &storage );
//	cvReleaseMat( &fiveparmeters );
//	cvDestroyWindow( "Ellipse" );
//	cvDestroyWindow( "Final" );
//
//
//}
//
////��ȡÿ��������Ӧ����Բ��������//
//void obtain5parameters( CvMat *P, int Psize, double &a, double &b, double &c, double &d, double &e ) {
//
//	int counter = Psize/5;				//ѭ���������(�����ȡ��Ĵ���)
//	int r,bestParaNum;					//���������r����Ѳ������������(������Բ���������е���Ѳ�������)
//	double ra,rb,rc,rd,re;				//��������(������Ѳ�������)
//
//	Mat Para = Mat::zeros( 5, counter, CV_64FC1 );			//��Ų�������
//	Mat ParaD = Mat::zeros( 1, counter, CV_64FC1 );			//��ֵ����(���ÿ������������Ӧ����ֵ)
//	Mat M = Mat::zeros( 5, 5, CV_64FC1 );					//���Է��������߾���
//	Mat X = Mat::zeros( 5, 1, CV_64FC1 );					//���Է�����Ľ�����
//	Mat Y = Mat::zeros( 5, 1, CV_64FC1 );					//���Է�������ұ�����
//	Mat MatD = Mat::zeros( 1, Psize, CV_64FC1 );			//���ÿ�����Ӧ���
//
//	for( int i = 0; i < 5; i++ ) {
//		Y.at<double>(i,0) = -1;
//	}//�������Է������ұ߾���
//
//	for( int i = 0; i < counter; i++ ) {
//		for( int j = 0; j < 5; j++ ) {
//			srand((i+1)*(j+1));			//�������������
//			r = rand()%Psize;			//���������(���ȡ��ʱʹ��)
//			M.at<double>(j,0) = pow( cvmGet( P, 0, r ), 2 );
//			M.at<double>(j,1) = cvmGet( P, 1, r )*cvmGet( P, 0, r );
//			M.at<double>(j,2) = pow ( cvmGet( P, 1, r ), 2 );
//			M.at<double>(j,3) = cvmGet( P, 0, r );
//			M.at<double>(j,4) = cvmGet( P, 1, r );
//		}//ͨ�����������5����������Է�������߾���
//		Mat Mt = M.t();
//		Mat MtM = Mt*M;
//		Mat Min = MtM.inv();
//		X = Min*Mt*Y;//�󷽳���Ľ�����(��С���˷�)
//		for( int k = 0; k < 5; k++ ) {
//			Para.at<double>(k,i) = X.at<double>(k,0);
//		}//���������������������
//	}//�����������
//	for( int i = 0; i < counter; i++ ) {
//		double DIJ;							//ÿ���������Ӧ�ĺ���ֵ
//		ra = Para.at<double>(0,i);
//		rb = Para.at<double>(1,i);
//		rc = Para.at<double>(2,i);
//		rd = Para.at<double>(3,i);
//		re = Para.at<double>(4,i);			//��������
//		for( int j = 0; j < Psize; j++) {
//			DIJ = pow(  ra*pow(cvmGet( P, 0, j ),2)+rb*cvmGet( P, 0, j )*cvmGet( P, 1, j )+rc*pow(cvmGet( P, 1, j ),2)+rd*cvmGet( P, 0, j )+re*cvmGet( P, 1, j )+1, 2 );
//			MatD.at<double>(0,j) = DIJ;
//		}
//		bestParaNum = obtainMedian( MatD, Psize );						//��ȡMatD��Ԫ����ֵ(Ԫ�ر��)
//		ParaD.at<double>(0,i) = MatD.at<double>(0,bestParaNum);			//��MatD�е���ֵ����ParaD������
//	}//��ȡParaD����(��β�������)
//	bestParaNum = obtainMedian( ParaD, counter);						//��ȡParaD��Ԫ����ֵ(Ԫ�ر�ţ��������Ѳ����������)
//	a = Para.at<double>(0,bestParaNum);
//	b = Para.at<double>(1,bestParaNum);
//	c = Para.at<double>(2,bestParaNum);
//	d = Para.at<double>(3,bestParaNum);
//	e = Para.at<double>(4,bestParaNum);									//����Ѳ����������
//	cvReleaseMat( &P );
//}
//
////����λ���Ӻ��������Ԫ�ر��//
//int obtainMedian( Mat Input, int Size ) {
//	int m;											//ð���������ֵ��Ӧ�ı��
//	int n = 0;										//ԭ����������Ԫ����ֵ��Ӧ�ı��
//	double flag = 1;								//�������������������Ԫ����ȱ�־λ
//	double tmp;										//ð�������еĹ��ɱ���
//	double r[Bubblelength] = {0};					//�ο�����(���������ĵȼ�ӳ�䣬�������ݸ�ʽһ�·���Ƚ�)
//	double BubbleSort[Bubblelength] = {0};			//��������
//	for( int i = 0; i < Size; i++ ) {
//		BubbleSort[i] = Input.at<double>(0,i);
//		r[i] = BubbleSort[i];
//	}//��ο��������������д��
//	for (int i = 0; i < Size; i++) {
//        for (int j = Size - 1; j > i; j--) {
//            if (BubbleSort[j] < BubbleSort[j-1]) {
//                tmp = BubbleSort[j-1];
//                BubbleSort[j-1] =  BubbleSort[j];
//                BubbleSort[j] = tmp;
//            }
//        }
//    }//ð������
//	if( Size%2 ) {
//		m = (Size-1)/2;
//	}
//	else {
//		m = Size/2;
//	}//��ȡ��λ��Ԫ�ر��(��������������ż�ж�)
//	while(flag != 0) {
//		flag = BubbleSort[m] - r[n];
//		n++;
//	}//�ȼ۱Ƚϣ������λ���ڲο������ж�Ӧ�ı��
//	return n-1;
//}
//
////ͶƱ�˲�ԭ��//
////���ڸ���һ��������󣬿��Ի����Բ����һ��//
////������һ��Ϊ����ȡ3*3���أ�����ڴ����������ڴ��������㣬��������1//
////ɨ����Բ�ϵ����е㣬�Ѽ�����ֵ����Բ�ܳ����ȣ����������ratio(���˴�������Բ�ĸ���)//
////��ratio�������ʼ�趨����ֵ���Ƚϣ������ж��Ƿ�Ϊ��Ѳ�������//
//void votingFilter( CvMat *InMat, Mat ImageMat, int MatSize, CvSize ImageSize, Mat &bestOutput, double T, double sAxis ) {
//	double longaxis,shortaxis,perimeter;					//������Բ����
//	Mat Rotation = Mat::zeros( 2, 2, CV_64FC1 );			//��ת����(��׼��Բ���ڵ�����ϵ��ʵ������ϵƫ��)
//	Mat center = Mat::zeros( 2, 1, CV_64FC1 );				//Բ������
//	Mat Pinxy = Mat::zeros( 2, 1, CV_64FC1 );				//���ڱ�׼��Բ��������ϵ�µ�����
//	Mat PinXY = Mat::zeros( 2, 1, CV_64FC1 );				//����ʵ������ϵ�µ�����
//	for( int i = 0; i < MatSize; i++ ) {
//		int counter = 0;												//��ʼ��������
//		double ratio = 0;												//��ʼ��������
//		center.at<double>(0,0) = cvmGet( InMat, 1, i );
//		center.at<double>(1,0) = cvmGet( InMat, 0, i );
//		longaxis = cvmGet( InMat, 2, i );
//		shortaxis = cvmGet( InMat, 3, i );
//		Rotation.at<double>(0,0) =  cos( cvmGet( InMat, 4, i ) );
//		Rotation.at<double>(0,1) =  -sin( cvmGet( InMat, 4, i ) );
//		Rotation.at<double>(1,0) =  sin( cvmGet( InMat, 4, i ) );
//		Rotation.at<double>(1,1) =  cos( cvmGet( InMat, 4, i ) );
//		perimeter = pi*(longaxis+shortaxis);							//�����������
//		if( longaxis >= sAxis && shortaxis >= sAxis ) {				//��һ�ι���(����ʼ�趨����С��Բ�˵�)
//			for ( int j = 0; j < (int)perimeter; j++) {				//ȡ��Բ�ܳ�������бȽ�(�ܺõ���Ӧ��С��һ������Բ)
//				int m,n;
//				Pinxy.at<double>(0,0) = longaxis*cos(j/perimeter);
//				Pinxy.at<double>(1,0) = shortaxis*sin(j/perimeter);
//				PinXY = Rotation*Pinxy + center;
//				m = (int)PinXY.at<double>(0,0);
//				n = (int)PinXY.at<double>(1,0);
//				int flag = ImageMat.at<double>(m,n) == 255 || ImageMat.at<double>(m+1,n) == 255 ||\
//				ImageMat.at<double>(m-1,n) == 255 || ImageMat.at<double>(m,n+1) == 255 ||\
//				ImageMat.at<double>(m+1,n+1) == 255 || ImageMat.at<double>(m-1,n+1) == 255 ||\
//				ImageMat.at<double>(m,n-1) == 255 || ImageMat.at<double>(m+1,n-1) == 255 ||\
//				ImageMat.at<double>(m-1,n-1);
//				if(flag) {												//�ڶ��ι���
//					counter = counter+1;
//				}
//				else{}
//			}
//			ratio = counter/perimeter;									//��ȡ������
//			if( ratio >= T ) {
//				for( int k = 0; k < 5; k++) {
//					bestOutput.at<double>(k,i) = cvmGet( InMat, k, i );
//				}
//			}//�����Ѳ�������
//			else {}
//		}
//		else {}
//	}
//	cvReleaseMat( &InMat );
//}
//
////�������Բ�����Ƶ���ϳ�һ��//
//void findRealEllipses( Mat BestM, int BestMsize, double Th, Mat &RealP, int &RealNum ) {
//	Mat FitEllipseMatrix = Mat::zeros( BestMsize, BestMsize+1, CV_64FC1 );		//������Ϊ��ͬ��Բ�����������д����ͬ��Բ��BestM�е�λ��
//	Mat relativeMatrix = Mat::zeros( 5, BestMsize, CV_64FC1 );	              //�Ƚ��þ���
//	for( int i = 0; i < BestMsize; i++ ) {
//		relativeMatrix.at<double>(0,i) = BestM.at<double>(0,i);
//		relativeMatrix.at<double>(1,i) = BestM.at<double>(1,i);
//		relativeMatrix.at<double>(2,i) = BestM.at<double>(2,i);
//		relativeMatrix.at<double>(3,i) = BestM.at<double>(3,i);
//		relativeMatrix.at<double>(4,i) = BestM.at<double>(4,i);
//	}
//	int differentEllipses = 0;
//	for( int i = 0; i < BestMsize; i++ ) {
//		int sameEllipses = 0;
//		if( relativeMatrix.at<double>(0,i) != 0 ) {
//			double relativeVariablex = relativeMatrix.at<double>(0,i);
//			double relativeVariabley = relativeMatrix.at<double>(1,i);
//			double relativeVariablea = relativeMatrix.at<double>(2,i);
//			double relativeVariableb = relativeMatrix.at<double>(3,i);
//			double relativeVariabletheta = relativeMatrix.at<double>(4,i);
//			for( int j = 0; j < BestMsize; j++) {
//				if( relativeMatrix.at<double>(0,j) != 0 ) {
//					double currentx = relativeMatrix.at<double>(0,j);
//					double currenty = relativeMatrix.at<double>(1,j);
//					double currenta = relativeMatrix.at<double>(2,j);
//					double currentb = relativeMatrix.at<double>(3,j);
//					double currenttheta = relativeMatrix.at<double>(4,j);
//					int flag = sqrt(pow(relativeVariablex-currentx,2)+pow(relativeVariabley-currenty,2)) < Th &&\
//						fabs(relativeVariablea-currenta) < 5 && fabs(relativeVariablea-currenta) < 5 &&\
//						fabs(relativeVariabletheta-currenttheta) < 2;
//					if( flag ) {
//						relativeMatrix.at<double>(0,j) = 0;
//						FitEllipseMatrix.at<double>(differentEllipses,sameEllipses+1) = j;
//						sameEllipses++;
//					}
//				}
//			}
//			FitEllipseMatrix.at<double>(differentEllipses,0) = sameEllipses;
//			differentEllipses++;
//		}
//	}
//	for( int i = 0; i < differentEllipses; i++ ) {
//		int sameE = (int)FitEllipseMatrix.at<double>(i,0);
//		for( int j = 0; j < sameE; j++) {
//			RealP.col(i) = RealP.col(i) + BestM.col((int)FitEllipseMatrix.at<double>(i,j+1));
//		}
//		RealP.col(i) = RealP.col(i)*(1/(double)sameE);
//	}
//	RealNum = differentEllipses;
//}
