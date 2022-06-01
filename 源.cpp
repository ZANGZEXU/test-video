#include<iostream>
#include<opencv2\opencv.hpp>
#include"KF.h"
using namespace std;
using namespace cv;

#define UNIT_PIXEL_W 0.0008234375
#define UNIT_PIXEL_H 0.000825
#define pianyi_x 50
#define pianyi_y 50


int main(int argc, char** argv) {

	string path = "D:/test4.mp4";
	VideoCapture cap(path);

	if (!cap.isOpened())
	{
		cout << "No camera or video input!\n" << endl;
		return -1;
	}
	int LowH = 93;
	int LowS = 98;
	int LowV = 13;

	int HighH = 255;
	int HighS = 180;
	int HighV = 60;

	const double f = 0.87;//摄像头焦距
	const double w = 1.88;//被测物体的宽度
	const double h = 4.25;//被测物体的高度

	int fps = cap.get(CAP_PROP_FPS);          //获取帧率
	int numFrames = cap.get(CAP_PROP_FRAME_COUNT);  //获取整个帧数
	double prePoint_xtl = 0, perPoint_ytl = 0, perPoint_xbr = 0, prePoint_ybr = 0;
	double dx = 0, dy = 0;
	Mat img, gray;
	while (true) {
		int num = 0;
		cap >> img;
		resize(img, img, Size(640, 480));
		//medianBlur(src, src,3);
		vector<Mat>channels;
		split(img, channels);
		Mat gray = channels.at(0) - channels.at(2);

		Mat binary;
		inRange(gray, Scalar(20), Scalar(150), binary);
		imshow("test10", binary);
		waitKey(100);
		Mat kenerl_erode = getStructuringElement(MORPH_RECT, Size(3, 3));
		Mat kenerl_dilate = getStructuringElement(MORPH_RECT, Size(15, 15));
		erode(binary, binary, kenerl_erode);
		dilate(binary, binary, kenerl_dilate);


		vector<vector<Point>>contours;
		vector<Vec4i>hierachy;
		findContours(binary, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
		//drawContours(img, contours, -1, Scalar(0, 0, 255), 2);


		vector<Point>maxAreaContours;
		double maxArea = 0;
		for (size_t i = 0; i < contours.size(); i++) {

			double area = fabs(contourArea(contours[i]));

			if (area > maxArea) {
				maxArea = area;
				maxAreaContours = contours[i];
			}

		}
		cout << "最大矩形的面积" << maxAreaContours.size() << endl;
		Rect rect = boundingRect(maxAreaContours);
		cout << "矩形左上角的坐标" << rect.tl() << "矩形的右下角坐标" << rect.br() << endl;

		rectangle(img, rect, Scalar(0, 0, 255), 2);

		dx = rect.tl().x - prePoint_xtl;
		dy = rect.tl().y - perPoint_ytl;

		cout << "x差值" << dx << '\t' << "y差值" << dy << endl;

		Rect _rect(Point(rect.tl().x + 3 * dx, rect.tl().y + 3 * dy),
			Point(rect.br().x + 3 * dx, rect.br().y + 3 * dy));
		rectangle(img, _rect, Scalar(0, 255, 0), 2);

		prePoint_xtl = rect.tl().x;
		perPoint_xbr = rect.br().x;
		perPoint_ytl = rect.tl().y;
		prePoint_ybr = rect.br().y;

		double width = rect.width * UNIT_PIXEL_W;
		double height = rect.height * UNIT_PIXEL_H;
		double distanceW = w * f / width;
		double distanceH = h * f / height;


		char disW[50], disH[50], average[50];
		sprintf_s(disW, "DistanceW:%.2fcm", distanceW);
		sprintf_s(disH, "DistanceH:%.2fcm", distanceH);
		sprintf_s(average, "DistanceH:%.2fcm", (distanceH + distanceW) / 2);
		putText(img, disW, Point(5, 20), FONT_HERSHEY_COMPLEX_SMALL, 1, Scalar(0, 255, 0), 1, 8);
		putText(img, disH, Point(5, 40), FONT_HERSHEY_COMPLEX_SMALL, 1, Scalar(0, 255, 0), 1, 8);
		putText(img, average, Point(5, 60), FONT_HERSHEY_COMPLEX_SMALL, 1, Scalar(0, 255, 0), 1, 8);
		imshow("output", img);
		waitKey(100);






	     }
		return 0;
}





















//Generating_Random_Measurement();
		//KF_Init();
		//KalmanFilter KF(2, 1, 0);
		//Mat state(2, 1, CV_32F);
		//Mat processNoise(2, 1, CV_32F);
		//Mat measurement = Mat::zeros(1, 1, CV_32F);
		//char code = (char)-1;
		//for (;;) {
		//	randn(state, Scalar::all(0), Scalar::all(0.1));
		//	KF.transitionMatrix =(Mat_<float>(2, 2) << 1, 1, 0, 1);

		//	setIdentity(KF.measurementMatrix);
		//	setIdentity(KF.processNoiseCov, Scalar::all(1e-5));
		//	setIdentity(KF.measurementNoiseCov, Scalar::all(1e-1));
		//	setIdentity(KF.errorCovPost, Scalar::all(1));
		//	randn(KF.statePost, Scalar::all(0), Scalar::all(0.1));
		//	for (;;) {
		//		Point2f center(img.cols * 0.5f, img.rows * 0.5f); 
		//		float R = img.cols / 3.f;
		//		double stateAngle = state.at<float>(0);
		//		Point statePt = calcPoint(center, R, stateAngle);

		//		Mat prediction = KF.predict();
		//		double predictAngle = prediction.at<float>(0);
		//		Point predictPt = calcPoint(center, R, predictAngle);

		//		randn(measurement, Scalar::all(0), Scalar::all(KF.measurementNoiseCov.at<float>(0)));

		//		measurement += KF.measurementMatrix * state;

		//		double measAngle = measurement.at<float>(0);
		//		Point measPt = calcPoint(center, R, measAngle);

  //              #define drawCross( center, color, d )                                 \
  //                 line( img, Point( center.x - d, center.y - d ),                \
  //                           Point( center.x + d, center.y + d ), color, 1, CV_AA, 0); \
  //                 line( img, Point( center.x + d, center.y - d ),                \
  //                           Point( center.x - d, center.y + d ), color, 1, CV_AA, 0 )

		//		

		//		img = Scalar::all(0);
		//		/*#define drawCross(statePt, Scalar(255, 255, 255), 3);
		//		#define drawCross(measPt, Scalar(0, 0, 255), 3);
		//		#define drawCross(predictPt, Scalar(0, 255, 0), 3);*/
		//		line(img, statePt, measPt, Scalar(0, 0, 255), 3);
		//		line(img, statePt, predictPt, Scalar(0, 255, 255), 3);

		//		KF.correct(measurement);

		//		randn(processNoise, Scalar(0), Scalar::all(sqrt(KF.processNoiseCov.at<float>(0, 0))));
		//		state = KF.transitionMatrix * state + processNoise;
/*	}
		}*/