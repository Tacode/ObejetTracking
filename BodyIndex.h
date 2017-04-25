//#ifndef __BODYINDEX_H__
//#define __BODYINDEX_H__
//#endif 
#pragma once
#include <stdio.h>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\core\mat.hpp>
#include <cv.h>
#include <Kinect.h>
#include <Windows.h>
using namespace cv;
using namespace std;

template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
	if (pInterfaceToRelease != NULL)
	{
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}

class BodyIndexBasic{
public:
	BodyIndexBasic();
	~BodyIndexBasic();	
	HRESULT InitializeDefaultSensor();
	void UpDate();
	void searchForMovement(cv::Mat thresholdImage, cv::Mat &cameraFeed);
	string intToString(int number);
	
	//bounding rectangle of the object, we will use the center of this as its position.
	
	
private:
	IKinectSensor * kinect;
	IBodyIndexFrameReader * pBodyIndexFrameReader;
	IColorFrameReader *pColorFrameReader;
	int height, ColorHeight;
	int width, ColorWidth;
	Mat img;
	Mat ColorImage;
	Mat thresholdImage;
	const static int SENSITIVITY_VALUE = 30;
	
	//ColorImageFormat imageFormat = ColorImageFormat_None;
};