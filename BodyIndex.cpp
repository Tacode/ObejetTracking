/*BodyIndex,识别人体*/
#include <stdio.h>
#include "BodyIndex.h"
using namespace cv;

string BodyIndexBasic::intToString(int number){
	//this function has a number input and string output
	std::stringstream ss;
	ss << number;
	return ss.str();
}

void BodyIndexBasic::searchForMovement(Mat thresholdImage, Mat &cameraFeed){
	//notice how we use the '&' operator for objectDetected and cameraFeed. This is because we wish
	//to take the values passed into the function and manipulate them, rather than just working with a copy.
	//eg. we draw to the cameraFeed to be displayed in the main() function.
	Mat temp;
	bool objectDetected = false;
	int theObject[2] = { 0, 0 };
	Rect objectBoundingRectangle = Rect(0, 0, 0, 0);
	thresholdImage.copyTo(temp);
	//these two vectors needed for output of findContours
	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;
	//find contours of filtered image using openCV findContours function
	//findContours(temp,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE );// retrieves all contours
	findContours(temp, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);// retrieves external contours

	//if contours vector is not empty, we have found some objects
	if (contours.size()>0)
		objectDetected = true;
	else 
		objectDetected = false;

	if (objectDetected){
		//the largest contour is found at the end of the contours vector
		//we will simply assume that the biggest contour is the object we are looking for.
		vector< vector<Point> > largestContourVec;
		largestContourVec.push_back(contours.at(contours.size() - 1));
		//make a bounding rectangle around the largest contour then find its centroid
		//this will be the object's final estimated position.
		objectBoundingRectangle = boundingRect(largestContourVec.at(0));
		int xpos = objectBoundingRectangle.x + objectBoundingRectangle.width / 2;
		int ypos = objectBoundingRectangle.y + objectBoundingRectangle.height / 2;

		//update the objects positions by changing the 'theObject' array values
		theObject[0] = xpos, theObject[1] = ypos;
	}
	//make some temp x and y variables so we dont have to type out so much
	int x = theObject[0];
	int y = theObject[1];

	//draw some crosshairs around the object
	circle(cameraFeed, Point(x, y), 20, Scalar(0, 0, 255), 2);
	line(cameraFeed, Point(x, y), Point(x, y - 25), Scalar(0, 0, 255), 2);
	line(cameraFeed, Point(x, y), Point(x, y + 25), Scalar(0, 0, 255), 2);
	line(cameraFeed, Point(x, y), Point(x - 25, y), Scalar(0, 0, 255), 2);
	line(cameraFeed, Point(x, y), Point(x + 25, y), Scalar(0, 0, 255), 2);

	//write the position of the object to the screen
	putText(cameraFeed, "Tracking object at (" + intToString(x) + "," + intToString(y) + ")", Point(x, y), 1, 1, Scalar(255, 0, 0), 2);
	cout << "(" + intToString(x) + ", " + intToString(y) + ")" << endl;
}

////////////////////////////////////////////////////////////
HRESULT BodyIndexBasic::InitializeDefaultSensor(){
	HRESULT hr;
	hr = GetDefaultKinectSensor(&kinect);
	if (FAILED(hr)){
		cout << "获取Kinect失败" << endl;
		return E_FAIL;
	}
	if (kinect){
		IBodyIndexFrameSource * pBodyIndexFrameSource = NULL;
		IColorFrameSource * pColorFrameSource = NULL;
		IFrameDescription *pFrameDescription = NULL;
		hr = kinect->Open();
		if (FAILED(hr)){
			cout << "打开Kinect失败" << endl;
			return E_FAIL;
		}

		/*初始化BodyIndex*/
		hr = kinect->get_BodyIndexFrameSource(&pBodyIndexFrameSource);
		if (!SUCCEEDED(hr)){
			cout << "获取Body源失败" << endl;
			return E_FAIL;
		}

		hr = pBodyIndexFrameSource->OpenReader(&pBodyIndexFrameReader);
		if (!SUCCEEDED(hr)){
			cout << "打开读取器失败" << endl;
			return E_FAIL;
		}

		/*初始化RGB摄像头*/
		hr = kinect->get_ColorFrameSource(&pColorFrameSource);
		if (!SUCCEEDED(hr)){
			cout << "获取Color源失败" << endl;
			return E_FAIL;
		}
		hr = pColorFrameSource->OpenReader(&pColorFrameReader);
		if (!SUCCEEDED(hr)){
			cout << "打开color读取器失败" << endl;
			return E_FAIL;
		}
		
		height = 0;
		width = 0;
		ColorHeight = 0;
		ColorWidth = 0;

		/*BodyIndex图的分辨率--深度图的分辨率*/
		pBodyIndexFrameSource->get_FrameDescription(&pFrameDescription);
		pFrameDescription->get_Height(&height);
		pFrameDescription->get_Width(&width);
		cout << "width= " << width << " height= " << height << endl;
		SafeRelease(pFrameDescription);
		pFrameDescription = NULL;
		///*RGB图的分辨率*/
		pColorFrameSource->get_FrameDescription(&pFrameDescription);
		pFrameDescription->get_Height(&ColorHeight);
		pFrameDescription->get_Width(&ColorWidth);
		cout << "ColorWidth= " << ColorWidth << " ColorHeight= " << ColorHeight << endl;

		SafeRelease(pBodyIndexFrameSource);
		SafeRelease(pColorFrameSource);
		SafeRelease(pFrameDescription);
	}
	img.create(height, width, CV_8UC3);
	ColorImage.create(ColorHeight, ColorWidth,CV_8UC4);
	ColorImage.setTo(0);
	img.setTo(0);
	return hr;
}

void BodyIndexBasic::UpDate(){
	img.setTo(0);
	/*ColorImage.setTo(0);*/
	if (!pBodyIndexFrameReader)
	{
		return;
	}
	if (!pColorFrameReader){
		return;
	}

	HRESULT hr = S_OK;
	IBodyIndexFrame * pBodyIndexFrame = NULL;
	IColorFrame *pColorFrame = NULL;
	/*BodyIndex图像获取及显示*/
	if (SUCCEEDED(hr)){
		hr = pBodyIndexFrameReader->AcquireLatestFrame(&pBodyIndexFrame);
	}
	if (SUCCEEDED(hr)){
		//BYTE *bodyIndexArray = new BYTE[height* width];//背景二值图是8为uchar，有人是黑色，没人是白色
		//pBodyIndexFrame->CopyFrameDataToArray(height* width, bodyIndexArray);
		//uchar* Data = (uchar*)img.data;
		//for (int j = 0; j < height * width; ++j){
		//	*Data = bodyIndexArray[j]; ++Data;
		//	*Data = bodyIndexArray[j]; ++Data;
		//	*Data = bodyIndexArray[j]; ++Data;
		//}
		//delete[] bodyIndexArray;
		UINT nBufferSize = 0;
		unsigned char* pBuffer = nullptr;
		pBodyIndexFrame->AccessUnderlyingBuffer(&nBufferSize, &pBuffer);
		for (int x = 0; x < height; x++)
		{
			for (int y = 0; y < width; y++)

			{
				unsigned int index = x *width + y;
				if (pBuffer[index] != 255)
				{
					img.at<Vec3b>(x, y) = Vec3b(0, 255, 0);
				}
				else
				{
					img.at<Vec3b>(x, y) = Vec3b(0, 0, 0);
				}
			}
		}
	}
	SafeRelease(pBodyIndexFrame);
	thresholdImage.create(height, width, CV_8UC3);
	cvtColor(img, thresholdImage, COLOR_BGR2GRAY);
	threshold(thresholdImage, thresholdImage, SENSITIVITY_VALUE, 255, THRESH_BINARY);
	searchForMovement(thresholdImage,img);
	cvNamedWindow("BodyIndexImage");
	imshow("BodyIndexImage", img);

	/*Color图像获取及显示*/
	hr = pColorFrameReader->AcquireLatestFrame(&pColorFrame);
	UINT ColorBufferSize = 0;
	uchar* pColorBuffer = NULL;
	//pColorFrame->get_RawColorImageFormat(&imageFormat);
	//cout << "imageformat is " << imageFormat << endl;
	Mat ColorImage(ColorHeight, ColorWidth, CV_8UC4);
	pColorBuffer = ColorImage.data;
	ColorBufferSize = ColorImage.rows*ColorImage.step;
	hr = pColorFrame->CopyConvertedFrameDataToArray(ColorBufferSize, reinterpret_cast<BYTE*>(pColorBuffer), ColorImageFormat_Bgra);
	cvNamedWindow("ColorImage");
	imshow("ColorImage", ColorImage);
	SafeRelease(pColorFrame);

	if (waitKey(34) == VK_ESCAPE){
		exit(0);
	}
		
}

void BodyIndexBasic::BodyIndexProcess(){

}

void BodyIndexBasic::ColorImageProcess(){
	
}

BodyIndexBasic::BodyIndexBasic(){

}

BodyIndexBasic::~BodyIndexBasic(){
	SafeRelease(pBodyIndexFrameReader);
	SafeRelease(pColorFrameReader);
	if (kinect)
	{
		kinect->Close();
	}
	SafeRelease(kinect);
}
