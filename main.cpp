#include "myKinect.h"
#include <iostream>
using namespace std;

int main()
{
	CBodyBasics myKinect;
	HRESULT hr = myKinect.InitializeDefaultSensor();
	bool flag = true;
	if (SUCCEEDED(hr)){
		while (flag){
			myKinect.Update();
		}
	}
	else{
		cout << "kinect initialization failed!" << endl;
		system("pause");
	}
}