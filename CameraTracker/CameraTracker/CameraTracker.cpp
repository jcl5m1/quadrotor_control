// CameraTracker.cpp : Defines the entry point for the console application.

#include "stdafx.h"
#include <Windows.h>
#include "Serial.h"
#include <stdio.h>
#include <iostream>
#include <opencv2\core\core.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\video\video.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#include <opencv2\features2d\features2d.hpp>

using namespace cv;
using namespace std;

Mat bgImg;
Mat fgImg;
Mat diffImg;
Mat displayImg;

char * bgFilename = "bgImg.png";

int frameCounter = 0;
int trackbarValue = 0;

int lift = 0;
int yaw = 128;
int pitch = 128;
int roll = 132;
int gyroDiv = 50;

#define ENABLE_SERIAL

#ifdef ENABLE_SERIAL
CSerial serial;

const int length = 8;
byte * packet = new byte[length];
bool resetGyro = false;
#endif


void SendQuadcopterData() {
#ifdef ENABLE_SERIAL
	packet[0] = 'g';
	packet[1] = lift;
	packet[2] = pitch;
	packet[3] = yaw;
	packet[4] = roll;
	packet[5] = gyroDiv;
	if (resetGyro)
		packet[6] = 'r';
	else
		packet[6] = 's';
	packet[7] = '\0';
	resetGyro = false;

	//printf("%d %d %d %d\n", lift, yaw, pitch, roll);

	if (serial.IsOpened()) {
		serial.SendData((const char *)packet, length);
	}
	else {
		printf("Serial not opened!!!\n");
	}
#endif
}


void onLiftChange(int v, void * ptr) {
	SendQuadcopterData();
}
void onYawChange(int v, void * ptr) {
	SendQuadcopterData();
}
void onPitchChange(int v, void * ptr) {
	SendQuadcopterData();
}
void onRollChange(int v, void * ptr) {
	SendQuadcopterData();
}
void onGyroDivChange(int v, void * ptr) {
	SendQuadcopterData();
}

int _tmain(int argc, _TCHAR* argv[])
{
	printf("Camera Tracker\n");

#ifdef ENABLE_SERIAL
	serial.Open(4, 57600);

	resetGyro = true;
	SendQuadcopterData();
#endif

	VideoCapture cap(0);

	Mat img;	

	bgImg = imread(bgFilename);

	if (!bgImg.empty()) {
		printf("Loaded BG img\n");
		frameCounter = 10;
	}

	char * imageWindowName = "Image";
	char * diffWindowName = "Diff";

	namedWindow(imageWindowName);
	namedWindow(diffWindowName);


	createTrackbar("Lift", imageWindowName, &lift, 255, onLiftChange);
	createTrackbar("Yaw", imageWindowName, &yaw, 255, onYawChange);
	createTrackbar("Pitch", imageWindowName, &pitch, 255, onPitchChange);
	createTrackbar("Roll", imageWindowName, &roll, 255, onRollChange);
	createTrackbar("GyroDiv", imageWindowName, &gyroDiv, 255, onGyroDivChange);

	while (1) {
		cap.read(img);

		if (img.empty()) {
			printf("Unable to grab frame\n");
			return 0;
		}

		if (frameCounter < 5)
			img.copyTo(bgImg);

		subtract(img, bgImg, fgImg);
		threshold(fgImg, fgImg, 150, 255, CV_THRESH_BINARY);

		imshow(imageWindowName, img);
		imshow(diffWindowName, fgImg);

		moveWindow(imageWindowName, 0, 0);
		moveWindow(diffWindowName, 0, img.rows + 300);

		char c = cvWaitKey(5);
		if (c == 27)
			break;
		if (c == 'r')
			frameCounter = 0;
		if (c == 'g') {
			resetGyro = true;
			SendQuadcopterData();
		}

		if (c == ' ') {
			lift = 0;
			SendQuadcopterData();
		}
		if (c == 's') {
			printf("Saving background image\n");
			imwrite(bgFilename, img);
		}

		//heartbeat
		if ((frameCounter % 15) == 0)
			SendQuadcopterData();

		frameCounter++;
	}


#ifdef ENABLE_SERIAL
	serial.Close();
#endif


	return 0;
}

