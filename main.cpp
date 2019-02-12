
#include "opencv2/core.hpp"
#include <opencv2/core/utility.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"

#include <cctype>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <iostream>
#include <algorithm>
#include <iterator>

using namespace cv;              // Namespace where all the C++ OpenCV functionality resides.
using namespace std;             // For input output operations.

void display_vector( vector<int> &v) //show vectoe elements.
{
	int x;
	cout << "Enter five integer values" << endl;

	for (int i = 0; i < (int)v.size(); i++)
		cout << v.at(i) << endl;
}

int main()
{
	int numCornersHor=9;
	int numCornersVer=6;
	Mat gray_image;
	Mat image;

	int numSquares = numCornersHor * numCornersVer;
	Size board_sz = Size(numCornersHor, numCornersVer);
	vector<vector<Point3f>> object_points; //the physical position of the corners (in 3D space)
	vector<vector<Point2f>> image_points;  //the location of the corners on in the image (in 2 dimensions)

	vector<cv::String> fn;
	glob("D://Camera_cal_images//*.jpg", fn, false);

	vector<Mat> images;
	size_t nb_chessboard = fn.size(); //number of image files in folder
	for (size_t i = 0; i < nb_chessboard; i++)
		images.push_back(imread(fn[i]));


	vector<Point3f> obj;
	for (int j = 0; j < numSquares; j++)
		obj.push_back(Point3f(j / numCornersHor, j%numCornersHor, 0.0f));

	vector<Point2f> corners;
	int successes = 0;
	bool found = false;

	for (int k = 0; k < nb_chessboard; k++) {

		image = images[k];
		found = findChessboardCorners(image, board_sz, corners, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FAST_CHECK | CALIB_CB_NORMALIZE_IMAGE);
		if (found)
		{
			cvtColor(image, gray_image, COLOR_RGB2GRAY);
			cornerSubPix(gray_image, corners, Size(11, 11), Size(-1, -1), TermCriteria(TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.1)));
			//drawChessboardCorners(gray_image, board_sz, corners, found);
			//imshow("chessboard", gray_image);
			image_points.push_back(corners);
			object_points.push_back(obj);
			cout<<"Snap stored= "<<k<<endl;
		}

	}

	
	Mat intrinsic = Mat(3, 3, CV_32FC1);
	Mat distCoeffs;
	vector<Mat> rvecs;
	vector<Mat> tvecs;
	intrinsic.ptr<float>(0)[0] = 1;
	intrinsic.ptr<float>(1)[1] = 1;
	calibrateCamera(object_points, image_points, images[0].size(), intrinsic, distCoeffs, rvecs, tvecs);
	
	//********************************************************************************************
	VideoCapture cap("D://video//MOVI0003.avi");
	// cap is the object of class video capture that tries to capture Bumpy.mp4
	if (!cap.isOpened()) // isOpened() returns true if capturing has been initialized.
	{
		cout << "Cannot open the video file. \n";
		return -1;
	}

	//get the frames rate of the video
	double fps = cap.get(CAP_PROP_FPS);
	cout << "Frames per seconds : " << fps << endl;

	String window_name = "My First Video";
	namedWindow(window_name, WINDOW_NORMAL); //create a window
	Mat frame;// Mat object is a basic image container. frame is an object of Mat.
	Mat imageUndistorted;
	while (1)
	{
		
		if (!cap.read(frame)) // if not success, break loop, read() decodes and captures the next frame.
		{
			cout << "\n Cannot read the video file. \n";
			break;
		}
		//calibrateCamera(object_points, image_points, frame.size(), intrinsic, distCoeffs, rvecs, tvecs);
		undistort(frame, imageUndistorted, intrinsic, distCoeffs);
		imshow("Line_Detection", imageUndistorted);
		// first argument: name of the window.
		// second argument: image to be shown(Mat object).

		//if (waitKey(0) == 27) // Wait for 'esc' key press to exit
			//break;

	}

	cap.release();
	
	return 0;
}
// END OF PROGRAM