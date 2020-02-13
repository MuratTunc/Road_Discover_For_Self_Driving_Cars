#include <opencv4/opencv2/core/core.hpp>
#include <opencv4/opencv2/highgui/highgui.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv4/opencv2/imgproc/imgproc.hpp>
#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "stdio.h"
#include <QDebug>
#include <string>

#include <vector>
#include <algorithm>

using namespace cv;
using namespace std;
MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent) , ui(new Ui::MainWindow)
{
    ui->setupUi(this);



    Initialize();        //get image size
    Calibrate();         //get cameramatrix

    Binary_Threshold();
    Birds_Eye_View();    //get perspectivematrix

    //Detect_Lane();
    //ui->image_lbl->setPixmap(QPixmap::fromImage(QImage(img_HSV.data, img_HSV.cols, img_HSV.rows, img_HSV.step, QImage::Format_RGB444)));

}

MainWindow::~MainWindow()
{
    delete ui;
}
void MainWindow::Initialize()
{
    namedWindow("T1");
    //*********************************************************//
    img_BGR=imread("test_images/test4.jpg");
    //img_BGR=imread("WEBCAM.jpg");
    imshow("img_BGR",img_BGR);

    imgWidth=img_BGR.cols;
    imgHeight=img_BGR.rows;
    imageSize=img_BGR.size();
    outputSize=img_BGR.size();
    cout <<"imgWidth="<<imgWidth <<endl<<"imgHeight=" <<imgHeight<<endl;

}
void MainWindow::Calibrate(){

    vector<string> filelist;
    string windowName="Detected points";

    // generate list of calibration image filename
    // named calibration1 to chessboard20 in camera_cal sub-direction
    for (int i=1; i<=20; i++) {

        stringstream str;
        str << "camera_cal/calibration" << i << ".jpg";
        filelist.push_back(str.str());

    }

    //2-1********************************************************************************************//
    Size boardSize(9,6);
    // the points on the chessboard
    vector<Point2f> imageCorners;
    vector<Point3f> objectCorners;

    // 3D Scene Points:
    // Initialize the chessboard corners
    // in the chessboard reference frame
    // The corners are at 3D location (X,Y,Z)= (i,j,0)
    for (int i=0; i<boardSize.height; i++) {
        for (int j=0; j<boardSize.width; j++) {

            objectCorners.push_back(Point3f(i, j, 0.0f));
        }
    }

    // 2D Image points:
    Mat image; // to contain chessboard image
    int successes = 0;
    // for all viewpoints
    for (int i=0; i<filelist.size(); i++) {
        image = imread(filelist[i],0);
        bool found = findChessboardCorners(image,         // image of chessboard pattern
                                               boardSize,     // size of pattern
                                               imageCorners); // list of detected corners

        // Get subpixel accuracy on the corners
        if (found) {
            cornerSubPix(image, imageCorners,
                             cv::Size(5, 5), // half size of serach window
                             cv::Size(-1, -1),
                             cv::TermCriteria(cv::TermCriteria::MAX_ITER +
                                              cv::TermCriteria::EPS,
                                              30,		// max number of iterations
                                              0.1));  // min accuracy


            //2-2****************************************************************************************//
            // If we have a good board, add it to our data
            if (imageCorners.size() == boardSize.area()) {
                imagePoints.push_back(imageCorners);     // 2D image points from one view
                objectPoints.push_back(objectCorners);  // corresponding 3D scene points
                successes++;
            }
        }

//        if (windowName.length()>0 && imageCorners.size() == boardSize.area()) {

//            //Draw the corners
//            cv::drawChessboardCorners(image, boardSize, imageCorners, found);
//            cv::imshow(windowName, image);
//            cv::waitKey(3000);
//        }
    }


    //2-3********************************************************************************************//
    // Set the calibration options
    // 8radialCoeffEnabled should be true if 8 radial coefficients are required (5 is default)
    // tangentialParamEnabled should be true if tangeantial distortion is present

    bool tangentialParamEnabled=true;
    bool radial8CoeffEnabled=false;
    // Set the flag used in cv::calibrateCamera()
    bool flag = 0;
    if (!tangentialParamEnabled)
        flag += cv::CALIB_ZERO_TANGENT_DIST;
    if (radial8CoeffEnabled)
        flag += cv::CALIB_RATIONAL_MODEL;


    //Output rotations and translations
    vector<Mat> rvecs, tvecs;

    calibrateCamera(objectPoints, // the 3D points
                   imagePoints,  // the image points
                   imageSize,    // image size
                   cameraMatrix, // output camera matrix
                   distCoeffs,   // output distortion matrix
                   rvecs, tvecs, // Rs, Ts
                   flag);        // set options

//    std::cout << " Camera intrinsic: " << cameraMatrix.rows << "x" << cameraMatrix.cols << std::endl;
//    std::cout << cameraMatrix.at<double>(0,0) << " " << cameraMatrix.at<double>(0,1) << " " << cameraMatrix.at<double>(0,2) << std::endl;
//    std::cout << cameraMatrix.at<double>(1,0) << " " << cameraMatrix.at<double>(1,1) << " " << cameraMatrix.at<double>(1,2) << std::endl;
//    std::cout << cameraMatrix.at<double>(2,0) << " " << cameraMatrix.at<double>(2,1) << " " << cameraMatrix.at<double>(2,2) << std::endl;

    initUndistortRectifyMap(
                cameraMatrix,  // computed camera matrix
                distCoeffs,    // computed distortion matrix
                cv::Mat(),     // optional rectification (none)
                cv::Mat(),     // camera matrix to generate undistorted
                outputSize,    // size of undistorted
                CV_32FC1,      // type of output map
                map1, map2);   // the x and y mapping functions
    // Apply mapping functions
    remap(img_BGR, img_undistorted, map1, map2, INTER_LINEAR); // interpolation type
    //imshow("undistorted",img_undistorted);


}
void MainWindow::Birds_Eye_View()
{
    //2-1-Define Source & Destination Points

    qint16 X_Top_Left=530;
    qint16 Y_Top_Left=440;

    qint16 X_Top_Right=720;
    qint16 Y_Top_Right=440;

    qint16 X_Bottom_Right=1060;
    qint16 Y_Bottom_Right=650;

    qint16 X_Bottom_Left=180;
    qint16 Y_Bottom_Left=650;

    qint16 Dst_diff=20;

    Point2f src[]={
        Point2f(X_Top_Left,Y_Top_Left),
        Point2f(X_Top_Right,Y_Top_Right),
        Point2f(X_Bottom_Right,Y_Bottom_Right),
        Point2f(X_Bottom_Left,Y_Bottom_Left),
    };

    Point2f dst[]={
        Point2f(250,0),
        Point2f(896,0),
        Point2f(896,720),
        Point2f(250,720),
    };
    Point PT1,PT2,PT3,PT4;
    PT1.x=X_Top_Left;
    PT1.y=Y_Top_Left;

    PT2.x=X_Top_Right;
    PT2.y=Y_Top_Right;

    PT3.x=X_Bottom_Right;
    PT3.y=Y_Bottom_Right;

    PT4.x=X_Bottom_Left;
    PT4.y=Y_Bottom_Left;
    Scalar Color( 0, 0, 255 );

//    //2-2-Draw Rectangle
//    line(undistorted,PT1,PT2,Color,2,LINE_8,0);
//    line(undistorted,PT2,PT3,Color,2,LINE_8,0);
//    line(undistorted,PT3,PT4,Color,2,LINE_8,0);
//    line(undistorted,PT4,PT1,Color,2,LINE_8,0);

    //imshow("undistorted",undistorted);

    //**********************************************************//

    //2-3-Warp Perspective Image
    //Prepare the matrix for transform and get the warped image.

    perspectiveMatrix=getPerspectiveTransform(src,dst);
    invert(perspectiveMatrix, invertedPerspectiveMatrix);
    warpPerspective(img_Binary, img_Wrap, perspectiveMatrix, imageSize, INTER_LINEAR, BORDER_CONSTANT);
    imshow("img_Wrap", img_Wrap);

}

void MainWindow::Binary_Threshold()
{

    cvtColor(img_undistorted, img_HSL, COLOR_BGR2HLS);
    //imshow("img_HSL", img_HSL);

    // Detect the object based on YELLOW filler on HSV Wrap image
    Scalar Yellow_Low(10,100,100);
    Scalar Yellow_High(100,255,255);
    inRange(img_HSL, Yellow_Low, Yellow_High, img_Yellow_Filtered);
    //imshow("Yellow Filter", img_Yellow_Filtered);


    // Detect the object based on YELLOW filler on HSV Wrap image
    Scalar White_Low(200,200,200);
    Scalar White_High(255,255,255);
    inRange(img_undistorted, White_Low, White_High, img_White_Filtered);
    //imshow("White Filter", img_White_Filtered);

    bitwise_or(img_Yellow_Filtered,img_White_Filtered,img_Binary);
    imshow("img_Binary", img_Binary);


}

void MainWindow::Detect_Lane()
{
     //Gray Scale
    cvtColor(img_BGR,img_GRAY,COLOR_BGR2GRAY);
    imshow("img_GRAY",img_GRAY);

    //Blur image
    Mat Gaussian_Blur;
    GaussianBlur( img_GRAY, Gaussian_Blur, Size( 5, 5 ), 2, 2 );
    imshow("Gaussian_Blur",Gaussian_Blur);

    //Canny (Edge Detection)
    Mat Canny_Image;
    const int lowThreshold = 0;
    const int highThreshold = 100;
    const int kernel_size = 3;
    Canny( Gaussian_Blur, Canny_Image, lowThreshold, highThreshold, kernel_size );
    imshow("Canny_Image",Canny_Image);

}



void MainWindow::Vertical_Lines()
{



}


void MainWindow::Hough_Transform()
{

    // Canny Edge detection
    qint16 Threshold_1=200;
    qint16 Threshold_2=250;
    Canny(img_GRAY, img_dst, Threshold_1, Threshold_2, 3);

    imshow("T1", img_dst);
    // Copy edges to the images that will display the results in BGR
    cvtColor(img_dst, img_cdst, COLOR_GRAY2BGR);
    img_cdstP = img_cdst.clone();

    // Standard Hough Line Transform
    vector<Vec2f> lines; // will hold the results of the detection
    HoughLines(img_dst, lines, 1, CV_PI/180, 250, 0, 0 ); // runs the actual detection


    // Draw the lines
    for( size_t i = 0; i < lines.size(); i++ )
    {
        float rho = lines[i][0], theta = lines[i][1];
        Point pt1, pt2;
        double a = cos(theta), b = sin(theta);
        double x0 = a*rho, y0 = b*rho;
        pt1.x = cvRound(x0 + 1000*(-b));
        pt1.y = cvRound(y0 + 1000*(a));
        pt2.x = cvRound(x0 - 1000*(-b));
        pt2.y = cvRound(y0 - 1000*(a));
        line( img_cdst, pt1, pt2, Scalar(0,0,255), 3, LINE_AA);
    }

    // Probabilistic Line Transform
    vector<Vec4i> linesP; // will hold the results of the detection
    HoughLinesP(img_dst, linesP, 1, CV_PI/180, 50, 50, 5 ); // runs the actual detection
    // Draw the lines
    for( size_t i = 0; i < linesP.size(); i++ )
    {
        Vec4i l = linesP[i];
        line( img_cdstP, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, LINE_AA);
    }

    imshow("Hough Result",img_cdstP);

}

void MainWindow::show_wait_destroy(const char* winname, Mat img) {
    imshow(winname, img);
    moveWindow(winname, 500, 0);
    waitKey(0);
    destroyWindow(winname);
}

