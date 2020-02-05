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



    Initialize();
    Calibrate();
   // Birds_Eye_View();
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
    img_BGR=imread("Road_Snap5.png");
    cvtColor(img_BGR, img_RGB,COLOR_BGR2RGB); //convert to RGB

    imgWidth=img_BGR.cols;
    imgHeight=img_BGR.rows;

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

        // Open the image
        image = imread(filelist[i],0);

        // Get the chessboard corners
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

            // If we have a good board, add it to our data
            if (imageCorners.size() == boardSize.area()) {
                imagePoints.push_back(imageCorners);     // 2D image points from one view
                objectPoints.push_back(objectCorners);  // corresponding 3D scene points
                successes++;
            }
        }

        if (windowName.length()>0 && imageCorners.size() == boardSize.area()) {

            //Draw the corners
            cv::drawChessboardCorners(image, boardSize, imageCorners, found);
            cv::imshow(windowName, image);
            cv::waitKey(3000);
        }
    }


}



void MainWindow::Detect_Lane()
{

        //3-1-Binary Convertion
        //Declare the output variables
        Mat dst, cdst, cdstP;
        Mat src= img_Wrap.clone();

        cvtColor(img_Wrap,dst,COLOR_RGB2GRAY); //convert to GRAYSCALE
        cvtColor(img_Wrap,cdstP,COLOR_RGB2GRAY); //convert to GRAYSCALE
        // Edge detection
        qint16 Threshold_1=50;
        qint16 Threshold_2=220;
        Canny(src, dst, Threshold_1, Threshold_2, 3);
        imshow("Canny", dst);

//        // Standard Hough Line Transform
//        vector<Vec2f> lines; // will hold the results of the detection
//        HoughLines(dst, lines, 1, CV_PI/180, 250, 0, 0 ); // runs the actual detection
//        // Draw the lines
//        for( size_t i = 0; i < lines.size(); i++ )
//        {
//            float rho = lines[i][0], theta = lines[i][1];
//            Point pt1, pt2;
//            double a = cos(theta), b = sin(theta);
//            double x0 = a*rho, y0 = b*rho;
//            pt1.x = cvRound(x0 + 1000*(-b));
//            pt1.y = cvRound(y0 + 1000*(a));
//            pt2.x = cvRound(x0 - 1000*(-b));
//            pt2.y = cvRound(y0 - 1000*(a));
//            line( dst, pt1, pt2, Scalar(0,0,255), 3, LINE_AA);
//        }
//        imshow("Hough Transform", dst);


        Mat maskWhite;
        qint16 Threshold=200;
        inRange(img_Wrap, Scalar(Threshold, Threshold, Threshold), Scalar(255, 255, 255), maskWhite);
        imshow("maskWhite", maskWhite);







}


void MainWindow::Birds_Eye_View()
{
    //2-1-Define Source & Destination Points

    qint16 X_Top_Left=590;
    qint16 Y_Top_Left=450;

    qint16 X_Top_Right=690;
    qint16 Y_Top_Right=450;

    qint16 X_Bottom_Right=1060;
    qint16 Y_Bottom_Right=650;

    qint16 X_Bottom_Left=300;
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

    //2-2-Draw Rectangle
    line(img_BGR,PT1,PT2,Scalar( 255, 0, 0 ),1,LINE_8,0);
    line(img_BGR,PT2,PT3,Scalar( 255, 0, 0 ),1,LINE_8,0);
    line(img_BGR,PT3,PT4,Scalar( 255, 0, 0 ),1,LINE_8,0);
    line(img_BGR,PT4,PT1,Scalar( 255, 0, 0 ),1,LINE_8,0);

    imshow("Original Image", img_BGR);
    //**********************************************************//

    //2-3-Warp Perspective Image
    //Prepare the matrix for transform and get the warped image.

    Mat perspectiveMatrix=getPerspectiveTransform(src,dst);
    img_Wrap=Mat(480, 640, CV_8UC3); //Destination for warped image
    Mat invertedPerspectiveMatrix;
    invert(perspectiveMatrix, invertedPerspectiveMatrix);
    warpPerspective(img_RGB, img_Wrap, perspectiveMatrix, img_Wrap.size(), INTER_LINEAR, BORDER_CONSTANT);

    imshow("warpPerspective",img_Wrap);



}
void MainWindow::HSV_Threshold()
{
    //medianBlur(img_BGR, img_BGR, 3);
    // Threshold the HSV image, keep only the red pixels

    inRange(img_HSV, Scalar(10, 0, 10), Scalar(190, 5, 150), lower_red_hue_range);
//    inRange(img_HSV, Scalar(0, 10, 10), Scalar(5, 255, 255), lower_red_hue_range);
//    inRange(img_HSV, Scalar(10, 100, 100), Scalar(200, 255, 255), upper_red_hue_range);
//    addWeighted(lower_red_hue_range, 1.0, upper_red_hue_range, 1.0, 0.0, red_hue_image);
    imshow("HSV", img_HSV);
    imshow("Lower Red", lower_red_hue_range);
    //imshow("Upper Red", upper_red_hue_range);

    //GaussianBlur(red_hue_image, red_hue_image, Size(9, 9), 2, 2);
   //imshow("Red Hue", red_hue_image);

}

void MainWindow::Vertical_Lines()
{



}
void MainWindow::Adaptive_Threshold()
{
    adaptiveThreshold(~img_GRAY, img_Binary, 100, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, 15, -2);
    imshow("T1", img_Binary);

}

void MainWindow::Threshold_RGB_White()
{
    qint16 Threshold=140;
    inRange(img_RGB, Scalar(Threshold, Threshold, Threshold), Scalar(255, 255, 255), img_RGB);
    imshow("T1", img_RGB);

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

