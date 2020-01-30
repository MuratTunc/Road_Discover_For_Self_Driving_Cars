#include <opencv4/opencv2/core/core.hpp>
#include <opencv4/opencv2/highgui/highgui.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv4/opencv2/imgproc/imgproc.hpp>
#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "stdio.h"
#include<QDebug>
#include<string>

#include <vector>
#include <algorithm>

using namespace cv;
using namespace std;
MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent) , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    namedWindow("T1");
    //*********************************************************//
    img_BGR=imread("Road_Snap2.png");
    cvtColor(img_BGR, img_RGB,COLOR_BGR2RGB); //convert to RGB
    cvtColor(img_BGR, img_GRAY,COLOR_BGR2GRAY); //convert to GRAY
    cvtColor(img_BGR, img_HSV,COLOR_BGR2HSV); //convert to GRAY;


    imgWidth=img_BGR.cols;
    imgHeight=img_BGR.rows;

    //qDebug()<<imgHeight;


    Birds_Eye_View();



    //Adaptive_Threshold();
    //Threshold_RGB_White();
    //Hough_Transform();
    //HSV_Threshold();


    //ui->image_lbl->setPixmap(QPixmap::fromImage(QImage(img_HSV.data, img_HSV.cols, img_HSV.rows, img_HSV.step, QImage::Format_RGB444)));

}

MainWindow::~MainWindow()
{
    delete ui;
}
void MainWindow::Birds_Eye_View()
{


    qint16 X_Top_Left=560;
    qint16 Y_Top_Left=415;

    qint16 X_Top_Right=710;
    qint16 Y_Top_Right=415;

    qint16 X_Bottom_Right=940;
    qint16 Y_Bottom_Right=600;

    qint16 X_Bottom_Left=340;
    qint16 Y_Bottom_Left=600;

    qint16 Dst_diff=20;

    Point2f src[]={
        Point2f(X_Top_Left,Y_Top_Left),
        Point2f(X_Top_Right,Y_Top_Right),
        Point2f(X_Bottom_Right,Y_Bottom_Right),
        Point2f(X_Bottom_Left,Y_Bottom_Left),
    };

    Point2f dst[]={
        Point2f(0,0),
        Point2f(640,0),
        Point2f(640,480),
        Point2f(0,480),
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

    line(img_BGR,PT1,PT2,Scalar( 255, 0, 0 ),1,LINE_8,0);
    line(img_BGR,PT2,PT3,Scalar( 255, 0, 0 ),1,LINE_8,0);
    line(img_BGR,PT3,PT4,Scalar( 255, 0, 0 ),1,LINE_8,0);
    line(img_BGR,PT4,PT1,Scalar( 255, 0, 0 ),1,LINE_8,0);

    imshow("Original Image", img_BGR);


    //Prepare the matrix for transform and get the warped image.

    Mat perspectiveMatrix=getPerspectiveTransform(src,dst);
    Mat img_Wrap(480, 640, CV_8UC3); //Destination for warped image
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

