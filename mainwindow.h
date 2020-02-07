#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <opencv4/opencv2/opencv.hpp>


QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE
using namespace std;
using namespace cv;
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    void Threshold_RGB_White();
    void Threshold_RGB_Yellow();
    void Hough_Transform();
    void HSV_Threshold();
    void Adaptive_Threshold();
    void Vertical_Lines();
    void show_wait_destroy(const char* winname, Mat img);
    void Birds_Eye_View();
    void Initialize();
    void Detect_Lane();
    void Calibrate();


private slots:

    
private:
    Ui::MainWindow *ui;
    Mat img_BGR;
    Mat img_RGB;
    Mat img_GRAY;
    Mat img_HSV;
    Mat img_dst;
    Mat img_cdst;
    Mat img_cdstP;
    Mat img_Binary;
    Mat lower_red_hue_range;
    Mat upper_red_hue_range;
    Mat red_hue_image;
    Mat img_Wrap;
    qint16 imgWidth;
    qint16 imgHeight;

    vector<vector<Point3f> > objectPoints;     // the image point positions in pixels
    vector<vector<Point2f> > imagePoints;     // output Matrices
    Mat cameraMatrix;
    Mat distCoeffs;
    Mat map1,map2;
    Size imageSize;
    Size outputSize;
    Mat undistorted;
};

#endif // MAINWINDOW_H
