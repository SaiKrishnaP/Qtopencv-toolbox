#ifndef DIALOG_H
#define DIALOG_H

#include <QDialog>
#include <QFileDialog>


#include"opencv/cv.h"
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/features2d/features2d.hpp>
#include<opencv2/nonfree/features2d.hpp>
#include<opencv2/nonfree/nonfree.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include<stdlib.h>
#include<iostream>
#include<stdio.h>
#include <ctype.h>
#include <math.h>


namespace Ui {
class Dialog;
}

class Dialog : public QDialog
{
    Q_OBJECT
    
public:
    explicit Dialog(QWidget *parent = 0);
    ~Dialog();
    cv::Mat saltpepper();
    cv::Mat hist();
    cv::Mat eqhist();
    cv::Mat dilate();
    cv::Mat erode();
    cv::Mat cross();
    cv::Mat rect();
    cv::Mat ellipse();
    cv::Mat ope();
    cv::Mat close();
    cv::Mat median();
    cv::Mat guss_blur();
    cv::Mat homo_blur();
    cv::Mat bilat_blur();
    cv::Mat sobel();
    cv::Mat laplacian();
    cv::Mat kernel();
    cv::Mat canny();
    cv::Mat hou_line();
    cv::Mat hou_cir();
    cv::Mat contours();
    cv::Mat boun_box();
    cv::Mat min_encir();
    cv::Mat harris();
    cv::Mat shi();
    cv::Mat fast();
    cv::Mat surf();
    cv::Mat sift();
    cv::Mat convexhull();

private:
    Ui::Dialog *ui;

    cv::VideoCapture capwebcam;
    cv::Mat matOriginal;
    cv::Mat matProcessed;

    QImage qimgOriginal;
    QImage qimgProcessed;

    std::vector<cv::Vec3f> vecCircles;
    std::vector<cv::Vec3f>::iterator itrCircles;

    QTimer* tmrTimer;

    int var, im , threshh;


public slots:
    void processFrameAndupdateGUI();

private slots:
    void on_btnPauseOrResume_clicked();
    void on_comboBox_currentIndexChanged(int index);
    void on_comboBox_2_activated(int index);
    void on_comboBox_4_activated(int index);
    void on_comboBox_5_activated(int index);
    void on_comboBox_6_activated(int index);
    void on_comboBox_3_activated(int index);
    void on_pushButton_clicked();
    void on_pushButton_2_clicked();
    void on_pushButton_3_clicked();
    void on_comboBox_7_activated(int index);
    void on_horizontalSlider_valueChanged(int value);
};

#endif // DIALOG_H
