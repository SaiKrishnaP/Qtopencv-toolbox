#include "dialog.h"
#include "ui_dialog.h"

#include<QtCore>


Dialog::Dialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Dialog)
{
    ui->setupUi(this);

    capwebcam.open(0);

    if(capwebcam.isOpened() == false)
    {
        ui->txtXYRadius->appendPlainText("Error: Webcam not acessed sucessfully");
        return;
    }

    tmrTimer = new QTimer(this);
    connect(tmrTimer, SIGNAL(timeout()), this, SLOT(processFrameAndupdateGUI()));
            tmrTimer->start(20);
}

Dialog::~Dialog()
{
    delete ui;
}

void Dialog::processFrameAndupdateGUI()
{
    if (im == 2)
    {
        capwebcam.read(matOriginal);
    }
//    capwebcam.read(matOriginal);
    if(matOriginal.empty() == true) return;

    cv::cvtColor(matOriginal, matOriginal, CV_BGR2RGB);

    int input;
    input = var;

  switch (input)
  {
    case 1: // Salt and Pepper Noise
    {
      matProcessed = saltpepper();
       QImage qimgProcessed((uchar*)matProcessed.data, matProcessed.cols, matProcessed.rows, matProcessed.step, QImage::Format_RGB888);
         ui->lblProcessed->setPixmap(QPixmap::fromImage(qimgProcessed));
     }break;

       case 2: // Gray
        {
          cv::cvtColor( matOriginal, matProcessed, CV_RGB2GRAY);
           QImage qimgProcessed((uchar*)matProcessed.data, matProcessed.cols, matProcessed.rows, matProcessed.step, QImage::Format_Indexed8);
            ui->lblProcessed->setPixmap(QPixmap::fromImage(qimgProcessed));
        }break;

       case 3: // HLS
        {
          cv::cvtColor( matOriginal, matProcessed, CV_RGB2HLS);
          QImage qimgProcessed((uchar*)matProcessed.data, matProcessed.cols, matProcessed.rows, matProcessed.step, QImage::Format_RGB888);
           ui->lblProcessed->setPixmap(QPixmap::fromImage(qimgProcessed));
         }break;

       case 4: //LAB
        {
          cv::cvtColor( matOriginal, matProcessed, CV_RGB2Lab);
           QImage qimgProcessed((uchar*)matProcessed.data, matProcessed.cols, matProcessed.rows, matProcessed.step, QImage::Format_RGB888);
            ui->lblProcessed->setPixmap(QPixmap::fromImage(qimgProcessed));
        }break;

       case 5: // RGB
        {
          cv::cvtColor( matOriginal, matProcessed, CV_BGR2RGB);
           QImage qimgProcessed((uchar*)matProcessed.data, matProcessed.cols, matProcessed.rows, matProcessed.step, QImage::Format_RGB888);
            ui->lblProcessed->setPixmap(QPixmap::fromImage(qimgProcessed));
        }break;

       case 6: //HSV
        {
          cv::cvtColor( matOriginal, matProcessed, CV_RGB2HSV);
           QImage qimgProcessed((uchar*)matProcessed.data, matProcessed.cols, matProcessed.rows, matProcessed.step, QImage::Format_RGB888);
            ui->lblProcessed->setPixmap(QPixmap::fromImage(qimgProcessed));
        }break;

       case 7: //YCrCb
        {
          cv::cvtColor( matOriginal, matProcessed, CV_RGB2YCrCb);
           QImage qimgProcessed((uchar*)matProcessed.data, matProcessed.cols, matProcessed.rows, matProcessed.step, QImage::Format_RGB888);
            ui->lblProcessed->setPixmap(QPixmap::fromImage(qimgProcessed));
        }break;

       case 8: //YUV
        {
          cv::cvtColor( matOriginal, matProcessed, CV_RGB2YUV);
           QImage qimgProcessed((uchar*)matProcessed.data, matProcessed.cols, matProcessed.rows, matProcessed.step, QImage::Format_RGB888);
            ui->lblProcessed->setPixmap(QPixmap::fromImage(qimgProcessed));
        }break;

    case 9: //Histogram
     {
       matProcessed = hist();
        QImage qimgProcessed((uchar*)matProcessed.data, matProcessed.cols, matProcessed.rows, matProcessed.step, QImage::Format_RGB888);
         ui->lblProcessed->setPixmap(QPixmap::fromImage(qimgProcessed));
     }break;

    case 10: //Equalise Histogram
     {
       matProcessed = eqhist();
        QImage qimgProcessed((uchar*)matProcessed.data, matProcessed.cols, matProcessed.rows, matProcessed.step, QImage::Format_Indexed8);
        ui->lblProcessed->setPixmap(QPixmap::fromImage(qimgProcessed));
     }break;

    case 11: // Dilate
     {
       matProcessed = dilate();
        QImage qimgProcessed((uchar*)matProcessed.data, matProcessed.cols, matProcessed.rows, matProcessed.step, QImage::Format_RGB888);
         ui->lblProcessed->setPixmap(QPixmap::fromImage(qimgProcessed));
     }break;

    case 12: //Erode
     {
       matProcessed = erode();
        QImage qimgProcessed((uchar*)matProcessed.data, matProcessed.cols, matProcessed.rows, matProcessed.step, QImage::Format_RGB888);
         ui->lblProcessed->setPixmap(QPixmap::fromImage(qimgProcessed));
     }break;

    case 13: // Cross
     {
       matProcessed = cross();
       QImage qimgProcessed((uchar*)matProcessed.data, matProcessed.cols, matProcessed.rows, matProcessed.step, QImage::Format_RGB888);
        ui->lblProcessed->setPixmap(QPixmap::fromImage(qimgProcessed));
     }break;

    case 14: // Ellipse
     {
       matProcessed = ellipse();
       QImage qimgProcessed((uchar*)matProcessed.data, matProcessed.cols, matProcessed.rows, matProcessed.step, QImage::Format_RGB888);
        ui->lblProcessed->setPixmap(QPixmap::fromImage(qimgProcessed));
     }break;

    case 15: //Open
     {
       matProcessed = ope();
       QImage qimgProcessed((uchar*)matProcessed.data, matProcessed.cols, matProcessed.rows, matProcessed.step, QImage::Format_RGB888);
        ui->lblProcessed->setPixmap(QPixmap::fromImage(qimgProcessed));
     }break;

    case 16: // Rect
     {
     matProcessed = rect();
       QImage qimgProcessed((uchar*)matProcessed.data, matProcessed.cols, matProcessed.rows, matProcessed.step, QImage::Format_RGB888);
        ui->lblProcessed->setPixmap(QPixmap::fromImage(qimgProcessed));
     }break;

 case 17: //Close
 {
     matProcessed = close();
       QImage qimgProcessed((uchar*)matProcessed.data, matProcessed.cols, matProcessed.rows, matProcessed.step, QImage::Format_RGB888);
        ui->lblProcessed->setPixmap(QPixmap::fromImage(qimgProcessed));

   }break;

    case 18:  // Homogenous Blur
     {
        matProcessed = homo_blur();
       QImage qimgProcessed((uchar*)matProcessed.data, matProcessed.cols, matProcessed.rows, matProcessed.step, QImage::Format_RGB888);
       ui->lblProcessed->setPixmap(QPixmap::fromImage(qimgProcessed));
     }break;

    case 19: // Gaussian Blur
     {
       matProcessed = guss_blur();
        QImage qimgProcessed((uchar*)matProcessed.data, matProcessed.cols, matProcessed.rows, matProcessed.step, QImage::Format_RGB888);
         ui->lblProcessed->setPixmap(QPixmap::fromImage(qimgProcessed));
     }break;

    case 20: // Median Blur
     {
       matProcessed = median();
        QImage qimgProcessed((uchar*)matProcessed.data, matProcessed.cols, matProcessed.rows, matProcessed.step, QImage::Format_RGB888);
        ui->lblProcessed->setPixmap(QPixmap::fromImage(qimgProcessed));
     }break;

    case 21: // Bilateral filter blur
     {
       matProcessed = bilat_blur();
        QImage qimgProcessed((uchar*)matProcessed.data, matProcessed.cols, matProcessed.rows, matProcessed.step, QImage::Format_RGB888);
          ui->lblProcessed->setPixmap(QPixmap::fromImage(qimgProcessed));
     }break;

    case 22: // Sobel
    {
      matProcessed = sobel();
       QImage qimgProcessed((uchar*)matProcessed.data, matProcessed.cols, matProcessed.rows, matProcessed.step, QImage::Format_Indexed8);
        ui->lblProcessed->setPixmap(QPixmap::fromImage(qimgProcessed));
    }break;

    case 23: // Laplacian
    {
      matProcessed = laplacian();
       QImage qimgProcessed((uchar*)matProcessed.data, matProcessed.cols, matProcessed.rows, matProcessed.step, QImage::Format_Indexed8);
        ui->lblProcessed->setPixmap(QPixmap::fromImage(qimgProcessed));
    }break;
    case 24: //Sharpen Image using Kernel
    {
      matProcessed = kernel();
       QImage qimgProcessed((uchar*)matProcessed.data, matProcessed.cols, matProcessed.rows, matProcessed.step, QImage::Format_RGB888);
        ui->lblProcessed->setPixmap(QPixmap::fromImage(qimgProcessed));
    }break;

  case 25: // Canny
    {
      matProcessed = canny();
       QImage qimgProcessed((uchar*)matProcessed.data, matProcessed.cols, matProcessed.rows, matProcessed.step, QImage::Format_Indexed8);
        ui->lblProcessed->setPixmap(QPixmap::fromImage(qimgProcessed));
    }break;

    case 26:  // Hough Lines
    {
        matProcessed = hou_line();
         QImage qimgProcessed((uchar*)matProcessed.data, matProcessed.cols, matProcessed.rows, matProcessed.step, QImage::Format_Indexed8);
          ui->lblProcessed->setPixmap(QPixmap::fromImage(qimgProcessed));
    }break;

    case 27:  //  Hough Circles
    {
      matProcessed = hou_cir();
      QImage qimgProcessed((uchar*)matProcessed.data, matProcessed.cols, matProcessed.rows, matProcessed.step, QImage::Format_Indexed8);
       ui->lblProcessed->setPixmap(QPixmap::fromImage(qimgProcessed));
    }break;
    case 28: // contours
    {
       matProcessed = contours();
        QImage qimgProcessed((uchar*)matProcessed.data, matProcessed.cols, matProcessed.rows, matProcessed.step, QImage::Format_RGB888);
         ui->lblProcessed->setPixmap(QPixmap::fromImage(qimgProcessed));
    }break;

    case 29: // Bounding Box
    {
        matProcessed = boun_box();
         QImage qimgProcessed((uchar*)matProcessed.data, matProcessed.cols, matProcessed.rows, matProcessed.step, QImage::Format_RGB888);
          ui->lblProcessed->setPixmap(QPixmap::fromImage(qimgProcessed));
    }break;

    case 30: //minimum enclosing circle
    {
      matProcessed = min_encir();
      QImage qimgProcessed((uchar*)matProcessed.data, matProcessed.cols, matProcessed.rows, matProcessed.step, QImage::Format_RGB888);
      ui->lblProcessed->setPixmap(QPixmap::fromImage(qimgProcessed));
    }break;

  case 31: // Harris Cornor Detector
  {
      matProcessed = harris();
       QImage qimgProcessed((uchar*)matProcessed.data, matProcessed.cols, matProcessed.rows, matProcessed.step, QImage::Format_Indexed8);
         ui->lblProcessed->setPixmap(QPixmap::fromImage(qimgProcessed));

  }break;

  case 32: // Shi-Tomasi corner detector
  {
      matProcessed = shi();
        QImage qimgProcessed((uchar*)matProcessed.data, matProcessed.cols, matProcessed.rows, matProcessed.step, QImage::Format_RGB888);
          ui->lblProcessed->setPixmap(QPixmap::fromImage(qimgProcessed));
  }break;

  case 33:  //Fast
  {
      matProcessed = fast();
       QImage qimgProcessed((uchar*)matProcessed.data, matProcessed.cols, matProcessed.rows, matProcessed.step, QImage::Format_Indexed8);
         ui->lblProcessed->setPixmap(QPixmap::fromImage(qimgProcessed));
  }break;

  case 34: //Surf
  {
    matProcessed = surf();
     QImage qimgProcessed((uchar*)matProcessed.data, matProcessed.cols, matProcessed.rows, matProcessed.step, QImage::Format_RGB888);
        ui->lblProcessed->setPixmap(QPixmap::fromImage(qimgProcessed));
  }break;

  case 35: // Sift
  {
      matProcessed = sift();
        QImage qimgProcessed((uchar*)matProcessed.data, matProcessed.cols, matProcessed.rows, matProcessed.step, QImage::Format_RGB888);
         ui->lblProcessed->setPixmap(QPixmap::fromImage(qimgProcessed));
  }break;

  case 36: // Convex Hull
  {
    matProcessed = convexhull();
     QImage qimgProcessed((uchar*)matProcessed.data, matProcessed.cols, matProcessed.rows, matProcessed.step, QImage::Format_RGB888);
      ui->lblProcessed->setPixmap(QPixmap::fromImage(qimgProcessed));
  }break;

}

   QImage qimgOriginal((uchar*)matOriginal.data, matOriginal.cols, matOriginal.rows, matOriginal.step, QImage::Format_RGB888);
//   QImage qimgProcessed((uchar*)matProcessed.data, matProcessed.cols, matProcessed.rows, matProcessed.step, QImage::Format_RGB888);

   ui->lblOriginal->setPixmap(QPixmap::fromImage(qimgOriginal));
//   ui->lblProcessed->setPixmap(QPixmap::fromImage(qimgProcessed));
}

void Dialog::on_btnPauseOrResume_clicked()
{
    if(tmrTimer->isActive() == true )
    {
        tmrTimer->stop();
        ui->btnPauseOrResume->setText("resume");
    }
    else {
        tmrTimer->start(20);
        ui->btnPauseOrResume->setText("pause");
    }
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////                                                /////////////////////////
////////////////////////////////////////////////////////////////         ComboBoxes and Switch Cases            /////////////////////////
///////////////////////////////////////////////////////////////                                                 /////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Dialog::on_comboBox_currentIndexChanged(int index)
{
    im = index;

    if (im == 1)
    {
        capwebcam.read(matOriginal);
    }
}

void Dialog::on_comboBox_2_activated(int index)  //////// salt and pepper, Histogram and Color Spaces combobox and switch cases ///////////////////////////////////
{
   var = index ; // 1- 10
}

void Dialog::on_comboBox_4_activated(int index) ///////////////  Morphological Operations  combobox and switch case  /////////////////////////////////////////////
{
    var = index + 10 ; // 11-17
}

void Dialog::on_comboBox_5_activated(int index) ////////////////// Diverse Linear Filters(Blur)  combobox and switch case  /////////////////////////////////////////
{
    var = index +  17 ; // 18-21
}

void Dialog::on_comboBox_3_activated(int index)//////////////////// Shape Descriptors or detectors combobox and switch case /////////////
{
    var = index + 21 ; // 22-30
}

void Dialog::on_comboBox_6_activated(int index) //////////////////// Feature detectors or extractors combobox and switch case ///////////
{
    var = index + 30 ; // 31-36
}


void Dialog::on_comboBox_7_activated(int index)
{
    switch(index)
    {
    case 1: //  Fundamental matrix 7 point method
    {
        cv::Mat img1, img2;

        QString filename1 = QFileDialog::getOpenFileName(
                    this,
                    tr("Select Image(s) to Open"),
                    QDir::toNativeSeparators(QDir::homePath()),
                    tr("Images (*.png *.bmp *.jpg *.jpeg *.tiff)"));

         if (!filename1.isEmpty())
                    {
                    img1 = cv::imread(filename1.toStdString());
                    }
         QString filename2 = QFileDialog::getOpenFileName(
                     this,
                     tr("Select Image(s) to Open"),
                     QDir::toNativeSeparators(QDir::homePath()),
                     tr("Images (*.png *.bmp *.jpg *.jpeg *.tiff)"));

          if (!filename2.isEmpty())
                     {
                     img2 = cv::imread(filename2.toStdString());
                     }

          cv::Mat img_1 = img1;
          cv::Mat img_2 = img2;
          cv::Mat gray_image1;
          cv::Mat gray_image2;

          cv::cvtColor(img_1, gray_image1, CV_RGB2GRAY);
          cv::cvtColor(img_2, gray_image2, CV_RGB2GRAY);

          int minHessian =400;
          cv::SurfFeatureDetector detector( minHessian);
          std::vector<cv::KeyPoint> KeyPoints_1, KeyPoints_2;
          detector.detect( gray_image1, KeyPoints_1 );
          detector.detect( gray_image2, KeyPoints_2 );

          cv::SurfDescriptorExtractor extractor;
          cv::Mat descriptors_1, descriptors_2;
          extractor.compute( gray_image1, KeyPoints_1, descriptors_1);
          extractor.compute( gray_image2, KeyPoints_2, descriptors_2);

      //    cv::BFMatcher matcher(cv::NORM_L2);
          cv::FlannBasedMatcher matcher;
          std::vector< cv::DMatch >matches;
          matcher.match( descriptors_1, descriptors_2, matches );

          cv::Mat img_matches;
          cv::drawMatches( img_1, KeyPoints_1, img_2, KeyPoints_2, matches, img_matches );

          double max_dist = 0; double min_dist = 100;

          //-- Quick calculation of max and min distances between keypoints
           for( int i = 0; i < descriptors_1.rows; i++ )
           { double dist = matches[i].distance;
           if( dist < min_dist ) min_dist = dist;
           if( dist > max_dist ) max_dist = dist;
           }

          //-- Use only "good" matches (i.e. whose distance is less than 3*min_dist )
           std::vector< cv::DMatch > good_matches;

          for( int i = 0; i < descriptors_1.rows; i++ )
           { if( matches[i].distance < 3*min_dist )
           { good_matches.push_back( matches[i]); }
           }
           std::vector< cv::Point2f > obj;
           std::vector< cv::Point2f > scene;

          for( int i = 0; i < good_matches.size(); i++ )
           {
           //-- Get the keypoints from the good matches
           obj.push_back( KeyPoints_1[ good_matches[i].queryIdx ].pt );
           scene.push_back( KeyPoints_2[ good_matches[i].trainIdx ].pt );
           }

          cv::Mat fundamental_matrix = cvCreateMat(3,3,CV_32FC1);
          cv::Mat fundamental = cv::findFundamentalMat(obj, scene, fundamental_matrix, CV_FM_7POINT, 3.0, 0.99);
          ui->txtXYRadius->appendPlainText(QString("The Fundamental Matrix using Ransac method is "));
          std::cout << fundamental << std::endl;

          matOriginal = img_1;
          matProcessed = img_2;

          QImage qimgOriginal((uchar*)matOriginal.data, matOriginal.cols, matOriginal.rows, matOriginal.step, QImage::Format_RGB888);
          QImage qimgProcessed((uchar*)matProcessed.data, matProcessed.cols, matProcessed.rows, matProcessed.step, QImage::Format_RGB888);

          ui->lblOriginal->setPixmap(QPixmap::fromImage(qimgOriginal));
          ui->lblProcessed->setPixmap(QPixmap::fromImage(qimgProcessed));

    }break;

    case 2: //  Fundamental matrix 8 point method
    {
        cv::Mat img1, img2;

        QString filename1 = QFileDialog::getOpenFileName(
                    this,
                    tr("Select Image(s) to Open"),
                    QDir::toNativeSeparators(QDir::homePath()),
                    tr("Images (*.png *.bmp *.jpg *.jpeg *.tiff)"));

         if (!filename1.isEmpty())
                    {
                    img1 = cv::imread(filename1.toStdString());
                    }
         QString filename2 = QFileDialog::getOpenFileName(
                     this,
                     tr("Select Image(s) to Open"),
                     QDir::toNativeSeparators(QDir::homePath()),
                     tr("Images (*.png *.bmp *.jpg *.jpeg *.tiff)"));

          if (!filename2.isEmpty())
                     {
                     img2 = cv::imread(filename2.toStdString());
                     }

          cv::Mat img_1 = img1;
          cv::Mat img_2 = img2;
          cv::Mat gray_image1;
          cv::Mat gray_image2;

          cv::cvtColor(img_1, gray_image1, CV_RGB2GRAY);
          cv::cvtColor(img_2, gray_image2, CV_RGB2GRAY);

          int minHessian =400;
          cv::SurfFeatureDetector detector( minHessian);
          std::vector<cv::KeyPoint> KeyPoints_1, KeyPoints_2;
          detector.detect( gray_image1, KeyPoints_1 );
          detector.detect( gray_image2, KeyPoints_2 );

          cv::SurfDescriptorExtractor extractor;
          cv::Mat descriptors_1, descriptors_2;
          extractor.compute( gray_image1, KeyPoints_1, descriptors_1);
          extractor.compute( gray_image2, KeyPoints_2, descriptors_2);

      //    cv::BFMatcher matcher(cv::NORM_L2);
          cv::FlannBasedMatcher matcher;
          std::vector< cv::DMatch >matches;
          matcher.match( descriptors_1, descriptors_2, matches );

          cv::Mat img_matches;
          cv::drawMatches( img_1, KeyPoints_1, img_2, KeyPoints_2, matches, img_matches );

          double max_dist = 0; double min_dist = 100;

          //-- Quick calculation of max and min distances between keypoints
           for( int i = 0; i < descriptors_1.rows; i++ )
           { double dist = matches[i].distance;
           if( dist < min_dist ) min_dist = dist;
           if( dist > max_dist ) max_dist = dist;
           }

          //-- Use only "good" matches (i.e. whose distance is less than 3*min_dist )
           std::vector< cv::DMatch > good_matches;

          for( int i = 0; i < descriptors_1.rows; i++ )
           { if( matches[i].distance < 3*min_dist )
           { good_matches.push_back( matches[i]); }
           }
           std::vector< cv::Point2f > obj;
           std::vector< cv::Point2f > scene;

          for( int i = 0; i < good_matches.size(); i++ )
           {
           //-- Get the keypoints from the good matches
           obj.push_back( KeyPoints_1[ good_matches[i].queryIdx ].pt );
           scene.push_back( KeyPoints_2[ good_matches[i].trainIdx ].pt );
           }

          cv::Mat fundamental_matrix = cvCreateMat(3,3,CV_32FC1);
          cv::Mat fundamental = cv::findFundamentalMat(obj, scene, fundamental_matrix, CV_FM_8POINT, 3.0, 0.99);
          ui->txtXYRadius->appendPlainText(QString("The Fundamental Matrix using Ransac method is "));
          std::cout << fundamental << std::endl;

          matOriginal = img_1;
          matProcessed = img_2;
          QImage qimgOriginal((uchar*)matOriginal.data, matOriginal.cols, matOriginal.rows, matOriginal.step, QImage::Format_RGB888);
          QImage qimgProcessed((uchar*)matProcessed.data, matProcessed.cols, matProcessed.rows, matProcessed.step, QImage::Format_RGB888);

          ui->lblOriginal->setPixmap(QPixmap::fromImage(qimgOriginal));
          ui->lblProcessed->setPixmap(QPixmap::fromImage(qimgProcessed));


    }break;

    case 3: //  Fundamental matrix RANSAC method
    {
        cv::Mat img1, img2;

        QString filename1 = QFileDialog::getOpenFileName(
                    this,
                    tr("Select Image(s) to Open"),
                    QDir::toNativeSeparators(QDir::homePath()),
                    tr("Images (*.png *.bmp *.jpg *.jpeg *.tiff)"));

         if (!filename1.isEmpty())
                    {
                    img1 = cv::imread(filename1.toStdString());
                    }
         QString filename2 = QFileDialog::getOpenFileName(
                     this,
                     tr("Select Image(s) to Open"),
                     QDir::toNativeSeparators(QDir::homePath()),
                     tr("Images (*.png *.bmp *.jpg *.jpeg *.tiff)"));

          if (!filename2.isEmpty())
                     {
                     img2 = cv::imread(filename2.toStdString());
                     }

          cv::Mat img_1 = img1;
          cv::Mat img_2 = img2;
          cv::Mat gray_image1;
          cv::Mat gray_image2;

          cv::cvtColor(img_1, gray_image1, CV_RGB2GRAY);
          cv::cvtColor(img_2, gray_image2, CV_RGB2GRAY);

          int minHessian =400;
          cv::SurfFeatureDetector detector( minHessian);
          std::vector<cv::KeyPoint> KeyPoints_1, KeyPoints_2;
          detector.detect( gray_image1, KeyPoints_1 );
          detector.detect( gray_image2, KeyPoints_2 );

          cv::SurfDescriptorExtractor extractor;
          cv::Mat descriptors_1, descriptors_2;
          extractor.compute( gray_image1, KeyPoints_1, descriptors_1);
          extractor.compute( gray_image2, KeyPoints_2, descriptors_2);

      //    cv::BFMatcher matcher(cv::NORM_L2);
          cv::FlannBasedMatcher matcher;
          std::vector< cv::DMatch >matches;
          matcher.match( descriptors_1, descriptors_2, matches );

          cv::Mat img_matches;
          cv::drawMatches( img_1, KeyPoints_1, img_2, KeyPoints_2, matches, img_matches );

          double max_dist = 0; double min_dist = 100;

          //-- Quick calculation of max and min distances between keypoints
           for( int i = 0; i < descriptors_1.rows; i++ )
           { double dist = matches[i].distance;
           if( dist < min_dist ) min_dist = dist;
           if( dist > max_dist ) max_dist = dist;
           }

          //-- Use only "good" matches (i.e. whose distance is less than 3*min_dist )
           std::vector< cv::DMatch > good_matches;

          for( int i = 0; i < descriptors_1.rows; i++ )
           { if( matches[i].distance < 3*min_dist )
           { good_matches.push_back( matches[i]); }
           }
           std::vector< cv::Point2f > obj;
           std::vector< cv::Point2f > scene;

          for( int i = 0; i < good_matches.size(); i++ )
           {
           //-- Get the keypoints from the good matches
           obj.push_back( KeyPoints_1[ good_matches[i].queryIdx ].pt );
           scene.push_back( KeyPoints_2[ good_matches[i].trainIdx ].pt );
           }

          cv::Mat fundamental_matrix = cvCreateMat(3,3,CV_32FC1);
          cv::Mat fundamental = cv::findFundamentalMat(obj, scene, fundamental_matrix, CV_FM_RANSAC, 3.0, 0.99);
          ui->txtXYRadius->appendPlainText(QString("The Fundamental Matrix using Ransac method is "));
          std::cout << fundamental << std::endl;

          matOriginal = img_1;
          matProcessed = img_2;

          QImage qimgOriginal((uchar*)matOriginal.data, matOriginal.cols, matOriginal.rows, matOriginal.step, QImage::Format_RGB888);
          QImage qimgProcessed((uchar*)matProcessed.data, matProcessed.cols, matProcessed.rows, matProcessed.step, QImage::Format_RGB888);

          ui->lblOriginal->setPixmap(QPixmap::fromImage(qimgOriginal));
          ui->lblProcessed->setPixmap(QPixmap::fromImage(qimgProcessed));

    }break;

  }

}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////                                   //////////////////////////////////////
////////////////////////////////////////////////////////////////          METHODS                  //////////////////////////////////////
///////////////////////////////////////////////////////////////                                    //////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

cv::Mat Dialog::saltpepper() /////////////////////////  Salt and pepper Noise//////////////////////////
{
  cv::Mat image;
  int n;
  image = matOriginal;
  n = 1000;
   for(int k=0; k<n; k++)
    {
      int i = rand()%image.cols;
      int j = rand()%image.rows;
       if(image.channels() == 1)
        {
          image.at<uchar>(j,i) = 255;
        }
        else if (image.channels() == 3)
        {
          image.at<cv::Vec3b>(j,i)[0] = 255;
          image.at<cv::Vec3b>(j,i)[1] = 255;
          image.at<cv::Vec3b>(j,i)[2] = 255;
        }
    }
    for(int k=0; k<n; k++)
    {
        int i = rand()%image.cols;
        int j = rand()%image.rows;

        if(image.channels() == 1)
        {
            image.at<uchar>(j,i) = 0;
        }
        else if (image.channels() == 3)
        {
            image.at<cv::Vec3b>(j,i)[0] = 0;
            image.at<cv::Vec3b>(j,i)[1] = 0;
            image.at<cv::Vec3b>(j,i)[2] = 0;
        }
    }
  matProcessed = image;
  return matProcessed;
}

cv::Mat Dialog::hist()///////////////////////////////    Histogram and Equalise Histogram    //////////////////////////////////////////////////
{
  cv::Mat src;
  src = matOriginal;
  cv::vector<cv::Mat> bgr_planes;
  cv::split( src, bgr_planes );
  int histSize = 256;
  float range[] = { 0, 256 } ;
  const float* histRange = { range };
  bool uniform = true; bool accumulate = false;
  cv::Mat b_hist, g_hist, r_hist;
  cv::calcHist( &bgr_planes[0], 1, 0, cv::Mat(), b_hist, 1, &histSize, &histRange, uniform, accumulate );
  cv::calcHist( &bgr_planes[1], 1, 0, cv::Mat(), g_hist, 1, &histSize, &histRange, uniform, accumulate );
  cv::calcHist( &bgr_planes[2], 1, 0, cv::Mat(), r_hist, 1, &histSize, &histRange, uniform, accumulate );

  int hist_w = 512; int hist_h = 400;
  int bin_w = cvRound( (double) hist_w/histSize );
  cv::Mat histImage( hist_h, hist_w, CV_8UC3, cv::Scalar( 0,0,0) );
// Normalize the result to [ 0, histImage.rows ]
  cv::normalize(b_hist, b_hist, 0, histImage.rows, cv::NORM_MINMAX, -1, cv::Mat() );
  cv::normalize(g_hist, g_hist, 0, histImage.rows, cv::NORM_MINMAX, -1, cv::Mat() );
  cv::normalize(r_hist, r_hist, 0, histImage.rows, cv::NORM_MINMAX, -1, cv::Mat() );

    for( int i = 1; i < histSize; i++ )
     {
       cv::line( histImage, cv::Point( bin_w*(i-1), hist_h - cvRound(b_hist.at<float>(i-1)) ) ,
                            cv::Point( bin_w*(i), hist_h - cvRound(b_hist.at<float>(i)) ),
                            cv::Scalar( 255, 0, 0), 2, 8, 0 );
       cv::line( histImage, cv::Point( bin_w*(i-1), hist_h - cvRound(g_hist.at<float>(i-1)) ) ,
                            cv::Point( bin_w*(i), hist_h - cvRound(g_hist.at<float>(i)) ),
                            cv::Scalar( 0, 255, 0), 2, 8, 0 );
       cv::line( histImage, cv::Point( bin_w*(i-1), hist_h - cvRound(r_hist.at<float>(i-1)) ) ,
                            cv::Point( bin_w*(i), hist_h - cvRound(r_hist.at<float>(i)) ),
                            cv::Scalar( 0, 0, 255), 2, 8, 0 );
     }
  return histImage;
}

cv::Mat Dialog::eqhist()
{
  cv::cvtColor( matOriginal, matProcessed, CV_RGB2GRAY );
  cv::equalizeHist( matProcessed, matProcessed );
  return matProcessed;
}////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

cv::Mat Dialog::dilate()//////////////////////////////       Morphological Operations      //////////////////////////////////////////////
{
  cv::Mat element = cv::getStructuringElement( cv::MORPH_DILATE, cv::Size( 19, 19),cv::Point( 1, 1) );
  cv::erode ( matOriginal, matProcessed, element);
  return matProcessed;
}

cv::Mat Dialog::erode()
{
  cv::Mat element = cv::getStructuringElement( cv::MORPH_ERODE, cv::Size( 19, 19),cv::Point( 1, 1) );
  cv::erode ( matOriginal, matProcessed, element);
  return matProcessed;
}

cv::Mat Dialog::cross()
{
    cv::Mat element = cv::getStructuringElement( cv::MORPH_CROSS, cv::Size( 19, 19),cv::Point( 1, 1) );
    cv::erode ( matOriginal, matProcessed, element);
    return matProcessed;
}

cv::Mat Dialog::rect()
{
    cv::Mat element = cv::getStructuringElement( cv::MORPH_RECT, cv::Size( 19, 19),cv::Point( 1, 1) );
    cv::erode ( matOriginal, matProcessed, element);
    return matProcessed;
}

cv::Mat Dialog::ellipse()
{
    cv::Mat element = cv::getStructuringElement( cv::MORPH_ELLIPSE, cv::Size( 19, 19),cv::Point( 1, 1) );
    cv::erode ( matOriginal, matProcessed, element);
    return matProcessed;
}

cv::Mat Dialog::ope()
{
  cv::Mat element = cv::getStructuringElement( cv::MORPH_OPEN, cv::Size( 19, 19),cv::Point( 1, 1) );
  cv::erode ( matOriginal, matProcessed, element);
  return matProcessed;
}

cv::Mat Dialog::close()
{
    cv::Mat element = cv::getStructuringElement(cv::MORPH_CLOSE, cv::Size(19,19), cv::Point(1,1) );
    cv::erode(matOriginal, matProcessed, element);
    return matProcessed;

}////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

cv::Mat Dialog::guss_blur()////////////////////////////////////////    Filters or Blurs  ///////////////////////////////////////////////
{
   for ( int i = 1; i < threshh; i = i + 2 )
    {
      cv::GaussianBlur( matOriginal, matProcessed, cv::Size( i, i ), 0, 0 );
    }
  return matProcessed;
}

cv::Mat Dialog::median()
{
    for ( int i = 1; i < threshh; i = i + 2 )
     {
       cv::medianBlur( matOriginal, matProcessed, i);
     }
   return matProcessed;
}

cv::Mat Dialog::bilat_blur()
{
  int MAX_KERNEL_LENGTH = 31;
    for ( int i = 1; i < MAX_KERNEL_LENGTH; i = i + 2 )
      {
        cv::bilateralFilter( matOriginal, matProcessed,i, i*2, i/2 );
      }
   return matProcessed;
}

cv::Mat Dialog::homo_blur()
{
  int MAX_KERNEL_LENGTH = 31;
   for ( int i = 1; i < MAX_KERNEL_LENGTH; i = i + 2 )
    {
      blur( matOriginal, matProcessed, cv::Size( i, i ), cv::Point(-1,-1) );
    }
  return matProcessed;

}////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

cv::Mat Dialog::sobel()//////////////////////////////////////// Shape Descriptors or detectors //////////////////////////////////////////
{
  cv::Mat grad;
  int scale = 1;
  int delta = 0;
  int ddepth = CV_16S;
  cv::GaussianBlur( matOriginal, matOriginal, cv::Size(3,3), 0, 0, cv::BORDER_DEFAULT );
  cv::cvtColor( matOriginal, matProcessed, CV_RGB2GRAY);
  cv::Mat grad_x, grad_y;
  cv::Mat abs_grad_x, abs_grad_y;
  cv::Sobel( matProcessed, grad_x, ddepth, 1, 0, 3, scale, delta, cv::BORDER_DEFAULT );
  cv::convertScaleAbs( grad_x, abs_grad_x);
  cv::Sobel( matProcessed, grad_y, ddepth, 1, 0, 3, scale, delta, cv::BORDER_DEFAULT );
  cv::convertScaleAbs( grad_y, abs_grad_y);
  cv::addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad );
  return grad;
}

cv::Mat Dialog::laplacian()
{

  cv::GaussianBlur( matOriginal, matOriginal, cv::Size(3,3), 0, 0, cv::BORDER_DEFAULT );
  cvtColor( matOriginal, matProcessed, CV_RGB2GRAY );
  cv::Laplacian( matProcessed, matProcessed, CV_16S, 1, 1, 0, cv::BORDER_DEFAULT );
  return matProcessed;

}

cv::Mat Dialog::kernel()
{
  cv::Mat kernel = (cv::Mat_<char>(3,3)<< 0, 1, 0,
                                           -1, 2, -1,
                                            0, 0, 0);
  cv::filter2D(matOriginal, matProcessed, CV_8U, kernel, cv::Point(-1,-1),0,cv::BORDER_DEFAULT);
  return matProcessed;
}

cv::Mat Dialog::canny()
{
  cv::blur(matOriginal, matProcessed, cv::Size(3,3), cv::Point(-1, -1), cv::BORDER_DEFAULT);
  cv::Canny(matOriginal, matProcessed, threshh, 90, 3);
  return matProcessed;
}

cv::Mat Dialog::hou_line()
{
  cv::Canny(matOriginal, matProcessed, threshh, 200, 3);
  cv::vector<cv::Vec2f> lines;
  cv::HoughLines(matProcessed, lines, 1, CV_PI/180, 100, 0, 0 );
  return matProcessed;
}

cv::Mat Dialog::hou_cir()
{
  cv::cvtColor( matOriginal, matProcessed, CV_RGB2GRAY );
  cv::GaussianBlur( matProcessed, matProcessed, cv::Size(9, 9), 2, 2 );
  cv::vector<cv::Vec3f> circles;
  cv::HoughCircles( matProcessed, circles, CV_HOUGH_GRADIENT, 1, matProcessed.rows/8, 200, 100, 0, 0 );
   for( size_t i = 0; i < circles.size(); i++ )
    {
      cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
      int radius = cvRound(circles[i][2]);
      cv::circle( matOriginal, center, 3, cv::Scalar(0,255,0), -1, 8, 0 );
      cv::circle( matOriginal, center, radius, cv::Scalar(0,0,255), 3, 8, 0 );
    }
  return matProcessed;
}

cv::Mat Dialog::contours()
{
//  int thresh = 100;
  cv::RNG rng(12345);
  cv::cvtColor( matOriginal, matProcessed, CV_BGR2GRAY );
  cv::blur( matProcessed, matProcessed, cv::Size(3,3) );
  cv::Mat canny_output;
  cv::vector<cv::vector<cv::Point> > contours;
  cv::vector<cv::Vec4i> hierarchy;
  cv::Canny( matProcessed, canny_output, threshh, threshh*2, 3 );
  cv::findContours( canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
  cv::Mat drawing = cv::Mat::zeros( canny_output.size(), CV_8UC3 );
   for( int i = 0; i< contours.size(); i++ )
    {
      cv::Scalar color = cv::Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
      cv::drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, cv::Point() );
    }
  return drawing;
}

cv::Mat Dialog::boun_box()
{
  cv::Mat src , src_gray;
  cv::RNG rng(12345);
  cv::Mat threshold_output;
  cv::vector<cv::vector<cv::Point> > contours;
  cv::vector<cv::Vec4i> hierarchy;
  src = matOriginal;
  cv::cvtColor( src, src_gray, CV_BGR2GRAY );
  cv::blur( src_gray, src_gray, cv::Size(3,3) );
  cv::threshold( src_gray, threshold_output, threshh, 255, cv::THRESH_BINARY );
  cv::findContours( threshold_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
  cv::vector<cv::vector<cv::Point> > contours_poly( contours.size() );
  cv::vector<cv::Rect> boundRect( contours.size() );

    for( int i = 0; i < contours.size(); i++ )
     {
       cv::approxPolyDP( cv::Mat(contours[i]), contours_poly[i], 3, true );
       boundRect[i] = cv::boundingRect( cv::Mat(contours_poly[i]) );
     }

  cv::Mat drawing = cv::Mat::zeros( threshold_output.size(), CV_8UC3 );
    for( int i = 0; i< contours.size(); i++ )
     {
       cv::Scalar color = cv::Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
       cv::rectangle( drawing, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0 );
     }
  return drawing;
}

cv::Mat Dialog::min_encir()
{
  cv::Mat src , src_gray;
  cv::RNG rng(12345);
  cv::Mat threshold_output;
  cv::vector<cv::vector<cv::Point> > contours;
  cv::vector<cv::Vec4i> hierarchy;
  src = matOriginal;
  cv::cvtColor( src, src_gray, CV_BGR2GRAY );
  cv::blur( src_gray, src_gray, cv::Size(3,3) );
  cv::threshold( src_gray, threshold_output, threshh, 255, cv::THRESH_BINARY );
  cv::findContours( threshold_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
  cv::vector<cv::vector<cv::Point> > contours_poly( contours.size() );
  cv::vector<cv::Point2f>center( contours.size() );
  cv::vector<float>radius( contours.size() );

    for( int i = 0; i < contours.size(); i++ )
     {
       cv::approxPolyDP( cv::Mat(contours[i]), contours_poly[i], 3, true );
       cv::minEnclosingCircle( (cv::Mat)contours_poly[i], center[i], radius[i] );
     }

  cv::Mat drawing = cv::Mat::zeros( threshold_output.size(), CV_8UC3 );
    for( int i = 0; i< contours.size(); i++ )
     {
       cv::Scalar color = cv::Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
       cv::drawContours( drawing, contours_poly, i, color, 1, 8, cv::vector<cv::Vec4i>(), 0, cv::Point() );
       cv::circle( drawing, center[i], (int)radius[i], color, 2, 8, 0 );
     }
 return drawing;
}////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

cv::Mat Dialog::harris() //////////////////////////////////     Feature Detectors      ////////////////////////////////////////////////////
{
  cv::Mat src, src_gray;
  src = matOriginal;
  cv::cvtColor( src, src_gray, CV_BGR2GRAY );
  cv::Mat dst, dst_norm, dst_norm_scaled;
  dst = cv::Mat::zeros( src.size(), CV_32FC1 );
  int blockSize = 5;
  int apertureSize = 3;
  double k = 0.04;
  cv::cornerHarris( src_gray, dst, blockSize, apertureSize, k, cv::BORDER_DEFAULT );
  cv::normalize( dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat() );
  cv::convertScaleAbs( dst_norm, dst_norm_scaled );
    for( int j = 0; j < dst_norm.rows ; j++ )
     {
      for( int i = 0; i < dst_norm.cols; i++ )
       {
         if( (int) dst_norm.at<float>(j,i) > 100 )
          {
            circle( dst_norm_scaled, cv::Point( i, j ), 5, cv::Scalar(0), 2, 8, 0 );
          }
       }
     }
  return dst_norm_scaled;
}

cv::Mat Dialog::shi()
{
  cv::Mat src, src_gray;
  src = matOriginal;
  int maxCorners = 23;
  cv::RNG rng(12345);
  cv::cvtColor( src, src_gray, CV_BGR2GRAY );
  if( maxCorners < 1 ) { maxCorners = 1; }
  cv::vector<cv::Point2f> corners;
  double qualityLevel = 0.01;
  double minDistance = 10;
  int blockSize = 3;
  bool useHarrisDetector = false;
  double k = 0.04;

  cv::Mat copy;
  copy = src.clone();
  cv::goodFeaturesToTrack( src_gray, corners, maxCorners, qualityLevel, minDistance, cv::Mat(), blockSize, useHarrisDetector, k );
  int r = 4;
   for( int i = 0; i < corners.size(); i++ )
    {
      circle( copy, corners[i], r, cv::Scalar(rng.uniform(0,255), rng.uniform(0,255), rng.uniform(0,255)), -1, 8, 0 );
    }
  return copy;
}

cv::Mat Dialog::fast()
{
  cv::Mat image = matOriginal;
  cv::Mat gray, outputImage;
  cv::vector<cv::KeyPoint> keypoints;
  cv::cvtColor(image, gray, CV_BGR2GRAY);
  cv::FAST(gray,keypoints, threshh, true);
  cv::Scalar keypointColor = cv::Scalar(255, 0, 0);
  drawKeypoints(gray, keypoints, outputImage, keypointColor, cv::DrawMatchesFlags::DEFAULT);
  return outputImage;
}

cv::Mat Dialog::sift()
{
  cv::Mat image = matOriginal;
  cv::Ptr<cv::FeatureDetector> FeatureDetector = cv::FeatureDetector::create("SIFT");
  cv::vector<cv::KeyPoint> keypoints;

  FeatureDetector->detect(image, keypoints);
  cv::Ptr<cv::DescriptorExtractor> featureExtractor = cv::DescriptorExtractor::create("SIFT");
  cv::Mat descriptors;
  featureExtractor->compute(image, keypoints, descriptors);
  cv::Mat outputImage;
  cv::Scalar keypointColor = cv::Scalar(255, 0, 0);
  drawKeypoints(image, keypoints, outputImage, keypointColor, cv::DrawMatchesFlags::DEFAULT);
  return outputImage;
}

cv::Mat Dialog::surf()
{
  int minHessian =400;
  cv::SurfFeatureDetector detector( minHessian );
  std::vector<cv::KeyPoint> KeyPoints_1;
  detector.detect( matOriginal, KeyPoints_1 );
  cv::SurfDescriptorExtractor extractor;
  cv::Mat descriptors_1;
  extractor.compute( matOriginal, KeyPoints_1, descriptors_1);
  cv::drawKeypoints(matOriginal, KeyPoints_1, matProcessed, cv::Scalar(255,0,0));
  return matProcessed;
}

cv::Mat Dialog::convexhull()
{
  cv::Mat src, src_gray;
  cv::RNG rng(12345);
  src = matOriginal;
  cv::cvtColor( src, src_gray, CV_BGR2GRAY );
  cv::blur( src_gray,src_gray, cv::Size(3,3) );
  cv::Mat threshold_output;
  cv::vector<cv::vector<cv::Point> > contours;
  cv::vector<cv::Vec4i> hierarchy;
  cv::threshold( src_gray, threshold_output, threshh, 255, cv::THRESH_BINARY );
  cv::findContours( threshold_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
  cv::vector<cv::vector<cv::Point> >hull( contours.size() );

   for( int i = 0; i < contours.size(); i++ )
    {
      cv::convexHull( cv::Mat(contours[i]), hull[i], false );
    }
  cv::Mat drawing = cv::Mat::zeros( threshold_output.size(), CV_8UC3 );
   for( int i = 0; i< contours.size(); i++ )
    {
      cv::Scalar color = cv::Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
      cv::drawContours( drawing, contours, i, color, 1, 8, cv::vector<cv::Vec4i>(), 0, cv::Point() );
      cv::drawContours( drawing, hull, i, color, 1, 8, cv::vector<cv::Vec4i>(), 0, cv::Point() );
    }
  return drawing;
}//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Dialog::on_pushButton_clicked() // Homography
{
    cv::Mat img1, img2;
    QString filename1 = QFileDialog::getOpenFileName(
                this,
                tr("Select Image(s) to Open"),
                QDir::toNativeSeparators(QDir::homePath()),
                tr("Images (*.png *.bmp *.jpg *.jpeg *.tiff)"));

     if (!filename1.isEmpty())
                {
                img1 = cv::imread(filename1.toStdString());
                }
     QString filename2 = QFileDialog::getOpenFileName(
                 this,
                 tr("Select Image(s) to Open"),
                 QDir::toNativeSeparators(QDir::homePath()),
                 tr("Images (*.png *.bmp *.jpg *.jpeg *.tiff)"));

      if (!filename2.isEmpty())
                 {
                 img2 = cv::imread(filename2.toStdString());
                 }

      cv::Mat img_object;
      cv::Mat img_scene;
      cv::cvtColor(img1, img_object, CV_BGR2RGB);
      cv::cvtColor(img2, img_scene, CV_BGR2RGB);
      int minHessian = 400;
      cv::SurfFeatureDetector detector( minHessian );
      std::vector<cv::KeyPoint> keypoints_object, keypoints_scene;

      detector.detect( img_object, keypoints_object );
      detector.detect( img_scene, keypoints_scene );

    //-- Step 2: Calculate descriptors (feature vectors)
      cv::SurfDescriptorExtractor extractor;
      cv::Mat descriptors_object, descriptors_scene;

      extractor.compute( img_object, keypoints_object, descriptors_object );
      extractor.compute( img_scene, keypoints_scene, descriptors_scene );

    //-- Step 3: Matching descriptor vectors using FLANN matcher
      cv::FlannBasedMatcher matcher;
      std::vector< cv::DMatch > matches;
      matcher.match( descriptors_object, descriptors_scene, matches );
      double max_dist = 0; double min_dist = 100;

    //-- Quick calculation of max and min distances between keypoints
       for( int i = 0; i < descriptors_object.rows; i++ )
         {
           double dist = matches[i].distance;
            if( dist < min_dist ) min_dist = dist;
              if( dist > max_dist ) max_dist = dist;
         }

    //-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
        std::vector< cv::DMatch > good_matches;

         for( int i = 0; i < descriptors_object.rows; i++ )
          {
            if( matches[i].distance < 3*min_dist )
           { good_matches.push_back( matches[i]); }
          }

      cv::Mat img_matches;
      drawMatches( img_object, keypoints_object, img_scene, keypoints_scene,
                     good_matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
                     cv::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

    //-- Localize the object from img_1 in img_2
      std::vector<cv::Point2f> obj;
      std::vector<cv::Point2f> scene;

        for( size_t i = 0; i < good_matches.size(); i++ )
         {
    //-- Get the keypoints from the good matches
           obj.push_back( keypoints_object[ good_matches[i].queryIdx ].pt );
           scene.push_back( keypoints_scene[ good_matches[i].trainIdx ].pt );
         }

      cv::Mat H = findHomography( obj, scene, CV_RANSAC );
      ui->txtXYRadius->appendPlainText(QString("The Homography Matrix is "));
      std::cout << H << std::endl;

      matOriginal = img_object;
      matProcessed = img_scene;

      QImage qimgOriginal((uchar*)matOriginal.data, matOriginal.cols, matOriginal.rows, matOriginal.step, QImage::Format_RGB888);
      QImage qimgProcessed((uchar*)matProcessed.data, matProcessed.cols, matProcessed.rows, matProcessed.step, QImage::Format_RGB888);

      ui->lblOriginal->setPixmap(QPixmap::fromImage(qimgOriginal));
      ui->lblProcessed->setPixmap(QPixmap::fromImage(qimgProcessed));


}

void Dialog::on_pushButton_2_clicked() //  Mosaic
{
    cv::Mat img1, img2;
    QString filename1 = QFileDialog::getOpenFileName(
                this,
                tr("Select Image(s) to Open"),
                QDir::toNativeSeparators(QDir::homePath()),
                tr("Images (*.png *.bmp *.jpg *.jpeg *.tiff)"));

     if (!filename1.isEmpty())
                {
                img1 = cv::imread(filename1.toStdString());
                }
     QString filename2 = QFileDialog::getOpenFileName(
                 this,
                 tr("Select Image(s) to Open"),
                 QDir::toNativeSeparators(QDir::homePath()),
                 tr("Images (*.png *.bmp *.jpg *.jpeg *.tiff)"));

      if (!filename2.isEmpty())
                 {
                 img2 = cv::imread(filename2.toStdString());
                 }

    cv::Mat image1= img1;
    cv::Mat image2= img2;
    cv::Mat gray_image1;
    cv::Mat gray_image2;
  // Convert to Grayscale
    cv::cvtColor( image1, gray_image1, CV_RGB2GRAY );
    cv::cvtColor( image2, gray_image2, CV_RGB2GRAY );

  //-- Step 1: Detect the keypoints using SURF Detector
    int minHessian = 400;
    cv::SurfFeatureDetector detector( minHessian );
    std::vector< cv::KeyPoint > keypoints_object, keypoints_scene;

    detector.detect( gray_image1, keypoints_object );
    detector.detect( gray_image2, keypoints_scene );

  //-- Step 2: Calculate descriptors (feature vectors)
    cv::SurfDescriptorExtractor extractor;
    cv::Mat descriptors_object, descriptors_scene;

    extractor.compute( gray_image1, keypoints_object, descriptors_object );
    extractor.compute( gray_image2, keypoints_scene, descriptors_scene );

  //-- Step 3: Matching descriptor vectors using FLANN matcher
    cv::FlannBasedMatcher matcher;
    std::vector< cv::DMatch > matches;
    matcher.match( descriptors_object, descriptors_scene, matches );
    double max_dist = 0; double min_dist = 100;

  //-- Quick calculation of max and min distances between keypoints
      for( int i = 0; i < descriptors_object.rows; i++ )
       {
         double dist = matches[i].distance;
          if( dist < min_dist ) min_dist = dist;
           if( dist > max_dist ) max_dist = dist;
       }

  //-- Use only "good" matches (i.e. whose distance is less than 3*min_dist )
    std::vector< cv::DMatch > good_matches;
       for( int i = 0; i < descriptors_object.rows; i++ )
        {
          if( matches[i].distance < 3*min_dist )
            { good_matches.push_back( matches[i]); }
        }
    std::vector< cv::Point2f > obj;
    std::vector< cv::Point2f > scene;
       for( int i = 0; i < good_matches.size(); i++ )
        {
  //-- Get the keypoints from the good matches
          obj.push_back( keypoints_object[ good_matches[i].queryIdx ].pt );
          scene.push_back( keypoints_scene[ good_matches[i].trainIdx ].pt );
        }
  // Find the Homography Matrix
    cv::Mat H = findHomography( obj, scene, CV_RANSAC );
  // Use the Homography Matrix to warp the images
    cv::Mat result;
    cv::warpPerspective(image2,result,H,cv::Size(2*image2.cols,image1.rows));
    cv::Mat half(result,cv::Rect(0,0,image1.cols,image1.rows));
    image1.copyTo(half);
    imshow( "Result", result );
}


void Dialog::on_pushButton_3_clicked()  //Matches between two images
{
    cv::Mat img1, img2;

    QString filename1 = QFileDialog::getOpenFileName(
                this,
                tr("Select Image(s) to Open"),
                QDir::toNativeSeparators(QDir::homePath()),
                tr("Images (*.png *.bmp *.jpg *.jpeg *.tiff)"));

     if (!filename1.isEmpty())
                {
                img1 = cv::imread(filename1.toStdString());
                }
     QString filename2 = QFileDialog::getOpenFileName(
                 this,
                 tr("Select Image(s) to Open"),
                 QDir::toNativeSeparators(QDir::homePath()),
                 tr("Images (*.png *.bmp *.jpg *.jpeg *.tiff)"));

      if (!filename2.isEmpty())
                 {
                 img2 = cv::imread(filename2.toStdString());
                 }

    cv::Mat img_object = img1;
    cv::Mat img_scene = img2;
    int minHessian = 400;
    cv::SurfFeatureDetector detector( minHessian );
    std::vector<cv::KeyPoint> keypoints_object, keypoints_scene;

    detector.detect( img_object, keypoints_object );
    detector.detect( img_scene, keypoints_scene );

  //-- Step 2: Calculate descriptors (feature vectors)
    cv::SurfDescriptorExtractor extractor;
    cv::Mat descriptors_object, descriptors_scene;

    extractor.compute( img_object, keypoints_object, descriptors_object );
    extractor.compute( img_scene, keypoints_scene, descriptors_scene );

  //-- Step 3: Matching descriptor vectors using FLANN matcher
    cv::FlannBasedMatcher matcher;
    std::vector< cv::DMatch > matches;
    matcher.match( descriptors_object, descriptors_scene, matches );
    double max_dist = 0; double min_dist = 100;

  //-- Quick calculation of max and min distances between keypoints
     for( int i = 0; i < descriptors_object.rows; i++ )
       {
         double dist = matches[i].distance;
          if( dist < min_dist ) min_dist = dist;
            if( dist > max_dist ) max_dist = dist;
       }

  //-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
      std::vector< cv::DMatch > good_matches;

       for( int i = 0; i < descriptors_object.rows; i++ )
        {
          if( matches[i].distance < 3*min_dist )
         { good_matches.push_back( matches[i]); }
        }

    cv::Mat img_matches;
    drawMatches( img_object, keypoints_object, img_scene, keypoints_scene,
                   good_matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
                   cv::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

  //-- Localize the object from img_1 in img_2
    std::vector<cv::Point2f> obj;
    std::vector<cv::Point2f> scene;

      for( size_t i = 0; i < good_matches.size(); i++ )
       {
  //-- Get the keypoints from the good matches
         obj.push_back( keypoints_object[ good_matches[i].queryIdx ].pt );
         scene.push_back( keypoints_scene[ good_matches[i].trainIdx ].pt );
       }

    cv::Mat H = findHomography( obj, scene, CV_RANSAC );

  //  -- Get the corners from the image_1 ( the object to be "detected" )
             std::vector<cv::Point2f> obj_corners(4);
             obj_corners[0] = cv::Point(0,0); obj_corners[1] = cv::Point( img_object.cols, 0 );
             obj_corners[2] = cv::Point( img_object.cols, img_object.rows ); obj_corners[3] = cv::Point( 0, img_object.rows );
             std::vector<cv::Point2f> scene_corners(4);

             cv::perspectiveTransform( obj_corners, scene_corners, H);

             //-- Draw lines between the corners (the mapped object in the scene - image_2 )
             cv::Point2f offset( (float)img_object.cols, 0);
             line( img_matches, scene_corners[0] + offset, scene_corners[1] + offset, cv::Scalar(0, 255, 0), 4 );
             line( img_matches, scene_corners[1] + offset, scene_corners[2] + offset, cv::Scalar( 0, 255, 0), 4 );
             line( img_matches, scene_corners[2] + offset, scene_corners[3] + offset, cv::Scalar( 0, 255, 0), 4 );
             line( img_matches, scene_corners[3] + offset, scene_corners[0] + offset, cv::Scalar( 0, 255, 0), 4 );

           cv::namedWindow("Result", cv::WINDOW_AUTOSIZE);
               imshow( "Result", img_matches );
}

void Dialog::on_horizontalSlider_valueChanged(int value)
{
    threshh = value;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////                                                /////////////////////////
////////////////////////////////////////////////////////////////                     END                        /////////////////////////
///////////////////////////////////////////////////////////////                                                 /////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
