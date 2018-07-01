// based on https://github.com/sourishg/stereo-calibration

#include "opencv2/opencv.hpp"
#include <stdio.h>
#include <iostream>

#include <QtCore>

int x = 0;

int main( int argc, char *argv[ ] ) {
    const cv::String keys =
    "{help h usage ? |                  |print help message}"
    "{hBoard         |6                 |Height of the board}"
    "{wBoard         |9                 |Width of the board}"
    "{hImg           |480               |Height of the board}"
    "{wImg           |640               |Width of the board}"
    "{imgFolder      |./calibrationImgs |Directory of images}"
    "{preL           |image_left_       |Left image name prefix. Ex: image_left_}"
    "{preR           |image_right_      |Right image name postfix. Ex: image_right_}"
    "{imgExt         |jpg               |Image extension. Ext: jpg,png etc}"
    "{cam1Ind        |0                 |Camera 1 Index}"
    "{cam2Ind        |1                 |Camera 2 Index}";

    cv::CommandLineParser parser( argc, argv, keys );
    parser.about("ImgGrabber v1.0.0");
    if ( parser.has( "help" ) ) {
        parser.printMessage( );
        return 0;
    }

    cv::String imgFolder    = parser.get<cv::String>( "imgFolder" );
    cv::String imgExt       = parser.get<cv::String>( "imgExt" );
    cv::String preL         = parser.get<cv::String>( "preL" );
    cv::String preR         = parser.get<cv::String>( "preR" );
    int imgWidth            = parser.get<int>       ( "wImg" );
    int imgHeight           = parser.get<int>       ( "hImg" );
    int cam1Ind             = parser.get<int>       ( "cam1Ind" );
    int cam2Ind             = parser.get<int>       ( "cam2Ind" );

    cv::Size boardSize( parser.get<int>( "wBoard" ), parser.get<int>( "hBoard" ) );

    std::cout << "output imgs dir: " << imgFolder << std::endl;

    cv::VideoCapture camera1( cam1Ind );
    cv::VideoCapture camera2( cam2Ind );

    if ( !camera1.isOpened( ) || !camera2.isOpened( ) ) {
      std::cout << "error opening cameras: " << cam1Ind << " " << cam2Ind << std::endl;
      return -1;
    }

    std::cout << "press any key for save pair img" << std::endl;

    std::vector<cv::Point2f> cornersLeft, cornersRight;
    cv::Mat img1, img2, lImg4Save, rImg4Save;
    while ( true ) {
        camera1 >> img1;
        camera2 >> img2;
        if ( img1.empty( ) || img2.empty( ) ) {
            cv::waitKey( 30 );
            continue;
        }
        cv::Size imgSize( imgWidth, imgHeight );
        cv::resize( img1, img1, imgSize );
        cv::resize( img2, img2, imgSize );

        bool foundLeft = cv::findChessboardCorners( img1, boardSize , cornersLeft, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE );
        bool foundRight = cv::findChessboardCorners( img2, boardSize, cornersRight, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE );

        img1.copyTo( lImg4Save );
        img2.copyTo( rImg4Save );

        if ( foundLeft ) {
            cv::drawChessboardCorners( img1, boardSize, cornersLeft, foundLeft );
        }

        if ( foundRight ) {
            cv::drawChessboardCorners( img2, boardSize, cornersRight, foundRight );
        }

        imshow( "left" , img1 );
        imshow( "right", img2 );
        foundLeft = foundRight = true;
        if ( cv::waitKey( 30 ) > 0 ) {
            if ( foundLeft && foundRight ) {
                x++;
                std::ostringstream leftString, rightString;
                leftString  << imgFolder << "/" << preL << x << "." << imgExt;
                rightString << imgFolder << "/" << preR << x << "." << imgExt;
                imwrite( leftString.str( ).c_str( ) , lImg4Save );
                imwrite( rightString.str( ).c_str( ), rImg4Save );
                std::cout << "save " << x << " imgs pair" << std::endl;
            }
        }
    }
    return 0;
}
