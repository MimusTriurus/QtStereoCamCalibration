// based on https://github.com/sourishg/stereo-calibration

#include <QCoreApplication>
#include "opencv2/opencv.hpp"
#include <iostream>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

std::vector< std::vector< cv::Point3f > > objectPoints;
std::vector< std::vector< cv::Point2f > > imagePoints1, imagePoints2;
std::vector< cv::Point2f > corners1, corners2;
std::vector< std::vector< cv::Point2f > > leftImgPoints, rightImgPoints;

cv::Mat img1, img2, gray1, gray2;

void loadImagePoints( int boardWidth, int boardHeight, int imgsCount, float squareSize,
                      const char* leftimgDir, const char* rightimgDir,
                      const char* leftImgFilename, const char* rightImgFilename, const char* ext )
{
    cv::Size boardSize = cv::Size( boardWidth, boardHeight );
    for ( int i = 1; i <= imgsCount; i++ ) {
        QString leftImg{ leftimgDir };
        leftImg.append( leftImgFilename );
        leftImg.append( QString::number( i ) );
        leftImg.append( "." );
        leftImg.append( ext );
        QString rightImg{ rightimgDir };
        rightImg.append( rightImgFilename );
        rightImg.append( QString::number( i ) );
        rightImg.append( "." );
        rightImg.append( ext );
        std::cout << "imread:" << leftImg.toStdString( ) << std::endl;
        img1 = cv::imread( leftImg.toStdString( ), CV_LOAD_IMAGE_COLOR );
        img2 = cv::imread( rightImg.toStdString( ), CV_LOAD_IMAGE_COLOR );
        cv::cvtColor( img1, gray1, CV_BGR2GRAY );
        cv::cvtColor( img2, gray2, CV_BGR2GRAY );
        bool found1 = false, found2 = false;
        found1 = cv::findChessboardCorners( img1, boardSize, corners1,
            CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
        found2 = cv::findChessboardCorners( img2, boardSize, corners2,
            CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);

        if ( found1 ) {
            cv::cornerSubPix( gray1, corners1, cv::Size( 5, 5 ), cv::Size( -1, -1 ),
            cv::TermCriteria( CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1 ) );
            cv::drawChessboardCorners( gray1, boardSize, corners1, found1 );
        }
        if ( found2 ) {
            cv::cornerSubPix( gray2, corners2, cv::Size( 5, 5 ), cv::Size( -1, -1 ),
            cv::TermCriteria( CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1 ) );
            cv::drawChessboardCorners( gray2, boardSize, corners2, found2 );
        }
        std::vector< cv::Point3f > obj;
        for ( int i = 0; i < boardHeight; i++ )
            for ( int j = 0; j < boardWidth; j++ )
                obj.push_back( cv::Point3f( ( float )j * squareSize, ( float )i * squareSize, 0 ) );

        if ( found1 && found2 ) {
            std::cout << i << ". Found corners!" << std::endl;
            imagePoints1.push_back( corners1 );
            imagePoints2.push_back( corners2 );
            objectPoints.push_back( obj );
        }
    }
    for ( int i = 0; i < imagePoints1.size( ); i++ ) {
        std::vector< cv::Point2f > v1, v2;
        for ( int j = 0; j < imagePoints1[ i ].size( ); j++ ) {
            v1.push_back( cv::Point2f( ( double )imagePoints1[ i ][ j ].x, ( double )imagePoints1[ i ][ j ].y ) );
            v2.push_back( cv::Point2f( ( double )imagePoints2[ i ][ j ].x, ( double )imagePoints2[ i ][ j ].y ) );
        }
        leftImgPoints.push_back( v1 );
        rightImgPoints.push_back( v2 );
    }
}

int main( int argc, char *argv[ ] ) {
    QCoreApplication a( argc, argv );

    const cv::String keys =
    "{help h usage ? |                  |print help message}"
    "{imgLFolder     |./calibrationImgs/|Directory of left cam images}"
    "{imgRFolder     |./calibrationImgs/|Directory of right cam images}"
    "{fLCalib        |./lIntrinsic.yaml |file left cam calib}"
    "{fRCalib        |./rIntrinsic.yaml |file right cam calib}"
    "{preL           |image_left_       |Left image name prefix}"
    "{preR           |image_right_      |Right image name prefix}"
    "{imgsCount      |10                |count of img}"
    "{outputFile     |extrinsics.yaml   |extrinsics stereo camera calibration params}"
    "{imgExt         |jpg               |Image extension. Ext: jpg,png etc}";

    cv::CommandLineParser parser( argc, argv, keys );
    parser.about("StereoCamCalibration v1.0.0");
    if ( parser.has( "help" ) ) {
        parser.printMessage( );
        return 0;
    }

    cv::String imgLFolder    = parser.get<cv::String>( "imgLFolder" );
    cv::String imgRFolder    = parser.get<cv::String>( "imgRFolder" );
    cv::String imgExt        = parser.get<cv::String>( "imgExt" );
    cv::String preL          = parser.get<cv::String>( "preL" );
    cv::String preR          = parser.get<cv::String>( "preR" );
    int imgsCount        = parser.get<int>   ( "imgsCount" );
    cv::String outputFile    = parser.get<cv::String>( "outputFile" );
    cv::String fLCalib       = parser.get<cv::String>( "fLCalib" );
    cv::String fRCalib       = parser.get<cv::String>( "fRCalib" );

    cv::FileStorage fsl( fLCalib, cv::FileStorage::READ );
    cv::FileStorage fsr( fRCalib, cv::FileStorage::READ );
    std::cout << "start laod image points" << std::endl;
    loadImagePoints( fsl[ "board_width" ], fsl[ "board_height" ], imgsCount, fsl[ "square_size" ],
                   imgLFolder.c_str( ), imgRFolder.c_str( ), preL.c_str( ), preR.c_str( ), imgExt.c_str( ) );

    std::cout << "Starting Calibration" << std::endl;
    cv::Mat K1, K2, R, F, E;
    cv::Vec3d T;
    cv::Mat D1, D2;
    fsl[ "K" ] >> K1;
    fsr[ "K" ] >> K2;
    fsl[ "D" ] >> D1;
    fsr[ "D" ] >> D2;
    int flag = 0;
    flag |= CV_CALIB_FIX_INTRINSIC;

    std::cout << "Read intrinsics" << std::endl;

    double rms = stereoCalibrate( objectPoints, leftImgPoints, rightImgPoints, K1, D1, K2, D2, img1.size( ), R, T, E, F );
    std::cout << "rms: " << rms << std::endl;

    cv::FileStorage fs1( outputFile, cv::FileStorage::WRITE );
    fs1 << "K1" << K1;
    fs1 << "K2" << K2;
    fs1 << "D1" << D1;
    fs1 << "D2" << D2;
    fs1 << "R" << R;
    fs1 << "T" << T;
    fs1 << "E" << E;
    fs1 << "F" << F;

    std::cout << "Done Calibration" << std::endl;

    std::cout << "Starting Rectification" << std::endl;

    cv::Mat R1, R2, P1, P2, Q;
    //flag = CV_CALIB_ZERO_DISPARITY;
    flag = 0; double alpha = -1;
    cv::Rect validROI[ 2 ];
    cv::stereoRectify( K1, D1, K2, D2, img1.size( ), R, T, R1, R2, P1, P2, Q, flag, alpha, img1.size( ), &validROI[ 0 ], &validROI[ 1 ] );

    std::cout << "roi1: " << validROI[ 0 ] << " roi2: " << validROI[ 1 ] << std::endl;

    fs1 << "R1" << R1;
    fs1 << "R2" << R2;
    fs1 << "P1" << P1;
    fs1 << "P2" << P2;
    fs1 << "Q" << Q;
    fs1 << "vRoi0" << validROI[ 0 ];
    fs1 << "vRoi1" << validROI[ 1 ];
    fs1.release( );
    std::cout << "Done Rectification" << std::endl;

    return a.exec( );
}
