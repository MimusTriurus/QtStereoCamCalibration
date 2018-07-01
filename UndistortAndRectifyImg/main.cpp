// based on https://github.com/sourishg/stereo-calibration

#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <iostream>

#include <QtCore>

using namespace std;

// params examples
// --lImgFilename=./calibrationImgs/image_left_1.jpg
// --rImgFilename=./calibrationImgs/image_right_1.jpg
// --calibFile=extrinsics.yaml
// --lImgOutFName=./rectL.jpg
// --rImgOutFName=./rectR.jpg

int main( int argc, char *argv[ ] ) {
    QCoreApplication a( argc, argv );

    const cv::String keys =
    "{help h usage ? |                                   |print help message}"
    "{lImgFilename   |./calibrationImgs/image_left_1.jpg |Left image path}"
    "{rImgFilename   |./calibrationImgs/image_right_1.jpg|Right image path}"
    "{calibFile      |extrinsics.yaml                    |Stereo calibration file name}"
    "{calibFileDir   |./                                 |Stereo calibration file dir}"
    "{lImgOutFName   |./rectL.jpg                        |Left undistorted imgage path}"
    "{rImgOutFName   |./rectR.jpg                        |Right undistorted image path}";

    cv::CommandLineParser parser( argc, argv, keys );
    parser.about("UndistortAndRectify v1.0.0");
    if ( parser.has( "help" ) ) {
        parser.printMessage( );
        return 0;
    }

    cv::String lImgFilename = parser.get<cv::String>( "lImgFilename" );
    cv::String rImgFilename = parser.get<cv::String>( "rImgFilename" );
    cv::String calibFile    = parser.get<cv::String>( "calibFile" );
    cv::String calibFileDir = parser.get<cv::String>( "calibFileDir" );
    cv::String lImgOutFName = parser.get<cv::String>( "lImgOutFName" );
    cv::String rImgOutFName = parser.get<cv::String>( "rImgOutFName" );

    cv::Mat img1 = imread( lImgFilename, 1 );
    cv::Mat img2 = imread( rImgFilename, 1 );
///////////////////////////////////////////////////////////////////////////
    cv::Mat R1, R2, P1, P2, Q;
    cv::Mat K1, K2, R;
    cv::Vec3d T;
    cv::Mat D1, D2;
    cv::Mat lmapx, lmapy;
    cv::Mat rmapx, rmapy;
    cv::Rect validROI[ 2 ];

    QString path{ calibFileDir.c_str( ) };
    path.append( calibFile.c_str( ) );
    QFile f( path );
    if ( !f.exists( ) )
        cout << "file:" << path.toStdString( ) << " not found" << endl;
    cv::FileStorage fs1( path.toStdString( ), cv::FileStorage::READ );

    fs1[ "K1" ] >> K1;
    fs1[ "K2" ] >> K2;
    fs1[ "D1" ] >> D1;
    fs1[ "D2" ] >> D2;
    fs1[ "R" ]  >> R;
    fs1[ "T" ]  >> T;

    fs1[ "R1" ] >> R1;
    fs1[ "R2" ] >> R2;
    fs1[ "P1" ] >> P1;
    fs1[ "P2" ] >> P2;
    fs1[ "Q" ]  >> Q;
    fs1[ "vRoi0" ] >> validROI[ 0 ];
    fs1[ "vRoi1" ] >> validROI[ 1 ];

    cv::Rect targetRect = validROI[ 0 ] & validROI[ 1 ];

    cv::Size frameSize = img1.size( );

    cv::initUndistortRectifyMap( K1, D1, R1, P1, frameSize, CV_32F, lmapx, lmapy );
    cv::initUndistortRectifyMap( K2, D2, R2, P2, frameSize, CV_32F, rmapx, rmapy );

    cv::remap( img1, img1 , lmapx, lmapy, cv::INTER_LINEAR );
    cv::remap( img2, img2, rmapx, rmapy, cv::INTER_LINEAR );

    img1 = img1( targetRect );
    img2 = img2( targetRect );
////////////////////////////////////////////////////////////////////////////////////

    cv::Mat stereoMatEdited = cv::Mat( img1.size( ).height, img1.size( ).width + img2.size( ).width, img1.type( ) );

    cv::Rect lRect = cv::Rect( 0, 0, img1.size( ).width, img1.size( ).height );
    cv::Rect rRect = cv::Rect( img1.size( ).width, 0, img2.size( ).width, img2.size( ).height );

    cv::Mat lF = stereoMatEdited( lRect );
    cv::Mat rF = stereoMatEdited( rRect );

    img1.copyTo( lF );
    img2.copyTo( rF );

    for( int j = 0; j < stereoMatEdited.rows; j += 16 )
        line( stereoMatEdited, cv::Point( 0, j ), cv::Point( stereoMatEdited.cols, j ), cv::Scalar( 255, 0, 0 ), 1, 8 );

    cv::imshow( "stereoEdited", stereoMatEdited );

    // save rectifired images
    imwrite( lImgOutFName, img1 );
    imwrite( rImgOutFName, img2 );

    return a.exec( );
}
