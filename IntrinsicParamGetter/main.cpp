// based on https://github.com/sourishg/stereo-calibration

#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <iostream>

#include <QtCore>

std::vector< std::vector< cv::Point3f > > objectPoints;
std::vector< std::vector< cv::Point2f > > imagePoints;
std::vector< cv::Point2f > corners;

cv::Mat img, gray;

void setupCalibration( int boardWidth, int boardHeight, int imgsCount,
                       float squareSize, const char* imgsDirectory, const char* imgsFilename,
                       const char* extension )
{
    cv::Size boardSize = cv::Size( boardWidth, boardHeight );
    for ( int k = 1; k <= imgsCount; k++ ) {
        char img_file[ 100 ];
        sprintf( img_file, "%s%s%d.%s", imgsDirectory, imgsFilename, k, extension );
        std::cout << img_file << std::endl;
        img = cv::imread( img_file, CV_LOAD_IMAGE_COLOR );
        if ( img.empty( ) ) {
            std::cout << "setupCalibration: " << img_file << " not found!" << std::endl;
            return;
        }
        cv::cvtColor( img, gray, CV_BGR2GRAY );

        bool found = false;
        found = cv::findChessboardCorners( img, boardSize, corners,
                                          CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS );
        if ( found ) {
            cornerSubPix( gray, corners, cv::Size( 5, 5 ), cv::Size( -1, -1 ),
                       cv::TermCriteria( CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1 ) );
            drawChessboardCorners( gray, boardSize, corners, found );
        }

        std::vector< cv::Point3f > obj;
        for ( int i = 0; i < boardHeight; i++ )
          for ( int j = 0; j < boardWidth; j++ )
            obj.push_back( cv::Point3f( ( float )j * squareSize, ( float )i * squareSize, 0 ) );

        if ( found ) {
          std::cout << k << ". Found corners!" << std::endl;
          imagePoints.push_back( corners );
          objectPoints.push_back( obj );
        }
    }
}

double computeReprojectionErrors(const std::vector< std::vector< cv::Point3f > >& objectPoints,
                                 const std::vector< std::vector< cv::Point2f > >& imagePoints,
                                 const std::vector< cv::Mat >& rvecs, const std::vector< cv::Mat >& tvecs,
                                 const cv::Mat& cameraMatrix , const cv::Mat& distCoeffs)
{
        std::vector< cv::Point2f > imagePoints2;
        int i, totalPoints = 0;
        double totalErr = 0, err;
        std::vector< float > perViewErrors;
        perViewErrors.resize( objectPoints.size( ) );

        for ( i = 0; i < ( int )objectPoints.size( ); ++i ) {
        projectPoints( cv::Mat( objectPoints[ i ]), rvecs[ i ], tvecs[ i ], cameraMatrix,
                      distCoeffs, imagePoints2 );
        err = cv::norm( cv::Mat( imagePoints[ i ] ), cv::Mat( imagePoints2 ), CV_L2 );
        int n = ( int )objectPoints[ i ].size( );
        perViewErrors[ i ] = ( float ) std::sqrt( err*err / n );
        totalErr += err * err;
        totalPoints += n;
    }
    return std::sqrt( totalErr / totalPoints );
}

//--outputFile=lIntrinsic.yaml --pre=image_left_  --imgsCount=35
//--outputFile=rIntrinsic.yaml --pre=image_right_  --imgsCount=35

int main( int argc, char *argv[ ] ) {
    const cv::String keys =
    "{help h usage ? |                  |print help message}"
    "{hBoard         |6                 |Height of the board}"
    "{wBoard         |9                 |Width of the board}"
    "{imgFolder      |./calibrationImgs/|Directory of images}"
    "{pre            |image_left_       |Image name prefix}"
    "{imgsCount      |10                |count of img}"
    "{squareSize     |0.02423           |square size of the board}"
    "{outputFile     |intrinsic.yaml    |intrinsic camera calibration file name}"
    "{imgExt         |jpg               |Image extension. Ext: jpg,png etc}";

    cv::CommandLineParser parser( argc, argv, keys );
    parser.about("Camera intrinsic params getter v1.0.0");
    if ( parser.has( "help" ) ) {
        parser.printMessage( );
        return 0;
    }

    cv::String imgFolder    = parser.get<cv::String>( "imgFolder" );
    cv::String imgExt       = parser.get<cv::String>( "imgExt" );
    cv::String pre          = parser.get<cv::String>( "pre" );
    int imgsCount           = parser.get<int>   ( "imgsCount" );
    float squareSize        = parser.get<float> ( "squareSize" );
    cv::String outputFile   = parser.get<cv::String>( "outputFile" );

    cv::Size boardSize( parser.get<int>( "wBoard" ), parser.get<int>( "hBoard" ) );
    std::cout << boardSize << std::endl;
    setupCalibration( boardSize.width, boardSize.height, imgsCount, squareSize,
                   imgFolder.c_str( ), pre.c_str( ), imgExt.c_str( ) );

    std::cout << ( "Starting Calibration" ) << std::endl;

    cv::Mat K;
    cv::Mat D;
    std::vector< cv::Mat > rvecs, tvecs;
    int flag = 0;
    flag |= CV_CALIB_FIX_K4;
    flag |= CV_CALIB_FIX_K5;
    calibrateCamera( objectPoints, imagePoints, img.size( ), K, D, rvecs, tvecs, flag );

    std::cout << "ReprojectionError: "
              << computeReprojectionErrors( objectPoints, imagePoints, rvecs, tvecs, K, D )
              << std::endl;

    cv::FileStorage fs( outputFile, cv::FileStorage::WRITE );
    fs << "K" << K;
    fs << "D" << D;
    fs << "board_width" << boardSize.width;
    fs << "board_height" << boardSize.height;
    fs << "square_size" << squareSize;

    std::cout << "Done Calibration" << std::endl;

    return 0;
}
