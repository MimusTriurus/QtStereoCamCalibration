# QtStereoCamCalibration
Calibration of stereocamera
---------
Project structure and application description:
---------
ImgGrabber - saving of stereopair image with calibration board; 

IntrinsicParamGetter - calculation of inner parameters during camera calibration (for left and right camera separately); 

StereoCamCalibration - calculation of parameters during stereocamera calibration; 

UndistortAndRectifyImg – checking of results after camera calibration. Distortion removal, rectification, framing and saving of calibrated stereopair as a file. 

System requirements: 
-------
1. ОС: Windows; 
2. Qt 5.5.1 or higher; 
3. Compiler with support for c++11 (as a default using MSVC 2013 64bit); 
4. OpenCv 3 (as a default using opencv_world331.dll from directory "QtStereoCamCalibration/thirdparty/lib/"). 

Setup for Windows: 
-------
1. Clone the repository; 
2. Switch to assembly pattern "Release"; 
3. Build main project QtStereoCamCalibration; 
4. Copy library opencv_world331.dll from directory "QtStereoCamCalibration/thirdparty/lib/" to the folder with executable file; 
5. Copy (if required) libraries Qt to the folder with executable file. 

Remark: 
-------
1. Compiled applications are located in folder “bin”; 
2. To get reference data launch application flagged as “--help”
3. Build - https://cloud.mail.ru/public/5iHb/CbBNiNDin
