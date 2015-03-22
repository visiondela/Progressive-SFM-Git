//Progressive Structure from Motion
//************************************************************************************************
//Matthew Westaway
//************************************************************************************************
//Calibration class
//************************************************************************************************





//#####################      HEADER FILE INCLUDES     #######################

#pragma once
#include "calibration.h"



//#####################      CONSTRUCTOR     #######################

Calibration::Calibration(vector<vector<Point2f>>imagepoints,vector<vector<Point3f>>arrayObjectPoints,double frame_width,double frame_length)
{
	
		theimagepoints = imagepoints;
		thearrayObjectPoints = arrayObjectPoints;
		width = frame_width;
		length = frame_length;
}

//#####################     FUNCTION: GET CAMERA/CALIBRATION MATRIX      #######################
Mat Calibration::get_cameramatrix()
{

		Size imageSize(width,length);
		//Calibrate Camera
		double rms = calibrateCamera(thearrayObjectPoints,theimagepoints,imageSize, cameraMatrix, distCoeffs,  rvecs, tvecs);
		cout<<"the RMS is "<<rms<<endl;
		return cameraMatrix;
		
}

//#####################     FUNCTION: GET DISTORTION COEFFICIENTS      #######################
Mat Calibration::get_disortioncoefficients()
{

		Size imageSize(width,length);
		//Calibrate Camera      
		calibrateCamera(thearrayObjectPoints,theimagepoints,imageSize, cameraMatrix, distCoeffs,  rvecs, tvecs);
		return distCoeffs;
		
}
