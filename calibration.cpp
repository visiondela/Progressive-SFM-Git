//Progressive SFM - Matthew Westaway - 2015-04-01
//Calibration.cpp

//Header file includes
#pragma once
#include "calibration.h"

//Constructor
Calibration::Calibration(vector<vector<Point2f>>imagepoints,vector<vector<Point3f>>arrayObjectPoints,double frame_width,double frame_length)
{
		theimagepoints = imagepoints;
		thearrayObjectPoints = arrayObjectPoints;
		width = frame_width;
		length = frame_length;
}

//Function to get calibration matrix
Mat Calibration::get_kmatrix()
{	
		Size imageSize(width,length);
		Mat KMatrix; //Calibration matrix
		Mat distCoeffs; //Dist coefficients
		vector<Mat> rvecs;   
		vector<Mat> tvecs;

		//Calibrate Camera
		double rms = calibrateCamera(thearrayObjectPoints,theimagepoints,imageSize, KMatrix, distCoeffs,  rvecs, tvecs);
		cout<<"the RMS is "<<rms<<endl;
		return KMatrix;
	
}

//Function to get distortion coefficients
Mat Calibration::get_disortioncoefficients()
{
		Size imageSize(width,length);
		Mat KMatrix; //Calibration matrix
		Mat distCoeffs; //Dist coefficients
		vector<Mat> rvecs;   
		vector<Mat> tvecs;

		//Calibrate Camera      
		calibrateCamera(thearrayObjectPoints,theimagepoints,imageSize, KMatrix, distCoeffs,  rvecs, tvecs);
		return distCoeffs;	
}
