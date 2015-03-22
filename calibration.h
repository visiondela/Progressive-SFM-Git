//Progressive Structure from Motion
//************************************************************************************************
//Matthew Westaway
//************************************************************************************************
//Calibration class headerfile
//************************************************************************************************





//#####################      INCLUDES     #######################

//OpenCV
#include <opencv2\core\core.hpp>
#include <opencv2\features2d\features2d.hpp>
#include <opencv2\features2d\features2d.hpp>
#include <opencv2\nonfree\features2d.hpp>
#include <opencv2\core\core.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\calib3d\calib3d.hpp>

//STL
#include <vector>

//STD
#include <iostream>

using namespace std;
using namespace cv;

//#####################      CALIBRATION CLASS     #######################

class Calibration
{
	//PRIVATE
	private:
		//VARIABLES
		Mat cameraMatrix; //cameraMatrix 1
		Mat distCoeffs; //Dist coefficients
		vector<Mat> rvecs;   
		vector<Mat> tvecs;
		vector<vector<Point2f>> theimagepoints; //Imagepoints
		vector<vector<Point3f>> thearrayObjectPoints; //Objectpoints
		double width,length;//frame width and length 

	//PUBLIC

	public:
		//CONSTRUCTOR	
		Calibration(vector<vector<Point2f>> imagepoints,vector<vector<Point3f>> arrayObjectPoints,double frame_width,double frame_length);
		//FUNCTION: GET CAMERA MATRIX
		Mat get_cameramatrix();
		//FUNCTION: GET DISTORTION COEFFICINETS 
		Mat get_disortioncoefficients();
	
		
};

