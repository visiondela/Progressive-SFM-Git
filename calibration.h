//Progressive SFM - Matthew Westaway - 2015-04-01
//Calibration.h

//OpenCV libraries
#include <opencv2\core\core.hpp>
#include <opencv2\features2d\features2d.hpp>
#include <opencv2\features2d\features2d.hpp>
#include <opencv2\nonfree\features2d.hpp>
#include <opencv2\core\core.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\calib3d\calib3d.hpp>

//STL libraries
#include <vector>

//STD libraries
#include <iostream>

using namespace std;
using namespace cv;

class Calibration
{

	private:
		vector<vector<Point2f>> theimagepoints; //Imagepoints
		vector<vector<Point3f>> thearrayObjectPoints; //Objectpoints
		double width; //Frame_width
		double length;//Frame length

	public:
		//Constructor
		Calibration(vector<vector<Point2f>> imagepoints,vector<vector<Point3f>> arrayObjectPoints,double frame_width,double frame_length);
		//Get calibration matrix
		Mat get_kmatrix();
		//Get distortion coefficients
		Mat get_disortioncoefficients();
	
		
};

