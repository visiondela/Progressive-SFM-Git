//Progressive Structure from Motion
//************************************************************************************************
//Matthew Westaway
//************************************************************************************************
//Camera motion class headerfile
//************************************************************************************************





//#####################      INCLUDES     #######################

//OPENCV
#include <opencv2\core\core.hpp>
#include <opencv2\features2d\features2d.hpp>
#include <opencv2\nonfree\features2d.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\calib3d\calib3d.hpp>

//BOOST
#include <boost\tuple\tuple.hpp> //to return tuples

//STD
#include <iostream>

using namespace cv;
using namespace std;

//#####################      CAMERA MOTION CLASS     #######################

class Cameramotion
{
	//PRIVATE
	private:
		//VARIABLES
		Mat img1; //Frame 1
		Mat img2; //Frame 2
		vector<KeyPoint> keypoints1, keypoints2; //Vector of keypoints data type includes 2d coords and scale and orietnation 
		Mat descriptors1,descriptors2; //Descriptors
		vector<DMatch> matches; //Matches
		Mat img_matches; //Img_matches
		Mat F; //Fundemental matrix
		Mat E; //Essential matrix
		Mat_<double> R ;// Rotation matrix
		Mat_<double> t ;//Translation matrix
		Matx34d P1; //camera matrix 2
		Matx34d P; //camera matrix 1
		Mat svd_u ; //SVD u
		Mat svd_vt ;//SVD vt
		Mat svd_w ;//SVD w
		vector<Point2f> imgpts1, imgpts2; //Vector of image points, point2f is a float 2d point (x,y)

	//PUBLIC 
	public:
		//CONSTRUCTOR		
		Cameramotion(Mat frame1,Mat frame2);
		//FUNCTION: FIND POINT MATCHES -RICH FEATURES
		boost::tuple<vector<KeyPoint> ,vector<KeyPoint> > Getpointmatches_richfeatures();
		//FUNCTION: GET FUNDAMENTAL MATRIX
		Mat Get_fundamentalmatrix();
		//FUNCTION: GET ESSENTIAL MATRIX
		Mat Get_essentialMatrix(Mat K,Mat F);
		//FUNCTION: GET CAMERA MATRIX
		Matx34d Get_cameraMatrix(Mat E);



		
};

	
