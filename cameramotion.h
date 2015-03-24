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
		
	//PUBLIC 
	public:
		//CONSTRUCTOR		
		Cameramotion(Mat frame1,Mat frame2);
		//FUNCTION: FIND POINT MATCHES -RICH FEATURES
		boost::tuple<vector<KeyPoint> ,vector<KeyPoint> , vector<DMatch> > Getpointmatches_richfeatures();
		//FUNCTION: GET FUNDAMENTAL MATRIX
		boost::tuple<vector<KeyPoint>,vector<KeyPoint>,Mat> Get_fundamentalmatrix(vector<KeyPoint> keypoints1,vector<KeyPoint> keypoints2,vector<DMatch>);
		//FUNCTION: GET ESSENTIAL MATRIX
		Mat Get_essentialMatrix(Mat K,Mat F);
		//FUNCTION: GET CAMERA MATRIX
		Matx34d Get_cameraMatrix(Mat E);



		
};

	
