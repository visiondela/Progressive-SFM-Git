//Progressive SFM - Matthew Westaway - 2015-04-01
//Reconstruction.h

//OpenCV Libraries
#include <opencv2\core\core.hpp>
#include <opencv2\features2d\features2d.hpp>
#include <opencv2\nonfree\features2d.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\calib3d\calib3d.hpp>

//Boost libraries
#include <boost\tuple\tuple.hpp> //to return tuples

//STD libraries
#include <iostream>

using namespace cv;
using namespace std;

class Reconstruction
{
	private:
		Mat img1; //Frame 1
		Mat img2; //Frame 2

	public:
		//Constructor	
		Reconstruction(Mat frame1,Mat frame2);
		//Get keypoints
		vector<KeyPoint> Getkeypoints_left();
		vector<KeyPoint> Getkeypoints_right();
		//Find matches
		vector<DMatch> Getmatches_richfeatures (vector<KeyPoint> keypoints1,vector<KeyPoint> keypoints2);
		//Prune matches
		vector<DMatch> Prunematches(vector<KeyPoint> keypoints1,vector<KeyPoint> keypoints2,vector<DMatch> matches);
		//Get fundamental matrix
		Mat_<double>  Getfundamentalmatrix(vector<KeyPoint> keypoints1,vector<KeyPoint> keypoints2,vector<DMatch> matches);
		//Get essential matrix
		Mat_<double>  Getessentialmatrix(Mat K,Mat F);
		//Get camera matrix
		Matx34d Getcameramatrix_right(Mat E, Mat K,vector<KeyPoint> keypoints1,vector<KeyPoint> keypoints2,vector<DMatch> matches);
		//Triangulate a point by linear least squares
		Mat_<double> Triangulatepoint_linearleastsquares(Point3d u1,Point3d u2,Matx34d P1,Matx34d P2);
		//Triangulate a point by iterative least squares
		Mat_<double> Triangulatepoint_iterativeleastsquares(Point3d u1,Point3d u2,Matx34d P1,Matx34d P2);
};

	
