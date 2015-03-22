//Progressive Structure from Motion
//************************************************************************************************
//Matthew Westaway
//************************************************************************************************
//Triangulate Points class headerfile
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

//STD 
#include <vector>

//STL
#include <iostream>

using namespace std;
using namespace cv;


//#####################      TRIANGULATE POINTS     #######################
class Triangulatepoints
{
	//PRIVATE
	private:
		//VARIABLES
		Matx34d P; //Camera 1 matrix
		Matx34d P1; //camera 2 matrix

	//PUBLIC
	public:
		//CONSTRUCTOR		
		Triangulatepoints(Matx34d Cameramatrix1, Matx34d Cameramatrix2);
		//FUNCTION: TRIANGULATE
		Mat_<double> Linearleastsquares_triangulation(Point3d point1,Point3d point2);
	
};