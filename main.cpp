//Progressive Structure from Motion
//************************************************************************************************
//Matthew Westaway
//************************************************************************************************
//Main Class
//************************************************************************************************





//###################		LIBRARY INCLUDES!         ###################

//OPENCV INCLUDES

#include <opencv2\highgui\highgui.hpp>	//For GUI and Video 
#include <opencv2\calib3d\calib3d.hpp>	//For calibration and 3D reconstruction	
#include <opencv2\core\core.hpp>		//For basic building blocks such as basic image container Mat
#include <opencv2\imgproc\imgproc.hpp>	//For image processing
#include <opencv2\features2d\features2d.hpp> //For keypoints and feature detection
#include <opencv2\nonfree\features2d.hpp>

//BOOST INCLUDES

#include <boost\thread.hpp>
#include <boost\lexical_cast.hpp> //for printing out int
#include <boost/date_time/posix_time/posix_time.hpp>

//POINT CLOUD LIBRARY INCLUDES

#include <pcl\common\common.h>
#include <pcl\visualization\cloud_viewer.h>
#include <pcl\visualization\pcl_visualizer.h>
#include <pcl\io\io.h>
#include <pcl\io\file_io.h>
#include <pcl\io\pcd_io.h>
#include <pcl\ModelCoefficients.h>
#include <pcl\point_types.h>
#include <pcl\sample_consensus\ransac.h>
#include <pcl\sample_consensus\sac_model_plane.h>
#include <pcl\filters\extract_indices.h>
#include <pcl\filters\statistical_outlier_removal.h>
#include <pcl\filters\voxel_grid.h>

//EIGEN INCLUDES

#include <Eigen\Eigen>

//STL INCLUDES
#include <vector>
#include <queue>

//STD INCLUDES
#include <iostream>
#include <time.h> //For use of computer time
#include <stdio.h>

//###################		HEADER FILE INCLUDES!         ###################

#include "cameramotion.h"				
#include "calibration.h"
#include "triangulatepoints.h"


using namespace cv;
using namespace std;

	


vector<Mat> frames;
int frames_reconstructed = 100000000;







Mat_<double> Stream_Calibrate (int stream_code,double frame_width,double frame_length)
{
	
	cv::namedWindow("Stream_Calibrate", WINDOW_NORMAL);

	VideoCapture cap(stream_code);

	

	if (!cap.isOpened())
	{
		cout << "The camera is not connected" << endl;
	}
	
	//Define variables for calibration
	
	vector<vector<Point2f>> imagepoints; //vector for image points in all images - 54 image points per image
	vector<vector<Point3f>> arrayObjectPoints; //vector for object points in all images - 54 object points per image	
	vector<Point3f> objectpoints; //Vector for 54 object points, each object point has xyz. same for all images
	Size patternsize(9,6); //Checkerboard pattern
	double f_width = frame_width;
	double f_length =frame_length;

	//Get location of the checkerboard
	Mat pattern = imread("C:/Users/Matthew/Documents/Visual Studio 2010/Projects/Program 2  - Progressive Structure from Motion/ProgressiveSFM/checkerboard.png"); //Get checkerboard pattern
	

	//Populate object points
	for(int y=0; y<6; ++y) 
	{
		for(int x=0; x<9; ++x)
		{
			objectpoints.push_back(Point3f(20*x,20*y,0));
		}
	}

	//Initialise the counters
	int calibration_no = 0; 
	int frame_no=0;

	while (calibration_no<11)
	{
		//Create frame variable to be captured 
		Mat frame;
		bool bSuccess = cap.read(frame); 
		if (!bSuccess) //if not true
		{
			cout << " Cannot read a frame from the stream . Check to see if the stream is connected and restart program" << endl;

		}
		
		//Show the pattern 
		imshow("Stream_Calibrate",  pattern); 
		waitKey(30);

		Mat frameGray; //Grey frame

		//If its the 100th frame capture the frame
		if (frame_no%100==0)
		{		
			vector<Point2f> corners; //vector for the 54 corner points 

			cvtColor(frame, frameGray, COLOR_BGR2GRAY); //Convert frame to grey

			//Check for calibration pattern
			bool found = findChessboardCorners( frameGray, patternsize, corners, CV_CALIB_CB_ADAPTIVE_THRESH + CV_CALIB_CB_FAST_CHECK + CV_CALIB_CB_NORMALIZE_IMAGE);
			if (found)
			{
				cout << "Calibration capture no"<<calibration_no<< "of 10" << endl; //Print out the calibration frame no
			
				//Get more accurate corners
				cornerSubPix( frameGray, corners, Size(11,11), Size(-1,-1), TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));
					
				//Add the found corner points to the image point vector
				imagepoints.push_back(corners);

				//Add object points to the object point vector
				arrayObjectPoints.push_back(objectpoints);
					
				//Draw the corners 
				drawChessboardCorners( frame,patternsize, Mat(corners), found );
					
				//Display corners captured for 100 miliseconds before going back to displaying hte pattern
				imshow("Stream_Calibrate",frame);
				waitKey(300); 

				//Clear the corners vector
				corners.clear();
			}
				
			else 
			{
				cout<<"failed"<<endl;	//If pattern not found
				calibration_no --;			//Decrement the capture number so the capture will be redone
			}

			calibration_no ++;				//Increment the capture number
		}
		frame_no++;
	}

	//Start calibration
	Calibration a(imagepoints,arrayObjectPoints,f_width,f_length) ;
	Mat_<double> KMatrix = a.get_cameramatrix(); //calibration matrix depends on the resolution of images- ie focal length, the radial distortion parameters are independent of the resolution
	return KMatrix;
}					
						
void Stream (int stream_code)
{
	cv::namedWindow("Stream", WINDOW_NORMAL);
	

	VideoCapture cap(stream_code);

	if (!cap.isOpened())
	{
		cout << "The camera is not connected" << endl;
		frames_reconstructed = 0;
		//do a loop back to void Stream after connect the camera
	}

	


	
	int frame_no=1;
	int frames_captured = 0;
	while(1)
	{
		
		Mat frame;
		bool bSuccess = cap.read(frame); 
		if (!bSuccess) //if not true
		{
			cout << " Cannot read a frame from the stream . Check to see if the stream is connected and restart program" << endl;
			frames_reconstructed = frame_no;
			break;
		}

		imshow("Stream",  frame);
		waitKey(30);
		cout<<frame_no<<endl;

		if (frame_no%20==0)
		{
			frames.push_back(frame);
			cout<<"Captured"<<endl;
			frames_captured++;
		}
		
		frame_no++;	
			
		if(waitKey(30) == 27) //wait for 'esc' key press for 30 ms. If 'esc' key is pressed, break loop
		{
                cout << "esc key is pressed by user" << endl; 
				frames_reconstructed = frames_captured;
                break;
		}
	}
}

void Stream_Process(Mat_<double> KMatrix)
{

	//Create the variables to save files	
	string thefilename; //Name of string where frames will be saved
	vector<int> compression_params; //Initialize the compression parameters which will allow us to save images vector that stores the compression parameters of the image
	compression_params.push_back(CV_IMWRITE_JPEG_QUALITY); //use JPEG compression
	compression_params.push_back(98); //this is the compression quality
	
	//Create the point cloud
	pcl::visualization::CloudViewer viewer("3D Reconstruction");
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud; //create the point cloud
	cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
		
	//Create RGBV value for the point cloud
	Vec3b rgbv(255,255,255);  //black rgbv value
	uint32_t rgb = ((uint32_t)rgbv[2]<<16 | (uint32_t)rgbv[1] << 8 | (uint32_t)rgbv[0]);

	//Initialise that the baseline has not been reconstructed
	string baseline_state= "Baseline not reconstructed";
	int a = 1 ;//counter
	
	while (a<frames_reconstructed)
	{
		Mat frame,frame_back;

		try
		{
			//Get frame
			frame = frames.at(a);
			frame_back=frames.at(a-1);
			
			
	//check for matches between new frame and the past frame








			//Reconstruct the baseline
			if (baseline_state!="Baseline reconstructed")
			{
			
				
				Cameramotion baseline(frame_back,frame); //send the two images

				//Get keypoints
				vector<KeyPoint> unrefined_keypoints1; //Keypoints in frame 1
				vector<KeyPoint> unrefined_keypoints2;//Keypoints in frame 2
				vector<DMatch> unrefined_matches;

				boost::tie(unrefined_keypoints1,unrefined_keypoints2,unrefined_matches)=baseline.Getpointmatches_richfeatures();
	
				//Get the fundamental matrix and refined keypoints
				vector<KeyPoint> pt_set1; //Keypoints in frame 1
				vector<KeyPoint> pt_set2;//Keypoints in frame 2
				Mat fundamentalmatrix;
				vector<DMatch> matches; //correct matches
				boost::tie(pt_set1,pt_set2,fundamentalmatrix,matches) = baseline.Get_fundamentalmatrix(unrefined_keypoints1,unrefined_keypoints2,unrefined_matches);
				
				//Get the essential matrix - must check if correct
				Mat essentialmatrix = baseline.Get_essentialMatrix(KMatrix,fundamentalmatrix);
	
				//Get the camera matrices
				Matx34d cameramatrix(0,0,0,0,0,0,0,0,0,0,0,0);
				Matx34d cameramatrix1 = baseline.Get_cameraMatrix(essentialmatrix);

				//Get calibrated camera matrix
				Mat_<double> KCameramatrix1 = KMatrix*Mat(cameramatrix1);

				//Triangulate the points
				Triangulatepoints base(cameramatrix,cameramatrix1); //send the two images
				unsigned int pts_size = pt_set1.size();
				cout<< pt_set1.size()<< " "<< pt_set2.size()<<endl;
				for (unsigned int i=0;i<pts_size;i++)//pt_set1.size()
				{	
					
					//Create homo keypoint 1
					Point2f kp = pt_set1[i].pt;
					Point3d u(kp.x,kp.y,1.0);
						
					//Normalize homo keypoint 1
					Mat_<double> um = KMatrix.inv()*Mat_<double>(u);
					u.x = um(0);
					u.y = um(1);
					u.z = um(2);
	
					//Create homo keypoint 2
					Point2f kp1 = pt_set2[i].pt;
					Point3d u1(kp1.x,kp1.y,1.0);

					//Normalize homo keypoint 2
					Mat_<double> um1 = KMatrix.inv()*Mat_<double>(u1);
					u1.x = um1(0);
					u1.y = um1(1);
					u1.z = um1(2);


					//Triangulate points
					Mat_<double> X = base.Linearleastsquares_triangulation(u,u1);

					//Calculate the reprojection error
					Mat_<double> xPt_img =  KCameramatrix1*X; //Get the second image coordinate from the triangulated 3D point
					Point2f xPt_img_(xPt_img(0)/xPt_img(2),xPt_img(1)/xPt_img(2)); 
					//error = norm(xPt_img_ - kp1);
	
					//ADD REPROJECTION ERROR TO REPROJECTION ERROR LIST
					//reproj_error.push_back(error); 
						
					//Convert 3D point type to PCL
					pcl::PointXYZRGB pclp; 
					pclp.x = X(0);
					pclp.y = X(1);
					pclp.z = X(2);
					pclp.rgb = *reinterpret_cast<float*>(&rgb);
						
					//Add 3D point to point cloud
					cloud->push_back(pclp);	

				}

				//Calculate the reprojection error
				//mean_projerror = mean (reproj_error);
				//cout<<"The mean reprojection error :" <<mean_projerror[0]<<endl;	
	
				//print out the point count
				//cout<<cloud->points.size()<<endl;
	
				//Visualize the point cloud 
				cloud->width = (uint32_t) cloud->points.size(); //number of points
				cloud->height = 1; //a list of points, one row of data
			
		
				viewer.showCloud(cloud);
		
				baseline_state="Baseline reconstructed";
				a++;
			}	

			else
			{
				//search between the current frame and the previous frame for matches
				
				vector<Point3f> ppcloud_existing; //3d points in current cloud 
				vector<Point2f> imgPoints_new; //2d points in new frame


				//Get matches
				Cameramotion baseline(frame_back,frame); //send the two images

				//Get keypoints
				vector<KeyPoint> unrefined_keypoints1; //Keypoints in frame 1
				vector<KeyPoint> unrefined_keypoints2;//Keypoints in frame 2
				vector<DMatch> unrefined_matches;

				boost::tie(unrefined_keypoints1,unrefined_keypoints2,unrefined_matches)=baseline.Getpointmatches_richfeatures();
	
				//Get the fundamental matrix and refined keypoints
				vector<KeyPoint> pt_set1; //Keypoints in frame 1
				vector<KeyPoint> pt_set2;//Keypoints in frame 2
				Mat fundamentalmatrix;
				vector<DMatch> matches; //correct matches
				boost::tie(pt_set1,pt_set2,fundamentalmatrix,matches) = baseline.Get_fundamentalmatrix(unrefined_keypoints1,unrefined_keypoints2,unrefined_matches);
				
				
			a++;
		}
		catch(...)
		{
			//cout<<"exception"<<endl;
		}
	}

	cout<<"Program finished"<<endl;
	system("pause");
}



int main(int argc, char** argv)
{
	
	
	double frame_width,frame_length; //Size of the frame
	Mat_<double> KMatrix;

	//Get the size of the stream
	cout<<"Please enter the size of the stream , width and then length:640 x 480 "<<endl;
	cin>>frame_width>>frame_length;
	
	Size frame_size(frame_width,frame_length);

	//Get the address of the stream
	cout<<"Enter the stream address: 2 or 0 or http://192.168.2.201:8080/video?x.mjpeg or http://192.168.43.1:8080/video?x.mjpeg or  http://10.117.45.249:8080/video?x.mjpeg "<<endl;
	int stream_code;
	cin>>stream_code;	//http://192.168.2.201:8080/video?x.mjpeg or http://192.168.43.1:8080/video?x.mjpeg
	

	//Find out whether the camera needs to be calibrared.
	cout<<"Enter: '1' to enter a calibration matrix '2' to random generate a calibration matrix or '3' to calibrate the camera"<<endl;
	string calib_yn;
	cin>>calib_yn;
	
	if (calib_yn == "1")
	{
		//Get the calibration matrix from the user
		double a,b,c,d,e,f,g,h,i;
		cout<<"Enter the calibration matrix: "<<endl;
		cin>>a>>b>>c>>d>>e>>f>>g>>h>>i;
		KMatrix = (Mat_<double>(3,3) << a,b,c,d,e,f,g,h,i);
	}

	else if (calib_yn == "2")
	{
		//Randomly generate the calibration matrix
		double max_w_h = MAX(frame_size.height,frame_size.width);
		KMatrix = (cv::Mat_<double>(3,3) <<	max_w_h ,0	, frame_size.width/2.0,0,			max_w_h,	frame_size.height/2.0,0,			0,			1);
	}

	else if (calib_yn == "3")
	{
		KMatrix = Stream_Calibrate(stream_code,frame_width,frame_length);
	}

	//Run capturing and processing at the same time
	boost::thread_group tgroup;

	tgroup.create_thread(boost::bind(&Stream,stream_code));
	//boost::this_thread::sleep(boost::posix_time::seconds(30));
	
	tgroup.create_thread(boost::bind(&Stream_Process,KMatrix));

	tgroup.join_all(); //join where one thread main..waits for all threads to finish

	

	return 0;	
}