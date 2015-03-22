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


int main(int argc, char** argv)
{
	
	//###################		DECLARE & INITIALIZE VARIABLES        ###################

	//VARIABLES FOR FRAMES

	int frame_no = 1; //Number of frames
	int capture_no ; //Number of frames captured
	vector<Mat> baselineframes; //Create a vector to store frames in
	string thefilename; //Name of string where frames will be saved
	double frame_width,frame_length; //Size of the frame

	//VARIABLES FOR CALIBRATION

	vector<Point2f> corners; //vector for the 54 corner points 
	vector<Point3f> objectpoints; //Vector for 54 object points, each object point has xyz. same for all images
	vector<vector<Point2f>> imagepoints; //vector for image points in all images - 54 image points per image
	vector<vector<Point3f>> arrayObjectPoints; //vector for object points in all images - 54 object points per image	
	Mat frameGray; //Grey frame
	Size patternsize(9,6); //Checkerboard pattern
	Mat_<double> KMatrix(3,3,CV_64F); //Create K matrix
	Mat pattern = imread("C:/Users/Matthew/OneDrive/MScGeomatics/Program 2  - Progressive Structure from Motion/ProgressiveSFM/checkerboard.png"); //Get checkerboard pattern
	
	//VARIABLES FOR RECONSTRUCTION

	vector<KeyPoint> pt_set1; //Keypoints in frame 1
	vector<KeyPoint> pt_set2;//Keypoints in frame 2
	Mat fundamentalmatrix;//Fundamental matrix
	Mat essentialmatrix; //Essential matrix
	Matx34d cameramatrix; //Camera matrix 1
	Matx34d cameramatrix1; //Camera matrix 2
	Mat_<double> KCameramatrix1; //Calibrated Camera matrix 1
	Point2f kp; //Image 1 keypoint
	Point2f kp1; //Image 2 keypoint
	Mat_<double> um; //Image 1 keypoint homgeneous and normalized
	Mat_<double> um1; //Image 2 keypoint homgeneous and normalized
	Mat_<double> X; //3D Triangulated point
	Mat_<double> xPt_img; //Reprojected image point
	double error; //reprojection error
	vector<double> reproj_error; //vector of the reprojection errors
	Scalar mean_projerror; //mean reprojection error 
	vector<Point3d> pointcloud; //final point cloud
	
	//VARIABLES FOR VISUALIZATION
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud; //create the point cloud
	cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
	Vec3b rgbv(255,255,255);  //black rgbv value
	uint32_t rgb = ((uint32_t)rgbv[2]<<16 | (uint32_t)rgbv[1] << 8 | (uint32_t)rgbv[0]);

	//VARIALBLES FOR SAVING THE IMAGE

	vector<int> compression_params; //Initialize the compression parameters which will allow us to save images vector that stores the compression parameters of the image
	compression_params.push_back(CV_IMWRITE_JPEG_QUALITY); //use JPEG compression
	compression_params.push_back(98); //this is the compression quality
	


	//###################   CREATE WINDOWS!   ###################
	
	namedWindow("Stream",WINDOW_NORMAL);
	resizeWindow("Stream",1920/2, 1080/2); //Resize window
	pcl::visualization::CloudViewer viewer("Cloud Viewer");


		
	//###################   SET UP THE STREAM!   ###################

	//GET SIZE OF THE STREAM FRAME
	cout<<"Please enter the size of the stream , width and then length: "<<endl;
	cin>>frame_width>>frame_length;
	Size frame_size(frame_width,frame_length);

	//GET THE ADDRESS OF THE STREAM
	cout<<"Enter the stream address: 2 or 0 or http://192.168.2.201:8080/video?x.mjpeg or http://192.168.43.1:8080/video?x.mjpeg "<<endl;
	string stream_code ;
	cin>>stream_code;	//http://192.168.2.201:8080/video?x.mjpeg or http://192.168.43.1:8080/video?x.mjpeg
	
	//CREATE INSTANCE OF THE STREAM CLASS
	VideoCapture cap(stream_code);
	
	//TEST IF THE STREAM IS WORKING
	if (!cap.isOpened())
	{
		cout << "The camera is not connected" << endl;
		return -3; //Close program if not working
	}

	//###################     CALIBRATE THE CAMERA!      ###################



	//FIND OUT WHETHER THE CAMERA NEEDS TO BE CALIBRATED OR IF THE CALIBRATION MATRIX IS KNOWN OR MUST BE RANDOMLY GENERATED
	cout<<"Enter: '1' to enter a calibration matrix '2' to random generate a calibration matrix or '3' to calibrate the camera"<<endl;
	string calib_yn;
	cin>>calib_yn;

	if (calib_yn == "1")
	{
		//GET THE CALIBRATION MATRIX FROM THE USER
		double a,b,c,d,e,f,g,h,i;
		cout<<"Enter the calibration matrix: "<<endl;
		cin>>a>>b>>c>>d>>e>>f>>g>>h>>i;
		KMatrix = (Mat_<double>(3,3) << a,b,c,d,e,f,g,h,i);
		capture_no = 11;	//Set capture number as 11 so to skip the calibrate iterations
	}

	else if (calib_yn == "2")
	{
		//RANDOMLY GENERATE CALIBRATION MATRIX 
		double max_w_h = MAX(frame_size.height,frame_size.width);
		KMatrix = (cv::Mat_<double>(3,3) <<	max_w_h ,0	, frame_size.width/2.0,0,			max_w_h,	frame_size.height/2.0,0,			0,			1);
		capture_no = 11;
	}

	else if (calib_yn == "3")
	{
		//DETERMINE THE CALIBRATION MATRIX FROM CHECKERBOARD CAPTURES

		//CREATE OBJECT POINTS FOR CHECKERBOARD
		for(int y=0; y<6; ++y) 
		{
			for(int x=0; x<9; ++x)
			{
				objectpoints.push_back(Point3f(20*x,20*y,0));
			}
		}
		capture_no = 1;		//Set capture number as 1 so to include the calibrate pattern capture iterations 
	}

	
	//###################     START THE CAMERA!      ###################


	//START AN INFINITE LOOP OF THE CAMERA
	while (1)
	{
		//###################     DISPLAY FRAMES!      ######################

		Mat frame;
		bool bSuccess = cap.read(frame); 
		cout<<frame_no<<endl; //Print out the frame number
		//CHECK IF FRAME DISPLAY WAS SUCCESFULL
		if (!bSuccess) //if not true
		{
			cout << " Cannot read a frame from the stream . Check to see if the stream is connected and restart program" << endl;
			return -3;
		}

		//###################     IF CAPTURE NUMBER LESS THAN TEN - CALIBRATION PATTERN CAPTURE MODE !      ######################

		if (capture_no <= 10) 
		{		
			//DISPLAY CALIBRATION PATTERN TO WINDOW
			imshow("Stream",  pattern); 
			waitKey(30);
			
			//###################     EVERY 100 FRAMES GRAB A FRAME!      ######################

			if (frame_no%100==0) 
			{
				cvtColor(frame, frameGray, COLOR_BGR2GRAY); //Convert frame to grey

				//###################     CHECK FOR CALIBRATION PATTERN!      ######################

				bool found = findChessboardCorners( frameGray, patternsize, corners, CV_CALIB_CB_ADAPTIVE_THRESH + CV_CALIB_CB_FAST_CHECK + CV_CALIB_CB_NORMALIZE_IMAGE);
				if (found)
				{
					cout << "Calibration capture no"<<capture_no<< "of 10" << endl; //Print out the calibration frame no
			
					//GET MORE ACCURATE CORNERS
					cornerSubPix( frameGray, corners, Size(11,11), Size(-1,-1), TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));
					
					//ADD CORNERS TO IMAGE POINT VECTOR 
					imagepoints.push_back(corners);

					//ADD OBJECT POINTS TO OBJECT POINT VECTOR
					arrayObjectPoints.push_back(objectpoints);
					
					//DRAW THE CORNERS 
					drawChessboardCorners( frame,patternsize, Mat(corners), found );
					imshow("Stream",frame);
					waitKey(100); //Display corners captured for 100 miliseconds before going back to displaying hte pattern

					//CLEAR CORNERS
					corners.clear();

					//SAVE FRAME TO FILE
					thefilename = "C:/Users/Matthew/OneDrive/MScGeomatics/Program 2  - Progressive Structure from Motion/ProgressiveSFM/Images/frame number "+ boost::lexical_cast<string>(frame_no) + "capture no "+ boost::lexical_cast<string>(capture_no) + " calibration.jpg";
					bool bSuccess = imwrite(thefilename, frame, compression_params);
					
					//CHECK TO SEE IF THE FILE WRITE IS SUCCESFULL
					if (!bSuccess)
					{
						cout << "ERROR: Failed to save the image" << endl;
					}
					
					//###################     IF CALIBRATION PATTERN CAPTURE NO = 10 : CALIBRATION PATTERN CAPTURE MODE OVER!      ######################

					if (capture_no==10)
					{
						//START CALIBRATION
						Calibration a(imagepoints,arrayObjectPoints,frame_width,frame_length) ;
						KMatrix = a.get_cameramatrix(); //calibration matrix depends on the resolution of images- ie focal length, the radial distortion parameters are independent of the resolution
						
						//SEE IF THE USER IS READY TO SCAN THE ROOM
						cout<<"Hit enter to start 3D model reconstruction , a frame will be captured every ten seconds"<<endl;
						waitKey(0);
					}
				}
				
				else 
				{
					cout<<"failed"<<endl;	//If pattern not found
					capture_no--;			//Decrement the capture number so the capture will be redone
				}

				capture_no++;				//Increment the capture number
			}

		}	

		//###################     IF CAPTURE NUMBER MORE THAN TEN - 3D RECONSTRUCTION MODE !      ######################

		else if (capture_no > 10)
		{
			//DISPLAY STREAM FRAMES TO WINDOW
			imshow("Stream",  frame);
			waitKey(30);
			
			//###################     EVERY 100 FRAMES GRAB A FRAME!      ######################

			if(frame_no%100==0) 
			{

				//DISPLAY THE FRAME
				imshow("Stream", frame);
				waitKey(30);
				cout << "Frame no"<<frame_no<< " captured " << endl; //Print out the frame number captured

				//SAVE THE FRAME
				thefilename = "C:/Users/Matthew/OneDrive/MScGeomatics/Program 2  - Progressive Structure from Motion/ProgressiveSFM/Images/frame number "+ boost::lexical_cast<string>(frame_no) + "capture no "+ boost::lexical_cast<string>(capture_no) + " .jpg";
				bool bSuccess = imwrite(thefilename, frame, compression_params);//Check to see if the file writing is working
				
				//CHECK TO SEE IF THE FILE WRITE IS SUCCESFULL
				if (!bSuccess)
				{
					cout << "ERROR: Failed to save the image" << endl;
				}
				
				//ADD FRAME VECTOR TO VECTOR OF FRAMES
				baselineframes.push_back(frame);
				
				//###################    IF FRAME VECTOR SIZE IS 2: INITIAL RECONSTRUCTION MODE!      ######################

				if (baselineframes.size()==2) //if we have first two frames then get the camera motion between them.
				{
					Cameramotion baseline(baselineframes.at(0),baselineframes.at(1)); //send the two images
					
					//GET KEYPOINTS
					boost::tie(pt_set1,pt_set2)=baseline.Getpointmatches_richfeatures();
					
					//GET FUNDAMENTAL MATRIX
					fundamentalmatrix = baseline.Get_fundamentalmatrix();

					//GET ESSENTIAL MATRIX
					essentialmatrix = baseline.Get_essentialMatrix(KMatrix,fundamentalmatrix);
				
					//GET CAMERA MATRIX 1
					cameramatrix1 = baseline.Get_cameraMatrix(essentialmatrix);
					//3x3 3x4 = 3x4
					//GET CALIBRATED CAMERA MATRIX
					KCameramatrix1 = KMatrix*Mat(cameramatrix1);
					cout<<"KCameramatrix1"<<KCameramatrix1<<endl;
					//CHECK IF THE FUNDAMENTAL MATRIX IS CORRECT
					//The fundemantal matrix has an error therefore this frame needs to be recaptured"
				
					//###################    TRIANGULATE POINTS    ######################
					Triangulatepoints base(cameramatrix,cameramatrix1); //send the two images

					for (unsigned int i=0;i<pt_set1.size();i++)
					{	
						cout<<i<<endl;
						//CREATE HEMOGENEOUS KEYPOINT 1
						kp = pt_set1[i].pt;
						Point3d u(kp.x,kp.y,1.0);
						
						//NORMALIZE HEMOGENEOUS KEYPOINT 1
					
						um = KMatrix.inv()*Mat_<double>(u);
						u.x = um(0);
						u.y = um(1);
						u.z = um(2);
		
						//CREATE HEMOGENEOUS KEYPOINT 2
						kp1 = pt_set2[i].pt;
						Point3d u1(kp1.x,kp1.y,1.0);

						//NORMALIZE HEMOGENEOUS KEYPOINT 2
						um1 = KMatrix.inv()*Mat_<double>(u1);
						u1.x = um1(0);
						u1.y = um1(1);
						u1.z = um1(2);
	
						//TRIANGULATE
						X = base.Linearleastsquares_triangulation(u,u1);
						cout<<"X"<<X<<endl;
						//CALCULATE REPROJECTION ERROR.
						xPt_img =  KCameramatrix1*X; //Get the second image coordinate from the triangulated 3D point
						Point2f xPt_img_(xPt_img(0)/xPt_img(2),xPt_img(1)/xPt_img(2)); 
						//error = norm(xPt_img_ - kp1);
						
						//ADD REPROJECTION ERROR TO REPROJECTION ERROR LIST
						//reproj_error.push_back(error); 
						
						//CONVERT 3D POINT TO POINT CLOUD 3D POINT TYPE
						pcl::PointXYZRGB pclp; 
						pclp.x = X(0);
						pclp.y = X(1);
						pclp.z = X(2);
						cout<<" "<<pclp.x<<" "<<pclp.y<<" "<<pclp.z<<endl;
						pclp.rgb = *reinterpret_cast<float*>(&rgb);
						
						//ADD 3D POINT TO POINT CLOUD
						cloud->push_back(pclp);
					
					}
					cout<<"done"<<endl;
					//CALCULATE REPROJECTION ERROR 
					mean_projerror = mean (reproj_error);
					cout<<"The mean reprojection error :" <<mean_projerror[0]<<endl;	
				
					//PRINT OUT THE POINT COUNT
					cout<<cloud->points.size()<<endl;
				
					//###################    VISUALIZE THE POINT CLOUD    ######################
					cloud->width = (uint32_t) cloud->points.size(); //number of points
					cloud->height = 1; //a list of points, one row of data
					viewer.showCloud(cloud);
								
				}

				//###################    IF FRAME VECTOR SIZE IS GREATER THAN 3: PROGRESSIVE RECONSTRUCTION MODE!      ######################
				if (baselineframes.size()>2)
				{
				
					cout<<"done"<<endl;
				}

				capture_no++; //Increment the capture number
			}
		}

		frame_no++; //Increment the frame number
	}
	return 0;	
}
	

	

