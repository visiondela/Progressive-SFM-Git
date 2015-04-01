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

#include "reconstruction.h"				
#include "calibration.h"



using namespace cv;
using namespace std;

	


vector<Mat> frames;
int frames_reconstructed = 100000000;



struct CloudPoint
{
	Point3d pt;
	vector<int> index_of_2d_origin; //1000frames
};




boost::tuple<Mat_<double>,Mat_<double> > Stream_Calibrate (string stream_code)
{
	
	cv::namedWindow("Stream_Calibrate");
	//cv::resizeWindow("Stream_Calibrate",frame_width/2,frame_length/2);

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
	double f_width = cap.get(CV_CAP_PROP_FRAME_WIDTH);
	double f_length = cap.get(CV_CAP_PROP_FRAME_HEIGHT);
	
	Size frameSize(static_cast<int>(f_width), static_cast<int>(f_length));

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
		cout<<frame_no<<endl;
		//If its the 100th frame capture the frame
		if (frame_no%10==0)
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
	Mat_<double> KMatrix = a.get_kmatrix(); //calibration matrix depends on the resolution of images- ie focal length, the radial distortion parameters are independent of the resolution
	Mat_<double> distortion_coeff = a.get_disortioncoefficients();
	return boost::make_tuple(KMatrix ,distortion_coeff);
}					

void Stream (string stream_code)
{
	cv::namedWindow("Stream");
	
	
	VideoCapture cap(stream_code);
	//cap.set(CV_CAP_PROP_POS_MSEC, 3000);

	//double fps = cap.get(CV_CAP_PROP_FPS); //get the frames per seconds of the video
 //   cout << "Frame per seconds : " << fps << endl;

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

void Stream_Process(Mat_<double> KMatrix,Mat_<double> distcoeff)
{
	
	//Setup the point cloud for visualiation
	pcl::visualization::CloudViewer viewer("3D Reconstruction");
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud; //create the point cloud
	cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
	Vec3b rgbv(255,255,255);  //black rgbv value for the point cloud
	uint32_t rgb = ((uint32_t)rgbv[2]<<16 | (uint32_t)rgbv[1] << 8 | (uint32_t)rgbv[0]);

	//Initialise that the state of the baseline
	string baseline_state = "Baseline has not been reconstructed";
	
	int a = 1 ;//Initialise the counter for the number of frames captured
	
	//Setup our global point cloud for processing
	vector <CloudPoint> pcloud; //our global point cloud
	vector<int> pcloud_status(10000,0); //too see if points been used before

	//Initialise our key variables
	vector<KeyPoint> keypoints_left; //Keypoints in left_frame
	vector<KeyPoint> keypoints_right;//Keypoints in right frame
	vector<DMatch> matches; //The matches
	Matx34d cameramatrix_left(1,0,0,0,0,1,0,0,0,0,1,0); //Camera matrix in left frame
	Matx34d cameramatrix_right; //camera matrix in right frame

	//Start processing loop
	while (a<frames_reconstructed)
	{
		//Create array to store the left and right frame in
		Mat frame_left;
		Mat frame_right;
		vector<double> reproj_error; //the reprojection error
		
		try
		{
			//Get frame
			frame_right = frames.at(a);
			frame_left = frames.at(a-1);

			//Initialise 3D point vector of existing 3D points for PNP
			vector<Point3f> ppcloud;
			
			//Initialise 2D point vector of points in new frame for PNP 
			vector<Point2f> imgpoints;

			//Reconstruct the baseline
			if (baseline_state != "reconstructed")
			{
				//Call the constructor
				Reconstruction baseline(frame_left,frame_right); 
				/*imshow("Frame left_baseline",frame_left);
				waitKey(30);
				imshow("Frame_right_baseline",frame_right);
				waitKey(30);*/
				//Get keypoints
			
				keypoints_left = baseline.Getkeypoints_left();
				keypoints_right = baseline.Getkeypoints_right();
				
				//Get matches
				matches = baseline.Getmatches_richfeatures(keypoints_left,keypoints_right);

				//Refine matches
				matches = baseline.Prunematches(keypoints_left,keypoints_right,matches);
				
				//Get the fundamental matrix 
				
				Mat fundamentalmatrix = baseline.Getfundamentalmatrix(keypoints_left,keypoints_right,matches);
				
				//Get the essential matrix
				Mat essentialmatrix = baseline.Getessentialmatrix(KMatrix,fundamentalmatrix);
			
				//Get the camera matrices
				cameramatrix_right = baseline.Getcameramatrix_right(essentialmatrix);
			
				//Update the state of reconstruction
		
				cout<<"cameramatrix_left"<<cameramatrix_left<<endl;
				cout<<"cameramatrix_right"<<cameramatrix_right<<endl;

				//Get calibrated camera matrix
				Mat_<double> KCameramatrix_right = KMatrix*Mat(cameramatrix_right);
				baseline_state = "reconstructed";
				
				//Go through each point
				for (unsigned int i=0;i<matches.size() ;i++)
				{	

					//Homogeneousize keypoint
					Point2f kp = keypoints_left[matches[i].queryIdx].pt;
					Point3d u(kp.x,kp.y,1.0);
					
					Point2f kp1 = keypoints_right[matches[i].trainIdx].pt;
					Point3d u1(kp1.x,kp1.y,1.0);

					//Normalize homogeenous keypoint
					Mat_<double> um = KMatrix.inv()*Mat_<double>(u);
					u.x = um(0);
					u.y = um(1);
					u.z = um(2);

					Mat_<double> um1 = KMatrix.inv()*Mat_<double>(u1);
					u1.x = um1(0);
					u1.y = um1(1);
					u1.z = um1(2);
					
					//Triangulate keypoint
					
					Mat_<double> X = baseline.Triangulatepoint_iterativeleastsquares(u,u1,cameramatrix_left,cameramatrix_right);
					
					//Calculate the reprojection error
					Mat_<double> xPt_img =  KCameramatrix_right*X; //Get the second image coordinate from the triangulated 3D point
					Point2f xPt_img_(xPt_img(0)/xPt_img(2),xPt_img(1)/xPt_img(2)); 
					//cout<<norm(xPt_img_ - kp1)<<endl;
					reproj_error.push_back(norm(xPt_img_ - kp1));
				
					//Add 3D point to our global point cloud
					CloudPoint newpoint;
					newpoint.pt = Point3d(X(0),X(1),X(2));
					newpoint.index_of_2d_origin.push_back(matches[i].queryIdx);
					newpoint.index_of_2d_origin.push_back(matches[i].trainIdx);
					pcloud.push_back(newpoint);
					cout<<"z"<<X(2)<<endl;
					//Convert 3D point type to PCL type
					pcl::PointXYZRGB pclp; 
					pclp.x = X(0);
					pclp.y = X(1);
					pclp.z = X(2);
					pclp.rgb = *reinterpret_cast<float*>(&rgb);
						
					//Add 3D point to point cloud
					cloud->push_back(pclp);	
			
				}

				//Calculate the mean reprojection error
				Scalar mean_projerror = mean(reproj_error);
				cout<<"The mean reprojection error :" <<mean_projerror[0]<<endl;	
	
				//Print out the point count
				//cout<<cloud->points.size()<<endl;
	
				//Visualize the point cloud 
				cloud->width = (uint32_t) cloud->points.size(); //number of points
				cloud->height = 1; //a list of points, one row of data
				viewer.showCloud(cloud);
		
				//Move onto the next frame
			
			}


			else if (baseline_state == "reconstructed")
			{
				//Call the constructor
				Reconstruction nextview(frame_left,frame_right); 
				
				//Get camera matrix and keypoints in new frame ( old right frame)
				cameramatrix_left = cameramatrix_right;
				keypoints_left = keypoints_right; //Use previous right frame keypoints 
				
				keypoints_right = nextview.Getkeypoints_right(); //Get keypoints in left frame
			

				//Get matches
				matches = nextview.Getmatches_richfeatures(keypoints_left,keypoints_right);
				matches = nextview.Prunematches(keypoints_left,keypoints_right,matches);

				for (int c = 0;c<matches.size();c++)
				{
					int index_in_old_view= matches[c].queryIdx;
					
					for (int d = 0 ; d<pcloud.size();d++)
					{
						if (index_in_old_view==pcloud[d].index_of_2d_origin.at(1) && pcloud_status[d]==0)
						{
							
							ppcloud.push_back(pcloud[d].pt); 
						
							//Add 2D point in current new frame to 
							Point2f pt = keypoints_right[matches[c].trainIdx].pt;
							imgpoints.push_back(pt);
					
							//We hve use the 3d point now
							pcloud_status[d]==1; 

							//We have found a corresponding 3D point for our 2D point so break
							break;
						}
					}
	
				}
			
				//Now we have an alligned pairing of 3D points in the scene to 2D points in the new frame- we use them to recover the camera position as follows
				Mat_<double> t,rvec,R,inliers;
				cout<<"size 3d points"<<ppcloud.size()<<"size image points"<<imgpoints.size()<<endl;
				
				//solvePnPRansac(ppcloud,imgpoints,KMatrix,distcoeff,rvec,t,false,CV_ITERATIVE );
				solvePnPRansac(ppcloud,imgpoints,KMatrix,distcoeff,rvec,t,false,500,8,0.9*imgpoints.size(),noArray(),ITERATIVE);
				Rodrigues(rvec,R);
				cout<<"R"<<R<<endl;
				cameramatrix_right = Matx34d(R(0,0),R(0,1),R(0,2),t(0),R(1,0),R(1,1),R(1,2),t(1),R(2,0),R(2,1),R(2,2),t(2));
				
				ppcloud.clear();
				imgpoints.clear();

				//Get calibrated camera matrix
				Mat_<double> KCameramatrix_right = KMatrix*Mat(cameramatrix_right);
				
				//Go through each point
				for (unsigned int i=0;i<matches.size() ;i++)
				{	

					//Homogeneousize keypoint
					Point2f kp = keypoints_left[matches[i].queryIdx].pt;
					Point3d u(kp.x,kp.y,1.0);
					
					Point2f kp1 = keypoints_right[matches[i].trainIdx].pt;
					Point3d u1(kp1.x,kp1.y,1.0);

					//Normalize homogeenous keypoint
					Mat_<double> um = KMatrix.inv()*Mat_<double>(u);
					u.x = um(0);
					u.y = um(1);
					u.z = um(2);

					Mat_<double> um1 = KMatrix.inv()*Mat_<double>(u1);
					u1.x = um1(0);
					u1.y = um1(1);
					u1.z = um1(2);
					
					//Triangulate keypoint
					Mat_<double> X = nextview.Triangulatepoint_iterativeleastsquares(u,u1,cameramatrix_left,cameramatrix_right);
					
					//Calculate the reprojection error
					Mat_<double> xPt_img =  KCameramatrix_right*X; //Get the second image coordinate from the triangulated 3D point
					Point2f xPt_img_(xPt_img(0)/xPt_img(2),xPt_img(1)/xPt_img(2)); 
					//cout<<norm(xPt_img_ - kp1)<<endl;
					reproj_error.push_back(norm(xPt_img_ - kp1));
				
					//Add 3D point to our global point cloud
					CloudPoint newpoint;
					newpoint.pt = Point3d(X(0),X(1),X(2));
					newpoint.index_of_2d_origin.push_back(matches[i].queryIdx);
					newpoint.index_of_2d_origin.push_back(matches[i].trainIdx);
					pcloud.push_back(newpoint);

					//Convert 3D point type to PCL type
					pcl::PointXYZRGB pclp; 
					pclp.x = X(0);
					pclp.y = X(1);
					pclp.z = X(2);
					pclp.rgb = *reinterpret_cast<float*>(&rgb);
						
					//Add 3D point to point cloud
					cloud->push_back(pclp);	
			
				}

				//Calculate the mean reprojection error
				Scalar mean_projerror = mean(reproj_error);
				cout<<"The mean reprojection error :" <<mean_projerror[0]<<endl;	
	
				//Print out the point count
				//cout<<cloud->points.size()<<endl;
	
				//Visualize the point cloud 
				cloud->width = (uint32_t) cloud->points.size(); //number of points
				cloud->height = 1; //a list of points, one row of data
				viewer.showCloud(cloud);
		
				
			}
			

			a++;
		}
		catch(...)
		{
			
		}
	}

	cout<<"Program finished"<<endl;
	system("pause");
}



int main(int argc, char** argv)
{
	//Intrinisic parameter variables
	Mat_<double> KMatrix; 
	Mat_<double> distortion_coeff;

	//Get the address of the stream
	cout<<"Enter the stream address: 2 or 0 or http://192.168.2.201:8080/video?x.mjpeg or http://192.168.43.1:8080/video?x.mjpeg or  http://10.117.45.249:8080/video?x.mjpeg "<<endl;
	string stream_code;
	stream_code = "C:/Users/Matthew/Documents/Visual Studio 2010/Projects/Program 2  - Progressive Structure from Motion/ProgressiveSFM/Images/20150330_091845old.mp4";
	string calib_code = "http://192.168.43.1:8080/video?x.mjpeg";
	
	//Find out whether the camera needs to be calibrared.
	cout<<"Enter: '1' to enter a calibration matrix '2' to random generate a calibration matrix or '3' to use last good calibration matrix or '4' to calibrate the camera"<<endl;
	int calib_yn;
	cin>>calib_yn;
	
	if (calib_yn == 1)
	{
		//Get the calibration matrix from the user
		double a,b,c,d,e,f,g,h,i;
		cout<<"Enter the calibration matrix: "<<endl;
		cin>>a>>b>>c>>d>>e>>f>>g>>h>>i;
		KMatrix = (Mat_<double>(3,3) << a,b,c,d,e,f,g,h,i);
	}

	else if (calib_yn == 2)
	{
		//Randomly generate the calibration matrix
		cout<<"Please enter the size of the stream , width and then length:640 x 480 "<<endl;
		double frame_width,frame_length;
		cin>>frame_width>>frame_length;
		double max_w_h = MAX(frame_length,frame_width);
		KMatrix = (cv::Mat_<double>(3,3) <<	max_w_h ,0	, frame_width/2.0,0,			max_w_h,	frame_length/2.0,0,			0,			1);
		distortion_coeff = cv::Mat_<double>::zeros(1,4);
	}

	else if (calib_yn == 3)
	{
		cout<<"test"<<endl;
		KMatrix = (Mat_<double>(3,3) << 1598.137105349493, 0, 930.2103833838157, 0, 1594.705715001694, 540.8798963534, 0, 0, 1);
		distortion_coeff = (Mat_<double>(1,5)<<0.1166458260081404, 0.2553138309028938, -0.001613174921803301,-0.001156601595796987, -1.812315167189046);
		
	}

	else if (calib_yn == 4)
	{
		
		boost::tie(KMatrix,distortion_coeff)=Stream_Calibrate(calib_code);
		cout<<"KMatrix "<<KMatrix<<endl;
		cout<<"distortion_coeff "<<distortion_coeff<<endl;
	}


	//Create a thread group for capturing and processing
	boost::thread_group tgroup;

	
	tgroup.create_thread(boost::bind(&Stream,stream_code));

	tgroup.create_thread(boost::bind(&Stream_Process,KMatrix,distortion_coeff));
	//boost::this_thread::sleep(boost::posix_time::milliseconds(30000000));

	

	//Join threads to main so to wait for all to finish
	tgroup.join_all(); 

	return 0;	
}