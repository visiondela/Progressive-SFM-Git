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
#include <boost\date_time\posix_time\posix_time.hpp>

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
#include "tbb\tbb.h"

//STD INCLUDES
#include <iostream>
#include <time.h> //For use of computer time
#include <stdio.h>
#include <boost/fusion/container/vector.hpp>
#include <boost/fusion/include/vector.hpp>
//###################		HEADER FILE INCLUDES!         ###################

#include "reconstruction.h"				
#include "calibration.h"




using namespace cv;
using namespace std;

	
pcl::visualization::CloudViewer viewer("Progressive SFM");

int frames_reconstructed = 100000000;


Size feed;
Size display;

struct CloudPoint
{
	Point3d pt;
	vector<int> index_of_2d_origin; //1000frames
};




boost::tuple<Mat_<double>,Mat_<double> > Stream_Calibrate (int stream_code)
{
	
	Mat frame;

	VideoCapture cap(stream_code);
	cap.set(CV_CAP_PROP_FRAME_WIDTH,feed.width);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT,feed.height);

	if (!cap.isOpened())
	{
		cout << "The camera is not connected. Connect the camera and then proceed" << endl;
		Stream_Calibrate(stream_code); //Rerun function
	}

	//Define variables for calibration
	
	vector<vector<Point2f>> imagepoints; //vector for image points in all images - 54 image points per image
	vector<vector<Point3f>> arrayObjectPoints; //vector for object points in all images - 54 object points per image	
	vector<Point3f> objectpoints; //Vector for 54 object points, each object point has xyz. same for all images
	Size patternsize(9,6); //Checkerboard pattern

	
	

	cout<<"Would you like a checkerboard for calibration? Y o N"<<endl;
	string calib_check;
	cin>>calib_check;

	if (calib_check == "Y")
	{
		Mat pattern = imread("C:/Users/Matthew/Documents/Visual Studio 2010/Projects/Program 2  - Progressive Structure from Motion/ProgressiveSFM/checkerboard.png"); //Get checkerboard pattern
		if (!pattern.data)
		{
			cout << "The image file could not be opened. Make sure the file is in the entered address" << endl;
			Stream_Calibrate(stream_code);
		}

		cv::namedWindow("Calibrate Pattern",WINDOW_NORMAL);
		Size checker = pattern.size();
		cv::resizeWindow("Calibrate Pattern",checker.width/3,checker.height/3);
		imshow("Calibrate Pattern",  pattern); 
		waitKey(30);
	}


	cv::namedWindow("Stream_Calibrate",WINDOW_NORMAL);

	cv::resizeWindow("Stream_Calibrate",display.width,display.height);

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
		//Mat frame;
		bool bSuccess = cap.read(frame); 
		if (!bSuccess) //if not true
		{
			cout << " Cannot read a frame from the stream . Check to see if the stream is connected and restart program" << endl;

		}
		
		//Show the frame
		imshow("Stream_Calibrate",  frame); 
		waitKey(30);
		

		Mat frameGray; //Grey frame
		cout<<frame_no<<endl;
		//If its the 100th frame capture the frame
		if (frame_no%50==0)
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
	Calibration a(imagepoints,arrayObjectPoints,feed.width,feed.height) ;
	Mat_<double> KMatrix = a.get_kmatrix(); //calibration matrix depends on the resolution of images- ie focal length, the radial distortion parameters are independent of the resolution
	Mat_<double> distortion_coeff = a.get_disortioncoefficients();
	destroyWindow("Stream_Calibrate");
	destroyWindow("Calibrate Pattern");
	return boost::make_tuple(KMatrix ,distortion_coeff);
}					

void Images_fromfile(tbb::concurrent_vector<cv::Mat> &frames)
{
	
	string Filename = "C:/Users/Matthew/Documents/Visual Studio 2010/Projects/Program 2  - Progressive Structure from Motion/ProgressiveSFM/Images_fromfile/Image_001.png";
	
	Mat frame = cv::imread(Filename.c_str());

	if (!frame.data)
	{
		cout << "The first image file could not be opened. Enter correct address and restart" << endl;
		string path;
		cin>>path;
		//Images_fromfile(frames);
	}

	Size fileimage = frame.size();

	cv::namedWindow("Fileviewer",WINDOW_NORMAL );
	cv::resizeWindow("Fileviewer",fileimage.width,fileimage.height);

	//Initialize some variables
	int frame_no=1;

	while(1)
	{
		frame = cv::imread(Filename.c_str());
		

		if (!frame.data)
		{
			break;
		}

		imshow("Fileviewer",  frame); 
		waitKey(30);
		
		frames.push_back(frame);
	
		frame_no++;
		string frame_Number = boost::lexical_cast<string,int>(frame_no);
		Filename = "C:/Users/Matthew/Documents/Visual Studio 2010/Projects/Program 2  - Progressive Structure from Motion/ProgressiveSFM/Images_fromfile/Image_00"+frame_Number+".png";
	}
	frames_reconstructed = frame_no;
}

void Stream (int stream_code,tbb::concurrent_vector<cv::Mat> &frames)
{
	//Create insance of capture class and check to see if camera is connected
	VideoCapture cap(stream_code);
	cap.set(CV_CAP_PROP_FRAME_WIDTH,feed.width);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT,feed.height);

	if (!cap.isOpened())
	{
		cout << "The camera is not connected. Connect the camera and then proceed" << endl;
		void Stream(int stream_code); //Rerun function
	}

	
	
	//Set up variables to save the frames to file
	vector<int> compression_params; //vector that stores the compression parameters of the image
    compression_params.push_back(CV_IMWRITE_JPEG_QUALITY); //specify the compression technique
    compression_params.push_back(98); //specify the compression quality



	//Create window to show the stream
	cv::namedWindow("Stream",WINDOW_NORMAL );
	cv::resizeWindow("Stream",display.width,display.height);

	
	//Initialize some variables
	int frame_no=1;
	int frames_captured = 0;

	//Run the camera infinite loop
	while(1)
	{
		cv::Mat frame;
		//Read a frame
		bool bSuccess = cap.read(frame); 
		if (!bSuccess) //If cant read frame
		{
			cout << " Cannot read a frame from the stream anymore. Capturing over - Processing will continue" << endl;
			frames_reconstructed = frame_no;
			break;
		}

		//Display the frame in window
		imshow("Stream",  frame);
		waitKey(30);

		//Print out the frame_number
		cout<<frame_no<<endl;
		if (frame_no>100 &frame_no%10==0)
		//if (frame_no>10 )
		{
			//Save frame to vector
			//tbb::concurrent_vector<cv::Mat>::iterator itr = frames.push_back(frame);
			frames.push_back(frame);
		
			cout<<"Captured frame number "<<frames_captured<<endl;
			
			string name = "C:/Users/Matthew/Documents/Visual Studio 2010/Projects/Program 2  - Progressive Structure from Motion/ProgressiveSFM/Captures/Image_00"+boost::lexical_cast<string,int>(frames_captured)+".jpg";
			bool bSuccess = imwrite(name, frame, compression_params); //write the image to file
	
			if ( !bSuccess )

			{

				 cout << "ERROR : Failed to save the image" << endl;

				 //system("pause"); //wait for a key press

			}
			frames_captured++;

		}
		
		//Increment the frame numbers
		frame_no++;	
		
		//Wait for Escape key to be pressed for 30 seconds
		if(waitKey(30) == 27) 
		{
                cout << "The escape key is pressed by user. Capturing over - Processing will continue" << endl; 
				frames_reconstructed = frames_captured;
                break;
		}

		
	}


}

void Stream_Process(Mat_<double> KMatrix,Mat_<double> distcoeff,tbb::concurrent_vector<cv::Mat> &frames,String point_matching)
{
	//Sleep the process thread untill the baseline frames have been captured
	boost::this_thread::sleep(boost::posix_time::milliseconds(10000));
	Mat frame;
	
	//Setup the point cloud for visualiation

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud; //Create the point cloud for visualization
	cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
	Vec3b rgbv(255,255,255);  //The black RGBV value for the point cloud
	uint32_t rgb = ((uint32_t)rgbv[2]<<16 | (uint32_t)rgbv[1] << 8 | (uint32_t)rgbv[0]);


	cv::namedWindow("Keypoints",WINDOW_NORMAL );
	cv::resizeWindow("Keypoints",display.width,display.height);
	cv::moveWindow("Keypoints",0,0);
	
	cv::namedWindow("Matches",WINDOW_NORMAL );
	cv::resizeWindow("Matches",2*display.width,display.height);
	cv::moveWindow("Matches",display.width,0);

	//Initialise that the state of the baseline reconstruction
	string baseline_state = "Baseline has not been reconstructed";
	
	
	int a = 1 ;//Initialise the counter for the number of frames captured
	
	//Set up variables to save the frames to file
	vector<int> compression_params; //vector that stores the compression parameters of the image
    compression_params.push_back(CV_IMWRITE_JPEG_QUALITY); //specify the compression technique
    compression_params.push_back(98); //specify the compression quality


	//Setup our global point cloud for processing
	vector <CloudPoint> pcloud; //Our global point cloud
	vector<int> pcloud_status(1000000,0); //too see if points been used before

	//Initialise our key variables
	vector<KeyPoint> keypoints_left; //Keypoints in left_frame
	vector<KeyPoint> keypoints_right;//Keypoints in right frame
	
	vector<DMatch> matches; //The matches
	
	Matx34d cameramatrix_left(1,0,0,0,0,1,0,0,0,0,1,0); //Camera matrix in left frame
	Matx34d cameramatrix_right; //camera matrix in right frame

	//Start processing loop
	while (a<frames_reconstructed)
	{

		vector<double> reproj_error; //the reprojection error
		
		try
		{
			//Get frame and undistort

			Mat frame_left_distorted= frames.at(a-1);
			Mat frame_right_distorted= frames.at(a);
		
			Mat frame_left;
			Mat frame_right;

			undistort(frame_right_distorted,frame_right,KMatrix,distcoeff);
			undistort(frame_left_distorted,frame_left,KMatrix,distcoeff);

	
			//Initialise 3D point vector of the existing 3D points for PNP
			vector<Point3f> ppcloud;

			//Initialise 2D point vector of points in new frame for PNP 
			vector<Point2f> imgpoints;

			//Call the constructor
			Reconstruction view(frame_left,frame_right); 

			//Detect keypoints and match

			if (point_matching == "Optical flow")
			{
				//Get keypoints
				keypoints_left = (baseline_state == "reconstructed") ? keypoints_right: view.Getkeypoints_left("FastFeatureDetector");
				
				keypoints_right = view.Getkeypoints_right("FastFeatureDetector");

				//Get matches
				matches =view.Getmatches_opticalflow(keypoints_left,keypoints_right);
		
			}
			
			else if (point_matching == "Rich features")
			{

				//Get keypoints
				keypoints_left = (baseline_state == "reconstructed") ? keypoints_right: view.Getkeypoints_left("SurfFeatureDetector");
				
				keypoints_right = view.Getkeypoints_right("SurfFeatureDetector");
				
				//Get matches
				matches =view.Getmatches_richfeatures(keypoints_left,keypoints_right);
					
				//Refine matches
				matches = view.Prunematches(keypoints_left,keypoints_right,matches);
			}

			//Get the fundamental matrix
			Mat fundamentalmatrix = view.Getfundamentalmatrix(keypoints_left,keypoints_right,matches);
				
			//Get the essential matrix
			Mat essentialmatrix = view.Getessentialmatrix(KMatrix,fundamentalmatrix);
			
			if (baseline_state != "reconstructed")
			{
				//Get the camera matrix from essential matrix - there are four possible options 
				cameramatrix_right = view.Getcameramatrix_right(essentialmatrix, KMatrix,keypoints_left,keypoints_right,matches);

				if (cameramatrix_right == Matx34d(1,0,0,0,0,1,0,0,0,0,1,0))
				{
					cout<<"The baseline frames are not good - the next pair of frames will be used as baseline frames"<<endl;
					a++;
				}
				
				else if (cameramatrix_right != Matx34d(1,0,0,0,0,1,0,0,0,0,1,0))
				{
			
					//Basline has now been reconsructed
					baseline_state = "reconstructed";
					a++;
					//Move onto the next frame 
				}

			}

			else if (baseline_state == "reconstructed")
			{
		
				//Get camera matrix and keypoints in new frame ( old right frame)
				cameramatrix_left = cameramatrix_right;
			
				//Populate imgpoints and ppcloud in order to do PNP
				for (int c = 0;c<matches.size();c++)
				{
					int index_in_old_view= matches[c].queryIdx;
					
					for (int d = 0 ; d<pcloud.size();d++)
					{
						//Check if any of the current 3D points were reconsructed with one of the matching points
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
				solvePnP(ppcloud,imgpoints,KMatrix,distcoeff,rvec,t,false);
				//solvePnPRansac(ppcloud,imgpoints,KMatrix,distcoeff,rvec,t,false,500,8,0.9*imgpoints.size(),noArray(),ITERATIVE);
				Rodrigues(rvec,R);

				cameramatrix_right = Matx34d(R(0,0),R(0,1),R(0,2),t(0),R(1,0),R(1,1),R(1,2),t(1),R(2,0),R(2,1),R(2,2),t(2));
				
				ppcloud.clear();
				imgpoints.clear();
				a++;
			}
			Mat_<double> KMatrixP = KMatrix * Mat(cameramatrix_right);
			//Go through each point
			for (unsigned int i=0;i<matches.size() ;i++)
			{	

				//Homogeneousize keypoint
				Point2f kp = keypoints_left[matches[i].queryIdx].pt;
				Point3d u(kp.x,kp.y,1.0);
					
				Point2f kp1 = keypoints_right[matches[i].trainIdx].pt;
				Point3d u1(kp1.x,kp1.y,1.0);

				//Normalize homogeenous keypoint
				Mat_<double> um =KMatrix.inv()*Mat_<double>(u);
				u.x = um(0);
				u.y = um(1);
				u.z = um(2);

				Mat_<double> um1 = KMatrix.inv()*Mat_<double>(u1);
				u1.x = um1(0);
				u1.y = um1(1);
				u1.z = um1(2);
					
				//Triangulate keypoint
				Mat_<double> X = view.Triangulatepoint_iterativeleastsquares(u,u1,cameramatrix_left,cameramatrix_right);
					
				//Calculate the reprojection error
				Mat_<double> xPt_img =  KMatrixP*X; //Get the second image coordinate from the triangulated 3D point
				Point2f xPt_img_(xPt_img(0)/xPt_img(2),xPt_img(1)/xPt_img(2)); 
	
					
				CloudPoint newpoint;
			
				//Add 3D point to our global point cloud
						
				reproj_error.push_back(norm(xPt_img_ - kp1));
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
				//cout<<"point "<<pclp<<endl;	
				//Add 3D point to point cloud
				cloud->push_back(pclp);	
				
			}

			//Calculate the mean reprojection error
			Scalar mean_projerror = mean(reproj_error);
			//Visualize the point cloud 
			cloud->width = (uint32_t) cloud->points.size(); //number of points
			cloud->height = 1; //a list of points, one row of data
			//viewer.addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud1", v1);
			viewer.showCloud(cloud);		
			cout<<"Capture number no...Reeconstructed with a mean reprojection error :" <<mean_projerror[0]<<"number of points is "<<cloud->points.size()<<endl;	

		
				
		}

		catch(...)
		{
		
		}

	}

	cout<<"Processing finished"<<endl;
	string x;
	cin>>x;
}


int main(int argc, char** argv)
{
	//Intrinisic parameter variables
	Mat_<double> KMatrix; 
	Mat_<double> distortion_coeff;
	//typename _Alloc = std::allocator<Mat>
	
	tbb::concurrent_vector<cv::Mat> frames;
	String point_matching ;


	cout<<"Enter 1 if you would like to use your video stream or 2 if you would like to use the test set of images"<<endl;
	int selection;
	cin>>selection;


	
	if (selection ==1)
	{
		//Get the address of the stream
		cout<<"Enter the stream address: "<<endl;
		int stream_code;
		cin>>stream_code;

		cout<<"What resolution would you like to capture in , width and then height eg 640 x 480 or 1920 x 1080 "<<endl;
		cin>>feed.width>>feed.height;
			
		display.width = feed.width;
		display.height = feed.height;
		
		cout<<"Would you like to use rich feature matching(1) or optical flow matching(2)?"<<endl;
		string u;
		cin>>u;
		point_matching = (u=="1")?"Rich features":"Optical flow";
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
			double max_w_h = MAX(feed.height,feed.width);
			KMatrix = (cv::Mat_<double>(3,3) <<	max_w_h ,0	, feed.width/2.0,0,			max_w_h,	feed.height/2.0,0,			0,		1);
			distortion_coeff = cv::Mat_<double>::zeros(1,4);
		}

		else if (calib_yn == 3)
		{
			KMatrix = (Mat_<double>(3,3) << 678.6992120541859, 0, 277.6049593770479,0, 675.1040994794219, 257.4813100669645, 0, 0, 1);
			distortion_coeff = (Mat_<double>(1,5)<<0.06027257772998455, 0.1295849070524481, -0.00038486251215363, 0.004457628673161475, -0.7370791624839784);
		}



		else if (calib_yn == 4)
		{
			boost::tie(KMatrix,distortion_coeff)=Stream_Calibrate(stream_code);
			cout<<"KMatrix "<<KMatrix<<endl;
			cout<<"distortion_coeff "<<distortion_coeff<<endl;
			string calib_good;
			cout<<"Are you happy with your calibration and would like to start 3D reconstruction - Y or N"<<endl;
			
			cin>>calib_good;
			if (calib_good != "Y")
			{
				boost::tie(KMatrix,distortion_coeff)=Stream_Calibrate(stream_code);
				cout<<"KMatrix "<<KMatrix<<endl;
				cout<<"distortion_coeff "<<distortion_coeff<<endl;
				cout<<"Push enter to begin 3D reconstruction"<<endl;
				waitKey(0);
			}

		}

		boost::thread t1(&Stream,stream_code, boost::ref(frames));
		boost::thread t2(&Stream_Process,KMatrix,distortion_coeff,boost::ref(frames),point_matching);
		t1.join();
		t2.join();
	}

	else if ( selection = 2)
	{
		KMatrix = (cv::Mat_<double>(3,3) <<	3072 ,0	, 3072/2.0,0,			3072,	2048/2.0,0,			0,			1);
		distortion_coeff = cv::Mat_<double>::zeros(1,4);
		boost::thread t1(&Images_fromfile,boost::ref(frames));
		boost::thread t2(&Stream_Process,KMatrix,distortion_coeff,boost::ref(frames),point_matching);
		t1.join();
		t2.join();
	}

	return 0;	
}