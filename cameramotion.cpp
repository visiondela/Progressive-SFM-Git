//Progressive Structure from Motion
//************************************************************************************************
//Matthew Westaway
//************************************************************************************************
//Camera motion class
//************************************************************************************************



		
		

	




//#####################      HEADER FILE INCLUDES     #######################
#pragma once
#include "cameramotion.h"

	

//#####################      CONSTRUCTOR!      #######################

Cameramotion::Cameramotion(Mat frame1,Mat frame2)
{
	img1 = frame1;
	img2 = frame2;
}

//#####################      FUNCTION: GET POINT MATCHES BY RICH FEATURES      #######################

boost::tuple<vector<KeyPoint>,vector<KeyPoint>,vector<DMatch> > Cameramotion::Getpointmatches_richfeatures() //return the rich features
{
	//DETECT KEYPOINTS
	vector<KeyPoint> keypoints1, keypoints2; //Vector of keypoints data type includes 2d coords and scale and orietnation 
	SurfFeatureDetector detector; 
	detector.detect(img1,keypoints1);
	detector.detect(img2,keypoints2);

	//DRAW KEYPOINTS

	//Mat img_keypoints_1; Mat img_keypoints_2;
	//drawKeypoints( img1, keypoints1, img_keypoints_1, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
	//drawKeypoints( img2, keypoints2, img_keypoints_2, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
	//
	////imshow("Stream", img_keypoints_1 );
	////waitKey(2000);

	////imshow("Stream", img_keypoints_2 );
	////waitKey(2000);

	//GET DESCRIPTORS FOR KEYPOINTS
	Mat descriptors1,descriptors2; //Descriptors
	SurfDescriptorExtractor extractor;
	extractor.compute(img1,keypoints1,descriptors1);
	extractor.compute(img2,keypoints2,descriptors2);

	//MATCH THE DESCRIPTORS
	vector<DMatch> matches; //Matches
	BFMatcher matcher;
	matcher.match(descriptors1,descriptors2,matches);
	  
	//DRAW THE MATCHES
	Mat img_matches; //Img_matches
	drawMatches(img1,keypoints1,img2,keypoints2,matches,img_matches);

	////Show matches
	/*imshow("Stream", img_matches);
	waitKey(400);*/

	return boost::make_tuple(keypoints1,keypoints2,matches);
}

//#####################      FUNCTION: GET FUNDAMENTAL MATRIX      ######################

boost::tuple<vector<KeyPoint>,vector<KeyPoint>,Mat,vector<DMatch>  > Cameramotion::Get_fundamentalmatrix(vector<KeyPoint> detected_keypoints1,vector<KeyPoint> detected_keypoints2,vector<DMatch> matches)
{
	
	//Need to eliminate keypoints based on the fundamental matrix
	vector<uchar> status(detected_keypoints1.size()); //unsigned char

	std::vector<KeyPoint> aligned_keypoints1, aligned_keypoints2;
	std::vector<KeyPoint> good_keypoints1,  good_keypoints2;

	Mat F;

	//Alligned keypoints
	for (unsigned int i=0; i<matches.size(); i++) 
	{
		assert(matches[i].queryIdx<detected_keypoints1.size());
		aligned_keypoints1.push_back(detected_keypoints1[matches[i].queryIdx]);
		assert(matches[i].trainIdx < detected_keypoints2.size());
		aligned_keypoints2.push_back(detected_keypoints2[matches[i].trainIdx]);
	}

	//Convert alligned keypoint to points
	vector<Point2f> imgpts1, imgpts2;
	
	for (int i = 0 ; i<aligned_keypoints1.size();i++)
	{
		imgpts1.push_back(aligned_keypoints1[i].pt);
	}

	for (int i = 0 ; i<aligned_keypoints2.size();i++)
	{
		imgpts2.push_back(aligned_keypoints2[i].pt);
	}
	//Find out the minimum distance points can be from epipolar line before they are marked as outliers and not included in calculating fundamental matrix

	double minVal,maxVal;
	cv::minMaxIdx(imgpts1,&minVal,&maxVal);
	//calculate the fundamental matrix using ransac
	F = findFundamentalMat(imgpts1,imgpts2, FM_RANSAC, 0.006 * maxVal, 0.99, status);

	//Create a structure that has the aligned keypoints and the index of their corresponding image point in frame 2

	struct CloudPoint
	{
		Point3d pt;
		vector<int>index_of_2d_origin;
	}


	//status is an array of N elements - element is set to 0 for outliers and 1 for current points
	vector<DMatch> new_matches;
	//print out how many of the keypoints correspond to the fundamental matrix
	cout<<"F Keeping" <<countNonZero(status)<< " / "<<status.size()<<endl;
	//get the keypoints which correspond to the fundamental matrix
	for (int i = 0; i<status.size(); i ++)
	{
		if (status[i]) //if the point is not an outlier - ie 1 which is true
		{
			good_keypoints1.push_back(aligned_keypoints1[i]);
			good_keypoints2.push_back(aligned_keypoints2[i]);
			
			if (matches.size() <= 0)  //Points are already aligned
			{ 
				new_matches.push_back(DMatch(matches[i].queryIdx,matches[i].trainIdx,matches[i].distance));
			} 
			else 
			{
				new_matches.push_back(matches[i]);
			}

		}
	}

	cout << matches.size() << " matches before, " << new_matches.size() << " new matches after Fundamental Matrix\n";
	//keep only those points that survived the fundamental matrix
	

	

	return boost::make_tuple(good_keypoints1,good_keypoints2,F,new_matches);

}

//#####################      FUNCTION: GET ESSENTIAL MATRIX      ######################

Mat Cameramotion::Get_essentialMatrix(Mat K,Mat F)
{
	Mat E = K.t()*F*K; //accordig to HZ equation

	return E;
}

//#####################      FUNCTION: GET CAMERAL MATRIX      ######################

Matx34d Cameramotion::Get_cameraMatrix(Mat E)
{
	//SINGULAR VALUE DECOMPOSE THE ESSENTIAL MATRIX
	SVD svd(E,SVD::MODIFY_A); 


	//GET THE ROTATION AND TRANSLATION MATRIX
	Matx33d W(0,-1,0,1,0,0,0,0,1);
		 
		
	//camera matrix 2
	Matx34d P; //camera matrix 1
	Mat_<double> R = svd.u*Mat(W)*svd.vt; //the Mat_ allows us to specify the date type as double
	Mat_<double> t = svd.u.col(2);

	//BUILD THE CAMERA MATRIX
	Matx34d P1(R(0,0),R(0,1),R(0,2),t(0),R(1,0),R(1,1),R(1,2),t(1),R(2,0),R(2,1),R(2,2),t(2)); //There are 4 possible camera matrices

	//Check to see if our camera matrices are fault free- if have an error we know our fundamental matrices - aquired from point matching may be erroneous
	//We check by seeing if the rotation element is a valid rotation matricx - must have determinant of (1)or (-1)

	//CHECK TO SEE IF THE CAMERA MATRIX IS FAULT FREE

	if (fabsf(determinant(R))-1.0>1e-07)
	{
		cerr<<"det(R) != +-1,0, this is not a rotation matrix"<<endl;
		Matx34d P(1,0,0,0,0,1,0,0,0,0,1,0);
		return P; //if not return old camera matrix
	}
	
	else
	{
		return P1;
	}
	
}