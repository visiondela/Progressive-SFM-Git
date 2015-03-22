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

boost::tuple<vector<KeyPoint>,vector<KeyPoint>> Cameramotion::Getpointmatches_richfeatures() //return the rich features
{
	//DETECT KEYPOINTS

	SurfFeatureDetector detector; 
	detector.detect(img1,keypoints1);
	detector.detect(img2,keypoints2);

	//DRAW KEYPOINTS

	Mat img_keypoints_1; Mat img_keypoints_2;
	drawKeypoints( img1, keypoints1, img_keypoints_1, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
	drawKeypoints( img2, keypoints2, img_keypoints_2, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
	
	imshow("Stream", img_keypoints_1 );
	waitKey(400);

	imshow("Stream", img_keypoints_2 );
	waitKey(400);

	//GET DESCRIPTORS FOR KEYPOINTS

	SurfDescriptorExtractor extractor;
	extractor.compute(img1,keypoints1,descriptors1);
	extractor.compute(img2,keypoints2,descriptors2);

	//MATCH THE DESCRIPTORS

	BFMatcher matcher;
	matcher.match(descriptors1,descriptors2,matches);
	  
	//DRAW THE MATCHES
	drawMatches(img1,keypoints1,img2,keypoints2,matches,img_matches);

	////Show matches
	imshow("Stream", img_matches);
	waitKey(400);

	return boost::make_tuple(keypoints1,keypoints2);
}

//#####################      FUNCTION: GET FUNDAMENTAL MATRIX      ######################

Mat Cameramotion::Get_fundamentalmatrix()
{
	//POPULATE THE VECTORS OF KEYPOINTS FOR EACH IMAGE
	for (unsigned int i = 0 ; i<matches.size(); i++) //unsigned means cant be negative only positive
	{
		imgpts1.push_back(keypoints1[matches[i].queryIdx].pt);//queryIdx is the "left image" trainIdx is the right image
		imgpts2.push_back(keypoints2[matches[i].trainIdx].pt);// .pt gets the point from the keypoint structure
	}
	
	//FIND THE FUNDAMENTAL MATRIX
	F = findFundamentalMat(imgpts1,imgpts2,FM_RANSAC,0.1,0.99);

	return F;
}

//#####################      FUNCTION: GET ESSENTIAL MATRIX      ######################

Mat Cameramotion::Get_essentialMatrix(Mat K,Mat F)
{
	E = K.t()*F*K; //accordig to HZ equation

	return E;
}

//#####################      FUNCTION: GET CAMERAL MATRIX      ######################

Matx34d Cameramotion::Get_cameraMatrix(Mat E)
{
	//SINGULAR VALUE DECOMPOSE THE ESSENTIAL MATRIX
	SVD svd(E,SVD::MODIFY_A); 
	svd_u = svd.u;
	svd_vt = svd.vt;
	svd_w = svd.w;

	//GET THE ROTATION AND TRANSLATION MATRIX
	Matx33d W(0,-1,0,1,0,0,0,0,1);
	R = svd_u*Mat(W)*svd_vt; //the Mat_ allows us to specify the date type as double
	t = svd_u.col(2);

	//BUILD THE CAMERA MATRIX
	P1 = Matx34d(R(0,0),R(0,1),R(0,2),t(0),R(1,0),R(1,1),R(1,2),t(1),R(2,0),R(2,1),R(2,2),t(2)); //There are 4 possible camera matrices

	//Check to see if our camera matrices are fault free- if have an error we know our fundamental matrices - aquired from point matching may be erroneous
	//We check by seeing if the rotation element is a valid rotation matricx - must have determinant of (1)or (-1)

	//CHECK TO SEE IF THE CAMERA MATRIX IS FAULT FREE

	if (fabsf(determinant(R))-1.0>1e-07)
	{
		cerr<<"det(R) != +-1,0, this is not a rotation matrix"<<endl;
		return 0;
	}
	
	else
	{
		return P1;
	}
	
}