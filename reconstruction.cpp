//Progressive SFM - Matthew Westaway - 2015-04-01
//Reconstruction.cpp

//Headerfile includes
#pragma once
#include "reconstruction.h"

//Constructor
Reconstruction::Reconstruction(Mat frame1,Mat frame2)
{
	img1 = frame1;
	img2 = frame2;
}

//Function to get keypoints in left image by SURF detector
vector<KeyPoint> Reconstruction::Getkeypoints_left(String method) 
{

	vector<KeyPoint> keypoints; //Vector of keypoints data type includes 2d coords and scale and orietnation 
	if (method == "SurfFeatureDetector")
	{
		SurfFeatureDetector detector; 
		detector.detect(img1,keypoints);
	}
	else if (method == "FastFeatureDetector")
	{
		FastFeatureDetector ffd;
		ffd.detect(img1,keypoints);
	}
	


	return keypoints;
}

//Function to get keypoints in right image by rich features
vector<KeyPoint> Reconstruction::Getkeypoints_right(String method) 
{
	vector<KeyPoint> keypoints; //Vector of keypoints data type includes 2d coords and scale and orietnation 
	if (method == "SurfFeatureDetector")
	{
		SurfFeatureDetector detector; 
		detector.detect(img2,keypoints);
	}
	else if (method == "FastFeatureDetector")
	{
		FastFeatureDetector ffd;
		ffd.detect(img2,keypoints);
	}

	Mat keypoints_right_image;
	drawKeypoints(img2,keypoints,keypoints_right_image);
	imshow("Keypoints",keypoints_right_image);
	waitKey(30);
	return keypoints;
}



vector<DMatch> Reconstruction::Getmatches_opticalflow (vector<KeyPoint> keypoints1,vector<KeyPoint> keypoints2) 
{
	//Convert Keypoints in left image to points
	vector<Point2f> kpoints_left;

	for (int i = 0;i<keypoints1.size();i++)
	{
		kpoints_left.push_back(keypoints1[i].pt);
	}

	//Convert Keypoints in right image to points
	vector<Point2f> kpoints_right;
	for (int i = 0;i<keypoints2.size();i++)
	{
		kpoints_right.push_back(keypoints2[i].pt);
	}
	



	//Create points in the right image - this will be location containing the calculated new positions of left points in the second image
	vector<Point2f> opoint_right;

	//Convert image to grey scale
	Mat img1_grey,img2_grey;
	cvtColor(img1,img1_grey,CV_RGB2GRAY);
	cvtColor(img2,img2_grey,CV_RGB2GRAY);

	//Calculate the optical flow field - how each left point moves accross the two images
	vector<uchar> vstatus;
	vector<float> verror;
	
	calcOpticalFlowPyrLK(img1_grey,img2_grey,kpoints_left,opoint_right,vstatus,verror);

	//Filter out the points with high error
	vector<Point2f>opoint_right_good;
	vector<int> opoint_right_good_index;

	for (int i = 0;i<vstatus.size();i++)
	{
		if (vstatus[i] && verror [i]<12)
		{
			//Keep the originl index of the point
			opoint_right_good_index.push_back(i);

			//Keep the point itself
			opoint_right_good.push_back(opoint_right[i]);
		}
		else
		{
			vstatus[i]=0;//a bad flow
		}
	}

	//Look round each optical flow point in the right image for keypoints and see if can make a match
	Mat opoint_right_good_flat = Mat(opoint_right_good).reshape(1,opoint_right_good.size());

	Mat kpoints_right_flat = Mat(kpoints_right).reshape(1,kpoints_right.size());
	
	cout<<"test4"<<endl;
	BFMatcher matcher(CV_L2);
	vector<DMatch> matches;
	vector<vector<DMatch>> nearest_neighbours; //the matches
	matcher.radiusMatch(opoint_right_good_flat,kpoints_right_flat,nearest_neighbours,2.0f); //2.f only two matches per point

	//Check that the found neighbors are unique (through away neighbours that are too close together, as they may be confusing)
	set <int> found_kpoints_right; //for duplicate prevention
	
	for (int i = 0 ; i<nearest_neighbours.size();i++) //Go throuhg matches
	{
		DMatch _m;

		if (nearest_neighbours[i].size()==1)
		{
			//Only one neighbours
			_m = nearest_neighbours[i][0];
		}

		else if (nearest_neighbours[i].size()>1)
		{
			//2 neighbours - check how close they are
			double ratio = nearest_neighbours[i][0].distance/nearest_neighbours[i][1].distance;
			nearest_neighbours[i][1].distance;

			if (ratio<0.7) //not to close 
			{
				//take the first neighbours
				_m = nearest_neighbours[i][0];
			}
			else 
			{
				//neighbouring points are too close we cant tell which point is better - throw away the point
				continue; //did not pass ratio
			}

		}
		else 
		{
			continue ; //no neighbours
		}

		if (found_kpoints_right.find(_m.trainIdx) == found_kpoints_right.end())
		{
			//The found neighbour was not yet used - match with original indexing
			_m.queryIdx = opoint_right_good_index[_m.queryIdx];
			matches.push_back(_m);//add this match
			found_kpoints_right.insert(_m.trainIdx);
		}
	}

	//cout<<" pruned "<<matches.size()<<" out of " <<nearest_neighbours.size();

	//Draw matches
	Mat matches_image;	
	drawMatches(img1,keypoints1,img2,keypoints2,matches,matches_image);
	imshow("Matches",matches_image);
	waitKey(30);

	return matches;

}
vector<DMatch> Reconstruction::Getmatches_richfeatures(vector<KeyPoint> keypoints1,vector<KeyPoint> keypoints2) 
{
	//Get descriptors for keypointsvisual
	Mat descriptors1,descriptors2; //Descriptors
	SurfDescriptorExtractor extractor;
	extractor.compute(img1,keypoints1,descriptors1);
	extractor.compute(img2,keypoints2,descriptors2);

	//Match the descriptors
	vector<DMatch> matches; //Matches
	BFMatcher matcher;
	matcher.match(descriptors1,descriptors2,matches);
	

	return matches;
}


//Function to prune the matches between two images - get good matches only
vector<DMatch> Reconstruction::Prunematches(vector<KeyPoint> detected_keypoints1,vector<KeyPoint> detected_keypoints2,vector<DMatch> matches)
{
	//Need to eliminate keypoints based on the fundamental matrix
	vector<uchar> status(detected_keypoints1.size()); //unsigned char

	std::vector<KeyPoint> aligned_keypoints1, aligned_keypoints2;
	std::vector<KeyPoint> good_keypoints1,  good_keypoints2;

	//Alligned keypoints
	for (unsigned int i=0; i<matches.size(); i++) 
	{
		assert(matches[i].queryIdx<detected_keypoints1.size());
		aligned_keypoints1.push_back(detected_keypoints1[matches[i].queryIdx]);
		assert(matches[i].trainIdx < detected_keypoints2.size());
		aligned_keypoints2.push_back(detected_keypoints2[matches[i].trainIdx]);
	}

	//Convert alligned keypoints to points
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

	//Calculate the fundamental matrix using ransac
	Mat F = findFundamentalMat(imgpts1,imgpts2, FM_RANSAC, 0.006 * maxVal, 0.99, status);

	//status is an array of N elements - element is set to 0 for outliers and 1 for current points
	vector<DMatch> new_matches;
	
	//print out how many of the keypoints correspond to the fundamental matrix
	//cout<<"F Keeping" <<countNonZero(status)<< " / "<<status.size()<<endl;
	
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

	//cout << matches.size() << " matches before, " << new_matches.size() << " new matches after Fundamental Matrix\n";
	//keep only those points that survived the fundamental matrix
	//matches = new_matches;
	
	//Draw matches
	Mat matches_image;	
	drawMatches(img1,detected_keypoints1,img2,detected_keypoints2,matches,matches_image);
	imshow("Matches",matches_image);
	waitKey(30);
	return new_matches;

}

//Function to get the fundamental matrix
Mat_<double> Reconstruction::Getfundamentalmatrix(vector<KeyPoint> keypoints1,vector<KeyPoint> keypoints2,vector<DMatch> matches)
{
	vector<Point2f> imgpts1, imgpts2;

	for (int i = 0 ;i<matches.size();i++)
	{
		imgpts1.push_back(keypoints1[matches[i].queryIdx].pt);
		imgpts2.push_back(keypoints2[matches[i].trainIdx].pt);
	}

	double minVal,maxVal;
	cv::minMaxIdx(imgpts1,&minVal,&maxVal);

	//Calculate the fundamental matrix using ransac
	Mat F = findFundamentalMat(imgpts1,imgpts2, FM_RANSAC, 0.006 * maxVal, 0.99);
	return F;
}

//Function to get essential matrix
Mat_<double> Reconstruction::Getessentialmatrix(Mat K,Mat F)
{
	Mat E = (K.t())*F*K; //accordig to HZ equation

	return E;
}

//Function to get camera matrix for right image
Matx34d Reconstruction::Getcameramatrix_right(Mat E, Mat KMatrix,vector<KeyPoint> keypoints1,vector<KeyPoint> keypoints2,vector<DMatch> matches)
{

	SVD svd(E); 
	Mat W = (Mat_<double>(3,3)<<0,-1,0,1,0,0,0,0,1);
	Mat u = svd.u;
	Mat vt = svd.vt;
	Mat_<double> R;
	Mat_<double> t;
	Matx34d P;
	vector<vector<Mat>> cameramatrices;
	vector<Mat> cameramatrix;

	//Add first option
	cameramatrix.push_back(u*W*vt);
	cameramatrix.push_back(u.col(2));
	cameramatrices.push_back(cameramatrix);
	cameramatrix.clear();

	//Add second option
	cameramatrix.push_back(u*W*vt);
	cameramatrix.push_back(-u.col(2));
	cameramatrices.push_back(cameramatrix);
	cameramatrix.clear();

	//Add third option
	cameramatrix.push_back(u*(W.t())*vt);
	cameramatrix.push_back(u.col(2));
	cameramatrices.push_back(cameramatrix);
	cameramatrix.clear();

	//Add fourth option
	cameramatrix.push_back(u*(W.t())*vt);
	cameramatrix.push_back(-u.col(2));
	cameramatrices.push_back(cameramatrix);
	cameramatrix.clear();
	
	int count;

	for (int a = 0;a<4;a++)
	{ 
		count = 0;
		//cout<<"a/4  "<<a<<endl;
		R = cameramatrices[a][0];
		t = cameramatrices[a][1]; 
			
		for (int i = 0 ;i<matches.size();i++)
		{
			//Create homogeenous keypoint
			Point2f t1 = keypoints1[matches[i].queryIdx].pt;
			Point3d test1_u(t1.x,t1.y,1.0);
					
			Point2f t2 = keypoints2[matches[i].trainIdx].pt;
			Point3d test2_u(t2.x,t2.y,1.0);

			//Normalize homogeenous keypoint
			Mat_<double> p1 = KMatrix.inv()*Mat_<double>(test1_u);
			Mat_<double> p2 = KMatrix.inv()*Mat_<double>(test2_u);

			Mat A = (Mat_<double>(1,3)<< p2(0),p2(1),p2(2));
			double Top = (R.row(0)-p2(0)*R.row(2)).dot(t.t());
			double Bottom = (R.row(0)-p2(0)*R.row(2)).dot(A);
			double z = Top/Bottom;
	
			Mat_<double> X_1 = (Mat_<double>(3,1)<< p1(0)*z,p1(1)*z,z);

			Mat_<double> X_2 = (R.t())*(X_1-t);

			if (X_1.at<double>(2)<0 || X_2.at<double>(2)<0)
			{
				count++;
			}
		}

		
		
		if (count<0.25*matches.size())
		{	

			break;
		}

	}

	if (count<0.25*matches.size())
	{
		P = Matx34d(R(0,0),R(0,1),R(0,2),t(0),R(1,0),R(1,1),R(1,2),t(1),R(2,0),R(2,1),R(2,2),t(2));
		cout<<"Correct baseline camera matrix found. The percentage of points reconstructed behind the camera is"<<100*count/matches.size()<<endl;
		return P;
	}

	else if (count>0.25*matches.size())
	{
		cout<<"ERROR-more than 25 percent many points behind the camera"<<endl;
		P = Matx34d(1,0,0,0,0,1,0,0,0,0,1,0);		
		return P;

	}
	
	else if (fabsf(determinant(R))-1.0>1e-07)
	{
		cout<<"ERROR-determinant rotation matrix not equal to 1"<<endl;
		//cerr<<"det(R) != +-1,0, this is not a rotation matrix"<<endl;
		P = Matx34d(1,0,0,0,0,1,0,0,0,0,1,0);
		return P; //if not return old camera matrix
	}
	
}

//Function to triangulate a point by linear least squares
Mat_<double> Reconstruction::Triangulatepoint_linearleastsquares(Point3d u,Point3d u1,Matx34d P,Matx34d P1)
{

	//We have two equations u = P*U and u1 = p2*U. We want to solve for U. We solve equations AU=B
	
	//Construct the A Matrix
	Matx43d A(u.x*P(2,0)-P(0,0),	u.x*P(2,1)-P(0,1),		u.x*P(2,2)-P(0,2),		
			  u.y*P(2,0)-P(1,0),	u.y*P(2,1)-P(1,1),		u.y*P(2,2)-P(1,2),		
			  u1.x*P1(2,0)-P1(0,0), u1.x*P1(2,1)-P1(0,1),	u1.x*P1(2,2)-P1(0,2),	
			  u1.y*P1(2,0)-P1(1,0), u1.y*P1(2,1)-P1(1,1),	u1.y*P1(2,2)-P1(1,2)
			  );
	Matx41d B(-(u.x*P(2,3)	-P(0,3)),
			  -(u.y*P(2,3)	-P(1,3)),
			  -(u1.x*P1(2,3)	-P1(0,3)),
			  -(u1.y*P1(2,3)	-P1(1,3)));
	
	Mat_<double> X;
	solve(A,B,X,DECOMP_SVD);
	return X;
}

//Function to triangulate a point by iterative least squares
Mat_<double> Reconstruction::Triangulatepoint_iterativeleastsquares(Point3d u,Point3d u1,Matx34d P,Matx34d P1)
{
	double wi = 1, wi1 = 1;
	Mat_<double> X(4,1);
	Mat_<double> X_ = Triangulatepoint_linearleastsquares(u,u,P,P1);
	X(0) = X_(0); X(1) = X_(1); X(2) = X_(2); X(3) = 1.0;
	for (int i=0; i<10; i++) { //Hartley suggests 10 iterations at most
		//recalculate weights
		double p2x = Mat_<double>(Mat_<double>(P).row(2)*X)(0);
		double p2x1 = Mat_<double>(Mat_<double>(P1).row(2)*X)(0);
		#define EPSILON 0.0001
		//breaking point
		if(fabsf(wi - p2x) <= EPSILON && fabsf(wi1 - p2x1) <= EPSILON) break;
		
		wi = p2x;
		wi1 = p2x1;
		
		//reweight equations and solve
		Matx43d A((u.x*P(2,0)-P(0,0))/wi,		(u.x*P(2,1)-P(0,1))/wi,			(u.x*P(2,2)-P(0,2))/wi,		
				  (u.y*P(2,0)-P(1,0))/wi,		(u.y*P(2,1)-P(1,1))/wi,			(u.y*P(2,2)-P(1,2))/wi,		
				  (u1.x*P1(2,0)-P1(0,0))/wi1,	(u1.x*P1(2,1)-P1(0,1))/wi1,		(u1.x*P1(2,2)-P1(0,2))/wi1,	
				  (u1.y*P1(2,0)-P1(1,0))/wi1,	(u1.y*P1(2,1)-P1(1,1))/wi1,		(u1.y*P1(2,2)-P1(1,2))/wi1
				  );
		Mat_<double> B = (Mat_<double>(4,1) <<	  -(u.x*P(2,3)	-P(0,3))/wi,
												  -(u.y*P(2,3)	-P(1,3))/wi,
												  -(u1.x*P1(2,3)	-P1(0,3))/wi1,
												  -(u1.y*P1(2,3)	-P1(1,3))/wi1
						  );
		
		solve(A,B,X_,DECOMP_SVD);
		X(0) = X_(0); X(1) = X_(1); X(2) = X_(2); X(3) = 1.0;
	}
	return X;
}