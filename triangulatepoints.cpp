//Progressive Structure from Motion
//************************************************************************************************
//Matthew Westaway
//************************************************************************************************
//Triangulate points class
//************************************************************************************************






//#####################    HEADERFILE INCLUDES     #######################

#pragma once
#include "triangulatepoints.h"


//#####################    CONSTRUCTOR     #######################

Triangulatepoints::Triangulatepoints(Matx34d Cameramatrix1, Matx34d Cameramatrix2)
{
	//Initialise the private variables
	P =  Cameramatrix1;
	P1 = Cameramatrix2;
}

//#####################    FUNCTION: LINEAR LEAST SQUARES TRIANGULATION    #######################

Mat_<double> Triangulatepoints::Linearleastsquares_triangulation(Point3d u,Point3d u1)
{

	//We have two equations u = P*U and u1 = p2*U. We want to solve for U. We solve equations AU=B
	
	//BUILD A MATRIX
	Matx43d A(u.x*P(2,0)-P(0,0),u.x*P(2,1)-P(0,1),u.x*P(2,2)-P(0,2),
		u.y*P(2,0)-P(1,0),u.y*P(2,1)-P(1,1),u.y*P(2,2)-P(1,2),
		u1.x*P1(2,0)-P1(0,0),u1.x*P1(2,1)-P1(0,1),u1.x*P1(2,2)-P1(0,2),
		u1.y*P1(2,0)-P1(1,0),u1.y*P1(2,1)-P1(1,1),u1.y*P1(2,2)-P1(1,2));
	

	//BUILD B MATRIX
	Matx41d B(-(u.x*P(2,3)-P(0,3)),
		-(u.y*P(2,3)-P(1,3)),
		-(u1.x*P1(2,3)-P1(0,3)),
		-(u1.y*P1(2,3)-P1(1,3)));
	
	Mat_<double> X;
	solve(A,B,X,DECOMP_SVD);
	
	//CONVERT X COORDINATE INTO HOMOGENEOUS X COORDINATE
	Mat X_ = (Mat_<double>(4,1)<<X(0),X(1),X(2),1);
	
	return X_;
}

