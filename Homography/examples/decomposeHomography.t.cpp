//==============================================================================
// decomposeHomography.t.cpp - Example program for homography decomposition
// Project       : Computer Vision Utilities (cvutils)
// Author        : Vilas Kumar Chitrakaran (cvilas@ces.clemson.edu)
//==============================================================================

#include "Homography.hpp"
#include "Transform.hpp"
#include <iostream>

using namespace std;

Matrix<3,3> computeRotationMatrix(double r, double p, double y);
 // compute rotation matrix from euler angles

//======================================================================== 
// main - demonstrates homography decomposition
//========================================================================  
int main()
{
 Matrix<3,3> H, R;
 Vector<3> t, n;
 double d;
 motion_params_t motion[2];
 
 // initialize a homography matrix
 R = computeRotationMatrix(M_PI/6.0, 0.0, -M_PI/3.0); // rotate 30*, 0*, -60*
 t = -0.1, 0, 0; // translate
 d = 1;     // distance to feature plane
 n = 1, 0, 0; // normal to feature plane
 H = R + (1.0/d) * t * transpose(n); // Euclidean homography
 H = (1.0/H(3,3)) * H;

 // decompose homography - two solutions
 decomposeHomography(H, motion[0], motion[1]);
 
 cout << "==== Actual motion parameters ==== " << endl;
 cout << "Rotation: " << endl << R << endl;
 cout << "Scaled Translation: " << (1.0/d) * transpose(t) << endl;
 cout << "Normal: " << transpose(n) << endl;
 
 for(int i = 0; i < 2; ++i) {
  cout << "========= SHASTRY'S SOLUTION " << i+1 << " =========" << endl;
  cout << "Rotation: " << endl << motion[i].rotation << endl;
  cout << "Scaled Translation: " << transpose(motion[i].scaledTranslation) << endl;
  cout << "Normal: " << transpose(motion[i].normal) << endl;
 }

 double tmp1, tmp2;
 tmp1 = dotProduct(motion[0].normal,n);
 tmp2 = dotProduct(motion[1].normal,n);
  
 if(tmp1 > tmp2)
  cout << "** Correct solution: 1 **" << endl << endl;
 else
  cout << "** Correct solution: 2 **" << endl << endl;
 
 // decompose homography - normal known
 decomposeHomography(H, n, motion[0]);
 cout << "========= FAUGERAS' SOLUTION with known normal =========" << endl;
 cout << "Rotation: " << endl << motion[0].rotation << endl;
 cout << "Scaled Translation: " << transpose(motion[0].scaledTranslation) << endl;
 cout << "Normal: " << transpose(motion[0].normal) << endl;
 
 return 0;
}


//======================================================================== 
// computeRotationMatrix
//========================================================================  
Matrix<3,3> computeRotationMatrix(double r, double p, double y)
{
 Matrix<3,3> R;
 Transform t;
 t = rpyRotation(r,p,y);
 t.getSubMatrix(1,1,R);
 return R;
}
