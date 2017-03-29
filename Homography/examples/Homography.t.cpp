//==============================================================================
// Homography.t.cpp - Example program for homography and decomposition
// Project       : Computer Vision Utilities (cvutils)
// Author        : Vilas Kumar Chitrakaran (cvilas@ces.clemson.edu)
//==============================================================================

#include "Homography.hpp"
#include "Transform.hpp"
#include <iostream>

using namespace std;

//==============================================================================
// main
//==============================================================================
int main()
{
 Matrix<3, 8> p2;            // pixel coordinates from second frame
 Matrix<3, 8> p1;            // pixel coordinates from first frame
 motion_params_t motion[2];  // recovered motion parameters
 Matrix<3, 3> Hn, Gn;        // Euclidean and projective Homography
 Matrix<3, 3> A;             // Camera intrinsic calibration matrix
 Vector<8> scale;            // scale factor
 
 // initialize for virtual parallax method
 ProjectiveHomography homography(8, e_vp);

 // initialize camera calibration matrix
 A = 2400, 0, 360, 0, 2400, 240, 0, 0, 1;
 
 // an example set of pixel coords for 8 non-coplanar features
 p1 = 310, 316, 577, 573, 359, 365, 550, 547,
      45, 288, 267, 58, 44, 287, 271, 53,
      1, 1, 1, 1, 1, 1, 1, 1;
 
 p2 = 315.011, 334.141, 590.244, 575.653, 363.81, 382.601, 564.053, 549.991,
     59.5576, 302.665, 264.716, 58.2838, 56.1194, 298.789, 271.933, 54.8274,
     1, 1, 1, 1, 1, 1, 1, 1;
 
 Matrix<3,3> tmp;
 
 // compute projective homography
 if(homography.compute(p2, p1, Gn, scale, tmp) == -1) {
  cerr << "Homography determination failed." << endl;
  return -1;
 }
 
 // get Euclidean Homography
 Hn = inverse(A) * Gn * A;
 cout << "Euclidean homography: " << endl << Hn << endl;
  
 // decompose homography and recover motion parameters
 if(decomposeHomography(Hn, motion[0], motion[1]) == -1) {
  cerr << "Homography Decomposition failed." << endl;
  return -1;
 }

 cout << "========= Recovered motion =========" << endl;
 for(int i = 0; i < 2; ++i) {
  cout << "========= Solution " << i << " ========" << endl;
  cout << "Rotation: " << endl << motion[i].rotation << endl;
  cout << "Scaled Translation: " << transpose(motion[i].scaledTranslation) << endl;
  cout << "normal: " << transpose(motion[i].normal) << endl;
 }
 return 0;
}


