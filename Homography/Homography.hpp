//==============================================================================
// Homography.hpp - Methods to compute and decompose homography matrix
//                  from image features.
//
// Project       : Computer Vision Utilities (cvutils)
// Author        : Vilas Kumar Chitrakaran (cvilas@ces.clemson.edu)
// Version       : 0.1 (December 2003)
// Compatibility : POSIX, GCC
//==============================================================================

#ifndef INCLUDED_HOMOGRAPHY_HPP
#define INCLUDED_HOMOGRAPHY_HPP

#include "Vector.hpp"
#include "RowVector.hpp"
#include "GSLCompat.hpp"
#include <gsl/gsl_linalg.h>
#include <gsl/gsl_combination.h>
#include <gsl/gsl_cblas.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_eigen.h>
#include <math.h>


//==============================================================================
/*! \struct _motion_params 
    \brief  Motion parameters that define a homography matrix */
//==============================================================================
typedef struct _motion_params
{
 Matrix<3,3> rotation;         /*!< Rotation between camera frames. */
 Vector<3> scaledTranslation;  /*!< Translation between camera frames, divided  
                                    by the distance from plane of image features
                                    to the optical center of the first camera. */
 Vector<3> normal;             /*!< Unit normal to the plane of image features
                                    defined in first camera frame. */
}motion_params_t;


//==============================================================================
/*! \enum _homography_method
    \brief Method used to compute homography matrix */
//==============================================================================
typedef enum _homography_method
{
 e_ls = 1,  //!< least squares method (for coplanar features)
 e_kk,      //!< Kanatani optimal algorithm (for coplanar features)
 e_vp       //!< Virtual parallax method (Malis) (for non-coplanar features)
}homography_method_t;


int decomposeHomography(const Matrix<3,3> &Hen, motion_params_t &motion0, 
              motion_params_t &motion1);
 /*!< Decompose a <b>Euclidean</b> homography to obtain the motion parameters. Two 
      physically possible solutions are returned. If the unit normal vector 
      is known (see class description), the solution to decomposition is unique,
      in which case use the subsequent method with the same name to compute
      the motion parameters more accurately. 
      <hr><br>
      IMPORTANT NOTE: This method does not produce valid solutions when there is
      no motion, or when the motion between camera frames is a pure rotation.
      <br><hr>
      \param Hen        The normalized Euclidean homography [Hen = inverse(C) * 
                        Hpn * C, where C is the camera intrinsic calibration 
                        matrix] (input).
      \param motion0
      \param motion1    The two physically possible solutions (output).
      \return           0 on success, -1 on error.
 */
 
int decomposeHomography(const Matrix<3,3> &Hen, const Vector<3> &n, 
              motion_params_t &motion);
 /*!< If the unit normal vector is known, this method can be used to decompose 
      <b>Euclidean</b> homography more accurately than the previous method with  
      the same name.
      \param Hen     The normalized Euclidean homography [Hen = inverse(C) * 
                    Hpn * C, where C is the camera intrinsic calibration 
                    matrix] (input).
      \param n       The known unit normal vector (see class description) (input).
      \param motion  The motion parameters from homography decomposition (output)
      \return        0 on success, -1 on error.
 */ 

//==============================================================================
// class ProjectiveHomography
//------------------------------------------------------------------------------
// \brief
// Homography computation functions.
//
// Homography relates pixel coordinates of coplanar feature points
// when recorded by a camera from two different poses. If the 3D 
// homogeneous coordinates of a feature point on a plane are 
// q1 = [x1/z1, y1/z1, 1]' and q2 = [x2/z2, y2/z2, 1]' relative to the 
// camera frame at position 1 (I1) and 2 (I2), respectively, we get the 
// following euclidean relationship:
// \code
// q2 = (z1/z2) * He * q1, 
// \endcode
// where '(z1/z2)' is called the 'depth ratio', and 'He' is the 'Euclidean 
// homography' defined as:
// \code
// He = (R + (t/d) * transpose(n)).
// \endcode
// Here 'R' is the 3x3 rotation matrix that defines the transformation
// I1 -> I2, 't' is the translation between I1 and I2 defined in frame I2, 
// 'd' and 'n' are the distance, and the unit normal to the plane of feature 
// points, respectively, when camera is at I1. Defined in terms of pixel 
// coordinates p1 (= C * q1) and p2 (= C * q2), for a calibrated camera with 
// 'C' as the internal calibration matrix, we get the following relationship: 
// \code
// p2 = (z1/z2) * Hp * p1, 
// \endcode
// where Hp is called 'projective homography'. The projective and Euclidean 
// homographies are hence related as follows:
// \code
// Hp = C * He * inverse(C). 
// \endcode
// Pay attention to these definitions when you use functions provided in this
// class. This class primarily provides methods to compute ~projective~ homography 
// from image coordinates of planar feature points, and methods to decompose 
// ~Euclidean~ homography into motion paramters R, t/d and n. Either the least
// squares technique (fast), or the optimal algorithm (slow) developed by
// Kanatani et al. can be utilized to compute the homography matrix, if the 
// features are coplanar. If the feature points are non-coplanar, an implementation
// of a method developed by Ezio Malis is provided that allows computation of 
// homography of a 'virtual' plane. 
// The algorithm given in Faugeras' book and Shastry's book are implemented for 
// decomposition of the homography matrix. 
//
// The implementation of optimal homography computation algorithm is not my 
// work. It is a minor adaption of the original implementation by Naoya Ohta and  
// Shimizu Yoshiyuki (1999/2/25) of Computer Science Dept., Gunma University.
//
// References:
// ===========
// <ul>
// <li> Least squares algorithm: See R. Sukthankar, R. G. Stockton, and 
//      M. D. Mullin, "Smarter Presentations: Exploiting Homography in Camera-
//      Projection Systems," Proc. of ICCV, 2001.
// <li> Kenichi Kanatani optimal algorithm: See K. Kanatani, N. Ohta, and 
//      Y. Kanazawa, "Optimal Homography Computation with a Reliability Measure," 
//      IEICE Trans. on Information and Systems, Vol. E83-D, No. 7, pp.1369-1374, 
//      2000.
// <li> Malis' virtual parallax algorithm: E. Malis, and F. Chaumette, 
//      "2 1/2 D Visual Servoing with Respect to Unknown Objects Through a New 
//      Estimation Scheme of Camera Displacement," IJCV, 37(1), 79-97, 2000.
// <li> Faugeras homography decomposition algorithm: See O.Faugeras, Three-
//      Dimensional Computer Vision, The MIT Press, ISBN: 0262061589, page 290.
// <li> Shastry's alternate decomposition algorithm: See  Y. Ma, S. Soatto, 
//      J. Košecká, and S. Sastry, An Invitation to 3D Vision, Springer-Verlag, 
//      ISBN: 0387008934, page 136.
// </ul>
// <b>Example Program:</b>
// \include Homography.t.cpp
// \include decomposeHomography.t.cpp
//==============================================================================

class ProjectiveHomography
{
 public:
  ProjectiveHomography(int nFeatures, homography_method_t method);
   // Default constructor. Allocates memory required for computations. 
   //  nFeatures  The number of feature points used in the computation
   //             of Homography.
   //  method     Specify method for computing homography (least squares 
   //             method, kanatani algorithm, or Malis' virtual 
   //             parallax method).        
   
  ~ProjectiveHomography();
   // Default destructor frees allocated resources.
   
  int compute(MatrixBase<> &p2, MatrixBase<> &p1, Matrix<3, 3> &Hpn, 
              VectorBase<> &sc, Matrix<3,3> &dev, int maxItr = 100);
   // Compute the normalized projective homography from homogeneous image
   // coordinates such that p2 = sc * Hpn * p1.
   //  p2, p1  Sets of homogeneous image coordinates arranged as columns of 
   //          a matrix. IMPORTANT NOTE: If using the virtual parallax method, 
   //          it will be assumed that the first three columns of this matrix 
   //          define the virtual plane (input).
   //  Hpn     The estimated projective homography, normalized by the element
   //          at (3,3), i.e., Hpn = [1.0/Hp(3,3)] * Hp. (output)
   //  sc      The scale factor for every feature point (output). IMPORTANT NOTE:
   //          If using the virtual parallax algorithm, only the first three
   //          elements of this vector (corresponding to features that define 
   //          the virtual plane) will have valid values. This is because 
   //          the scale factor for all other features cannot be accurately 
   //          determined without using information from decomposition of the
   //          homography.
   //  dev     Deviation matrix, valid only when using the optimal 
   //          algorithm (output).
   //  maxItr  Maximum number of iteration allowed for convergence, valid only
   //          when using the optimal algorithm (input).
   //  return  number of iterations if using the optimal algorithm, 
   //          1 if using the least squares method, and -1 on error.
   
 protected:
  // ========== END OF INTERFACE ==========
 private:
  double *d_multWork;
  int multiply3n(const MatrixBase<> &a, const MatrixBase<> &b, MatrixBase<> &c);
  int computeLS(const MatrixBase<> &p2, const MatrixBase<> &p1, Matrix<3, 3> &Hpn, 
              VectorBase<> &sc);
  int computeKK(const MatrixBase<> &p2, const MatrixBase<> &p1, Matrix<3, 3> &Hpn, 
              VectorBase<> &sc, Matrix<3,3> &dev, int maxItr);
  int computeVP(MatrixBase<> &p2, MatrixBase<> &p1, Matrix<3, 3> &Hpn, 
              VectorBase<> &sc);
  int d_computeMethod; 
  int d_nFeatures;     
  gsl_matrix *d_gslu;
  gsl_vector *d_gslb;
  Vector<3> *d_gslx1;
  Vector<3> *d_gslx2;
  int d_numcombos;
  gsl_matrix *d_gslc;
  gsl_matrix *d_gslct;
  gsl_matrix *d_gslcaug;
  gsl_eigen_symmv_workspace *d_gslwork1;
  gsl_eigen_symmv_workspace *d_gslwork2;
  gsl_combination *d_gslcombos;
};

#endif // #ifndef INCLUDED_HOMOGRAPHY_HPP
