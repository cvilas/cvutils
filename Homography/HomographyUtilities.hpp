//==============================================================================
// HomographyUtilities.hpp - Utility functions for internal use by functions in 
//                           Homography.hpp
//
// Project       : Computer Vision Utilities (cvutils)
// Author        : Vilas Kumar Chitrakaran (cvilas@ces.clemson.edu)
//               : Tensor math functions programmed by Naoya Ohta
//                 (1998/11/11) Computer Science Department, 
//                 Gunma University
// Version       : 0.1 (December 2003)
// Compatibility : POSIX, GCC
//==============================================================================

#ifndef INCLUDED_HOMOGRAPHYUTILITIES_HPP
#define INCLUDED_HOMOGRAPHYUTILITIES_HPP

#include "Vector.hpp"
#include "GSLCompat.hpp"
#include "gsl/gsl_eigen.h"
#include <math.h>
#include <inttypes.h>

//========================================================================  
// This file contains code originally developed by researchers such as
// Naoya Ohta and others in Dr Kenichi Kanatani's laboratory at 
// Okayama University and Gunma University. Their code has been modified
// for use with the Math Library.
//========================================================================  

class Tensor3333
{
 public:
  // constructors
  Tensor3333();
  Tensor3333(const Tensor3333&);

  // initializer
  void clear();

  // operators
  Tensor3333& operator=(const Tensor3333&);
  double& operator()(int, int, int, int);
  const double& operator()(int, int, int, int) const;

  // arithmetic operators
  Tensor3333& operator+=(const Tensor3333&);
  Tensor3333& operator-=(const Tensor3333&);
  Tensor3333& operator*=(double);
  Tensor3333& operator/=(double);

  friend Tensor3333 operator+(const Tensor3333&, const Tensor3333&);
  friend Tensor3333 operator-(const Tensor3333&, const Tensor3333&);
  friend Tensor3333 operator-(const Tensor3333&);
  friend Matrix<3,3> operator*(const Tensor3333&, const Matrix<3,3> &m);
  friend Tensor3333 operator*(double, const Tensor3333&);
  friend Tensor3333 operator*(const Tensor3333&, double);
  friend Tensor3333 operator/(const Tensor3333&, double);

 private:
  double _t[81];
};

Matrix<9,9> type99(const Tensor3333 &);    // 3333-Tensor -----> 99-martix
Tensor3333 type3333(const Matrix<9,9> &);  //             <-----
Matrix<6,6> type66(const Tensor3333 &);    // (33)(33)-Tensor -> 66-martix
Tensor3333 type3333s(const Matrix<6,6> &); //                 <-
Vector<9> type9(const Matrix<3,3> &);      // 33-matrix -------> 9-vectorK
Matrix<3,3> type33(const Vector<9> &);     //           <-------
Vector<6> type6(const Matrix<3,3> &);      // (33)-matrix -----> 6-vectorK
Matrix<3,3> type33s(const Vector<6> &);    //             <-----
Vector<3> type3(const Matrix<3,3> &);      // [33]-matrix -----> 3-vectorK
Matrix<3,3> type33a(const Vector<3> &);    //             <-----

// Tensor product of 33matrix and 33matrix
Tensor3333 tensprod(const Matrix<3,3> &, const Matrix<3,3> &);

// eigenvalues and eigenmatrices
void eigens(const Tensor3333 &t, Matrix<3,3> *ma, Vector<9> &v);

// generalized inverse
int ginvs(const Tensor3333 &t, Tensor3333 &inv, int rank);

template<int r, int c, class T>
T innerProduct(const Matrix<r, c, T> &m1, const Matrix<r, c, T> &m2);
 //  return  Inner product of matrices \m1 and \m2

template<class T>
void exteriorProduct(const Matrix<3,3, T> &m1, const Matrix<3,3, T> &m2, Matrix<3,3, T> &out);
 // mout = Exterior product of matrices \m1 and \m2

template<class T>
void exteriorProduct(const Matrix<3,3, T> &m, const Vector<3, T> &v, Matrix<3,3, T> &mout);
 // mout = Exterior product of \m and \v
 
template<class T>
void exteriorProduct(const Vector<3,T> &v, const Matrix<3,3, T> &m, Matrix<3,3, T> &mout);
 // mout = Exterior product of \v and \m

template<int s, class T>
void symmetrize(Matrix<s, s, T> &m, Matrix<s, s, T> &mout);
 // Symmetrization of matrix mout = (m + m^T).

template<int s, class T>
void antiSymmetrize(Matrix<s, s, T> &m, Matrix<s, s, T> &mout);
 // Anti-Symmetrization of matrix mout = (m - m^T).

template<class T>
int skewSymmetricToVector(const Matrix<3, 3, T> &m, Vector<3, T> &v);
 // Generate a 3x1 vector \v from its skew-symmetric matrix
 // representation \m.
 //  return  0 on success, -1 if m is not skew symmetric

template<int s>
int ginvs(const Matrix<s, s> &m, int rk, Matrix<s, s> &inv, double cn = 1.0e10);
 // Generalized inverse using spectral decomposition
 //  m       The matrix whose inverse we want to calculate
 //  rk      Designated rank
 //  inv     The inverse (output)
 //  cn      condition number
 //  return  0 on success, -1 on error.

template<int s>
int eigens(const Matrix<s, s> &matrix, Matrix<s, s> &evec, Vector<s> &eval);
 // spectral decomposition for symmetric matrix. Note that this function
 // does not check to ensure that input matrix is symmetric; that is upto 
 // the user.
 //  matrix  input matrix
 //  evec    eigen vectors arranged as columns
 //  eval    eigen values of the matrix \matrix.
 //  return  0 on success, -1 on failure
	
int eeps(int, int, int);
 //  return  Eddington's epsilon.

unsigned int nchoosek(unsigned int n, unsigned int k);
 // compute number of combinations of k items 
 // from n items.


//======================================================================== 
// innerProduct
//========================================================================  
template<int r, int c, class T>
T innerProduct(const Matrix<r, c, T> &m1, const Matrix<r, c, T> &m2)
{
 T product;
 product = 0.0;
 for(int col = 1; col <= c; ++col)
  for(int row = 1; row <= r; ++row)
   product += m1(col, row) * m2(col, row);
 return product;
}


//======================================================================== 
// exteriorProduct
//========================================================================  
template<class T>
void exteriorProduct(const Matrix<3,3, T> &m1, const Matrix<3,3, T> &m2, Matrix<3,3, T> &n)
{
 n(1,1) = m1(2,2)*m2(3,3) - m1(2,3)*m2(3,2)
          - m1(3,2)*m2(2,3) + m1(3,3)*m2(2,2);
 n(1,2) = - m1(2,1)*m2(3,3) + m1(2,3)*m2(3,1)
          + m1(3,1)*m2(2,3) - m1(3,3)*m2(2,1);
 n(1,3) = m1(2,1)*m2(3,2) - m1(2,2)*m2(3,1)
          - m1(3,1)*m2(2,2) + m1(3,2)*m2(2,1);
 n(2,1) = - m1(1,2)*m2(3,3) + m1(1,3)*m2(3,2)
         + m1(3,2)*m2(1,3) - m1(3,3)*m2(1,2);
 n(2,2) = m1(1,1)*m2(3,3) - m1(1,3)*m2(3,1)
          - m1(3,1)*m2(1,3) + m1(3,3)*m2(1,1);
 n(2,3) = - m1(1,1)*m2(3,2) + m1(1,2)*m2(3,1)
          + m1(3,1)*m2(1,2) - m1(3,2)*m2(1,1);
 n(3,1) = m1(1,2)*m2(2,3) - m1(1,3)*m2(2,2)
          - m1(2,2)*m2(1,3) + m1(2,3)*m2(1,2);
 n(3,2) = - m1(1,1)*m2(2,3) + m1(1,3)*m2(2,1)
          + m1(2,1)*m2(1,3) - m1(2,3)*m2(1,1);
 n(3,3) = m1(1,1)*m2(2,2) - m1(1,2)*m2(2,1)
          - m1(2,1)*m2(1,2) + m1(2,2)*m2(1,1);
}


template<class T>
void exteriorProduct(const Matrix<3,3, T> &m, const Vector<3, T> &v, Matrix<3,3, T> &n)
{
 for(int i = 0; i < 3; ++i) {
  n(i+1, 1) = m(i+1, 3) * v(2) - m(i+1, 2) * v(3);
  n(i+1, 2) = m(i+1, 1) * v(3) - m(i+1, 3) * v(1);
  n(i+1, 3) = m(i+1, 2) * v(1) - m(i+1, 1) * v(2);
 }
}

template<class T>
void exteriorProduct(const Vector<3,T> &v, const Matrix<3,3, T> &m,  Matrix<3,3,T> &n)
{
 for(int i = 0; i < 3; ++i) {
  n(1, i+1) = v(2) * m(3, i+1) - v(3) * m(2, i+1);
  n(2, i+1) = v(3) * m(1, i+1) - v(1) * m(3, i+1);
  n(3, i+1) = v(1) * m(2, i+1) - v(2) * m(1, i+1);
 }
}


//======================================================================== 
// symmetrize
//========================================================================  
template<int s, class T>
void symmetrize(Matrix<s, s, T> &m, Matrix<s, s, T> &out)
{
 out = (m + transpose(m))/2.0;
}


//======================================================================== 
// antiSymmetrize
//========================================================================  
template<int s, class T>
void antiSymmetrize(Matrix<s, s, T> &m, Matrix<s, s, T> &out)
{
 out = (m - transpose(m))/2.0;
}


//======================================================================== 
// skewSymmetricToVector
//========================================================================  
template <class T>
int skewSymmetricToVector(const Matrix<3, 3, T> &m, Vector<3,T> &v)
{	
 if( transpose(m) != -1.0 * m )	{
  return -1;
 }
 v = m(3,2), m(1,3), m(2,1);
 return 0;
}


//======================================================================== 
// ginvs
//========================================================================  
template<int size>
int ginvs(const Matrix<size, size> &m, int rk, Matrix<size, size> &inv, double cn)
{
 int  i;

 // check designated rank
 if(rk < 1 || rk > size) {
  fprintf(stderr, "%s\n", "ginvs: invalid rank.");
  return(-1); 
 }

 // check designated condition number
 if(cn < 1.0) {
  fprintf(stderr, "%s\n", "ginvs: invalid condition number.");
  return(-1); 
 }

 // memory allocation for work buffers
 Matrix<size, size> u, diagV;
 Vector<size> v;
 int idx[size];

 diagV = 0.0;
  
 // spectral decomposition
 eigens(m,u,v);

 // sort by absolute value of eigenvalues
 for(i = 0; i < size; ++i)  
  idx[i] = i+1;
	
 for(i = 0; i < size-1; i++)
  for(int j = i+1; j < size; j++) {
   double absi = v(idx[i]), absj = v(idx[j]);
   if(absi < 0.0) absi *= -1.0;
   if(absj < 0.0) absj *= -1.0;
   if(absj > absi) { int k = idx[i]; idx[i] = idx[j]; idx[j] = k; }
  }

 // check condition number
 double emax = v(idx[0]), emin = v(idx[rk-1]);
 if(emax < 0.0) emax *= -1.0;
 if(emin < 0.0) emin *= -1.0;
 if(emax >= cn*emin) {
  fprintf(stderr, "%s\n", "ginvs: ill condition.");
  return(-1); 
 }
    
 // compute generalized inverse
 for(i = 0; i < rk; i++)
  diagV(i+1,i+1) = 1.0/v(idx[i]);
 inv = u * diagV * transpose(u);
 
 return 0;
}


//======================================================================== 
// eigens
//========================================================================  
template<int size>
int eigens(const Matrix<size, size> &matrix, Matrix<size, size> &evec, Vector<size> &eval)
{
 Matrix<size, size> mat;
 gsl_eigen_symmv_workspace gh_work;
 gsl_matrix gh_mat;
 gsl_vector gh_eval;
 gsl_matrix gh_evec;

 mat = matrix;
	
 // set up gsl workspace struct...
 double work[4 * size];
 gh_work.d = work;
 gh_work.sd = &(work[size]);
 gh_work.gc = &(work[2 * size]);
 gh_work.gs = &(work[3 * size]);
 gh_work.size = size;
	
 GSLCompat_matrix(&mat, &gh_mat);
 GSLCompat_matrix(&evec, &gh_evec);
 GSLCompat_vector(&eval, &gh_eval);
	
 // Get the eigen vectors and sort them 
 if( gsl_eigen_symmv(&gh_mat, &gh_eval, &gh_evec, &gh_work) )
  return -1;
 if( gsl_eigen_symmv_sort(&gh_eval, &gh_evec, GSL_EIGEN_SORT_VAL_DESC) )
  return -1;
        
 return 0;
}


#endif // INCLUDED_HOMOGRAPHYUTILITIES_HPP
