//==============================================================================
// HomographyUtilities.cpp - Implementation of utility functions
//
// Project       : Computer Vision Utilities (cvutils)
// Author        : Vilas Kumar Chitrakaran (cvilas@ces.clemson.edu)
//               : Tensor math functions programmed by Naoya Ohta
//                 (1998/11/11) Computer Science Department, 
//                 Gunma University
// Version       : 0.1 (December 2003)
// Compatibility : POSIX, GCC
//==============================================================================

#include "HomographyUtilities.hpp"

#define SQRTOFTWO 1.41421356237309504880168872421

//constructors
Tensor3333::Tensor3333()
{
 for(int i=0; i<81; i++) _t[i] = 0.0;
}

Tensor3333::Tensor3333(const Tensor3333& t)
{
 for(int i=0; i<81; i++) _t[i] = t._t[i];
}

// initializer
void Tensor3333::clear()
{
 for(int i=0; i<81; i++) _t[i] = 0.0;
}

// operators
Tensor3333& Tensor3333::operator=(const Tensor3333& t)
{
 if(&t != this)
 for(int i=0; i<81; i++) _t[i] = t._t[i];
 return *this;
}

double& Tensor3333::operator()(int i, int j, int k, int l)
{
 return _t[27*i+9*j+3*k+l];
}

const double& Tensor3333::operator()(int i, int j, int k, int l) const
{
 return _t[27*i+9*j+3*k+l];
}

// arithmetic operators
Tensor3333& Tensor3333::operator+=(const Tensor3333& t)
{
 for(int i=0; i<81; i++) _t[i] += t._t[i];
 return *this;
}

Tensor3333& Tensor3333::operator-=(const Tensor3333& t)
{
 for(int i=0; i<81; i++) _t[i] -= t._t[i];
 return *this;
}

Tensor3333& Tensor3333::operator*=(double a)
{
 for(int i=0; i<81; i++) _t[i] *= a;
 return *this;
}

Tensor3333& Tensor3333::operator/=(double a)
{
 for(int i=0; i<81; i++) _t[i] /= a;
 return *this;
}

Tensor3333 operator+(const Tensor3333& t1, const Tensor3333& t2)
{
 Tensor3333 s;
 for(int i=0; i<81; i++) s._t[i] = t1._t[i] + t2._t[i];
 return s;
}

Tensor3333 operator-(const Tensor3333& t1, const Tensor3333& t2)
{
 Tensor3333 s;
 for(int i=0; i<81; i++) s._t[i] = t1._t[i] - t2._t[i];
 return s;
}

Tensor3333 operator-(const Tensor3333& t)
{
 Tensor3333 s;
 for(int i=0; i<81; i++) s._t[i] = -t._t[i];
 return s;
}

Matrix<3,3> operator*(const Tensor3333& t, const Matrix<3,3> &m)
{
 int i, j, k, l;
 Matrix<3,3> n;
 for(i=0; i<3; i++)
  for(j=0; j<3; j++) {
   double sum = 0.0;
    for(k=0; k<3; k++)
     for(l=0; l<3; l++)
      sum += t._t[27*i+9*j+3*k+l]*m(k+1, l+1);     
     n(i+1, j+1) = sum;
  }
 return n;
}


Tensor3333 operator*(double a, const Tensor3333& t)
{
 Tensor3333 s;
 for(int i=0; i<81; i++) s._t[i] = a*t._t[i];
 return s;
}

Tensor3333 operator*(const Tensor3333& t, double a)
{
 Tensor3333 s;
 for(int i=0; i<81; i++) s._t[i] = a*t._t[i];
 return s;
}

Tensor3333 operator/(const Tensor3333& t, double a)
{
 Tensor3333 s;
 for(int i=0; i<81; i++) s._t[i] = t._t[i]/a;
 return s;
}

// conversion from 3333-Tensor to 99-martix
Matrix<9,9> type99(const Tensor3333 &t)
{
 Matrix<9,9> n;
 for(int i=0; i<3; i++)
  for(int j=0; j<3; j++)
   for(int k=0; k<3; k++)
    for(int l=0; l<3; l++)
     n(3*i+j+1, 3*k+l+1) = t(i,j,k,l);
 return n;
}

// conversion from 99-martix to 3333-Tensor
Tensor3333 type3333(const Matrix<9,9> &m)
{
 Tensor3333 s;
 for(int i=0; i<3; i++)
  for(int j=0; j<3; j++)
   for(int k=0; k<3; k++)
    for(int l=0; l<3; l++)
     s(i,j,k,l) = m(3*i+j+1, 3*k+l+1);
 return s;
}

// conversion from (33)(33)-Tensor to 66-martix 
Matrix<6,6> type66(const Tensor3333 &t)
{
/*// CHECK FOR SYMMETRY
 int i, j, k, l;
 for(i=0; i<3; i++)
  for(k=0; k<3; k++)
   for(l=k+1; l<3; l++)
    if(t(i,i,l,k) != t(i,i,k,l)) {
     fprintf(stderr,"type66(Tensor3333&): not symmetric Tensor.\n");
     exit(1); 
    }

  for(i=0; i<3; i++)
    for(j=i+1; j<3; j++)
      for(k=0; k<3; k++)
        for(l=k+1; l<3; l++)
          if(t(i,j,k,l) != t(i,j,l,k) || t(i,j,k,l) != t(j,i,k,l) ||
	     t(i,j,k,l) != t(j,i,l,k)) {
            fprintf(stderr,"type66(Tensor3333&): not symmetric Tensor.\n");
            exit(1); }
*/
  Matrix<6,6> n;

  n(1, 1) = t(0,0,0,0);
  n(2, 1) = t(1,1,0,0);
  n(3, 1) = t(2,2,0,0);
  n(4, 1) = SQRTOFTWO*t(1,2,0,0);
  n(5, 1) = SQRTOFTWO*t(2,0,0,0);
  n(6, 1) = SQRTOFTWO*t(0,1,0,0);

  n(1, 2) = t(0,0,1,1);
  n(2, 2) = t(1,1,1,1);
  n(3, 2) = t(2,2,1,1);
  n(4, 2) = SQRTOFTWO*t(1,2,1,1);
  n(5, 2) = SQRTOFTWO*t(2,0,1,1);
  n(6, 2) = SQRTOFTWO*t(0,1,1,1);

  n(1, 3) = t(0,0,2,2);
  n(2, 3) = t(1,1,2,2);
  n(3, 3) = t(2,2,2,2);
  n(4, 3) = SQRTOFTWO*t(1,2,2,2);
  n(5, 3) = SQRTOFTWO*t(2,0,2,2);
  n(6, 3) = SQRTOFTWO*t(0,1,2,2);

  n(1, 4) = SQRTOFTWO*t(0,0,1,2);
  n(1, 4) = SQRTOFTWO*t(1,1,1,2);
  n(2, 4) = SQRTOFTWO*t(2,2,1,2);
  n(3, 4) = 2.0*t(1,2,1,2);
  n(4, 4) = 2.0*t(2,0,1,2);
  n(5, 4) = 2.0*t(0,1,1,2);

  n(1, 5) = SQRTOFTWO*t(0,0,2,0);
  n(2, 5) = SQRTOFTWO*t(1,1,2,0);
  n(3, 5) = SQRTOFTWO*t(2,2,2,0);
  n(4, 5) = 2.0*t(1,2,2,0);
  n(5, 5) = 2.0*t(2,0,2,0);
  n(6, 5) = 2.0*t(0,1,2,0);

  n(1, 6) = SQRTOFTWO*t(0,0,0,1);
  n(2, 6) = SQRTOFTWO*t(1,1,0,1);
  n(3, 6) = SQRTOFTWO*t(2,2,0,1);
  n(4, 6) = 2.0*t(1,2,0,1);
  n(5, 6) = 2.0*t(2,0,0,1);
  n(6, 6) = 2.0*t(0,1,0,1);

  return n;
}


// conversion from 66-martix to (33)(33)-Tensor
Tensor3333 type3333s(const Matrix<6,6> &m)
{
  Tensor3333 s;
  s(0,0,0,0) = m(1,1);
  s(0,0,1,1) = m(1,2);
  s(0,0,2,2) = m(1,3);
  s(0,0,1,2) = s(0,0,2,1) = m(1,4)/SQRTOFTWO;
  s(0,0,2,0) = s(0,0,0,2) = m(1,6)/SQRTOFTWO;
  s(0,0,0,1) = s(0,0,1,0) = m(1,6)/SQRTOFTWO;

  s(1,1,0,0) = m(2,1);
  s(1,1,1,1) = m(2,2);
  s(1,1,2,2) = m(2,3);
  s(1,1,1,2) = s(1,1,2,1) = m(2,4)/SQRTOFTWO;
  s(1,1,2,0) = s(1,1,0,2) = m(2,5)/SQRTOFTWO;
  s(1,1,0,1) = s(1,1,1,0) = m(2,6)/SQRTOFTWO;

  s(2,2,0,0) = m(3, 1);
  s(2,2,1,1) = m(3, 2);
  s(2,2,2,2) = m(3, 3);
  s(2,2,1,2) = s(2,2,2,1) = m(3, 4)/SQRTOFTWO;
  s(2,2,2,0) = s(2,2,0,2) = m(3, 5)/SQRTOFTWO;
  s(2,2,0,1) = s(2,2,1,0) = m(3, 6)/SQRTOFTWO;

  s(1,2,0,0) = s(2,1,0,0) = m(4, 1)/SQRTOFTWO;
  s(1,2,1,1) = s(2,1,1,1) = m(4, 2)/SQRTOFTWO;
  s(1,2,2,2) = s(2,1,2,2) = m(4, 3)/SQRTOFTWO;
  s(1,2,1,2) = s(1,2,2,1) = s(2,1,1,2) = s(2,1,2,1) = m(4, 4)/2.0;
  s(1,2,2,0) = s(1,2,0,2) = s(2,1,2,0) = s(2,1,0,2) = m(4, 5)/2.0;
  s(1,2,0,1) = s(1,2,1,0) = s(2,1,0,1) = s(2,1,1,0) = m(4, 6)/2.0;

  s(2,0,0,0) = s(0,2,0,0) = m(5, 1)/SQRTOFTWO;
  s(2,0,1,1) = s(0,2,1,1) = m(5, 2)/SQRTOFTWO;
  s(2,0,2,2) = s(0,2,2,2) = m(5, 3)/SQRTOFTWO;
  s(2,0,1,2) = s(2,0,2,1) = s(0,2,1,2) = s(0,2,2,1) = m(5, 4)/2.0;
  s(2,0,2,0) = s(2,0,0,2) = s(0,2,2,0) = s(0,2,0,2) = m(5, 5)/2.0;
  s(2,0,0,1) = s(2,0,1,0) = s(0,2,0,1) = s(0,2,1,0) = m(5, 6)/2.0;

  s(0,1,0,0) = s(1,0,0,0) = m(6, 1)/SQRTOFTWO;
  s(0,1,1,1) = s(1,0,1,1) = m(6, 2)/SQRTOFTWO;
  s(0,1,2,2) = s(1,0,2,2) = m(6, 3)/SQRTOFTWO;
  s(0,1,1,2) = s(0,1,2,1) = s(1,0,1,2) = s(1,0,2,1) = m(6, 4)/2.0;
  s(0,1,2,0) = s(0,1,0,2) = s(1,0,2,0) = s(1,0,0,2) = m(6, 5)/2.0;
  s(0,1,0,1) = s(0,1,1,0) = s(1,0,0,1) = s(1,0,1,0) = m(6, 6)/2.0;

  return s;
}

// conversion from 33-matrix to 9-vectorK
Vector<9> type9(const Matrix<3,3> &m)
{
 Vector<9> u;
 for(int i=0; i<3; i++)
  for(int j=0; j<3; j++)
   u(3*i+j+1) = m(i+1, j+1);
 return u;
}

// conversion from 9-vectorK to 33-matrix
Matrix<3,3> type33(const Vector<9> &v)
{
 Matrix<3,3> n;
 for(int i=0; i<3; i++)
  for(int j=0; j<3; j++)
   n(i+1, j+1) = v(3*i+j+1);
 return n;
}

// conversion from (33)-matrix to 6-vectorK
Vector<6> type6(const Matrix<3,3> &m)
{
/*
 if(m(1,2) != m(2,1) || m(1,2) != m(3,1) || m(2,3) != m(3,2)) {
    fprintf(stderr,"type6(matrix&): not symmetric matrix.\n");
    exit(1); }
*/

 Vector<6> u;
 u(1) = m(1,1);     u(2) = m(2,2);     u(3) = m(3,3);
 u(4) = SQRTOFTWO*m(2,3); u(5) = SQRTOFTWO*m(3,1); u(6) = SQRTOFTWO*m(1,2);

 return u;
}


// conversion from 6-vectorK to (33)-matrix
Matrix<3,3> type33s(const Vector<6> &v)
{
 Matrix<3,3> n;
 n(1,1) = v(1);     n(1,2) = v(6)/SQRTOFTWO; n(1,3) = v(5)/SQRTOFTWO;
 n(2,1) = v(6)/SQRTOFTWO; n(2,2) = v(2);     n(2,3) = v(4)/SQRTOFTWO;
 n(3,1) = v(5)/SQRTOFTWO; n(3,2) = v(4)/SQRTOFTWO; n(3,3) = v(3);
 return n;
}


// conversion from [33]-matrix to 3-vectorK
Vector<3> type3(const Matrix<3,3> &m)
{
 /*
  if(m(1,1) != 0.0 || m(2,2) != 0.0 || m(3,3) != 0.0 ||
     m(1,2) != -m(2,1) || m(1,3) != -m(3,1) || m(2,3) != -m(3,2)) {
    fprintf(stderr,"type3(matrix&): not antisymmetric matrix.\n");
    exit(1); }
 */
 Vector<3> u;
 u(1) = m(3,2), m(1,3), m(2,1);
 return u;
}

// conversion from 3-vectorK to [33]-matrix
Matrix<3,3> type33a(const Vector<3> &v)
{
 Matrix<3,3> n;
 n(1,1) = 0.0;
 n(2,2) = 0.0;
 n(3,3) = 0.0;
 n(3,2) = v(1); 
 n(2,3) = -v(1);
 n(1,3) = v(2); 
 n(2,1) = -v(2);
 n(2,1) = v(3); 
 n(1,2) = -v(3);
 return n;
}


// Tensor product of 33matrix and 33matrix
Tensor3333 tensprod(const Matrix<3,3> &m1, const Matrix<3,3> &m2)
{
 Tensor3333 s;
 for(int i=0; i<3; i++)
  for(int j=0; j<3; j++)
   for(int k=0; k<3; k++)
    for(int l=0; l<3; l++)
     s(i,j,k,l) = m1(i+1, j+1)*m2(k+1, l+1);
 return s;
}

// eigenvalues and eigenmatrices
void eigens(const Tensor3333 &t, Matrix<3,3> *ma, Vector<9> &v)
{
 Matrix<9,9> ev;
 Matrix<9,9> mt, smt;
 mt = type99(t);
 symmetrize(mt, smt);
 eigens(smt,ev, v);
 for(int i = 0; i < 9; i++) {
  ma[i] = ev(1, i+1), ev(2, i+1), ev(3, i+1), 
          ev(4, i+1), ev(5, i+1), ev(6, i+1), 
	  ev(7, i+1), ev(8, i+1), ev(9, i+1);
 }
}

// generalized inverse
int ginvs(const Tensor3333& t, int rank, Tensor3333 &inv)
{
 Matrix<9,9> mt, smt, minv;
 mt = type99(t);
 symmetrize(mt, smt);
 ginvs(smt,rank,minv);
 inv = type3333(minv);
 return 0;
}


//========================================================================  
// eeps
//========================================================================  
int eeps(int i, int j, int k)
{
  if((i==0 && j==1 && k==2) || (i==1 && j==2 && k==0) ||
     (i==2 && j==0 && k==1))
    return 1;
  else if((i==0 && j==2 && k==1) || (i==1 && j==0 && k==2) ||
	  (i==2 && j==1 && k==0))
    return -1;
  else
    return 0;
}


//======================================================================== 
// nchoosek
//========================================================================  
unsigned int nchoosek(unsigned int n, unsigned int k)
{
 uint64_t top = 1;
 uint64_t bottom = 1;
 
 if(n < k)
  return 0;
  
 uint64_t i;
 for(i = k+1; i <= n; i++) {
  top = top * i;
 }
 
 for(i = n-k; i > 0; i--) {
  bottom = bottom * i;
 }

 return (top/bottom);
}

 
 
