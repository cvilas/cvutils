//==============================================================================
// Homography.cpp - Implementation of class ProjectiveHomography
//
// Project       : Computer Vision Utilities (cvutils)
// Author        : Vilas Kumar Chitrakaran (cvilas@ces.clemson.edu)
// Version       : 0.1 (December 2003)
// Compatibility : POSIX, GCC
//==============================================================================

#include "Homography.hpp"
#include "HomographyUtilities.hpp"

//#define DEBUG

static int scale(const MatrixBase<> &i1, const MatrixBase<> &i2, 
                 Vector<3> *x1, Vector<3> *x2,
                 Vector<3> &ic1, Vector<3> &ic2, double &f0);

static Matrix<3,3> crdcnv(const Matrix<3,3> &H, const Vector<3> &ic1, 
		          const Vector<3> &ic2, double f0);

int getSubMatrix(const MatrixBase<> &s, int pr, int pc, MatrixBase<> &sm);
static double norm(const Matrix<3,3> &m);


//==============================================================================
// ProjectiveHomography::ProjectiveHomography
//==============================================================================
ProjectiveHomography::ProjectiveHomography(int nFeatures, homography_method_t m)
{
 d_gslu = NULL;
 d_gslb = NULL;
 d_gslx1 = NULL;
 d_gslx2 = NULL;
 d_gslc = NULL;
 d_gslct = NULL;
 d_gslcaug = NULL;
 d_gslwork1 = NULL;
 d_gslwork2 = NULL;
 d_gslcombos = NULL;

 d_computeMethod = m;
 d_nFeatures = nFeatures;

 if(d_nFeatures == 0)
  return;
  
 if(d_computeMethod == e_ls) {
  d_gslu = gsl_matrix_alloc(2 * d_nFeatures, 8);
  d_gslb = gsl_vector_alloc(2 * d_nFeatures);
 } else if(d_computeMethod == e_kk) {
  d_gslx2 = new Vector<3>[d_nFeatures];
  d_gslx1 = new Vector<3>[d_nFeatures];
 } else if (d_computeMethod == e_vp) { 
  d_multWork = new double[3 * d_nFeatures];
  d_numcombos = nchoosek(nFeatures-3,3);
  d_gslcombos = gsl_combination_alloc(nFeatures-3,3);
  d_gslc = gsl_matrix_alloc(d_numcombos,7);
  d_gslct = gsl_matrix_alloc(7, d_numcombos);
  d_gslcaug = gsl_matrix_alloc(7, 7);
  d_gslwork1 = gsl_eigen_symmv_alloc(7);
  d_gslwork2 = gsl_eigen_symmv_alloc(3);
 } else {
  fprintf(stderr, "[ProjectiveHomography::~ProjectiveHomography]: ERROR unknown method specified.\n");
  return;
 }
}


//==============================================================================
// ProjectiveHomography::~ProjectiveHomography
//==============================================================================
ProjectiveHomography::~ProjectiveHomography()
{
 if(d_computeMethod == e_ls) {
  if(d_gslu) gsl_matrix_free(d_gslu);
  if(d_gslb) gsl_vector_free(d_gslb);
  d_gslu = NULL;
  d_gslb = NULL;
 } else if(d_computeMethod == e_kk) {
  if(d_gslx2) delete [] d_gslx2;
  if(d_gslx1) delete [] d_gslx1;
  d_gslx2 = NULL;
  d_gslx1 = NULL;
 } else if(d_computeMethod == e_vp){
  delete d_multWork;
  gsl_combination_free(d_gslcombos);
  gsl_matrix_free(d_gslc);
  gsl_matrix_free(d_gslct);
  gsl_matrix_free(d_gslcaug);
  gsl_eigen_symmv_free(d_gslwork1);
  gsl_eigen_symmv_free(d_gslwork2);
 }
}


//==============================================================================
// ProjectiveHomography::compute
//==============================================================================
int ProjectiveHomography::compute(MatrixBase<> &p2, MatrixBase<> &p1, 
                              Matrix<3,3> &Hpn, VectorBase<> &sc, 
                              Matrix<3,3> &dev, int maxItr)
{
 if(sc.getNumElements() != d_nFeatures || p1.getNumColumns() != d_nFeatures
   || p2.getNumColumns() != d_nFeatures || p1.getNumRows() != 3 
   || p2.getNumRows() != 3) {
  fprintf(stderr, "[ProjectiveHomography::compute]: %s\n%s%d\n", 
          "Either arguments have incorrect dimensions",
          "or number of features not ", d_nFeatures);
  return -1;
 }

 if( d_computeMethod == e_ls )
  return computeLS(p2, p1, Hpn, sc);
 
 if(d_computeMethod == e_kk)
  return computeKK(p2, p1, Hpn, sc, dev, maxItr);
  
 if( d_computeMethod == e_vp )
  return computeVP(p2, p1, Hpn, sc);
 
 // should not get here
 return -1;
}


//==============================================================================
// ProjectiveHomography::computeLS
//==============================================================================
int ProjectiveHomography::computeLS(const MatrixBase<> &p2, const MatrixBase<> &p1, 
                                    Matrix<3, 3> &Hpn, VectorBase<> &sc)
{
 Vector<8> work, s, x;
 Matrix<8,8> v;
 gsl_vector gsl_work, gsl_s, gsl_x;
 gsl_matrix gsl_v;

 GSLCompat_vector(&work, &gsl_work);
 GSLCompat_vector(&s, &gsl_s);
 GSLCompat_vector(&x, &gsl_x);
 GSLCompat_matrix(&v, &gsl_v);
 
 int row, i;

 for (i = 1; i <= d_nFeatures; ++i) {
  row = 2 * i - 1;
  gsl_matrix_set(d_gslu, row - 1, 0, p1.getElement(1, i));
  gsl_matrix_set(d_gslu, row - 1, 1, p1.getElement(2, i));
  gsl_matrix_set(d_gslu, row - 1, 2, 1.0);
  gsl_matrix_set(d_gslu, row - 1, 3, 0.0);
  gsl_matrix_set(d_gslu, row - 1, 4, 0.0);
  gsl_matrix_set(d_gslu, row - 1, 5, 0.0);
  gsl_matrix_set(d_gslu, row - 1, 6, -1.0 * p1.getElement(1, i) * p2.getElement(1, i));
  gsl_matrix_set(d_gslu, row - 1, 7, -1.0 * p1.getElement(2, i) * p2.getElement(1, i));
  gsl_vector_set(d_gslb, row - 1, 1.0 * p2.getElement(1, i));
 
  row = 2 * i;
  gsl_matrix_set(d_gslu, row - 1, 0, 0.0);
  gsl_matrix_set(d_gslu, row - 1, 1, 0.0);
  gsl_matrix_set(d_gslu, row - 1, 2, 0.0);
  gsl_matrix_set(d_gslu, row - 1, 3, p1.getElement(1, i));
  gsl_matrix_set(d_gslu, row - 1, 4, p1.getElement(2, i));
  gsl_matrix_set(d_gslu, row - 1, 5, 1.0);		
  gsl_matrix_set(d_gslu, row - 1, 6, -1.0 * p1.getElement(1, i) * p2.getElement(2, i));
  gsl_matrix_set(d_gslu, row - 1, 7, -1.0 * p1.getElement(2, i) * p2.getElement(2, i));
  gsl_vector_set(d_gslb, row - 1, 1.0 * p2.getElement(2, i));
 }

 if(gsl_linalg_SV_decomp(d_gslu, &gsl_v, &gsl_s, &gsl_work))
  return -1;
		
 if(gsl_linalg_SV_solve(d_gslu, &gsl_v, &gsl_s, d_gslb, &gsl_x))
  return -1;
	
 Hpn = x(1), x(2), x(3), 
       x(4), x(5), x(6),
       x(7), x(8), 1.0;
	
 for (int i = 1; i <= d_nFeatures; ++i) {
  sc.setElement(i, 1.0/(Hpn(3,1) * p1.getElement(1,i) 
                   + Hpn(3,2) * p1.getElement(2,i) + 1.0));
 }
 return 0;
}

//==============================================================================
// ProjectiveHomography::computeKK
//==============================================================================
int ProjectiveHomography::computeKK(const MatrixBase<> &p2, const MatrixBase<> &p1, 
                                    Matrix<3, 3> &Hpn, VectorBase<> &sc, 
                                    Matrix<3,3> &dev, int maxItr)
{
 Tensor3333 M, N;
 Matrix<3,3> W, A[3], H[9];
 Vector<3> e1, e2, e3;
 Vector<9> lmd;
 Vector<3> x1, x2;
 Matrix<3,3> Vx1, Vx2;
 Matrix<3,3> m1, m2, m1m2, m1m2Sym, tmp1;
 double epslmd;
 double eps2;
	
 double c;
 int a, i, j, k, l, it;
 Vector<3> imgCentroid1, imgCentroid2;
 double f0 = 1;

 if ( scale(p2, p1, d_gslx2, d_gslx1, imgCentroid2, imgCentroid1, f0) == -1) {
  return -1;
 }

 // initialization
 lmd = 0;
 e1 = 1.0, 0, 0;
 e2 = 0, 1.0, 0;
 e3 = 0, 0, 1.0;

 for(int i = 0; i < 9; ++i) H[i] = 0;

 c = 0.0;
 W = unitMatrix<3>();
 Vx1 = 1.0, 0, 0, 0, 1.0, 0, 0, 0, 0;
 Vx2 = 1.0, 0, 0, 0, 1.0, 0, 0, 0, 0;
 epslmd = 1e-10;

 // renormalization loop
 for(it = 1; ; it++) {
  M.clear(); N.clear();
  for(a = 1; a <= d_nFeatures; a++) {
   int n, m, p, q;
 
   x2 = d_gslx2[a-1];
   x1 = d_gslx1[a-1]; 
 
   // computation of weight matrix W for current point a
   if(it != 1) {
    exteriorProduct((H[8] * Vx1 * transpose(H[8])), x2, tmp1);
    exteriorProduct( x2, tmp1, m1 );
    exteriorProduct(Vx2, (Vector<3>)(H[8] * x1), tmp1);
    exteriorProduct( (Vector<3>)(H[8] * x1), tmp1, m2 );
    m1m2 = m1+m2;
    symmetrize(m1m2, m1m2Sym);
    ginvs( m1m2Sym, 2, W );
   }
  
   // accumulation for a half part of tensor N
   for(i=0; i<3; i++)
    for(j=0; j<3; j++)
     for(k=0; k<3; k++)
      for(l=0; l<3; l++)
       if((3*(i-1)+j) <= (3*(k-1)+l)) {
        for(m=0; m<3; m++)
         for(n=0; n<3; n++)
          for(p=0; p<3; p++)
           for(q=0; q<3; q++) {
            int s = eeps(i,m,p)*eeps(k,n,q);
     if(s != 0)
      N(i,j,k,l) += double(s) * W(m+1, n+1) 
                 *  (Vx1(j+1,l+1) * x2(p+1) * x2(q+1) 
                 + Vx2(p+1, q+1) * x1(j+1) * x1(l+1));
           }
       }

   // accumulation for tensor M
   exteriorProduct( e1, (x2 * transpose(x1)), A[0] );
   exteriorProduct( e2, (x2 * transpose(x1)), A[1] );
   exteriorProduct( e3, (x2 * transpose(x1)), A[2] );
    for (k=0; k<3; k++)
    for (l=0; l<3; l++)
     M += W(k+1, l+1) * tensprod(A[k],A[l]);
  } // loop end of a
 
  // fill the other half of tensor N
  for(i=0; i<3; i++)
   for(j=0; j<3; j++)
    for(k=0; k<3; k++)
     for(l=0; l<3; l++)
      if((3*(i-1)+j) > (3*(k-1)+l))
       N(i,j,k,l) = N(k,l,i,j);
  M /= double(d_nFeatures);
  N /= double(d_nFeatures);

  // computation of eigenmatrices
  if(it == 1) {
   eigens(M,H,lmd);
  } else {
   Tensor3333 tmp;
   tmp = M-c*N;
   eigens(tmp,H,lmd);
  }
   
  // condition to quit the renormalization loop
  if(fabs(lmd(9)) < epslmd) break;
   if(it == maxItr) { it = -1; break; }
	
  // update c
  c += lmd(9)/innerProduct(H[8], N * H[8]);
 } // renormalization loop end

 Hpn = H[8];
 if(determinant(Hpn) < 0.0) Hpn = -1.0 * Hpn; // just eliminate the sign ambiguity.
  eps2 = c/(1.0 - 4.0/double(d_nFeatures));
 dev = sqrt(eps2/(lmd(8) * double(d_nFeatures)))*H[7];

 Hpn = crdcnv(Hpn, imgCentroid1, imgCentroid2, f0);
 Hpn = Hpn * (1.0/Hpn(3,3));
	
 // compute scale for each image correspondance
 for (int i = 1; i <= d_nFeatures; ++i)
  sc.setElement(i, 1.0/(Hpn(3,1) * p1.getElement(1,i) 
                       + Hpn(3,2) * p1.getElement(2,i) 
					   + Hpn(3,3)));
 return it;
}


//==============================================================================
// ProjectiveHomography::computeVP
//==============================================================================
int ProjectiveHomography::computeVP(MatrixBase<> &p2, MatrixBase<> &p1, 
                                    Matrix<3, 3> &Hpn, VectorBase<> &sc)
{
 int n = p2.getNumColumns();

 // need atleast 8 points
 if(n < 8 ) {
  fprintf(stderr, "%s\n", "[computeVP] : Require atleast 8 feature points");
  return -1;
 }
 
 // form transformation matrices
 Matrix<3,3> vp_m, vp_mc;
 getSubMatrix(p1, 1, 1, vp_mc);
 getSubMatrix(p2, 1, 1, vp_m);
 
 if( (fabs(determinant(vp_m)) < 1e-5) || (fabs(determinant(vp_mc)) < 1e-5) ) {
  fprintf(stderr, "[computeVP] : Transformation matrices singular.\n");
  return (-1);
 }
 
 // do coordinate transformation - will undo this transformation in the end
 multiply3n(inverse(vp_m), p2, p2);
 multiply3n(inverse(vp_mc), p1, p1);

 // generate n!/(6(n-3)!) linear equations based on epipolar constraints
 int i,j,k;
 gsl_combination_init_first(d_gslcombos);
 int eqn = 0;
 do
 {
  i = gsl_combination_get(d_gslcombos,0) + 4;
  j = gsl_combination_get(d_gslcombos,1) + 4;
  k = gsl_combination_get(d_gslcombos,2) + 4;
  
  gsl_matrix_set(d_gslc, eqn, 0, 
   p2.getElement(3,i)*p2.getElement(3,j)*p2.getElement(2,k)*p1.getElement(1,k) 
   * (p1.getElement(1,j)*p1.getElement(2,i) - p1.getElement(1,i)*p1.getElement(2,j))
   + p2.getElement(3,i)*p2.getElement(3,k)*p2.getElement(2,j)*p1.getElement(1,j) 
   * (p1.getElement(1,i)*p1.getElement(2,k) - p1.getElement(1,k)*p1.getElement(2,i))
   + p2.getElement(3,j)*p2.getElement(3,k)*p2.getElement(2,i)*p1.getElement(1,i) 
   * (p1.getElement(1,k)*p1.getElement(2,j) - p1.getElement(1,j)*p1.getElement(2,k)));

  gsl_matrix_set(d_gslc, eqn, 1, 
   p2.getElement(3,i)*p2.getElement(3,j)*p2.getElement(1,k)*p1.getElement(2,k) 
   * (p1.getElement(1,i)*p1.getElement(2,j) - p1.getElement(1,j)*p1.getElement(2,i))
   + p2.getElement(3,i)*p2.getElement(3,k)*p2.getElement(1,j)*p1.getElement(2,j) 
   * (p1.getElement(1,k)*p1.getElement(2,i) - p1.getElement(1,i)*p1.getElement(2,k))
   + p2.getElement(3,j)*p2.getElement(3,k)*p2.getElement(1,i)*p1.getElement(2,i) 
   * (p1.getElement(1,j)*p1.getElement(2,k) - p1.getElement(1,k)*p1.getElement(2,j)));

  gsl_matrix_set(d_gslc, eqn, 2, 
   p2.getElement(2,i)*p2.getElement(2,k)*p2.getElement(3,j)*p1.getElement(1,j) 
   * (p1.getElement(1,i)*p1.getElement(3,k) - p1.getElement(1,k)*p1.getElement(3,i))
   + p2.getElement(2,i)*p2.getElement(2,j)*p2.getElement(3,k)*p1.getElement(1,k) 
   * (p1.getElement(1,j)*p1.getElement(3,i) - p1.getElement(1,i)*p1.getElement(3,j))
   + p2.getElement(2,j)*p2.getElement(2,k)*p2.getElement(3,i)*p1.getElement(1,i) 
   * (p1.getElement(1,k)*p1.getElement(3,j) - p1.getElement(1,j)*p1.getElement(3,k)));

  gsl_matrix_set(d_gslc, eqn, 3, 
   p2.getElement(1,i)*p2.getElement(1,k)*p2.getElement(3,j)*p1.getElement(2,j) 
   * (p1.getElement(2,i)*p1.getElement(3,k) - p1.getElement(2,k)*p1.getElement(3,i))
   + p2.getElement(1,i)*p2.getElement(1,j)*p2.getElement(3,k)*p1.getElement(2,k) 
   * (p1.getElement(2,j)*p1.getElement(3,i) - p1.getElement(2,i)*p1.getElement(3,j))
   + p2.getElement(1,j)*p2.getElement(1,k)*p2.getElement(3,i)*p1.getElement(2,i) 
   * (p1.getElement(2,k)*p1.getElement(3,j) - p1.getElement(2,j)*p1.getElement(3,k)));

  gsl_matrix_set(d_gslc, eqn, 4, 
   p2.getElement(2,j)*p2.getElement(2,k)*p2.getElement(1,i)*p1.getElement(3,i) 
   * (p1.getElement(1,j)*p1.getElement(3,k) - p1.getElement(1,k)*p1.getElement(3,j))
   + p2.getElement(2,i)*p2.getElement(2,k)*p2.getElement(1,j)*p1.getElement(3,j) 
   * (p1.getElement(1,k)*p1.getElement(3,i) - p1.getElement(1,i)*p1.getElement(3,k))
   + p2.getElement(2,i)*p2.getElement(2,j)*p2.getElement(1,k)*p1.getElement(3,k) 
   * (p1.getElement(1,i)*p1.getElement(3,j) - p1.getElement(1,j)*p1.getElement(3,i)));

  gsl_matrix_set(d_gslc, eqn, 5, 
   p2.getElement(1,j)*p2.getElement(1,k)*p2.getElement(2,i)*p1.getElement(3,i) 
   * (p1.getElement(2,j)*p1.getElement(3,k) - p1.getElement(2,k)*p1.getElement(3,j))
   + p2.getElement(1,i)*p2.getElement(1,k)*p2.getElement(2,j)*p1.getElement(3,j) 
   * (p1.getElement(2,k)*p1.getElement(3,i) - p1.getElement(2,i)*p1.getElement(3,k))
   + p2.getElement(1,i)*p2.getElement(1,j)*p2.getElement(2,k)*p1.getElement(3,k) 
   * (p1.getElement(2,i)*p1.getElement(3,j) - p1.getElement(2,j)*p1.getElement(3,i)));

  gsl_matrix_set(d_gslc, eqn, 6, 
   p2.getElement(1,i)*p2.getElement(2,k)*p2.getElement(3,j) 
   * (p1.getElement(1,k)*p1.getElement(2,j)*p1.getElement(3,i) 
   - p1.getElement(1,j)*p1.getElement(2,i)*p1.getElement(3,k))
   + p2.getElement(1,k)*p2.getElement(2,i)*p2.getElement(3,j) 
   * (p1.getElement(1,j)*p1.getElement(2,k)*p1.getElement(3,i) 
   - p1.getElement(1,i)*p1.getElement(2,j)*p1.getElement(3,k))
   + p2.getElement(1,i)*p2.getElement(2,j)*p2.getElement(3,k) 
   * (p1.getElement(1,k)*p1.getElement(2,i)*p1.getElement(3,j) 
   - p1.getElement(1,j)*p1.getElement(2,k)*p1.getElement(3,i))
   + p2.getElement(1,j)*p2.getElement(2,i)*p2.getElement(3,k) 
   * (p1.getElement(1,i)*p1.getElement(2,k)*p1.getElement(3,j) 
   - p1.getElement(1,k)*p1.getElement(2,j)*p1.getElement(3,i))
   + p2.getElement(1,k)*p2.getElement(2,j)*p2.getElement(3,i) 
   * (p1.getElement(1,j)*p1.getElement(2,i)*p1.getElement(3,k) 
   - p1.getElement(1,i)*p1.getElement(2,k)*p1.getElement(3,j))
   + p2.getElement(1,j)*p2.getElement(2,k)*p2.getElement(3,i) 
   * (p1.getElement(1,i)*p1.getElement(2,j)*p1.getElement(3,k) 
   - p1.getElement(1,k)*p1.getElement(2,i)*p1.getElement(3,j)));
  eqn += 1;
 }while(gsl_combination_next(d_gslcombos) == GSL_SUCCESS);
 

 // find eigen vector corres. to lowest eigen value for (d_gslc^T)d_gslc
 gsl_vector vp_eval1; Vector<7> eval1; GSLCompat_vector(&eval1, &vp_eval1);
 gsl_matrix vp_evec1; Matrix<7,7> evec1; GSLCompat_matrix(&evec1, &vp_evec1);
 gsl_matrix_transpose_memcpy(d_gslct, d_gslc);
 gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, d_gslct, d_gslc, 0.0, d_gslcaug);
 gsl_eigen_symmv(d_gslcaug, &vp_eval1, &vp_evec1, d_gslwork1);
 gsl_eigen_symmv_sort(&vp_eval1, &vp_evec1, GSL_EIGEN_SORT_ABS_ASC);

 // if rank(C) = 1, find Hpn now
 Matrix<3,3> g;
 g = unitMatrix<3>();

 if(fabs(eval1(6)) < 1e-5) 
 {
  g(1,1) = -gsl_matrix_get(d_gslc, 0, 4)/gsl_matrix_get(d_gslc, 0, 2); //c5/c3
  g(2,2) = -gsl_matrix_get(d_gslc, 0, 5)/gsl_matrix_get(d_gslc, 0, 3); //c6/c4
  g(3,3) = 1.0;
 }
 else
 {
  // Create XtX from eigen vector of CtC
  Matrix<10,3> X;
  Matrix<3,3> XtX; gsl_matrix vp_xtx; GSLCompat_matrix(&XtX, &vp_xtx);
  X = evec1(2,1), -evec1(1,1), 0         ,
      0         , -evec1(3,1),  evec1(1,1),
      evec1(4,1), 0          , -evec1(2,1),
      evec1(7,1), 0          , -evec1(1,1),
      evec1(5,1), 0          , -evec1(3,1),
      evec1(7,1), -evec1(3,1), 0          ,
      evec1(4,1), -evec1(7,1), 0          ,
      0         ,  evec1(6,1), -evec1(4,1),
      evec1(6,1), 0          , -evec1(7,1),
      evec1(6,1), -evec1(5,1), 0          ;
  XtX = transpose(X) * X;

  // find eigen vector corresponding to min eigen value of XtX.
  gsl_vector vp_eval2; Vector<3> eval2; GSLCompat_vector(&eval2, &vp_eval2);
  gsl_matrix vp_evec2; Matrix<3,3> evec2; GSLCompat_matrix(&evec2, &vp_evec2);
  gsl_eigen_symmv(&vp_xtx, &vp_eval2, &vp_evec2, d_gslwork2);
  gsl_eigen_symmv_sort(&vp_eval2, &vp_evec2, GSL_EIGEN_SORT_ABS_ASC);
 
  // Develop Hpn
  int sign;
  if( evec2(1,1) < 0 )
   sign = -1;
  else
   sign = 1;
  
  g(1,1) = sign * evec2(1,1);
  g(2,2) = sign * evec2(2,1);
  g(3,3) = sign * evec2(3,1);
 }
 
 Hpn = vp_m * (g * inverse(vp_mc));
 
 sc.setElement(1, Hpn(3,3)/g(1,1));
 sc.setElement(2, Hpn(3,3)/g(2,2));
 sc.setElement(3, Hpn(3,3)/g(3,3));
 
 Hpn = Hpn * (1.0/Hpn(3,3)); 

 // undo original transformation on pixels
 multiply3n(vp_m, p2, p2);
 multiply3n(vp_mc, p1, p1);

 return 0;
}

//==============================================================================
// decomposeHomography
//==============================================================================
int decomposeHomography(const Matrix<3,3> &Hen, motion_params_t &motion0, 
                        motion_params_t &motion1)
{
 //----------------------------------------------------------------------
 // This implementation follows the algorithm given in Shastry's book
 //----------------------------------------------------------------------
 Matrix<3,3> HtH;
 HtH = transpose(Hen) * Hen;
 Matrix<3,3> U, V;
 Vector<3> S, work;

 gsl_matrix gsl_u;
 gsl_matrix gsl_v;
 gsl_vector gsl_s;
 gsl_vector gsl_work;

 GSLCompat_matrix(&U, &gsl_u);
 GSLCompat_matrix(&V, &gsl_v);
 GSLCompat_vector(&S, &gsl_s);
 GSLCompat_vector(&work, &gsl_work);
 
 U = HtH;
	
 //SVD of homography matrix
 if( gsl_linalg_SV_decomp(&gsl_u, &gsl_v, &gsl_s, &gsl_work) )
  return -1;

#ifdef DEBUG
 std::cout << "[ProjectiveHomography::decompose(S)]: Singular values: " << transpose(S) 
           << std::endl;
#endif

 // degenerate case?
 if( S(1) - S(3) < 1e-5) {
  fprintf(stderr, "[ProjectiveHomography::decompose]: No motion or pure rotation case?\n");
  return -1;
 }

 double scaleSq = 1.0/S(2);
 double scale = sqrt(scaleSq);
 S *= scaleSq;
 
 Vector<3> tmp1, tmp2;
 double tmp3;
 Vector<3> u1, u2, v1, v2, v3;
 
 U.getSubMatrix(1,1,v1);
 U.getSubMatrix(1,2,v2);
 U.getSubMatrix(1,3,v3);

 tmp1 = v1 * sqrt(1.0 - S(3));
 tmp2 = v3 * sqrt(S(1) - 1.0);
 tmp3 = 1.0/sqrt(S(1) - S(3));
 
 u1 = tmp3 * (tmp1 + tmp2);
 u2 = tmp3 * (tmp1 - tmp2);

 Matrix<3,3> U1, U2, W1, W2;

 U1.setSubMatrix(1,1,v2);
 U1.setSubMatrix(1,2,u1);
 tmp1 = crossProduct(v2, u1);
 U1.setSubMatrix(1,3,tmp1);

 U2.setSubMatrix(1,1,v2);
 U2.setSubMatrix(1,2,u2);
 tmp1 = crossProduct(v2, u2);
 U2.setSubMatrix(1,3,tmp1);
 
 tmp1 = scale*Hen*v2;
 tmp2 = scale*Hen*u1;
 W1.setSubMatrix(1,1,tmp1);
 W1.setSubMatrix(1,2,tmp2);
 tmp2 = crossProduct(tmp1, tmp2);
 W1.setSubMatrix(1,3,tmp2);
 tmp2 = scale*Hen*u2;
 W2.setSubMatrix(1,1,tmp1);
 W2.setSubMatrix(1,2,tmp2);
 tmp2 = crossProduct(tmp1, tmp2);
 W2.setSubMatrix(1,3,tmp2);

 double sign = 1.0;
 
 motion0.normal = crossProduct(v2,u1);
 (motion0.normal(3) < 0)?(sign=-1.0):(sign=1.0);
 motion0.rotation = W1 * transpose(U1);
 motion0.normal = sign * motion0.normal;
 motion0.scaledTranslation = (scale*Hen - motion0.rotation)*motion0.normal;
 
 motion1.normal = crossProduct(v2,u2);
 (motion1.normal(3) < 0)?(sign=-1.0):(sign=1.0);
 motion1.rotation = W2 * transpose(U2);
 motion1.normal = sign * motion1.normal;
 motion1.scaledTranslation = (scale*Hen - motion1.rotation)*motion1.normal;

 return 0;
}


int decomposeHomography(const Matrix<3,3> &Hen, const Vector<3> &n, 
                                motion_params_t &motion)
{
 //----------------------------------------------------------------------
 // This implementation follows the algorithm given in Faugeras' book
 //----------------------------------------------------------------------
 Matrix<3,3> U;
 Matrix<3,3> V;
 Vector<3> n_d;
 Matrix<3,3> R_dash;
 Vector<3> D;
 Vector<3> x_dash, work;
 double sign;
 double d_dash;
 float st, ct; // sin(theta) and cos(theta). 'float' type to 
	       // avoid numerical problems
 double scaleFactor; // the 'd' in decomposition alg. p 290 faugeras

 //---------------------------------------------------------
 // Equations. See Faugeras for description of the algorithm
 // R_bar = sign.U.R_dash.V^T
 // x_f_bar = U.x_dash;
 // n_star = V.n_d;
 // d_star = sign.d_dash;
 //---------------------------------------------------------

 gsl_matrix gsl_u;
 gsl_matrix gsl_v;
 gsl_vector gsl_s;
 gsl_vector gsl_work;

 GSLCompat_matrix(&U, &gsl_u);
 GSLCompat_matrix(&V, &gsl_v);
 GSLCompat_vector(&D, &gsl_s);
 GSLCompat_vector(&work, &gsl_work);
 
 U = Hen;
	
 //SVD of homography matrix
 if( gsl_linalg_SV_decomp(&gsl_u, &gsl_v, &gsl_s, &gsl_work) )
  return -1;
	
#ifdef DEBUG
 std::cout << "[ProjectiveHomography::decompose(F)]: Singular values: " << transpose(D) 
           << std::endl;
#endif

 sign = determinant(U) * determinant(V);
 n_d = transpose(V) * n;

 // The object is always visible to camera, hence distance to the object plane 
 // is always positive. This determines sign of d_dash.
 scaleFactor = sign * D(2);
 if(scaleFactor >= 0) 
  d_dash = D(2);
 else {
  d_dash = -1 * D(2); 
  scaleFactor = -1 * scaleFactor;
 }
 
 // all equal singular values (pure rotation, or no motion)
 if( (fabs(D(1) - D(2)) <= 1e-5) && (fabs(D(2) - D(3)) <= 1e-5) ) {
  if(d_dash > 0) {
   R_dash = 1, 0, 0, 0, 1, 0, 0, 0, 1; // unit matrix
   x_dash = 0;
  } else {
   // R_dash = -I + 2n_d.n_d^T
   R_dash = 2 * n_d(1) * n_d(1) - 1, 2 * n_d(1) * n_d(2), 2 * n_d(1) * n_d(3),
	    2 * n_d(1) * n_d(2), 2 * n_d(2) * n_d(2) - 1, 2 * n_d(2) * n_d(3),
	    2 * n_d(1) * n_d(3), 2 * n_d(2) * n_d(3), 2 * n_d(3) * n_d(3) - 1;
   x_dash = (-2 * d_dash) * n_d(1), 0, (-2 * d_dash) * n_d(3);
  }
 } 
 else { // general case. handles two equal singular values also (translation parallel to normal)
 //if( (fabs(D(1) - D(2)) > 1e-5) && (fabs(D(2) - D(3)) > 1e-5) ) {
  if( d_dash > 0) {
   st = (D(1) - D(3)) * (n_d(1) * n_d(3)) / D(2);
   ct = (D(2) * D(2) + D(1) * D(3))/(D(2) * (D(1) + D(3)));
   R_dash = ct, 0, -1*st, 0, 1, 0, st, 0, ct;
   x_dash = (D(1) - D(3)) * n_d(1), 0, -1 * (D(1) - D(3)) * n_d(3);
  }else{
   st = (D(1) + D(3)) * (n_d(1) * n_d(3)) / D(2);
   ct = (D(1) * D(3) - D(2) * D(2))/(D(2) * (D(1) - D(3)));
   R_dash = ct, 0, st, 0, -1, 0, st, 0, -1*ct;
   x_dash = (D(1) + D(3)) * n_d(1), 0, (D(1) + D(3)) * n_d(3);
  }
 }

/*
 // two equal singular values (translation is normal to the plane) - CHECK for ERROR
 if( (fabs(D(1) - D(2)) <= 1e-5) || (fabs(D(2) - D(3)) <= 1e-5) ) {
  if(d_dash > 0) {
//   std::cout << "DDASH+" << std::endl;
   R_dash = 1, 0, 0, 0, 1, 0, 0, 0, 1;
   x_dash = (D(3) - D(1)) * n_d(1), 0, (D(3) - D(1)) * n_d(3);
  } else {
//   std::cout << "DDASH-" << std::endl;
   R_dash = -1, 0, 0, 0, -1, 0, 0, 0, 1;
   x_dash = (D(3) + D(1)) * n_d(1), 0, (D(3) + D(1)) * n_d(3);
  }
 }
*/

 // more outputs
 motion.rotation = sign * (U * (R_dash * transpose(V)));
 motion.scaledTranslation = (1.0/scaleFactor) * U * x_dash;
 motion.normal = n;

 return 0;
}


//===========================================================================
// scale
//===========================================================================
int scale(const MatrixBase<> &i1, const MatrixBase<> &i2, 
		  Vector<3> *x1, Vector<3> *x2,
          Vector<3> &ic1, Vector<3> &ic2, double &f0)
{
 int nPoints;

 nPoints = i1.getNumColumns();
 if ( i2.getNumColumns() != nPoints || i1.getNumRows() != 3 
	  || i2.getNumRows() != 3) {
  fprintf(stderr, "[scale]: Arguments have incorrect dimensions.\n");
  return -1;
 }
 
 // compute centroids
 ic1 = 0;
 ic2 = 0;

 for(int a = 1; a <= nPoints; ++a) {
  ic1(1) += i1.getElement(1, a);
  ic1(2) += i1.getElement(2, a);
  ic1(3) += i1.getElement(3, a);
  ic2(1) += i2.getElement(1, a);
  ic2(2) += i2.getElement(2, a);
  ic2(3) += i2.getElement(3, a); 
 }

 ic1 = ic1/double(nPoints);
 ic2 = ic2/double(nPoints);

 // search min and max values
 double i1xmin, i1xmax, i1ymin, i1ymax, i2xmin, i2xmax, i2ymin, i2ymax;
 i1xmin = i1xmax = i1.getElement(1,1); i1ymin = i1ymax = i1.getElement(2,1);
 i2xmin = i2xmax = i2.getElement(1,1); i2ymin = i2ymax = i2.getElement(2,1);

 for (int a = 2; a <= nPoints; ++a) {
  if(i1xmin > i1.getElement(1, a)) i1xmin = i1.getElement(1, a);
  if(i1xmax < i1.getElement(1, a)) i1xmax = i1.getElement(1, a);
  if(i1ymin > i1.getElement(2, a)) i1ymin = i1.getElement(2, a);
  if(i1ymax < i1.getElement(2, a)) i1ymax = i1.getElement(2, a);

  if(i2xmin > i2.getElement(1, a)) i2xmin = i2.getElement(1, a);
  if(i2xmax < i2.getElement(1, a)) i2xmax = i2.getElement(1, a);
  if(i2ymin > i2.getElement(2, a)) i2ymin = i2.getElement(2, a);
  if(i2ymax < i2.getElement(2, a)) i2ymax = i2.getElement(2, a);
 }

 f0 = i1xmax - i1xmin;
 if(f0 < (i1ymax-i1ymin)) f0 = i1ymax - i1ymin;
 if(f0 < (i2xmax-i2xmin)) f0 = i2xmax - i2xmin;
 if(f0 < (i2ymax-i2ymin)) f0 = i2ymax - i2ymin;
 if(f0 == 0.0) return 1;

 f0 *= 2.0;

 // compute scaled data
 for(int a = 1; a <= nPoints; ++a) {
  x1[a-1](1) = (ic1(1) - i1.getElement(1, a))/f0;
  x1[a-1](2) = (i1.getElement(2, a) - ic1(2))/f0;
  x1[a-1](3) = 1.0;

  x2[a-1](1) = (ic2(1) - i2.getElement(1, a))/f0;
  x2[a-1](2) = (i2.getElement(2, a) - ic2(2))/f0;
  x2[a-1](3) = 1.0;
 }

 return 0;
}

//===========================================================================
// crdcnv
//===========================================================================
Matrix<3,3> crdcnv(const Matrix<3,3> &H, const Vector<3> &ic1, const Vector<3> &ic2, double f0)
{
 Matrix<3,3> A, B, product;
 double normVal;
  
 A = 0;
 B = 0;

 A(1,1) = -1.0/f0;
 A(2,2) = 1.0/f0;
 A(1,3) = ic1(1)/f0;
 A(2,3) = -ic1(2)/f0;
 A(3,3) = 1.0;
  
 B(1,1) = -f0;
 B(2,2) = f0;
 B(1,3) = ic2(1);
 B(2,3) = ic2(2);
 B(3,3) = 1.0;
  
 product = B*H*A;
 normVal = norm(product);
 return (product/normVal);
}


//===========================================================================
// norm
//===========================================================================
double norm(const Matrix<3,3> &m)
{
 double n = 0;
 for(int i = 1; i <= 3; i++)
  for (int j = 1; j <= 3; j++)
   n += m(i,j)*m(i,j);
 return (sqrt(n));
}


//===========================================================================
// getSubMatrix
//===========================================================================
int getSubMatrix(const MatrixBase<> &s, int pr, int pc, MatrixBase<> &sm)
{
 int nr = s.getNumRows();
 int nc = s.getNumColumns();
 int snr = sm.getNumRows();
 int snc = sm.getNumColumns();
 
 if( (pr < 1) || (pr > nr) || (pc < 1) || (pc > nc) || (snc > (nc - pc + 1)) 
     || (snr > (nr - pr + 1)) ) {
  return -1;
 }

 int c;
 int r = pr;
 for (int sr = 1; sr <= snr; ++sr) {
  c = pc;
  for (int sc = 1; sc <= snc; ++sc) {
   sm.setElement(sr, sc, s.getElement(r,c));
   ++c;
  }
  ++r;
 }
 return 0;
}


//===========================================================================
// multiply3n
//===========================================================================
int ProjectiveHomography::multiply3n(const MatrixBase<> &lh, const MatrixBase<> &rh, MatrixBase<> &d)
{
 int r1 = lh.getNumRows();
 int c1 = lh.getNumColumns();
 int r2 = rh.getNumRows();
 int c2 = rh.getNumColumns();
 if( (c1 != r2) || (d.getNumRows() != r1) || (d.getNumColumns() != c2) ) {
  return -1;
 }
 
 double pe;
 int i = 0;
 for (int r = 1; r <= r1; ++r) {
  for (int c = 1; c <= c2; ++c) {
   pe = 0;
   for (int com = 1; com <= c1; ++com)
    pe += lh.getElement(r, com) * rh.getElement(com, c);
   d_multWork[i] = pe;
   ++i;
   //d.setElement(r, c, pe);
  }
 }
 // copy result
 i = 0;
 for(int r = 1; r <= r1; ++r)
  for(int c = 1; c <=c2; ++c) {
   d.setElement(r,c,d_multWork[i]);
   ++i;
  }
 return 0;
}

