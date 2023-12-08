
/*
Copyright 2011. All rights reserved.
Institute of Measurement and Control Systems
Karlsruhe Institute of Technology, Germany

This file is part of libviso2.
Authors: Andreas Geiger

libviso2 is free software; you can redistribute it and/or modify it under the
terms of the GNU General Public License as published by the Free Software
Foundation; either version 2 of the License, or any later version.

libviso2 is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
libviso2; if not, write to the Free Software Foundation, Inc., 51 Franklin
Street, Fifth Floor, Boston, MA 02110-1301, USA 
*/

#include "matrix_.h"
#include <cmath>

#define SWAP(a,b) {temp=a;a=b;b=temp;}
#define SIGN(a,b) ((b) >= 0.0 ? fabs(a) : -fabs(a))
static FLOAT sqrarg;
#define SQR(a) ((sqrarg=(a)) == 0.0 ? 0.0 : sqrarg*sqrarg)
static FLOAT maxarg1,maxarg2;
#define FMAX(a,b) (maxarg1=(a),maxarg2=(b),(maxarg1) > (maxarg2) ? (maxarg1) : (maxarg2))
static int32_t iminarg1,iminarg2;
#define IMIN(a,b) (iminarg1=(a),iminarg2=(b),(iminarg1) < (iminarg2) ? (iminarg1) : (iminarg2))


using namespace std;

Matrix_::Matrix_ () {
  m   = 0;
  n   = 0;
  val = 0;
}

Matrix_::Matrix_ (const int32_t m_,const int32_t n_) {
  allocateMemory(m_,n_);
}

Matrix_::Matrix_ (const int32_t m_,const int32_t n_,const FLOAT* val_) {
  allocateMemory(m_,n_);
  int32_t k=0;
  for (int32_t i=0; i<m_; i++)
    for (int32_t j=0; j<n_; j++)
      val[i][j] = val_[k++];
}

Matrix_::Matrix_ (const Matrix_ &M) {
  allocateMemory(M.m,M.n);
  for (int32_t i=0; i<M.m; i++)
    memcpy(val[i],M.val[i],M.n*sizeof(FLOAT));
}

Matrix_::~Matrix_ () {
  releaseMemory();
}

Matrix_& Matrix_::operator= (const Matrix_ &M) {
  if (this!=&M) {
    if (M.m!=m || M.n!=n) {
      releaseMemory();
      allocateMemory(M.m,M.n);
    }
    if (M.n>0)
      for (int32_t i=0; i<M.m; i++)
        memcpy(val[i],M.val[i],M.n*sizeof(FLOAT));
  }
  return *this;
}

void Matrix_::getData(FLOAT* val_,int32_t i1,int32_t j1,int32_t i2,int32_t j2) {
  if (i2==-1) i2 = m-1;
  if (j2==-1) j2 = n-1;
  int32_t k=0;
  for (int32_t i=i1; i<=i2; i++)
    for (int32_t j=j1; j<=j2; j++)
      val_[k++] = val[i][j];
}

Matrix_ Matrix_::getMat(int32_t i1,int32_t j1,int32_t i2,int32_t j2) {
  if (i2==-1) i2 = m-1;
  if (j2==-1) j2 = n-1;
  if (i1<0 || i2>=m || j1<0 || j2>=n || i2<i1 || j2<j1) {
    cerr << "ERROR: Cannot get submatrix [" << i1 << ".." << i2 <<
        "] x [" << j1 << ".." << j2 << "]" <<
        " of a (" << m << "x" << n << ") Matrix_." << endl;
    exit(0);
  }
  Matrix_ M(i2-i1+1,j2-j1+1);
  for (int32_t i=0; i<M.m; i++)
    for (int32_t j=0; j<M.n; j++)
      M.val[i][j] = val[i1+i][j1+j];
  return M;
}

void Matrix_::setMat(const Matrix_ &M,const int32_t i1,const int32_t j1) {
  if (i1<0 || j1<0 || i1+M.m>m || j1+M.n>n) {
    cerr << "ERROR: Cannot set submatrix [" << i1 << ".." << i1+M.m-1 <<
        "] x [" << j1 << ".." << j1+M.n-1 << "]" <<
        " of a (" << m << "x" << n << ") Matrix_." << endl;
    exit(0);
  }
  for (int32_t i=0; i<M.m; i++)
    for (int32_t j=0; j<M.n; j++)
      val[i1+i][j1+j] = M.val[i][j];
}

void Matrix_::setVal(FLOAT s,int32_t i1,int32_t j1,int32_t i2,int32_t j2) {
  if (i2==-1) i2 = m-1;
  if (j2==-1) j2 = n-1;
  if (i2<i1 || j2<j1) {
    cerr << "ERROR in setVal: Indices must be ordered (i1<=i2, j1<=j2)." << endl;
    exit(0);
  }
  for (int32_t i=i1; i<=i2; i++)
    for (int32_t j=j1; j<=j2; j++)
      val[i][j] = s;
}

void Matrix_::setDiag(FLOAT s,int32_t i1,int32_t i2) {
  if (i2==-1) i2 = min(m-1,n-1);
  for (int32_t i=i1; i<=i2; i++)
    val[i][i] = s;
}

void Matrix_::zero() {
  setVal(0);
}

Matrix_ Matrix_::extractCols (vector<int> idx) {
  Matrix_ M(m,idx.size());
  for (int32_t j=0; j<M.n; j++)
    if (idx[j]<n)
      for (int32_t i=0; i<m; i++)
        M.val[i][j] = val[i][idx[j]];
  return M;
}

Matrix_ Matrix_::eye (const int32_t m) {
  Matrix_ M(m,m);
  for (int32_t i=0; i<m; i++)
    M.val[i][i] = 1;
  return M;
}

void Matrix_::eye () {
  for (int32_t i=0; i<m; i++)
    for (int32_t j=0; j<n; j++)
      val[i][j] = 0;
  for (int32_t i=0; i<min(m,n); i++)
    val[i][i] = 1;
}

Matrix_ Matrix_::diag (const Matrix_ &M) {
  if (M.m>1 && M.n==1) {
    Matrix_ D(M.m,M.m);
    for (int32_t i=0; i<M.m; i++)
      D.val[i][i] = M.val[i][0];
    return D;
  } else if (M.m==1 && M.n>1) {
    Matrix_ D(M.n,M.n);
    for (int32_t i=0; i<M.n; i++)
      D.val[i][i] = M.val[0][i];
    return D;
  }
  cout << "ERROR: Trying to create diagonal Matrix_ from vector of size (" << M.m << "x" << M.n << ")" << endl;
  exit(0);
}

Matrix_ Matrix_::reshape(const Matrix_ &M,int32_t m_,int32_t n_) {
  if (M.m*M.n != m_*n_) {
    cerr << "ERROR: Trying to reshape a Matrix_ of size (" << M.m << "x" << M.n <<
            ") to size (" << m_ << "x" << n_ << ")" << endl;
    exit(0);
  }
  Matrix_ M2(m_,n_);
  for (int32_t k=0; k<m_*n_; k++) {
    int32_t i1 = k/M.n;
    int32_t j1 = k%M.n;
    int32_t i2 = k/n_;
    int32_t j2 = k%n_;
    M2.val[i2][j2] = M.val[i1][j1];
  }
  return M2;
}

Matrix_ Matrix_::rotMatX (const FLOAT &angle) {
  FLOAT s = sin(angle);
  FLOAT c = cos(angle);
  Matrix_ R(3,3);
  R.val[0][0] = +1;
  R.val[1][1] = +c;
  R.val[1][2] = -s;
  R.val[2][1] = +s;
  R.val[2][2] = +c;
  return R;
}

Matrix_ Matrix_::rotMatY (const FLOAT &angle) {
  FLOAT s = sin(angle);
  FLOAT c = cos(angle);
  Matrix_ R(3,3);
  R.val[0][0] = +c;
  R.val[0][2] = +s;
  R.val[1][1] = +1;
  R.val[2][0] = -s;
  R.val[2][2] = +c;
  return R;
}

Matrix_ Matrix_::rotMatZ (const FLOAT &angle) {
  FLOAT s = sin(angle);
  FLOAT c = cos(angle);
  Matrix_ R(3,3);
  R.val[0][0] = +c;
  R.val[0][1] = -s;
  R.val[1][0] = +s;
  R.val[1][1] = +c;
  R.val[2][2] = +1;
  return R;
}

Matrix_ Matrix_::operator+ (const Matrix_ &M) {
  const Matrix_ &A = *this;
  const Matrix_ &B = M;
  if (A.m!=B.m || A.n!=B.n) {
    cerr << "ERROR: Trying to add matrices of size (" << A.m << "x" << A.n <<
        ") and (" << B.m << "x" << B.n << ")" << endl;
    exit(0);
  }
  Matrix_ C(A.m,A.n);
  for (int32_t i=0; i<m; i++)
    for (int32_t j=0; j<n; j++)
      C.val[i][j] = A.val[i][j]+B.val[i][j];
  return C;
}

Matrix_ Matrix_::operator- (const Matrix_ &M) {
  const Matrix_ &A = *this;
  const Matrix_ &B = M;
  if (A.m!=B.m || A.n!=B.n) {
    cerr << "ERROR: Trying to subtract matrices of size (" << A.m << "x" << A.n <<
        ") and (" << B.m << "x" << B.n << ")" << endl;
    exit(0);
  }
  Matrix_ C(A.m,A.n);
  for (int32_t i=0; i<m; i++)
    for (int32_t j=0; j<n; j++)
      C.val[i][j] = A.val[i][j]-B.val[i][j];
  return C;
}

Matrix_ Matrix_::operator* (const Matrix_ &M) {
  const Matrix_ &A = *this;
  const Matrix_ &B = M;
  if (A.n!=B.m) {
    cerr << "ERROR: Trying to multiply matrices of size (" << A.m << "x" << A.n <<
        ") and (" << B.m << "x" << B.n << ")" << endl;
    exit(0);
  }
  Matrix_ C(A.m,B.n);
  for (int32_t i=0; i<A.m; i++)
    for (int32_t j=0; j<B.n; j++)
      for (int32_t k=0; k<A.n; k++)
        C.val[i][j] += A.val[i][k]*B.val[k][j];
  return C;
}

Matrix_ Matrix_::operator* (const FLOAT &s) {
  Matrix_ C(m,n);
  for (int32_t i=0; i<m; i++)
    for (int32_t j=0; j<n; j++)
      C.val[i][j] = val[i][j]*s;
  return C;
}

Matrix_ Matrix_::operator/ (const Matrix_ &M) {
  const Matrix_ &A = *this;
  const Matrix_ &B = M;
  
  if (A.m==B.m && A.n==B.n) {
    Matrix_ C(A.m,A.n);
    for (int32_t i=0; i<A.m; i++)
      for (int32_t j=0; j<A.n; j++)
        if (B.val[i][j]!=0)
          C.val[i][j] = A.val[i][j]/B.val[i][j];
    return C;
    
  } else if (A.m==B.m && B.n==1) {
    Matrix_ C(A.m,A.n);
    for (int32_t i=0; i<A.m; i++)
      for (int32_t j=0; j<A.n; j++)
        if (B.val[i][0]!=0)
          C.val[i][j] = A.val[i][j]/B.val[i][0];
    return C;
    
  } else if (A.n==B.n && B.m==1) {
    Matrix_ C(A.m,A.n);
    for (int32_t i=0; i<A.m; i++)
      for (int32_t j=0; j<A.n; j++)
        if (B.val[0][j]!=0)
          C.val[i][j] = A.val[i][j]/B.val[0][j];
    return C;
    
  } else {
    cerr << "ERROR: Trying to divide matrices of size (" << A.m << "x" << A.n <<
        ") and (" << B.m << "x" << B.n << ")" << endl;
    exit(0);
  } 
}

Matrix_ Matrix_::operator/ (const FLOAT &s) {
  if (fabs(s)<1e-20) {
    cerr << "ERROR: Trying to divide by zero!" << endl;
    exit(0);
  }
  Matrix_ C(m,n);
  for (int32_t i=0; i<m; i++)
    for (int32_t j=0; j<n; j++)
      C.val[i][j] = val[i][j]/s;
  return C;
}

Matrix_ Matrix_::operator- () {
  Matrix_ C(m,n);
  for (int32_t i=0; i<m; i++)
    for (int32_t j=0; j<n; j++)
      C.val[i][j] = -val[i][j];
  return C;
}

Matrix_ Matrix_::operator~ () {
  Matrix_ C(n,m);
  for (int32_t i=0; i<m; i++)
    for (int32_t j=0; j<n; j++)
      C.val[j][i] = val[i][j];
  return C;
}

FLOAT Matrix_::l2norm () {
  FLOAT norm = 0;
  for (int32_t i=0; i<m; i++)
    for (int32_t j=0; j<n; j++)
      norm += val[i][j]*val[i][j];
  return sqrt(norm);
}

FLOAT Matrix_::mean () {
  FLOAT mean = 0;
  for (int32_t i=0; i<m; i++)
    for (int32_t j=0; j<n; j++)
      mean += val[i][j];
  return mean/(FLOAT)(m*n);
}

Matrix_ Matrix_::cross (const Matrix_ &a, const Matrix_ &b) {
  if (a.m!=3 || a.n!=1 || b.m!=3 || b.n!=1) {
    cerr << "ERROR: Cross product vectors must be of size (3x1)" << endl;
    exit(0);
  }
  Matrix_ c(3,1);
  c.val[0][0] = a.val[1][0]*b.val[2][0]-a.val[2][0]*b.val[1][0];
  c.val[1][0] = a.val[2][0]*b.val[0][0]-a.val[0][0]*b.val[2][0];
  c.val[2][0] = a.val[0][0]*b.val[1][0]-a.val[1][0]*b.val[0][0];
  return c;
}

Matrix_ Matrix_::inv (const Matrix_ &M) {
  if (M.m!=M.n) {
    cerr << "ERROR: Trying to invert Matrix_ of size (" << M.m << "x" << M.n << ")" << endl;
    exit(0);
  }
  Matrix_ A(M);
  Matrix_ B = eye(M.m);
  B.solve(A);
  return B;
}

bool Matrix_::inv () {
  if (m!=n) {
    cerr << "ERROR: Trying to invert Matrix_ of size (" << m << "x" << n << ")" << endl;
    exit(0);
  }
  Matrix_ A(*this);
  eye();
  solve(A);
  return true;
}

FLOAT Matrix_::det () {
  
  if (m != n) {
    cerr << "ERROR: Trying to compute determinant of a Matrix_ of size (" << m << "x" << n << ")" << endl;
    exit(0);
  }
    
  Matrix_ A(*this);
  int32_t *idx = (int32_t*)malloc(m*sizeof(int32_t));
  FLOAT d = 1;
  A.lu(idx,d);
  for( int32_t i=0; i<m; i++)
    d *= A.val[i][i];
  free(idx);
  return d;
}

bool Matrix_::solve (const Matrix_ &M, FLOAT eps) {
  
  // substitutes
  const Matrix_ &A = M;
  Matrix_ &B       = *this;
  
  if (A.m != A.n || A.m != B.m || A.m<1 || B.n<1) {
    cerr << "ERROR: Trying to eliminate matrices of size (" << A.m << "x" << A.n <<
            ") and (" << B.m << "x" << B.n << ")" << endl;
    exit(0);
  }
  
  // index vectors for bookkeeping on the pivoting
  int32_t* indxc = new int32_t[m];
  int32_t* indxr = new int32_t[m];
  int32_t* ipiv  = new int32_t[m];
  
  // loop variables
  int32_t i, icol, irow, j, k, l, ll;
  FLOAT big, dum, pivinv, temp;
  
  // initialize pivots to zero
  for (j=0;j<m;j++) ipiv[j]=0;
  
  // main loop over the columns to be reduced
  for (i=0;i<m;i++) {
    
    big=0.0;
    
    // search for a pivot element
    for (j=0;j<m;j++)
      if (ipiv[j]!=1)
        for (k=0;k<m;k++)
          if (ipiv[k]==0)
            if (fabs(A.val[j][k])>=big) {
      big=fabs(A.val[j][k]);
      irow=j;
      icol=k;
            }
    ++(ipiv[icol]);
    
    // We now have the pivot element, so we interchange rows, if needed, to put the pivot
    // element on the diagonal. The columns are not physically interchanged, only relabeled.
    if (irow != icol) {
      for (l=0;l<m;l++) SWAP(A.val[irow][l], A.val[icol][l])
      for (l=0;l<n;l++) SWAP(B.val[irow][l], B.val[icol][l])
    }
    
    indxr[i]=irow; // We are now ready to divide the pivot row by the
    indxc[i]=icol; // pivot element, located at irow and icol.
    
    // check for singularity
    if (fabs(A.val[icol][icol]) < eps) {
      delete[] indxc;
      delete[] indxr;
      delete[] ipiv;
      return false;
    }
    
    pivinv=1.0/A.val[icol][icol];
    A.val[icol][icol]=1.0;
    for (l=0;l<m;l++) A.val[icol][l] *= pivinv;
    for (l=0;l<n;l++) B.val[icol][l] *= pivinv;
    
    // Next, we reduce the rows except for the pivot one
    for (ll=0;ll<m;ll++)
      if (ll!=icol) {
      dum = A.val[ll][icol];
      A.val[ll][icol] = 0.0;
      for (l=0;l<m;l++) A.val[ll][l] -= A.val[icol][l]*dum;
      for (l=0;l<n;l++) B.val[ll][l] -= B.val[icol][l]*dum;
      }
  }
  
  // This is the end of the main loop over columns of the reduction. It only remains to unscramble
  // the solution in view of the column interchanges. We do this by interchanging pairs of
  // columns in the reverse order that the permutation was built up.
  for (l=m-1;l>=0;l--) {
    if (indxr[l]!=indxc[l])
      for (k=0;k<m;k++)
        SWAP(A.val[k][indxr[l]], A.val[k][indxc[l]])
  }
  
  // success
  delete[] indxc;
  delete[] indxr;
  delete[] ipiv;
  return true;
}

// Given a Matrix_ a[1..n][1..n], this routine replaces it by the LU decomposition of a rowwise
// permutation of itself. a and n are input. a is output, arranged as in equation (2.3.14) above;
// indx[1..n] is an output vector that records the row permutation effected by the partial
// pivoting; d is output as ±1 depending on whether the number of row interchanges was even
// or odd, respectively. This routine is used in combination with lubksb to solve linear equations
// or invert a Matrix_.

bool Matrix_::lu(int32_t *idx, FLOAT &d, FLOAT eps) {
  
  if (m != n) {
    cerr << "ERROR: Trying to LU decompose a Matrix_ of size (" << m << "x" << n << ")" << endl;
    exit(0);
  }
  
  int32_t i,imax,j,k;
  FLOAT   big,dum,sum,temp;
  FLOAT* vv = (FLOAT*)malloc(n*sizeof(FLOAT)); // vv stores the implicit scaling of each row.
  d = 1.0;
  for (i=0; i<n; i++) { // Loop over rows to get the implicit scaling information.
    big = 0.0;
    for (j=0; j<n; j++)
      if ((temp=fabs(val[i][j]))>big)
        big = temp;
    if (big == 0.0) { // No nonzero largest element.
      free(vv);
      return false;
    }
    vv[i] = 1.0/big; // Save the scaling.
  }
  for (j=0; j<n; j++) { // This is the loop over columns of Crout’s method.
    for (i=0; i<j; i++) { // This is equation (2.3.12) except for i = j.
      sum = val[i][j];
      for (k=0; k<i; k++)
        sum -= val[i][k]*val[k][j];
      val[i][j] = sum;
    }
    big = 0.0; // Initialize the search for largest pivot element.
    for (i=j; i<n; i++) {
      sum = val[i][j];
      for (k=0; k<j; k++)
        sum -= val[i][k]*val[k][j];
      val[i][j] = sum;
      if ( (dum=vv[i]*fabs(sum))>=big) {
        big  = dum;