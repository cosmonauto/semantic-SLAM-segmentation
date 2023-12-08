
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