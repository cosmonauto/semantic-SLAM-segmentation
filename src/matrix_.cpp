
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