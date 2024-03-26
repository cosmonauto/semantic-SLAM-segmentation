
/*
   IRTES-SET laboratory
   Authors: You Li (liyou026@gmail.com)
   Descirption: This a sample code of my PhD works
*/

#include "vo_stereo.hpp"

using namespace std;

VisualOdometryStereo::VisualOdometryStereo (parameters param) : param(param), VisualOdometry(param)
{
}

VisualOdometryStereo::~VisualOdometryStereo() {
}


bool VisualOdometryStereo::Process(QuadFeatureMatch& quadmatcher)
{

    quadmatches.clear();
    int num = quadmatcher.quadmatches.size();

    //copy to visual odometry
    for(int i = 0; i < num; i++)
    {
        pmatch tmp;
        tmp.u1c = quadmatcher.quadmatches[i].u1c;
        tmp.v1c = quadmatcher.quadmatches[i].v1c;

        tmp.u2c = quadmatcher.quadmatches[i].u2c;
        tmp.v2c = quadmatcher.quadmatches[i].v2c;

        tmp.u1p = quadmatcher.quadmatches[i].u1p;
        tmp.v1p = quadmatcher.quadmatches[i].v1p;

        tmp.u2p = quadmatcher.quadmatches[i].u2p;
        tmp.v2p = quadmatcher.quadmatches[i].v2p;

        quadmatches.push_back(tmp);
    }

    return updateMotion();
}

vector<double> VisualOdometryStereo::estimateMotion (std::vector<pmatch> &quadmatches) {

  // return value
  bool success = true;

  // compute minimum distance for RANSAC samples
  double width=0,height=0;

  for (vector<pmatch>::iterator it=quadmatches.begin(); it!=quadmatches.end(); it++) {
    if (it->u1c>width)  width  = it->u1c;
    if (it->v1c>height) height = it->v1c;
  }

  // get number of matches
  int N  = quadmatches.size();
  if (N<6)
    return vector<double>();

  // allocate dynamic memory
  X          = new double[N];
  Y          = new double[N];
  Z          = new double[N];
  J          = new double[4*N*6];
  p_predict  = new double[4*N];
  p_observe  = new double[4*N];
  p_residual = new double[4*N];

  // project matches of previous image into 3d
  for (int i=0; i<N; i++) {
    double d = max(quadmatches[i].u1p - quadmatches[i].u2p,1.0f);
    X[i] = (quadmatches[i].u1p-param.calib.cu)*param.base/d;
    Y[i] = (quadmatches[i].v1p-param.calib.cv)*param.base/d;
    Z[i] = param.calib.f*param.base/d;
  }

  // loop variables
  vector<double> tr_delta;
  vector<double> tr_delta_curr;
  tr_delta_curr.resize(6);

  // clear parameter vector
  inliers.clear();

  // initial RANSAC estimate
  for (int k=0;k<param.ransac_iters;k++) {

    // draw random sample set
    vector<int> active = getRandomSample(N,3);

    // clear parameter vector
    for (int i=0; i<6; i++)
      tr_delta_curr[i] = 0;

    // minimize reprojection errors
    VisualOdometryStereo::result result = UPDATED;
    int iter=0;
    while (result==UPDATED) {
      result = updateParameters(quadmatches,active,tr_delta_curr,1,1e-6);
      if (iter++ > 20 || result==CONVERGED)
        break;
    }

    // overwrite best parameters if we have more inliers
    if (result!=FAILED) {
      vector<int> inliers_curr = getInlier(quadmatches,tr_delta_curr);
      if (inliers_curr.size()>inliers.size()) {
        inliers = inliers_curr;
        tr_delta = tr_delta_curr;
      }
    }
  }

  // final optimization (refinement)
  if (inliers.size()>=6) {
    int iter=0;
    VisualOdometryStereo::result result = UPDATED;
    while (result==UPDATED) {
      result = updateParameters(quadmatches,inliers,tr_delta,1,1e-8);
      if (iter++ > 100 || result==CONVERGED)
        break;
    }

    // not converged
    if (result!=CONVERGED)
      success = false;

  // not enough inliers
  } else {
    success = false;
  }

//  //allocate the inlier matches and outlier matches
  getInOutMatches(quadmatches,inliers);


  // release dynamic memory
  delete X;delete Y;
  delete Z;delete J;
  delete p_predict;
  delete p_observe;
  delete p_residual;

  // parameter estimate succeeded?
  if (success) return tr_delta;
  else         return vector<double>();
}





vector<int> VisualOdometryStereo::getInlier(std::vector<pmatch>& quadmatches,vector<double> &tr) {

  // mark all observations active
  vector<int> active;
  for (int i=0; i<(int)quadmatches.size(); i++)
    active.push_back(i);

  // extract observations and compute predictions
  computeObservations(quadmatches,active);
  computeResidualsAndJacobian(tr,active);

  // compute inliers
  vector<int> inliers;
  for (int i=0; i<(int)quadmatches.size(); i++)
    if (pow(p_observe[4*i+0]-p_predict[4*i+0],2)+pow(p_observe[4*i+1]-p_predict[4*i+1],2) +
        pow(p_observe[4*i+2]-p_predict[4*i+2],2)+pow(p_observe[4*i+3]-p_predict[4*i+3],2) < param.inlier_threshold*param.inlier_threshold)
      inliers.push_back(i);
  return inliers;
}




void VisualOdometryStereo::getInOutMatches(std::vector<pmatch>& quadmatches, vector<int>& inliers)
{
  int numMatched = quadmatches.size();

  quadmatches_inlier.clear();
  quadmatches_outlier.clear();

  for(int i = 0; i < numMatched; i++)
  {
    if(std::find(inliers.begin(),inliers.end(),i)!=inliers.end())
    {
      quadmatches_inlier.push_back(quadmatches[i]);
    }
    else
    {
      quadmatches_outlier.push_back(quadmatches[i]);
    }

  }

}

VisualOdometryStereo::result VisualOdometryStereo::updateParameters(std::vector<pmatch>& quadmatches,vector<int> &active,
                                                                       vector<double> &tr,double step_size,double eps)
{

  // we need at least 3 observations
  if (active.size()<3)
    return FAILED;
