/* \Lane estimate:sub image windomm fusion using EKF.  
 *      \TrackLanes.cpp
 * \Copyright (C) 2019 Sunny Tseng
 *
 * 
 */
#include "TrackLanes.h"
using namespace std;

namespace LaneDetector
{
void InitLaneKalmanFilter(cv::KalmanFilter &laneKalmanFilter, cv::Mat &laneKalmanMeasureMat, int &laneKalmanIdx, float dt)
{
  printf("INIT EKF");
  laneKalmanFilter.transitionMatrix = (cv::Mat_<float>(8, 8) << 1, 0, dt, 0, 0, 0, 0, 0, // x_top_left
                                       0, 1, 0, dt, 0, 0, 0, 0,                          // y_top_left
                                       0, 0, 1, 0, 0, 0, 0, 0,                           // vx_top_left
                                       0, 0, 0, 1, 0, 0, 0, 0,                           // vy_top_left
                                       0, 0, 0, 0, 1, 0, dt, 0,                          // x_bottom_right
                                       0, 0, 0, 0, 0, 1, 0, dt,                          // y_bottom_right
                                       0, 0, 0, 0, 0, 0, 1, 0,                           // vx_bottom_right
                                       0, 0, 0, 0, 0, 0, 0, 1);                          // v_y_bottom_right

  cv::setIdentity(laneKalmanFilter.measurementMatrix);                          //A
  cv::setIdentity(laneKalmanFilter.processNoiseCov, cv::Scalar::all(1e-5));     //Q
  cv::setIdentity(laneKalmanFilter.measurementNoiseCov, cv::Scalar::all(1e-1)); //R
  cv::setIdentity(laneKalmanFilter.errorCovPost, cv::Scalar::all(1));           //P

  cv::randn(laneKalmanFilter.statePost, cv::Scalar::all(0), cv::Scalar::all(0.1));

  //Reset the State and Measure Mat
  laneKalmanMeasureMat = cv::Mat::zeros(8, 1, CV_32F);
  std::cout << "Reset Measure >> " << std::endl;

  laneKalmanIdx = 0;
}
const int state_size =4; //Also change CvParticle2D32f,Particle  in LaneDetector.hh
bool ParticleCmp(const Particle &p1, const Particle &p2)
{
  return p1.weight > p2.weight;
}

void mcvDrawPoints(CvMat *image, CvPoint2D32f p[state_size], CvScalar color, int width)
{
  //put the control points with circles
  for (int i = 0; i < state_size; i++)
    cvCircle(image, cvPointFrom32f(p[i]), width, color, -1);
}
void CalculateSample(const CvMat *im, LaneDetectorConf *lineConf, LaneState *samples, LineState *state)
{
  // y is not change, 5, 35, 65 ,100

  Particle p;
  samples->ipmSplines.clear();
  samples->ipmBoxes.clear();
  samples->samples.clear();
  samples->Splines.clear();
  samples->ipmBoxes = state->ipmBoxes;
  samples->ipmSplines = state->ipmSplines;
  //if(state->ipmSplines.size())
  //cout << "\nDebug" <<state->ipmSplines[0].tangent;
  for (int i = 0; i < (int)state->ipmSplines.size(); i++)
  {

    //extend spline

    CvMat *points = mcvEvalBezierSpline(state->ipmSplines[i], .005);
    CvMat *ps = mcvExtendPoints_lsTangent(im, points,
                                lineConf->extendIPMAngleThreshold,
                                lineConf->extendIPMMeanDirAngleThreshold,
                                lineConf->extendIPMLinePixelsTangent,
                                lineConf->extendIPMLinePixelsNormal,
                                lineConf->extendIPMContThreshold,
                                lineConf->extendIPMDeviationThreshold,
                                cvRect(0, lineConf->extendIPMRectTop,
                                       im->width - 1,
                                       lineConf->extendIPMRectBottom - lineConf->extendIPMRectTop),state->ipmSplines[i],
                                false);

    //refit
    float diff_min[state_size];
    std::fill(diff_min, diff_min + state_size, 1000);
    Spline spline = mcvFitBezierSpline(ps, lineConf->ransacSplineDegree);

    int ind = 0;
    for (int l = 0; l < state_size; l++){
      float diff;
      for (int j = ind; j < ps->height; ++j)
      {
        CvPoint2D32f p0 = cvPoint2D32f(CV_MAT_ELEM(*ps, float, j, 0),
                                      CV_MAT_ELEM(*ps, float, j, 1));
            diff = abs(p0.y - 40 - l * (im->height-50) / (state_size-1));
            if (diff < diff_min[l])
            {
              p.points[l].x = p0.x;
              diff_min[l] = diff;
              if(diff< 0.005 || j > ps->height/4*(l+1)){
                ind = j;
                break;
              }
            }
            
      }

    }
    for (int l = 0; l < state_size; l++)
    {
      p.points[l].y =40 + l * (im->height-50)  / (state_size-1);
    }
    
    samples->samples.push_back(p);
    //cout << "\nlabel:" << state->ipmSplines[i].line.startPoint.x;
    //cout << "\n measurment" << p.points[0].x << "\t"<<p.points[1].x << "\t"<<p.points[2].x << "\t"<<p.points[3].x ;
  }
}

//1226//
/*
void CalculateSample(const CvMat *im, LaneDetectorConf *lineConf, LaneState *samples, LineState *state)
{
  // y is not change, 5, 35, 65 ,100

  Particle p;
  samples->ipmSplines.clear();
  samples->ipmBoxes.clear();
  samples->samples.clear();
  samples->Splines.clear();
  samples->ipmBoxes = state->ipmBoxes;
  samples->ipmSplines = state->ipmSplines;

  for (int i = 0; i < (int)state->ipmSplines.size(); i++)
  {

    //extend spline

    CvMat *points = mcvEvalBezierSpline(state->ipmSplines[i], .01);
    CvMat *ps = mcvExtendPoints(im, points,
                                lineConf->extendIPMAngleThreshold,
                                lineConf->extendIPMMeanDirAngleThreshold,
                                lineConf->extendIPMLinePixelsTangent,
                                lineConf->extendIPMLinePixelsNormal,
                                lineConf->extendIPMContThreshold,
                                lineConf->extendIPMDeviationThreshold,
                                cvRect(0, lineConf->extendIPMRectTop,
                                       im->width - 1,
                                       lineConf->extendIPMRectBottom - lineConf->extendIPMRectTop),
                                false);
    //SHOW_MAT(points, "points of the lane:");
    //cout <<"\t"<< CV_MAT_ELEM(*ps, float, 1, 1) << "\t" << ps-> width;
    int step = (ps->height - 20) / (state_size - 1);// cout << step <<"(((\t";
    //refit splin
    float diff_min[state_size];
    std::fill(diff_min, diff_min + state_size, 1000);
    Spline spline = mcvFitBezierSpline(ps, lineConf->ransacSplineDegree);
    for (int j = 1; j < ps->height; ++j)
    {
      CvPoint2D32f p0 = cvPoint2D32f(CV_MAT_ELEM(*ps, float, j, 0),
                                     CV_MAT_ELEM(*ps, float, j, 1));
      CvPoint2D32f pp[state_size];
      std::fill(pp, pp + state_size, p0);
      float diff[state_size];
      for (int l = 0; l < state_size; l++)
      {
        diff[l] = abs(pp[l].y - 40 - l * (im->height-50) / (state_size-1));
        if (diff[l] < diff_min[l])
        {
          p.points[l].x = pp[l].x;
          diff_min[l] = diff[l];
        }
      }
    }
    for (int l = 0; l < state_size; l++)
    {
      p.points[l].y =40 + l * (im->height-60)  / (state_size-1);
      //cout << p.points[l].y << '\n';
    }
    
    samples->samples.push_back(p);
  }
}
*/
void CalculateSample_init(const CvMat *im, LaneDetectorConf *lineConf, LaneState *samples, LineState *state)
{
  // y is not change, 5, 35, 65 ,100

  Particle p;
  samples->ipmSplines.clear();
  samples->ipmBoxes.clear();
  samples->samples.clear();
  samples->Splines.clear();
  //samples->ipmBoxes = state->ipmBoxes;
  samples->ipmSplines = state->bestSplines;
  //if(state->ipmSplines.size())
  //cout << "\nDebug" <<state->ipmSplines[0].tangent;
  for (int i = 0; i < (int)state->bestSplines.size(); i++)
  {

    //extend spline

    CvMat *points = mcvEvalBezierSpline(state->bestSplines[i], .005);
    CvMat *ps = mcvExtendPoints_lsTangent(im, points,
                                lineConf->extendIPMAngleThreshold,
                                lineConf->extendIPMMeanDirAngleThreshold,
                                lineConf->extendIPMLinePixelsTangent,
                                lineConf->extendIPMLinePixelsNormal,
                                lineConf->extendIPMContThreshold,
                                lineConf->extendIPMDeviationThreshold,
                                cvRect(0, lineConf->extendIPMRectTop,
                                       im->width - 1,
                                       lineConf->extendIPMRectBottom - lineConf->extendIPMRectTop),state->ipmSplines[i],
                                false);

    //refit
    float diff_min[state_size];
    std::fill(diff_min, diff_min + state_size, 1000);
    Spline spline = mcvFitBezierSpline(ps, lineConf->ransacSplineDegree);

    int ind = 0;
    for (int l = 0; l < state_size; l++){
      float diff;
      for (int j = ind; j < ps->height; ++j)
      {
        CvPoint2D32f p0 = cvPoint2D32f(CV_MAT_ELEM(*ps, float, j, 0),
                                      CV_MAT_ELEM(*ps, float, j, 1));
            diff = abs(p0.y - 40 - l * (im->height-50) / (state_size-1));
            if (diff < diff_min[l])
            {
              p.points[l].x = p0.x;
              diff_min[l] = diff;
              if(diff< 0.005 || j > ps->height/4*(l+1)){
                ind = j;
                break;
              }
            }
            
      }

    }
    for (int l = 0; l < state_size; l++)
    {
      p.points[l].y =40 + l * (im->height-50)  / (state_size-1);
    }
    
    samples->samples.push_back(p);
    //cout << "\nlabel:" << state->ipmSplines[i].line.startPoint.x;
    //cout << "\n measurment" << p.points[0].x << "\t"<<p.points[1].x << "\t"<<p.points[2].x << "\t"<<p.points[3].x ;
  }
}
double SIGMA0 = 2;
double SIGMA1 = 0.005;
double SIGMA2 = 0.00001;


void InitPF(LaneState *measurements, LaneState *pre_state, float num)
{

  cv::RNG rng;
  CvParticle2D32f ps_;

  
  pre_state->ipmSplines.clear();
  pre_state->samples.clear();
  pre_state->Splines.clear();

  pre_state->ipmSplines = measurements->ipmSplines;
  pre_state->samples = measurements->samples;
  for (int i = 0; i < (int)pre_state->samples.size(); i++)
  { //number of lane

    for (int k = 0; k < num; k++)
    {
      pre_state->samples[i].weight.push_back(1 / num);
      for (int j = 0; j < state_size; j++)
      {
        ps_.p[j].x = pre_state->samples[i].points[j].x + rng.gaussian(SIGMA0);
        ps_.p[j].y = pre_state->samples[i].points[j].y;
      }
      pre_state->samples[i].ps.push_back(ps_);
    }
  }

}
/*
void InitPF(LaneState *measurements, LaneState *state, float num)
{
  cv::RNG rng;
  CvParticle2D32f ps_;
  vector<float> scores;
  //cout << measurements->ipmSplines.size();
  //for (int i = 0; i < (int)measurements->ipmSplines.size(); i++)
  // SHOW_SPLINE (measurements->ipmSplines[i]);
  for (int i = 0; i < (int)measurements->ipmSplines.size(); i++)
    scores.push_back(measurements->ipmSplines[i].score);

  mcvGroupMeasurement(measurements->ipmSplines,scores,measurements->samples,measurements->ipmBoxes);
  
  state->ipmBoxes.clear();
  state->ipmSplines.clear();
  state->samples.clear();
  state->Splines.clear();
  //state->ipmBoxes = measurements->ipmBoxes;
  state->ipmSplines = measurements->ipmSplines;
  state->samples = measurements->samples;

  for (int i = 0; i < (int)state->samples.size(); i++)
  { //number of lane

    for (int k = 0; k < num; k++)
    {
      state->samples[i].weight.push_back(1 / num);
      for (int j = 0; j < state_size; j++)
      {
        ps_.p[j].x = state->samples[i].points[j].x + rng.gaussian(SIGMA0);
        ps_.p[j].y = state->samples[i].points[j].y;
      }
      state->samples[i].ps.push_back(ps_);
    }
  }

}
*/
void ResetPF(LaneState *measurements, LaneState *state, float num, int ind, int m_ind)
{
  cv::RNG rng;
  CvParticle2D32f ps_;

  state->ipmBoxes[ind] = measurements->ipmBoxes[m_ind];
  state->ipmSplines[ind] = measurements->ipmSplines[m_ind];
  state->samples[ind] = measurements->samples[m_ind];
  state->samples[ind].ps.clear();
  for (int k = 0; k < num; k++)
  {
    state->samples[ind].weight.push_back(1 / num);
    for (int j = 0; j < state_size; j++)
    {
      ps_.p[j].x = state->samples[ind].points[j].x + rng.gaussian(SIGMA0);
      ps_.p[j].y = state->samples[ind].points[j].y;
    }
    state->samples[ind].ps.push_back(ps_);
  }

  /*try Mat
  cv::Mat mat(2, 4, CV_64FC1);
  double mean = 0.0;
  double stddev = 500.0 / 3.0; // 99.7% of values will be inside [-500, +500] interval
  randn(mat, cv::Scalar(mean), cv::Scalar(stddev));
  cout << mat;*/
}

double SIGMA5 = 0.01;//0.01;//30point 0.0001;
double SIGMA6 =4;//6 ;//30point 1.5;
void laneEvalModel(Particle* sample,float motionV,float motionW, LaneDetectorConf *lineConf){
  cv::RNG rng((unsigned)time(NULL));
  for (int k = 0; k < sample->ps.size(); k++)
  {
    
    
    for(int i=0;i<state_size;i++)
    {
      float n = rng.gaussian(4);
      float v =rng.gaussian(0.004);
      float x_k =  sample->ps[k].p[i].x;
      float y_k = lineConf->ipmHeight- sample->ps[k].p[i].y;
      sample->ps[k].p[i].x =x_k*cos(motionW)- y_k*sin(motionW)+
                            (motionV)*sin(motionW/2);

      sample->ps[k].p[i].y =lineConf->ipmHeight-(x_k*sin(motionW)+ y_k*cos(motionW)-
                            (motionV)*cos(motionW/2));
                            //5*(motionW)* (lineConf->ipmHeight-sample->ps[k].p[i].y);

      sample->ps[k].p[i].x +=n;
      //sample->ps[k].p[i].y -= (motionV*0.01+v);
    }

    
  }
}

void prediction(const CvMat *im,LaneState *prestate, LaneState *state,
                float motionV,float motionW, LaneDetectorConf *lineConf,const IPMInfo *ipmInfo)
{

  CvParticle2D32f ps_;


  state->ipmBoxes.clear();
  state->ipmSplines.clear();
  state->samples.clear();
  state->Splines.clear();
  state->ipmBoxes = prestate->ipmBoxes;
  state->ipmSplines = prestate->ipmSplines;
  state->samples = prestate->samples;

  for (int i = 0; i < (int)prestate->samples.size(); i++)
  {
    //cout << state->ipmSplines[i].noUpdate<<"updatetime\n";
    if ( state->ipmSplines[i].noUpdate>2)
    {
      laneEvalModel(&(state->samples[i]),0,0,lineConf);
      state->ipmSplines[i].noUpdate=0;
      continue;
    } 
    laneEvalModel(&(state->samples[i]),motionV,motionW,lineConf);

    // fit new spline and find out the point on certain y
    //*** Warnning: Take too long*///
    for (int k = 1; k <prestate->samples[i].ps.size(); ++k)
    {
      float diff_min[state_size];
      std::fill(diff_min, diff_min + state_size, 1000);
      CvMat *m = CvParticle2D32f2Mat(state->samples[i].ps[k]);
      //SHOW_MAT(m);
      Spline spline = mcvFitBezierSpline(m,lineConf->ransacSplineDegree);
      CvMat *points = mcvEvalBezierSpline(spline, .01); //.005
      CvMat *ps = mcvExtendPoints(im, points,
                                lineConf->extendIPMAngleThreshold,
                                lineConf->extendIPMMeanDirAngleThreshold,
                                lineConf->extendIPMLinePixelsTangent,
                                lineConf->extendIPMLinePixelsNormal,
                                lineConf->extendIPMContThreshold,
                                lineConf->extendIPMDeviationThreshold,
                                cvRect(0, lineConf->extendIPMRectTop,
                                       im->width - 1,
                                       lineConf->extendIPMRectBottom - lineConf->extendIPMRectTop),false);

      for (int j = 1; j < ps->height; ++j)
      {
        CvPoint2D32f p0 = cvPoint2D32f(CV_MAT_ELEM(*ps, float, j, 0),
                                      CV_MAT_ELEM(*ps, float, j, 1));
        CvPoint2D32f pp[state_size];
        std::fill(pp, pp + state_size, p0);
        float diff[state_size];
        for (int l = 0; l < state_size; l++)
        {
          state->samples[i].ps[k].p[l].y = 40 + l * (im->height-50) / (state_size-1);
          diff[l] = abs(pp[l].y - 40 - l * (im->height-50) / (state_size-1));
          if (diff[l] < diff_min[l])
          {
            state->samples[i].ps[k].p[l].x = pp[l].x;
            diff_min[l] = diff[l];
          }

        }
         
      }

      int ind =0;
      for (int l = 0; l < state_size; l++){
        float diff;
        state->samples[i].ps[k].p[l].y = 40 + l * (im->height-50) / (state_size-1);
        for (int j = ind; j < ps->height; ++j)
        {
          CvPoint2D32f p0 = cvPoint2D32f(CV_MAT_ELEM(*ps, float, j, 0),
                                      CV_MAT_ELEM(*ps, float, j, 1));
          diff = abs(p0.y - 40 - l * (im->height-50) / (state_size-1));
          if (diff < diff_min[l])
          {
              state->samples[i].ps[k].p[l].x = p0.x;
              diff_min[l] = diff;
              if(diff< 0.005 || j < ps->height/4*(l+1)){
                ind = j;
                break;
              }
            }
            
      }

    }

   }

  }

}
//12/25// HAL lane model
/*void prediction(LaneState *prestate, LaneState *state,
                float motion,LaneDetectorConf *LineConf,const IPMInfo *ipmInfo)
{
  float y_[]={40,100,160,220};
  CvMat y = cvMat(1, 4, CV_32FC1, y_);
  CvMat *e_ = cvCreateMat(1, 4, CV_32FC1);
  //e_ =mcvEvalBezierSplinediff(prestate->ipmSplines[0], &y,LineConf);

  CvParticle2D32f ps_;

  float e[state_size];

  state->ipmBoxes.clear();
  state->ipmSplines.clear();
  state->samples.clear();
  state->Splines.clear();
  state->ipmBoxes = prestate->ipmBoxes;
  state->ipmSplines = prestate->ipmSplines;
  state->samples = prestate->samples;

  for (int i = 0; i < (int)prestate->samples.size(); i++)
  {
    e_ =mcvEvalBezierSplinediff(prestate->ipmSplines[i], &y,LineConf);
    for(int i=0;i<state_size;i++)
    {
      e[i] = CV_MAT_ELEM(*e_, float, 0, i)*ipmInfo->xScale;   
    }
    e[0] -= motion*(LineConf->ipmHeight- y_[3]);
    e[1]-=motion;
    //cout << "\n$$$$$$" <<  e[0]<< "\t" << e[1]<<"\t"<< e[2]<<"\t"<<e[3]<<"\t"<<ipmInfo->xScale;

    for (int k = 0; k < prestate->samples[i].ps.size(); k++)
    {
      e_ =mcvEvalBezierSplinediff(prestate->ipmSplines[i], &y,LineConf);
      for(int i=0;i<state_size;i++)
        e[i] = CV_MAT_ELEM(*e_, float, 0, i)*ipmInfo->xScale;   
      e[0] -= motion*(LineConf->ipmHeight- y_[3]);
      e[1]-=motion;

      state->samples[i].ps[k].p[3].x += e[0];
      state->samples[i].ps[k].p[2].x = state->samples[i].ps[k].p[2].x + 
                                        e[0]+e[1]*60;
      state->samples[i].ps[k].p[1].x = state->samples[i].ps[k].p[1].x + 
                                        e[0]+e[1]*120+
                                        e[2]*60;
     state->samples[i].ps[k].p[0].x = state->samples[i].ps[k].p[0].x + 
                                        e[0]+e[1]*180+
                                        e[2]*120+
                                        e[3]*60;

    //for (int j = state_size-1 ;j >-1 ; j--)
    //  {
    //    state->samples[i].ps[k].p[j].x += (e[0]);
         
    //    for (int l = state_size-j-1; l >0 ; l--)
     //   {
    //      state->samples[i].ps[k].p[j].x += e[l] * (prestate->samples[i].points[state_size-l].y - prestate->samples[i].points[j].y);

    //    }
    //
    //  }//end for
      

    }
  }
}*/
//12/20//
/*
void prediction(LaneState *prestate, LaneState *state,float motion)
{
  cv::RNG rng((unsigned)time(NULL));
  CvParticle2D32f ps_;

  float e[state_size];

  state->ipmBoxes.clear();
  state->ipmSplines.clear();
  state->samples.clear();
  state->Splines.clear();
  state->ipmBoxes = prestate->ipmBoxes;
  state->ipmSplines = prestate->ipmSplines;
  state->samples = prestate->samples;
  cout << motion << "@@@@@@@@@@@@@@"; 
  //float r;
  for (int i = 0; i < (int)prestate->samples.size(); i++)
  {
    //if(abs(prestate->samples[i].cur)<50) {SIGMA6*2;}
    for (int k = 0; k < prestate->samples[i].ps.size(); k++)
    {
      float dy0k = rng.gaussian(SIGMA6);
      //float r= abs(dy0k)* motion*1;
      if(abs(motion)<0.005){
        for (int j = state_size-1 ;j >-1 ; j--){
          e[state_size-j-1] = rng.gaussian(SIGMA5);
          
          
          state->samples[i].ps[k].p[j].x += dy0k;

          for (int l = state_size-j-1; l >0 ; l--)
          {
            state->samples[i].ps[k].p[j].x += e[l-1] * (prestate->samples[i].points[j].y - prestate->samples[i].points[j +l].y);
          }
        }//end for
      }
      else if(motion>0.005)
      {
        for (int j = state_size-1 ;j >-1 ; j--)
          e[state_size-j-1] = rng.gaussian(SIGMA5);
        e[0] = abs(e[0])+motion/20;
        e[1] = abs(e[1])+motion/20;
        for (int j = state_size-1 ;j >-1 ; j--)
        {
          
          state->samples[i].ps[k].p[j].x += dy0k;

          for (int l = state_size-j-1; l >0 ; l--)
          {
            state->samples[i].ps[k].p[j].x += e[l-1] * (prestate->samples[i].points[j].y - prestate->samples[i].points[j +l].y);
          }
        }//end for
      }
      else{
        for (int j = state_size-1 ;j >-1 ; j--)
          e[state_size-j-1] = rng.gaussian(SIGMA5);
          e[0] = abs(e[0])+motion/20;
          e[1] = abs(e[1])+motion/20;
        for (int j = state_size-1 ;j >-1 ; j--){
          state->samples[i].ps[k].p[j].x += dy0k;

          for (int l = state_size-j-1; l >0 ; l--)
          {
            state->samples[i].ps[k].p[j].x -= e[l-1] * (prestate->samples[i].points[j].y - prestate->samples[i].points[j +l].y);
          }
        }//end for
      }
    }
  }
}*/
/*
void prediction(LaneState *prestate, LaneState *state,float motion)
{
  cv::RNG rng;
  CvParticle2D32f ps_;

  float e[state_size];

  state->ipmBoxes.clear();
  state->ipmSplines.clear();
  state->samples.clear();
  state->Splines.clear();
  state->ipmBoxes = prestate->ipmBoxes;
  state->ipmSplines = prestate->ipmSplines;
  state->samples = prestate->samples;
  //cout << motion << "@@@@@@@@@@@@@@"; 
  //float r;
  for (int i = 0; i < (int)prestate->samples.size(); i++)
  {
    //if(abs(prestate->samples[i].cur)<50) {SIGMA6*2;}
    for (int k = 0; k < prestate->samples[i].ps.size(); k++)
    {
      float dy0k = rng.gaussian(SIGMA6);
      //float r= abs(dy0k)* motion*1;
      
      for (int j = state_size-1 ;j >-1 ; j--)
      {
        e[state_size-j-1] = rng.gaussian(SIGMA5);
        if(abs(motion)<0.001){

          e[state_size-j-1] = rng.gaussian(SIGMA5);
        }
        else if (motion>0.001){
           //dy0k= dy0k+ motion/50;
          e[state_size-j-1] = rng.gaussian(SIGMA5);
          e[state_size-j-1] = abs(e[state_size-j-1])+motion/20;
        }
        else{
            //dy0k= dy0k+ motion/50;
          e[state_size-j-1] = rng.gaussian(SIGMA5);
          e[state_size-j-1] = -1*abs(e[state_size-j-1])+motion/20;
        }
        
        state->samples[i].ps[k].p[j].x += dy0k;

        for (int l = state_size-j-1; l >0 ; l--)
        {
          state->samples[i].ps[k].p[j].x += e[l-1] * (prestate->samples[i].points[j].y - prestate->samples[i].points[j +l].y);
        }
      }
    }
  }
}
*/
/*
void prediction(LaneState *prestate, LaneState *state)
{
  cv::RNG rng;
  CvParticle2D32f ps_;

  float e[state_size];

  state->ipmBoxes.clear();
  state->ipmSplines.clear();
  state->samples.clear();
  state->Splines.clear();
  state->ipmBoxes = prestate->ipmBoxes;
  state->ipmSplines = prestate->ipmSplines;
  state->samples = prestate->samples;

  for (int i = 0; i < (int)prestate->samples.size(); i++)
  {

    for (int k = 0; k < prestate->samples[i].ps.size(); k++)
    {
      float dy0k = rng.gaussian(SIGMA6);
      //float r0 = rng.gaussian(SIGMA5);
      //float r1 = rng.gaussian(SIGMA5);
      //float r2 = rng.gaussian(SIGMA5);
      //loat r3 = rng.gaussian(SIGMA5);

      //state->samples[i].ps[k].p[0].x += r0;
      //state->samples[i].ps[k].p[1].x += r1;
      //state->samples[i].ps[k].p[2].x += r2;
      //state->samples[i].ps[k].p[3].x += r3;

      for (int j = 0; j < state_size; j++)
      {
        e[j] = rng.gaussian(0.01);

        state->samples[i].ps[k].p[j].x += dy0k;

        for (int l = j; l > 0; l--)
        {
          state->samples[i].ps[k].p[j].x += e[j - l] * (prestate->samples[i].points[j].y - prestate->samples[i].points[j - l].y);
        }
      }
    }
  }
}*/

CvMat *CvPoint2D32f2Mat(const CvPoint2D32f vec[state_size])
{
  CvMat *mat = 0;
  //create the matrix
  mat = cvCreateMat(state_size, 2, CV_32FC1);
  //loop and get values
  for (int i = 0; i < state_size; i++)
  {
    CV_MAT_ELEM(*mat, float, i, 0) = vec[i].x;
    CV_MAT_ELEM(*mat, float, i, 1) = vec[i].y;
  }

  //return
  return mat;
}

CvMat *CvParticle2D32f2Mat(const CvParticle2D32f vec)
{
  CvMat *mat = 0;
  //create the matrix
  mat = cvCreateMat(state_size, 2, CV_32FC1);
  //loop and get values
  for (int i = 0; i < state_size; i++)
  {
    CV_MAT_ELEM(*mat, float, i, 0) = vec.p[i].x;
    CV_MAT_ELEM(*mat, float, i, 1) = vec.p[i].y;
  }

  //return
  return mat;
}
double SIGMA4 = 5;

LaneState *update(LaneState *state, LaneState *measurement, float num, const IPMInfo *ipmInfo)
{
  // update weight
  vector<CvParticle2D32f> update_ps;
  vector<double> update_weight;
  double q, sumq, aw, w0, w,q_;
  int ind;
  vector<int> update_list;


  vector<CvParticle2D32f>::iterator iter; vector<double>::iterator iter1; vector<int>::iterator iter2;
  for (int i=0; i< (int)state->ipmSplines.size(); i++){ //number of line
    update_ps.clear();
    update_weight.clear();
    ind = state->ipmSplines[i].label;
    sumq = 0;
    update_list.clear();
    for (int l = 0; l < (int)measurement->ipmSplines.size(); l++)
    { // find matched measurement
     //cout << "\n measurement label:" <<measurement->ipmSplines[l].label << "\n";
      if(measurement->ipmSplines[l].label == ind)
        update_list.push_back(l);
        
    }
    //cout << update_list.size()<<"\tlabel:" <<ind << "\n";
    if (update_list.size() ==0)
    {
      state->ipmSplines[i].noUpdate+=1;
      continue; 
    }
    
    
    
    for (iter = state->samples[i].ps.begin(), iter1 = state->samples[i].weight.begin(); iter != state->samples[i].ps.end(), iter1 != state->samples[i].weight.end(); ++iter, ++iter1)
    {
      
      q = 0; 

      for(iter2=update_list.begin();iter2 != update_list.end();iter2++){ 
        q_=0;
        for (int j = 0; j < state_size; j++)
        {
         q_+= pow((iter->p[j].x - measurement->samples[*iter2].points[j].x), 2);
        }//
         q += exp(-q_ / pow(SIGMA4, 2));
      }
      q*=state->samples[i].weight[*iter1];

      update_ps.push_back(*iter);
      update_weight.push_back(q);
        

    }

    state->samples[i].weight.clear();
    state->samples[i].weight = update_weight;
    update_weight.clear();
    state->samples[i].ps.clear();
    state->samples[i].ps = update_ps;
    update_ps.clear();

    for (int k = 0; k < state->samples[i].ps.size(); k++)
    {
      sumq += state->samples[i].weight[k];
    }

    for (int k = 0; k < state->samples[i].ps.size(); k++)
    {
      state->samples[i].weight[k] /= sumq;
     
    }
      //cout <<"\n"<< state->samples[i].ps.size() << "**************" << i <<"\n";
  }// end for each line
  
  // calculate the 4 control points according to weight
  for (int i = 0; i < (int)state->samples.size(); i++)
  {

    for (int j = 0; j < state_size; j++)
    {
      aw = 0;
      for (int k = 0; k < state->samples[i].ps.size(); k++)
      {
        w = state->samples[i].weight[k] * state->samples[i].ps[k].p[j].x;
        aw += w;
      }
      state->samples[i].points[j].x = aw;
      //cout << aw<<"aw\n";
    }
  }

  // change 4 points to spline
  vector<Spline> newSplines;
  for (int i = 0; i < (int)state->samples.size(); i++)
  { // number of line
    CvMat *m = CvPoint2D32f2Mat(state->samples[i].points);
    //SHOW_MAT(m);
    Spline newSpline = mcvFitBezierSpline(m, 3);
    PredictSlope(m,&(newSpline.line),ipmInfo);
    //cout << "PredictSlope"<<newSpline.line.rtheta[1];
    newSpline.label = state->ipmSplines[i].label;
    newSpline.tangent = state->ipmSplines[i].tangent;
    newSpline.noUpdate = state->ipmSplines[i].noUpdate;
    newSplines.push_back(newSpline);
    //SHOW_LINE((newSpline.line), "line");      
  }
  state->ipmSplines.clear();
  state->ipmSplines = newSplines;

  return state;
}
void PredictSlope(const CvMat *cpoints, Line *line, const IPMInfo *ipmInfo )
{
  CvMat *W = cvCreateMat(2, 1, CV_32FC1);
  CvMat *V = cvCreateMat(2, 2, CV_32FC1);
  float meanX = 0, meanY = 0;
  CvScalar mean;
  CvMat row1, row2;

  CvMat *cpointst = cvCreateMat(cpoints->rows,cpoints->cols, CV_32FC1);
  cvCopy(cpoints, cpointst);
  cvGetCol(cpointst, &row1, 0);
  mean = cvAvg(&row1);
  meanX = (float)mean.val[0];
  cvSubS(&row1, mean, &row1);
  //same for second row
  cvGetCol(cpointst, &row2, 1);
  mean = cvAvg(&row2);
  meanY = (float)mean.val[0];
  cvSubS(&row2, mean, &row2);
  cvSVD(cpointst, W, 0, V, CV_SVD_V_T);

    //get the [a,b] which is the second column corresponding to
  //smaller singular value
  float a, b, c;
  a = CV_MAT_ELEM(*V, float, 0, 1);
  b = CV_MAT_ELEM(*V, float, 1, 1);

  //c = -meanX*a-meanY*b
  c = -(meanX * a + meanY * b);
  // cout << a <<"\t" << b << "\t"<<c;
  //compute r and theta
  //theta = atan(b/a)
  //r = meanX cos(theta) + meanY sin(theta)
  float r, theta;
  theta = -atan2(b, a);
  r = meanX * cos(theta) + meanY * sin(theta);
    //correct
  if (r < 0)
  {
    //correct r
    r = -r;
    //correct theta
    theta += CV_PI;
    if (theta > CV_PI)
      theta -= 2 * CV_PI;

  }

  //else{
  //  theta=-theta;
  //}

  cvReleaseMat(&cpointst);
  cvReleaseMat(&W);
  cvReleaseMat(&V);

  //we have the index of min and max points, and
  //their distance, so just get them and compute
  //the end points

  line->startPoint.x = -c/a-b/a*(ipmInfo->height-1);
  line->startPoint.y =0;

  line->endPoint.x = -c/a;
  line->endPoint.y = ipmInfo->height-1;
  //theta = atan2(line->endPoint.y-line->startPoint.y,line->endPoint.x -line->startPoint.x);
  //theta -= CV_PI/2;
  //SHOW_LINE(*line, "line");
  line->rtheta[0]=r;
  line->rtheta[1]=theta;

}
/// 
/*
LaneState *update(LaneState *state, LaneState *measurement, float num)
{
  // update weight
  vector<CvParticle2D32f> update_ps;
  vector<double> update_weight;
  double q, sumq, aw, w0, w;
  int ind;
  vector<int> update_list;
  // for (int i = 1; i <= state->samples.size(); i++)
  //  update_list.push_back(i);

  vector<CvParticle2D32f>::iterator iter;
  vector<double>::iterator iter1;
  
  for (int i = 0; i < (int)measurement->ipmSplines.size(); i++)
  { //number of lines of measurement
     update_ps.clear();
    update_weight.clear();
    sumq = 0;
    int k = -1;
    ind = measurement->ipmSplines[i].label;
    
    bool find = false;
    //find the same label to udpate

    for (int l = 0; l < (int)state->samples.size(); l++)
    { //number of line of current state
      if (state->ipmSplines[l].label == ind)
      {
        ind = l;
        find = true;
        //update_list[l] = -1;
        break;
      }
    }
    //calculate weight
    
    if (find == true)
    {
      for (iter = state->samples[ind].ps.begin(), iter1 = state->samples[ind].weight.begin(); iter != state->samples[ind].ps.end(), iter1 != state->samples[ind].weight.end(); ++iter, ++iter1)
      {
        q = 0;
        k++;
        for (int j = 0; j < state_size; j++)
        {
          q += exp(-(pow((iter->p[j].x - measurement->samples[i].points[j].x), 2) / pow(SIGMA4, 2)));
        }

        if (q > 10e-7)
        {
          update_ps.push_back(*iter);
          update_weight.push_back(q);
        }
      }
      if (update_weight.size() < num / 100)
      {
        //cout << "\nnonononono\n";
        //state->samples[ind].update_t += 1;
        continue;
      }
      else
      {
        state->samples[ind].weight.clear();
        state->samples[ind].weight = update_weight;
        update_weight.clear();
        state->samples[ind].ps.clear();
        state->samples[ind].ps = update_ps;
        update_ps.clear();
        //state->samples[ind].update_t = 0;
      }

      for (int k = 0; k < state->samples[ind].ps.size(); k++)
      {
        sumq += state->samples[ind].weight[k];
      }
      for (int k = 0; k < state->samples[ind].ps.size(); k++)
      {
        state->samples[ind].weight[k] /= sumq;
      }
      cout << state->samples[ind].ps.size() << "**************" << ind <<"\n";

    }
  } //end for number of lines of measurement

  // calculate the 4 control points according to weight
  for (int i = 0; i < (int)state->samples.size(); i++)
  {

    for (int j = 0; j < state_size; j++)
    {
      aw = 0;
      for (int k = 0; k < state->samples[i].ps.size(); k++)
      {
        w = state->samples[i].weight[k] * state->samples[i].ps[k].p[j].x;
        aw += w;
      }
      state->samples[i].points[j].x = aw;
      //cout << aw<<"\n";
    }
  }*/
  // penalty to those has not been updated.  (Under-deverloped)
 /* vector<int>::iterator iter2;
  for (iter2 = update_list.begin(); iter2 == update_list.end(); iter2++)
  {
    if (*iter2 != -1)
      state->samples[*iter2].update_t += 1;
    if (state->samples[*iter2].update_t > 2)
    {
      //ResetPF(measurement, state, num ,ind,m_ind);
      cout << "\n"
           << *iter2;
    }
  }*/
  /*
  // change 4 points to spline
  vector<Spline> newSplines;
  for (int i = 0; i < (int)state->samples.size(); i++)
  { // number of line
    CvMat *m = CvPoint2D32f2Mat(state->samples[i].points);
    //SHOW_MAT(m);
    Spline newSpline = mcvFitBezierSpline(m, 3);
    newSpline.label = state->ipmSplines[i].label;

    newSplines.push_back(newSpline);
  }
  state->ipmSplines.clear();
  state->ipmSplines = newSplines;

  return state;
}*/
// 4/12 

/*LaneState* update(LaneState* state,LaneState* measurement){
  // update weight

  double q,sumq,aw,w0,w;
  vector<CvParticle2D32f>::iterator iter;vector<double>::iterator iter1;
  for (int i=0; i< (int)state->samples.size(); i++){ //number of line
    sumq = 0;   
    int k=-1;
    for (iter =state->samples[i].ps.begin(), iter1 = state->samples[i].weight.begin();iter <= state->samples[i].ps.end(),iter1 != state->samples[i].weight.end();++iter,  ++iter1){
      q = 0;
      k++;
      for (int j=0; j<4; j++){
       q += exp(-(pow((iter->p[j].x
                    -  measurement-> samples[i].points[j].x),2)/ pow(SIGMA4,2)) );
      }

      if (q < 10e-30){
        state->samples[i].ps.erase(iter); 
        state->samples[i].weight.erase(iter1);
        k--;iter--;iter1--;
      }
      else{
      state->samples[i].weight[k] *= q;     
      sumq +=state->samples[i].weight[k];
     }
    }

    for (int k=0; k< state->samples[i].ps.size(); k++){
        state->samples[i].weight[k] /= sumq; 
    }
    cout << state->samples[i].ps.size() << "**************\n";
  } 
  // calculate the 4 control points according to weight
  for (int i=0; i< (int)state->samples.size(); i++){
    
    for (int j=0; j<4; j++){
     aw = 0;
     for (int k=0; k< state->samples[i].ps.size(); k++){
        w = state->samples[i].weight[k] * state->samples[i].ps[k].p[j].x;
        aw += w;
      }
        state->samples[i].points[j].x = aw;
        //cout << aw<<"\n";
    }     
  }
  // change 4 points to spline
  state->ipmSplines.clear();
  for (int i=0; i< (int)state->samples.size(); i++){
    CvMat* m = mcvCvPoint2D32f2Mat(state->samples[i].points);
    //SHOW_MAT(m); 
    Spline newSpline = mcvFitBezierSpline(m,3);
    state->ipmSplines.push_back(newSpline);
    
  }

  return state;
  
}*/
double uniform_random()
{
  srand(time(NULL));
  float x = (float)rand() / (RAND_MAX + 1.0);
  return x;
}
/*
void resample(LaneState* state,float num){
  if ((int)state->samples[0].ps.size()>=num)
    return;
  int diff_num = fabs((int)state->samples[0].ps.size()-num); int new_num;
  cout<< "\n77777777777" <<diff_num ;
  double sumq;
  vector<CvParticle2D32f>::iterator iter;int k; vector<double>::iterator iter1;
  for (int i=0; i< (int)state->samples.size(); i++){ //number of line
    k=-1;
    sumq=0;
    for (iter =state->samples[i].ps.begin(), iter1 = state->samples[i].weight.begin();iter <= state->samples[i].ps.end(),iter1<=state->samples[i].weight.end();++iter,  ++iter1){
      k++;
      new_num = (floor)(diff_num*(*iter1));
      if (new_num>0){
        state->samples[i].ps.insert(iter,new_num,*iter); 
        state->samples[i].weight.insert(iter1,new_num-1,*iter1);
        cout<< "\n=========" <<new_num << "\t" << state->samples[i].weight.size() << "\t"<<k;
        iter+=(new_num-1);iter1+=(new_num-1);
     
    }
 
 
    }


    }
  }
}*/
/* 5/08
void resample(LaneState *state, float num, float Nth){

  vector<CvParticle2D32f> update_ps;
  vector<double> update_weight;

  for (int i = 0; i < (int)state->ipmSplines.size(); i++)
  { //number of line
    int n = (int)state->samples[i].ps.size();
    if (n > (int)Nth)
    {
      continue;
    }

    double beta = 0;
    int ind = (int)(uniform_random() * n);
    double maxw = *std::max_element(state->samples[i].weight.begin(), state->samples[i].weight.end());
    for (int j = 0; j < (int)num; j++)
    {
      double ram = uniform_random();
      beta += ram * maxw*2;
      while (beta > state->samples[i].weight[ind])
      {
        beta -= state->samples[i].weight[ind];
        ind = (ind + 1) % n;
      }

      update_ps.push_back(state->samples[i].ps[ind]);
    }
    for (int k = 0; k < update_ps.size(); k++)
    {
      update_weight.push_back(1 / num);
    }
    //state->samples[i].ps.clear();
    state->samples[i].weight.clear();
    state->samples[i].ps = update_ps;
    state->samples[i].weight = update_weight;
    //cout <<"resample particle number " <<update_weight.size()";
    update_ps.clear();
    update_weight.clear();
  }
}
*/
void resample(LaneState *state, float num, float Nth){

  vector<CvParticle2D32f> update_ps;
  vector<double> update_weight;
  
  for (int i = 0; i < (int)state->ipmSplines.size(); i++)
  { //number of line
    float Neff =0.0;
    int n = (int)state->samples[i].ps.size();
    for (int l = 0; l < (int)num; l++)
      Neff+=pow(state->samples[i].weight[l],2);
    Neff  = 1/Neff;
    //cout << "\nskip" << Neff;
    if (Neff >Nth)
    {
      
      continue;
    }

    double beta = 0;
    int ind = (int)(uniform_random() * num);
    double maxw = *std::max_element(state->samples[i].weight.begin(), state->samples[i].weight.end());
    for (int j = 0; j < (int)num; j++)
    {
      double ram = uniform_random();
      beta += ram * maxw*2;
      while (beta > state->samples[i].weight[ind])
      {
        beta -= state->samples[i].weight[ind];
        ind = (ind + 1) % int(num);
      }

      update_ps.push_back(state->samples[i].ps[ind]);
    }
    for (int k = 0; k < update_ps.size(); k++)
    {
      update_weight.push_back(1 / num);
    }
    state->samples[i].ps.clear();
    state->samples[i].weight.clear();
    state->samples[i].ps = update_ps;
    state->samples[i].weight = update_weight;
    //cout <<"resample particle number " <<update_weight.size()";
    update_ps.clear();
    update_weight.clear();
  }
}

void updateCenterlineWid(LaneState *state,CenterLineState *center_line,CameraInfo *cameraInfo){
      
  if (state->samples.size()<2)
    return;
  center_line->wid[0] = abs(state->samples[0].points[1].x- state->samples[1].points[1].x);
  center_line->wid[1] = abs(state->samples[0].points[2].x- state->samples[1].points[2].x);
  center_line->pitch =  (center_line->wid[1]-center_line->wid[0])/center_line->wid[0]*cameraInfo->cameraHeight/(2-1)/191/24; // 1 point/ length of each pixel/ pixel between point 
  cout << "debug update center line\t" << center_line->wid[1] <<"\t"<< center_line->wid[0] << "\t" << center_line->pitch ;
  return;
}
void updateCenterlineHeight(CenterLineState *center_line){
      
if(isnan(center_line->wid[1])||isnan(center_line->wid[0]))
  return;
center_line->hc[0] =center_line->hc[0]*center_line->hc[1]/(center_line->wid[0] +center_line->wid[1] )*2;
center_line->hc[1] =  (center_line->wid[0]+center_line->wid[1])/2;  
cout << "debug update center line hieght\t" << center_line->hc[0] <<"\t"<< center_line->hc[1] ;
return;
}
void InitCenterline(LaneState *state,CenterLineState *center_line,
                        CameraInfo *cameraInfo)
 {
      
  center_line->wid[0] = abs(state->samples[0].points[1].x- state->samples[1].points[1].x);
  center_line->wid[1] = abs(state->samples[0].points[2].x- state->samples[1].points[2].x);
  center_line->hc[0] =  cameraInfo->cameraHeight;
  center_line->hc[1] =  (center_line->wid[0]+center_line->wid[1])/2;  
  center_line->pitch =  (center_line->wid[1]-center_line->wid[0])/center_line->wid[0]*cameraInfo->cameraHeight/(2-1)/191/24; // 1 point/ length of each pixel/ pixel between point 
  
  cout << "debug Init update center line\t" << center_line->wid[1] <<"\t"<< center_line->wid[0] << "\t" << center_line->pitch << "\t" <<center_line->hc[0];
  return;
}
CvMat* CvPoint2D32f2Mat_inv(const CvPoint2D32f vec[state_size])
{
  CvMat *mat = 0;
    //create the matrix
    mat = cvCreateMat(2, state_size, CV_32FC1);
    //loop and get values
    for (int i=0; i<state_size; i++){
      CV_MAT_ELEM(*mat, float,0, i) = vec[i].x;
      CV_MAT_ELEM(*mat, float, 1,i) = vec[i].y;
    }

  //return
  return mat;
}
void PointImIPM2World(CvPoint2D32f &point, CvPoint2D32f &new_point,
                        const IPMInfo *ipmInfo)
{
  //x-direction
  new_point.x = point.x/ ipmInfo->xScale;
  new_point.x += ipmInfo->xLimits[0];
  //y-direction
  new_point.y = point.y/ ipmInfo->yScale;
  new_point.y = ipmInfo->yLimits[1] - new_point.y;

}


void Compute_curve(LaneState* state){
  
  float cur =0;
  for (int i = 0; i < (int)state->samples.size(); i++)
  {
    if (state->samples[i].points[0].x-state->samples[i].points[state_size-1].x!=0){
      cur = (state->samples[i].points[0].y-state->samples[i].points[state_size-1].y) /(state->samples[i].points[0].x-state->samples[i].points[state_size-1].x);}
    else{
      cur=100;}
    state->ipmSplines[i].cur=cur;
    cout << "\ncur:"<<cur;
  }
  
}


//**************************************
// Main Tracking  Part
void TrackLanes_PF(float motionV,float motionW,CvMat *dbipm, LaneState *measurements,
                  IPMInfo *ipmInfo,LaneDetectorConf *LineConf,CameraInfo *cameraInfo, 
                  LaneState *pre_state, LineState *state, bool *isInit,CenterLineState *center_line)
{
 
  // get meas5urement
  float np = 1500;
  if (*isInit == false)
  {
  printf("Start Track, calculate measurement");
  CalculateSample_init(dbipm, LineConf, measurements, state);
  printf("n control point \n");
  InitPF(measurements, pre_state, np);   
  InitCenterline(pre_state,center_line,cameraInfo);
  printf("Inital PF \n");
  *isInit = true;
  cin.get();
  return;
  }
  printf("\nStart Track, calculate measurement");
  CalculateSample(dbipm, LineConf, measurements, state);
  printf("\nn control point ");
  
  LaneState lane_state_next;
  LaneState *new_state;
  //for(int i=0;i<state_size;i++){
  // cout << "\n++" << measurements->samples[0].points[i].x << "\t" <<  measurements->samples[0].points[i].y << "\n";
  //}
  // initial lane state


  //for(int i=0;i<state_size;i++){
  // cout << "\n++" << measurements->samples[0].points[i].x << "\t" <<  measurements->samples[0].points[i].y << "\n";
  //}
  //Prediction
  clock_t startTime = clock();
  printf("\nPrediction ");
  prediction(dbipm,pre_state, &lane_state_next,motionV,motionW,LineConf,ipmInfo);

  //1225//prediction(pre_state, &lane_state_next,motionW,LineConf,ipmInfo);
  //1220//prediction(pre_state, &lane_state_next,motion);
  CvMat *db ,*dbClr;
  dbClr = cvCreateMat(dbipm->height, dbipm->width, dbipm->type);
  db= cvCreateMat(dbClr->rows, dbClr->cols,CV_32FC3);
  cvCopy(dbipm, dbClr);
  cvCvtColor(dbClr, db, CV_GRAY2RGB);
  for (int i = 0; i < (int)lane_state_next.samples.size(); i++)
  {
    for (int k = 0; k < pre_state->samples[0].ps.size(); k++)
    {
      mcvDrawPoints(db, pre_state->samples[i].ps[k].p, CV_RGB(0, 0, 0), 0.5);
    }
  }
    
  //SHOW_IMAGE(db, "sample",10);

  //Update
  printf("\nUpdate");
  new_state = update(&lane_state_next, measurements, np,ipmInfo);
  *pre_state = *new_state;

  //for(int i=0;i<state_size;i++){
  // cout << "\n**" << pre_state->samples[0].points[i].x << "\t" <<  pre_state->samples[0].points[i].y << "\n";
  //}

  resample(pre_state, np, np /10);
  printf("\nResampling\n");
  //Compute_curve(pre_state);

  for (int i = 0; i < (int)measurements->samples.size(); i++)
  {
    mcvDrawPoints(db,  measurements->samples[i].points, CV_RGB(0,1,1), 3);
    mcvDrawSpline_noControlPoints(db, measurements->ipmSplines[i], CV_RGB(0,1,1), 1);
  }
  
  for (int i = 0; i < (int)lane_state_next.samples.size(); i++)
  {
    //mcvDrawPoints(db,  measurements->samples[i].points, CV_RGB(1,1,1), 3);
    mcvDrawSpline_noControlPoints(db, pre_state->ipmSplines[i], CV_RGB(1,0,0), 2);
    mcvDrawPoints(db, pre_state->samples[i].points, CV_RGB(1,0, 0), 3);
    //mcvDrawLine(db, pre_state->ipmSplines[i].line, CV_RGB(0, 1, 0), 1);
   
  }


  
  SHOW_IMAGE(db, "sample", 1);
  cvReleaseMat(&db);
  cvReleaseMat(&dbClr);
  //}//#endif

  // For line dtector  to process next frame
  state->ipmSplines.clear();
  state->ipmBoxes.clear();
  state->ipmSplines = pre_state->ipmSplines;
  //cout << state->ipmSplines.size()<<"qqqqqqqqqqqqqqq\n";
  state->ipmBoxes = pre_state->ipmBoxes;

  /*Update Camera Info*/
  printf("Update camera INFO\n");
  updateCenterlineWid(pre_state,center_line,cameraInfo);
  updateCenterlineHeight(center_line);
  if(! isnan(center_line->wid[1])& ! isnan(center_line->wid[0]))
  mcvUpdateCameraInfo (cameraInfo, center_line->pitch, center_line->hc[0]);
  cout << "\n~~~" << cameraInfo->pitch* 180/CV_PI;

 /* log gl/obal point*/
  for (int i = 0; i < (int)pre_state->samples.size(); i++){
    for( int j=0; j<state_size;j++)
    {
    PointImIPM2World(pre_state->samples[i].points[j],pre_state->samples[i].points_w[j], ipmInfo);
    //cout << pre_state->samples[i].points_w[j].x<< "\n";
    }  
  }
}

} // namespace LaneDetector

/* Test  pointer
      int *a, *b;
      int c,d;
      c=1;d=2;
      a = &c;
      b = &d;
      *b = *a;
      cout << *b;
      */
/*  Find maximum element in a vector
int myints[] = {3,7,2,5,6,4,9};
  std::cout << "The smallest element is " << *std::min_element(myints,myints+7) << '\n';
  std::cout << "The largest element is "  << *std::max_element(myints,myints+7) << '\n';
  */