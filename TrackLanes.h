/* \Lane estimate:sub image windomm fusion using EKF.  
 *      \TrackLanes.h
 * \Copyright (C) 2019 Sunny Tseng
 *
 * 
 */
#ifndef LaneDetector_TrackLanes_h
#define LaneDetector_TrackLanes_h

#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include <stdio.h>
#include "LaneDetector.hh"
#include "mcv.hh"
#include "InversePerspectiveMapping.hh"
#include <iostream>

namespace LaneDetector
{   
    void InitLaneKalmanFilter(cv::KalmanFilter &laneKalmanFilter, cv::Mat &laneKalmanMeasureMat, int &laneKalmanIdx,float dt);
    bool ParticleCmp(const Particle &p1, const Particle &p2);
    //function definitions
    /**
     * This function calculate the four control pints of the spline,
     * {y_0,y_1,y_2,_3}
     *
     * \param 
     *     
     * \param 
     * \param 
     */
    void CalculateSample( const CvMat *im, LaneDetectorConf *lineConf,LaneState* samples,LineState* state);
    /** This function draws point onto the passed image
     *
     * \param particle 
     * \param line color
     * \param width line width
     *
     */
    void mcvDrawPoints(CvMat *image, CvPoint2D32f p, CvScalar color, int width);
    /*
     * This function update center line
     * \param lane state
     * \param centerline
     */
  void updateCenterlineWid(LaneState *state,CenterLineState *center_line,CameraInfo *cameraInfo);
      /*
     * This function initial center line
     * \param lane state
     * \param centerline
     */
  void InitCenterline(LaneState *state,CenterLineState *center_line,CameraInfo *cameraInfo);
    /*
     * This function  tracks the particle filter .
     * \param image for debug
     * \param current state: x(k)
     * \param measurement: y(k)
     * \param flag to init PF
     * \state of cneter lane
     */
    void TrackLanes_PF(float motionV,float motionW,CvMat *dbipm,LaneState* measurements,IPMInfo *ipmInfo,LaneDetectorConf *stopLineConf,CameraInfo* cameraInfo,LaneState* pre_state,LineState* state,bool *isInit,CenterLineState *center_line);
    /**
     * This function initialize the particle filter with related params.
     * \param first measurement
     * \param current state: x(k)
     * \param number of particle
     */
    void InitPF(LaneState* measurements,LaneState* state,float num);
        /**
     * This function reset the particle filter with related params, while the particle has not update for several time.
     * \param first measurement
     * \param current state: x(k)
     * \param number of particle
     * \index of sample state 
     * \index of measurement state
     */
     void ResetPF(LaneState *measurements, LaneState *state, float num,int ind,int m_ind);
     /*
     * This function is the prediction step of the particle filter .
     * \param current state: x(k)
     * \param next state: bar_x(k+1)
     */
    void prediction(const CvMat *im,LaneState *prestate, LaneState *state,
                float motionV,float motionW,LaneDetectorConf *stopLineConf,const IPMInfo *ipmInfo);
    //1220//void prediction(LaneState *prestate, LaneState *state,float motion);
    //1225//void prediction(LaneState* prestate,LaneState* state,
            //        float motion,LaneDetectorConf *stopLineConf,const IPMInfo *ipmInfo);
 
     /*
     * This function is the udpate step of the particle filter .
     * \param current state: x(k)
     * \param measurement: bar_x(k+1)
     */
    LaneState* update(LaneState* state,LaneState* measurement,float num, const IPMInfo *ipmInfo);
    void PredictSlope(const CvMat *cpoints, Line* line, const IPMInfo *ipmInfo);
     /*
     * This function is the resampling step of the particle filter .
     * \param current state: x(k)
     * \param number of particle
     */
    void resample(LaneState* state,float num, float Nth);
    double uniform_random();
    void PointImIPM2World(CvPoint2D32f*point,CvPoint2D32f *new_point, const IPMInfo *ipmInfo);
    
     /*
     * This functionis  for compute the curveness(not exact the curvature) 
     * \param current state: x(k)
     */
    void Compute_curve(LaneState* state);
    
    CvMat *CvParticle2D32f2Mat(const CvParticle2D32f vec);

}
#endif