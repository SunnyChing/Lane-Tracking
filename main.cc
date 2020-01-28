/**
 * \file main.cc
 * \author Mohamed Aly <malaa@caltech.edu>
 * \date Wed Oct 6, 2010
 *
 */

#include "main.hh"

#include "cmdline.h"
#include "LaneDetector.hh"

#include <stdio.h>
#include <fstream>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <ctime>
#include <string>
#include <cv.h>
#include <highgui.h>
using namespace cv;
// Useful message macro
#define MSG(fmt, ...) \
  (fprintf(stdout, "%s:%d msg   " fmt "\n", __FILE__, __LINE__, ##__VA_ARGS__) ? 0 : 0)

// Useful error macro
#define ERROR(fmt, ...) \
  (fprintf(stderr, "%s:%d error " fmt "\n", __FILE__, __LINE__, ##__VA_ARGS__) ? -1 : -1)


namespace LaneDetector
{

/**
 * This function reads lines from the input file into a vector of strings
 *
 * \param filename the input file name
 * \param lines the output vector of lines
 */
bool ReadLines(const char* filename, vector<string> *lines)
{
  // make sure it's not NULL
  if (!lines)
    return false;
  // resize
  lines->clear();

  ifstream  file;
  file.open(filename, ifstream::in);
  char buf[5000];
  // read lines and process
  while (file.getline(buf, 5000))
  {
    string str(buf);
    lines->push_back(str);
  }
  // close
  file.close();
  return true;
}
bool ReadLinesfloat(const char* filename, vector<float> *lines)
{
  // make sure it's not NULL
  if (!lines)
    return false;
  // resize
  lines->clear();

  ifstream  file;
  file.open(filename, ifstream::in);
  char buf[5000];
  // read lines and process
  while (file.getline(buf, 5000))
  {
    float f=  atof(buf);
    lines->push_back(f);
  }
  // close
  file.close();
  return true;
}

/**
 * This function processes an input image and detects lanes/stoplines
 * based on the passed in command line arguments
 *
 * \param filename the input file name
 * \param cameraInfo the camera calibration info
 * \param lanesConf the lane detection settings
 * \param stoplinesConf the stop line detection settings
 * \param options the command line arguments
 * \param outputFile the output file stream to write output lanes to
 * \param index the image index (used for saving output files)
 * \param elapsedTime if NOT NULL, it is accumulated with clock ticks for
 *        the detection operation
 */
void ProcessImage(float motionV,float motionW,bool *isInit, const char* filename,IPMInfo& ipmInfo, CameraInfo& cameraInfo,
                  LaneDetectorConf& lanesConf, LaneDetectorConf& stoplinesConf,
                  gengetopt_args_info& options, ofstream* outputFile,ofstream* outputFile1,ofstream* outputFile2,
                  int index, clock_t *elapsedTime, LineState* line_state,LaneState* measurement, LaneState* prestate,CenterLineState *center_line)
{
  bool SAVE_IMAGE_IPM =false;
  // load the image
  CvMat *raw_mat, *mat, *dbipm;
  mcvLoadImage(filename, &raw_mat, &mat);

  
  // detect lanes
  vector<FLOAT> lineScores, splineScores;
  vector<Line> lines;
  vector<Spline> splines;
  dbipm = cvCreateMat((int)lanesConf.ipmHeight, (int)lanesConf.ipmWidth, mat->type);
 
  clock_t startTime = clock();
  mcvGetLanes(mat, raw_mat, &lines, &lineScores, &splines, &splineScores,
             &ipmInfo, &cameraInfo, &lanesConf,dbipm,line_state);  

    #warning "Sunny LaneTracking here"
  //printf("%s\n", *is+ Init ? "true" : "false");
 TrackLanes_PF(motionV,motionW,dbipm,measurement,&ipmInfo,&lanesConf, &cameraInfo,prestate,line_state, isInit,center_line);
  clock_t endTime = clock();
/*
  if(SAVE_IMAGE_IPM){
    string f = string(filename);
    std::size_t botDirPos = f.find_last_of("/");
    string file = "../IPM_image0228/IPM_" + f.substr(botDirPos+1, f.length()-4);
    Mat b =Mat(dbipm, true);
    b.convertTo(b, CV_32FC3, 255.0); 
    imwrite(file,b);
  }*/
  // cout <<measurement->samples[0].points[0].x<< "\n";
  cvReleaseMat(&dbipm);

  MSG("Found %d lanes in %f msec", splines.size(),
     static_cast<double>(endTime - startTime) / CLOCKS_PER_SEC * 1000.);
  
 
  // update elapsed time
  if (elapsedTime)
    (*elapsedTime) += endTime - startTime;

  // save results?
  if (options.save_lanes_flag && outputFile && outputFile->is_open() && outputFile1 && outputFile1->is_open())
  {
    (*outputFile) << "frame#" << setw(8) << setfill('0') << index <<
      " has " << prestate->ipmSplines.size() << " splines" << endl;
    for (int i=0; i<prestate->ipmSplines.size(); ++i)
    {
      (*outputFile) << "\tspline#" << i+1 << " has " <<
       prestate->ipmSplines[i].degree+1 << " points and score " <<
        prestate->ipmSplines[i].score << endl;
      for (int j=0; j<=prestate->ipmSplines[i].degree; ++j)
        (*outputFile) << "\t\t" <<
          prestate->ipmSplines[i].points[j].x << ", " <<
          prestate->ipmSplines[i].points[j].y << endl;
    }
    //(*outputFile) << center_line->wid[0] << "\t" << center_line->wid[1] << "\t"<<center_line->hc[0]<<endl;
  
    //Sunny
    
    for (int j=0; j<prestate->samples.size(); ++j){
      for (int i=0; i<sizeof(prestate->samples[j].points)/8; ++i){
        (*outputFile1)<< prestate->samples[j].points[i].x << "\t";

      }
      (*outputFile1)<< endl;
    }
    (*outputFile2) << "frame#" << setw(8) << setfill('0') << index <<
      " has " << measurement->ipmSplines.size() << " splines" << endl;
    for (int i=0; i<measurement->ipmSplines.size(); ++i)
    {
      (*outputFile2) << "\tspline#" << i+1 << " has " <<
       measurement->ipmSplines[i].degree+1 << " points and score " <<
        measurement->ipmSplines[i].score << endl;
      for (int j=0; j<=measurement->ipmSplines[i].degree; ++j)
        (*outputFile2) << "\t\t" <<
          measurement->ipmSplines[i].points[j].x << ", " <<
          measurement->ipmSplines[i].points[j].y << endl;
    }
  }

  // show or save
  if (options.show_flag || options.save_images_flag)
  {
    // show detected lanes
    CvMat *imDisplay = cvCloneMat(raw_mat);
    // convert to BGR
//     cvCvtColor(raw_mat, imDisplay, CV_RGB2BGR);
    if (lanesConf.ransacLine && !lanesConf.ransacSpline)
      for(int i=0; i<lines.size(); i++)
        mcvDrawLine(imDisplay, lines[i], CV_RGB(0,125,0), 3);
    // print lanes
    if (lanesConf.ransacSpline)
    {
      for(int i=0; i<splines.size(); i++)
      {
       // if (splines[i].color == LINE_COLOR_YELLOW)
          //mcvDrawSpline(imDisplay, splines[i], CV_RGB(255,255,0), 3);
        //else
          //mcvDrawSpline(imDisplay, splines[i], CV_RGB(0,255,0), 3);
        // print numbers?
        if (options.show_lane_numbers_flag)
        {
          char str[256];
          sprintf(str, "%d", i);
         // mcvDrawText(imDisplay, str,
          //           cvPointFrom32f(splines[i].points[splines[i].degree]),
           //                           1, CV_RGB(0, 0, 255));
        }
      }
    }
    // show?
    if (options.show_flag)
    {
      // set the wait value
      int wait = options.step_flag ? 0 : options.wait_arg;
      // show image with detected lanes
      SHOW_IMAGE(imDisplay, "Detected Lanes", wait);
    }
    // save?
    if (options.save_images_flag)
    {
      // file name
      stringstream ss;
      ss << filename << options.output_suffix_arg << "_" << setw(6) <<
        setfill('0') << index << ".png";
      string outFilename = ss.str();
      // save the image file
      MSG("Writing output image: %s", outFilename.c_str());
      cvSaveImage(outFilename.c_str(), imDisplay);
    }
    // clear
    cvReleaseMat(&imDisplay);
  }

  cvReleaseMat(&raw_mat);
  cvReleaseMat(&mat);

}


int Process(int argc, char** argv)
{
  // parse the command line paramters
  gengetopt_args_info options;
  if (cmdline_parser (argc, argv,  &options) < 0)
    return -1;

  // read the camera configurations
  CameraInfo cameraInfo;
  IPMInfo ipmInfo;
  mcvInitCameraInfo(options.camera_conf_arg, &cameraInfo);
  MSG("Loaded camera file");

  // read the configurations
  LaneDetectorConf lanesConf, stoplinesConf;
  if (!options.no_lanes_flag)
  {
    mcvInitLaneDetectorConf(options.lanes_conf_arg, &lanesConf);
    MSG("Loaded lanes config file");
  }
  if (!options.no_stoplines_flag)
  {
    mcvInitLaneDetectorConf(options.stoplines_conf_arg, &stoplinesConf);
    MSG("Loaded stop lines config file");
  }

  // set debug to true
  if (options.debug_flag)
    DEBUG_LINES = 1;
  //Sunny PF initi
  LineState line_state;
  LaneState measurement, lane_state;
  CenterLineState center_line;
  bool* isInit =new bool; 
  *isInit = false;
  // Initialize Lane Kalman Filter
  //cv::KalmanFilter laneKalmanFilter(8, 8, 0);//(rho, theta, delta_rho, delta_theta)x2
  //cv::Mat laneKalmanMeasureMat(8, 1, CV_32F, cv::Scalar::all(0));//(rho, theta, delta_rho, delta_theta)
  //int    laneKalmanIdx     = 0;    //Marker of start kalmam
  float dt =0.1;
  //InitLaneKalmanFilter(laneKalmanFilter, laneKalmanMeasureMat, laneKalmanIdx,dt);
  //Sunny load motion input
  vector<float> motionW;
  vector<float> motionV;
  float m,v;
  stringstream s,s_;
  s << options.list_file_arg  << "wu.txt";
  ReadLinesfloat(s.str().c_str(), &motionW);
  s_ << options.list_file_arg  << "vf.txt";
  ReadLinesfloat(s_.str().c_str(), &motionV);
  int i;
  // process a single image
  if (options.image_file_given)
  {
    // elapsed time
    clock_t elapsed = 0;
    
    ProcessImage(motionV[i]*0.1,motionW[i]*0.1,isInit,options.image_file_arg, ipmInfo,cameraInfo, lanesConf, stoplinesConf,
                  options, NULL,NULL,NULL,elapsed,  0, &line_state ,&measurement, &lane_state,&center_line);
    double elapsedTime = static_cast<double>(elapsed) / CLOCKS_PER_SEC;
    MSG("Total time %f secs for 1 image = %f Hz", elapsedTime,
        1. / elapsedTime);
    i++;
  }

  // process a list of images
  if (options.list_file_given)
  {
    // get the path if exists
    string path = "";
    if (options.list_path_given)
      path = options.list_path_arg;

    // read file
    vector<string> files;

    ReadLines(options.list_file_arg, &files);


    int numImages = files.size();
    if (numImages<1)
      ERROR("File %s is empty", options.list_file_arg);
    else
    {
      // save results?
      ofstream outputFile,outputFile_point ,outputFile_aly;
      stringstream ss,sss,ssss;
      if (options.save_lanes_flag)
      {
        ss << options.list_file_arg << options.output_suffix_arg << ".txt";
        sss << options.list_file_arg << options.output_suffix_arg << "points.txt";
        ssss << options.list_file_arg << options.output_suffix_arg << "aly.txt";
        outputFile.open(ss.str().c_str(), ios_base::out);
        outputFile_point.open(sss.str().c_str(), ios_base::out);
        outputFile_aly.open(ssss.str().c_str(), ios_base::out);
      }

      // elapsed time
      clock_t elapsed = 0;
      // loop
      
      for (int i=0; i<numImages; ++i)
      {
        //if(i==0)m=0;
        //else m=motionW[i]-motionW[i-1];
        string imageFile = path + files[i];
        
        MSG("Processing image: %s", imageFile.c_str());
        
        ProcessImage(motionV[i]*0.1,motionW[i]*0.1, isInit,imageFile.c_str(),ipmInfo, cameraInfo, lanesConf, stoplinesConf,
                     options, &outputFile, &outputFile_point, &outputFile_aly,i, &elapsed,&line_state,&measurement, &lane_state, &center_line);

      }
      double elapsedTime = static_cast<double>(elapsed) / CLOCKS_PER_SEC;
      MSG("Total time %f secs for %d images = %f Hz",
          elapsedTime, numImages, numImages / elapsedTime);

      // close results file (if open)
      if (options.save_lanes_flag)
      {
        outputFile.close();
        MSG("Results written to %s", ss.str().c_str());
      }
    }
  }

  return 0;
}
void LoadMotion( char* path, vector<float> motion, vector<string> lines){

}


} // namespace LaneDetector



using LaneDetector::Process;

// main entry point
int main(int argc, char** argv)
{
  return Process(argc, argv);
}
