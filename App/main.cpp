/*
Copyright 2011. All rights reserved.
Institute of Measurement and Control Systems
Karlsruhe Institute of Technology, Germany

This file is part of libelas.
Authors: Andreas Geiger

libelas is free software; you can redistribute it and/or modify it under the
terms of the GNU General Public License as published by the Free Software
Foundation; either version 3 of the License, or any later version.

libelas is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
libelas; if not, write to the Free Software Foundation, Inc., 51 Franklin
Street, Fifth Floor, Boston, MA 02110-1301, USA 
*/

// Demo program showing how libelas can be used, try "./elas -h" for help

#include <iostream>
#include <LIBELAS/src/elas.h>
#include <LIBELAS/src/image.h>
#include "Timer.h"
#include <dirent.h>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <Eigen/Core>
#include <HAL/Utils/GetPot>
#include <HAL/Camera/CameraDevice.h>
#include <calibu/Calibu.h>

#include <HAL/Camera/CameraDevice.h>
#include <PbMsgs/Image.h>
#include <calibu/cam/CameraRig.h>
#include <calibu/cam/CameraXml.h>
#include <kangaroo/ImageIntrinsics.h>
#include <kangaroo/kangaroo.h>
#include <opencv2/imgcodecs/imgcodecs_c.h>

using namespace std;

// compute disparities of pgm image input pair file_1, file_2

std::vector<std::string> ScanDir(
    const char*     cDir,
    std::string     sKeyword)
{
  DIR *dir;
  struct dirent *ent;
  std::vector<std::string> vFileNames;
  if ((dir = opendir (cDir)) != NULL)
  {
    std::string sFileName;

    while ((ent = readdir (dir)) != NULL)
    {
      sFileName = std::string(ent->d_name);
      if(sFileName.find(sKeyword)!=std::string::npos)
      {
        vFileNames.push_back(sFileName);
      }
    }
    closedir (dir);
  }
  else
  {
    std::cout<<"Error! Could not open directory "<<std::string(cDir)<<std::endl;
  }

  std::cout<<"[ScanDirectory] Found "<<vFileNames.size()<<
             " files with keyword: "<<sKeyword<<" in dir:"<<std::string(cDir)<<std::endl;
  return vFileNames;
}

void ShowVisableDepth(
    std::string     sWndName,
    const cv::Mat   Depth)
{
  double min;
  double max;
  cv::minMaxIdx(Depth, &min, &max);
  cv::Mat adjMap;
  cv::convertScaleAbs(Depth, adjMap, 255 / max);
  cv::imshow(sWndName, adjMap);
  cv::waitKey(1);
}

bool WritePDM( const std::string&  FileName, const cv::Mat& Image )
{
  if( Image.type() != CV_32FC1 ) {
    return false;
  }

  std::ofstream pFile( FileName.c_str(), std::ios::out | std::ios::binary );
  pFile << "P7" << std::endl;
  pFile << Image.cols << " " << Image.rows << std::endl;
  const unsigned int Size = Image.elemSize1() * Image.rows * Image.cols;
  pFile << 4294967295 << std::endl;
  pFile.write( (const char*)Image.data, Size );
  pFile.close();

  std::cout<<"[WritePDM] save pdm success, height:"<<Image.rows<<", width:"<<Image.cols<<", file: "
          <<FileName<<std::endl;
  return true;

}

void ShowHeatDepthMat(std::string sName, cv::Mat Depth)
{
  double min;
  double max;
  cv::minMaxIdx(Depth, &min, &max);

  if(max >100.d)
  {
    max = 100.d;
  }

  cv::Mat adjMap;
  Depth.convertTo(adjMap, CV_8UC1, 255 / (max-min), -min);
  cv::Mat falseColorsMap;
  cv::applyColorMap(adjMap, falseColorsMap, cv::COLORMAP_WINTER);

  //  std::string sString = "Min:"+std::to_string(min)+",Max:"+std::to_string(max);
  //  cv::putText(falseColorsMap, sString, cvPoint(50,50), cv::FONT_HERSHEY_DUPLEX, 1.0,
  //              cvScalar(0,0,255), 2, CV_AA);

  std::cout<<"max depth:"<<max<<", min depth:"<<min<<std::endl;
  cv::imshow(sName, falseColorsMap);
  cv::waitKey(1);
}

Sophus::SE3d T_rlFromCamModelRDF(
    const calibu::CameraModelAndTransform& lcmod,
    const calibu::CameraModelAndTransform& rcmod,
    const Eigen::Matrix3d& targetRDF)
{
  // Transformation matrix to adjust to target RDF
  Eigen::Matrix4d Tadj[2] = {Eigen::Matrix4d::Identity(),Eigen::Matrix4d::Identity()};
  Tadj[0].block<3,3>(0,0) = targetRDF.transpose() * lcmod.camera.RDF();
  Tadj[1].block<3,3>(0,0) = targetRDF.transpose() * rcmod.camera.RDF();

  // Computer Poses in our adjust coordinate system
  const Eigen::Matrix4d T_lw_ = Tadj[0] * lcmod.T_wc.matrix().inverse();
  const Eigen::Matrix4d T_rw_ = Tadj[1] * rcmod.T_wc.matrix().inverse();

  // Computer transformation to right camera frame from left
  const Eigen::Matrix4d T_rl = T_rw_ * T_lw_.inverse();

  return Sophus::SE3d(T_rl.block<3,3>(0,0), T_rl.block<3,1>(0,3) );
}

int main (int argc, char** argv) {

  GetPot cl( argc, argv );

  std::string sLeftDir = cl.follow( "NONE", "-l" );
  std::string sRightDir = cl.follow("NONE", "-r");
  std::string scmod = cl.follow("","-cmod");
  std::string sOutDir = cl.follow("NONE", "-out");

  if(sLeftDir == "NONE" || sRightDir == "NONE" || scmod == "NONE" /*|| sOutDir == "NONE"*/)
  {
    std::cerr<<"Error! Please input valid arguements. e.g."
               "-l /Users/luma/Code/DataSet/LoopStereo/"<<
               "-l /Users/luma/Code/DataSet/LoopStereo/"<<
               "-cmod /Users/luma/Code/DataSet/LoopStereo/cameras.xml"<<
               "-o /Users/luma/Code/DataSet/LoopStereo/Depth/" <<std::endl;
    return false;
  }

  const calibu::CameraRig rig = calibu::ReadXmlRig(scmod);

  if( rig.cameras.size() != 2 ) {
    std::cerr << "Two camera models are required to run this program!" << std::endl;
    exit(1);
  }

  Eigen::Matrix3f CamModel0 = rig.cameras[0].camera.K().cast<float>();
  Eigen::Matrix3f CamModel1 = rig.cameras[1].camera.K().cast<float>();

  roo::ImageIntrinsics camMod[] = {
    {CamModel0(0,0),CamModel0(1,1),CamModel0(0,2),CamModel0(1,2)},
    {CamModel1(0,0),CamModel1(1,1),CamModel1(0,2),CamModel1(1,2)}
  };

  const Eigen::Matrix3d& Kl = camMod[0][0].Matrix();

  // print selected camera model
  std::cout << "Camera Model used: " << std::endl << camMod[0][0].Matrix() << std::endl;

  Eigen::Matrix3d RDFvision;RDFvision<< 1,0,0,  0,1,0,  0,0,1;
  Eigen::Matrix3d RDFrobot; RDFrobot << 0,1,0,  0,0, 1,  1,0,0;
  Eigen::Matrix4d T_vis_ro = Eigen::Matrix4d::Identity();
  T_vis_ro.block<3,3>(0,0) = RDFvision.transpose() * RDFrobot;
  Eigen::Matrix4d T_ro_vis = Eigen::Matrix4d::Identity();
  T_ro_vis.block<3,3>(0,0) = RDFrobot.transpose() * RDFvision;

  const Sophus::SE3d T_rl = T_rlFromCamModelRDF(rig.cameras[0], rig.cameras[1], RDFvision);
  const double baseline = T_rl.translation().norm();

  std::cout << "Baseline is: " << baseline << std::endl;

  /// --------------------------------------------------------------------------
  // now read images
  std::vector<std::string> vLeftImgPaths = ScanDir(sLeftDir.c_str(), "Left");
  std::vector<std::string> vRightImgPaths = ScanDir(sLeftDir.c_str(), "Right");

  string sLeftName = sLeftDir + vLeftImgPaths[0];
  cv::Mat TestMat = cv::imread(sLeftName, CV_LOAD_IMAGE_COLOR);

  /// --------------------------------------------------------------------------
  // prepare the system for the task
  const unsigned int width = TestMat.cols;
  const unsigned int height = TestMat.rows;

  roo::Image<float, roo::TargetDevice, roo::Manage> dDisparity(width, height);
  roo::Image<float, roo::TargetDevice, roo::Manage> dDepth(width, height);
  cv::Mat hDisparity1 = cv::Mat(height, width, CV_32FC1);
  cv::Mat hDisparity2 = cv::Mat(height, width, CV_32FC1);
  cv::Mat hDepth = cv::Mat(height, width, CV_32FC1);

  // ELAS image format
  image<uchar> *I1 = new image<uchar>(width, height);
  image<uchar> *I2 = new image<uchar>(width, height);

  // allocate memory for disparity images
  int32_t dims[3];
  dims[0] = width; // bytes per line = width
  dims[1] = height; // bytes per line = width
  dims[2] = width; // bytes per line = width

  // set up ELAS process
  Elas::parameters param;
  param.postprocess_only_left = false;
  Elas elas(param);

  // now process
  while(vLeftImgPaths.size() == vRightImgPaths.size() && vLeftImgPaths.size()!=0)
  {
    string sLeftName = sLeftDir + vLeftImgPaths[0];
    string sRightName =  sRightDir + vRightImgPaths[0];

    std::cout<<"Processing: "<<sLeftName<<std::endl;

    // load images
    I1 = loadPGM(sLeftName.c_str());
    I2 = loadPGM(sRightName.c_str());

    // get image width and height
    int32_t width  = I1->width();
    int32_t height = I1->height();

    // allocate memory for disparity images
    const int32_t dims[3] = {width,height,width}; // bytes per line = width

    // process
    elas.process(I1->data, I2->data, (float*)hDisparity1.data,(float*)hDisparity2.data, dims);

    // -----
    // upload disparity to GPU
    dDisparity.MemcpyFromHost((float*)hDisparity1.data);

    // convert disparity to depth
    roo::Disp2Depth(dDisparity, dDepth, Kl(0,0), baseline);

    // download depth from GPU
    dDepth.MemcpyToHost( hDepth.data );

    bool bSaveDepthImages = true;

    if(bSaveDepthImages)
    {
      char output_1[1024];
      strncpy(output_1, sLeftName.c_str(), strlen(sLeftName.c_str())-4);
      string sFileNameLeft = string(output_1) + "-Depth.pdm";
      WritePDM(sFileNameLeft, hDepth );
    }

    //    ShowHeatDepthMat("depth image", hDepth);
    //    cv::waitKey(1);

    // free memory
    vLeftImgPaths.erase(vLeftImgPaths.begin());
    vRightImgPaths.erase(vRightImgPaths.begin());
  }

  delete I1;
  delete I2;


  cout << "... done!" << endl;

  return 0;
}
