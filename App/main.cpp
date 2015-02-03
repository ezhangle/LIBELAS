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
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include "GetPot"

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


void process (const char* file_1,const char* file_2, bool bSaveDepthImages) {

  // load images
  image<uchar> *I1,*I2;
  I1 = loadPGM(file_1);
  I2 = loadPGM(file_2);

  // check for correct size
  if (I1->width()<=0 || I1->height() <=0 || I2->width()<=0 || I2->height() <=0 ||
      I1->width()!=I2->width() || I1->height()!=I2->height()) {
    cout << "ERROR: Images must be of same size, but" << endl;
    cout << "       I1: " << I1->width() <<  " x " << I1->height() <<
            ", I2: " << I2->width() <<  " x " << I2->height() << endl;
    delete I1;
    delete I2;
    return;
  }

  // get image width and height
  int32_t width  = I1->width();
  int32_t height = I1->height();

  // allocate memory for disparity images
  const int32_t dims[3] = {width,height,width}; // bytes per line = width
  float* D1_data = (float*)malloc(width*height*sizeof(float));
  float* D2_data = (float*)malloc(width*height*sizeof(float));

  double dTime = DDTR::_Tic();

  // process
  Elas::parameters param;
  param.postprocess_only_left = false;
  Elas elas(param);
  elas.process(I1->data,I2->data,D1_data,D2_data,dims);

  // find maximum disparity for scaling output disparity images to [0..255]
  float disp_max = 0;
  for (int32_t i=0; i<width*height; i++) {
    if (D1_data[i]>disp_max) disp_max = D1_data[i];
    if (D2_data[i]>disp_max) disp_max = D2_data[i];
  }

  // show results
  cv::Mat LGray(height, width, CV_8U, I1->data);
  //  cv::Mat RGray(height, width, CV_8U, I2->data);
  cv::imshow("left gray", LGray);
  //  cv::imshow("Reft gray", RGray);

  cv::Mat lDepth(height, width, CV_32F, D1_data);
  //  cv::Mat rDepth(height, width, CV_32F, D1_data);
  ShowVisableDepth("left depth", lDepth);
  //  ShowVisableDepth("right depth", rDepth);
  //  cv::waitKey(1);

  if(bSaveDepthImages)
  {
    char output_1[1024];
    char output_2[1024];
    strncpy(output_1,file_1,strlen(file_1)-4);
    strncpy(output_2,file_2,strlen(file_2)-4);

    string sFileNameLeft = string(output_1) + "-Depth.pdm";
    string sFileNameRight = string(output_2) + "-Depth.pdm";

    cv::Mat left(height, width, CV_32F, D1_data);
    cv::Mat right(height, width, CV_32F, D2_data);

    WritePDM(sFileNameLeft, left );
    WritePDM(sFileNameRight, right );
  }

  cout << "[Process] elas done! use time: "<<DDTR::_Toc(dTime) << endl;

  // free memory
  delete I1;
  delete I2;
  free(D1_data);
  free(D2_data);
}

int main (int argc, char** argv)
{
  GetPot cl( argc, argv );
  std::string sLeftDir = cl.follow( "NONE", "-l" );
  std::string sRightDir = cl.follow("NONE", "-r");

  if(sLeftDir == "NONE" || sRightDir == "NONE" )
  {
    std::cerr<<"Error! Please input valid arguements. e.g."
               "-l /Users/luma/Code/DataSet/LoopStereo/"<<
               "-l /Users/luma/Code/DataSet/LoopStereo/"<<std::endl;
    return false;
  }

  std::vector<std::string> m_vLeftImgPaths = ScanDir(sLeftDir.c_str(), "Left");
  std::vector<std::string> m_vRightImgPaths = ScanDir(sLeftDir.c_str(), "Right");

  // now process
  while(m_vLeftImgPaths.size() == m_vRightImgPaths.size() && m_vLeftImgPaths.size()!=0)
  {
    string sLeftName = sLeftDir + m_vLeftImgPaths[0];
    string sRightName =  sRightDir + m_vRightImgPaths[0];
    process(sLeftName.c_str(), sRightName.c_str() , false);
    m_vLeftImgPaths.erase(m_vLeftImgPaths.begin());
    m_vRightImgPaths.erase(m_vRightImgPaths.begin());
  }

  return 0;
}


