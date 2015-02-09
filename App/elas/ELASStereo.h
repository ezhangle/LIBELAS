#ifndef ELASSTEREO_H
#define ELASSTEREO_H

#include <iostream>
#include <LIBELAS/src/elas.h>
#include <LIBELAS/src/image.h>
#include "Timer.h"
#include <dirent.h>
#include <string>
#include <vector>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs/imgcodecs_c.h>
#include <Eigen/Core>
#include <HAL/Utils/GetPot>
#include <HAL/Camera/CameraDevice.h>
#include <PbMsgs/Image.h>
#include <calibu/cam/CameraRig.h>
#include <calibu/cam/CameraXml.h>
#include <kangaroo/kangaroo.h>

inline std::vector<std::string> ScanDir(const char* cDir, std::string sKeyword) {
  DIR* dir;
  struct dirent* ent;
  std::vector<std::string> vFileNames;
  if ((dir = opendir(cDir)) != NULL) {
    std::string sFileName;

    while ((ent = readdir(dir)) != NULL) {
      sFileName = std::string(ent->d_name);
      if (sFileName.find(sKeyword) != std::string::npos) {
        vFileNames.push_back(sFileName);
      }
    }
    closedir(dir);
  } else {
    std::cout << "Error! Could not open directory " << std::string(cDir)
              << std::endl;
  }

  std::cout << "[ScanDirectory] Found " << vFileNames.size()
            << " files with keyword: " << sKeyword
            << " in dir:" << std::string(cDir) << std::endl;
  return vFileNames;
}

inline std::vector<std::string> ScanDir(
    const char*     cDir,
    std::string     sKeyword,
    std::string     sFormat)
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
        if(sFileName.find(sFormat)!=std::string::npos)
        {
          vFileNames.push_back(sFileName);
        }
      }
    }
    closedir (dir);
  }
  else
  {
    std::cout<<"Error! Could not open directory "<<std::string(cDir)<<std::endl;
  }

  std::cout<<"[ScanDirectory] Found "<<vFileNames.size()<<
             " files with keyword: "<<sKeyword<<", format: "<<sFormat<<std::endl;
  return vFileNames;
}

inline void ShowVisableDepth(std::string sWndName, const cv::Mat Depth) {
  double min;
  double max;
  cv::minMaxIdx(Depth, &min, &max);
  cv::Mat adjMap;
  cv::convertScaleAbs(Depth, adjMap, 255 / max);
  cv::imshow(sWndName, adjMap);
  cv::waitKey(1);
}

inline bool WritePDM(const std::string& FileName, const cv::Mat& Image) {
  if (Image.type() != CV_32FC1) {
    return false;
  }

  std::ofstream pFile(FileName.c_str(), std::ios::out | std::ios::binary);
  pFile << "P7" << std::endl;
  pFile << Image.cols << " " << Image.rows << std::endl;
  const unsigned int Size = Image.elemSize1() * Image.rows * Image.cols;
  pFile << 4294967295 << std::endl;
  pFile.write((const char*)Image.data, Size);
  pFile.close();

  std::cout << "[WritePDM] save pdm success, height:" << Image.rows
            << ", width:" << Image.cols << ", file: " << FileName << std::endl;
  return true;
}

inline void ShowHeatDepthMat(std::string sName, cv::Mat Depth) {
  double min;
  double max;
  cv::minMaxIdx(Depth, &min, &max);

  if (max > 100.d) {
    max = 100.d;
  }

  cv::Mat adjMap;
  Depth.convertTo(adjMap, CV_8UC1, 255 / (max - min), -min);
  cv::Mat falseColorsMap;
  cv::applyColorMap(adjMap, falseColorsMap, cv::COLORMAP_WINTER);

  std::cout << "max depth:" << max << ", min depth:" << min << std::endl;
  cv::imshow(sName, falseColorsMap);
  cv::waitKey(1);
}

inline Sophus::SE3d T_rlFromCamModelRDF(const calibu::CameraModelAndTransform& lcmod,
                                 const calibu::CameraModelAndTransform& rcmod,
                                 const Eigen::Matrix3d& targetRDF) {
  // Transformation matrix to adjust to target RDF
  Eigen::Matrix4d Tadj[2] = {Eigen::Matrix4d::Identity(),
                             Eigen::Matrix4d::Identity()};
  Tadj[0].block<3, 3>(0, 0) = targetRDF.transpose() * lcmod.camera.RDF();
  Tadj[1].block<3, 3>(0, 0) = targetRDF.transpose() * rcmod.camera.RDF();

  // Computer Poses in our adjust coordinate system
  const Eigen::Matrix4d T_lw_ = Tadj[0] * lcmod.T_wc.matrix().inverse();
  const Eigen::Matrix4d T_rw_ = Tadj[1] * rcmod.T_wc.matrix().inverse();

  // Computer transformation to right camera frame from left
  const Eigen::Matrix4d T_rl = T_rw_ * T_lw_.inverse();

  return Sophus::SE3d(T_rl.block<3, 3>(0, 0), T_rl.block<3, 1>(0, 3));
}

class ELASStereo {
 public:
  ELASStereo(calibu::CameraRig& rig, const unsigned int width,
             const unsigned int height);

  ~ELASStereo() {
    delete m_I1;
    delete m_I2;
  }

  bool InitELAS();

  void Run(std::string sLeftDir, std::string sRightName);

  void Run();

 public:
  Eigen::Matrix3d m_Kl;
  unsigned int m_width;
  unsigned int m_height;
  double m_baseline;

  roo::Image<float, roo::TargetDevice, roo::Manage> m_dDisparity;
  roo::Image<float, roo::TargetDevice, roo::Manage> m_dDepth;
  cv::Mat m_hDisparity1;
  cv::Mat m_hDisparity2;
  cv::Mat m_hDepth;

  // ELAS image format
  ELAS::image<uchar>* m_I1;
  ELAS::image<uchar>* m_I2;
  Elas* m_pelas;
};

#endif  // ELASSTEREO_H
