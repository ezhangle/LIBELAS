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

#include "ELASStereo.h"

using namespace std;

void Help()
{
  std::cerr << "Error! Please input valid arguements. e.g."
               "-l /Users/luma/Code/DataSet/LoopStereo/"
            << "-l /Users/luma/Code/DataSet/LoopStereo/"
            << "-cmod /Users/luma/Code/DataSet/LoopStereo/cameras.xml"
            << "-o /Users/luma/Code/DataSet/LoopStereo/Depth/" << std::endl;
}

int main(int argc, char** argv) {
  GetPot cl(argc, argv);

  std::string sLeftDir = cl.follow("NONE", "-l");
  std::string sRightDir = cl.follow("NONE", "-r");
  std::string scmod = cl.follow("", "-cmod");
  std::string sOutDir = cl.follow("NONE", "-out");

  if (sLeftDir == "NONE" || sRightDir == "NONE" ||
      scmod == "NONE" /*|| sOutDir == "NONE"*/) {
    Help();
    return false;
  }

  calibu::CameraRig rig = calibu::ReadXmlRig(scmod);
  ELASStereo elas(rig, 640, 480);
  elas.InitELAS();

  // --- now process
  std::vector<std::string> m_vLeftPaths;
  std::vector<std::string> m_vRightPaths;
  m_vLeftPaths = ScanDir(sLeftDir.c_str(), "Left", ".pgm");
  m_vRightPaths = ScanDir(sRightDir.c_str(), "Right", ".pgm");

  bool bSaveDepth = true;

  while (m_vLeftPaths.size() == m_vRightPaths.size() &&
         m_vLeftPaths.size() != 0)
  {
    std::string sLeftName  = sLeftDir + m_vLeftPaths[0];
    std::string sRightName  = sRightDir + m_vRightPaths[0];

    elas.Run(sLeftName, sRightName);

    //    ShowHeatDepthMat("depth image", elas.m_hDepth);
    //    cv::waitKey(1);

    // -----
    if (bSaveDepth) {
      char output_1[1024];
      strncpy(output_1, sLeftName.c_str(), strlen(sLeftName.c_str()) - 4);
      std::string sFileNameLeft = std::string(output_1) + "-Depth.pdm";
      WritePDM(sFileNameLeft, elas.m_hDepth);
    }

    // -----
    // free memory
    m_vLeftPaths.erase(m_vLeftPaths.begin());
    m_vRightPaths.erase(m_vRightPaths.begin());
  }

  return 0;
}

