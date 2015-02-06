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
  std::string scmod = cl.follow("", "-cmod");
  std::string scam = cl.follow("NONE", "-cam");
  std::string sDir = cl.follow("NONE", "-dir");     // dir or the log file

  if ( scmod == "NONE" || scam == "NONE" || sDir =="NONE") {
    Help();
    return false;
  }

  hal::Camera camera = hal::Camera(scam.c_str());
  std::shared_ptr<pb::ImageArray> images = pb::ImageArray::Create();

  const unsigned int width = camera.Width();
  const unsigned int height = camera.Height();

  calibu::CameraRig rig = calibu::ReadXmlRig(scmod);
  ELASStereo elas(rig, width, height);
  elas.InitELAS();

  int nFrame = 0;
  bool bSaveDepth = false;
  // now process
  while (camera.Capture(*images))
  {
    //    memcpy(imPtr(elas.m_I1, 0, 0), images->at(0)->data(), elas.m_width * elas.m_height);
    //    memcpy(imPtr(elas.m_I2, 0, 0), images->at(1)->data(), elas.m_width * elas.m_height);
    //    elas.Run();

    // -----
    if (bSaveDepth) {
      char cNum[25];
      sprintf(cNum, "%05d", nFrame);
      std::string sNum(cNum);
      std::string sFileNameLeft = sDir + sNum + "-Depth.pdm";
      WritePDM(sFileNameLeft, elas.m_hDepth);
    }

    printf("finish frame: %d", nFrame);
    nFrame++;
  }

  return 0;
}



