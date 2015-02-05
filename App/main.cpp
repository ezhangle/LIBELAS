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

int main(int argc, char** argv) {
  GetPot cl(argc, argv);

  std::string sLeftDir = cl.follow("NONE", "-l");
  std::string sRightDir = cl.follow("NONE", "-r");
  std::string scmod = cl.follow("", "-cmod");
  std::string sOutDir = cl.follow("NONE", "-out");

  if (sLeftDir == "NONE" || sRightDir == "NONE" ||
      scmod == "NONE" /*|| sOutDir == "NONE"*/) {
    std::cerr << "Error! Please input valid arguements. e.g."
                 "-l /Users/luma/Code/DataSet/LoopStereo/"
              << "-l /Users/luma/Code/DataSet/LoopStereo/"
              << "-cmod /Users/luma/Code/DataSet/LoopStereo/cameras.xml"
              << "-o /Users/luma/Code/DataSet/LoopStereo/Depth/" << std::endl;
    return false;
  }

  calibu::CameraRig rig = calibu::ReadXmlRig(scmod);
  ELASStereo elas(rig, 640, 480);
  elas.InitELAS(sLeftDir, sRightDir);

  while (elas.m_vLeftPaths.size() == elas.m_vRightPaths.size() &&
         elas.m_vLeftPaths.size() != 0)
  {
    elas.Run(sLeftDir, sRightDir);
  }

  //  delete elas.m_I1;
  //  delete elas.m_I2;

}
