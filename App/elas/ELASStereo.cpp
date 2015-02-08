#include "ELASStereo.h"


ELASStereo::ELASStereo(calibu::CameraRig& rig, const unsigned int width,
                       const unsigned int height)
  : m_dDisparity(width, height), m_dDepth(width, height) {
  if (rig.cameras.size() != 2) {
    std::cerr << "Two camera models are required to run this program!"
              << std::endl;
    exit(1);
  }

  m_width = width;
  m_height = height;

  Eigen::Matrix3f CamModel0 = rig.cameras[0].camera.K().cast<float>();
  Eigen::Matrix3f CamModel1 = rig.cameras[1].camera.K().cast<float>();

  roo::ImageIntrinsics camMod[] = {
    {CamModel0(0, 0), CamModel0(1, 1), CamModel0(0, 2), CamModel0(1, 2)},
    {CamModel1(0, 0), CamModel1(1, 1), CamModel1(0, 2), CamModel1(1, 2)}};

  m_Kl = camMod[0][0].Matrix();

  // print selected camera model
  std::cout << "Camera Model used: " << std::endl << camMod[0][0].Matrix()
      << std::endl;

  Eigen::Matrix3d RDFvision;
  RDFvision << 1, 0, 0, 0, 1, 0, 0, 0, 1;
  Eigen::Matrix3d RDFrobot;
  RDFrobot << 0, 1, 0, 0, 0, 1, 1, 0, 0;
  Eigen::Matrix4d T_vis_ro = Eigen::Matrix4d::Identity();
  T_vis_ro.block<3, 3>(0, 0) = RDFvision.transpose() * RDFrobot;
  Eigen::Matrix4d T_ro_vis = Eigen::Matrix4d::Identity();
  T_ro_vis.block<3, 3>(0, 0) = RDFrobot.transpose() * RDFvision;

  const Sophus::SE3d T_rl =
      T_rlFromCamModelRDF(rig.cameras[0], rig.cameras[1], RDFvision);
  m_baseline = T_rl.translation().norm();

  std::cout << "Baseline is: " << m_baseline << std::endl;
}

bool ELASStereo::InitELAS() {

  m_hDisparity1 = cv::Mat(m_height, m_width, CV_32FC1);
  m_hDisparity2 = cv::Mat(m_height, m_width, CV_32FC1);
  m_hDepth = cv::Mat(m_height, m_width, CV_32FC1);

  // ELAS image format
  m_I1 = new image<uchar>(m_width, m_height);
  m_I2 = new image<uchar>(m_width, m_height);
}

void ELASStereo::Run(std::string sLeftName, std::string sRightName) {
  // load images
  m_I1 = loadPGM(sLeftName.c_str());
  m_I2 = loadPGM(sRightName.c_str());

  std::cout << "[Run] before Processing: " << sLeftName << std::endl;

  // get image width and height
  int32_t width = m_I1->width();
  int32_t height = m_I1->height();

  // allocate memory for disparity images
  const int32_t dims[3] = {width, height, width};  // bytes per line = width

  // process
  Elas::parameters param;
  param.postprocess_only_left = false;
  Elas elas(param);
  elas.process(m_I1->data, m_I2->data, (float*)m_hDisparity1.data,
               (float*)m_hDisparity2.data, dims);

  // -----
  // upload disparity to GPU
  m_dDisparity.MemcpyFromHost((float*)m_hDisparity1.data);

  // convert disparity to depth
  roo::Disp2Depth(m_dDisparity, m_dDepth, m_Kl(0, 0), m_baseline);

  // download depth from GPU
  m_dDepth.MemcpyToHost(m_hDepth.data);
}


void ELASStereo::Run() {
  // get image width and height
  int32_t width = m_I1->width();
  int32_t height = m_I1->height();

  // allocate memory for disparity images
  const int32_t dims[3] = {width, height, width};  // bytes per line = width

  // process
  Elas::parameters param;
  param.postprocess_only_left = false;
  Elas elas(param);
  elas.process(m_I1->data, m_I2->data, (float*)m_hDisparity1.data,
               (float*)m_hDisparity2.data, dims);

  // -----
  // upload disparity to GPU
  m_dDisparity.MemcpyFromHost((float*)m_hDisparity1.data);

  // convert disparity to depth
  roo::Disp2Depth(m_dDisparity, m_dDepth, m_Kl(0, 0), m_baseline);

  // download depth from GPU
  m_dDepth.MemcpyToHost(m_hDepth.data);
}
