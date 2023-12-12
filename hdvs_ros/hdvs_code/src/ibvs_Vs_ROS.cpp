#include <ros/ros.h>
#include <iostream>

#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>

//#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/treefksolverpos_recursive.hpp>
#include <kdl/treeiksolverpos_online.hpp>
#include <kdl/treeiksolvervel_wdls.hpp>
#include <kdl/treejnttojacsolver.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <visp/vpImage.h>
#include <visp/vpImageConvert.h>
#include <visp/vpKalmanFilter.h>
#include <visp/vpSimulatorCamera.h>
#include <visp3/core/vpCameraParameters.h>
#include <visp3/detection/vpDetectorAprilTag.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/gui/vpPlot.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/robot/vpRobotFranka.h>
#include <visp3/sensor/vpRealSense2.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/visual_features/vpFeaturePoint.h>
#include <visp3/vs/vpServo.h>
#include <visp3/vs/vpServoDisplay.h>

#include <visp/vp1394TwoGrabber.h>
#include <visp/vpDot2.h>
#include <visp/vpFeatureDepth.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpVelocityTwistMatrix.h>
#include <visp_ros/vpROSRobot.h>

#include <visp_ros/vpROSGrabber.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/LU>
#include <chrono>

#include <deque>
#include <iostream>
#include <mutex>
#include <vector>

//#if defined(VISP_HAVE_REALSENSE2) && (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11) &&                                    \
//   (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI)) && defined(VISP_HAVE_FRANKA)

std::mutex robot_1_joint_values_mtx;
KDL::JntArray *robot_1_joint_positions_ptr;
std::vector<double> robot_1_joint_positions;
std::vector<double> robot_1_joint_velocities;

std::vector<double> lower_joint_limits = { -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973 };
std::vector<double> upper_joint_limits = { 2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973 };

void display_point_trajectory(const vpImage<unsigned char> &I, const std::vector<vpImagePoint> &vip,
                              std::vector<vpImagePoint> *traj_vip)
{
  for (size_t i = 0; i < vip.size(); i++)
  {
    if (traj_vip[i].size())
    {
      // Add the point only if distance with the previous > 1 pixel
      if (vpImagePoint::distance(vip[i], traj_vip[i].back()) > 1.)
      {
        traj_vip[i].push_back(vip[i]);
      }
    }
    else
    {
      traj_vip[i].push_back(vip[i]);
    }
  }
  for (size_t i = 0; i < vip.size(); i++)
  {
    for (size_t j = 1; j < traj_vip[i].size(); j++)
    {
      vpDisplay::displayLine(I, traj_vip[i][j - 1], traj_vip[i][j], vpColor::green, 2);
    }
  }
}

void robot_1_joint_state_callback(const sensor_msgs::JointState::ConstPtr &msg)
{
  robot_1_joint_values_mtx.lock();
  if (robot_1_joint_positions_ptr)
  {
    std::vector<double> positions = msg->position;
    std::vector<double> velocities = msg->velocity;
    for (int i = 0; i < positions.size(); i++)
    {
      (*robot_1_joint_positions_ptr)(i) = positions[i];  // This is used by KDL
      robot_1_joint_positions[i] = positions[i];         // This is used by VISP
      robot_1_joint_velocities[i] = velocities[i];
    }
  }
  robot_1_joint_values_mtx.unlock();
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "erl_vs_node");
  ros::NodeHandle nh;
  ros::Rate loop_rate(40);
  ros::Publisher robot_1_joint_vel_pub =
      nh.advertise<std_msgs::Float64MultiArray>("robot_1/panda_arm_controller/command", 1000);
  ros::Subscriber robot_1_joint_state_sub = nh.subscribe("robot_1/joint_states", 1000, robot_1_joint_state_callback);

  double opt_tagSize = 0.120;
  std::string opt_robot_ip = "172.16.0.2";
  std::string opt_eMc_filename = "";
  bool display_tag = false;
  int opt_quad_decimate = 2;
  bool opt_verbose = false;
  bool opt_plot = true;
  bool opt_adaptive_gain_IBVS = true;
  bool opt_adaptive_gain_PVBS = false;
  bool opt_task_sequencing_IBVS = true;
  bool opt_task_sequencing_PBVS = true;
  double convergence_threshold = 0.00009;
  double convergence_threshold_t = 0.0005, convergence_threshold_tu = vpMath::rad(0.5);
  bool has_converged = false;
  bool has_converged_PBVS = false;

  vpColVector X(7u, 0.0);

  vpColVector q_dot(7u, 0.0);
  vpColVector q_dot_DLS(7u, 0.0);
  vpColVector q_dot_Decouple_DLS(7u, 0.0);

  vpMatrix J(6u, 7u);

  vpMatrix J_V(3u, 7u);
  vpMatrix J_V_DLS(7u, 3u);
  vpMatrix J_V_DLS_Null(3u, 3u);

  vpMatrix J_W(3u, 7u);
  vpMatrix J_W_DLS(7u, 3u);
  vpMatrix J_W_DLS_Null(3u, 3u);

  int landa_1 = 0.02;
  int landa_2 = 0.02;
  int i, j;

  vpMatrix Ie_seven;
  vpMatrix Ie_three;
  vpMatrix Ie_Six;

  Ie_seven.eye(7);
  Ie_three.eye(3);
  Ie_Six.eye(6);

  for (int i = 1; i < argc; i++)
  {
    if (std::string(argv[i]) == "--tag_size" && i + 1 < argc)
    {
      opt_tagSize = std::stod(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--ip" && i + 1 < argc)
    {
      opt_robot_ip = std::string(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--eMc" && i + 1 < argc)
    {
      opt_eMc_filename = std::string(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--verbose")
    {
      opt_verbose = true;
    }
    else if (std::string(argv[i]) == "--plot")
    {
      opt_plot = true;
    }
    else if (std::string(argv[i]) == "--adaptive_gain")
    {
      opt_adaptive_gain_IBVS = true;
    }
    else if (std::string(argv[i]) == "--task_sequencing")
    {
      opt_task_sequencing_IBVS = true;
    }
    else if (std::string(argv[i]) == "--adaptive_gain")
    {
      opt_adaptive_gain_PVBS = true;
    }
    else if (std::string(argv[i]) == "--task_sequencing")
    {
      opt_task_sequencing_PBVS = true;
    }
    else if (std::string(argv[i]) == "--quad_decimate" && i + 1 < argc)
    {
      opt_quad_decimate = std::stoi(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--no-convergence-threshold")
    {
      convergence_threshold = 0.;
    }
    else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h")
    {
      std::cout << argv[0] << " [--ip <default " << opt_robot_ip << ">] [--tag_size <marker size in meter; default "
                << opt_tagSize << ">] [--eMc <eMc extrinsic file>] "
                << "[--quad_decimate <decimation; default " << opt_quad_decimate
                << ">] [--adaptive_gain] [--plot] [--task_sequencing] [--no-convergence-threshold] [--verbose] "
                   "[--help] [-h]"
                << "\n";
      return EXIT_SUCCESS;
    }
  }
  KDL::Tree robot_1_tree;
  std::string robot_1_description;
  nh.param("robot_1/robot_description", robot_1_description, std::string());
  if (!kdl_parser::treeFromString(robot_1_description, robot_1_tree))
  {
    return 1;
  }

  KDL::SegmentMap::const_iterator robot_1_current_segment = robot_1_tree.getRootSegment();
  while (robot_1_current_segment->second.children.size() > 0)
  {
    robot_1_current_segment = robot_1_current_segment->second.children[0];
  }

  KDL::TreeJntToJacSolver robot_1_jacobian_solver(robot_1_tree);
  KDL::TreeFkSolverPos_recursive robot_1_fk_solver(robot_1_tree);
  // KDL::TreeFkSolverPos_recursive *robot_1_fk_pos_solver_ptr;
  // KDL::TreeIkSolverVel_wdls *robot_1_ik_vel_solver_ptr;
  // KDL::TreeIkSolverPos_Online *robot_1_ik_pos_solver_ptr;
  // std::vector<KDL::Segment> jointed_segments;

  // KDL::ChainIkSolverVel_pinv::ChainIkSolverVel();
  // (const Chain &chain, double eps = 0.00001, int maxiter = 150);

  robot_1_joint_values_mtx.lock();
  robot_1_joint_positions_ptr = new KDL::JntArray(robot_1_tree.getNrOfJoints());
  for (int i = 0; i < robot_1_tree.getNrOfJoints(); i++)
  {
    robot_1_joint_positions.push_back(0);
    robot_1_joint_velocities.push_back(0);
  }
  robot_1_joint_values_mtx.unlock();

  KDL::Jacobian robot_1_jacobian(robot_1_tree.getNrOfJoints());
  // robot_1_jacobian.changeBase();

  for (int i = 0; i < 5; i++)
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  vpROSRobot robot;

  try
  {
    vpImage<unsigned char> I;
    vpROSGrabber g;

    g.setImageTopic("robot_1/camera/image_raw");
    g.setRectify(false);
    g.open(I);

    // Get camera extrinsics
    vpPoseVector ePc;
    ePc.buildFrom(0.0365, 0, 0.054, M_PI, -M_PI_2, 0);

    vpHomogeneousMatrix eMc(ePc);
    vpCameraParameters cam;
    cam.initPersProjWithoutDistortion(557.89661, 550.29545, 318.59805, 243.64224);
    // cam.initPersProjWithoutDistortion(550.89661, 540.29545, 300.59805, 220.64224);  //bad calibration

#if defined(VISP_HAVE_X11)
    vpDisplayX dc(I, 10, 10, "Color image");
#elif defined(VISP_HAVE_GDI)
    vpDisplayGDI dc(I, 10, 10, "Color image");
#endif

    vpDetectorAprilTag::vpAprilTagFamily tagFamily = vpDetectorAprilTag::TAG_36h11;
    vpDetectorAprilTag::vpPoseEstimationMethod poseEstimationMethod = vpDetectorAprilTag::HOMOGRAPHY_VIRTUAL_VS;
    vpDetectorAprilTag detector(tagFamily);
    detector.setAprilTagPoseEstimationMethod(poseEstimationMethod);
    detector.setDisplayTag(display_tag);
    detector.setAprilTagQuadDecimate(opt_quad_decimate);

    // Servo
    vpHomogeneousMatrix cdMc_IBVS, cMo_IBVS, oMo_IBVS;

    // Desired pose used to compute the desired features
    vpHomogeneousMatrix cdMo_IBVS(vpTranslationVector(0, 0, opt_tagSize * 3),  // 3 times tag with along camera z axis
                                  vpRotationMatrix({ 1, 0, 0, 0, -1, 0, 0, 0, -1 }));

    // Create visual features
    std::vector<vpFeaturePoint> p(4), pd(4);  // We use 4 points

    // Define 4 3D points corresponding to the CAD model of the Apriltag
    std::vector<vpPoint> point(4);
    point[0].setWorldCoordinates(-opt_tagSize / 4.5, -opt_tagSize / 4.5, 0);
    point[1].setWorldCoordinates(opt_tagSize / 4.5, -opt_tagSize / 4.5, 0);
    point[2].setWorldCoordinates(opt_tagSize / 4.5, opt_tagSize / 4.5, 0);
    point[3].setWorldCoordinates(-opt_tagSize / 4.5, opt_tagSize / 4.5, 0);

    vpServo task_IBVS;

    // Add the 4 visual feature points
    for (size_t i = 0; i < p.size(); i++)
    {
      task_IBVS.addFeature(p[i], pd[i]);
    }
    task_IBVS.setServo(vpServo::EYEINHAND_CAMERA);
    task_IBVS.setInteractionMatrixType(vpServo::CURRENT);
    // robot.setJointImpedance({300,300,300,300,300,300,300});

    //--------------------------PBVS Servo-----------------------
    // Servo
    vpServo task_PBVS;
    vpHomogeneousMatrix cdMc_PBVS, cMo_PBVS, oMo_PBVS;

    // Desired pose used to compute the desired features
    vpHomogeneousMatrix cdMo_PBVS(vpTranslationVector(0, 0, opt_tagSize * 6.5),  // 3 times tag with along camera z axis
                                  vpRotationMatrix({ 1, 0, 0, 0, -1, 0, 0, 0, -1 }));

    cdMc_PBVS = cdMo_PBVS * cMo_PBVS.inverse();
    vpFeatureTranslation t(vpFeatureTranslation::cdMc);
    vpFeatureThetaU tu(vpFeatureThetaU::cdRc);
    t.buildFrom(cdMc_PBVS);
    tu.buildFrom(cdMc_PBVS);

    vpFeatureTranslation td(vpFeatureTranslation::cdMc);
    vpFeatureThetaU tud(vpFeatureThetaU::cdRc);

    task_PBVS.addFeature(t, td);
    task_PBVS.addFeature(tu, tud);
    task_PBVS.setServo(vpServo::EYEINHAND_CAMERA);
    task_PBVS.setInteractionMatrixType(vpServo::CURRENT);

    if (opt_adaptive_gain_PVBS)
    {
      vpAdaptiveGain lambda(1.5, 0.4, 30);  // lambda(0)=4, lambda(oo)=0.4 and lambda'(0)=30
      task_PBVS.setLambda(lambda);
    }
    else
    {
      task_PBVS.setLambda(0.5);
    }
    //-------------------------PBVS--------------------------

    if (opt_adaptive_gain_IBVS)
    {
      vpAdaptiveGain lambda(1.5, 0.4, 30);
      task_IBVS.setLambda(lambda);
    }
    else
    {
      task_IBVS.setLambda(0.5);
    }

    vpPlot *plotter = nullptr;
    int iter_plot = 0;

    if (opt_plot)
    {
      plotter = new vpPlot(3, static_cast<int>(250 * 2), 500, static_cast<int>(I.getWidth()) + 80, 10,
                           "Real time curves plotter");
      plotter->setTitle(0, "Visual features error");
      plotter->setTitle(1, "Camera velocities");
      plotter->setTitle(2, "Joints");
      // plotter->setTitle(3, "Sum Squire Error");

      plotter->initGraph(0, 8);
      plotter->initGraph(1, 6);
      plotter->initGraph(2, 7);
      // plotter->initGraph(3, 1);

      plotter->setLegend(0, 0, "error_feat_p1_x");
      plotter->setLegend(0, 1, "error_feat_p1_y");
      plotter->setLegend(0, 2, "error_feat_p2_x");
      plotter->setLegend(0, 3, "error_feat_p2_y");
      plotter->setLegend(0, 4, "error_feat_p3_x");
      plotter->setLegend(0, 5, "error_feat_p3_y");
      plotter->setLegend(0, 6, "error_feat_p4_x");
      plotter->setLegend(0, 7, "error_feat_p4_y");
      plotter->setLegend(1, 0, "vc_x");
      plotter->setLegend(1, 1, "vc_y");
      plotter->setLegend(1, 2, "vc_z");
      plotter->setLegend(1, 3, "wc_x");
      plotter->setLegend(1, 4, "wc_y");
      plotter->setLegend(1, 5, "wc_z");
      plotter->setLegend(2, 0, "J_1");
      plotter->setLegend(2, 1, "J_2");
      plotter->setLegend(2, 2, "J_3");
      plotter->setLegend(2, 3, "J_4");
      plotter->setLegend(2, 4, "J_5");
      plotter->setLegend(2, 5, "J_6");
      plotter->setLegend(2, 6, "J_7");
      // plotter->setLegend(3, 1, "error");
    }

    bool final_quit = false;
    bool send_velocities = false;
    bool servo_started = false;
    has_converged = false;
    has_converged_PBVS = false;

    std::vector<vpImagePoint> *traj_vip = nullptr;
    std::vector<vpImagePoint> *traj_corners = nullptr;  // To memorize point trajectory

    //---------------------------------------prediction------------------------------
    // types:: none, kalman-lowpass , lowpass , kalman
    std::string filter_type = "none";

    double lowpass_gain = 0.4;

    vpKalmanFilter kalman_filter(robot_1_tree.getNrOfJoints(), robot_1_tree.getNrOfJoints(), 1);
    kalman_filter.F.eye(robot_1_tree.getNrOfJoints());
    kalman_filter.H.eye(robot_1_tree.getNrOfJoints());
    kalman_filter.Q.eye(robot_1_tree.getNrOfJoints());
    kalman_filter.Q *= 0.05;
    kalman_filter.R.eye(robot_1_tree.getNrOfJoints());
    kalman_filter.R *= 0.5;
    //------------------------------------------------------------------------

    static double t_init_servo = vpTime::measureTimeMs();

    // robot.set_eMc(eMc);  // Set location of the camera wrt end-effector frame
    KDL::Frame camera_frame, marker_frame;
    KDL::Vector camera_origin, marker_origin;
    KDL::Rotation camera_orientation, marker_orientation;

    //--------------------------------------MAIN Control Loop(While)-----------------------
    while (ros::ok() && !has_converged && !final_quit)
    {
      // double t_start = vpTime::measureTimeMs();
      ros::Time t_start = ros::Time::now();

      g.acquire(I);
      vpDisplay::display(I);

      std::vector<vpHomogeneousMatrix> cMo_vec;
      detector.detect(I, opt_tagSize, cam, cMo_vec);

      std::stringstream ss;
      ss << "Left click to " << (send_velocities ? "stop the robot" : "servo the robot") << ", right click to quit.";
      vpDisplay::displayText(I, 20, 20, ss.str(), vpColor::red);

      //-----------------------------PBVS V_c--------------------------------------

      vpColVector v_c_PBVS(6);
      vpColVector v_c_IBVS(6);

      // Only one tag is detected
      if (cMo_vec.size() == 1)
      {
        cMo_IBVS = cMo_vec[0];
        cMo_PBVS = cMo_vec[0];

        static bool first_time = true;
        if (first_time)
        {
          // Introduce security wrt tag positionning in order to avoid PI rotation
          std::vector<vpHomogeneousMatrix> v_oMo_IBVS(2), v_cdMc_IBVS(2);
          v_oMo_IBVS[1].buildFrom(0, 0, 0, 0, 0, M_PI);
          std::vector<vpHomogeneousMatrix> v_oMo_PBVS(2), v_cdMc_PBVS(2);
          v_oMo_PBVS[1].buildFrom(0, 0, 0, 0, 0, M_PI);

          for (size_t i = 0; i < 2; i++)
          {
            v_cdMc_PBVS[i] = cdMo_PBVS * v_oMo_PBVS[i] * cMo_PBVS.inverse();
          }
          if (std::fabs(v_cdMc_PBVS[0].getThetaUVector().getTheta()) <
              std::fabs(v_cdMc_PBVS[1].getThetaUVector().getTheta()))
          {
            oMo_PBVS = v_oMo_PBVS[0];
          }
          else
          {
            // std::cout << "Desired frame modified to avoid PI rotation of the camera" << std::endl;
            oMo_PBVS = v_oMo_PBVS[1];  // Introduce PI rotation
          }
          // }
          //------------------------------------------IBVS-------------------------------------

          //         static bool first_time = true;
          //         if (first_time) {
          // Introduce security wrt tag positionning in order to avoid PI rotation

          for (size_t i = 0; i < 2; i++)
          {
            v_cdMc_IBVS[i] = cdMo_IBVS * v_oMo_IBVS[i] * cMo_IBVS.inverse();
          }
          if (std::fabs(v_cdMc_IBVS[0].getThetaUVector().getTheta()) <
              std::fabs(v_cdMc_IBVS[1].getThetaUVector().getTheta()))
          {
            oMo_IBVS = v_oMo_IBVS[0];
          }
          else
          {
            std::cout << " avoid PI rotation of the camera" << std::endl;
            // oMo = v_oMo[1];
            oMo_IBVS = v_oMo_IBVS[1];
            // Introduce PI rotation
          }
          // Compute the desired position of the features from the desired pose
          for (size_t i = 0; i < point.size(); i++)
          {
            vpColVector cP, p;
            point[i].changeFrame(cdMo_IBVS * oMo_IBVS, cP);
            point[i].projection(cP, p);

            pd[i].set_x(p[0]);
            pd[i].set_y(p[1]);
            pd[i].set_Z(cP[2]);
            // std::cout << "in\n";
          }
        }

        //-----------------------------------------IBVS END---------------------------------

        //-----------------------------------------PBVS Update features---------------------
        // Update visual features
        cdMc_PBVS = cdMo_PBVS * oMo_PBVS * cMo_PBVS.inverse();
        t.buildFrom(cdMc_PBVS);
        tu.buildFrom(cdMc_PBVS);

        if (opt_task_sequencing_PBVS)
        {
          if (!servo_started)
          {
            if (send_velocities)
            {
              servo_started = true;
            }
            t_init_servo = vpTime::measureTimeMs();
          }
          v_c_PBVS = task_PBVS.computeControlLaw((vpTime::measureTimeMs() - t_init_servo) / 1000.);
        }
        else
        {
          v_c_PBVS = task_PBVS.computeControlLaw();
        }

        //--------------------------------------------IBVS Update features---------------------------------------
        //-----Get tag corners
        std::vector<vpImagePoint> corners = detector.getPolygon(0);

        // Update visual features
        for (size_t i = 0; i < corners.size(); i++)
        {
          // Update the point feature from the tag corners location
          vpFeatureBuilder::create(p[i], cam, corners[i]);
          // Set the feature Z coordinate from the pose
          vpColVector cP;
          point[i].changeFrame(cMo_IBVS, cP);

          p[i].set_Z(cP[2]);
          // std::cout<< cP[2];
        }

        if (opt_task_sequencing_IBVS)
        {
          if (!servo_started)
          {
            if (send_velocities)
            {
              servo_started = true;
            }
            t_init_servo = vpTime::measureTimeMs();
          }
          v_c_IBVS = task_IBVS.computeControlLaw((vpTime::measureTimeMs() - t_init_servo) / 1000.);
        }
        else
        {
          v_c_IBVS = task_IBVS.computeControlLaw();
          // v_c = task.computeControlLaw(iter*robot.getSamplingTime());
        }
        //----------------------------------------------------Update features END----------------------------

        //--------------------------------------------PBVS Display---------------------------------------

        vpDisplay::displayFrame(I, cdMo_PBVS * oMo_PBVS, cam, opt_tagSize / 1.5, vpColor::yellow, 2);
        vpDisplay::displayFrame(I, cMo_PBVS, cam, opt_tagSize / 2, vpColor::none, 3);

        // Get tag corners
        std::vector<vpImagePoint> vip = detector.getPolygon(0);

        // Get the tag cog corresponding to the projection of the tag frame in the image
        vip.push_back(detector.getCog(0));

        // Display the trajectory of the points
        if (first_time)
        {
          traj_vip = new std::vector<vpImagePoint>[vip.size()];
        }
        display_point_trajectory(I, vip, traj_vip);

        // if (opt_plot)
        // {
        //   plotter->plot(0, iter_plot, task_PBVS.getError());
        //   plotter->plot(1, iter_plot, v_c_PBVS);
        //   iter_plot++;
        // }

        if (opt_verbose)
        {
          std::cout << "v_c: " << v_c_PBVS.t() << std::endl;
        }

        vpTranslationVector cd_t_c = cdMc_PBVS.getTranslationVector();
        vpThetaUVector cd_tu_c = cdMc_PBVS.getThetaUVector();
        double error_tr = sqrt(cd_t_c.sumSquare());
        double error_tu = vpMath::deg(sqrt(cd_tu_c.sumSquare()));

        ss.str("");
        ss << "error_t: " << error_tr;
        vpDisplay::displayText(I, 20, static_cast<int>(I.getWidth()) - 150, ss.str(), vpColor::red);
        ss.str("");
        ss << "error_tu: " << error_tu;
        vpDisplay::displayText(I, 40, static_cast<int>(I.getWidth()) - 150, ss.str(), vpColor::red);

        if (opt_verbose)
          std::cout << "error translation: " << error_tr << " ; error rotation: " << error_tu << std::endl;

        if (error_tr < convergence_threshold_t && error_tu < convergence_threshold_tu)
        {
          has_converged_PBVS = true;
          // new
          has_converged = true;
          // std::cout << "Servo task has converged" << std::endl;;
          // vpDisplay::displayText(I, 100, 20, "Servo task has converged", vpColor::red);
        }

        //----------------------------------------------IBVS  Display------------------------------------

        // Display the current and desired feature points in the image display
        vpServoDisplay::display(task_IBVS, cam, I);
        for (size_t i = 0; i < corners.size(); i++)
        {
          std::stringstream ss;
          ss << i;
          // Display current point indexes
          vpDisplay::displayText(I, corners[i] + vpImagePoint(15, 15), ss.str(), vpColor::red);
          // Display desired point indexes
          vpImagePoint ip;
          vpMeterPixelConversion::convertPoint(cam, pd[i].get_x(), pd[i].get_y(), ip);
          vpDisplay::displayText(I, ip + vpImagePoint(15, 15), ss.str(), vpColor::red);
        }
        if (first_time)
        {
          traj_corners = new std::vector<vpImagePoint>[corners.size()];
        }
        // Display the trajectory of the points used as features
        display_point_trajectory(I, corners, traj_corners);

        // if (opt_plot)
        // {
        //   plotter->plot(0, iter_plot, task_IBVS.getError());
        //   plotter->plot(1, iter_plot, v_c_Hybrid);
        //   vpColVector h;
        //   vpColVector joint_positions(robot_1_joint_positions);
        //   plotter->plot(2, iter_plot, joint_positions);
        //   iter_plot++;
        // }

        if (opt_verbose)
        {
          std::cout << "v_c: " << v_c_IBVS.t() << std::endl;
        }

        double error_IBVS = task_IBVS.getError().sumSquare();
        ss.str("");
        ss << "error: " << error_IBVS;
        vpDisplay::displayText(I, 20, static_cast<int>(I.getWidth()) - 150, ss.str(), vpColor::red);

        if (opt_verbose)
          std::cout << "error: " << error_IBVS << std::endl;

        if (error_IBVS < convergence_threshold)
        {
          has_converged = true;
          // has_converged = false;

          std::cout << "Servo task has converged"
                    << "\n";
          vpDisplay::displayText(I, 100, 20, "Servo task has converged", vpColor::red);
        }

        //---------------------------------------------------IBVS END------------

        if (first_time)
        {
          first_time = false;
        }
      }  // end if (cMo_vec.size() == 1)

      else
      {
        v_c_IBVS = 0;
        v_c_PBVS = 0;
      }

      if (!send_velocities)
      {
        v_c_IBVS = 0;
        v_c_PBVS = 0;
      }

      vpColVector v_c_Hybrid(6);

      v_c_Hybrid[0] = v_c_IBVS[0];
      v_c_Hybrid[1] = v_c_IBVS[1];
      v_c_Hybrid[2] = v_c_IBVS[2];
      v_c_Hybrid[3] = v_c_IBVS[3];
      v_c_Hybrid[4] = v_c_IBVS[4];
      v_c_Hybrid[5] = v_c_IBVS[5];

      // double error_IBVS = task_IBVS.getError().sumSquare();
      // vpColVector error_sumSquire(error_IBVS);

      if (opt_plot)
      {
        plotter->plot(0, iter_plot, task_IBVS.getError());
        plotter->plot(1, iter_plot, v_c_Hybrid);
        vpColVector joint_positions(robot_1_joint_positions);
        plotter->plot(2, iter_plot, joint_positions);
        // plotter->plot(3, iter_plot, error_sumSquire);
        iter_plot++;
      }

      //------------------------------------ KDL -----------------------------------------------

      robot_1_joint_values_mtx.lock();
      if (robot_1_jacobian_solver.JntToJac(*robot_1_joint_positions_ptr, robot_1_jacobian,
                                           robot_1_current_segment->second.segment.getName()))
      {
        robot_1_joint_values_mtx.unlock();
        break;
      }
      if (robot_1_fk_solver.JntToCart(*robot_1_joint_positions_ptr, camera_frame,
                                      robot_1_current_segment->second.segment.getName()))
      {
        robot_1_joint_values_mtx.unlock();
        break;
      }
      robot_1_joint_values_mtx.unlock();

      vpMatrix fJe(robot_1_jacobian.rows(), robot_1_jacobian.columns());
      for (int i = 0; i < robot_1_jacobian.rows(); i++)
      {
        for (int j = 0; j < robot_1_jacobian.columns(); j++)
        {
          fJe[i][j] = robot_1_jacobian.data(i, j);
        }
      }

      //---------------------------------------------------joint velocities -----------------------------------------

      vpColVector joint_velocities;
      vpMatrix jacobianPInv = fJe.transpose() * (fJe * fJe.transpose()).inverseByLU();
      vpMatrix identity;
      identity.eye(7);
      // joint_velocities = jacobianPInv * v_c_2;

      vpColVector v_c_Hybrid_ee(6);

      v_c_Hybrid_ee[0] = v_c_Hybrid[2];
      v_c_Hybrid_ee[1] = -v_c_Hybrid[0];
      v_c_Hybrid_ee[2] = -v_c_Hybrid[1];
      v_c_Hybrid_ee[3] = v_c_Hybrid[5];
      v_c_Hybrid_ee[4] = -v_c_Hybrid[3];
      v_c_Hybrid_ee[5] = -v_c_Hybrid[4];

      // std::cout << task_IBVS.getTaskSingularValues() << std::endl;

      // joint_velocities += task_IBVS.secondaryTaskJointLimitAvoidance(
      //     joint_positions1, old_joint_velocities, joint_lower_limits, joint_upper_limits, 0.4, 0.9, 0.8);
      // task_IBVS.set_eJe(fJe);
      // joint_velocities += task_IBVS.secondaryTaskJointLimitAvoidance(joint_positions1, old_joint_velocities,
      //                                                                joint_lower_limits, joint_upper_limits);

      //-----------Decouopling image jacobian matrix------------------

      for (i = 0; i < 3; ++i)
      {
        for (j = 0; j < 7; ++j)
        {
          J_V[i][j] = fJe[i][j];
          // std::cout<<J_V[i][j]<<" ";
        }
        // std::cout<<std::endl;
      }

      for (i = 0; i < 3; ++i)
      {
        for (j = 0; j < 7; ++j)
        {
          J_W[i][j] = fJe[i + 3][j];
        }
      }

      vpColVector V_Hybrid(3);
      V_Hybrid[0] = v_c_Hybrid_ee[0];
      V_Hybrid[1] = v_c_Hybrid_ee[1];
      V_Hybrid[2] = v_c_Hybrid_ee[2];

      vpColVector W_Hybrid(3);
      W_Hybrid[0] = v_c_Hybrid_ee[3];
      W_Hybrid[1] = v_c_Hybrid_ee[4];
      W_Hybrid[2] = v_c_Hybrid_ee[5];

      //      //------------------Decoupled   velocity  jacobian with damped least squire method-----------
      J_V_DLS_Null = J_V * J_V.t() + pow(landa_1, 2) * Ie_three;
      J_V_DLS = J_V.t() * J_V_DLS_Null.inverseByLU();

      //      //------------------Decoupled orientation jacobian with damped least squire method-----------
      J_W_DLS_Null = J_W * J_W.t() + pow(landa_2, 2) * Ie_three;
      J_W_DLS = J_W.t() * J_W_DLS_Null.inverseByLU();
      //
      //
      //
      vpMatrix J_ND_Null(6u, 6u);
      J_ND_Null = fJe * fJe.t() + pow(landa_1, 2) * Ie_Six;

      //   // -------------------------------------- Control law ---------------------------------------
      // q_dot = eJe.pseudoInverse() * v_c_Hybrid;
      q_dot = fJe.pseudoInverse() * v_c_Hybrid_ee;

      q_dot_DLS = fJe.t() * J_ND_Null.inverseByLU() * v_c_Hybrid_ee;

      q_dot_Decouple_DLS = J_V_DLS * V_Hybrid + (Ie_seven - J_V_DLS * J_V) * J_W_DLS * W_Hybrid;

      //----------------------------------------     Send to the robot

      std_msgs::Float64MultiArray joint_vel_msg;

      joint_velocities = fJe.pseudoInverse() * v_c_Hybrid_ee;

      vpColVector joint_positions1(robot_1_joint_positions);
      vpColVector old_joint_velocities(robot_1_joint_velocities);
      vpColVector joint_lower_limits(lower_joint_limits);
      vpColVector joint_upper_limits(upper_joint_limits);

      //---------------------------------------------------kalman filter-------------------------------------
      if (filter_type == "none")
      {
        for (int i = 0; i < joint_velocities.size(); i++)
        {
          joint_vel_msg.data.push_back(joint_velocities[i]);
          // joint_vel_msg.data.push_back(q_dot_DLS[i]);
          // joint_vel_msg.data.push_back(q_dot_Decouple_DLS[i]);
        }
      }
      else if (filter_type == "lowpass")
      {
        robot_1_joint_values_mtx.lock();
        for (int i = 0; i < joint_velocities.size(); i++)
        {
          joint_vel_msg.data.push_back((1 - lowpass_gain) * robot_1_joint_velocities[i] +
                                       lowpass_gain * joint_velocities[i]);
        }
        robot_1_joint_values_mtx.unlock();
      }
      else if (filter_type == "kalman")
      {
        kalman_filter.prediction();
        kalman_filter.filtering(joint_velocities);
        vpColVector new_joint_velocities = kalman_filter.Xest;
        for (int i = 0; i < new_joint_velocities.size(); i++)
        {
          joint_vel_msg.data.push_back(new_joint_velocities[i]);
        }
      }
      else if (filter_type == "kalman-lowpass")
      {
        robot_1_joint_values_mtx.lock();
        vpColVector current_joint_velocities(robot_1_joint_velocities);
        robot_1_joint_values_mtx.unlock();
        kalman_filter.prediction();
        kalman_filter.filtering((1 - lowpass_gain) * current_joint_velocities + lowpass_gain * joint_velocities);
        vpColVector new_joint_velocities = kalman_filter.Xest;
        for (int i = 0; i < new_joint_velocities.size(); i++)
        {
          joint_vel_msg.data.push_back(new_joint_velocities[i]);
        }
      }
      else
      {
        std::cout << "Unrecognised filter type." << std::endl;
        break;
      }
      //--------------------------------------------------------------------------kalman end------------------------

      //------------------------send to robot -----------------------------------
      robot_1_joint_vel_pub.publish(joint_vel_msg);

      {
        std::stringstream ss;
        ss << "Loop time: " << ros::Time::now() - t_start << " ms";
        vpDisplay::displayText(I, 40, 20, ss.str(), vpColor::red);
      }
      vpDisplay::flush(I);
      ros::spinOnce();
      loop_rate.sleep();

      vpMouseButton::vpMouseButtonType button;
      if (vpDisplay::getClick(I, button, false))
      {
        switch (button)
        {
          case vpMouseButton::button1:
            send_velocities = !send_velocities;
            break;

          case vpMouseButton::button3:
            final_quit = true;
            v_c_IBVS = 0;
            v_c_PBVS = 0;
            break;

          default:
            break;
        }
      }
    }

    std::cout << "Stop the robot " << std::endl;

    if (opt_plot && plotter != nullptr)
    {
      // delete plotter;
      //#ifdef VISP_HAVE_DISPLAY
      plotter->saveData(0, "error.dat");
      plotter->saveData(1, "vc.dat");
      plotter->saveData(2, "Joint.dat");
      //#endif
      plotter = nullptr;
    }

    task_IBVS.kill();

    if (!final_quit)
    {
      while (!final_quit)
      {
        g.acquire(I);
        vpDisplay::display(I);

        vpDisplay::displayText(I, 20, 20, "Click to quit the program.", vpColor::red);
        // vpDisplay::displayText(I, 40, 20, "Visual servo converged.", vpCDisplayolor::red);

        if (vpDisplay::getClick(I, false))
        {
          final_quit = true;
        }

        vpDisplay::flush(I);
      }
    }
    if (traj_corners)
    {
      delete[] traj_corners;
    }
  }

  catch (const vpException &e)
  {
    std::cout << "ViSP exception: " << e.what() << std::endl;
    std::cout << "Stop the robot " << std::endl;
    robot.setRobotState(vpRobot::STATE_STOP);
    return EXIT_FAILURE;
  }

  catch (const std::exception &e)
  {
    std::cout << "Franka exception: " << e.what() << std::endl;
    return EXIT_FAILURE;
  }

  return 0;
}
