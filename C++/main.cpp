#include <iostream>

 #include <visp3/core/vpCameraParameters.h>
 #include <visp3/gui/vpDisplayGDI.h>
 #include <visp3/gui/vpDisplayX.h>
 #include <visp3/io/vpImageIo.h>
 #include <visp3/sensor/vpRealSense2.h>
 #include <visp3/robot/vpRobotFranka.h>
 #include <visp3/detection/vpDetectorAprilTag.h>
 #include <visp3/visual_features/vpFeatureBuilder.h>
 #include <visp3/visual_features/vpFeaturePoint.h>
 #include <visp3/vs/vpServo.h>
 #include <visp3/vs/vpServoDisplay.h>
 #include <visp3/gui/vpPlot.h>
 #include "vpRobotFrankaExtended.h"

 #if defined(VISP_HAVE_REALSENSE2) && (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11) && \
 (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI)) && defined(VISP_HAVE_FRANKA)
 void display_point_trajectory(const vpImage<unsigned char> &I, const std::vector<vpImagePoint> &vip,
                               std::vector<vpImagePoint> *traj_vip)
 {
   for (size_t i = 0; i < vip.size(); i++) {
     if (traj_vip[i].size()) {
       // Add the point only if distance with the previous > 1 pixel
       if (vpImagePoint::distance(vip[i], traj_vip[i].back()) > 1.) {
         traj_vip[i].push_back(vip[i]);
       }
     }
     else {
       traj_vip[i].push_back(vip[i]);
     }
   }
   for (size_t i = 0; i < vip.size(); i++) {
     for (size_t j = 1; j < traj_vip[i].size(); j++) {
       vpDisplay::displayLine(I, traj_vip[i][j - 1], traj_vip[i][j], vpColor::green, 2);
     }
   }
 }

 int main(int argc, char **argv)
 {
 //-------------------------------------------------initialize------------------------------
   double opt_tagSize = 0.120;
   std::string opt_robot_ip = "172.16.0.2";
   std::string opt_eMc_filename = "";
   std::string Controller = "HDVS"; //Controller types : IBVS, PBVS, HDVS, HYVS
   bool display_tag = true;
   int opt_quad_decimate = 2;
   bool opt_verbose = false;
   bool opt_plot = false;
   bool opt_adaptive_gain_IBVS = true;
   bool opt_adaptive_gain_PVBS = true;
   bool opt_task_sequencing_IBVS = true;
   bool opt_task_sequencing_PBVS = true;
   double convergence_threshold = 0.00009;
   double convergence_threshold_t = 0.0005, convergence_threshold_tu = vpMath::rad(0.5);
   bool has_converged = false;
   bool has_converged_PBVS = false;

   vpColVector  X(7u,0.0);
   vpColVector  q_dot(7u,0.0);
   vpColVector  q_dot_DLS(7u,0.0);
   vpColVector  q_dot_Decouple_DLS(7u,0.0);
   vpMatrix    J(6u,7u);
   vpMatrix    J_V(3u,7u);
   vpMatrix    J_V_DLS(7u,3u);
   vpMatrix    J_V_DLS_Null(3u,3u);
   vpMatrix    J_W(3u,7u);
   vpMatrix    J_W_DLS(7u,3u);
   vpMatrix    J_W_DLS_Null(3u,3u);
   int         landa_1 = 0.01;
   int         landa_2 = 0.01;
   int         i,j;
     vpMatrix    Ie_seven;
     vpMatrix    Ie_three;
     vpMatrix    Ie_Six;
     Ie_seven.eye(7);
     Ie_three.eye(3);
     Ie_Six.eye(6);

   for (int i = 1; i < argc; i++) {
     if (std::string(argv[i]) == "--tag_size" && i + 1 < argc) {
       opt_tagSize = std::stod(argv[i + 1]);
     }
     else if (std::string(argv[i]) == "--ip" && i + 1 < argc) {
       opt_robot_ip = std::string(argv[i + 1]);
     }
     else if (std::string(argv[i]) == "--eMc" && i + 1 < argc) {
       opt_eMc_filename = std::string(argv[i + 1]);
     }
     else if (std::string(argv[i]) == "--verbose") {
       opt_verbose = true;
     }
     else if (std::string(argv[i]) == "--plot") {
       opt_plot = true;
     }
     else if (std::string(argv[i]) == "--adaptive_gain") {
       opt_adaptive_gain_IBVS = true;
     }
     else if (std::string(argv[i]) == "--task_sequencing") {
       opt_task_sequencing_IBVS = true;
     }
      else if (std::string(argv[i]) == "--adaptive_gain") {
       opt_adaptive_gain_PVBS = true;
     }
     else if (std::string(argv[i]) == "--task_sequencing") {
       opt_task_sequencing_PBVS = true;
     }
     else if (std::string(argv[i]) == "--quad_decimate" && i + 1 < argc) {
       opt_quad_decimate = std::stoi(argv[i + 1]);
     }
     else if (std::string(argv[i]) == "--no-convergence-threshold") {
       convergence_threshold = 0.;
     }
     else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
       std::cout << argv[0] << " [--ip <default " << opt_robot_ip << ">] [--tag_size <marker size in meter; default " << opt_tagSize << ">] [--eMc <eMc extrinsic file>] "
                            << "[--quad_decimate <decimation; default " << opt_quad_decimate << ">] [--adaptive_gain] [--plot] [--task_sequencing] [--no-convergence-threshold] [--verbose] [--help] [-h]"
                            << "\n";
       return EXIT_SUCCESS;
     }
   }

   vpRobotFrankaExtended robot;

   try {
        robot.connect(opt_robot_ip);
//        vpColVector h{};
//        robot.getPosition(vpRobotFranka::JOINT_STATE,h);
//        std::cout<<h<<'\n';

        robot.setRobotState(vpRobot::STATE_POSITION_CONTROL);
       // const vpColVector homePosition{std::vector<double>{2.711796941, -0.4, -2.662726228, -1.5, 0.0, 3.269520677, 0.6720384158}};  //looking forward
       const vpColVector homePosition{std::vector<double>{2.711796941, 0.5704239652, -2.662726228, -2.564509519, 0.2588469911, 2.069520677, 0.6720384158}}; //looking down
     // const vpColVector homePosition{std::vector<double>{2.711796941, -0.4, -2.662726228, -1.5, 0.0, 3.269520677, 0.6720384158}}; //looking forward POMIT 1 and 2
     // const vpColVector homePosition{std::vector<double>{2.340008298, 0.6441855505, -2.077582322, -2.41145959, 0.5137678226, 3, 0.6850247996}}; //POINT3
      //  const vpColVector homePosition{std::vector<double>{2.340008298, 0.6441855505, -2.077582322, -2.41145959, 0.5137678226, 2.689148097, 0.6850247996}}; //Dual arm
      // const vpColVector homePosition{std::vector<double>{0,(-M_PI_4 - M_PI_2 / 3.0) / 2, 0, (-3.0 * M_PI_4 - 2.0 * M_PI_2 / 3.0) / 2, 0,  M_PI_2 + 1.2 * M_PI_4, -(M_PI_2 + M_PI_2) / 2 + M_PI_2}}; //Simulation ROS ROBOT1
      //const vpColVector homePosition{std::vector<double>{M_PI_4 / 4,(-M_PI_4 - M_PI_2 / 3.0) / 2 + M_PI_2 / 2.5,0,(-3.0 * M_PI_4 - 2.0 * M_PI_2 / 3.0) / 2 + M_PI_4 / 1.5,0.4 * M_PI_4 - M_PI_2,(M_PI_2 + 0.85 * M_PI_4) / 2 + 0.2 * M_PI_2, (M_PI_2 + M_PI_2) / 2 - M_PI_2 / 4 + M_PI_2 / 8 - M_PI_2 / 10}}; //Simulation ROS ROBOT2

        robot.setPosition(vpRobotFranka::JOINT_STATE, homePosition);
//      vpColVector h{};
//      robot.getPosition(vpRobotFranka::JOINT_STATE,h);
//      std::cout<<h<<std::endl;


     //robot.setJointImpedance({800,800,800,2500,2500,2500,2500});
     robot.setJointImpedance({300,300,300,600,800,400,400});

     vpRealSense2 rs;
     rs2::config config;
     unsigned int width = 640, height = 480;
     config.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGBA8, 30);
     config.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
     config.enable_stream(RS2_STREAM_INFRARED, 640, 480, RS2_FORMAT_Y8, 30);
     rs.open(config);

     // Get camera extrinsics
     vpPoseVector ePc;
     // Set camera extrinsics default values
     ePc[0] = 0.0337731; ePc[1] = -0.00535012; ePc[2] = -0.0523339;
     ePc[3] = -0.247294; ePc[4] = -0.306729; ePc[5] = 1.53055;

     // If provided, read camera extrinsics from --eMc <file>
     if (!opt_eMc_filename.empty()) {
       ePc.loadYAML(opt_eMc_filename, ePc);
     }
     else {
       //std::cout << "Warning, opt_eMc_filename is empty! Use hard coded values." << "\n";
     }
     vpHomogeneousMatrix eMc(ePc);
     // std::cout << "eMc:\n" << eMc << "\n";

     // Get camera intrinsics
     vpCameraParameters cam = rs.getCameraParameters(RS2_STREAM_COLOR, vpCameraParameters::perspectiveProjWithDistortion);
    //  std::cout << "cam:\n" << cam << "\n";

     vpImage<unsigned char> I(height, width);

 #if defined(VISP_HAVE_X11)
     vpDisplayX dc(I, 10, 10, "Color image");
 #elif defined(VISP_HAVE_GDI)
     vpDisplayGDI dc(I, 10, 10, "Color image");
 #endif

     vpDetectorAprilTag::vpAprilTagFamily tagFamily = vpDetectorAprilTag::TAG_36h11;
     vpDetectorAprilTag::vpPoseEstimationMethod poseEstimationMethod = vpDetectorAprilTag::HOMOGRAPHY_VIRTUAL_VS;
     //vpDetectorAprilTag::vpPoseEstimationMethod poseEstimationMethod = vpDetectorAprilTag::BEST_RESIDUAL_VIRTUAL_VS;
     vpDetectorAprilTag detector(tagFamily);
     detector.setAprilTagPoseEstimationMethod(poseEstimationMethod);
     detector.setDisplayTag(display_tag);
     detector.setAprilTagQuadDecimate(opt_quad_decimate);

     // Servo
     vpHomogeneousMatrix cdMc_IBVS, cMo_IBVS, oMo_IBVS;

     // Desired pose used to compute the desired features
     vpHomogeneousMatrix cdMo_IBVS( vpTranslationVector(0, 0, opt_tagSize * 7), // 3 times tag with along camera z axis
                               vpRotationMatrix( {1, 0, 0, 0, -1, 0, 0, 0, -1} ) );

     // Create visual features
     std::vector<vpFeaturePoint> p(4), pd(4); // We use 4 points

     // Define 4 3D points corresponding to the CAD model of the Apriltag
     std::vector<vpPoint> point(4);
     point[0].setWorldCoordinates(-opt_tagSize/2., -opt_tagSize/2., 0);
     point[1].setWorldCoordinates( opt_tagSize/2., -opt_tagSize/2., 0);
     point[2].setWorldCoordinates( opt_tagSize/2.,  opt_tagSize/2., 0);
     point[3].setWorldCoordinates(-opt_tagSize/2.,  opt_tagSize/2., 0);

     vpServo task_IBVS;

     // Add the 4 visual feature points
     for (size_t i = 0; i < p.size(); i++) {
       task_IBVS.addFeature(p[i], pd[i]);
     }
     task_IBVS.setServo(vpServo::EYEINHAND_CAMERA);
     task_IBVS.setInteractionMatrixType(vpServo::CURRENT);
     //robot.setJointImpedance({300,300,300,300,300,300,300});

    //--------------------------PBVS Servo-----------------------
     // Servo
     vpServo task_PBVS;
     vpHomogeneousMatrix cdMc_PBVS, cMo_PBVS, oMo_PBVS;

     // Desired pose used to compute the desired features
     vpHomogeneousMatrix cdMo_PBVS( vpTranslationVector(0, 0, opt_tagSize * 7), // 3 times tag with along camera z axis
                               vpRotationMatrix( {1, 0, 0, 0, -1, 0, 0, 0, -1} ) );

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

      if (opt_adaptive_gain_PVBS) {
       vpAdaptiveGain lambda(1, 0.4, 30); // lambda(0)=4, lambda(oo)=0.4 and lambda'(0)=30
       task_PBVS.setLambda(lambda);
     }
     else {
       task_PBVS.setLambda(0.5);
     }
     //-------------------------PBVS--------------------------

     if (opt_adaptive_gain_IBVS) {
       vpAdaptiveGain lambda(1, 0.4, 30);
       task_IBVS.setLambda(lambda);
     }
     else {
       task_IBVS.setLambda(0.5);
     }

     vpPlot *plotter = nullptr;
     int iter_plot = 0;

     if (opt_plot) {
       plotter = new vpPlot(3, static_cast<int>(250 * 2), 500, static_cast<int>(I.getWidth()) + 80, 10, "Real time curves plotter");
       plotter->setTitle(0, "Visual features error");
       plotter->setTitle(1, "Camera velocities");
              plotter->setTitle(2, "Joints");

       plotter->initGraph(0, 8);
       plotter->initGraph(1, 6);
       plotter->initGraph(2, 7);

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
     }

     bool final_quit = false;
     bool send_velocities = false;
     bool servo_started = false;
     has_converged = false;
     has_converged_PBVS = false;

     std::vector<vpImagePoint> *traj_vip = nullptr;
     std::vector<vpImagePoint> *traj_corners = nullptr; // To memorize point trajectory

     static double t_init_servo = vpTime::measureTimeMs();

     robot.set_eMc(eMc); // Set location of the camera wrt end-effector frame
     robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL);


//----------------------------------------------------------------MAIN Control Loop(While)-----------------------
     while (!has_converged && !final_quit) {
       double t_start = vpTime::measureTimeMs();

       rs.acquire(I);

       vpDisplay::display(I);

       //std::vector<vpHomogeneousMatrix> cMo_vec_IBVS;
     //  detector.detect(I, opt_tagSize, cam, cMo_vec_IBVS);
       std::vector<vpHomogeneousMatrix> cMo_vec;
       detector.detect(I, opt_tagSize, cam, cMo_vec);


       std::stringstream ss;
       ss << "Left click to " << (send_velocities ? "stop the robot" : "servo the robot") << ", right click to quit.";
       vpDisplay::displayText(I, 20, 20, ss.str(), vpColor::red);

//---------------------------------------PBVS V_c--------------------------------------

     vpColVector v_c_PBVS(6);
     vpColVector v_c_IBVS(6);

       // Only one tag is detected
       if (cMo_vec.size() == 1) {
         cMo_IBVS = cMo_vec[0];
         cMo_PBVS = cMo_vec[0];

         static bool first_time = true;
         if (first_time) {
           // Introduce security wrt tag positionning in order to avoid PI rotation
           std::vector<vpHomogeneousMatrix> v_oMo_IBVS(2), v_cdMc_IBVS(2);
           v_oMo_IBVS[1].buildFrom(0, 0, 0, 0, 0, M_PI);
           std::vector<vpHomogeneousMatrix> v_oMo_PBVS(2), v_cdMc_PBVS(2);
           v_oMo_PBVS[1].buildFrom(0, 0, 0, 0, 0, M_PI);


           for (size_t i = 0; i < 2; i++) {
             v_cdMc_PBVS[i] = cdMo_PBVS * v_oMo_PBVS[i] * cMo_PBVS.inverse();
           }
           if (std::fabs(v_cdMc_PBVS[0].getThetaUVector().getTheta()) < std::fabs(v_cdMc_PBVS[1].getThetaUVector().getTheta())) {
             oMo_PBVS = v_oMo_PBVS[0];
           }
           else {
            // std::cout << "Desired frame modified to avoid PI rotation of the camera" << std::endl;
             oMo_PBVS = v_oMo_PBVS[1];   // Introduce PI rotation
           }
        // }
//------------------------------------------IBVS-------------------------------------

//         static bool first_time = true;
//         if (first_time) {
           // Introduce security wrt tag positionning in order to avoid PI rotation

           for (size_t i = 0; i < 2; i++) {
             v_cdMc_IBVS[i] = cdMo_IBVS * v_oMo_IBVS[i] * cMo_IBVS.inverse();
           }
           if (std::fabs(v_cdMc_IBVS[0].getThetaUVector().getTheta()) < std::fabs(v_cdMc_IBVS[1].getThetaUVector().getTheta())) {
             oMo_IBVS = v_oMo_IBVS[0];
           }
           else {
            std::cout << " avoid PI rotation of the camera" << std::endl;
            // oMo = v_oMo[1];
             oMo_IBVS = v_oMo_IBVS[1];
               // Introduce PI rotation
           }
           // Compute the desired position of the features from the desired pose
           for (size_t i = 0; i < point.size(); i++) {
             vpColVector cP, p;
             point[i].changeFrame(cdMo_IBVS * oMo_IBVS, cP);
             point[i].projection(cP, p);

             pd[i].set_x(p[0]);
             pd[i].set_y(p[1]);
             pd[i].set_Z(cP[2]);
             //std::cout << "in\n";
           }
         }

//-----------------------------------------IBVS END---------------------------------


//-----------------------------------------PBVS Update features---------------------
         // Update visual features
         cdMc_PBVS = cdMo_PBVS * oMo_PBVS * cMo_PBVS.inverse();
         t.buildFrom(cdMc_PBVS);
         tu.buildFrom(cdMc_PBVS);

         if (opt_task_sequencing_PBVS) {
           if (! servo_started) {
             if (send_velocities) {
               servo_started = true;
             }
             t_init_servo = vpTime::measureTimeMs();
           }
           v_c_PBVS = task_PBVS.computeControlLaw((vpTime::measureTimeMs() - t_init_servo)/1000.);
         }
         else {
           v_c_PBVS = task_PBVS.computeControlLaw();
         }


//--------------------------------------------IBVS Update features---------------------------------------
         //-----Get tag corners
         std::vector<vpImagePoint> corners = detector.getPolygon(0);

         // Update visual features
         for (size_t i = 0; i < corners.size(); i++) {
           // Update the point feature from the tag corners location
           vpFeatureBuilder::create(p[i], cam, corners[i]);
           // Set the feature Z coordinate from the pose
           vpColVector cP;
           point[i].changeFrame(cMo_IBVS, cP);

           p[i].set_Z(cP[2]);
          // std::cout<< cP[2];
         }

         if (opt_task_sequencing_IBVS) {
           if (! servo_started) {
             if (send_velocities) {
               servo_started = true;
             }
             t_init_servo = vpTime::measureTimeMs();
           }
           v_c_IBVS = task_IBVS.computeControlLaw((vpTime::measureTimeMs() - t_init_servo)/1000.);
         }
         else {
           v_c_IBVS = task_IBVS.computeControlLaw();
           //v_c = task.computeControlLaw(iter*robot.getSamplingTime());
         }
//----------------------------------------------------Update features END----------------------------


 //--------------------------------------------PBVS Display---------------------------------------

         vpDisplay::displayFrame(I, cdMo_PBVS * oMo_PBVS, cam, opt_tagSize / 1.5, vpColor::yellow, 2);
         vpDisplay::displayFrame(I, cMo_PBVS,  cam, opt_tagSize / 2,   vpColor::none,   3);

          // Get tag corners
            std::vector<vpImagePoint> vip = detector.getPolygon(0);

         // Get the tag cog corresponding to the projection of the tag frame in the image
            vip.push_back(detector.getCog(0));

          //Display the trajectory of the points
         if (first_time) {
            traj_vip = new std::vector<vpImagePoint> [vip.size()];
         }
        display_point_trajectory(I, vip, traj_vip);


//if (Controller=="PBVS"){
//         if (opt_plot) {
//           plotter->plot(0, iter_plot, task_PBVS.getError());
//           plotter->plot(1, iter_plot, v_c_PBVS);
//           vpColVector h;
//           robot.getPosition(vpRobotFranka::JOINT_STATE,h);
//           plotter->plot(2, iter_plot, h);
//           iter_plot++;
//         }
//}

         if (opt_verbose) {
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

         if (error_tr < convergence_threshold_t && error_tu < convergence_threshold_tu) {
           has_converged_PBVS = true;
           //std::cout << "Servo task has converged" << std::endl;;
           //vpDisplay::displayText(I, 100, 20, "Servo task has converged", vpColor::red);
         }

 //----------------------------------------------IBVS  Display------------------------------------

         // Display the current and desired feature points in the image display
         vpServoDisplay::display(task_IBVS, cam, I);
         for (size_t i = 0; i < corners.size(); i++) {
           std::stringstream ss;
           ss << i;
           // Display current point indexes
           vpDisplay::displayText(I, corners[i]+vpImagePoint(15, 15), ss.str(), vpColor::red);
           // Display desired point indexes
           vpImagePoint ip;
           vpMeterPixelConversion::convertPoint(cam, pd[i].get_x(), pd[i].get_y(), ip);
           vpDisplay::displayText(I, ip+vpImagePoint(15, 15), ss.str(), vpColor::red);
         }
         if (first_time) {
            traj_corners = new std::vector<vpImagePoint> [corners.size()];
         }
         // Display the trajectory of the points used as features
         display_point_trajectory(I, corners, traj_corners);

         if (opt_verbose) {
           std::cout << "v_c: " << v_c_IBVS.t() << std::endl;
         }

         double error_IBVS = task_IBVS.getError().sumSquare();
         ss.str("");
         ss << "error: " << error_IBVS;
         vpDisplay::displayText(I, 20, static_cast<int>(I.getWidth()) - 150, ss.str(), vpColor::red);

         if (opt_verbose)
           std::cout << "error: " << error_IBVS << std::endl;

         if (error_IBVS < convergence_threshold) {
           has_converged = true;
           std::cout << "Servo task has converged" << "\n";
           vpDisplay::displayText(I, 100, 20, "Servo task has converged", vpColor::red);
         }


//---------------------------------------------------IBVS END------------

         if (first_time) {
           first_time = false;
         }
       } // end if (cMo_vec.size() == 1)

       else {
         v_c_IBVS = 0;
         v_c_PBVS = 0;
       }

       if (!send_velocities) {
         v_c_IBVS = 0;
         v_c_PBVS = 0;
       }

  vpColVector v_c_Hybrid(6);
  vpColVector v_c_Hyb(6);

    v_c_Hybrid[0] = v_c_IBVS[0];
    v_c_Hybrid[1] = v_c_IBVS[1];
    v_c_Hybrid[2] = v_c_PBVS[2];
    v_c_Hybrid[3] = v_c_IBVS[3];
    v_c_Hybrid[4] = v_c_IBVS[4];
    v_c_Hybrid[5] = v_c_PBVS[5];
    v_c_Hyb[0] = v_c_IBVS[0];
    v_c_Hyb[1] = v_c_IBVS[1];
    v_c_Hyb[2] = v_c_PBVS[2];
    v_c_Hyb[3] = v_c_PBVS[3];
    v_c_Hyb[4] = v_c_PBVS[4];
    v_c_Hyb[5] = v_c_PBVS[5];



//---------------------------------------------plots----------------------------
    if (opt_plot) {
           plotter->plot(0, iter_plot, task_IBVS.getError());
            if (Controller == "IBVS"){
            plotter->plot(1, iter_plot, v_c_IBVS);
            }else if(Controller == "PBVS"){
            plotter->plot(1, iter_plot, v_c_PBVS);
            }else if(Controller == "HDVS"){
            plotter->plot(1, iter_plot, v_c_Hybrid);
            }else if(Controller == "HYVS"){
            plotter->plot(1, iter_plot, v_c_Hyb);
            }
           vpColVector h;
           robot.getPosition(vpRobotFranka::JOINT_STATE,h);
           plotter->plot(2, iter_plot, h);
           iter_plot++;
         }

//-----------Decouopling image jacobian matrix------------------

         robot.get_eJe(J);

         for(i=0; i<3; ++i) {
         for(j=0; j<7; ++j){
         J_V[i][j]=J[i][j];
          //std::cout<<J_V[i][j]<<" ";
            }
         // std::cout<<std::endl;
            }

         for(i=0; i<3; ++i) {
         for(j=0; j<7; ++j){
          J_W[i][j]=J[i+3][j];
            }
            }

        vpColVector V_Hybrid(3);
            V_Hybrid[0] = v_c_Hybrid[0];
            V_Hybrid[1] = v_c_Hybrid[1];
            V_Hybrid[2] = v_c_Hybrid[2];

        vpColVector W_Hybrid(3);
            W_Hybrid[0] = v_c_Hybrid[3];
            W_Hybrid[1] = v_c_Hybrid[4];
            W_Hybrid[2] = v_c_Hybrid[5];


//      //------------------Decoupled   velocity  jacobian with damped least squire method-----------
      J_V_DLS_Null = J_V*J_V.t()+pow(landa_1,2)*Ie_three;
      J_V_DLS = J_V.t()*J_V_DLS_Null.inverseByLU();


//      //------------------Decoupled orientation jacobian with damped least squire method-----------
      J_W_DLS_Null = J_W*J_W.t()+pow(landa_2,2)*Ie_three;
      J_W_DLS = J_W.t()*J_W_DLS_Null.inverseByLU();
//
//
//
        vpMatrix J_ND_Null(6u,6u);
        J_ND_Null =J*J.t()+pow(landa_1,2)*Ie_Six;

         // -------------------------------------- Control law ---------------------------------------
        q_dot=J.pseudoInverse()*v_c_Hybrid;

        q_dot_DLS= J.t()*J_ND_Null.inverseByLU()*v_c_Hybrid;

        q_dot_Decouple_DLS = J_V_DLS*V_Hybrid+(Ie_seven-J_V_DLS*J_V)*J_W_DLS*W_Hybrid;

//----------------------------------------     Send to the robot ----------------------------------------------------------------------

    //-----------------------------------------------------------Image Jacobian-------------------------------------------
      if (Controller == "IBVS"){
          robot.setVelocity(vpRobot::CAMERA_FRAME, v_c_IBVS);
      }else if(Controller == "PBVS"){
          robot.setVelocity(vpRobot::CAMERA_FRAME, v_c_PBVS);
      }else if(Controller == "HDVS"){
          robot.setVelocity(vpRobotFranka::JOINT_STATE, q_dot_DLS);
      }else if(Controller == "HYVS"){
          robot.setVelocity(vpRobot::CAMERA_FRAME, v_c_Hyb);
      }


       {
         std::stringstream ss;
         ss << "Loop time: " << vpTime::measureTimeMs() - t_start << " ms";
         vpDisplay::displayText(I, 40, 20, ss.str(), vpColor::red);
       }
       vpDisplay::flush(I);

       vpMouseButton::vpMouseButtonType button;
       if (vpDisplay::getClick(I, button, false)) {
         switch (button) {
         case vpMouseButton::button1:
           send_velocities = !send_velocities;
           break;

         case vpMouseButton::button3:
           final_quit = true;
           v_c_IBVS = 0;          // v_c_PBVS = 0;
           break;

         default:
           break;
         }
       }
     }

     std::cout << "Stop the robot " << std::endl;
    // usleep(50000);
     robot.setRobotState(vpRobot::STATE_STOP);
 //-------------------------------------PBVS V_c END----------------------------------------------------------

//------------------------------------------------------------detecting battery surface--------------------------------------------
//    if(has_converged)
//    {
//          vpPoseVector desiredPose{};
//          robot.getPosition(vpRobotFranka::END_EFFECTOR_FRAME, desiredPose);
//          robot.setPoseEndEffector(desiredPose, 0.05);
//          desiredPose[0] += 0.13;
//          desiredPose[1] += 0.05;
//          desiredPose[2] += 0.06;
////          desiredPose[0] += eMc[0][3];
////          desiredPose[1] += eMc[1][3];
////          desiredPose[2] -= eMc[2][3];
//          robot.setPoseEndEffector(desiredPose, 0.05);
//          std::cout << robot.lowerEndEffectorWithForceFeedback(2, 0.03) << '\n';
//          std::cout << "The module detected" << '\n';
//
//    }
 //------------------------------------------------------------------------------------------

// ------------------------------------------------------------save plots data------------------------


     if (opt_plot && plotter != nullptr) {
       //delete plotter;
       //#ifdef VISP_HAVE_DISPLAY
  plotter->saveData(0, "error.dat");
  plotter->saveData(1, "vc.dat");
  plotter->saveData(2, "Joint.dat");
//#endif
       plotter = nullptr;
     }

     task_IBVS.kill();

     if (!final_quit) {
       while (!final_quit) {
         rs.acquire(I);
         vpDisplay::display(I);

         vpDisplay::displayText(I, 20, 20, "Click to quit the program.", vpColor::red);
         //vpDisplay::displayText(I, 40, 20, "Visual servo converged.", vpCDisplayolor::red);

         if (vpDisplay::getClick(I, false)) {
           final_quit = true;
         }

         vpDisplay::flush(I);
       }
     }
     if (traj_corners) {
       delete [] traj_corners;
     }
   }

   catch(const vpException &e) {
     std::cout << "ViSP exception: " << e.what() << std::endl;
     std::cout << "Stop the robot " << std::endl;
     robot.setRobotState(vpRobot::STATE_STOP);
     return EXIT_FAILURE;
   }
   catch(const franka::NetworkException &e) {
     std::cout << "Franka network exception: " << e.what() << std::endl;
     std::cout << "Check if you are connected to the Franka robot"
               << " or if you specified the right IP using --ip command line option set by default to 192.168.1.1. " << std::endl;
     return EXIT_FAILURE;
   }
   catch(const std::exception &e) {
     std::cout << "Franka exception: " << e.what() << std::endl;
     return EXIT_FAILURE;
   }

   return 0;
 }

 #else
 int main()
 {
 #if !defined(VISP_HAVE_REALSENSE2)
   std::cout << "Install librealsense-2.x" << std::endl;
 #endif
 #if (VISP_CXX_STANDARD < VISP_CXX_STANDARD_11)
   std::cout << "Build ViSP with c++11 or higher compiler flag (cmake -DUSE_CXX_STANDARD=11)." << std::endl;
 #endif
 #if !defined(VISP_HAVE_FRANKA)
   std::cout << "Install libfranka." << std::endl;
 #endif
   return 0;
 }
 #endif
