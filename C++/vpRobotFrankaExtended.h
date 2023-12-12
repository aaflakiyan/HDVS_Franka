#ifndef vpRobotFrankaExtended_h
#define vpRobotFrankaExtended_h

#include "visp3/robot/vpRobotFranka.h"

class vpRobotFrankaExtended : public vpRobotFranka
{
public:
    static constexpr unsigned int k_numJoints{7};

    inline vpRobotFrankaExtended() : vpRobotFranka() {};
    inline vpRobotFrankaExtended(const std::string& franka_address, franka::RealtimeConfig realtime_config = franka::RealtimeConfig::kEnforce)
        : vpRobotFranka(franka_address, realtime_config) {};
    inline virtual ~vpRobotFrankaExtended() {};
    vpRobotFrankaExtended(const vpRobotFrankaExtended& copy) = delete;
    vpRobotFrankaExtended(vpRobotFrankaExtended&& move) = delete;
    vpRobotFrankaExtended& operator=(const vpRobotFrankaExtended& copy) = delete;
    vpRobotFrankaExtended& operator=(vpRobotFrankaExtended& move) = delete;

    void setJointImpedance(const std::array<double, k_numJoints>& jointImpedances);
    void setCartesianImpedance(const std::array<double, 6>& cartesianImpedances);
    void setHomePosition(const vpColVector& jointPosition);
    void setPoseEndEffector(const vpPoseVector& desiredPose, double relSpeed=0.3);
    void setPositionToHome();
    double lowerEndEffectorWithForceFeedback(const double forceThreshold=0.8, double loweringVelocity=0.01);

private:
    vpColVector objectiveVelAvoidLimits(double k0);
    vpColVector jointVelFromCartesian(const vpColVector& cartesianVel, const vpColVector& objectiveVel, const vpMatrix& jacobian) const;
    double getRequiredMotionTime(const double errNorm, const std::array<double, 3>& cartesianLimits, const double relSpeed) const;
    double polynomial(const double x, const std::vector<double>& coeffs) const;
    double trajectory1D(const double time, const double err, const double finishTime) const;

    vpColVector m_homePosition{std::vector<double>{0.118737, 0.199942, -0.0110379, -2.67651, 0.00768487, 2.83461, 0.872444}};

    static const std::array<double, k_numJoints> k_q_min;
    static const std::array<double, k_numJoints> k_q_max;
    static const std::array<double, k_numJoints> k_dq_max;
    static const std::array<double, k_numJoints> k_ddq_max;
    static const std::array<double, k_numJoints> k_dddq_max;
    static const std::array<double, k_numJoints> k_torqueLimits;
    static const std::array<double, k_numJoints> k_rotatumLimits;
    static const std::array<double, 3> k_translationalLimits;
    static const std::array<double, 3> k_rotationalLimits;
    static const std::array<double, 3> k_elbowLimits;
    static const std::array<double, 3> k_conservativeLimits;
};
#endif
