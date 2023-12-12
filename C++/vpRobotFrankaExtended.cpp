#include "vpRobotFrankaExtended.h"

#include <boost/algorithm/clamp.hpp>

const std::array<double, vpRobotFrankaExtended::k_numJoints> vpRobotFrankaExtended::k_q_min{-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973};
const std::array<double, vpRobotFrankaExtended::k_numJoints> vpRobotFrankaExtended::k_q_max{2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973};
const std::array<double, vpRobotFrankaExtended::k_numJoints> vpRobotFrankaExtended::k_dq_max{2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100};
const std::array<double, vpRobotFrankaExtended::k_numJoints> vpRobotFrankaExtended::k_ddq_max{15.0, 7.5, 10.0, 12.5, 15.0, 20.0, 20.0};
const std::array<double, vpRobotFrankaExtended::k_numJoints> vpRobotFrankaExtended::k_dddq_max{7500, 3750, 5000, 6250, 7500, 10000, 10000};
const std::array<double, vpRobotFrankaExtended::k_numJoints> vpRobotFrankaExtended::k_torqueLimits{87, 87, 87, 87, 12, 12, 12};
const std::array<double, vpRobotFrankaExtended::k_numJoints> vpRobotFrankaExtended::k_rotatumLimits{1000, 1000, 1000, 1000, 1000, 1000};
const std::array<double, 3> vpRobotFrankaExtended::k_translationalLimits{1.7000, 13.0000, 6500.0000};
const std::array<double, 3> vpRobotFrankaExtended::k_rotationalLimits{2.5000, 25.0000, 12500.0000};
const std::array<double, 3> vpRobotFrankaExtended::k_elbowLimits{2.1750, 10.0000, 5000.0000};
const std::array<double, 3> vpRobotFrankaExtended::k_conservativeLimits{1.7000, 10.0000, 5000.0000};


void vpRobotFrankaExtended::setJointImpedance(const std::array<double, k_numJoints>& jointImpedances)
{
    getHandler()->setJointImpedance(jointImpedances);
}

void vpRobotFrankaExtended::setCartesianImpedance(const std::array<double, 6>& cartesianImpedances)
{
    getHandler()->setCartesianImpedance(cartesianImpedances);
}

// Lowers the end effector with a given lowering velocity until a force acting on the end effector (resolved along the z-axis) is
// measured that exceeds forceThreshold. This is a blocking function.
// Returns the measured value of the force that exceeded the threshold.
// TODO: Generalise this to move in any direction with forces resolved along that particular direction
// TODO: Potentially make this a function that can be called repeatedly in a control loop as vpRobotFranka::setVelocity so the user can control when it stops.
double vpRobotFrankaExtended::lowerEndEffectorWithForceFeedback(const double forceThreshold, double loweringVelocity)
{
    // Enforce that a lowering velocity actually is a lowering velocity
    loweringVelocity = -std::abs(loweringVelocity);
    double measuredForce{0.0};
    vpColVector forcesEndEffector(6u, 0.0);
    vpColVector cartesianVelocities{std::vector<double>{0.0, 0.0, loweringVelocity, 0.0, 0.0, 0.0}};
    vpColVector jointVelocities(k_numJoints, 0.0);
    vpMatrix zeroJacobian{};
    setRobotState(STATE_VELOCITY_CONTROL);
    while(measuredForce < forceThreshold)
    {
        get_fJe(zeroJacobian);
        jointVelocities = zeroJacobian.pseudoInverse() * cartesianVelocities;
        setVelocity(vpRobotFranka::JOINT_STATE, jointVelocities);
        getForceTorque(vpRobotFranka::END_EFFECTOR_FRAME, forcesEndEffector);
        measuredForce = forcesEndEffector[2];
    }
    setRobotState(STATE_STOP);
    return measuredForce;
}

// Returns the value of a polynomial of order coeffs.size() evaluated at the position x. Coefficients are from low to high order, e.g. a, bx, cx^2 ...
double vpRobotFrankaExtended::polynomial(const double x, const std::vector<double>& coeffs) const
{
    // Working as intended
	double func{0};
	for (int i{0}; i < coeffs.size(); ++i)
	{
		func += coeffs[i] * std::pow(x, i);
	}
	return func;
}

// Takes a euclidean norm of the error between two position states e.g. an intial and final x,y,z position / a,B,Y Euler angles
// Takes limits for velocity, acceleration and jerk (again translational or rotational limits)
// Returns the required minimum time to move between the intial and final state that will not exceed some fraction, relSpeed, of these limits
double vpRobotFrankaExtended::getRequiredMotionTime(const double errNorm, const std::array<double, 3>& cartesianLimits, const double relSpeed) const
{
    // Working as intended
    vpColVector motionTimes{cartesianLimits.size(), 0.0};
    motionTimes[0] = 15 * errNorm / (8 * relSpeed * cartesianLimits[0]);
    motionTimes[1] = std::sqrt(45 * errNorm / (8 * relSpeed * cartesianLimits[1]));
    motionTimes[2] = std::cbrt(60 * errNorm / (relSpeed * cartesianLimits[2]));
    return motionTimes.getMaxValue();
}

// Returns the value assumed by a state at a given time along a smooth path between intial and final states
// given an error between intial and final state of err and a desired finish time to minimise the error
// TODO: This is currently impelemented as a velocity trajectory, want to make this more general.
double vpRobotFrankaExtended::trajectory1D(const double time, const double err, const double finishTime) const
{
    std::vector<double> coeffs{
        0.0,
        0.0,
        3 * 10 * err / std::pow(finishTime, 3),
        4 * (-15) * err / std::pow(finishTime, 4),
        5 * 6 * err / std::pow(finishTime, 5),
        0.0
    };
    return polynomial(time, coeffs);
}

vpColVector vpRobotFrankaExtended::objectiveVelAvoidLimits(double k0)
{
    vpColVector jointLimitsObjectiveVel(k_numJoints, 0.0);
    vpColVector currentJointPosition(k_numJoints, 0.0);
    getPosition(JOINT_STATE, currentJointPosition);
    double midLimit{0};
    for(int i{0}; i < k_numJoints; ++i)
    {
        midLimit = (k_q_min[i] + k_q_max[i]) / 2;
        jointLimitsObjectiveVel[i] = -(k0/k_numJoints) * (currentJointPosition[i] - midLimit) / (k_q_max[i] - k_q_min[i]);
    }
    return jointLimitsObjectiveVel;
}

vpColVector vpRobotFrankaExtended::jointVelFromCartesian(const vpColVector& cartesianVel, const vpColVector& objectiveVel, const vpMatrix& jacobian) const
{
    vpColVector currentJointVel(k_numJoints, 0.0);
    vpMatrix jacobianPInv{jacobian.pseudoInverse()};
    vpMatrix identity(k_numJoints, k_numJoints);
    identity.eye();
    currentJointVel = jacobianPInv * cartesianVel;
    currentJointVel += (identity - (jacobianPInv * jacobian)) * objectiveVel;

    return currentJointVel;
}

void vpRobotFrankaExtended::setPoseEndEffector(const vpPoseVector& desiredPose, double relSpeed)
{
    if(relSpeed <= 0 || relSpeed > 1)
	{
		std::cout << "Invalid value for relSpeed: choose a value between 0 (exclusive) and 1 (inclusive). Clamping relSpeed.\n";
		relSpeed = boost::algorithm::clamp(relSpeed, 0, 1);
	}
    setMaxTranslationVelocity(k_translationalLimits[0]);
    setMaxRotationVelocity(k_rotationalLimits[0]);

	vpPoseVector startPose{desiredPose};
	getPosition(END_EFFECTOR_FRAME, startPose);
	const vpColVector poseErr{static_cast<vpColVector>(desiredPose) - startPose};
    const vpColVector positionErr{poseErr.extract(0, 3)};
    const vpColVector rotationErr{poseErr.extract(3, 3)};
    const double translationFinishTime{getRequiredMotionTime(positionErr.euclideanNorm(), k_conservativeLimits, relSpeed)};
    const double rotationFinishTime{getRequiredMotionTime(rotationErr.euclideanNorm(), k_conservativeLimits, relSpeed)};

	constexpr double k_timePerCycle{0.001};
    // Bake the trajectory before motion starts so that the control loop can run at the required speed
    // TODO: Have an option to do this at compile-time or multi-threaded
    // TODO: Convert to using joint velocities
    vpColVector plannedJointPositions(7u, 0.0);
    getPosition(JOINT_STATE, plannedJointPositions);
    vpMatrix zeroJacobian{};
    std::vector<vpColVector> trajectory;
	for(double plannedTime{0}; plannedTime < translationFinishTime || plannedTime < rotationFinishTime; plannedTime += k_timePerCycle)
	{
        vpColVector velocity(7u, 0.0);
		vpColVector velocityCart(6u, 0.0);
		for(int i{0}; i < positionErr.size(); ++i)
        {
            velocityCart[i] = trajectory1D(plannedTime, positionErr[i], translationFinishTime);
        }
        for(int i{0}; i < rotationErr.size(); ++i)
        {
            // Bugged currently, rotational velocity seems to spiral out of control.
            // TODO: Fix this
            //velocity[i+positionErr.size()] = trajectory1D(plannedTime, rotationErr[i], rotationFinishTime);
        }
        get_fJe(plannedJointPositions, zeroJacobian);
        velocity = jointVelFromCartesian(velocityCart, objectiveVelAvoidLimits(1), zeroJacobian);
        trajectory.push_back(velocity);
        plannedJointPositions += velocity * k_timePerCycle;
	}

    int index{0};
    double time{0};
    double startTime{vpTime::measureTimeSecond()};
    setRobotState(vpRobotFranka::STATE_VELOCITY_CONTROL);
    while (time < translationFinishTime || time < rotationFinishTime)
    {
		// Map the current *real* time to the closest corresponding index in the baked trajectory array,
        // each index increment corresponding to a timestep of k_timePerCycle
		index = boost::algorithm::clamp(static_cast<int>(std::round(time / k_timePerCycle)), 0, trajectory.size() - 1);
	    setVelocity(JOINT_STATE, trajectory[index]);
        time = vpTime::measureTimeSecond() - startTime;
    }
    setRobotState(STATE_STOP);
}

// Sets the robot (joint) positions to the home position
void vpRobotFrankaExtended::setPositionToHome()
{
    setRobotState(STATE_POSITION_CONTROL);
    setPosition(JOINT_STATE, m_homePosition);
    setRobotState(STATE_STOP);
}

// Sets the home position in joint space
void vpRobotFrankaExtended::setHomePosition(const vpColVector& jointPosition)
{
    m_homePosition = jointPosition;
}

/*
// Set the 6DoF pose of the end effector relative to the robot base frame.
// Currently, velocity is set by proportional control with the 6DoF error taken as the difference between the desired and current 6DoF poses.
// relSpeed is the constant of proportionality for this proportional control. Value of around 0.3 recommended for now.
void setPoseEndEffector(vpRobotFranka& robot, const vpPoseVector& desiredPose, const double relSpeed=0.3)
{
    // When the error of each coordinate drops below this, the motion is finished.
    // Can't use 0 due to floating point precision errors and physical limitations of the arm.
    constexpr double ktargetErrNorm = 5e-4;
	vpPoseVector currentPose{desiredPose};
	vpColVector poseErr(6, 0.0);
    bool finishedMotion{false};

    while (!finishedMotion)
    {
        robot.getPosition(vpRobotFranka::END_EFFECTOR_FRAME, currentPose);
        poseErr = static_cast<vpColVector>(desiredPose) - currentPose;
	    robot.setVelocity(vpRobotFranka::REFERENCE_FRAME, relSpeed * poseErr);
	    for(int i{0}; i < poseErr.size(); ++i)
        {
            if(poseErr[i] >= ktargetErrNorm) break;
            if(i == (poseErr.size() - 1)) finishedMotion = true;
        }
    } ;
    std::cout << poseErr << "\n\n";
}
*/
