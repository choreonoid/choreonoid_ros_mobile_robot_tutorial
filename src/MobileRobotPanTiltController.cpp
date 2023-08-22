#include <cnoid/SimpleController>
#include <ros/node_handle.h>
#include <geometry_msgs/Vector3.h>
#include <mutex>

class MobileRobotPanTiltController : public cnoid::SimpleController
{
public:
    virtual bool initialize(cnoid::SimpleControllerIO* io) override;
    void commandCallback(const geometry_msgs::Vector3& omega);
    virtual bool control() override;

private:
    cnoid::Link* joints[2];
    std::unique_ptr<ros::NodeHandle> node;
    ros::Subscriber subscriber;
    geometry_msgs::Vector3 command;
    std::mutex commandMutex;
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(MobileRobotPanTiltController)

bool MobileRobotPanTiltController::initialize(cnoid::SimpleControllerIO* io)
{
    auto body = io->body();
    joints[0] = body->joint("PanLink");
    joints[1] = body->joint("TiltLink");
    for(int i = 0; i < 2; ++i){
        cnoid::Link* joint = joints[i];
        joint->setActuationMode(JointTorque);
        io->enableInput(joint, JointAngle | JointVelocity);
        io->enableOutput(joint, JointTorque);
    }

    node = std::make_unique<ros::NodeHandle>();
    subscriber = node->subscribe(
        "/cmd_joint_vel", 1, &MobileRobotPanTiltController::commandCallback, this);

    return true;
}

void MobileRobotPanTiltController::commandCallback(const geometry_msgs::Vector3& omega)
{
    std::lock_guard<std::mutex> lock(commandMutex);
    command = omega;
}

bool MobileRobotPanTiltController::control()
{
    constexpr double kd = 0.5;
    double dq_target[2];

    {
        std::lock_guard<std::mutex> lock(commandMutex);
        dq_target[0] = command.z;
        dq_target[1] = command.y;
    }
    
    for(int i=0; i < 2; ++i){
        cnoid::Link* joint = joints[i];
        joint->u() = kd * (dq_target[i] - joint->dq());
    }

    return true;
}
