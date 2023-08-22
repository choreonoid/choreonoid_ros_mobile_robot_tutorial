#include <cnoid/SimpleController>
#include <ros/node_handle.h>
#include <geometry_msgs/Twist.h>
#include <mutex>

class MobileRobotDriveController : public cnoid::SimpleController
{
public:
    virtual bool initialize(cnoid::SimpleControllerIO* io) override;
    void commandCallback(const geometry_msgs::Twist& twist);
    virtual bool control() override;

private:
    cnoid::Link* wheels[2];
    std::unique_ptr<ros::NodeHandle> node;
    ros::Subscriber subscriber;
    geometry_msgs::Twist command;
    std::mutex commandMutex;
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(MobileRobotDriveController)

bool MobileRobotDriveController::initialize(cnoid::SimpleControllerIO* io)
{
    auto body = io->body();
    wheels[0] = body->joint("RightWheel");
    wheels[1] = body->joint("LeftWheel");
    for(int i=0; i < 2; ++i){
        auto wheel = wheels[i];
        wheel->setActuationMode(JointTorque);
        io->enableInput(wheel, JointVelocity);
        io->enableOutput(wheel, JointTorque);
    }

    node = std::make_unique<ros::NodeHandle>();
    subscriber = node->subscribe(
        "/cmd_vel", 1, &MobileRobotDriveController::commandCallback, this);
    
    return true;
}

void MobileRobotDriveController::commandCallback(const geometry_msgs::Twist& twist)
{
    std::lock_guard<std::mutex> lock(commandMutex);
    command = twist;
}

bool MobileRobotDriveController::control()
{
    constexpr double wheelRadius = 0.076;
    constexpr double halfAxleWidth = 0.145;
    constexpr double kd = 0.5;
    double dq_target[2];
    
    {
        std::lock_guard<std::mutex> lock(commandMutex);
        double dq_x = command.linear.x / wheelRadius;
        double dq_yaw = command.angular.z * halfAxleWidth / wheelRadius;
        dq_target[0] = dq_x + dq_yaw;
        dq_target[1] = dq_x - dq_yaw;
    }
    
    for(int i=0; i < 2; ++i){
        auto wheel = wheels[i];
        wheel->u() = kd * (dq_target[i] - wheel->dq());
    }
    
    return true;
}
