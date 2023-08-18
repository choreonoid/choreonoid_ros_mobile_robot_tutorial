#include <cnoid/SimpleController>
#include <ros/node_handle.h>
#include <geometry_msgs/Twist.h>
#include <mutex>

using namespace std;
using namespace cnoid;


class MobileRobotTwistController : public SimpleController
{
    std::unique_ptr<ros::NodeHandle> node;
    ros::Subscriber subscriber;
    geometry_msgs::Twist command;
    std::mutex commandMutex;
    Link* wheels[2];
    double dq_target[2];

    virtual bool initialize(SimpleControllerIO* io) override;
    void commandCallback(const geometry_msgs::Twist& twist);
    virtual bool control() override;
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(MobileRobotTwistController)


bool MobileRobotTwistController::initialize(SimpleControllerIO* io)
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

    node = make_unique<ros::NodeHandle>(body->name());
    subscriber = node->subscribe(
        "/cmd_vel", 1, &MobileRobotTwistController::commandCallback, this);
    
    return true;
}

void MobileRobotTwistController::commandCallback(const geometry_msgs::Twist& msg)
{
    std::lock_guard<std::mutex> lock(commandMutex);
    command = msg;
}

bool MobileRobotTwistController::control()
{
    constexpr double wheelRadius = 0.076;
    constexpr double halfAxleWidth = 0.142;
    constexpr double kdd = 5.0;
    double dq_target[2];
    
    {
        std::lock_guard<std::mutex> lock(commandMutex);
        double dq_x = command.linear.x / wheelRadius;
        double dq_yaw = command.angular.z * halfAxleWidth / wheelRadius;
        dq_target[0] = dq_x - dq_yaw;
        dq_target[1] = dq_x + dq_yaw;
    }
    
    for(int i=0; i < 2; ++i){
        auto wheel = wheels[i];
        wheel->u() = kdd * (dq_target[i] - wheel->dq());
    }
    
    return true;
}
