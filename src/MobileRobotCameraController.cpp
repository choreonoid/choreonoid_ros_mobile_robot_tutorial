#include <cnoid/SimpleController>
#include <ros/node_handle.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <mutex>

using namespace std;
using namespace cnoid;


class MobileRobotCameraController : public SimpleController
{
    std::unique_ptr<ros::NodeHandle> node;
    ros::Subscriber subscriber;
    sensor_msgs::JointState command;
    std::mutex commandMutex;
    Link* joints[2];
    double q_target[2];

    virtual bool initialize(SimpleControllerIO* io) override;
    void commandCallback(const sensor_msgs::JointState& msg);
    virtual bool control() override;
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(MobileRobotCameraController)


bool MobileRobotCameraController::initialize(SimpleControllerIO* io)
{
    auto body = io->body();

    joints[0] = body->joint("PanLink");
    joints[1] = body->joint("TiltLink");

    for(int i = 0; i < 2; ++i){
        Link* joint = joints[i];
        joint->setActuationMode(JointTorque);
        io->enableInput(joint, JointAngle | JointVelocity);
        io->enableOutput(joint, JointTorque);
    }

    node = make_unique<ros::NodeHandle>(body->name() + "Camera");
    subscriber = node->subscribe(
        "/joint_command", 1, &MobileRobotCameraController::commandCallback, this);

    return true;
}

void MobileRobotCameraController::commandCallback(const sensor_msgs::JointState& msg)
{
    std::lock_guard<std::mutex> lock(commandMutex);
    command = msg;
}

bool MobileRobotCameraController::control()
{
    const double kp = 0.1;
    const double kd = 0.05;

    {
        std::lock_guard<std::mutex> lock(commandMutex);
        if (command.position.size() >= 2){
            for(int i = 0; i < 2; ++i){
                q_target[i] = command.position.at(i);
            }
        }
    }
    
    for(int i=0; i < 2; ++i){
        Link* joint = joints[i];
        joint->u() = - kp * (joint->q() - q_target[i]) - kd * joint->dq();
    }

    return true;
}
