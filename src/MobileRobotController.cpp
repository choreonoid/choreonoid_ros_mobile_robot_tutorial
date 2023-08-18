#include <cnoid/SimpleController>

using namespace cnoid;

class MobileRobotController : public SimpleController
{
    Link* wheels[2];

    virtual bool initialize(SimpleControllerIO* io) override;
    virtual bool control() override;
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(MobileRobotController)

bool MobileRobotController::initialize(SimpleControllerIO* io)
{
    wheels[0] = io->body()->joint("RightWheel");
    wheels[1] = io->body()->joint("LeftWheel");
    for(int i=0; i < 2; ++i){
        auto wheel = wheels[i];
        wheel->setActuationMode(JointTorque);
        io->enableOutput(wheel, JointTorque);
    }
    return true;
}

bool MobileRobotController::control()
{
    wheel[0]->u() = 1.0;
    wheel[0]->u() = -1.0;
    return true;
}
