#include "RigidBodySystemSimulator.h"


#include <vector>


struct RigidBody
{
	Vec3 position = Vec3();
	Vec3 scale = Vec3();
	Quat rotation = Quat();
	Vec3 linearVelocity = Vec3();
	Vec3 angularVelociy = Vec3();
	Vec3 angularMomentum = Vec3();
	Vec3 force = Vec3();
	Vec3 torque = Vec3();
	Vec3 inverseInertia= Vec3(); //Diagonal of matrix
	float mass = 1.;
};

std::vector<RigidBody> rigidBodies;

Vec3 calcInvInertiaCube(Vec3 size, float mass)
{
	float factor = mass / 12.;
    float w2 = size.x * size.x;
    float h2 = size.y * size.y;
    float d2 = size.z * size.z;
    return Vec3(1. / (factor * (h2 + d2)),
    			1. / (factor * (w2 + d2)),
   				1. / (factor * (h2 + w2)));
}

// Construtors
RigidBodySystemSimulator::RigidBodySystemSimulator(){}

// Functions
const char * RigidBodySystemSimulator::getTestCasesStr()
{
	return "demo1,demo2,demo3,demo4";
}
void RigidBodySystemSimulator::initUI(DrawingUtilitiesClass * DUC)
{
	this->DUC = DUC;
	rigidBodies.clear();
	switch (m_iTestCase){
		case 1:
			addRigidBody(Vec3(),Vec3(1.,.6,.5),2.);
			setOrientationOf(0,Quat(Vec3(0,0,1),M_PI*.5));
		break;


	}
}
void RigidBodySystemSimulator::reset()
{
	rigidBodies.clear();
}
void RigidBodySystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	for(auto &rb: rigidBodies){
		DUC->drawRigidBody(/*TODO:*/rb);
	}
}
void RigidBodySystemSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
}
void RigidBodySystemSimulator::externalForcesCalculations(float timeElapsed){}
void RigidBodySystemSimulator::simulateTimestep(float timeStep){}
void RigidBodySystemSimulator::onClick(int x, int y){}
void RigidBodySystemSimulator::onMouse(int x, int y){}

	// ExtraFunctions
int RigidBodySystemSimulator::getNumberOfRigidBodies()
{
	return static_cast<int>(rigidBodies.size());
}
Vec3 RigidBodySystemSimulator::getPositionOfRigidBody(int i)
{
	if(i < getNumberOfRigidBodies())
		return rigidBodies[i].position;
	return Vec3();
}
Vec3 RigidBodySystemSimulator::getLinearVelocityOfRigidBody(int i)
{
	if(i < getNumberOfRigidBodies())
		return rigidBodies[i].linearVelocity;
	return Vec3();
}
Vec3 RigidBodySystemSimulator::getAngularVelocityOfRigidBody(int i)
{
	if(i < getNumberOfRigidBodies())
		return rigidBodies[i].angularVelociy;
	return Vec3();
}
void RigidBodySystemSimulator::applyForceOnBody(int i, Vec3 loc, Vec3 force)
{
	if(!(i < getNumberOfRigidBodies()))
		return;
	rigidBodies[i].force += force;
	rigidBodies[i].torque += cross(loc,force);
		
}
void RigidBodySystemSimulator::addRigidBody(Vec3 position, Vec3 size, int mass)
{
	RigidBody rb{};
	rb.position = position;
	rb.scale = size;
	rb.mass = mass;
	rigidBodies.emplace_back(rb);

}
void RigidBodySystemSimulator::setOrientationOf(int i,Quat orientation)
{
	if(!(i < getNumberOfRigidBodies()))
		return;
	rigidBodies[i].rotation = orientation;
}
void RigidBodySystemSimulator::setVelocityOf(int i, Vec3 velocity)
{
	if(!(i < getNumberOfRigidBodies()))
		return;
	rigidBodies[i].linearVelocity = velocity;
}
