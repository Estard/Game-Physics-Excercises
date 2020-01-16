	#include "RigidBodySystemSimulator.h"


#include <vector>


struct RigidBody
{
	Vec3 position = Vec3();
	Vec3 scale = Vec3();
	Quat rotation = Quat(Vec3(1,0,0),0);
	Vec3 linearVelocity = Vec3();
	Vec3 angularVelocity = Vec3();
	Vec3 angularMomentum = Vec3();
	Vec3 force = Vec3();
	Vec3 torque = Vec3();
	Vec3 inverseInertia = Vec3(); //Diagonal of matrix
	float mass = 1.;
};

std::vector<RigidBody> rigidBodies;

Vec3 calcInvInertiaCube(Vec3 size, float mass)
{
	double factor = mass / 12.;
	double w2 = size.x * size.x;
	double h2 = size.y * size.y;
	double d2 = size.z * size.z;
	return Vec3(1. / (factor * (h2 + d2)),
		1. / (factor * (w2 + d2)),
		1. / (factor * (h2 + w2)));
}

Vec3 calcInvInertiaSphere(float radius, float mass, bool solid = true)
{
	float mrr = mass*radius*radius;
	float factor = solid?5.:3.;
	return Vec3((1./mrr)*.5*factor);
}

// Construtors
RigidBodySystemSimulator::RigidBodySystemSimulator() {
	rigidBodies.clear();
}

// Functions
const char* RigidBodySystemSimulator::getTestCasesStr()
{
	return "demo1,demo2,demo3,demo4";
}
void RigidBodySystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
	rigidBodies.clear();
	switch (m_iTestCase) {
	case 0:
		addRigidBody(Vec3(), Vec3(1., .6, .5), 2.);
		setOrientationOf(0, Quat(Vec3(0, 0, 1), M_PI * .5));
		applyForceOnBody(0, Vec3(.3, .5, .25), Vec3(1, 1, 0));
		simulateTimestep(1);
		{
		auto lin_vel = getLinearVelocityOfRigidBody(0);
		auto ang_vel = getAngularVelocityOfRigidBody(0);
		//auto worldspc_vel = lin_vel + 
		std::cout << "[Linear Velocity] x: " << lin_vel.x << " y: " << lin_vel.y << " z: " << lin_vel.z << "\n";
		std::cout << "[Angular Velocity] x: " << ang_vel.x << " y: " << ang_vel.y << " z: " << ang_vel.z << "\n";
		std::cout << "[World Velocity] ToDo\n";
		}
		break;
	case 1:
		addRigidBody(Vec3(), Vec3(1., .6, .5), 2.);
		setOrientationOf(0, Quat(Vec3(0, 0, 1), M_PI * .5));
		applyForceOnBody(0, Vec3(.3, .5, .25), Vec3(1, 1, 0));
		simulateTimestep(0.01);
		break;
	case 2:
		addRigidBody(Vec3(), Vec3(1., .6, .5), 2.);
		addRigidBody(Vec3(2,2,2), Vec3(1., .6, .5), 2.);
		setVelocityOf(1,Vec3(-1,-1,-1));
		setOrientationOf(1,Quat(Vec3(1,1,0),M_PI*.5));
	break;

	case 3:
		addRigidBody(Vec3(), Vec3(1., .6, .5), 2.);
		addRigidBody(Vec3(2,2,2), Vec3(1., .6, .5), 2.);
		addRigidBody(Vec3(5,0,0), Vec3(1., .6, .5), 2.);
		addRigidBody(Vec3(-2,-2,-2), Vec3(1., .6, .5), 2.);
		setVelocityOf(1,Vec3(-1,-1,-1));
		setVelocityOf(3,Vec3(1,1,1));
		setVelocityOf(2,Vec3(-1,0,0));
		setOrientationOf(1,Quat(Vec3(1,1,0),M_PI*.5));
	break;
	}
}
void RigidBodySystemSimulator::reset()
{
	rigidBodies.clear();
}
void RigidBodySystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	for (auto& rb : rigidBodies) {
		auto rot = rb.rotation.getRotMat();
		Mat4 scale;
		scale.initScaling(rb.scale.x, rb.scale.y, rb.scale.z);
		Mat4 translate;
		translate.initTranslation(rb.position.x, rb.position.y, rb.position.z);
		DUC->drawRigidBody(scale * rot * translate);
	}

	//DUC->drawSphere(Vec3(0, 0, 2), Vec3(0.05, 0.05, 0.05));
}
void RigidBodySystemSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
}
void RigidBodySystemSimulator::externalForcesCalculations(float timeElapsed) {}
void RigidBodySystemSimulator::simulateTimestep(float timeStep)
{
	for (auto& rb : rigidBodies) {
		rb.position += timeStep * rb.linearVelocity;
		rb.linearVelocity += timeStep * rb.force / rb.mass;

		rb.rotation = rb.rotation + (Quat(rb.angularVelocity.x, rb.angularVelocity.y, rb.angularVelocity.z, 0) * rb.rotation) * .5 * timeStep;

		rb.rotation = rb.rotation.unit();
		rb.angularMomentum += timeStep * rb.torque;
		
		Mat4 Rot = rb.rotation.getRotMat();
		Mat4 RotT = Rot.inverse();
		Mat4 tmp;
		tmp.initScaling(rb.inverseInertia.x, rb.inverseInertia.y, rb.inverseInertia.z);
		std::cout << rb.inverseInertia << "\n";
		Mat4 Im1 = Rot * tmp * RotT;

		rb.angularVelocity = Im1 * rb.angularMomentum;
		std::cerr << rb.angularVelocity.x << "\n";

		rb.force = Vec3(0.);
		rb.torque = Vec3(0.);
	}
}

void RigidBodySystemSimulator::onClick(int x, int y) {
	if (m_iTestCase == 1)
	{
		applyForceOnBody(0, Vec3(.3, .5, .25), Vec3(1, 1, 0));
	}
}
void RigidBodySystemSimulator::onMouse(int x, int y) {}

// ExtraFunctions
int RigidBodySystemSimulator::getNumberOfRigidBodies()
{
	return static_cast<int>(rigidBodies.size());
}
Vec3 RigidBodySystemSimulator::getPositionOfRigidBody(int i)
{
	if (i < getNumberOfRigidBodies())
		return rigidBodies[i].position;
	return Vec3();
}
Vec3 RigidBodySystemSimulator::getLinearVelocityOfRigidBody(int i)
{
	if (i < getNumberOfRigidBodies())
		return rigidBodies[i].linearVelocity;
	return Vec3();
}
Vec3 RigidBodySystemSimulator::getAngularVelocityOfRigidBody(int i)
{
	if (i < getNumberOfRigidBodies())
		return rigidBodies[i].angularVelocity;
	return Vec3();
}
void RigidBodySystemSimulator::applyForceOnBody(int i, Vec3 loc, Vec3 force)
{
	if (!(i < getNumberOfRigidBodies()))
		return;
	rigidBodies[i].force += force;
	rigidBodies[i].torque += cross(loc - rigidBodies[i].position, force);

}
void RigidBodySystemSimulator::addRigidBody(Vec3 position, Vec3 size, int mass)
{
	RigidBody rb{};
	rb.position = position;
	rb.scale = size;
	rb.mass = mass;
	rb.inverseInertia = calcInvInertiaCube(rb.scale, rb.mass);
	rigidBodies.push_back(rb);

}
void RigidBodySystemSimulator::setOrientationOf(int i, Quat orientation)
{
	if (!(i < getNumberOfRigidBodies()))
		return;
	rigidBodies[i].rotation = orientation.unit();
}
void RigidBodySystemSimulator::setVelocityOf(int i, Vec3 velocity)
{
	if (!(i < getNumberOfRigidBodies()))
		return;
	rigidBodies[i].linearVelocity = velocity;
}
