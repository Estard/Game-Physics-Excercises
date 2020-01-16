#include "RigidBodySystemSimulator.h"
#include "collisionDetect.h"



Vec3 RigidBodySystemSimulator::calcInvInertiaCube(Vec3 size, float mass)
{
	double factor = mass / 12.;
	double w2 = size.x * size.x;
	double h2 = size.y * size.y;
	double d2 = size.z * size.z;
	return Vec3(1. / (factor * (h2 + d2)),
		1. / (factor * (w2 + d2)),
		1. / (factor * (h2 + w2)));
}

Vec3 RigidBodySystemSimulator::calcInvInertiaSphere(float radius, float mass, bool solid)
{
	float mrr = mass*radius*radius;
	float factor = solid?5.:3.;
	return Vec3((1./mrr)*.5*factor);
}

// Construtors
RigidBodySystemSimulator::RigidBodySystemSimulator() {
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

	gravitation = false;

	netMass = 1.0f;
	netCollision = false;
	netBounciness = 0.0f;
	netDamping = 0.1f;

	basketScale = 1.0f;
	basketSegmnets = 4;

	TwAddVarRW(DUC->g_pTweakBar, "Gravitation", TW_TYPE_BOOLCPP, &gravitation, "");
	TwAddSeparator(DUC->g_pTweakBar, "sep0", NULL);

	TwAddButton(DUC->g_pTweakBar, "Remove balls", removeBasketballsCallback, NULL, NULL);
	TwAddSeparator(DUC->g_pTweakBar, "sep1", NULL);

	TwAddVarRW(DUC->g_pTweakBar, "Net Mass", TW_TYPE_FLOAT, &netMass, "min=0.001");
	TwAddVarRW(DUC->g_pTweakBar, "Net Damping", TW_TYPE_FLOAT, &netDamping, "min=0.0");
	TwAddVarRW(DUC->g_pTweakBar, "Net Bounciness", TW_TYPE_FLOAT, &netBounciness, "min=0.0");
	TwAddVarRW(DUC->g_pTweakBar, "Net Collision", TW_TYPE_BOOLCPP, &netCollision, "");
	TwAddSeparator(DUC->g_pTweakBar, "sep2", NULL);

	TwAddVarRW(DUC->g_pTweakBar, "Basket Segments", TW_TYPE_INT32, &basketSegmnets, "min=1");
	TwAddVarRW(DUC->g_pTweakBar, "Basket Scale", TW_TYPE_FLOAT, &basketScale, "min=0.001");
	TwAddSeparator(DUC->g_pTweakBar, "sep3", NULL);

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

void RigidBodySystemSimulator::reset()
{
	rigidBodies.clear();
}
void RigidBodySystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	for (auto& rb : rigidBodies) {
		if (rb.isSphere) {
			DUC->drawSphere(rb.position, Vec3(rb.scale.x));
		}
		else
		{
			auto rot = rb.rotation.getRotMat();
			Mat4 scale;
			scale.initScaling(rb.scale.x, rb.scale.y, rb.scale.z);
			Mat4 translate;
			translate.initTranslation(rb.position.x, rb.position.y, rb.position.z);
			DUC->drawRigidBody(scale * rot * translate);
		}
	}
}
void RigidBodySystemSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
}
void RigidBodySystemSimulator::externalForcesCalculations(float timeElapsed) {}

void  RigidBodySystemSimulator::integrate(RigidBody &rb)
{		
		rb.linearVelocity += timeStep * rb.force / rb.mass;

		rb.rotation = rb.rotation + (Quat(rb.angularVelocity.x, rb.angularVelocity.y, rb.angularVelocity.z, 0) * rb.rotation) * .5 * timeStep;

		rb.rotation = rb.rotation.unit();
		rb.angularMomentum += timeStep * rb.torque;
		
		Mat4 Rot = rb.rotation.getRotMat();
		Mat4 RotT = Rot.inverse();
		Mat4 tmp;
		tmp.initScaling(rb.inverseInertia.x, rb.inverseInertia.y, rb.inverseInertia.z);
		Mat4 Im1 = Rot * tmp * RotT;

		rb.angularVelocity = Im1 * rb.angularMomentum;

		rb.position += timeStep * rb.linearVelocity;

		rb.force = Vec3(0.);
		rb.torque = Vec3(0.);
}

void RigidBodySystemSimulator::simulateTimestep(float timeStep)
{
	for (auto& rb : rigidBodies)
	{
		if(!rb.isStatic)
			integrate(rb);
	}
}

void RigidBodySystemSimulator::onClick(int x, int y) {
}
void RigidBodySystemSimulator::onMouse(int x, int y) {
}

// ExtraFunctions
void RigidBodySystemSimulator::applyForceOnBody(RigidBody &rb, Vec3 loc, Vec3 force)
{
	rb.force += force;
	rb.torque += cross(loc - rb.position, force);
}
void RigidBodySystemSimulator::addRigidBody(Vec3 position, Vec3 size, int mass, bool isSphere)
{
	RigidBody rb{};
	rb.position = position;
	rb.scale = size;
	rb.mass = mass;
	rb.inverseInertia = calcInvInertiaCube(rb.scale, rb.mass);
}
/*
CollisionInfo RigidBodySystemSimulator::getCollision(RigidBody a, RigidBody b)
{
	return CollisionInfo{};
}
*/

void RigidBodySystemSimulator::addBasket(Vec3 position, double scale, int segments) {
	float angle = 2. * M_PI / (double)segments;
		for (int i = 0; i < ((segments >= 4) ? segments : 4); i++) {
			RigidBody rb{};
			rb.position = position
				+ Vec3(1 * scale, 0., 0.) * sin(angle*i)
				+ Vec3(0., 0., 1 * scale) * cos(angle*i);
			rb.scale = scale * Vec3(sin(angle / 2.) * 2., .1, .1);
			rb.rotation = Quat(Vec3(1, 0, 0), M_PI * .25) * Quat(Vec3(0, 1, 0), 2 * M_PI * ((double)i / (double)segments));
			rigidBodies.push_back(rb);
		}
	RigidBody wall{};
	wall.position = position + scale * Vec3(-0.5, 0, 2.);
	wall.scale = scale * Vec3(3., 3., .1);
	rigidBodies.push_back(wall);
}

// UI Callback Methods
void TW_CALL RigidBodySystemSimulator::removeBasketballsCallback(void* optionalData) {
	// TODO: Implement this lateron
	std::cout << "TODO: Implement deleting of Basketballs\n";
}