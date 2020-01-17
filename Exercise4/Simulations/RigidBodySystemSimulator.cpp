#include "RigidBodySystemSimulator.h"


Scalar dot(Quat &a, Quat &b)
{
	return a.dot(b);
}

Quat conjugate(Quat &q)
{
	return Quat(-q.x, -q.y, -q.z,q.w,);
}

Quat inverse(Quat &q)
{
	return conjugate(q) / dot(q, q);
}

Vec3 rotate(Quat &q, Vec3 &v)
{
	return q.getRotMat()*v;
}
Vec3 max(Vec3 &a, Vec3 &b)
{
	return Vec3(max(a.x,b.x),max(a.y,b.y),max(a.z,b.z));
}

Vec3 abs(Vec3 &a)
{
	return Vec3(abs(a.x),abs(a.y),abs(a.z));
}


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

	TwAddVarRW(DUC->g_pTweakBar, "Gravitation", TW_TYPE_BOOLCPP, &gravitation, "");
	TwAddVarRW(DUC->g_pTweakBar, "Time Step", TW_TYPE_DOUBLE, &timeStep, "min=0.00001 max=2");
	TwAddSeparator(DUC->g_pTweakBar, "sep0", NULL);

	TwAddButton(DUC->g_pTweakBar, "Remove balls", removeBasketballsCallback, NULL, NULL);
	TwAddVarRW(DUC->g_pTweakBar, "Ball Mass", TW_TYPE_DOUBLE, &ballMass, "min=0.001 max=100");
	TwAddVarRW(DUC->g_pTweakBar, "Ball Scale", TW_TYPE_DOUBLE, &ballScale, "min=0.001 max=100");
	TwAddSeparator(DUC->g_pTweakBar, "sep1", NULL);

	TwAddVarRW(DUC->g_pTweakBar, "Net Mass", TW_TYPE_DOUBLE, &netMass, "min=0.0001");
	TwAddVarRW(DUC->g_pTweakBar, "Net Damping", TW_TYPE_DOUBLE, &netDamping, "min=0.0");
	TwAddVarRW(DUC->g_pTweakBar, "Net Bounciness", TW_TYPE_DOUBLE, &netBounciness, "min=0.0");
	TwAddVarRW(DUC->g_pTweakBar, "Net Collision", TW_TYPE_BOOLCPP, &netCollision, "");
	TwAddSeparator(DUC->g_pTweakBar, "sep2", NULL);

	TwAddVarRW(DUC->g_pTweakBar, "Basket Segments", TW_TYPE_INT32, &basketSegmnets, "min=1");
	TwAddVarRW(DUC->g_pTweakBar, "Basket Scale", TW_TYPE_DOUBLE, &basketScale, "min=0.001");
	TwAddSeparator(DUC->g_pTweakBar, "sep3", NULL);
	initScene();
	}

void RigidBodySystemSimulator::initScene()
{
	addBasket(Vec3(0, 0.5, 2), basketScale, basketSegmnets);
}

void RigidBodySystemSimulator::reset()
{
	rigidBodies.clear();
	initScene();
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

		rb.force = Vec3(0., gravitation?-10:0, 0);
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
void RigidBodySystemSimulator::addRigidBody(Vec3 position, Vec3 size, int mass, Quat rotation = Quat(), bool isSphere = false, bool isStatic = false)
{
	RigidBody rb{};
	rb.position = position;
	rb.scale = size;
	rb.mass = mass;
	rb.isSphere = isSphere;
	rb.isStatic = isStatic;
	rb.rotation = rotation;
	rb.inverseInertia = calcInvInertiaCube(rb.scale, rb.mass);
	rigidBodies.push_back(rb);
}


CollisionInfo RigidBodySystemSimulator::getCollision(RigidBody &a, RigidBody &b)
{
	CollisionInfo collision = CollisionInfo{};
	collision.isValid = false;
	collision.collisionPointWorld = Vec3(0.0);
	collision.normalWorld = Vec3(0.0);
	collision.depth = 0.0;

	if (a.isSphere || b.isSphere)
	{
		if (a.isSphere && b.isSphere)
		{
			double distance =((a.position.x - b.position.x) * (a.position.x - b.position.x) +
				(a.position.y - b.position.y) * (a.position.y - b.position.y) +
				(a.position.z - b.position.z) * (a.position.z - b.position.z));
			collision.isValid = distance < (a.scale.x + b.scale.x)*(a.scale.x + b.scale.x);
			if (collision.isValid)
			{
				collision.collisionPointWorld = a.position + (b.position - a.position) / 2;
				//TODO: collision normal with friction to add rotation;
			}
		}
		if (a.isSphere)
		{
			collision = checkCollisionSphereCube(a, b);
		}
		else
		{
			collision =		checkCollisionSphereCube(b, a);
		}
	}
	else if(!(a.isStatic && b.isStatic))
	{
		GamePhysics::Mat4 matA = a.rotation.getRotMat();
		matA.initScaling(a.scale.x, a.scale.y, a.scale.z);
		matA.initTranslation(a.position.x, a.position.y, a.position.z);

		GamePhysics::Mat4 matB = b.rotation.getRotMat();
		matB.initScaling(b.scale.x, b.scale.y, b.scale.z);
		matB.initTranslation(b.position.x, b.position.y, b.position.z);
		collision = checkCollisionSAT(matB, matA);
	}
	return collision;
}

CollisionInfo RigidBodySystemSimulator::checkCollisionSphereCube(RigidBody &sphere, RigidBody &box)
{
	CollisionInfo collision = CollisionInfo{};
	collision.isValid = false;
	collision.collisionPointWorld = Vec3(0.0);
	collision.normalWorld = Vec3(0.0);
	collision.depth = 0.0;

	GamePhysics::Mat4 matBox = box.rotation.getRotMat();
	matBox.initScaling(box.scale.x, box.scale.y, box.scale.z);
	matBox.initTranslation(box.position.x, box.position.y, box.position.z);
	vector<Vec3> corners;
	for (int i = 0; i < 8; i++) {
		int x = i % 2 * 2 - 1;
		int y = i / 2 % 2 * 2 - 1;
		int z = i / 4 % 2 * 2 - 1;
		corners[i] = matBox.transformVector(Vec3(x, y, z));
	}
	double maxX = corners[0].x;
	double minX = corners[0].x;
	double maxY = corners[0].y;
	double minY = corners[0].y;
	double maxZ = corners[0].z;
	double minZ = corners[0].z;
	for (int i = 1; i < 8; i++)
	{
		double maxX = corners[i].x > maxX ? corners[i].x : maxX;
		double minX = corners[i].x < minX ? corners[i].x : minX;
		double maxY = corners[i].y > maxY ? corners[i].y : maxY;
		double minY = corners[i].y < minY ? corners[i].y : minY;
		double maxZ = corners[i].z > maxZ ? corners[i].z : maxZ;
		double minZ = corners[i].z < minZ ? corners[i].z : minZ;
	}

	
	double x = std::max(minX, std::min(sphere.position.x, maxX));
	double y = std::max(minY, std::min(sphere.position.y, maxY));
	double z = std::max(minZ, std::min(sphere.position.z, maxZ));

	double distance = (x - sphere.position.x) * (x - sphere.position.x) +
			(y - sphere.position.y) * (y - sphere.position.y) +
			(z - sphere.position.z) * (z - sphere.position.z);

	collision.isValid = distance < (sphere.scale.x * sphere.scale.x);
	if (collision.isValid)
	{
		collision.collisionPointWorld = Vec3(x, y, z);
	}
	return collision;
}

void RigidBodySystemSimulator::addBasket(Vec3 position, double scale, int segments) {
	addRigidBody(Vec3(0, -1, 0), Vec3(200, 0.001, 200), netMass, Quat(), false, true);
	float angle = 2. * M_PI / (double)segments;
		for (int i = 0; i < ((segments >= 4) ? segments : 4); i++) {
			RigidBody rb{};
			rb.position = position
				+ Vec3(1 * scale, 0., 0.) * sin(angle*i)
				+ Vec3(0., 0., 1 * scale) * cos(angle*i);
			rb.scale = scale * Vec3(sin(angle / 2.) * 2., .1, .1);
			rb.rotation = Quat(Vec3(1, 0, 0), M_PI * .25) * Quat(Vec3(0, 1, 0), 2 * M_PI * ((double)i / (double)segments));
			addRigidBody(rb.position, rb.scale, netMass, rb.rotation, false, true);
		}
	RigidBody wall{};
	wall.position = position + scale * Vec3(0, 0, 1.1);
	wall.scale = scale * Vec3(3., 3., .1);
	addRigidBody(wall.position, wall.scale, netMass, Quat(), false , true);	
	addRigidBody(Vec3(0, 4, 2), Vec3(0.2, 0, 0), ballMass, Quat(), true);
}

// UI Callback Methods
void TW_CALL RigidBodySystemSimulator::removeBasketballsCallback(void* optionalData) {
	// TODO: Implement this lateron
	std::cout << "TODO: Implement deleting of Basketballs\n";
}