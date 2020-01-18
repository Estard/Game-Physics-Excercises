#include "RigidBodySystemSimulator.h"

Vec3 lastPoint = Vec3();
Vec3 lastNormal = Vec3();

Mat4 RigidBodySystemSimulator::calcInvInertiaCube(Vec3 size, double mass)
{
	double factor = mass / 12.;
	double w2 = size.x * size.x;
	double h2 = size.y * size.y;
	double d2 = size.z * size.z;
	Vec3 scale = Vec3(1. / (factor * (h2 + d2)),
		1. / (factor * (w2 + d2)),
		1. / (factor * (h2 + w2)));
	Mat4 ret;
	ret.initScaling(scale.x, scale.y, scale.z);
	return ret;
}

Mat4 RigidBodySystemSimulator::calcInvInertiaSphere(double radius, double mass, bool solid)
{
	double mrr = mass*radius*radius;
	double factor = solid?5.:3.;
	Mat4 ret;
	Vec3 scale = Vec3((1. / mrr) * .5 * factor);
	ret.initScaling(scale.x, scale.y, scale.z);
	return ret;
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

	TwAddVarRW(DUC->g_pTweakBar, "Gravitation", TW_TYPE_DOUBLE, &gravitation, "");
	TwAddSeparator(DUC->g_pTweakBar, "sep0", NULL);

	TwAddButton(DUC->g_pTweakBar, "Remove balls", removeBasketballsCallback, NULL, NULL);
	TwAddVarRW(DUC->g_pTweakBar, "Ball Mass", TW_TYPE_DOUBLE, &ballMass, "min=0.001 max=100");
	TwAddVarRW(DUC->g_pTweakBar, "Ball Scale", TW_TYPE_DOUBLE, &ballScale, "min=0.001 max=100");
	TwAddVarRW(DUC->g_pTweakBar, "Elasticity", TW_TYPE_DOUBLE, &elasticity, "min=0.0 max=1.0");
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

	addRigidBody(Vec3(0.9, 2, 2), Vec3(0.3, 0.5, 0.5), ballMass, "Ball", true);
	//addRigidBody(Vec3(0, -0.5, 0), Vec3(0.5, 0.2, 0.5), ballMass, "Ball1", false, true);
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
	DUC->beginLine();
	DUC->drawLine(lastPoint, Vec3(1, 0, 0), lastPoint + lastNormal, Vec3(0, 1, 0));
	DUC->endLine();
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
		Mat4 Im1 = Rot * rb.inverseInertia * RotT;
		rb.inverseInertiaT = Im1;

		rb.angularVelocity = Im1 * rb.angularMomentum;

		rb.position += timeStep * rb.linearVelocity;

		rb.force = Vec3(0., -gravitation * rb.mass, 0);
		rb.torque = Vec3(0.);
}

void RigidBodySystemSimulator::simulateTimestep(float timeStep)
{
	for (auto& rb : rigidBodies)
	{
		if(!rb.isStatic)
			integrate(rb);
	}

	for (int i = 0; i < rigidBodies.size()-1; i++)
	{
		for (int j = i + 1; j < rigidBodies.size(); j++)
		{
			auto ci = getCollision(rigidBodies[i], rigidBodies[j]);
			resolveCollision(rigidBodies[i], rigidBodies[j], ci);
		}
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

void RigidBodySystemSimulator::addRigidBody(Vec3 position, Vec3 size, double mass, std::string name, bool isSphere, bool isStatic, Quat rotation)
{
	RigidBody rb{};
	rb.position = position;
	rb.scale = size;
	rb.mass = mass;
	rb.isSphere = isSphere;
	rb.isStatic = isStatic;
	rb.rotation = rotation;
	rb.name = name;
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
	if (a.isStatic && b.isStatic)
	{
		return collision;
	}

	if (a.isSphere || b.isSphere)
	{
		if (a.isSphere && b.isSphere)
		{
			double distance = norm(b.position - a.position);
			collision.isValid = distance < (a.scale.x + b.scale.x);
			collision.depth = ((a.scale.x + b.scale.x) - distance) / 2;
			if (collision.isValid)
			{
				collision.collisionPointWorld = a.position + normalize(b.position - a.position) * (a.scale.x - collision.depth);
				collision.normalWorld = getNormalized(a.position - b.position);
			}
		}
		else {
			if (a.isSphere)
			{
				collision = checkCollisionSphereCube(a, b);
			}
			else
			{
				collision = checkCollisionSphereCube(b, a);
				collision.normalWorld *= -1;
			}
		}
	}
	else
	{
		auto rotA = a.rotation.getRotMat();
		Mat4 scaleA;
		scaleA.initScaling(a.scale.x, a.scale.y, a.scale.z);
		Mat4 translateA;
		translateA.initTranslation(a.position.x, a.position.y, a.position.z);

		auto rotB = b.rotation.getRotMat();
		Mat4 scaleB;
		scaleB.initScaling(b.scale.x, b.scale.y, b.scale.z);
		Mat4 translateB;
		translateB.initTranslation(b.position.x, b.position.y, b.position.z);
		collision = checkCollisionSAT(rotA*scaleA*translateA, rotB*scaleB*translateB);
	}
	/*if (collision.isValid)
		std::cout << "Collision between " << a.name << " and " << b.name << " at " << collision.collisionPointWorld.toString() << std::endl;*/
	if (collision.isValid && lastPoint.x == 0)
	{
		lastPoint = collision.collisionPointWorld;
		lastNormal = collision.normalWorld;
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
	Vec3 sphereMiddleRtoBox = sphere.position - box.position;
	Mat4 matBox = box.rotation.getRotMat().inverse();
	sphereMiddleRtoBox = matBox * (sphereMiddleRtoBox);
	Vec3 distVec = sphereMiddleRtoBox.getAbsolutes() - (box.scale/2);
	distVec = distVec.maximize(Vec3(0.));
	if (norm(distVec) < sphere.scale.x)
	{
		collision.depth = sphere.scale.x - norm(distVec)*.5;
		collision.isValid = true;
		distVec = box.rotation.getRotMat().transformVector(distVec * Vec3(sphereMiddleRtoBox.x < 0 ? -1 : 1, sphereMiddleRtoBox.y < 0 ? -1 : 1, sphereMiddleRtoBox.z < 0 ? -1 : 1));
		collision.collisionPointWorld = sphere.position - distVec;
		collision.normalWorld = normalize(distVec);
	}
	
	return collision;
}

void RigidBodySystemSimulator::resolveCollision(RigidBody &a,RigidBody &b, CollisionInfo &ci)
{
	if (!ci.isValid)
		return;
	Vec3 middleAToPoint = ci.collisionPointWorld - a.position;
	Vec3 middleBToPoint = ci.collisionPointWorld - b.position;

	Vec3 velA = a.linearVelocity + cross(a.angularVelocity, middleAToPoint);
	Vec3 velB = b.linearVelocity + cross(b.angularVelocity, middleBToPoint);

	Vec3 vrel = (velA - velB) * (-(1.0 + elasticity));

	double relVelonNormal = dot(vrel, ci.normalWorld);
	if (relVelonNormal < 0.0) //already seperating
		return;

	double inverseMasses = (a.isStatic ? 0 : (1.0 / a.mass)) + (b.isStatic ? 0 : (1.0 / b.mass));
	
	Mat4 invA, invB;
	if (!a.isStatic)
		invA = a.inverseInertiaT;
	if (!b.isStatic)
		invB = b.inverseInertiaT;

	Vec3 xaN = invA.transformVector(cross(middleAToPoint, ci.normalWorld));
	Vec3 xbN = invB.transformVector(cross(middleBToPoint, ci.normalWorld));

	xaN = cross(xaN, middleAToPoint);
	xbN = cross(xbN, middleBToPoint);
	double xabN2 = dot(xaN + xbN, ci.normalWorld);
	double denominator = inverseMasses + xabN2;
	double impulse = relVelonNormal / denominator;

	Vec3 impulseNormal = impulse * ci.normalWorld;
	if(!a.isStatic)
		a.position += (ci.normalWorld * ci.depth) / (b.isStatic ? 1 : 2);
	if(!b.isStatic)
		b.position -= (ci.normalWorld * ci.depth) / (a.isStatic ? 1 : 2);

	a.linearVelocity += (a.isStatic ? Vec3(0.) : (impulseNormal / a.mass));
	b.linearVelocity -= (b.isStatic ? Vec3(0.) : (impulseNormal / b.mass));

	a.angularMomentum += (a.isStatic ? Vec3(0.) : cross(middleAToPoint, impulseNormal));
	b.angularMomentum -= (b.isStatic ? Vec3(0.) : cross(middleBToPoint, impulseNormal));

	std::cout << "Collision!!" << std::endl;
}

void RigidBodySystemSimulator::addBasket(Vec3 position, double scale, int segments) {
	addRigidBody(Vec3(0, -1, 0), Vec3(200, 0.001, 200), netMass, "Floor", false, true);
	float angle = 2. * M_PI / (double)segments;
		for (int i = 0; i < ((segments >= 4) ? segments : 4); i++) {
			RigidBody rb{};
			rb.position = position
				+ Vec3(1 * scale, 0., 0.) * sin(angle*i)
				+ Vec3(0., 0., 1 * scale) * cos(angle*i);
			rb.scale = scale * Vec3(sin(angle / 2.) * 2., .1, .1);
			rb.rotation = Quat(Vec3(1, 0, 0), M_PI * .25) * Quat(Vec3(0, 1, 0), 2 * M_PI * ((double)i / (double)segments));
			addRigidBody(rb.position, Vec3(0.1, 0, 0), netMass, "segment " + std::to_string(i), true, true, rb.rotation);
		}
	RigidBody wall{};
	wall.position = position + scale * Vec3(0, 0, 1.1);
	wall.scale = scale * Vec3(3., 3., .1);
	addRigidBody(wall.position, wall.scale, netMass, "wall", false , true);
}

// UI Callback Methods
void TW_CALL RigidBodySystemSimulator::removeBasketballsCallback(void* optionalData) {
	// TODO: Implement this lateron
	std::cout << "TODO: Implement deleting of Basketballs\n";
}
