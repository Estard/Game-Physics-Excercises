#include "RigidBodySystemSimulator.h"

Vec3 lastPoint = Vec3();
Vec3 lastNormal = Vec3();

Mat4 quatToRot(Quat const& q)
{
	Mat4 Result;
	double qxx=(q.x * q.x);
	double qyy=(q.y * q.y);
	double qzz=(q.z * q.z);
	double qxz=(q.x * q.z);
	double qxy=(q.x * q.y);
	double qyz=(q.y * q.z);
	double qwx=(q.w * q.x);
	double qwy=(q.w * q.y);
	double qwz=(q.w * q.z);
	Result.value[0][0] = (1) - (2) * (qyy +  qzz);
	Result.value[0][1] = (2) * (qxy + qwz);
	Result.value[0][2] = (2) * (qxz - qwy);
	Result.value[1][0] = (2) * (qxy - qwz);
	Result.value[1][1] = (1) - (2) * (qxx +  qzz);
	Result.value[1][2] = (2) * (qyz + qwx);
	Result.value[2][0] = (2) * (qxz + qwy);
	Result.value[2][1] = (2) * (qyz - qwx);
	Result.value[2][2] = (1) - (2) * (qxx +  qyy);
	return Result;
}


Quat inverse(Quat &q)
{
	//conjugated Quaternion
	Quat conj(-q.x,-q.y,-q.z,q.w);
	//dot product for renormalization
	double dot = q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
	conj = conj * (1/dot);
	return conj;
}

Vec3 rotate(Quat &q, Vec3 &v)
{
		Vec3 QuatVector(q.x, q.y, q.z);
		Vec3 uv = cross(QuatVector, v);
		Vec3 uuv= cross(QuatVector, uv);
		return v + ((uv * q.w) + uuv) * 2.;
}

Vec3 sign(Vec3 &v)
{
	Vec3 sign(int(v.x>0.)-int(v.x<0.),int(v.y>0.)-int(v.y<0.),int(v.y>0.)-int(v.y<0.));
	return sign;
}

bool isBall(RigidBody& rb)
{
	return (rb.name.compare("Ball") == 0);
}


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

	TwAddButton(DUC->g_pTweakBar, "Remove balls", removeBasketballsCallback, this, NULL);
	TwAddVarRW(DUC->g_pTweakBar, "Ball Mass", TW_TYPE_DOUBLE, &ballMass, "min=0.001 max=100");
	TwAddVarRW(DUC->g_pTweakBar, "Ball Scale", TW_TYPE_DOUBLE, &ballScale, "min=0.001 max=100");
	TwAddVarRW(DUC->g_pTweakBar, "Elasticity", TW_TYPE_DOUBLE, &elasticity, "min=0.0 max=1.0");
	TwAddSeparator(DUC->g_pTweakBar, "sep1", NULL);

	TwAddVarRW(DUC->g_pTweakBar, "Net Mass", TW_TYPE_DOUBLE, &netMass, "min=0.0001");
	TwAddVarRW(DUC->g_pTweakBar, "Net Stiffness", TW_TYPE_DOUBLE, &stiffness, "min=3");
	TwAddSeparator(DUC->g_pTweakBar, "sep2", NULL);

	TwAddVarRW(DUC->g_pTweakBar, "Basket Segments", TW_TYPE_INT32, &basketSegmnets, "min=1");
	TwAddVarRW(DUC->g_pTweakBar, "Basket Scale", TW_TYPE_DOUBLE, &basketScale, "min=0.001");
	TwAddSeparator(DUC->g_pTweakBar, "sep3", NULL);

	TwAddVarRW(DUC->g_pTweakBar, "Throw Velocity Min", TW_TYPE_DOUBLE, &throwVelocityMin, "");
	TwAddVarRW(DUC->g_pTweakBar, "Throw Velocity Max", TW_TYPE_DOUBLE, &throwVelocityMax, "");
	TwAddVarRW(DUC->g_pTweakBar, "Throw Velocity Windup", TW_TYPE_DOUBLE, &throwVelocityWindUp, "");
	TwAddSeparator(DUC->g_pTweakBar, "sep4", NULL);
	initScene();
	}

void RigidBodySystemSimulator::initScene()
{
	addBasket(Vec3(0, -1, 1), basketScale, basketSegmnets);
	addNet(Vec3(0, -1, 1), basketScale, 5);

	addRigidBody(Vec3(0, -5, 0), Vec3(200, 0.1, 200), netMass, "Floor", false, true);
	//addRigidBody(Vec3(0.9, 3, 1.8), Vec3(0.3, 0.5, 0.5), ballMass, "Ball", true);
	//addRigidBody(Vec3(0, -0.5, 0), Vec3(0.5, 0.2, 0.5), ballMass, "Ball1", false, true);
}

void RigidBodySystemSimulator::reset()
{
	rigidBodies.clear();
	lastPoint = Vec3();
	lastNormal = Vec3();
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
	for (auto& s : springs)
	{
		DUC->beginLine();
		DUC->drawLine(rigidBodies[s.massPoint1].position, Vec3(1, 0, 0), rigidBodies[s.massPoint2].position, Vec3(1, 0, 0));
		DUC->endLine();
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
		rb.angularMomentum += timeStep * rb.torque -(1. - elasticity) * norm(rb.angularVelocity);
		
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
	applyForces();
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

void RigidBodySystemSimulator::onClick(int x, int y, int duration) {
	// Logic for throwing balls
	if (x > 217 || y > 338) // shitty filtering of clicks on the tweakbar
	{
		DirectX::XMVECTOR camPos = DUC->g_camera.GetEyePt();
		DirectX::XMVECTOR viewPos = DUC->g_camera.GetLookAtPt();
		DirectX::XMVECTOR viewDir = viewPos - camPos;
		viewDir = DirectX::XMVector3Normalize(viewDir);

		DirectX::XMVECTOR spawnPos = camPos + 2.0 * viewDir;
		
		// XMVector elements cant be accessed, copy to float3
		XMFLOAT3 f_spawnPos;    
		XMStoreFloat3(&f_spawnPos, spawnPos);



		addRigidBody(Vec3(f_spawnPos.x, f_spawnPos.y, f_spawnPos.z), Vec3(0.5, 0.2, 0.5), ballMass, "Ball", true, false);
		std::cout << "Spawned Ball at: " << f_spawnPos.x << "|" << f_spawnPos.y << "|" << f_spawnPos.z;
		
		// Fish for reference to our rb since addRigidBody doesnt return a reference to the rb created
		RigidBody& rb = rigidBodies[rigidBodies.size() - 1];

		double velocityMul = throwVelocityMin + static_cast<double>(duration) / static_cast<double>(1000.0f * 1.0f / throwVelocityWindUp);
		velocityMul = min(throwVelocityMax, velocityMul);
		std::cout << " with velocity " << velocityMul << "\n";
		rb.linearVelocity = viewDir * velocityMul;

	}
}

void RigidBodySystemSimulator::onMouse(int x, int y) {
}

// ExtraFunctions
void RigidBodySystemSimulator::applyForces()
{
	for (auto& sp : springs)
	{
		RigidBody& m1 = rigidBodies[sp.massPoint1];
		RigidBody& m2 = rigidBodies[sp.massPoint2];
		double lengthDiff = norm(m1.position - m2.position) - sp.initialLength;
		double totalForce = stiffness * lengthDiff;
		Vec3 dir = getNormalized(m1.position - m2.position);
		applyForceOnBody(m1, m1.position, -dir * totalForce);
		applyForceOnBody(m2, m2.position, dir * totalForce);
	}
}

void RigidBodySystemSimulator::applyForceOnBody(RigidBody &rb, Vec3 loc, Vec3 force)
{
	rb.force += force - (1. -elasticity) * rb.linearVelocity;
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
			collision.isValid = distance <= (a.scale.x + b.scale.x);
			collision.depth = ((a.scale.x + b.scale.x) - distance) / 2;
			if (collision.isValid)
			{
				collision.collisionPointWorld = a.position + getNormalized(b.position - a.position) * (a.scale.x - collision.depth);
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
		collision.normalWorld *= -1;
	}
	if (collision.isValid)
		std::cout << "Collision between " << a.name << " and " << b.name << " with normal " << collision.normalWorld << std::endl;
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
	//sphereMiddleRtoBox = matBox * (sphereMiddleRtoBox);
	sphereMiddleRtoBox = rotate(inverse(box.rotation),sphereMiddleRtoBox);
	Vec3 distVec = sphereMiddleRtoBox.getAbsolutes() - (box.scale/2);
	distVec = distVec.maximize(Vec3(0.));
	if (norm(distVec) <= sphere.scale.x)
	{
		collision.depth = sphere.scale.x - norm(distVec)*.5;
		collision.isValid = true;
	//	distVec = quatToRot(box.rotation).transformVector(distVec * Vec3(sphereMiddleRtoBox.x < 0 ? -1 : 1, sphereMiddleRtoBox.y < 0 ? -1 : 1, sphereMiddleRtoBox.z < 0 ? -1 : 1));
		distVec = rotate(box.rotation,distVec*sign(sphereMiddleRtoBox));
		collision.collisionPointWorld = sphere.position - distVec;
		collision.normalWorld = getNormalized(distVec);
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
}

void RigidBodySystemSimulator::addBasket(Vec3 position, double scale, int segments) {
	float angle = 2. * M_PI / (double)segments;
		for (int i = 0; i < ((segments >= 4) ? segments : 4); i++) {
			RigidBody rb{};
			rb.position = position
				+ Vec3(1 * scale, 0., 0.) * sin(angle*i)
				+ Vec3(0., 0., 1 * scale) * cos(angle*i);
			rb.scale = scale * Vec3(sin(angle / 2.) * 2., .1, .1);
			rb.rotation = Quat(Vec3(1, 0, 0), M_PI * .25) * Quat(Vec3(0, 1, 0), 2 * M_PI * ((double)i / (double)segments));
			addRigidBody(rb.position, rb.scale, netMass, "segment " + std::to_string(i), false, true, rb.rotation);
		}
	RigidBody wall{};
	wall.position = position + scale * Vec3(0, 0, 1.1);
	wall.scale = scale * Vec3(3., 3., .1);
	addRigidBody(wall.position, wall.scale, netMass, "wall", false , true);
}

void RigidBodySystemSimulator::addNet(Vec3 position, double scale, int segments) {
	int offset = rigidBodies.size();
	double angle = 2. * M_PI / (double)segments;
	for (int j = 0; j < segments; j++) {
		for (int i = 0; i < ((segments >= 4) ? segments : 4); i++) {
			RigidBody rb{};
			rb.position = position + Vec3(0., -j * scale / (double)segments, 0.)
				+ Vec3(1 * scale, 0., 0.) * sin(angle * i + (((j % 2) != 0) ? (angle / 2) : 0))
				+ Vec3(0., 0., 1 * scale) * cos(angle * i + (((j % 2) != 0) ? (angle / 2) : 0));
			rb.scale = scale * Vec3(.1, .1, .1);
			addRigidBody(rb.position, Vec3(0.03, 0, 0), netMass, "net " + std::to_string(i), true, j == 0);
			if (j != 0) {//all except top row add \  /
				addSpring(offset + i + (j - 1) * segments, offset + i + j * segments);
				addSpring(offset + (((j % 2) == 0) ? (i - 1 + segments) % segments : (i + 1) % segments) + (j - 1) * segments, offset + i + j * segments);
			}
			if (i != 0) {//all except 1st of row add:  <-
				addSpring(offset + i + (j * segments) - 1, offset + i + j * segments);
			}
			if (i == segments - 1) {//for the last in row, add ->
				addSpring(offset + (j * segments), offset + i + j * segments);
			}
		}
	}
}

void RigidBodySystemSimulator::addSpring(int mp, int mp2)
{
	Spring s = Spring{};
	s.massPoint1 = mp;
	s.massPoint2 = mp2;
	s.initialLength = norm(rigidBodies[mp].position - rigidBodies[mp2].position);
	springs.push_back(s);
}
// UI Callback Methods
void TW_CALL RigidBodySystemSimulator::removeBasketballsCallback(void* optionalData) {

	RigidBodySystemSimulator* rbss = reinterpret_cast<RigidBodySystemSimulator*>(optionalData);
	rbss->removeBasketballs();
}

void RigidBodySystemSimulator::removeBasketballs()
{
	auto it = std::remove_if(rigidBodies.begin(), rigidBodies.end(), isBall);
	rigidBodies.erase(it, rigidBodies.end());
}



