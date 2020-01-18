#ifndef RIGIDBODYSYSTEMSIMULATOR_h
#define RIGIDBODYSYSTEMSIMULATOR_h
#include "Simulator.h"
#include "collisionDetect.h"
#include <vector>
#include <string>
#define TESTCASEUSEDTORUNTEST 2


struct RigidBody
{
	Vec3 position = Vec3();
	Quat rotation = Quat(Vec3(1,0,0),0);
	Vec3 scale = Vec3();
	Vec3 linearVelocity = Vec3();
	Vec3 angularVelocity = Vec3();
	Vec3 angularMomentum = Vec3();
	Vec3 force = Vec3();
	Vec3 torque = Vec3();
	Vec3 inverseInertia = Vec3(); //Diagonal of matrix
	double mass = 1.;

	bool isSphere;
	bool isStatic;
	std::string name;
};


struct Spring
{
    int massPoint1,massPoint2;
};


class RigidBodySystemSimulator:public Simulator{
public:
	// Construtors
	RigidBodySystemSimulator();
	
	// Functions
	const char * getTestCasesStr();
	void initUI(DrawingUtilitiesClass * DUC);
	void initScene();
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void externalForcesCalculations(float timeElapsed);
	void simulateTimestep(float timeStep);
	void onClick(int x, int y);
	void onMouse(int x, int y);

	// ExtraFunctions
	void applyForceOnBody(RigidBody& rb, Vec3 loc, Vec3 force);
	void addRigidBody(Vec3 position, Vec3 size, double mass, std::string name = "", bool isSphere = false, bool isStatic = false, Quat rotation = Quat(Vec3(1, 0, 0), 0));

	CollisionInfo getCollision(RigidBody &a, RigidBody &b);
	void resolveCollision(RigidBody &a,RigidBody &b, CollisionInfo &ci);

	CollisionInfo checkCollisionSphereCube(RigidBody &sphere, RigidBody &box);

	void addBasket(Vec3 position, double scale, int segments);

	// UI Callbacks
	static void TW_CALL removeBasketballsCallback(void* optionalData);

private:
	// Attributes
	// add your RigidBodySystem data members, for e.g., 
	Vec3 m_externalForce;
	std::vector<RigidBody> rigidBodies;
	std::vector<Spring> springs;
	std::vector<Vec3> leap;

	Vec3 calcInvInertiaSphere(double radius, double mass, bool solid = true);
	Vec3 calcInvInertiaCube(Vec3 size, double mass);
	void integrate(RigidBody &rb);


	// UI Attributes
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;
	
	double netDamping = 0.1;
	double netMass = 1.0;
	double friction = 0.1;
	bool netCollision = true;
	double netBounciness = 0.0;
	double gravitation = 10.0;
	double ballMass = 10;
	double ballScale = 1;
	double timeStep = 0.01;
	double elasticity = 0.0;

	double basketScale = 1.0;
	int basketSegmnets = 10;
	};
#endif