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
	Vec3 leapVelocity = Vec3();
	Vec3 angularVelocity = Vec3();
	Vec3 angularMomentum = Vec3();
	Vec3 force = Vec3();
	Vec3 torque = Vec3();
	Mat4 inverseInertia = Mat4(); //Diagonal of matrix
	Mat4 inverseInertiaT = Mat4();
	double mass = 1.;

	bool isSphere;
	bool isStatic;
	std::string name;
};


struct Spring
{
	int massPoint1, massPoint2;
	double initialLength;
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
	void onClick(int x, int y, int duration);
	void onMouse(int x, int y);

	void applyForces();

	// ExtraFunctions
	void applyForceOnBody(RigidBody& rb, Vec3 loc, Vec3 force);
	void addRigidBody(Vec3 position, Vec3 size, double mass, std::string name = "", bool isSphere = false, bool isStatic = false, Quat rotation = Quat(Vec3(1, 0, 0), 0));

	CollisionInfo getCollision(RigidBody &a, RigidBody &b);
	void resolveCollision(RigidBody &a,RigidBody &b, CollisionInfo &ci);

	CollisionInfo checkCollisionSphereCube(RigidBody &sphere, RigidBody &box);

	void addBasket(Vec3 position, double scale, int segments);
	void addNet(Vec3 position, double scale, int segments);
	void addSpring(int mp, int mp2);

	// UI Callbacks
	static void TW_CALL removeBasketballsCallback(void* optionalData);
	void removeBasketballs();

private:
	// Attributes
	// add your RigidBodySystem data members, for e.g., 
	Vec3 m_externalForce;
	std::vector<RigidBody> rigidBodies;
	std::vector<Spring> springs;
	std::vector<int> marked;

	Mat4 calcInvInertiaSphere(double radius, double mass, bool solid = true);
	Mat4 calcInvInertiaCube(Vec3 size, double mass);
	void integrate(RigidBody &rb);


	// UI Attributes
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;
	
	double netMass = 1.0;
	int netSegments = 5;
	double gravitation = 1.0;
	double ballMass = 10;
	double ballScale = 1;
	double timeStep = 0.01;
	double elasticity = 0.9;
	double stiffness = 5.;
	double damping = 0.1;

	double basketScale = 1.0;
	int basketSegmnets = 10;

	double throwForceMin = 1000;
	double throwForceMax = 6000;
	double throwForceWindUp = 5.0;

	int anzahlBall = 0;
	int score = 0;
	};
#endif
