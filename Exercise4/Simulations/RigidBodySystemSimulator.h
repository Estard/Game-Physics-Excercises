#ifndef RIGIDBODYSYSTEMSIMULATOR_h
#define RIGIDBODYSYSTEMSIMULATOR_h
#include "Simulator.h"
#include "collisionDetect.h"
#include <vector>
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
	bool isScene;
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
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void externalForcesCalculations(float timeElapsed);
	void simulateTimestep(float timeStep);
	void onClick(int x, int y);
	void onMouse(int x, int y);

	// ExtraFunctions
	void applyForceOnBody(RigidBody& rb, Vec3 loc, Vec3 force);
	void addRigidBody(Vec3 position, Vec3 size, int mass, bool isSphere);

	CollisionInfo getCollision(RigidBody &a, RigidBody &b);

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

	double knotMass;
	double knotRestingDistance;
	double stiffness;
	double bounciness;
	double damping;
	double ballMass;
	double timeStep;

	Vec3 calcInvInertiaSphere(float radius, float mass, bool solid = true);
	Vec3 calcInvInertiaCube(Vec3 size, float mass);
	void integrate(RigidBody &rb);

	CollisionInfo sphereSphereCollision(RigidBody &rb0, RigidBody &rb1);
	CollisionInfo sphereCubeCollision(RigidBody &sphere, RigidBody &cube);


	// UI Attributes
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;

	// Net Simulation Attributes
	float netMass;
	float netDamping;
	bool netCollision;
	float netBounciness;
	float gravitation;

	float basketScale;
	int basketSegmnets;
	};
#endif