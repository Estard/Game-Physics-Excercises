#ifndef DIFFUSIONSIMULATOR_h
#define DIFFUSIONSIMULATOR_h

#include "Simulator.h"
#include "vectorbase.h"
#include <cstdint>

//impement your own grid class for saving grid data
class Grid {	//Could have been a private struct
public:
	// Construtors
	Grid(uint32_t n = 16, uint32_t m = 16);

	std::vector<Real>& currentValues();
	std::vector<Real>& nextValues();
	void applyUpdates();
	void resize(uint32_t n, uint32_t m);
	uint32_t n();
	uint32_t m();


private:
	// Attributes
	uint32_t _n,_m;
	bool buffer;
	std::vector<Real> values0;
	std::vector<Real> values1; //Double buffer to avoid many heap allocations
};



class DiffusionSimulator:public Simulator{
public:
	// Construtor
	DiffusionSimulator();
	~DiffusionSimulator();

	// Functions
	const char * getTestCasesStr();
	void initUI(DrawingUtilitiesClass * DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void simulateTimestep(float timeStep);
	void externalForcesCalculations(float timeElapsed) {};
	void onClick(int x, int y);
	void onMouse(int x, int y);
	// Specific Functions
	void drawObjects();
	Grid* diffuseTemperatureExplicit();
	void diffuseTemperatureImplicit();

private:
	// Attributes
	Grid *T; //save results of every time step
	double alpha;
	Vec3  m_vfMovableObjectPos;
	Vec3  m_vfMovableObjectFinalPos;
	Vec3  m_vfRotate;
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;
	
};

#endif
