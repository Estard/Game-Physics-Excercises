#include "DiffusionSimulator.h"
#include "pcgsolver.h"
using namespace std;

Grid::Grid(uint32_t n, uint32_t m): _n(n),_m(m),buffer(false),
					values0(std::vector<Real>(_n*_m,0)),
					values1(std::vector<Real>(_n*_m,0))
{
}

void Grid::resize(uint32_t n, uint32_t m)
{
	this->_n = n;
	this->_m = m;
	values0.resize(n*m);
	values1.resize(n*m);
}

uint32_t Grid::n(){return _n;}
uint32_t Grid::m(){return _m;}

std::vector<Real>& Grid::currentValues()
{return buffer?values1:values0;}

std::vector<Real>& Grid::nextValues()
{return buffer?values0:values1;}

void Grid::applyUpdates()
{buffer = !buffer;}



#include <chrono>
struct Timer
{

  Timer() :
      startTime(std::chrono::high_resolution_clock::now())
  {}

  void reset()
  {
    startTime = std::chrono::high_resolution_clock::now();
  }

  double deltaTime()
  {
    auto endTime = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration(endTime - startTime);  // @suppress("Invalid arguments")
    return duration.count();
  }

  double deltaTimeMilli()
  {
    auto endTime = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
        endTime - startTime);
    return duration.count();
  }

 private:
  std::chrono::high_resolution_clock::time_point startTime;

};




DiffusionSimulator::DiffusionSimulator():T(new Grid()),alpha(1.)
{
	m_iTestCase = 0;
	m_vfMovableObjectPos = Vec3();
	m_vfMovableObjectFinalPos = Vec3();
	m_vfRotate = Vec3();
	// to be implemented
}

DiffusionSimulator::~DiffusionSimulator()
{
	delete(T);
}


const char * DiffusionSimulator::getTestCasesStr(){
	return "Explicit_solver, Implicit_solver";
}

void DiffusionSimulator::reset(){
		m_mouse.x = m_mouse.y = 0;
		m_trackmouse.x = m_trackmouse.y = 0;
		m_oldtrackmouse.x = m_oldtrackmouse.y = 0;

}

void fillT(Grid* T) {//add your own parameters
	// to be implemented
	//fill T with solved vector x
	//make sure that the temperature in boundary cells stays zero

	uint32_t n = T->n();
	uint32_t m = T->m();
	auto& values0 = T->currentValues();
	auto& values1 = T->nextValues();
	for (uint32_t y = 0; y < m; y++) {
		for (uint32_t x = 0; x < n; x++) {
			float t = 2.*cos((x*(y - m / 2)) / ((n + m)*.5));
			bool isEdge = x == 0 || x + 1 == n || y == 0 || y + 1 == m;
			float v = isEdge ? 0. : t;
			values0[x + y * n] = v;
			values1[x + y * n] = v;
		}
	}
}

void DiffusionSimulator::initUI(DrawingUtilitiesClass * DUC)
{
	this->DUC = DUC;
	fillT(T);
	// to be implemented
}

void DiffusionSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	m_vfMovableObjectPos = Vec3(0, 0, 0);
	m_vfRotate = Vec3(0, 0, 0);
	//
	//to be implemented
	//
	switch (m_iTestCase)
	{
	case 0:
		cout << "Explicit solver!\n";
		break;
	case 1:
		cout << "Implicit solver!\n";
		break;
	default:
		cout << "Empty Test!\n";
		break;
	}
}

Grid* DiffusionSimulator::diffuseTemperatureExplicit(Real timestep) {//add your own parameters
	// to be implemented
	//make sure that the temperature in boundary cells stays zero
	
	auto &now = T->currentValues();
    	auto &next = T->nextValues();
	uint32_t n = T->n();
	uint32_t m = T->m();

    	for (uint32_t y = 1; y < m - 1; y++) {
     		for (uint32_t x = 1; x < n - 1; x++) {
        		Real t = now[y * n + x];
        		Real txl = now[y * n + x - 1];
        		Real txr = now[y * n + x + 1];
        		Real tyl = now[(y - 1) * n + x];
        		Real tyr = now[(y + 1) * n + x];
        		Real dx = (txl - 2. * t + txr);
        		Real dy = (tyl - 2. * t + tyr);
        		Real dt = alpha * (dx + dy);
        		next[y * n + x] = t + timestep * dt;
     		}
    	}

	T->applyUpdates();
	return T;
}

void setupB(std::vector<Real>& b) {//add your own parameters
	// to be implemented
	//set vector B[sizeX*sizeY]
	for (int i = 0; i < 25; i++) {
		b.at(i) = 0;
	}
}


void setupA(SparseMatrix<Real>& A, double factor, Grid* T) {//add your own parameters
	// to be implemented
	//setup Matrix A[sizeX*sizeY*sizeZ, sizeX*sizeY*sizeZ]
	// set with:  A.set_element( index1, index2 , value );
	// if needed, read with: A(index1, index2);
	// avoid zero rows in A -> set the diagonal value for boundary cells to 1.0

	int d = static_cast<int>(T->n()*T->m());
	for (int i = 0; i < d; i++) {
			A.set_element(i, i, 1); // set diagonal
	}
	
	uint32_t n = T->n(); 
	uint32_t m = T->m();
	uint32_t x = 0, y = 0;
	for (uint32_t row = 0; row < static_cast<uint32_t>(A.n); row++) {
		if (!(x == 0 || y == 0 || x == n-1 || y == m -1)) {
			A.set_element(row, x + (y - 1)*n, -factor);
			A.set_element(row, x - 1 + y * n, -factor);
			A.set_element(row, x + y * n, 1. + 4.*factor);
			A.set_element(row, x + 1 + y * n, -factor);
			A.set_element(row, x + (y + 1)*n, -factor);
		}
		x++;
		if(x==n){
			y++;
			x = 0;
		}

	}
}


void DiffusionSimulator::diffuseTemperatureImplicit(Real timestep) {//add your own parameters
	// solve A T = b
	// to be implemented
	uint32_t N = T->n()*T->m();//N = sizeX*sizeY*sizeZ
	SparseMatrix<Real> A(N);
	std::vector<Real>& b = T->currentValues();

	setupA(A, alpha*timestep,T);
	//setupB(b);

	// perform solve
	Real pcg_target_residual = 1e-05;
	Real pcg_max_iterations = 1000;
	Real ret_pcg_residual = 1e10;
	int  ret_pcg_iterations = -1;

	SparsePCGSolver<Real> solver;
	solver.set_solver_parameters(pcg_target_residual, pcg_max_iterations, 0.97, 0.25);

	std::vector<Real>& x = T->nextValues();
	for (int j = 0; j < N; ++j) { x[j] = 0.; }
	Timer t;
	// preconditioners: 0 off, 1 diagonal, 2 incomplete cholesky
	solver.solve(A, b, x, ret_pcg_residual, ret_pcg_iterations, 0);
	// x contains the new temperature values
	//std::cout << "Solver took: "<<t.deltaTime()<<'\n';
	uint32_t n = T->n();
	uint32_t m = T->m();
	for (uint32_t y = 0; y < m; y++) {
		for (uint32_t i = 0; i < n; i++) {
			if (i == 0 || y == 0 || i == n - 1 || y == m - 1) {
				x[i + y * n] = 0;
			}
		}
	}
	//fillT();//copy x to T
	T->applyUpdates();
}



void DiffusionSimulator::simulateTimestep(float timeStep)
{
	// to be implemented
	// update current setup for each frame
	switch (m_iTestCase)
	{
	case 0:
		T = diffuseTemperatureExplicit(timeStep);
		break;
	case 1:
		diffuseTemperatureImplicit(timeStep);
		break;
	}
}

void DiffusionSimulator::drawObjects()
{
	// to be implemented
	//visualization
	Vec3 color = Vec3();
	auto &now = T->currentValues();
	for(uint32_t y = 0; y < T->m();y++) 
		for(uint32_t x = 0; x < T->n();x++) {
			float t = now[x+y*T->n()];
      			DUC->setUpLighting(Vec3(), Vec3 (t,0,-t), 50., Vec3(t,0,-t));
			DUC->drawSphere(Vec3(x,y,0), Vec3(1,1,1));	
    		}

}


void DiffusionSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	drawObjects();
}

void DiffusionSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void DiffusionSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}
