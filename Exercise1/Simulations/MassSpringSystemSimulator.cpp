#include "MassSpringSystemSimulator.h"

#include<iostream>
#include<vector>
#include<string>
#include<sstream>

struct MassPoint
{
    Vec3 position,velocity;
    bool isFixed;
    Vec3 force;
    float mass;
};
struct Spring
{
    int massPoint1,massPoint2;
    float initialLength;
    float stiffness;
};

std::string vec3ToString(Vec3 &v)
{
    std::stringstream ss;
    ss << v.x << ' ' << v.y <<' ' << v.z;
    return ss.str();
}

std::string massPointToString(MassPoint &ms)
{
 std::stringstream ss;
 ss << "Mass Point, isFixed : " << ms.isFixed;
 ss << "\nPosition: "<<vec3ToString(ms.position);
 ss << "\nVelocity: "<<vec3ToString(ms.velocity);
 return ss.str();
}

std::vector<MassPoint> massPoints;

std::string springToString(Spring &s)
{
    std::stringstream ss;
    ss << "String initial length: "<<s.initialLength;
    ss << "\nMassPoint1:\n"<<massPointToString(massPoints[s.massPoint1]);
    ss << "\nMassPoint2:\n"<<massPointToString(massPoints[s.massPoint2]);
    return ss.str();
    

}

std::vector<Spring> springs;

int methode = EULER;
MassSpringSystemSimulator::MassSpringSystemSimulator()
{
     m_fMass = 1.;
	 m_fStiffness = 1.;
	 m_fDamping = 0.;
	 m_mouse = Point2D();
	 m_trackmouse = Point2D();
	 m_oldtrackmouse = Point2D();
	 m_iIntegrator = 0;
}


/*DEPRECATED
void EulerIntegration(MassPoint &ms,float timestep)
{
    ms.position += timestep*ms.velocity;
    ms.velocity += ms.force*(timestep/ms.mass);
}

void MidPointIntegration(MassPoint &ms,float timestep)
{
    ms.position += timestep*ms.velocity;
    ms.velocity += ms.force*(timestep/ms.mass);
}
*/


void applyForces(std::vector<Spring> &springs, std::vector<MassPoint> &massPoints)
{
    //Clear Forces
     for(auto &mp : massPoints)
            mp.force = Vec3();
    
    //Spring Forces
    for(auto &s: springs){
        MassPoint &m1 = massPoints[s.massPoint1];
        MassPoint &m2 = massPoints[s.massPoint2];
        float lengthDif = sqrt(m1.position.squaredDistanceTo(m2.position)) -s.initialLength;
        float totalForce = s.stiffness*lengthDif;
        Vec3 dir = m2.position-m1.position;
        normalize(dir);
        if(!m1.isFixed)
            m1.force += dir*totalForce;
        if(!m2.isFixed)
            m2.force -= dir*totalForce;
    }
}

void EulerIntegration(std::vector<Spring> &springs,std::vector<MassPoint> &mps,float timestep)
{
    for(auto &ms: mps)
    {
        ms.position += timestep*ms.velocity;
        ms.velocity += ms.force*(timestep/ms.mass);
    }
}

void MidpointIntegration(std::vector<Spring> &springs,std::vector<MassPoint> &mps,float timestep)
{
    std::vector<Spring> tmpSprings = springs;
    std::vector<MassPoint> tmpMassPoints = mps;

    EulerIntegration(tmpSprings,tmpMassPoints,timestep*.5);
    applyForces(tmpSprings,tmpMassPoints);
    for(size_t i = 0; i < mps.size();i++){
        mps[i].position += timestep*tmpMassPoints[i].velocity;
        mps[i].velocity += timestep*tmpMassPoints[i].force/mps[i].mass;
    }
    
}

 

void MassSpringSystemSimulator::simulateTimestep(float timeStep)
{
	for (auto &s : springs) {
		//Apply Forces
		//TODO: m_fDamping
		applyForces(springs, massPoints);

		//Integrate based on Method
		switch (methode)
		{
		case EULER:
			EulerIntegration(springs, massPoints, timeStep);
			break;
		case MIDPOINT:
			MidpointIntegration(springs, massPoints, timeStep);
			break;
		default:
			break;
		}
#if 0
		//Collision with GroundPlane
		if (ms.position < 0) {
			ms.position = 0;
			ms.velocity.z = 0; //*= -1 für bounciness
		}
#endif
		for (auto &s : springs) {
			std::cout << "Updates Springs: " << springToString(s) << std::endl;
		}
		std::cout << getNumberOfSprings() << std::endl;
	}
}
		/*Boring Getters and Setters*/
		void MassSpringSystemSimulator::setMass(float mass)
		{
			m_fMass = mass;
			for (auto &ms : massPoints)
				ms.mass = m_fMass;

		}
		void MassSpringSystemSimulator::setStiffness(float stiffness)
		{
			m_fStiffness = stiffness;
			for (auto &s : springs)
				s.stiffness = m_fStiffness;
		}
		void MassSpringSystemSimulator::setDampingFactor(float damping)
		{
			m_fDamping = damping;
		}
		int MassSpringSystemSimulator::addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed)
		{
			MassPoint mp = { position,Velocity,isFixed,Vec3(),m_fMass };
			massPoints.push_back(mp);
			return massPoints.size() -1;
		}
		void MassSpringSystemSimulator::addSpring(int masspoint1, int masspoint2, float initialLength)
		{
			Spring s = { masspoint1,masspoint2,initialLength, m_fStiffness };
			springs.push_back(s);
		}
		int MassSpringSystemSimulator::getNumberOfMassPoints()
		{
			return static_cast<int>(massPoints.size());
		}
		int MassSpringSystemSimulator::getNumberOfSprings()
		{
			return static_cast<int>(springs.size());
		}
		Vec3 MassSpringSystemSimulator::getPositionOfMassPoint(int index)
		{
			if (index <= massPoints.size())
				return massPoints[index].position;
			else
				return Vec3();
		}
		Vec3 MassSpringSystemSimulator::getVelocityOfMassPoint(int index)
		{
			if (index <= massPoints.size())
				return massPoints[index].velocity;
			else
				return Vec3();
		}
		void MassSpringSystemSimulator::applyExternalForce(Vec3 force)
		{
			m_externalForce = force;
		}

		void MassSpringSystemSimulator::onClick(int x, int y)
		{
			m_trackmouse.x = x;
			m_trackmouse.y = y;
		}

		void MassSpringSystemSimulator::onMouse(int x, int y)
		{
			m_oldtrackmouse.x = x;
			m_oldtrackmouse.y = y;
			m_trackmouse.x = x;
			m_trackmouse.y = y;
		}

		void MassSpringSystemSimulator::notifyCaseChanged(int testCase)
		{
			m_iTestCase = testCase;
		}

		void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed)
		{
		}


		const char* MassSpringSystemSimulator::getTestCasesStr()
		{
			return "Demo1,Demo2,Demo3,Demo4";
		}

		void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass * DUC)
		{
			this->DUC = DUC;
			switch (m_iTestCase)

			TwAddVarRW(DUC->g_pTweakBar, "Simulation Methode", TW_TYPE_INT32, &methode, "min=0 max=2");
			setMass(10);
			setStiffness(40);
			int m0 = addMassPoint(Vec3(0, 0, 0), Vec3(-1, 0, 0), false);
			int m1 = addMassPoint(Vec3(0, 2, 0), Vec3(1, 0, 0), false);
			addSpring(m0, m1, 1);
		}

		void MassSpringSystemSimulator::reset()
		{
			m_mouse.x = m_mouse.y = 0;
			m_trackmouse.x = m_trackmouse.y = 0;
			m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
		}

		void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
		{
			switch (m_iTestCase)
			{
			case 0: drawDemo1(); break;
			}
		}

		void MassSpringSystemSimulator::drawDemo1() {

				DUC->setUpLighting(Vec3(),0.4*Vec3(1,1,1),100,0.6*Vec3(0.97,0.86,1));
			for(auto &mp: masspoints)
			{
				DUC->drawSphere(mp.position,Vec3(mp.mass,mp.mass,mp.mass));
			}
		
			
			
		}
	