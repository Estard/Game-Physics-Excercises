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


std::vector<MassPoint> massPoints;
std::vector<Spring> springs;
std::vector<Vec3> leap;
int methode = EULER;
float internTimestep = -1;
bool gravitation = false;
bool collision = false;
float bounciness = 0.0;

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
std::string springToString(Spring &s)
{
    std::stringstream ss;
    ss << "String initial length: "<<s.initialLength;
    ss << "\nMassPoint1:\n"<<massPointToString(massPoints[s.massPoint1]);
    ss << "\nMassPoint2:\n"<<massPointToString(massPoints[s.massPoint2]);
    return ss.str();
    

}


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

void applyForces(std::vector<Spring> &springs, std::vector<MassPoint> &massPoints, float damping)
{    
    //Spring Forces
    for(auto &s: springs){
        MassPoint &m1 = massPoints[s.massPoint1];
        MassPoint &m2 = massPoints[s.massPoint2];
        float lengthDif = sqrt(m1.position.squaredDistanceTo(m2.position)) -s.initialLength;
        float totalForce = s.stiffness*lengthDif;
        Vec3 dir = m2.position-m1.position;
        normalize(dir);
        m1.force += dir*totalForce;
        m2.force -= dir*totalForce;
    }
	for (auto& m : massPoints) {
		m.force += -damping * m.velocity;
	}
}
void EulerIntegration(std::vector<Spring> &springs,std::vector<MassPoint> &mps,float timestep)
{
    for(auto &ms: mps)
    {
		if (!ms.isFixed)
		{
			ms.position += timestep * ms.velocity;
			ms.velocity += ms.force * (timestep / ms.mass);
		}
    }
}
void MidpointIntegration(std::vector<Spring> &springs,std::vector<MassPoint> &mps,float timestep, float damping)
{
    std::vector<Spring> tmpSprings = springs;
    std::vector<MassPoint> tmpMassPoints = mps;

    EulerIntegration(tmpSprings,tmpMassPoints,timestep*.5);
    applyForces(tmpSprings,tmpMassPoints, damping);
    for(size_t i = 0; i < mps.size();i++){
		if (!mps[i].isFixed)
		{
			mps[i].position += timestep * tmpMassPoints[i].velocity;
			mps[i].velocity += timestep * tmpMassPoints[i].force / mps[i].mass;
		}
    }    
}
void LeapFrogIntegration(std::vector<Spring>& springs, std::vector<MassPoint>& mps, float timestep, float damping) 
{
	for (int i = 0; i < mps.size(); i++)
	{
		mps[i].velocity = mps[i].force * (timestep / mps[i].mass) + leap[i];
		if(!mps[i].isFixed)
			mps[i].position += timestep * mps[i].velocity;
		leap[i] = mps[i].velocity;
	}
}

 

void MassSpringSystemSimulator::simulateTimestep(float timeStep)
{
	if(!massPoints.empty() && massPoints[0].mass != m_fMass)
		setMass(m_fMass);

	if(!(internTimestep < 0) )
		timeStep = internTimestep;
	//Apply Forces
	applyForces(springs, massPoints, m_fDamping);
	
	//Integrate based on Method
	switch (methode)
	{
	case EULER:
		leap.clear();
		EulerIntegration(springs, massPoints, timeStep);
		break;
	case MIDPOINT:
		leap.clear();
		MidpointIntegration(springs, massPoints, timeStep, m_fDamping);
		break;
	case LEAPFROG:
		if (leap.size() == 0) {
			for (auto& ms : massPoints)
			{
				leap.push_back(ms.velocity);
			}
		}
		LeapFrogIntegration(springs, massPoints, timeStep, m_fDamping);
		break;
	default:
		break;
	}
	//Collision with GroundPlane
	if (collision)
	{
		for (int i = 0; i<getNumberOfMassPoints(); i++)
		{
			if (massPoints[i].position.y < -1) {
				massPoints[i].position.y = -1;
				massPoints[i].velocity.y *= -bounciness; //*= -1 für bounciness
				if(methode == LEAPFROG)
					leap[i].y *= -bounciness;
			}
		}
	}
	for (auto &s : springs) {
		std::cout << "Updates Springs: " << springToString(s) << std::endl;
	}
	std::cout << getNumberOfSprings() << std::endl;
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
			return getNumberOfMassPoints() -1;
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
				for (auto& m : massPoints)
				{
					m.force = Vec3();
					if(gravitation)
						m.force += 9.8 * m.mass * Vec3(0, -1, 0);
				}
		}


		const char* MassSpringSystemSimulator::getTestCasesStr()
		{
			return "Demo1,Demo2,Demo3,Demo4,Demo5";
		}


		void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass * DUC)
		{
			this->DUC = DUC;
			springs.clear();
			massPoints.clear();
			m_fDamping = 0.;

			setMass(10);
			setStiffness(40);
			gravitation = false;
			collision = false;
			if(m_iTestCase < 3){
				int m0 = addMassPoint(Vec3(0, 0, 0), Vec3(1, 0, 0), false);
				int m1 = addMassPoint(Vec3(0, 2, 0), Vec3(-1, 0, 0), false);
				addSpring(m0, m1, 1);
			}
			else if(m_iTestCase == 3)
			{
				addMassPoint(Vec3(-4, 2, 0), Vec3(0, -1, 0), false);
				addMassPoint(Vec3(-3, 2, 0), Vec3(1, 0, 0), false);
				addMassPoint(Vec3(-2, 2, 0), Vec3(0, 0, -1), false);
				addMassPoint(Vec3(-3, 0, 0), Vec3(-1, 0, 0), false);
				addMassPoint(Vec3(-1, 2, 0), Vec3(0, 0, 1), false);
				addMassPoint(Vec3(-1, 0, 0), Vec3(1, 0, 0), false);
				addMassPoint(Vec3(1, 0, 0), Vec3(), false);
				addMassPoint(Vec3(1, 2, 0), Vec3(1, 0, 0), false);
				addMassPoint(Vec3(2, 0, 0), Vec3(0, 3, 0), false);
				addMassPoint(Vec3(2, 2, 0), Vec3(), true);
				addMassPoint(Vec3(3, 0, 0), Vec3(), false);
				addMassPoint(Vec3(4, 2, 0), Vec3(), false);
				addMassPoint(Vec3(4, 0, 0), Vec3(), false);

				addSpring(0, 1, 0.5);
				addSpring(1, 2, 0.5);
				addSpring(1, 3, 1);
				addSpring(4, 5, 1);
				addSpring(5, 6, 1);
				addSpring(6, 7, 1);
				addSpring(8, 9, 1);
				addSpring(9, 10, 1);
				addSpring(10, 11, 1);
				addSpring(11, 12, 1);
			}
			else
			{
				addMassPoint(Vec3(), Vec3(), false);
				addMassPoint(Vec3(0, 2, 0), Vec3(), true);
				addMassPoint(Vec3(1, 0, 0), Vec3(-1, 0, 0), false);
				addSpring(0, 1, 1);
				addSpring(0, 2, 2);
			}
			switch (m_iTestCase)
			{
				case 0:
					methode = EULER;
					internTimestep = .1;
				break;
				case 1:
					methode = EULER;
					internTimestep = .005;
				break;
				case 2:
					methode = MIDPOINT;
					internTimestep = .005;
				break;
				default:
					internTimestep = -1;
					TwAddVarRW(DUC->g_pTweakBar, "Simulation Methode", TW_TYPE_INT32, &methode, "min=0 max=2");
					TwAddVarRW(DUC->g_pTweakBar, "Damping", TW_TYPE_FLOAT, &m_fDamping, "min=0.0 max=1.0");
					TwAddVarRW(DUC->g_pTweakBar, "Collision", TW_TYPE_BOOLCPP, &collision, "");
					TwAddVarRW(DUC->g_pTweakBar, "Bounciness", TW_TYPE_FLOAT, &bounciness, "min=0.0 max=1.0");
					methode = EULER;
					TwAddVarRW(DUC->g_pTweakBar, "Mass", TW_TYPE_FLOAT, &m_fMass, "min=0.001");
					TwAddVarRW(DUC->g_pTweakBar, "Gravitation", TW_TYPE_BOOLCPP, &gravitation, "");
				break;
			}			
		}

		void MassSpringSystemSimulator::reset()
		{
			m_mouse.x = m_mouse.y = 0;
			m_trackmouse.x = m_trackmouse.y = 0;
			m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
		}

		void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
		{
			DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, 0.6 * Vec3(0.97, 0.86, 1));
			for (auto& mp : massPoints)
			{
				DUC->drawSphere(mp.position, Vec3(.05, .05, .05));
			}
			for (auto& s : springs)
			{
				DUC->beginLine();
				DUC->drawLine(massPoints[s.massPoint1].position, Vec3(0., 1., 1.),
					massPoints[s.massPoint2].position, Vec3(0., 1., 1.));
				DUC->endLine();
			}
		}
	