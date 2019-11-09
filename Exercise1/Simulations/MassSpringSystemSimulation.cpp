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

std::string springToString(Spring &s)
{
    std::stringstream ss;
    ss << "String initial length: "<<s.initialLength;
    ss << "\nMassPoint1:\n"<<massPointToString(s.massPoint1);
    ss << "\nMassPoint2:\n"<<massPointToString(s.massPoint2);
    return ss.str();
    

}

std::vector<MassPoint> massPoints;
std::vector<Spring> springs;

int methode = EULER;
MassSpringSystemSimulator::MassSpringSystemSimulator()
{
     m_fMass = 1.;
	 m_fStiffness = 1.;
	 m_fDamping = 0.;
}

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


 

void MassSpringSystemSimulator::simulateTimestep(float timeStep);
{
    for(auto &mp : massPoints){
            mp.force = Vec3();//Clear Forces
            //TODO: Gravity;
        }

    //Apply Forces
    for(auto &s : springs){
        std::cout << springToString(s);
        MassPoint &m1 = massPoints[s.massPoint1];
        MassPoint &m2 = massPoints[s.massPoint1];
        if(m1.isFixed&&m2.isFixed)
            continue;
        float lengthDif = distance(m1.postion,m2.position)-s.initialLength;
        float totalForce = m_fStiffness*lengthDif;
        if(m1.isFixed){
            m2.force += totalForce*(m1.position-m2.position);
        }
        else if(m2.isFixed){
            m1.force += totalForce*(m2.position-m1.position);
        }
        else{
            m1.force += totalForce*(m2.position-m1.position);
            m2.force += totalForce*(m1.position-m2.position);
        }
    }

    //Integrate based on Method
    switch (methode)
    {
    case EULER:
        for(auto &ms : massPoints){
           EulerIntegration(ms,timeStep);
        }
        break;
    case MIDPOINT:
        for(auto &ms : massPoints){
           MidPointIntegration(ms,timeStep);
        }
        break;
    default:
        break;
    }
#if 0
      //Collision with GroundPlane
    if(ms.position<0){
        ms.position = 0;
        ms.velocity.z = 0; //*= -1 für bounciness
    }
#endif

    for(auto &s : springs){
        std::cout <<"\nUpdates Springs: " <<springToString(s);
}


/*Boring Getters and Setters*/
void MassSpringSystemSimulator::setMass(float mass)
{
    m_fMass = mass;
}
void MassSpringSystemSimulator::setStiffness(float stiffness)
{
    m_fStiffness = stiffness;
}
void MassSpringSystemSimulator::setDampingFactor(float damping)
{
     m_fDamping = damping;
}
int MassSpringSystemSimulator::addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed)
{
    MassPoint mp = {position,Velocity,isFixed,Vec3(),m_fMass};
    massPoints.push_back(mp);
}
void MassSpringSystemSimulator::addSpring(int masspoint1, int masspoint2, float initialLength)
{
    Spring s = {masspoint1,masspoint2,initialLength};
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
    if(index<=massPoints.size())
        return massPoints[i].position;
    else
        return Vec3();
}
Vec3 MassSpringSystemSimulator::getVelocityOfMassPoint(int index)
{
    if(index<=massPoints.size())
        return massPoints[i].velocity;
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


void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass * DUC)
{
	this->DUC = DUC;
	switch (m_iTestCase)

    TwAddVarRW(DUC->g_pTweakBar, "Simulation Methode", TW_TYPE_INT32, &methode, "min=0 max=2");

}
