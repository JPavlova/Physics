#include "SphereSystemSimulator.h"

std::function<float(float)> SphereSystemSimulator::m_Kernels[5] = {
	[](float x) {return 1.0f; },              // Constant, m_iKernel = 0
	[](float x) {return 1.0f - x; },          // Linear, m_iKernel = 1, as given in the exercise Sheet, x = d/2r
	[](float x) {return (1.0f - x)*(1.0f - x); }, // Quadratic, m_iKernel = 2
	[](float x) {return 1.0f / (x)-1.0f; },     // Weak Electric Charge, m_iKernel = 3
	[](float x) {return 1.0f / (x*x) - 1.0f; },   // Electric Charge, m_iKernel = 4
};

// SphereSystemSimulator member functions

int diffSize;

SphereSystemSimulator::SphereSystemSimulator()
{
	m_iTestCase = 0;
	m_iNumSpheres = 10;
	m_fRadius = 0.2f;
	m_fMass = 2.0f;
	
}

const char* SphereSystemSimulator::getTestCasesStr()
{
	return "Force Brutale, Demo2, Demo3";
}

void SphereSystemSimulator::initUI(DrawingUtilitiesClass * DUC)
{
	this->DUC = DUC;
	switch (m_iTestCase)
	{
	case 0:
		TwAddVarRW(DUC->g_pTweakBar, "Number of Spheres", TW_TYPE_INT32, &m_iNumSpheres, "" );
		TwAddVarRW(DUC->g_pTweakBar, "Radius", TW_TYPE_FLOAT, &m_fRadius, "");
		TwAddVarRW(DUC->g_pTweakBar, "Mass", TW_TYPE_FLOAT, &m_fMass, "");
		break;
	case 1: 
		TwAddVarRW(DUC->g_pTweakBar, "Number of Spheres", TW_TYPE_INT32, &m_iNumSpheres, "");
		TwAddVarRW(DUC->g_pTweakBar, "Radius", TW_TYPE_FLOAT, &m_fRadius, "");
		TwAddVarRW(DUC->g_pTweakBar, "Mass", TW_TYPE_FLOAT, &m_fMass, "");
		break; 
	case 2: 
		TwAddVarRW(DUC->g_pTweakBar, "Number of Spheres", TW_TYPE_INT32, &m_iNumSpheres, "");
		TwAddVarRW(DUC->g_pTweakBar, "Radius", TW_TYPE_FLOAT, &m_fRadius, "");
		TwAddVarRW(DUC->g_pTweakBar, "Mass", TW_TYPE_FLOAT, &m_fMass, "");
		break;
	}
}

void SphereSystemSimulator::reset()
{
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void SphereSystemSimulator::drawFrame(ID3D11DeviceContext* pd3ImmediateContext)
{
	switch (m_iTestCase)
	{
	case 0: 
		for (int i = 0; i < m_iNumSpheres; i++)
		{
			DUC->setUpLighting(Vec3(), 0.4*Vec3(1, 1, 1), 100, 0.6*Vec3(1, 1, 0));
			DUC->drawSphere(Vec3(m_sSphere[i].pos.x /*+i*0.1f*/, m_sSphere[i].pos.y, m_sSphere[i].pos.z), Vec3(m_fRadius, m_fRadius, m_fRadius));
		}
		break;
	}
}

void SphereSystemSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	switch (m_iTestCase)
	{
	case 0:
		m_iNumSpheres = 1; 
		m_fMass = 2.0f;
		m_fRadius = 0.02f;
		break;
	case 1: break;
	case 2: break; 
	default: break;
	}
}

void SphereSystemSimulator::externalForcesCalculations(float timeElapsed)
{
	Vec3 pullforce(0, 0, 0);
	Point2D mouseDiff;
	mouseDiff.x = m_trackmouse.x - m_oldtrackmouse.x;
	mouseDiff.y = m_trackmouse.y - m_oldtrackmouse.y;
	if (mouseDiff.x != 0 || mouseDiff.y != 0)
	{
		Mat4 worldViewInv = Mat4(DUC->g_camera.GetWorldMatrix() * DUC->g_camera.GetViewMatrix());
		worldViewInv = worldViewInv.inverse();
		Vec3 forceView = Vec3((float)mouseDiff.x, (float)-mouseDiff.y, 0);
		Vec3 forceWorld = worldViewInv.transformVectorNormal(forceView);
		float forceScale = 0.03f;
		pullforce = pullforce + (forceWorld * forceScale);
	}
	m_externalForce = pullforce;
}

void SphereSystemSimulator::simulateTimestep(float timeStep)
{
	sphere s = { Vec3(0,0,0), Vec3(0,0,0), Vec3(0,0,0) };
	switch (m_iTestCase)
	{
	case 0:
		if (m_sSphere.size() < m_iNumSpheres)
		{
			diffSize = m_iNumSpheres - m_sSphere.size();
			for (int i = 0; i < diffSize; i++)
			{
				s.pos = Vec3(0, 0.4f,0);
				
				m_sSphere.push_back(s);
			}
			cout << m_sSphere.size() << "\n";
		}
		else if (m_sSphere.size() > m_iNumSpheres)
		{
			diffSize = m_sSphere.size() - m_iNumSpheres; 
			m_sSphere._Pop_back_n(diffSize);
		}
		
		//Calculate Spheres
		for (int i = 0; i < m_iNumSpheres; i++)
		{

			m_fForce = Vec3(0, -9.81f*m_fMass, 0) + m_externalForce;
			m_fAcceleration = m_fForce / m_fMass;

			
			m_sSphere[i].pos = m_sSphere[i].pos + (timeStep / 2)*m_sSphere[i].vel;
			m_sSphere[i].vel = m_sSphere[i].vel + (timeStep / 2)*m_fAcceleration;

			
			m_fForce -= m_sSphere[i].rep_force;
			m_fAcceleration = m_fForce / m_fMass;

			m_sSphere[i].pos = m_sSphere[i].pos + timeStep*m_sSphere[i].vel;
			detectBoxCollision(i);
			detectCollisionSimple(i);
			m_sSphere[i].vel = m_sSphere[i].vel + timeStep*m_fAcceleration;

			m_sSphere[i].rep_force = Vec3(0,0,0);
		}

		
		break; 
	case 1: break; 
	case 2: break;
	default: break;
	}
}

void SphereSystemSimulator::detectCollisionSimple(int i)
{
	float d;
	Vec3 tmp;
	for (int j = 0; j < m_iNumSpheres; j++)
	{
		if (i == j)
		{
			break;
		}
		else
		{				
			d = m_sSphere[i].pos.squaredDistanceTo(m_sSphere[j].pos);
			if (d < 2 * m_fRadius)
			{
				tmp = m_sSphere[i].pos - m_sSphere[j].pos; 
				tmp = tmp / d;
				m_sSphere[i].rep_force += tmp*1.0f/d*d;
				m_sSphere[j].rep_force -= tmp*1.0f/d*d;
			
			}
		}
	}
	
}

void SphereSystemSimulator::detectCollisionBoundries(int i, int j)
{
	float tmp;
	tmp = m_sSphere[i].pos.squaredDistanceTo(m_sSphere[j].pos);
	if (tmp > m_fRadius * 2)
	{

	}
}

void SphereSystemSimulator::detectBoxCollision(int i)
{
		if ((m_sSphere[i].pos.x + m_fRadius) >= 0.5f)
		{
			//Collision mit Rechts
			m_sSphere[i].pos.x = 0.5f - m_fRadius;
			m_sSphere[i].rep_force -= Vec3(1/pow(m_fRadius,2),0,0);

		}
		if ((m_sSphere[i].pos.x - m_fRadius) <= -0.5f)
		{
			//Collision mit Links
			m_sSphere[i].pos.x = -0.5f + m_fRadius;
			m_sSphere[i].rep_force += Vec3(1 / pow(m_fRadius, 2),0,0);
		}

		if ((m_sSphere[i].pos.y + m_fRadius) >= 0.5f)
		{
			//Collision mit Oben
			m_sSphere[i].pos.y = 0.5f - m_fRadius;
			m_sSphere[i].rep_force -= Vec3(0, 1 / pow(m_fRadius, 2),0);
		}
		if ((m_sSphere[i].pos.y - m_fRadius) <= -0.5f)
		{
			//Collision mit Unten
			m_sSphere[i].pos.y = -0.5f + m_fRadius;
			m_sSphere[i].rep_force += Vec3(0, 1 / pow(m_fRadius, 2),0);
		}

		if ((m_sSphere[i].pos.z + m_fRadius) >= 0.5f)
		{
			//Collision mit Vorne
			m_sSphere[i].pos.z = 0.5f - m_fRadius;
			m_sSphere[i].rep_force -= Vec3(0,0, 1 / pow(m_fRadius, 2));
		}
		if ((m_sSphere[i].pos.z - m_fRadius) <= -0.5f)
		{
			//Collision mit Hinten
			m_sSphere[i].pos.z = -0.5f + m_fRadius;
			m_sSphere[i].rep_force += Vec3(0,0, 1 / pow(m_fRadius, 2));
		}
}

void SphereSystemSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x; 
	m_trackmouse.y = y;
}

void SphereSystemSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}