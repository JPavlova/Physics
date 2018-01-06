#include "MassSpringSystemSimulator.h"


struct massPoint
{
	Vec3 pos;
	Vec3 vel;
	bool isFixed;
};

struct spring
{
	int mp1;
	int mp2;
	float iniLenght;

};

std::vector<massPoint> points;
std::vector<spring> springs;

//DEBUG REASONS
Vec3 f1;
Vec3 f2;
Vec3 p1;
Vec3 p2;
Vec3 v1;
Vec3 v2;
Vec3 a1;
Vec3 a2;

float l, L;

Vec3 force = { 0,0,0 };
int pointI;

MassSpringSystemSimulator::MassSpringSystemSimulator()
{
	m_iTestCase = 0;
	m_fSphereSize = 0.025f;
	setMass(10.0f);
	setDampingFactor(1.0f);
	setStiffness(100.0f);

}

const char * MassSpringSystemSimulator::getTestCasesStr()
{
	return "Demo1, Demo2, Demo3, Demo4";
}

void MassSpringSystemSimulator::setMass(float mass)
{
	MassSpringSystemSimulator::m_fMass = mass;
}

void MassSpringSystemSimulator::setStiffness(float stiffness)
{
	MassSpringSystemSimulator::m_fStiffness = stiffness;
}

void MassSpringSystemSimulator::setDampingFactor(float damping)
{
	MassSpringSystemSimulator::m_fDamping = damping;
}

void MassSpringSystemSimulator::addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed)
{
	massPoint mp = { position, Velocity, isFixed };
	points.push_back(mp);
}

void MassSpringSystemSimulator::addSpring(int masspoint1, int masspoint2, float initialLength)
{
	spring s = {masspoint1, masspoint2, initialLength};
	springs.push_back(s);
}

int MassSpringSystemSimulator::getNumberOfMassPoints()
{
	return (int) points.size();
}

int MassSpringSystemSimulator::getNumberOfSprings()
{
	return (int) springs.size();
}

Vec3 MassSpringSystemSimulator::getPositionOfMassPoint(int index)
{
	return points[index].pos;
}

Vec3 MassSpringSystemSimulator::getVelocityOfMassPoint(int index)
{
	return points[index].vel;
}

void MassSpringSystemSimulator::applyExternalForce(Vec3 force)
{
	
	switch (m_iTestCase)
	{
	case 0: 
		break;
	case 1: 
		
		break;
	case 2: 
		break;
	case 3:
		for (int i = 0; i < points.size(); i++)
		{

		}
		break;
	default:
		break;
	}
}

/*UI Functions*/
void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass * DUC)
{
	this->DUC = DUC;
	switch (m_iTestCase)
	{
	case 0: 
		break;
	case 1: 
		break;
	case 2:
		break;
	case 3: 
		TwAddVarRW(DUC->g_pTweakBar, "Borders", TW_TYPE_BOOLCPP, &borders, "");
		TwAddVarRW(DUC->g_pTweakBar, "Euler", TW_TYPE_BOOLCPP, &euler, "");
		break;
	default:break;
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
	switch (m_iTestCase)
	{
	case 0: 
		////DUC->setUpLighting(Vec3(), 0.4*Vec3(1, 1, 1), 100, 0.6*Vec3(1, 1, 0));
		////DUC->drawSphere(Vec3(0.5f, 0,0), Vec3(0.025f, 0.025f, 0.025f));

		break;
	case 1:
		DUC->setUpLighting(Vec3(), 0.4*Vec3(1, 1, 1), 100, 0.6*Vec3(1,1,0));
		DUC->drawSphere(points[0].pos, Vec3(m_fSphereSize, m_fSphereSize, m_fSphereSize));

		DUC->setUpLighting(Vec3(), 0.4*Vec3(1, 1, 1), 100, 0.6*Vec3(1, 0, 0));
		DUC->drawSphere(points[1].pos, Vec3(m_fSphereSize, m_fSphereSize, m_fSphereSize));

		DUC->beginLine();
		DUC->drawLine(points[0].pos, Vec3(1, 1, 0), points[1].pos, Vec3(1, 0, 0));
		DUC->endLine();
		break;
	case 2:
		DUC->setUpLighting(Vec3(), 0.4*Vec3(1, 1, 1), 100, 0.6*Vec3(1, 1, 0));
		DUC->drawSphere(points[0].pos, Vec3(m_fSphereSize, m_fSphereSize, m_fSphereSize));

		DUC->setUpLighting(Vec3(), 0.4*Vec3(1, 1, 1), 100, 0.6*Vec3(1, 0, 0));
		DUC->drawSphere(points[1].pos, Vec3(m_fSphereSize, m_fSphereSize, m_fSphereSize));

		DUC->beginLine();
		DUC->drawLine(points[0].pos, Vec3(1, 1, 0), points[1].pos, Vec3(1, 0, 0));
		DUC->endLine();
		break;
	case 3:
		m_fSphereSize = 0.05f;
		for (int i = 0; i < points.size(); i++)
		{
			DUC->setUpLighting(Vec3(), 0.4*Vec3(1, 1, 1), 100, 0.6*Vec3(1, 1, 0));
			DUC->drawSphere(points[i].pos, Vec3(m_fSphereSize, m_fSphereSize, m_fSphereSize));
		}

		for (int j = 0; j < springs.size(); j++)
		{
			DUC->beginLine();
			DUC->drawLine(points[springs[j].mp1].pos, Vec3(1, 1, 0),
				points[springs[j].mp2].pos, Vec3(1, 0, 0));
			DUC->endLine();
		}

		break;
	}
}



void MassSpringSystemSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	switch (m_iTestCase)
	{
	case 0:
		cout << "Demo 1\n";
		points.clear();
		springs.clear();

		addMassPoint(Vec3(0, 0, 0), Vec3(-1, 0, 0), false);
		addMassPoint(Vec3(0, 2, 0), Vec3(1, 0, 0), false);

		addSpring(0, 1, 1);
		//Calculate Euler 
		L = springs[0].iniLenght;
		l = sqrt(pow(points[0].pos.x - points[1].pos.x, 2) + pow(points[0].pos.y - points[1].pos.y, 2) + pow(points[0].pos.z - points[1].pos.z, 2));
		// Calculate Force
		f1 = -m_fStiffness*(l - L)*((points[0].pos - points[1].pos) / l);
		f2 = -m_fStiffness*(l - L)*((points[1].pos - points[0].pos) / l);

		//Calculate accelerations
		a1 = f1 / m_fMass;
		a2 = f2 / m_fMass;

		//Euler! 
		p1 = points[0].pos + 0.1*points[0].vel;
		v1 = points[0].vel + 0.1*a1;

		p2 = points[1].pos + 0.1*points[1].vel;
		v2 = points[1].vel + 0.1*a2;

		cout << "Euler: \np1: " << p1 << " v1: "<< v1 << "\n";
		cout << "p2: " << p2 << " v2: " << v2 <<"\n";

		//Calculate Midpoint

		//Calculate Length between Points
		L = springs[0].iniLenght;
		l = sqrt(pow(points[0].pos.x - points[1].pos.x, 2) + pow(points[0].pos.y - points[1].pos.y, 2) + pow(points[0].pos.z - points[1].pos.z, 2));

		// Calculate Force
		f1 = -m_fStiffness*(l - L)*((points[0].pos - points[1].pos) / l);
		f2 = -m_fStiffness*(l - L)*((points[1].pos - points[0].pos) / l);

		//Calculate accelerations
		a1 = f1 / m_fMass;
		a2 = f1 / m_fMass;

		//Midpoint! 
		p1 = points[0].pos + 0.05*points[0].vel;
		v1 = points[0].vel + 0.05*a1;

		p2 = points[1].pos + 0.05*points[1].vel;
		v2 = points[1].vel + 0.05*a2;

		// Calculate Force
		f1 = -m_fStiffness*(l - L)*((p1 - p2) / l);
		f2 = -m_fStiffness*(l - L)*((p2 - p1) / l);

		a1 = f1 / m_fMass;
		a2 = f2 / m_fMass;

		p1 = points[0].pos + 0.1*v1;
		v1 = points[0].vel + 0.1*a1;

		p2 = points[1].pos + 0.1*v2;
		v2 = points[1].vel + 0.1*a2;

		cout << "Midpoint: \np1: " << p1 << " v1: " << v1 << "\n";
		cout << "p2: " << p2 << " v2: " << v2 << "\n";

		break;
	case 1:
		cout << "Demo 2\n";
		//Clear all vectors
		points.clear();
		springs.clear();

		addMassPoint(Vec3(0, 0, 0), Vec3(0, 0.1, 0), false);
		addMassPoint(Vec3(0, 0.2, 0), Vec3(0, -0.1, 0), false);

		addSpring(0, 1, 0.5f);
		
		break;
	case 2:
		cout << "Demo 3\n";
		points.clear();
		springs.clear();

		addMassPoint(Vec3(0, 0, 0), Vec3(0, 0, 0), false);
		addMassPoint(Vec3(0, 0.2, 0), Vec3(0, 0, 0), false);

		addSpring(0, 1, 0.5f);
		break;
	case 3:
		cout << "Demo 4 \n";
		points.clear();
		springs.clear();

		setMass(0.5f);

		addMassPoint(Vec3(0.4f, 0.4f, -0.4f), Vec3(0, 0, 0), true);
		addMassPoint(Vec3(-0.4f, 0.4f, -0.4f), Vec3(0, 0, 0), true);
		addMassPoint(Vec3(0.4f, 0.4f, 0.4f), Vec3(0, 0, 0), true);
		addMassPoint(Vec3(-0.4f, 0.4f, 0.4f), Vec3(0, 0, 0), true);

		addMassPoint(Vec3(0.4f, -0.4f, -0.4f), Vec3(0, 0, 0), false);
		addMassPoint(Vec3(-0.4f, -0.4f, -0.4f), Vec3(0, 0, 0), false);
		addMassPoint(Vec3(0.4f, -0.4f, 0.4f), Vec3(0, 0, 0), false);
		addMassPoint(Vec3(-0.4f, -0.4f, 0.4f), Vec3(0, 0, 0), false);

		addMassPoint(Vec3(0.2f, 0, 0), Vec3(0, 0, 0), false);
		addMassPoint(Vec3(-0.2f, 0, 0), Vec3(0, 0, 0), false);

		addSpring(0, 1, 0.8f);
		addSpring(1, 3, 0.8f);
		addSpring(3, 2, 0.8f);
		addSpring(2, 0, 0.8f);

		addSpring(4, 5, 0.8f);
		addSpring(5, 7, 0.8f);
		addSpring(7, 6, 0.8f);
		addSpring(6, 4, 0.8f);

		addSpring(0, 4, 0.8f);
		addSpring(2, 6, 0.8f);
		addSpring(1, 5, 0.8f);
		addSpring(3, 7, 0.8f);

	//	addSpring(8, 9, 0.8f);

		addSpring(0, 8, 0.8f);
		addSpring(2, 8, 0.8f);
		//addSpring(4, 8, 0.8f);
		//addSpring(6, 8, 0.8f);

		addSpring(1, 9, 0.8f);
		addSpring(3, 9, 0.8f);
		//addSpring(5, 9, 0.8f);
		//addSpring(7, 9, 0.8f);
	//	makeSpheres(10);
	//	setSprings();
		break;
	default:
		cout << "Empty Test!\n";
		break;
	}
}

void MassSpringSystemSimulator::makeSpheres(int numSpheres)
{
	std::mt19937 eng;
	std::uniform_real_distribution<float> randPos(-0.30f, 0.30f);
	for (int i = 0; i < numSpheres; i++)
	{
		addMassPoint(Vec3(randPos(eng), randPos(eng), randPos(eng)), Vec3(), false);
	}
}

void MassSpringSystemSimulator::setSprings()
{
	for (int i = 0; i < points.size(); i++)
	{
		for (int j = i + 1; j < points.size(); j++)
		{
			l = sqrt(pow(points[j].pos.x - points[i].pos.x, 2) + pow(points[j].pos.y - points[i].pos.y, 2) + pow(points[j].pos.z - points[i].pos.z, 2));

			if (l < 0.5f)
			{
				addSpring(i, j, l);
			}
		}
	}
}

void MassSpringSystemSimulator::checkObstacles()
{
	float t;
	for (int i = 0; i < points.size(); i++)
	{
		for (int j = i + 1; j < points.size(); j++)
		{
			l = sqrt(pow(points[j].pos.x - points[i].pos.x, 2) + pow(points[j].pos.y - points[i].pos.y, 2) + pow(points[j].pos.z - points[i].pos.z, 2));

			if (l < 0.05f)
			{
				t = points[j].pos.X - points[i].pos.X;
				if (t < 0.05f)
				{
					points[j].pos.X = points[j].pos.X + (0.05f - t);
				}

				t = points[j].pos.Y - points[i].pos.Y;
				if (t < 0.05f)
				{
					points[j].pos.Y = points[j].pos.Y + (0.05f - t);
				}

				t = points[j].pos.Z - points[i].pos.Z;
				if (t < 0.05f)
				{
					points[j].pos.Z = points[j].pos.Z + (0.05f - t);
				}
			}
		}
	}
}

void MassSpringSystemSimulator::checkBorders(int inum)
{
	int i = inum;

		if (points[i].pos.X > 0.5f)
		{
			points[i].pos.X = 0.5f;

		}
		if (points[i].pos.X < -0.5f)
		{
			points[i].pos.X = -0.5f;
		}

		if (points[i].pos.Y > 0.5f)
		{
			points[i].pos.Y = 0.5f;

		}
		if (points[i].pos.Y < -0.5f)
		{
			points[i].pos.Y = -0.5f;
		}

		if (points[i].pos.Z > 0.5f)
		{
			points[i].pos.Z = 0.5f;

		}
		if (points[i].pos.Z < -0.5f)
		{
			points[i].pos.Z = -0.5f;
		}
	
}

void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed)
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

void MassSpringSystemSimulator::simulateTimestep(float timeStep)
{
	switch (m_iTestCase)
	{
	case 0: break;
	case 1:
			L = springs[0].iniLenght;
			l = sqrt(pow(points[0].pos.x - points[1].pos.x, 2) + pow(points[0].pos.y - points[1].pos.y, 2) + pow(points[0].pos.z - points[1].pos.z, 2));
			// Calculate Force
			f1 = -m_fStiffness*(l - L)*((points[0].pos - points[1].pos) / l);
			f2 = -m_fStiffness*(l - L)*((points[1].pos - points[0].pos) / l);

			//Calculate accelerations
			a1 = f1 / m_fMass;
			a2 = f2 / m_fMass;

			//Euler! 
			points[0].pos = points[0].pos + timeStep*points[0].vel;
			points[1].pos = points[1].pos + timeStep*points[1].vel;

			checkBorders(0);
			checkBorders(1);

			points[1].vel = points[1].vel + timeStep*a2;
			points[0].vel = points[0].vel + timeStep*a1;


		break; 
	case 2: 
		L = springs[0].iniLenght;
		l = sqrt(pow(points[0].pos.x - points[1].pos.x, 2) + pow(points[0].pos.y - points[1].pos.y, 2) + pow(points[0].pos.z - points[1].pos.z, 2));

		// Calculate Force
		f1 = -m_fStiffness*(l - L)*((points[0].pos - points[1].pos) / l);
		f2 = -m_fStiffness*(l - L)*((points[1].pos - points[0].pos) / l);

		//Calculate accelerations
		a1 = f1 / m_fMass;
		a2 = f1 / m_fMass;

		//Midpoint! 
		p1 = points[0].pos + (timeStep/2)*points[0].vel;
		p2 = points[1].pos + (timeStep / 2)*points[1].vel;

		checkBorders(0);
		checkBorders(1);

		v1 = points[0].vel + (timeStep / 2)*a1;
		v2 = points[1].vel + (timeStep / 2)*a2;

		// Calculate Force
		f1 = -m_fStiffness*(l - L)*((p1 - p2) / l);
		f2 = -m_fStiffness*(l - L)*((p2 - p1) / l);

		a1 = f1 / m_fMass;
		a2 = f2 / m_fMass;

		points[0].pos = points[0].pos + timeStep*v1;
		points[1].pos = points[1].pos + timeStep*v2;


		checkBorders(0);
		checkBorders(1);

		points[0].vel = points[0].vel + timeStep*a1;
		points[1].vel = points[1].vel + timeStep*a2;

		checkBorders(0);
		checkBorders(1);
		break;
	case 3:

		if (euler)
		{
			for (int i = 0; i < springs.size(); i++)
			{
				
				l = points[springs[i].mp1].pos.squaredDistanceTo(points[springs[i].mp2].pos);
				

				if (l != springs[i].iniLenght)
				{
					if (euler)
					{
						if (!points[springs[i].mp1].isFixed)
						{
						
							f1 = calculateInternalForce(springs[i].mp1);
							f1 += Vec3(0, -9.81*m_fMass, 0) + (-m_fDamping*points[springs[i].mp1].vel) + m_externalForce;
							a1 = f1 / m_fMass;

							points[springs[i].mp1].pos += timeStep*points[springs[i].mp1].vel;
							
							if (borders)
							{
								checkBorders(springs[i].mp1);
							}
							points[springs[i].mp1].vel += timeStep*a1;
							f1 = Vec3(0, 0, 0);
							
						}

						if (!points[springs[i].mp2].isFixed)
						{
							
							f2 = calculateInternalForce(springs[i].mp2);
							f2 += Vec3(0, -9.81*m_fMass, 0) + (-m_fDamping*points[springs[i].mp2].vel)+m_externalForce;
							a2 = f2 / m_fMass;

							points[springs[i].mp2].pos += timeStep*points[springs[i].mp2].vel;
							if (borders)
							{
								checkBorders(springs[i].mp2);
							}
							points[springs[i].mp2].vel += timeStep*a2;
							
						}

					}
				}
			}

		}
		else if(!euler)
 {
			for (int i = 0; i < springs.size(); i++)
			{
				l = points[springs[i].mp1].pos.squaredDistanceTo(points[springs[i].mp2].pos);

				if (l != springs[i].iniLenght)
				{ 
					if (!points[springs[i].mp1].isFixed)
					{
						f1 = calculateInternalForce(springs[i].mp1);
						f1 += Vec3(0, -9.81*m_fMass, 0) + (-m_fDamping*points[springs[i].mp1].vel) + m_externalForce;
						a1 = f1 / m_fMass;

						p1 = points[springs[i].mp1].pos;

						points[springs[i].mp1].pos = points[springs[i].mp1].pos + (timeStep / 2) * points[springs[i].mp1].vel;

						v1 = points[springs[i].mp1].vel + timeStep / 2 * a1;

						f1 = calculateInternalForce(springs[i].mp1);
						f1 += Vec3(0, -9.81*m_fMass, 0) + (-m_fDamping*points[springs[i].mp1].vel)+m_externalForce;

						a1 = f1 / m_fMass;

						points[springs[i].mp1].pos = p1 + timeStep*v1;

						points[springs[i].mp1].vel += timeStep*a1;
					}
					if (!points[springs[i].mp2].isFixed)
					{
						f2 = calculateInternalForce(springs[i].mp2);
						f2 += Vec3(0, -9.81*m_fMass, 0) + (-m_fDamping*points[springs[i].mp2].vel) + m_externalForce;

						a2 = f2 / m_fMass;

						p2 = points[springs[i].mp2].pos;

						points[springs[i].mp2].pos = points[springs[i].mp2].pos + (timeStep / 2) * points[springs[i].mp1].vel;

						v2 = points[springs[i].mp2].vel + timeStep / 2 * a2;


						f2 = calculateInternalForce(springs[i].mp2);
						f2 += Vec3(0, -9.81*m_fMass, 0) + (-m_fDamping*points[springs[i].mp2].vel)+m_externalForce;

						a2 = f2 / m_fMass;

						points[springs[i].mp2].pos = p2 + timeStep*v2;


						points[springs[i].mp2].vel += timeStep*a2;
					}
				}
			}

		}
	
		
		break;
	default: break;
	}
}

Vec3 MassSpringSystemSimulator::calculateInternalForce(int p)
{

	float length;
	Vec3 totalInForce;
	
	for (int i = 0; i < springs.size(); i++)
	{
		if (springs[i].mp1 == p)
		{
			length = points[springs[i].mp2].pos.squaredDistanceTo(points[springs[i].mp1].pos);
			totalInForce += -1*m_fStiffness*(length - springs[i].iniLenght)*((points[p].pos - points[springs[i].mp2].pos) / length);
		}
		if (springs[i].mp2 == p)
		{
			length = points[springs[i].mp1].pos.squaredDistanceTo(points[springs[i].mp2].pos);
			totalInForce += -1*m_fStiffness*(length - springs[i].iniLenght)*((points[p].pos - points[springs[i].mp1].pos) / length);
		}
	}
	return totalInForce;
	
}

void MassSpringSystemSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;


}

Vec3 MassSpringSystemSimulator::crossProduct(Vec3 m1, Vec3 m2)
{
	Vec3 result;
	result.x = m1.y*m2.z - m1.z*m2.y;
	result.y = m1.z*m2.x - m1.x*m2.z;
	result.z = m1.x*m2.y - m1.y*m2.x;

	return result;
}

void MassSpringSystemSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}
