#include "RigidBodySystemSimulator.h"
#include "collisionDetect.h"

struct rigidB
{
	Vec3 pos; 
	Vec3 lVel; 
	Vec3 wVel; 
	Quat r; 
	Vec3 size;
	float m; 
};

Mat4 rot;
Quat tmp;

Vec3 f; 
Vec3 poi; 
Vec3 q; 
Vec3 L;

Mat4 I_inv;
Mat4 newI_inv;

Vec3 worldPos; 
Vec3 worldVel;

CollisionInfo colInfo;

Mat4 BodyRot; 
Mat4 BodyScale;
Mat4 BodyTrans;
Mat4 Body;

std::vector<rigidB> rigidBodies; 

RigidBodySystemSimulator::RigidBodySystemSimulator()
{
	m_iTestCase = 0;

}

const char * RigidBodySystemSimulator::getTestCasesStr()
{
	return "Demo1, Demo2, Demo3, Demo4";
}

void RigidBodySystemSimulator::initUI(DrawingUtilitiesClass * DUC)
{

}

void RigidBodySystemSimulator::reset()
{
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void RigidBodySystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	switch (m_iTestCase)
	{
	//case 0:
	//	DUC->setUpLighting(Vec3(), 0.4*Vec3(1, 1, 1), 100, 0.6*Vec3(1, 1, 0));
	//	DUC->drawSphere(Vec3(0.5f, 0,0), Vec3(0.025f, 0.025f, 0.025f));

	//	break;
	//case 1:
	//	DUC->setUpLighting(Vec3(1,1,1), Vec3(1, 1, 1), 100, Vec3(1, 1, 0));
	//	DUC->drawSphere(Vec3(0,0,0), Vec3(0.25f, 0.25f, 0.25f));

	//	break;
	}
}

void RigidBodySystemSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	switch (m_iTestCase)
	{
	case 0:
		rigidBodies.clear();
		addRigidBody(Vec3(0, 0, 0), Vec3(1, 0.6f, 0.5f), 2.0f);

		rot.initRotationZ(90);

		setOrientationOf(0, rotMatToQuat(rot));

		f = { 1, 1, 0 };
		poi = rigidBodies[0].pos - Vec3(0.3f, 0.5f, 0.25f);
		q = cross(poi, f);
		L = { 0,0,0 };

		rigidBodies[0].pos = rigidBodies[0].pos + 2 * rigidBodies[0].lVel;
		rigidBodies[0].lVel = rigidBodies[0].lVel + 2 * f / 2;

		L = L + 2 * q;


		I_inv;
		I_inv.initId();
		I_inv.value[0][0] = (1.0f / 12.0f * rigidBodies[0].m*(pow(rigidBodies[0].size.y, 2) + pow(rigidBodies[0].size.z, 2)));
		I_inv.value[1][1] = (1.0f / 12.0f * rigidBodies[0].m*(pow(rigidBodies[0].size.x, 2) + pow(rigidBodies[0].size.z, 2)));
		I_inv.value[2][2] = (1.0f / 12.0f * rigidBodies[0].m*(pow(rigidBodies[0].size.x, 2) + pow(rigidBodies[0].size.y, 2)));
		I_inv.value[3][3] = 0;
		I_inv.inverse();

		newI_inv;
		newI_inv = rigidBodies[0].r.getRotMat().operator*(I_inv.operator*(rigidBodies[0].r.getRotMat().inverse()));

		rigidBodies[0].wVel = newI_inv.operator*(L);

		worldPos = rigidBodies[0].pos + rigidBodies[0].r.getRotMat().operator*(poi);
		worldVel = rigidBodies[0].lVel + cross(rigidBodies[0].wVel, poi);

		cout << "linear Velocity: "<< rigidBodies[0].lVel << "\n";
		cout << "angular Velocity: "<<rigidBodies[0].wVel << "\n";
		cout << "world Velocity: " << worldVel << "\n";
		
		break;
	case 1:
		rigidBodies.clear();

		addRigidBody(Vec3(0, 0, 0), Vec3(1, 0.6f, 0.5f), 2.0f);

		rot.initRotationZ(90);

		setOrientationOf(0, rotMatToQuat(rot));
		L = Vec3(0, 0, 0);
		break;
	case 2:
		rigidBodies.clear();
		break;
	case 3:
		rigidBodies.clear();
		break;
	}
}

void RigidBodySystemSimulator::externalForcesCalculations(float timeElapsed)
{
	
}

void RigidBodySystemSimulator::simulateTimestep(float timeStep)
{
	switch (m_iTestCase)
	{
	case 0: break;
	case 1:
		f = m_externalForce;
		q = cross(rigidBodies[0].pos, f);

		rigidBodies[0].pos += timeStep*rigidBodies[0].lVel;
		rigidBodies[0].lVel += timeStep*f / rigidBodies[0].m;

		rot = rigidBodies[0].r.getRotMat();
		L += timeStep*q; 

		I_inv.initId();
		I_inv.value[0][0] = (1.0f / 12.0f * rigidBodies[0].m*(pow(rigidBodies[0].size.y, 2) + pow(rigidBodies[0].size.z, 2)));
		I_inv.value[1][1] = (1.0f / 12.0f * rigidBodies[0].m*(pow(rigidBodies[0].size.x, 2) + pow(rigidBodies[0].size.z, 2)));
		I_inv.value[2][2] = (1.0f / 12.0f * rigidBodies[0].m*(pow(rigidBodies[0].size.x, 2) + pow(rigidBodies[0].size.y, 2)));
		I_inv.value[3][3] = 0;
		I_inv.inverse();

		newI_inv = rigidBodies[0].r.getRotMat().operator*(I_inv.operator*(rigidBodies[0].r.getRotMat().inverse()));

		rigidBodies[0].wVel = newI_inv.operator*(L);

		worldPos = rigidBodies[0].pos + rigidBodies[0].r.getRotMat().operator*(rigidBodies[0].pos);
		worldVel = rigidBodies[0].lVel + cross(rigidBodies[0].wVel, rigidBodies[0].pos);
		
		break;
	case 2: break;
	case 3: break;
	default: break;
	}
	
}

void RigidBodySystemSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void RigidBodySystemSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

// ExtraFunctions
int RigidBodySystemSimulator::getNumberOfRigidBodies()
{
	return rigidBodies.size();
}

Vec3 RigidBodySystemSimulator::getPositionOfRigidBody(int i)
{
	return rigidBodies[i].pos;
}

Vec3 RigidBodySystemSimulator::getLinearVelocityOfRigidBody(int i)
{
	return rigidBodies[i].lVel;
}

Vec3 RigidBodySystemSimulator::getAngularVelocityOfRigidBody(int i)
{
	return rigidBodies[i].wVel;
}

void RigidBodySystemSimulator::applyForceOnBody(int i, Vec3 loc, Vec3 force)
{

}

void RigidBodySystemSimulator::addRigidBody(Vec3 position, Vec3 size, int mass)
{
	Vec3 zero = { 0,0,0 };
	Quat ori = { 0,0,0,0 };
	rigidB tmp = { position, zero, zero, ori, size, mass };
	rigidBodies.push_back(tmp);
}

void RigidBodySystemSimulator::setOrientationOf(int i, Quat orientation)
{
	rigidBodies[i].r = orientation;
}

void RigidBodySystemSimulator::setVelocityOf(int i, Vec3 velocity)
{
	rigidBodies[i].lVel = velocity;
}

Quat RigidBodySystemSimulator::rotMatToQuat(Mat4 m)
{
	Quat quatTmp(m);
	return quatTmp;
}