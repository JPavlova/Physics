#include "RigidBodySystemSimulator.h"


struct rigidB
{
	Vec3 pos; 
	Vec3 lVel; 
	Vec3 wVel; 
	Quat r; 
	Vec3 size;
	float m; 

	Vec3 L; 
	Mat4 I_inv;
};

Mat4 rot;
Quat tmp;

Vec3 f; 
Vec3 poi; 
Vec3 q; 
Vec3 L_tmp;

Mat4 I_inv_tmp;
Mat4 newI_inv;

Vec3 worldPos; 
Vec3 worldVel;

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
	this->DUC = DUC;
	switch (m_iTestCase)
	{
	case 0: break; 
	case 1: 
		TwAddVarRW(DUC->g_pTweakBar, "Borders", TW_TYPE_BOOLCPP, &borders, "");
		break;
	case 2: 
		TwAddVarRW(DUC->g_pTweakBar, "Borders", TW_TYPE_BOOLCPP, &borders, "");
		break; 
	case 3: 
		TwAddVarRW(DUC->g_pTweakBar, "Borders", TW_TYPE_BOOLCPP, &borders, "");
		break;
	default: break;

	}
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
	case 0:
		/*DUC->setUpLighting(Vec3(), 0.4*Vec3(1, 1, 1), 100, 0.6*Vec3(1, 1, 0));
		DUC->drawSphere(Vec3(0, 0,0), Vec3(0.025f, 0.025f, 0.025f));*/
		DUC->setUpLighting(Vec3(), 0.3*Vec3(1, 1, 1), 100, 0.9*Vec3(1, 1, 0));
		BodyRot.initRotationXYZ(0, 0, 0);
		BodyTrans.initTranslation(0, -0.6f, 0);
		BodyScale.initScaling(5.0f, 0.1f, 5.0f);
		Body = BodyScale * BodyRot * BodyTrans;
		DUC->drawRigidBody(Body);
		break;
	case 1:
		DUC->setUpLighting(Vec3(), 0.3*Vec3(1, 1, 1), 100, 0.9*Vec3(1, 1, 0));
		BodyRot = rigidBodies[0].r.getRotMat();
		BodyTrans.initTranslation(rigidBodies[0].pos.x, rigidBodies[0].pos.y, rigidBodies[0].pos.z);
		BodyScale.initScaling(0.2);
		Body = BodyScale* BodyRot * BodyTrans;
		DUC->drawRigidBody(Body);
		break;
	case 2: 
		DUC->setUpLighting(Vec3(), 0.3*Vec3(1, 1, 1), 100, 0.9*Vec3(1, 1, 0)); 
		BodyRot = rigidBodies[0].r.getRotMat();
		BodyTrans.initTranslation(rigidBodies[0].pos.x, rigidBodies[0].pos.y, rigidBodies[0].pos.z);
		BodyScale.initScaling(rigidBodies[0].size.x, rigidBodies[0].size.y, rigidBodies[0].size.z);
		Body = BodyScale* BodyRot * BodyTrans;
		DUC->drawRigidBody(Body);

		DUC->setUpLighting(Vec3(), 0.3*Vec3(1, 1, 1), 100, 0.9*Vec3(1, 0, 0));
		BodyRot = rigidBodies[1].r.getRotMat();
		BodyTrans.initTranslation(rigidBodies[1].pos.x, rigidBodies[1].pos.y, rigidBodies[1].pos.z);
		BodyScale.initScaling(rigidBodies[1].size.x, rigidBodies[1].size.y, rigidBodies[1].size.z);
		Body = BodyScale* BodyRot * BodyTrans;
		DUC->drawRigidBody(Body);
		break; 
	case 3: 
		DUC->setUpLighting(Vec3(), 0.3*Vec3(1, 1, 1), 100, 0.9*Vec3(1, 0, 0));
		BodyRot = rigidBodies[0].r.getRotMat();
		BodyTrans.initTranslation(rigidBodies[0].pos.x, rigidBodies[0].pos.y, rigidBodies[0].pos.z);
		BodyScale.initScaling(rigidBodies[0].size.x, rigidBodies[0].size.y, rigidBodies[0].size.z);
		Body = BodyScale* BodyRot * BodyTrans;
		DUC->drawRigidBody(Body);

		DUC->setUpLighting(Vec3(), 0.3*Vec3(1, 1, 1), 100, 0.9*Vec3(0, 1, 0));
		BodyRot = rigidBodies[1].r.getRotMat();
		BodyTrans.initTranslation(rigidBodies[1].pos.x, rigidBodies[1].pos.y, rigidBodies[1].pos.z);
		BodyScale.initScaling(rigidBodies[1].size.x, rigidBodies[1].size.y, rigidBodies[1].size.z);
		Body = BodyScale* BodyRot * BodyTrans;
		DUC->drawRigidBody(Body);

		DUC->setUpLighting(Vec3(), 0.3*Vec3(1, 1, 1), 100, 0.9*Vec3(0, 0, 1));
		BodyRot = rigidBodies[2].r.getRotMat();
		BodyTrans.initTranslation(rigidBodies[2].pos.x, rigidBodies[2].pos.y, rigidBodies[2].pos.z);
		BodyScale.initScaling(rigidBodies[2].size.x, rigidBodies[2].size.y, rigidBodies[2].size.z);
		Body = BodyScale* BodyRot * BodyTrans;
		DUC->drawRigidBody(Body);

		DUC->setUpLighting(Vec3(), 0.3*Vec3(1, 1, 1), 100, 0.9*Vec3(1, 1, 0));
		BodyRot = rigidBodies[3].r.getRotMat();
		BodyTrans.initTranslation(rigidBodies[3].pos.x, rigidBodies[3].pos.y, rigidBodies[3].pos.z);
		BodyScale.initScaling(rigidBodies[3].size.x, rigidBodies[3].size.y, rigidBodies[3].size.z);
		Body = BodyScale* BodyRot * BodyTrans;
		DUC->drawRigidBody(Body);

		//Boden
		DUC->setUpLighting(Vec3(), 0.3*Vec3(1, 1, 1), 100, 0.9*Vec3(1, 1, 1));
		BodyRot.initRotationXYZ(0, 0, 0);
		BodyTrans.initTranslation(0, -0.6f, 0);
		BodyScale.initScaling(5.0f, 0.1f, 5.0f);
		Body = BodyScale * BodyRot * BodyTrans;
		DUC->drawRigidBody(Body);

		break;
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
		L_tmp = { 0,0,0 };

		if (borders)
		{
			checkForBorders(0);
		}

		rigidBodies[0].pos = rigidBodies[0].pos + 2 * rigidBodies[0].lVel;
		rigidBodies[0].lVel = rigidBodies[0].lVel + 2 * f / 2;

		L_tmp = L_tmp + 2 * q;

		I_inv_tmp;
		I_inv_tmp.initId();
		I_inv_tmp.value[0][0] = (1.0f / 12.0f * rigidBodies[0].m*(pow(rigidBodies[0].size.y, 2) + pow(rigidBodies[0].size.z, 2)));
		I_inv_tmp.value[1][1] = (1.0f / 12.0f * rigidBodies[0].m*(pow(rigidBodies[0].size.x, 2) + pow(rigidBodies[0].size.z, 2)));
		I_inv_tmp.value[2][2] = (1.0f / 12.0f * rigidBodies[0].m*(pow(rigidBodies[0].size.x, 2) + pow(rigidBodies[0].size.y, 2)));
		I_inv_tmp.value[3][3] = 0;
		I_inv_tmp.inverse();

		newI_inv;
		newI_inv = rigidBodies[0].r.getRotMat().operator*(I_inv_tmp.operator*(rigidBodies[0].r.getRotMat().inverse()));

		rigidBodies[0].wVel = newI_inv.operator*(L_tmp);

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
		rigidBodies[0].lVel = Vec3(0.25f, 0, 0);
		rigidBodies[0].I_inv.initId();
		rigidBodies[0].I_inv.value[0][0] = (1.0f / 12.0f * rigidBodies[0].m *(pow(rigidBodies[0].size.y, 2) + pow(rigidBodies[0].size.z, 2)));
		rigidBodies[0].I_inv.value[1][1] = (1.0f / 12.0f * rigidBodies[0].m *(pow(rigidBodies[0].size.x, 2) + pow(rigidBodies[0].size.z, 2)));
		rigidBodies[0].I_inv.value[2][2] = (1.0f / 12.0f * rigidBodies[0].m *(pow(rigidBodies[0].size.x, 2) + pow(rigidBodies[0].size.y, 2)));
		rigidBodies[0].I_inv.value[3][3] = 0;
		rigidBodies[0].I_inv.inverse();
		L_tmp = Vec3(0, 0, 0);
		break;
	case 2:
		rigidBodies.clear();

		addRigidBody(Vec3(0.25f, 0, 0), Vec3(0.25f, 0.25f, 0.25f), 1.5f); 
		rot.initRotationX(0); 
		setOrientationOf(0, rotMatToQuat(rot)); 
		rigidBodies[0].lVel = Vec3(-0.25f, 0, 0);
		rigidBodies[0].I_inv.initId();
		rigidBodies[0].I_inv.value[0][0] = (1.0f / 12.0f * rigidBodies[0].m*(pow(rigidBodies[0].size.y, 2) + pow(rigidBodies[0].size.z, 2)));
		rigidBodies[0].I_inv.value[1][1] = (1.0f / 12.0f * rigidBodies[0].m*(pow(rigidBodies[0].size.x, 2) + pow(rigidBodies[0].size.z, 2)));
		rigidBodies[0].I_inv.value[2][2] = (1.0f / 12.0f * rigidBodies[0].m*(pow(rigidBodies[0].size.x, 2) + pow(rigidBodies[0].size.y, 2)));
		rigidBodies[0].I_inv.value[3][3] = 0;
		rigidBodies[0].I_inv.inverse();

		addRigidBody(Vec3(-0.25, 0, 0), Vec3(0.25f, 0.25f, 0.25f), 2.5f); 
		rot.initRotationXYZ(45, 45, 0); 
		setOrientationOf(1, rotMatToQuat(rot));
		rigidBodies[1].lVel = Vec3(0.25f, 0, 0);
		rigidBodies[1].I_inv.value[0][0] = (1.0f / 12.0f * rigidBodies[0].m*(pow(rigidBodies[0].size.y, 2) + pow(rigidBodies[0].size.z, 2)));
		rigidBodies[1].I_inv.value[1][1] = (1.0f / 12.0f * rigidBodies[0].m*(pow(rigidBodies[0].size.x, 2) + pow(rigidBodies[0].size.z, 2)));
		rigidBodies[1].I_inv.value[2][2] = (1.0f / 12.0f * rigidBodies[0].m*(pow(rigidBodies[0].size.x, 2) + pow(rigidBodies[0].size.y, 2)));
		rigidBodies[1].I_inv.value[3][3] = 0;
		rigidBodies[1].I_inv.inverse();;

		break;
	case 3:
		rigidBodies.clear();

		addRigidBody(Vec3(0.25f, 0, 0), Vec3(0.25f, 0.25f, 0.25f), 1.5f);
		rot.initRotationX(0);
		setOrientationOf(0, rotMatToQuat(rot));
		rigidBodies[0].lVel = Vec3(0, 0, 0);
		rigidBodies[0].I_inv.initId();
		rigidBodies[0].I_inv.value[0][0] = (1.0f / 12.0f * rigidBodies[0].m*(pow(rigidBodies[0].size.y, 2) + pow(rigidBodies[0].size.z, 2)));
		rigidBodies[0].I_inv.value[1][1] = (1.0f / 12.0f * rigidBodies[0].m*(pow(rigidBodies[0].size.x, 2) + pow(rigidBodies[0].size.z, 2)));
		rigidBodies[0].I_inv.value[2][2] = (1.0f / 12.0f * rigidBodies[0].m*(pow(rigidBodies[0].size.x, 2) + pow(rigidBodies[0].size.y, 2)));
		rigidBodies[0].I_inv.value[3][3] = 0;
		rigidBodies[0].I_inv.inverse();

		addRigidBody(Vec3(0.20f, 0.26f, 0), Vec3(0.25f, 0.25f, 0.25f), 1.5f);
		rot.initRotationXYZ(45,45,0);
		setOrientationOf(1, rotMatToQuat(rot));
		rigidBodies[1].lVel = Vec3(0, 0, 0);
		rigidBodies[1].I_inv.initId();
		rigidBodies[1].I_inv.value[0][0] = (1.0f / 12.0f * rigidBodies[1].m*(pow(rigidBodies[1].size.y, 2) + pow(rigidBodies[1].size.z, 2)));
		rigidBodies[1].I_inv.value[1][1] = (1.0f / 12.0f * rigidBodies[1].m*(pow(rigidBodies[1].size.x, 2) + pow(rigidBodies[1].size.z, 2)));
		rigidBodies[1].I_inv.value[2][2] = (1.0f / 12.0f * rigidBodies[1].m*(pow(rigidBodies[1].size.x, 2) + pow(rigidBodies[1].size.y, 2)));
		rigidBodies[1].I_inv.value[3][3] = 0;
		rigidBodies[1].I_inv.inverse();

		addRigidBody(Vec3(-0.15f, -0.26f, 0), Vec3(0.25f, 0.25f, 0.25f), 1.5f);
		rot.initRotationX(0);
		setOrientationOf(2, rotMatToQuat(rot));
		rigidBodies[2].lVel = Vec3(0.25f, 0, 0);
		rigidBodies[2].I_inv.initId();
		rigidBodies[2].I_inv.value[0][0] = (1.0f / 12.0f * rigidBodies[2].m*(pow(rigidBodies[2].size.y, 2) + pow(rigidBodies[2].size.z, 2)));
		rigidBodies[2].I_inv.value[1][1] = (1.0f / 12.0f * rigidBodies[2].m*(pow(rigidBodies[2].size.x, 2) + pow(rigidBodies[2].size.z, 2)));
		rigidBodies[2].I_inv.value[2][2] = (1.0f / 12.0f * rigidBodies[2].m*(pow(rigidBodies[2].size.x, 2) + pow(rigidBodies[2].size.y, 2)));
		rigidBodies[2].I_inv.value[3][3] = 0;
		rigidBodies[2].I_inv.inverse();

		addRigidBody(Vec3(0.25f, -0.26f, 0.20f), Vec3(0.25f, 0.25f, 0.25f), 1.5f);
		rot.initRotationX(0);
		setOrientationOf(3, rotMatToQuat(rot));
		rigidBodies[3].lVel = Vec3(-0.25f, 0, 0);
		rigidBodies[3].I_inv.initId();
		rigidBodies[3].I_inv.value[0][0] = (1.0f / 12.0f * rigidBodies[3].m*(pow(rigidBodies[3].size.y, 2) + pow(rigidBodies[3].size.z, 2)));
		rigidBodies[3].I_inv.value[1][1] = (1.0f / 12.0f * rigidBodies[3].m*(pow(rigidBodies[3].size.x, 2) + pow(rigidBodies[3].size.z, 2)));
		rigidBodies[3].I_inv.value[2][2] = (1.0f / 12.0f * rigidBodies[3].m*(pow(rigidBodies[3].size.x, 2) + pow(rigidBodies[3].size.y, 2)));
		rigidBodies[3].I_inv.value[3][3] = 0;
		rigidBodies[3].I_inv.inverse();
		break;
	}
}

void RigidBodySystemSimulator::externalForcesCalculations(float timeElapsed)
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

void RigidBodySystemSimulator::simulateTimestep(float timeStep)
{
	switch (m_iTestCase)
	{
	case 0: break;
	case 1:
		f = m_externalForce; //+ Vec3(0, -9.81*rigidBodies[0].m, 0);
		q = cross(rigidBodies[0].pos, f);

		rigidBodies[0].pos += timeStep*rigidBodies[0].lVel;
		if (borders)
		{
			checkForBorders(0);
		}
		rigidBodies[0].lVel += timeStep*f / rigidBodies[0].m;

		rigidBodies[0].r = rigidBodies[0].r + timeStep / 2 * Quat(0, rigidBodies[0].wVel.x, rigidBodies[0].wVel.y, rigidBodies[0].wVel.z)*rigidBodies[0].r;
		L_tmp += timeStep*q; 

		//I_inv_tmp.initId();
		//I_inv_tmp.value[0][0] = (1.0f / 12.0f * rigidBodies[0].m*(pow(rigidBodies[0].size.y, 2) + pow(rigidBodies[0].size.z, 2)));
		//I_inv_tmp.value[1][1] = (1.0f / 12.0f * rigidBodies[0].m*(pow(rigidBodies[0].size.x, 2) + pow(rigidBodies[0].size.z, 2)));
		//I_inv_tmp.value[2][2] = (1.0f / 12.0f * rigidBodies[0].m*(pow(rigidBodies[0].size.x, 2) + pow(rigidBodies[0].size.y, 2)));
		//I_inv_tmp.value[3][3] = 0;
		//I_inv_tmp.inverse();

		rigidBodies[0].I_inv = rigidBodies[0].r.getRotMat().operator*(rigidBodies[0].I_inv.operator*(rigidBodies[0].r.getRotMat().inverse()));

		rigidBodies[0].wVel = newI_inv.operator*(L_tmp);


		worldPos = rigidBodies[0].pos + rigidBodies[0].r.getRotMat().operator*(rigidBodies[0].pos);

		worldVel = rigidBodies[0].lVel + cross(rigidBodies[0].wVel, rigidBodies[0].pos);
		
		break;
	case 2:
		//Calculate All Movement first, then check for collisison 
		for (int i = 0; i < getNumberOfRigidBodies(); i++)
		{
			//External Forces
			f = m_externalForce;
			q = cross(rigidBodies[i].pos, f); 

			//Euler Step
			rigidBodies[i].pos += timeStep*rigidBodies[i].lVel;
			rigidBodies[i].lVel += timeStep*f / rigidBodies[i].m;

			//Rotation & Stuff
			rigidBodies[i].r = rigidBodies[i].r + timeStep / 2 * Quat(0, rigidBodies[i].wVel.x, rigidBodies[i].wVel.y, rigidBodies[i].wVel.z)*rigidBodies[i].r;

			rigidBodies[i].L += timeStep*q;
			rigidBodies[i].I_inv = rigidBodies[i].r.getRotMat() * rigidBodies[i].I_inv * rigidBodies[i].r.getRotMat().inverse();
			rigidBodies[i].wVel = rigidBodies[i].I_inv * rigidBodies[i].L;
		}

		checkForCollision();
		break;
	case 3:
		//Calculate All Movement first, then check for collisison 
		for (int i = 0; i < getNumberOfRigidBodies(); i++)
		{
			//External Forces
			f = m_externalForce + Vec3(0, -9.81*rigidBodies[i].m, 0);
			q = cross(rigidBodies[i].pos, f);

			//Euler Step
			rigidBodies[i].pos += timeStep*rigidBodies[i].lVel;

			rigidBodies[i].lVel += timeStep*f / rigidBodies[i].m;

			//Rotation & Stuff
			rigidBodies[i].r = rigidBodies[i].r + timeStep / 2 * Quat(0, rigidBodies[i].wVel.x, rigidBodies[i].wVel.y, rigidBodies[i].wVel.z)*rigidBodies[i].r;

			rigidBodies[i].L += timeStep*q;
			rigidBodies[i].I_inv = rigidBodies[i].r.getRotMat() * rigidBodies[i].I_inv * rigidBodies[i].r.getRotMat().inverse();
			rigidBodies[i].wVel = rigidBodies[i].I_inv * rigidBodies[i].L;
		}
		checkForWallCollision();

		checkForCollision();
		
		break;
	default: break;
	}
	
}

bool RigidBodySystemSimulator::checkForWallCollision()
{
	CollisionInfo col;
	Mat4 wall, wallR, wallT, wallS;
	Mat4 BodyA, bodyR, bodyT, bodyS;

	wallR.initRotationX(180);
	wallR.initTranslation(0, -0.6f, 0); 
	wallS.initScaling(5.0f, 0.1f, 5.0f);
	wall = wallS * wallR * wallT; 

	cout << "Collision with Wall \n";

	for (int i = 0; i < rigidBodies.size(); i++)
	{
		bodyR = rigidBodies[i].r.getRotMat();
		bodyT.initTranslation(rigidBodies[i].pos.x, rigidBodies[i].pos.y, rigidBodies[i].pos.z);
		bodyS.initScaling(rigidBodies[i].size.x, rigidBodies[i].size.y, rigidBodies[i].size.z);
		BodyA = bodyS* bodyR * bodyT;

		col = checkCollisionSAT(BodyA, wall);

		if (col.isValid)
		{
			cout << "Collision is Valid \n";
			calculateImpulseWall(i, col);
		}
	}

	return true;
}

void RigidBodySystemSimulator::calculateImpulseWall(int b, CollisionInfo col)
{
	float c = 0.1f;
	Vec3 z = Vec3(0, -0.6f, 0);
	float v_rel = dot(col.normalWorld, (z - rigidBodies[b].pos));

	//if collision
	if (v_rel < 0)
	{
		float J = -(1 + c)* dot((z - rigidBodies[b].lVel), col.normalWorld);
		J = J / (1 / rigidBodies[b].m) +
			dot((cross(rigidBodies[b].I_inv * (cross(rigidBodies[b].pos, col.normalWorld)), rigidBodies[b].pos)), col.normalWorld);

		rigidBodies[b].lVel = rigidBodies[b].lVel + J*col.normalWorld / rigidBodies[b].m;
		rigidBodies[b].L = rigidBodies[b].L + cross(rigidBodies[b].pos, J*col.normalWorld);
		cout << J << "\n";
	}

}

bool RigidBodySystemSimulator::checkForCollision()
{
	CollisionInfo col; 
	Mat4 BodyA, BodyB;
	Mat4 bodyR, bodyT, bodyS;

	for (int i = 0; i < rigidBodies.size(); i++)
	{
		bodyR = rigidBodies[i].r.getRotMat();
		bodyT.initTranslation(rigidBodies[i].pos.x, rigidBodies[i].pos.y, rigidBodies[i].pos.z);
		bodyS.initScaling(rigidBodies[i].size.x, rigidBodies[i].size.y, rigidBodies[i].size.z);
		BodyA = bodyS* bodyR * bodyT;

		for (int j = 0; j < rigidBodies.size(); j++)
		{
			if (i == j)
			{
				break;
			}

			bodyR = rigidBodies[j].r.getRotMat();
			bodyT.initTranslation(rigidBodies[j].pos.x, rigidBodies[j].pos.y, rigidBodies[j].pos.z);
			bodyS.initScaling(rigidBodies[j].size.x, rigidBodies[j].size.y, rigidBodies[j].size.z);
			BodyB = bodyS* bodyR * bodyT;

			col = checkCollisionSAT(BodyA, BodyB);

			if (col.isValid)
			{
				calculateImpulse(i, j, col);
			}
		}
	}

	return true;
}

void RigidBodySystemSimulator::calculateImpulse(int a, int b, CollisionInfo coll)
{
	float c = 0.1f;
	float v_rel;
	Vec3 n = (rigidBodies[a].lVel - rigidBodies[b].lVel);
	v_rel = dot(coll.normalWorld, n);
	
	//if collision
	if (v_rel < 0)
	{
		float J = -(1 + c)* dot((rigidBodies[a].lVel - rigidBodies[b].lVel), coll.normalWorld);
		J = J / (1 / rigidBodies[a].m) + (1 / rigidBodies[b].m) +
			dot(cross(rigidBodies[a].I_inv* (cross(rigidBodies[a].pos, coll.normalWorld)), rigidBodies[a].pos) +
			   (cross(rigidBodies[b].I_inv * (cross(rigidBodies[b].pos, coll.normalWorld)), rigidBodies[b].pos)),
			    coll.normalWorld);
		
		rigidBodies[a].lVel = rigidBodies[a].lVel + J*coll.normalWorld / rigidBodies[a].m;
		rigidBodies[b].lVel = rigidBodies[b].lVel - J*coll.normalWorld / rigidBodies[b].m;

		rigidBodies[a].L = rigidBodies[a].L + cross(rigidBodies[a].pos, J*coll.normalWorld);
		rigidBodies[b].L = rigidBodies[b].L - cross(rigidBodies[b].pos, J*coll.normalWorld);
	}
}

void RigidBodySystemSimulator::checkForBorders(int rgdNum)
{
	int i = rgdNum;
	if (rigidBodies[i].pos.X > 0.5f)
	{
		rigidBodies[i].pos.X = 0.5f;

	}
	if (rigidBodies[i].pos.X < -0.5f)
	{
		rigidBodies[i].pos.X = -0.5f;
	}

	if (rigidBodies[i].pos.Y > 0.5f)
	{
		rigidBodies[i].pos.Y = 0.5f;

	}
	if (rigidBodies[i].pos.Y < -0.5f)
	{
		rigidBodies[i].pos.Y = -0.5f;
	}

	if (rigidBodies[i].pos.Z > 0.5f)
	{
		rigidBodies[i].pos.Z = 0.5f;

	}
	if (rigidBodies[i].pos.Z < -0.5f)
	{
		rigidBodies[i].pos.Z = -0.5f;
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
	rigidB tmp = { position, zero, zero, ori, size, mass, zero};
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