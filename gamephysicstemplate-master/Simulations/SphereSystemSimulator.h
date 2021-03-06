#ifndef SPHSYSTEMSIMULATOR_h
#define SPHSYSTEMSIMULATOR_h
#include "Simulator.h"
//#include "spheresystem.h", add your sphere system header file

#define NAIVEACC 0
#define GRIDACC 1

class SphereSystemSimulator:public Simulator{
public:
	// Construtors
	SphereSystemSimulator();
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

	//My Functions 
	void detectBoxCollision(int i);
	void detectCollisionSimple(int i);
	void detectCollisionGrid(int i, int j);
	void makeGrid();

	struct grid {
		int num_sphere; 
		vector<int> spheres; 
		Vec3 borders;
	};

	struct sphere {
		Vec3 pos;
		Vec3 vel;
		Vec3 force;
	};
	
protected:
	// Attributes
	Vec3 m_externalForce;
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;
	float m_fMass;
	float m_fRadius;
	float m_fForceScaling;
	float m_fDamping;
	int   m_iNumSpheres;
	
	int   m_iKernel; // index of the m_Kernels[5], more detials in SphereSystemSimulator.cpp
	static std::function<float(float)> m_Kernels[5];
	
	int   m_iAccelerator; 

	
	vector<sphere> m_sSphere;
	Vec3 m_fForce;
	Vec3 m_fAcceleration;
	// switch between NAIVEACC and GRIDACC, (optionally, KDTREEACC, 2)
	
	//SphereSystem * m_pSphereSystem; // add your own sphere system member!
	// for Demo 3 only:
	// you will need multiple SphereSystem objects to do comparisons in Demo 3
	// m_iAccelerator should be ignored.
	// SphereSystem * m_pSphereSystemGrid; 

};

#endif