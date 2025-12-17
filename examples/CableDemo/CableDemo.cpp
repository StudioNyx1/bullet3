#include "../OpenGLWindow/GLInstanceGraphicsShape.h"
#include "Bullet3Common/b3FileUtils.h"

#include "../Utils/b3BulletDefaultFileIO.h"
#include "../Importers/ImportObjDemo/LoadMeshFromObj.h"
#include "../CommonInterfaces/CommonRigidBodyBase.h"

#include "btBulletDynamicsCommon.h"
#include "BulletSoftBody/btSoftRigidDynamicsWorld.h"

#include "LinearMath/btQuickprof.h"
#include "LinearMath/btIDebugDraw.h"

#include <stdio.h>  //printf debugging
#include "LinearMath/btConvexHull.h"
#include "BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h"
#include "BulletSoftBody/btSoftBodyHelpers.h"

#include "CableDemo.h"
#include "GL_ShapeDrawer.h"
#include "../CommonInterfaces/CommonParameterInterface.h"

#include "LinearMath/btAlignedObjectArray.h"
#include "BulletSoftBody/btSoftBody.h"
#include "BulletCable/btCable.h"

#include "BulletCollision/GImpact/btGImpactShape.h"

#include "BulletDynamics/MLCPSolvers/btDantzigSolver.h"
#include "BulletDynamics/MLCPSolvers/btMLCPSolver.h"
#include <BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolverMt.h>
#include "BulletCollision/CollisionDispatch/btCollisionDispatcherMt.h"

#include <iostream>
#include <chrono>
#include <deque>
#include <BulletCollision/CollisionDispatch/btInternalEdgeUtility.h>
#include <BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h>
#include <BulletCollision/CollisionShapes/btBvhTriangleMeshShape.h>
#include <BulletCollision/CollisionShapes/btSdfCollisionShape.h>

// #include "BunnyMesh.h"

class btBroadphaseInterface;
class btCollisionShape;
class btOverlappingPairCache;
class btCollisionDispatcher;
class btConstraintSolver;
struct btCollisionAlgorithmCreateFunc;
class btDefaultCollisionConfiguration;

///collisions between two btSoftBody's
class btSoftSoftCollisionAlgorithm;

///collisions between a btSoftBody and a btRigidBody
class btSoftRididCollisionAlgorithm;
class btSoftRigidDynamicsWorld;


struct GlobalData
{
	btScalar substepsCount{4};
};


struct StabilityData
{
	// Rigidbodies
	btScalar A_mass{100.0};
	btScalar B_mass{0.0};
	btScalar C_mass{100.0};
	btScalar Ground_offset{0.4};

	// Tweaks
	btScalar A_massRatio{0.0};
	btScalar A_MassImpact{0.0};
	btScalar D_massRatio{0.0};

	// Cable
	btScalar Cable_resolution{5.0};
	btScalar Cable_iteration{100.0};
	btScalar Cable_linearMass{1.0};
	btScalar Cable_length{0.4};
	btScalar Cable_targetLength{1.5};
	btScalar Cable_growSpeed{0.5};
	bool Cable_LRA{false};
	bool Cable_AnchorPlacement{true};
	btScalar Cable_DistanceMode{0};
	btScalar Cable_massRatioActivationThreshold{0.0};

	// Used to reset to test init values
	bool AutoResetTensionTest{false};
};

StabilityData StabilityTensionData{};
StabilityData StabilityA18Data{};
GlobalData globals{};


struct CableStepController
{
	bool   enabled = false;
	bool   sessionActive = false;
};

class CableDemo : public CommonRigidBodyBase
{
public:
	btCable* m_cable = nullptr; // set this to the cable you want to inspect
	CableStepController m_cableStep;

	StabilityData* m_stabilityData{nullptr};

	void toggleCableIterStepMode()
	{
		m_cableStep.enabled = !m_cableStep.enabled;
		if (!m_cableStep.enabled && m_cableStep.sessionActive && m_cable)
		{
			m_cable->endIterativeSolve();
			m_dynamicsWorld->EndStepSimulationOneCable();
			m_cableStep.sessionActive = false;
		}
	}

	void beginCableIterSession()
	{
		if (!m_cable || !m_cableStep.enabled || m_cableStep.sessionActive)
			return;

		m_dynamicsWorld->stepSimulationOneCable(_deltaTime, 1, _deltaTime, m_cable);
		m_cable->beginIterativeSolve();
		m_cableStep.sessionActive = true;
	}

	void stepCableOneIteration()
	{
		if (!m_cable || !m_cableStep.enabled)
			return;

		if (!m_cableStep.sessionActive)
			beginCableIterSession();

		bool more = m_cable->stepOneIteration();
		// After each iteration, request a redrawing to see the intermediate result
		renderSingleFrame();

		if (!more)
		{
			m_cable->endIterativeSolve();
			m_dynamicsWorld->EndStepSimulationOneCable();
			m_cableStep.sessionActive = false;
		}
	}

	void renderSingleFrame() {
		// draws
		for (int i = 0; i < m_dynamicsWorld->getNumCollisionObjects(); ++i)
		{
			btCollisionObject* co = m_dynamicsWorld->getCollisionObjectArray().at(i);
			btCollisionShape* cs = co->getCollisionShape();
			btVector3 AabbMin, AabbMax;
			cs->getAabb(co->getWorldTransform(), AabbMin, AabbMax);

			// m_dynamicsWorld->getDebugDrawer()->drawBox(AabbMin, AabbMax, btVector3(0,1,1));
			// m_dynamicsWorld->getDebugDrawer()->drawSphere(co->getWorldTransform().getOrigin(), 0.05, btVector3(1, 0, 1));
		}
	}
	
	// In your main simulation tick, pause normal stepping when in cable-step mode.
	void tick(btScalar dt, int subStep, btScalar fixedTimeStep)
	{
		// Normal path
		{
			if (m_cableStep.enabled)
			{
				// Don't run the full physics step while visually stepping iterations.
				// Only draw the current state.
				renderSingleFrame();
				return;
			}

			current_ticks = clock();

			// Normal simulation path
			m_dynamicsWorld->stepSimulation(dt, subStep, fixedTimeStep);

			delta_ticks = clock() - current_ticks;
			renderSingleFrame();
		}

		//// Benchmark
		//{
		//	// Warm-up step to avoid cold start overhead
		//	m_dynamicsWorld->stepSimulation(dt, subStep, fixedTimeStep);
		//	
		//	// Simulate many steps to smooth out timing noise
		//	const int iterations = 10;

		//	btClock timer;
		//	timer.reset();
		//	for (int i = 0; i < iterations; i++)
		//	{
		//		m_dynamicsWorld->stepSimulation(dt, subStep, fixedTimeStep);
		//	}

		//	const btScalar ms = timer.getTimeMilliseconds();
		//	const btScalar deltaFrame = ms / iterations;
		//	cumulatedDelta += deltaFrame;
		//	const btScalar deltaAverage = cumulatedDelta / tickCount;
		//	std::cout << "(Tick = " << tickCount << ") --> substeps=" << subStep
		//			  << " Time=" << deltaFrame << " ms."
		//			  << " Average Time=" << deltaAverage << " ms."
		//		      << std::endl;
		//	tickCount++;

		//	renderSingleFrame();
		//}
	}

	// Benchmark
	int tickCount = 1;
	btScalar cumulatedDelta = 0;
	
	btScalar posX;
	btScalar posY;
	btScalar margin;

	int substepSolver;  // User controlled in globals.substepsCount
	btAlignedObjectArray<btSoftSoftCollisionAlgorithm*> m_SoftSoftCollisionAlgorithms;

	btAlignedObjectArray<btSoftRididCollisionAlgorithm*> m_SoftRigidCollisionAlgorithms;

	btSoftBodyWorldInfo m_softBodyWorldInfo;

	bool m_autocam;
	bool m_cutting;
	bool m_raycast;
	btScalar m_animtime;
	btClock m_clock;
	int m_lastmousepos[2];
	btVector3 m_impact;
	btSoftBody::sRayCast m_results;
	btSoftBody::Node* m_node;
	btVector3 m_goal;
	bool m_drag;

	//keep the collision shapes, for deletion/cleanup
	btAlignedObjectArray<btCollisionShape*> m_collisionShapes;

	btBroadphaseInterface* m_broadphase;

	//btCollisionDispatcherMt* m_dispatcher;
	btCollisionDispatcher* m_dispatcher;

	btConstraintSolver* m_solver;

	btCollisionAlgorithmCreateFunc* m_boxBoxCF;

	btDefaultCollisionConfiguration* m_collisionConfiguration;

private:
	int m_currentDemoIndex;
	bool m_applyForceOnRigidbody;
	bool m_moveBody;
	bool m_attachLock;
	bool m_printAnchorData;
	bool m_close = false;
	bool m_grow = false;
	bool m_growWithSpeedAndDistance = false;
	bool m_shrink = false;
	btScalar speed = 0;

	btScalar _deltaTime;

	int nbDelta = 0;
	int totalDelta = 0;

	btScalar growthSpeed = 0.5;
	btScalar growthDistance = 6;
	btScalar initialCableLength = 0;
	// Calculate the time in seconds
	std::chrono::steady_clock::duration theoricalTime;

	bool m_printFPS;
	clock_t current_ticks, delta_ticks;
	clock_t m_fps = 0;

	btVector3 m_cameraPosition;
	btScalar m_cameraDistance;
	btScalar m_cameraPitch;
	btScalar m_cameraYaw;
	int nbFrame = 0;
	int nbStep = 0;

	using Clock = std::chrono::high_resolution_clock;

	std::chrono::steady_clock::time_point startingTime{Clock::now()};

	btVector3 m_waterCurrent;

public:
	void initPhysics();

	void exitPhysics();

	virtual void resetCamera()
	{
		//@todo depends on current_demo?
		float targetPos[3] = {(float)m_cameraPosition.x(), (float)m_cameraPosition.y(), (float)m_cameraPosition.z()};
		m_guiHelper->resetCamera(m_cameraDistance, m_cameraPitch, m_cameraYaw, targetPos[0], targetPos[1], targetPos[2]);
	}

	void SetCameraPosition(btVector3 cameraPosition)
	{
		m_cameraPosition = cameraPosition;
	}

	void SetCameraRotation(btScalar distance, btScalar pitch, btScalar yaw)
	{
		m_cameraDistance = distance;
		m_cameraPitch = pitch;
		m_cameraYaw = yaw;
	}

	CableDemo(struct GUIHelperInterface* helper) : CommonRigidBodyBase(helper), m_drag(false)
	{
		m_applyForceOnRigidbody = false;
		m_attachLock = false;
		m_moveBody = false;
		m_printFPS = false;
		m_printAnchorData = false;
		m_cameraPosition = btVector3(0, 3, 0);
		m_cameraDistance = 5;
		m_cameraPitch = 0;
		m_cameraYaw = 180;

		m_waterCurrent = btVector3(1, 0, 0);
	}

	virtual ~CableDemo()
	{
		btAssert(m_dynamicsWorld == 0);
	}

	//virtual void clientMoveAndDisplay();

	//virtual void displayCallback();

	void createStack(btCollisionShape* boxShape, float halfCubeSize, int size, float zPos);

	virtual void setDrawClusters(bool drawClusters);

	virtual const btSoftRigidDynamicsWorld* getSoftDynamicsWorld() const
	{
		///just make it a btSoftRigidDynamicsWorld please
		///or we will add type checking
		return (btSoftRigidDynamicsWorld*)m_dynamicsWorld;
	}

	virtual btSoftRigidDynamicsWorld* getSoftDynamicsWorld()
	{
		///just make it a btSoftRigidDynamicsWorld please
		///or we will add type checking
		return (btSoftRigidDynamicsWorld*)m_dynamicsWorld;
	}

	//
	//void	clientResetScene();
	void renderme();

	/*
	* 4 -> Move body (=20)
	* 6 -> Move body (=-20)
	* 9 -> Grow cable with speed and distance
	* A -> apply force on 
	* E -> Increase water current (only used on demo 21)
	* T -> Print cable tension
	* O -> Increase grow distance (+=0.5)
	* P -> Decrease grow speed (+=0.5)
	* D -> print cable distance
	* F -> Print FPS
	* H -> Enable "Grow" and disable "shrink"
	* J -> Disable "Grow" and enable "shrink"
	* K -> Disable "Grow" and disable "shrink"
	* L -> Decrease grow distance (-=0.5)
	* M -> Decrease grow speed (-=0.5
	* X -> Move body (+=1)
	* C -> Move body (-=1)
	* N -> attach Lock
	*/

	bool keyboardCallback(int key, int state) override
	{
		if (key == 'a' && state)
		{
			m_applyForceOnRigidbody = !m_applyForceOnRigidbody;
		}

		if (key == 'd' && state)
		{
			PrintDistance_DemoCableForce();
		}

		if (key == 'h' && state)
		{
			m_grow = true;
			m_shrink = false;
		}

		if (key == 'j' && state)
		{
			m_grow = false;
			m_shrink = true;
		}

		if (key == 'k' && state)
		{
			m_grow = false;
			m_shrink = false;
		}

		if (key == 'e' && state)
		{
			m_waterCurrent += btVector3(1, 0, 0);
			b3Printf("Water current length : %f ", m_waterCurrent.norm());
		}

		if (key == '9' && state)
		{
			GrowsWithSpeedAndDistance(growthSpeed, growthDistance);
		}

		if (key == 'o' && state)
		{
			ChangeAndPrintGrowDistance(0.5);
		}

		if (key == 'l' && state)
		{
			ChangeAndPrintGrowDistance(-0.5);
		}

		if (key == 'p' && state)
		{
			ChangeAndPrintGrowSpeed(0.5);
		}

		if (key == 'm' && state)
		{
			ChangeAndPrintGrowSpeed(-0.5);
		}

		if (key == 'x' && state)
		{
			m_moveBody = true;
			if (speed >= 0)
				speed += 1;
			else
				speed = 0.0;
		}

		if (key == 'c' && state)
		{
			m_moveBody = true;
			if (speed > 0)
				speed = 0;
			else
				speed -= 1;
		}

		if (key == '4' && state)
		{
			m_moveBody = true;
			speed = 20;
		}

		if (key == '6' && state)
		{
			m_moveBody = true;
			speed = -20;
		}

		if (key == 'f' && state)
		{
			m_printFPS = !m_printFPS;
		}

		if (key == 'n' && state)
		{
			m_attachLock = true;
		}

		if (key == 't' && state)
		{
			m_printAnchorData = !m_printAnchorData;
		}

		if (key == 'y' && state) // toggle stepping mode
			toggleCableIterStepMode();
		else if (key == 'u' && state) // do one iteration
			stepCableOneIteration();

		return false;
	}

	void mouseFunc(int button, int state, int x, int y);

	void mouseMotionFunc(int x, int y);

	void Grows(float dt, bool test = true)
	{
		if (getSoftDynamicsWorld()->getSoftBodyArray().size() == 0) return;
		btCable* _cable = (btCable*)getSoftDynamicsWorld()->getSoftBodyArray()[0];
		if (test)
			_cable->WantedSpeed = 0.25;
		else
			_cable->WantedSpeed = 0;
	}

	void GrowsWithSpeedAndDistance(double speed, double distance, bool test = true)
	{
		if (getSoftDynamicsWorld()->getSoftBodyArray().size() == 0) return;

		btCable* _cable = (btCable*)getSoftDynamicsWorld()->getSoftBodyArray()[0];
		if (test)
		{
			initialCableLength = _cable->getRestLength();
			double theoricalTime_d = (distance - initialCableLength) / speed;
			nbDelta = 0;
			// Convert seconds to a duration
			auto duration = std::chrono::duration<double>(theoricalTime_d);
			theoricalTime = std::chrono::duration_cast<std::chrono::system_clock::duration>(duration);
			b3Printf("Change Cable Length with speed : %f , and target length %f. It should take %f secondes with a cable of %f", speed, distance, theoricalTime_d, initialCableLength);
			startingTime = Clock::now();
			m_growWithSpeedAndDistance = true;
			_cable->setWantedGrowSpeedAndDistance(speed, distance);
		}
		else
		{
			m_growWithSpeedAndDistance = false;
			_cable->WantedSpeed = 0;
		}
	}

	btVector3 computeNormalForce(int nodeIndex, btVector3 relativeVel, btVector3 link, btVector3 linkDir, float radius, float density, float coefDragNormal)
	{
		float lengthRelativeVel = relativeVel.norm();
		if (lengthRelativeVel < 0.001f) return btVector3(0, 0, 0);

		// See Morison's equation to calculate drag https://www.orcina.com/webhelp/OrcaFlex/Content/html/Morison%27sequation.htm
		btVector3 relativeVelNormalized = relativeVel.normalized();
		btVector3 u = relativeVelNormalized.cross(linkDir);
		btVector3 n = linkDir.cross(u);
		const float effectiveAreaNormal = std::max(0.0, std::min(n.dot(relativeVelNormalized), 1.0));
		const float areaLink = 2.0f * radius * link.norm();
		btVector3 normalForce = -0.5f * density * coefDragNormal * areaLink * effectiveAreaNormal * lengthRelativeVel * relativeVel;
		return normalForce;
	}

	void ApplyForcesNodes(btVector3 waterCurrent)
	{
		if (getSoftDynamicsWorld()->getSoftBodyArray().size() == 0) return;
		btCable* cable = (btCable*)getSoftDynamicsWorld()->getSoftBodyArray()[0];

		btScalar density = 1027;
		btScalar radius = cable->getCableData()->radius;
		btScalar coefficient = cable->getCableData()->normalDragCoefficient;

		for (int i = 1; i < cable->m_nodes.size() - 1; ++i)
		{
			btVector3 nodePosition = cable->m_nodes[i].m_x;
			btVector3 nodeVelocity = btVector3(cable->getNodeData()[i].velocity_x, cable->getNodeData()[i].velocity_y, cable->getNodeData()[i].velocity_z);

			const float waterFactor = 1.0f;

			// Initialize the forces/velocities
			btVector3 waterNormalForce = btVector3(0, 0, 0);

			// Water velocity
			btVector3 relativeVelWater = nodeVelocity - waterCurrent;

			// Previous Node
			btVector3 prevPos = cable->m_nodes[i - 1].m_x;

			// Next Node
			btVector3 nextPos = cable->m_nodes[i + 1].m_x;

			const btVector3 linkBefore = (nodePosition - prevPos);
			const btVector3 linkAfter = (nextPos - nodePosition);

			const btVector3 linkDirBefore = linkBefore.normalized();
			const btVector3 linkDirAfter = linkAfter.normalized();

			// Forces factors (before)
			float linkBeforeWaterFactor = 1.0;

			// Forces factors (after)
			float linkAfterWaterFactor = 1.0;

			waterNormalForce += computeNormalForce(i, relativeVelWater, linkBefore, linkDirBefore, radius, density, 1.2) * linkBeforeWaterFactor / 2.0f;
			waterNormalForce += computeNormalForce(i, relativeVelWater, linkAfter, linkDirAfter, radius, density, 1.2) * linkAfterWaterFactor / 2.0f;

			float addedMass = density * 3.1415926535f * (radius * radius) * linkBefore.length() * 0.5f;
			addedMass += density * 3.1415926535f * (radius * radius) * linkAfter.length() * 0.5f;

			//	cable->m_nodes[i].m_f += waterNormalForce;

			getSoftDynamicsWorld()->m_nodeForces[i].x = waterNormalForce.getX();
			getSoftDynamicsWorld()->m_nodeForces[i].y = waterNormalForce.getY();
			getSoftDynamicsWorld()->m_nodeForces[i].z = waterNormalForce.getZ();
			getSoftDynamicsWorld()->m_nodeForces[i].ma = addedMass;
		}
	}

	void SetCableGrowSpeed(btScalar value)
	{
		growthSpeed = value;
	}

	void SetCableTargetLength(btScalar value)
	{
		growthDistance = value;

		if (growthDistance < 0.0)
		{
			growthDistance = 0.0;
		}
	}

	void ChangeAndPrintGrowSpeed(btScalar addedValue)
	{
		growthSpeed += addedValue;

		b3Printf("New GrowthSpeed Value : %f ", growthSpeed);
	}

	void ChangeAndPrintGrowDistance(btScalar addedValue)
	{
		growthDistance += addedValue;

		if (growthDistance <= 0)
		{
			growthDistance = 0.0;
		}
		b3Printf("New GrowthDistance Value : %f ", growthDistance);
	}

	void Shrinks(float dt)
	{
		if (getSoftDynamicsWorld()->getSoftBodyArray().size() == 0) return;
		btCable* _cable = (btCable*)getSoftDynamicsWorld()->getSoftBodyArray()[0];

		_cable->WantedSpeed = -0.25;
	}

	GUIHelperInterface* getGUIHelper()
	{
		return m_guiHelper;
	}

	virtual void renderScene()
	{
		CommonRigidBodyBase::renderScene();
		btSoftRigidDynamicsWorld* softWorld = getSoftDynamicsWorld();

		for (int i = 0; i < softWorld->getSoftBodyArray().size(); i++)
		{
			btSoftBody* psb = (btSoftBody*)softWorld->getSoftBodyArray()[i];

			//if (softWorld->getDebugDrawer() && !(softWorld->getDebugDrawer()->getDebugMode() & (btIDebugDraw::DBG_DrawWireframe)))
			{
				btSoftBodyHelpers::DrawFrame(psb, softWorld->getDebugDrawer());
				btSoftBodyHelpers::Draw(psb, softWorld->getDebugDrawer(), softWorld->getDrawFlags());
			}
		}
	}

	void MoveBody()
	{
		btSoftRigidDynamicsWorld* softWorld = getSoftDynamicsWorld();

		// A18
		if (m_currentDemoIndex == 8)
		{
			//btCollisionObject* objKey = softWorld->getCollisionObjectArray().at(0);
			//btTransform tr = objKey->getWorldTransform();
			//tr.setOrigin(tr.getOrigin() + btVector3(speed*0.01, 0, 0));
			//objKey->setWorldTransform(tr);

			btCollisionObject* objKey = softWorld->getCollisionObjectArray().at(2);
			btRigidBody* obj = (btRigidBody*)objKey;
			obj->applyCentralForce(btVector3(speed * 500, 0, 0));
		}

		if (m_currentDemoIndex == 17)
		{
			//btCollisionObject* objKey = softWorld->getCollisionObjectArray().at(0);
			//btTransform tr = objKey->getWorldTransform();
			//tr.setOrigin(tr.getOrigin() + btVector3(speed*0.01, 0, 0));
			//objKey->setWorldTransform(tr);

			btCollisionObject* objKey = softWorld->getCollisionObjectArray().at(0);
			btRigidBody* obj = (btRigidBody*)objKey;
			obj->applyCentralForce(btVector3(speed * 100, 0, 0));
		}
		/*
		btCollisionObject* objKey = softWorld->getCollisionObjectArray().at(0);
		btRigidBody* obj = (btRigidBody*)objKey;
		//obj->applyCentralForce(btVector3(speed*500, 0, 0));
		obj->applyCentralForce(btVector3(0, 0, speed * -5000));
		*/
		//cout << obj->getLinearVelocity().length() * softWorld->getSolverInfo().m_timeStep << endl;

		// Test Corner
		/*
		btCollisionObject* objKey = softWorld->getCollisionObjectArray().at(3);
		btTransform tr = objKey->getWorldTransform();
		tr.setOrigin(tr.getOrigin() + btVector3(speed, 0, 0));
		objKey->setWorldTransform(tr);
		*/
		//btQuaternion t = tr.getRotation();
		//t.setY(t.getY() + speed);
		//tr.setRotation(t);
		//objKey->forceActivationState(ACTIVE_TAG);
	}

	void attachLock()
	{
		btCable* cable = (btCable*)getSoftDynamicsWorld()->getSoftBodyArray().at(0);
		btRigidBody* a18 = (btRigidBody*)getSoftDynamicsWorld()->getCollisionObjectArray().at(6);

		/*
		btRigidBody* lest = (btRigidBody*)softWorld.at(1);
		// Disable collision with the lest
		for (int i = 2; i < 8; i++)
		{
			auto temp = softWorld.at(i);
			auto x = temp->getCollisionShape();
			lest->setIgnoreCollisionCheck(temp,true);
		}

		// Set anchor on the locker
		//a18->clearForces();
		*/
		a18->setDamping(0.8, 0.8);
		cable->appendAnchor(1, a18);
		cable->appendAnchor(5, a18);
		cable->setUseCollision(false);
		cable->setUseLRA(false);
		m_attachLock = false;
	}

	void printAnchorData()
	{
		for (int i = 0; i < getSoftDynamicsWorld()->getSoftBodyArray().size(); ++i)
		{
			btCable* cable = (btCable*)getSoftDynamicsWorld()->getSoftBodyArray().at(i);
			for (int i = 0; i < cable->m_anchors.size(); i++)
			{
				auto anchor = cable->m_anchors.at(i);
				auto node = anchor.m_node;
				auto body = anchor.m_body;

				const btScalar distanceReal = cable->getLength();
				const btScalar distanceRest = cable->getRestLength();

				cout << "Anchor[" << i << "]-Node[" << node->index << "]:" << "\n\t"
					<< "Mass (Body):" << 1.0 / body->getInvMass() << "\n\t"
					<< "Last Tension: " << anchor.m_lastTension.length() << "\n\t"
					<< "Total Tension: " << anchor.m_totalTension.length() / substepSolver << "\n\t"
					<< "Distance Anchor-Node: " << node->m_x.distance(body->getWorldTransform() * anchor.m_local) << "\n\t"
					 << "Distance Cable (Rest): " << distanceRest << "\n\t"
					 << "Distance Cable (Real): " << distanceReal << "\n\t"
					 << "Delta Distance Cable (Real - Rest): " << distanceReal - distanceRest << "\n\t"
					<< "Mass Cable: " << cable->getTotalMass() << "\n\t"
					<< "Mass Node: " << 1.0 / node->m_im << "\n\t"
					<< "Substeps count: " << substepSolver << "\n\t"
					<< endl;
			}
		}
	}

	void stepSimulation(float deltaTime) override
	{
		_deltaTime = deltaTime;
		if (nbFrame % 10 == 0)
		{
			nbStep++;
		}
		nbFrame++;

		if (m_dynamicsWorld)
		{
			if (m_currentDemoIndex == 26)
			{
				btScalar margin = 0.05;

				btVector3 fromPos = btVector3(posX, 0, 0);
				fromPos.setY(9);
				btVector3 toPos = btVector3(posY, 0, 0);
				toPos.setY(9);

				if (fromPos.distance(toPos) > FLT_EPSILON)
				{
					for (int i = 0; i < 10; ++i)
					{
						int k = i > 5 ? 5 : i;
						btRigidBody* rb = btRigidBody::upcast(m_dynamicsWorld->getCollisionObjectArray()[k]);
						// if (k < 5) rb->setAngularVelocity(btVector3(0,0,1));

						fromPos.setY(9 - 2 * i);
						toPos.setY(11 - 2 * i);

						if (fromPos.distance(toPos) < FLT_EPSILON) continue;
						btVector3 rayDirection = (toPos - fromPos).normalized();
						m_dynamicsWorld->getDebugDrawer()->drawLine(toPos, fromPos, btVector3(1, 0, 0));

						btCollisionWorld::ClosestRayResultCallback result(fromPos, toPos);
						result.m_flags = btTriangleRaycastCallback::kF_FilterBackfaces;
						result.m_flags |= btTriangleRaycastCallback::kF_UseGjkConvexCastRaytest;
						m_dynamicsWorld->rayTestSingleWithMargin(btTransform(btQuaternion(), fromPos), btTransform(btQuaternion(), toPos),
							rb, rb->getCollisionShape(), rb->getWorldTransform(), result, margin);
						if (result.hasHit())
						{
							m_dynamicsWorld->getDebugDrawer()->drawSphere(result.m_hitPointWorld, margin, btVector3(0, 0, 1));
							m_dynamicsWorld->getDebugDrawer()->drawLine(result.m_hitPointWorld, result.m_hitPointWorld + result.m_hitNormalWorld * 0.2, btVector3(0, 0, 1));
						}
					}
				}
			}

			if (m_currentDemoIndex == 27)
			{
				struct ContactNotInfo
				{
					btVector3 normal;
				};
				
				btVector3 position = btVector3(posX, 0, 0);

				for (int i = 0; i < 6; ++i)
				{
					int k = i > 5 ? 5 : i;
					btRigidBody* rb = btRigidBody::upcast(m_dynamicsWorld->getCollisionObjectArray()[k]);

					position.setY((9 - posY) - 2 * i);
					m_dynamicsWorld->getDebugDrawer()->drawSphere(position, margin, btVector3(1, 0, 0));

					btSphereShape nodeShape(margin + FLT_EPSILON);
					btCollisionObject tmp;
					tmp.setCollisionShape(&nodeShape);
					tmp.setWorldTransform(btTransform(btQuaternion::getIdentity(), position));

					btCable::MyContactResultCallback firstResult(0.0, &tmp, rb);
					getSoftDynamicsWorld()->contactPairTest(&tmp, rb, firstResult);
					if (firstResult.m_connected == 0) continue;
					
					// Update data to redo the contactPairTest 		
					int idxDeque = 0;
					vector<std::tuple<btVector3, ContactNotInfo>> contactInfo;

					ContactNotInfo contact;
					contact.normal = firstResult.contactNorm;
					contactInfo.emplace_back(firstResult.contactPoint + contact.normal * (margin + FLT_EPSILON), contact);
					
					nodeShape = btSphereShape(margin);
					tmp.setCollisionShape(&nodeShape);

					// Simulate collisions to get the best potential point
					while (true)
					{
						auto contactData = contactInfo[idxDeque];
						btVector3 outPosition = std::get<btVector3>(contactData);
						
						// Set the position to node collision object
						tmp.setWorldTransform(btTransform(btQuaternion::getIdentity(), outPosition));

						btCable::MyContactResultCallback loopResult(0.0, &tmp, rb);
						getSoftDynamicsWorld()->contactPairTest(&tmp, rb, loopResult);

						if (!loopResult.m_connected) idxDeque++;
						else
						{
							ContactNotInfo contact;
							contact.normal = loopResult.contactNorm;
							contactInfo.emplace_back(loopResult.contactPoint + loopResult.contactNorm * (margin + FLT_EPSILON), contact);

							contactInfo.erase(contactInfo.begin() + idxDeque);
						}

						if (idxDeque >= contactInfo.size()) break;		
					}

					// Sort and find the position closest to the current node
					idxDeque = 0;
					btVector3 bestOutPosition = std::get<btVector3>(contactInfo[idxDeque]);
					btScalar bestDistancePosition = bestOutPosition.distance(position);
					for (int idxOut = 1; idxOut < contactInfo.size(); ++idxOut)
					{
						btVector3 outPosition = std::get<btVector3>(contactInfo[idxOut]);
						btScalar distance = outPosition.distance(position);
						
						if(distance < bestDistancePosition)
						{
							bestOutPosition = outPosition;
							bestDistancePosition = distance;
							idxDeque = idxOut;
						}
					}

					// Update output structs with the best potential collision
					ContactNotInfo bestContact = std::get<ContactNotInfo>(contactInfo[idxDeque]);
					m_dynamicsWorld->getDebugDrawer()->drawSphere(bestOutPosition, margin, btVector3(0, 0, 1));
				}
			}

			const auto currentTime{Clock::now()};
			if (m_applyForceOnRigidbody)
			{
				auto wall = (btRigidBody*)m_dynamicsWorld->getCollisionObjectArray().at(2);
				wall->applyForce(btVector3(0, 0, 10) * (btScalar(1.0) / wall->getInvMass()), btVector3(0, 10, 0));
				//AddConstantForce_DemoCableForce();
			}

			if (m_currentDemoIndex == 1 && m_applyForceOnRigidbody)
			{
				AddConstantForce_DemoCableForceUp();
			}

			if (m_moveBody)
				MoveBody();

			if (m_attachLock)
			{
				if (getSoftDynamicsWorld()->getSoftBodyArray().size() == 0) return;
				btCable* _cable = (btCable*)getSoftDynamicsWorld()->getSoftBodyArray()[0];
				_cable->setUseLRA(true);
			}
			//attachLock();
			if (m_printAnchorData)
				printAnchorData();

			if (m_grow)
				Grows(deltaTime);
			else
			{
				if (!m_growWithSpeedAndDistance)
					Grows(deltaTime, false);
			}
			if (m_shrink)
				Shrinks(deltaTime);

			// Test grow speed
			if (m_growWithSpeedAndDistance)
			{
				b3Printf("Dt is %f with a speed of %f, which gives a dt speed of %f. ", deltaTime, growthSpeed, growthSpeed * deltaTime);

				std::chrono::steady_clock::duration timeSpent = currentTime - startingTime;
				if (getSoftDynamicsWorld()->getSoftBodyArray().size() == 0) return;
				btCable* _cable = (btCable*)getSoftDynamicsWorld()->getSoftBodyArray()[0];

				if (initialCableLength < growthDistance)
				{
					if (_cable->getRestLength() >= growthDistance)
					{
						b3Printf("total dt length : %f. Total duration of delta : %f", nbDelta * growthSpeed * deltaTime, nbDelta * deltaTime);
						b3Printf("Time to reach %f meters from initial Cable length (%f) is %f secondes while theorical time was %f ", growthDistance, initialCableLength, std::chrono::duration<double>(timeSpent).count(), std::chrono::duration<double>(theoricalTime).count());
						m_growWithSpeedAndDistance = false;
					}
				}
				else
				{
					if (_cable->getRestLength() <= growthDistance)
					{
						b3Printf("total dt length : %f. Total duration of delta : %f", nbDelta * growthSpeed * deltaTime, nbDelta * deltaTime);
						b3Printf("Time to reach %f meters from initial Cable length (%f) is %f secondes while theorical time was %f ", growthDistance, initialCableLength, std::chrono::duration<double>(timeSpent).count(), std::chrono::duration<double>(theoricalTime).count());
						m_growWithSpeedAndDistance = false;
					}
				}

				nbDelta += 1;
			}

			if (m_currentDemoIndex == 21)  // Cable hydro force
			{
				ApplyForcesNodes(m_waterCurrent);
			}

			int subStep = substepSolver;
			tick(deltaTime, subStep, deltaTime / subStep);
			
			if (m_printFPS && delta_ticks > 0)
			{
				m_fps = CLOCKS_PER_SEC / delta_ticks;
				b3Printf("FPS: %d ", m_fps);
			}
		}
	}

	void AddConstantForce_DemoCableForce()
	{
		btCollisionObjectArray collisionArray = getSoftDynamicsWorld()->getCollisionObjectArray();
		for (int i = 0; i < 5; i++)
		{
			int index = 1 + i * 3;
			if (index < collisionArray.size())
			{
				float force = btPow(10, (i + 1));
				btRigidBody* rb = (btRigidBody*)collisionArray[index];
				rb->applyCentralForce(btVector3(0, 0, force));
			}
		}
	}

	void AddConstantForce_DemoCableForceUp()
	{
		btCollisionObjectArray collisionArray = getSoftDynamicsWorld()->getCollisionObjectArray();
		for (int i = 0; i < 5; i++)
		{
			int index = 0 + i * 3;
			if (index < collisionArray.size())
			{
				float force = btPow(10, (i + 1));
				btRigidBody* rb = (btRigidBody*)collisionArray[index];
				rb->applyCentralForce(btVector3(0, 0, force));
			}
		}
	}

	void PrintDistance_DemoCableForce()
	{
		btCollisionObjectArray collisionArray = getSoftDynamicsWorld()->getCollisionObjectArray();
		//btSoftBodyArray& softArray = getSoftDynamicsWorld()->getSoftBodyArray();
		btSoftBodyArray& softArray = getSoftDynamicsWorld()->getSoftBodyArray();

		std::cout << "---" << std::endl;
		for (int i = 0; i < softArray.size(); i++)
		{
			btCable* cable = (btCable*)softArray[i];
			PrintDistance(i, cable);
		}
		/*
		for (int i = 0; i < softArray.size(); i++)
		{
			int index = 2 + i * 3;
			if (index < collisionArray.size())
			{
				btCable* cable = (btCable*)collisionArray[index];
				PrintDistance(i, cable);
			}
		}*/
	}

	void PrintDistance(int indexCable, btCable* cable)
	{
		btSoftBody::Anchor a0 = cable->m_anchors[0];
		btVector3 worldPositionAnchor0 = a0.m_body->getCenterOfMassPosition() + a0.m_c1;
		btVector3 worldPositionNode0 = a0.m_node->m_x;

		btSoftBody::Anchor a1 = cable->m_anchors[1];
		btVector3 worldPositionAnchor1 = a1.m_body->getCenterOfMassPosition() + a1.m_c1;
		btVector3 worldPositionNode1 = a1.m_node->m_x;

		float distance0 = worldPositionAnchor0.distance(worldPositionNode0) * 100;  // Convert in cm
		float distance1 = worldPositionAnchor1.distance(worldPositionNode1) * 100;  // Convert in cm

		b3Printf("Cable : % i - Distance Anchor0-Node %f cm | Distance Anchor1-Node %f cm | - Cable length %f | - Impluse: %f N", indexCable, distance0, distance1, cable->getLength(), cable->getTensionAt(0).length());
		b3Printf("Distance: %f - Distance Reel: %f", cable->getLength(), cable->getRestLength());
	}

	btCable* createCable(int resolution, int iteration, btScalar totalMass, btVector3 posWorldAnchorBodyA, btVector3 posWorldAnchorBodyB, btRigidBody* bodyB = nullptr, btRigidBody* bodyA = nullptr, bool DisableCollisionOnA = true, bool DisableCollisionOnB = true)
	{
		// Nodes' positions
		btVector3* positionNodes = new btVector3[resolution];
		btScalar* massNodes = new btScalar[resolution];

		for (int i = 0; i < resolution; ++i)
		{
			const btScalar t = i / (btScalar)(resolution - 1);
			positionNodes[i] = lerp(posWorldAnchorBodyB, posWorldAnchorBodyA, t);
			massNodes[i] = 1;
		}

		// Cable's creation
		btCable* cable = new btCable(&m_softBodyWorldInfo, getSoftDynamicsWorld(), resolution, 0, positionNodes, massNodes);
		//cable->setTotalMass(totalMass);
		cable->updateNodesMass();

		cable->setUseCollision(false);
		if (bodyB != nullptr)
			cable->appendAnchor(0, bodyB, posWorldAnchorBodyB - bodyB->getWorldTransform().getOrigin(), DisableCollisionOnB);
		if (bodyA != nullptr)
			cable->appendAnchor(cable->m_nodes.size() - 1, bodyA, posWorldAnchorBodyA - bodyA->getWorldTransform().getOrigin(), DisableCollisionOnA);
		// Cable's config

		// cable->setTotalMass(totalMass);
		cable->m_cfg.piterations = iteration;
		cable->m_cfg.kAHR = 1;
		cable->setUseLRA(true);
		cable->setCollisionMargin(0.01);
		cable->getCollisionShape()->setMargin(0.01);
		//cable->m_materials[0]->m_kLST = 0.02; // Stiffness

		// Add cable to the world
		getSoftDynamicsWorld()->addSoftBody(cable, 8, 8);
		cable->setCollisionParameters(2, 4);
		return cable;
	}

	btCable* createCableWaypoint(int resolution, int iteration, btScalar linearMass, btAlignedObjectArray<btVector3> anchorPos, btRigidBody* bodyA = nullptr, btRigidBody* bodyB = nullptr, bool DisableCollisionOnA = true, bool DisableCollisionOnB = true)
	{
		int s = anchorPos.size() - 1;
		btScalar totalDist = 0;
		for (int i = 0; i < s; i++)
		{
			totalDist += (anchorPos.at(i) - anchorPos.at(i + 1)).length();
		}

		int numberOfNode = 0;
		for (int i = 0; i < s; i++)
		{
			btScalar dist = (anchorPos.at(i) - anchorPos.at(i + 1)).length();
			numberOfNode += dist / totalDist * resolution;
		}
		int resolutionReel = numberOfNode - (anchorPos.size() - 2);

		// Nodes' positions
		//resolution = resolutionReel * s;
		btVector3* positionNodes = new btVector3[resolutionReel];
		btScalar* massNodes = new btScalar[resolutionReel];

		int position = 0;
		// j = number of links between anchors
		for (int j = 0; j < s; j++)
		{
			btScalar dist = (anchorPos.at(j) - anchorPos.at(j + 1)).length();
			int limite = dist / totalDist * resolution;
			for (int i = 0; i < limite; ++i)
			{
				const btScalar t = i / (btScalar)(limite - 1);
				btVector3 pos = lerp(anchorPos.at(j), anchorPos.at(j + 1), t);
				positionNodes[position] = pos;
				massNodes[position] = 1;
				if (i != limite - 1)
					position++;
			}
		}
		// Cable's creation
		btCable* cable = new btCable(&m_softBodyWorldInfo, getSoftDynamicsWorld(), resolutionReel, 0, positionNodes, massNodes);
		cable->setLinearMass(linearMass);
		cable->setUseCollision(false);
		if (bodyA != nullptr)
			cable->appendAnchor(0, bodyA, anchorPos.at(0) - bodyA->getWorldTransform().getOrigin(), DisableCollisionOnA);
		if (bodyB != nullptr)
			cable->appendAnchor(cable->m_nodes.size() - 1, bodyB, anchorPos.at(s) - bodyB->getWorldTransform().getOrigin(), DisableCollisionOnB);
		// Cable's config
		//cable->setTotalMass(totalMass);
		cable->updateNodesMass();
		cable->m_cfg.piterations = iteration;
		cable->m_cfg.kAHR = 1;

		cable->setCollisionParameters(2, 4);
		// Add cable to the world
		getSoftDynamicsWorld()->addSoftBody(cable);

		return cable;
	}
};

btTriangleMesh* buildCylinderMesh(float length,
								  float height,
								  int slices = 32,
								  int stacks = 1)
{
	const float radius = height * 0.5f;  // height here = diameter
	const float halfL = length * 0.5f;

	auto* mesh = new btTriangleMesh(/*use32bitIndices=*/true, /*use4componentVertices=*/false);

	// -----------------------------------------------------------------------
	// 1. Curved side ─────────────────────────────────────────────────────────
	// -----------------------------------------------------------------------
	for (int iStack = 0; iStack < stacks; ++iStack)
	{
		float x0 = halfL - length * iStack / stacks;  // from +X → -X
		float x1 = halfL - length * (iStack + 1) / stacks;

		for (int iSlice = 0; iSlice < slices; ++iSlice)
		{
			float a0 = 2.f * SIMD_PI * iSlice / slices;
			float a1 = 2.f * SIMD_PI * (iSlice + 1) / slices;

			btVector3 v00(x0, radius * btSin(a0), radius * btCos(a0));
			btVector3 v01(x0, radius * btSin(a1), radius * btCos(a1));
			btVector3 v10(x1, radius * btSin(a0), radius * btCos(a0));
			btVector3 v11(x1, radius * btSin(a1), radius * btCos(a1));

			mesh->addTriangle(v00, v11, v10);  // <-- swapped v10 / v11
			mesh->addTriangle(v00, v01, v11);  // <-- swapped v11 / v01
		}
	}

	// -----------------------------------------------------------------------
	// 2. End-caps (disc at +X and -X) ────────────────────────────────────────
	// -----------------------------------------------------------------------
	for (int cap = 0; cap < 2; ++cap)
	{
		float xCap = (cap == 0 ? halfL : -halfL);  // +X then -X
		float normal = (cap == 0 ? 1.f : -1.f);    // outward

		btVector3 centre(xCap, 0, 0);

		for (int iSlice = 0; iSlice < slices; ++iSlice)
		{
			float a0 = 2.f * SIMD_PI * iSlice / slices;
			float a1 = 2.f * SIMD_PI * (iSlice + 1) / slices;

			btVector3 v0(xCap, radius * btSin(a0), radius * btCos(a0));
			btVector3 v1(xCap, radius * btSin(a1), radius * btCos(a1));

			// ------------------ end-caps ---------------------
			if (normal > 0)                         // +X cap
				mesh->addTriangle(centre, v1, v0);  // swap
			else                                    // –X cap
				mesh->addTriangle(centre, v0, v1);  // swap
		}
	}

	return mesh;
}

static void Init_Nodes(CableDemo* pdemo)
{
	// Shape
	btCollisionShape* boxShape = new btBoxShape(btVector3(0.5, 0.5, 0.5));

	// Masses
	btScalar massKinematic(0);
	btScalar massPhysic(10);

	// Rotation
	btQuaternion rotation(0, 0, 0, 1);

	// Transform
	btTransform transformKinematic;
	transformKinematic.setIdentity();
	transformKinematic.setRotation(rotation);

	btTransform transformPhysic;
	transformPhysic.setIdentity();
	transformPhysic.setRotation(rotation);

	// Resolution's cable
	int resolution = 10;
	int iteration = 50;

	// Create 12 cube and 6 cables
	for (int i = 0; i < 5; i++)
	{
		if (i == 1)
		{
			resolution = 50;
		}
		else if (i == 2)
		{
			resolution = 100;
		}
		else if (i == 3)
		{
			resolution = 500;
		}
		else if (i == 4)
		{
			resolution = 1000;
		}

		// Positions
		btVector3 positionKinematic(-5 + (i * 2.5f), 10, 0);
		btVector3 positionPhysic(-5 + (i * 2.5f), 4, 0);
		transformKinematic.setOrigin(positionKinematic);
		transformPhysic.setOrigin(positionPhysic);

		// Create the rigidbodys
		btRigidBody* kinematic = pdemo->createRigidBody(massKinematic, transformKinematic, boxShape);
		btRigidBody* physic = pdemo->createRigidBody(massPhysic, transformPhysic, boxShape);

		// Anchor's positions
		btVector3 anchorPositionKinematic = positionKinematic - btVector3(0, -0.5, 0);
		btVector3 anchorPositionPhysic = positionPhysic + btVector3(0, 0.5, 0);

		btCable* cable = pdemo->createCable(resolution, iteration, resolution, anchorPositionKinematic, anchorPositionPhysic, physic, kinematic);
		cable->setCollisionParameters(2, 4);
	}
}

static void Init_Weigths(CableDemo* pdemo)
{
	// Shape
	btCollisionShape* boxShape = new btBoxShape(btVector3(0.5, 0.5, 0.5));

	// Masses
	btScalar massKinematic(0);
	btScalar massPhysic(1);

	// Rotation
	btQuaternion rotation(0, 0, 0, 1);

	// Transform
	btTransform transformKinematic;
	transformKinematic.setIdentity();
	transformKinematic.setRotation(rotation);

	btTransform transformPhysic;
	transformPhysic.setIdentity();
	transformPhysic.setRotation(rotation);

	// Resolution's cable
	int resolution = 20;
	int iteration = 100;

	// Create 12 cube and 6 cables
	for (int i = 0; i < 6; i++)
	{
		if (i > 0)
		{
			massPhysic = btPow(10, i);
		}

		// Positions
		btVector3 positionKinematic(-5 + (i * 2.5f), 10, 0);
		btVector3 positionPhysic(-5 + (i * 2.5f), 4, 0);
		transformKinematic.setOrigin(positionKinematic);
		transformPhysic.setOrigin(positionPhysic);

		// Create the rigidbodys
		btRigidBody* kinematic = pdemo->createRigidBody(massKinematic, transformKinematic, boxShape);
		btRigidBody* physic = pdemo->createRigidBody(massPhysic, transformPhysic, boxShape);

		// Anchor's positions
		btVector3 anchorPositionKinematic = positionKinematic - btVector3(0, -0.5, 0);
		btVector3 anchorPositionPhysic = positionPhysic + btVector3(0, 0.5, 0);

		btCable* cable = pdemo->createCable(resolution, iteration, 200, anchorPositionKinematic, anchorPositionPhysic, physic, kinematic);
		cable->setCollisionParameters(1, 2);

		cable->m_anchors[0].m_bodyMassRatio = 1;
		cable->m_anchors[1].m_bodyMassRatio = 1;
	}
}

static void Init_Iterations(CableDemo* pdemo)
{
	// Shape
	btCollisionShape* boxShape = new btBoxShape(btVector3(0.5, 0.5, 0.5));

	// Masses
	btScalar massKinematic(0);
	btScalar massPhysic(1000);

	// Rotation
	btQuaternion rotation(0, 0, 0, 1);

	// Transform
	btTransform transformKinematic;
	transformKinematic.setIdentity();
	transformKinematic.setRotation(rotation);

	btTransform transformPhysic;
	transformPhysic.setIdentity();
	transformPhysic.setRotation(rotation);

	// Resolution's cable
	int resolution = 10;
	int iteration = 1;

	// Create 12 cube and 6 cables
	for (int i = 0; i < 5; i++)
	{
		if (i > 0)
		{
			iteration = btPow(10, i);
		}

		// Positions
		btVector3 positionKinematic(-5 + (i * 2.5f), 10, 0);
		btVector3 positionPhysic(-5 + (i * 2.5f), 4, 0);
		transformKinematic.setOrigin(positionKinematic);
		transformPhysic.setOrigin(positionPhysic);

		// Create the rigidbodys
		btRigidBody* kinematic = pdemo->createRigidBody(massKinematic, transformKinematic, boxShape);  // mASS kINECMATIC

		btRigidBody* physic = pdemo->createRigidBody(massPhysic, transformPhysic, boxShape);

		// Anchor's positions
		btVector3 anchorPositionKinematic = positionKinematic - btVector3(0, -0.5, 0);
		btVector3 anchorPositionPhysic = positionPhysic + btVector3(0, 0.5, 0);

		pdemo->createCable(resolution, iteration, 1, anchorPositionKinematic, anchorPositionPhysic, physic, kinematic);
	}
}

static void Init_Lengths(CableDemo* pdemo)
{
	// Shape
	btCollisionShape* boxShape = new btBoxShape(btVector3(0.5, 0.5, 0.5));

	// Masses
	btScalar massKinematic(0);
	btScalar massPhysic(10);

	// Rotation
	btQuaternion rotation(0, 0, 0, 1);

	// Transform
	btTransform transformKinematic;
	transformKinematic.setIdentity();
	transformKinematic.setRotation(rotation);

	btTransform transformPhysic;
	transformPhysic.setIdentity();
	transformPhysic.setRotation(rotation);

	// Resolution's cable
	int resolution = 20;
	int iteration = 50;

	// Positions
	btVector3 positionKinematic(-5, 7, 0);

	// Create 12 cube and 6 cables
	for (int i = 0; i < 5; i++)
	{
		if (i == 1)
		{
			positionKinematic = btVector3(-2.5, 10, 0);
		}
		else if (i == 2)
		{
			positionKinematic = btVector3(0, 30, 0);
		}
		else if (i == 3)
		{
			positionKinematic = btVector3(2.5, 105, 0);
		}
		else if (i == 4)
		{
			positionKinematic = btVector3(5, 505, 0);
		}

		btVector3 positionPhysic(-5 + (i * 2.5f), 4, 0);
		transformKinematic.setOrigin(positionKinematic);
		transformPhysic.setOrigin(positionPhysic);

		// Create the rigidbodys
		btRigidBody* kinematic = pdemo->createRigidBody(massKinematic, transformKinematic, boxShape);
		btRigidBody* physic = pdemo->createRigidBody(massPhysic, transformPhysic, boxShape);

		// Anchor's positions
		btVector3 anchorPositionKinematic = positionKinematic - btVector3(0, -0.5, 0);
		btVector3 anchorPositionPhysic = positionPhysic + btVector3(0, 0.5, 0);

		pdemo->createCable(resolution, iteration, 1, anchorPositionKinematic, anchorPositionPhysic, physic, kinematic);
	}
}

static void Init_CableBending(CableDemo* pdemo)
{
	// Shape
	btCollisionShape* cubeShape = new btBoxShape(btVector3(0.5, 0.5, 0.5));
	btCollisionShape* emptyShape = new btBoxShape(btVector3(0, 0, 0));

	// Parameters
	btScalar cubeMass(0);
	btVector3 cubeOrigin(0, 0, 0);
	btVector3 startPositionCable(2, 2, 0);
	btVector3 endPositionCable(0, 2, 0);
	btQuaternion cubeRotation(0, 0, 0, 1);

	// Transform
	btTransform cubeTransform;
	cubeTransform.setIdentity();
	cubeTransform.setRotation(cubeRotation);

	cubeTransform.setOrigin(cubeOrigin);
	// btRigidBody* cube = pdemo->createRigidBody(cubeMass, cubeTransform, cubeShape);

	cubeTransform.setOrigin(startPositionCable);
	btRigidBody* startAttach = pdemo->createRigidBody(0, cubeTransform, emptyShape);

	cubeTransform.setOrigin(endPositionCable);
	btRigidBody* endAttach = pdemo->createRigidBody(0, cubeTransform, emptyShape);

	// Resolution's cable
	int resolution = 20;
	int iteration = 120;

	btCable* cable = pdemo->createCable(resolution, iteration, 3, startPositionCable, endPositionCable, startAttach);
	cable->appendAnchor(1, endAttach, cable->m_nodes[1].m_x - endPositionCable, true);
	cable->setUseLRA(true);
	cable->setInvertLRA(true);
	cable->setUseBending(true);
	cable->setUseHydroAero(false);
	cable->setUseCollision(true);
	cable->setCollisionParameters(3, 6);
	cable->setCableRadius(0.005);
	cable->setCollisionMargin(0.01);
	cable->setBendingStiffness(0.4);
}

static void Init_TwoCablesOneCube(CableDemo* pdemo)
{
	// Shape
	btCollisionShape* boxShape = new btBoxShape(btVector3(0.5, 0.5, 0.5));
	btCollisionShape* sphereShape = new btSphereShape(0.1);

	// Mass
	btScalar massPhysic(10);
	btScalar massKinematic(0);

	// Rotation
	btQuaternion rotationPhysic(0, 0, 0, 1);
	btQuaternion rotationKinematic(0, 0, 0, 1);

	// Position
	btVector3 positionPhysic(0, 0, 0);
	btVector3 anchorMidPosition(0, 0.5, 0);
	btVector3 anchorLeftPosition(2 * sqrt(2.0) / 2, 0.5 + 2 * sqrt(2.0) / 2, 0);
	btVector3 anchorRightPosition(2 * -sqrt(2.0) / 2, 0.5 + 2 * sqrt(2.0) / 2, 0);

	// Transform
	btTransform transformPhysics;
	transformPhysics.setIdentity();
	transformPhysics.setRotation(rotationPhysic);
	transformPhysics.setOrigin(positionPhysic);
	btTransform transformLeftKinematic;
	transformLeftKinematic.setIdentity();
	transformLeftKinematic.setRotation(rotationKinematic);
	transformLeftKinematic.setOrigin(anchorLeftPosition);
	btTransform transformRightKinematic;
	transformRightKinematic.setIdentity();
	transformRightKinematic.setRotation(rotationKinematic);
	transformRightKinematic.setOrigin(anchorRightPosition);

	// Create rigidbodies
	btRigidBody* physic = pdemo->createRigidBody(massPhysic, transformPhysics, boxShape);
	btRigidBody* leftKinematic = pdemo->createRigidBody(massKinematic, transformLeftKinematic, sphereShape);
	btRigidBody* rightKinematic = pdemo->createRigidBody(massKinematic, transformRightKinematic, sphereShape);

	// Resolution's cable
	int resolution = 20;
	int iteration = 200;

	// 1rst cable
	btCable* rightCable = pdemo->createCable(resolution, iteration, 20, anchorRightPosition, anchorMidPosition, physic, rightKinematic);
	rightCable->setUseLRA(true);
	btCable* leftCable = pdemo->createCable(resolution, iteration, 20, anchorLeftPosition, anchorMidPosition, physic, leftKinematic);
	leftCable->setUseLRA(true);
}

static void Init_CableForceDown(CableDemo* pdemo)
{
	// Shape
	btCollisionShape* boxShape = new btBoxShape(btVector3(0.25, 0.25, 0.25));

	// Masses
	btScalar massKinematic(0);
	btScalar massPhysic(10);

	// Rotation
	btQuaternion rotation(0, 0, 0, 1);

	// Transform
	btTransform transformKinematic;
	transformKinematic.setIdentity();
	transformKinematic.setRotation(rotation);

	btTransform transformPhysic;
	transformPhysic.setIdentity();
	transformPhysic.setRotation(rotation);

	// Resolution's cable
	int resolution = 84;
	int iteration = 120;

	// Positions
	btVector3 positionKinematic(0, 3, 0);
	btVector3 positionPhysic(0, 3 - 6.68, 0);
	transformKinematic.setOrigin(positionKinematic);
	transformPhysic.setOrigin(positionPhysic);

	// Create the rigidbodys
	btRigidBody* kinematic = pdemo->createRigidBody(massKinematic, transformKinematic, boxShape);
	btRigidBody* physic = pdemo->createRigidBody(massKinematic, transformPhysic, boxShape);

	// Anchor's positions
	btVector3 anchorPositionKinematic = positionKinematic;
	btVector3 anchorPositionPhysic = positionPhysic;

	btCable* cable = pdemo->createCable(resolution, iteration, 1, anchorPositionKinematic, anchorPositionPhysic, physic, kinematic);
}

static void Init_CableHydro(CableDemo* pdemo)
{
	// Shape
	btCollisionShape* boxShape = new btBoxShape(btVector3(0.25, 0.25, 0.25));

	// Masses
	btScalar massKinematic(0);

	// Rotation
	btQuaternion rotation(0, 0, 0, 1);

	// Transform
	btTransform transformKinematic;
	transformKinematic.setIdentity();
	transformKinematic.setRotation(rotation);

	btTransform transformPhysic;
	transformPhysic.setIdentity();
	transformPhysic.setRotation(rotation);

	// Resolution's cable
	int resolution = 84;
	int iteration = 120;
	btScalar cableLength = 6.68;
	btScalar linearMass = 0.003;  // 3gr per meter
	btScalar cableTotalMass = cableLength * linearMass;

	// Positions
	btScalar cubeUpPosition = 3;

	btVector3 positionKinematic(0, cubeUpPosition, 0);
	btVector3 positionPhysic(0, cubeUpPosition - cableLength, 0);
	transformKinematic.setOrigin(positionKinematic);
	transformPhysic.setOrigin(positionPhysic);

	// Create the rigidbodys
	btRigidBody* kinematic = pdemo->createRigidBody(massKinematic, transformKinematic, boxShape);
	btRigidBody* physic = pdemo->createRigidBody(massKinematic, transformPhysic, boxShape);

	// Anchor's positions
	btVector3 anchorPositionKinematic = positionKinematic;
	btVector3 anchorPositionPhysic = positionPhysic;

	// Cable
	btCable* cable = pdemo->createCable(resolution, iteration, cableTotalMass, anchorPositionKinematic, anchorPositionPhysic, physic, kinematic);

	cable->setCableNormalDragCoefficient(1.2f);
	cable->setCableRadius(0.0048f);

	b3Printf("Cable length :%f - Total cable mass: %f - DragCoefficient:%f - Radius:%f ", cableLength, cableTotalMass, cable->getCableData()->normalDragCoefficient, cable->getCableData()->radius);
}

static void Init_CableForceUp(CableDemo* pdemo)
{
	// Shape
	btCollisionShape* boxShape = new btBoxShape(btVector3(0.5, 0.5, 0.5));
	// Cable 0: 10 nodes ; 5 m ; 2 bodies (0 & 10 kg) ; static
	//
	// Masses
	btScalar massKinematic(10);
	btScalar massPhysic(10);

	// Rotation
	btQuaternion rotation(0, 0, 0, 1);

	// Transform
	btTransform transformKinematic;
	transformKinematic.setIdentity();
	transformKinematic.setRotation(rotation);

	btTransform transformPhysic;
	transformPhysic.setIdentity();
	transformPhysic.setRotation(rotation);

	// Resolution's cable
	int resolution = 20;
	int iteration = 50;

	pdemo->m_softBodyWorldInfo.m_gravity = btVector3(0, 0, 0);
	pdemo->getSoftDynamicsWorld()->setGravity(btVector3(0, 0, 0));

	// Create 10 cube and 5 cables
	for (int i = 0; i < 5; i++)
	{
		// Positions
		btVector3 positionKinematic(-5 + (i * 2.5f), 10, 0);
		btVector3 positionPhysic(-5 + (i * 2.5f), 4, 0);
		transformKinematic.setOrigin(positionKinematic);
		transformPhysic.setOrigin(positionPhysic);

		// Create the rigidbodys
		btRigidBody* kinematic = pdemo->createRigidBody(massKinematic, transformKinematic, boxShape);
		btRigidBody* physic = pdemo->createRigidBody(massPhysic, transformPhysic, boxShape);

		kinematic->clearGravity();
		physic->clearGravity();

		// Anchor's positions
		btVector3 anchorPositionKinematic = positionKinematic - btVector3(0, -0.5, 0);
		btVector3 anchorPositionPhysic = positionPhysic + btVector3(0, 0.5, 0);

		pdemo->createCable(resolution, iteration, 1, anchorPositionKinematic, anchorPositionPhysic, physic, kinematic);
	}
}

static void Init_TestArse(CableDemo* pdemo)
{
	// Shape
	btCollisionShape* supportShape = new btBoxShape(btVector3(16, 2, 2));
	btCollisionShape* arseShape = new btBoxShape(btVector3(13, 5, 5));

	// Masses
	btScalar massKinematic(0);
	btScalar massPhysic(7500);

	// Position / Rotation
	btVector3 positionKinematic(0, 20, 0);
	btVector3 positionPhysic(0, 10, 0);
	btQuaternion rotation(0, 0, 0, 1);

	// Transform
	btTransform transformKinematic;
	transformKinematic.setIdentity();
	transformKinematic.setRotation(rotation);
	transformKinematic.setOrigin(positionKinematic);

	btTransform transformPhysic;
	transformPhysic.setIdentity();
	transformPhysic.setRotation(rotation);
	transformPhysic.setOrigin(positionPhysic);

	// Resolution's cable
	int resolution = 20;
	int iteration = 20;

	// Create the rigidbodys
	btRigidBody* kinematic = pdemo->createRigidBody(massKinematic, transformKinematic, supportShape);
	btRigidBody* physic = pdemo->createRigidBody(massPhysic, transformPhysic, arseShape);

	// Anchor's positions
	btVector3 anchorPositionKinematic = positionKinematic + btVector3(-3.8, -2.0, 0);
	btVector3 anchorPositionPhysic = positionPhysic + btVector3(-4, 5, 0);

	pdemo->createCable(resolution, iteration, 1, anchorPositionKinematic, anchorPositionPhysic, physic, kinematic);

	// Anchor's positions
	anchorPositionKinematic = positionKinematic + btVector3(3.8, -2.0, 0);
	anchorPositionPhysic = positionPhysic + btVector3(4, 5, 0);

	pdemo->createCable(resolution, iteration, 1, anchorPositionKinematic, anchorPositionPhysic, physic, kinematic);

	pdemo->SetCameraPosition(btVector3(0, 17, 15));
}

static void Init_TestCollisionFreeCableWithStaticCube(CableDemo* pdemo)
{
	// Shape
	btCollisionShape* shape = new btBoxShape(btVector3(0.5, 0.5, 0.5));

	// Position / Rotation
	btVector3 positionKinematic(0, 10.1, -3);
	btVector3 positionPhysic(0, 10.1, 4);
	btVector3 positionWall(0, 8, 0);
	btQuaternion rotation(0, 0, 0, 1);

	// Transform
	btTransform transformKinematic;
	transformKinematic.setIdentity();
	transformKinematic.setRotation(rotation);
	transformKinematic.setOrigin(positionKinematic);

	btTransform transformPhysic;
	transformPhysic.setIdentity();
	transformPhysic.setRotation(rotation);
	transformPhysic.setOrigin(positionPhysic);

	btTransform transformWall;
	transformWall.setIdentity();
	transformWall.setOrigin(positionWall);

	// Resolution's cables
	int resolution = 80;
	int iteration = 80;

	// Create the rigidbodys
	btRigidBody* kinematic = pdemo->createRigidBody(0, transformKinematic, shape);
	btRigidBody* physic = pdemo->createRigidBody(10, transformPhysic, shape);

	static const btVector3 kHalfExtents(0.75, 0.5, 0.25);

	static const btVector3 kCubeVerts[8] = {
		{-kHalfExtents.x(), -kHalfExtents.y(), -kHalfExtents.z()},  // 0
		{kHalfExtents.x(), -kHalfExtents.y(), -kHalfExtents.z()},   // 1
		{kHalfExtents.x(), kHalfExtents.y(), -kHalfExtents.z()},    // 2
		{-kHalfExtents.x(), kHalfExtents.y(), -kHalfExtents.z()},   // 3
		{-kHalfExtents.x(), -kHalfExtents.y(), kHalfExtents.z()},   // 4
		{kHalfExtents.x(), -kHalfExtents.y(), kHalfExtents.z()},    // 5
		{kHalfExtents.x(), kHalfExtents.y(), kHalfExtents.z()},     // 6
		{-kHalfExtents.x(), kHalfExtents.y(), kHalfExtents.z()}     // 7
	};

	static const unsigned int kCubeIdx[36] = {
		0, 2, 1, 2, 0, 3,  // −Z
		4, 5, 6, 6, 7, 4,  // +Z
		0, 5, 4, 5, 0, 1,  // −Y
		3, 6, 2, 6, 3, 7,  // +Y
		1, 6, 5, 6, 1, 2,  // +X
		0, 7, 3, 7, 0, 4   // −X
	};

	btIndexedMesh mesh;
	{
		mesh.m_numTriangles = 12;
		mesh.m_triangleIndexBase = reinterpret_cast<const unsigned char*>(kCubeIdx);
		mesh.m_triangleIndexStride = 3 * sizeof(unsigned int);
		mesh.m_numVertices = 8;
		mesh.m_vertexBase = reinterpret_cast<const unsigned char*>(kCubeVerts);
		mesh.m_vertexStride = sizeof(btVector3);
		mesh.m_indexType = PHY_INTEGER;
		mesh.m_vertexType = PHY_DOUBLE;
	}

	static btTriangleIndexVertexArray triArray;
	triArray.addIndexedMesh(mesh, PHY_INTEGER);

	btGImpactMeshShape* wallShape = new btGImpactMeshShape(&triArray);
	wallShape->setMargin(0);
	wallShape->updateBound();

	btCollisionObject* wallCableCollision = new btCollisionObject();
	wallCableCollision->setCollisionShape(wallShape);
	wallCableCollision->setWorldTransform(transformWall);
	pdemo->getDynamicsWorld()->addCollisionObject(wallCableCollision, 8, 8);

	btTransform wallCableCollisionLocalTransfom = btTransform();
	wallCableCollisionLocalTransfom.setIdentity();

	btRigidBody* wall = pdemo->createCableRigidBody(0, transformWall, new btBoxShape(btVector3(0.5, 0.5, 0.5)));
	wall->getCollisionShape()->setMargin(0);

	// Anchor's positions
	btVector3 anchorPositionKinematic = positionKinematic + btVector3(0, 0, 0);
	btVector3 anchorPositionPhysic = positionPhysic + btVector3(0, 0, 0);

	btCable* cable = pdemo->createCable(resolution, iteration, 1.61, anchorPositionKinematic, anchorPositionPhysic, physic, kinematic);
	cable->setUseCollision(true);
	cable->setUseLRA(false);
	cable->getCollisionShape()->setMargin(0.01);
	cable->setCollisionMargin(0.01);
	cable->setCollisionParameters(1, 2);

	pdemo->SetCameraPosition(btVector3(0, 10, 0));
}

static void Init_TestCableCollisionMt(CableDemo* pdemo)
{
	// Benchmark part
	btVector3 boxSize(1.5f, 1.5f, 1.5f);
	float boxMass = 1.0f;
	float sphereRadius = 1.5f;
	float sphereMass = 1.0f;
	float capsuleHalf = 2.0f;
	float capsuleRadius = 1.0f;
	float capsuleMass = 1.0f;

	{
		int size = 10;
		int height = 10;

		const float cubeSize = boxSize[0];
		float spacing = 2.0f;
		btVector3 pos(0.0f, 20.0f, 0.0f);
		float offset = -size * (cubeSize * 2.0f + spacing) * 0.5f;

		int numBodies = 0;

		for (int k = 0; k < height; k++)
		{
			for (int j = 0; j < size; j++)
			{
				pos[2] = offset + (float)j * (cubeSize * 2.0f + spacing);
				for (int i = 0; i < size; i++)
				{
					pos[0] = offset + (float)i * (cubeSize * 2.0f + spacing);
					btVector3 bpos = btVector3(0, 25, 0) + btVector3(5.0f, 1.0f, 5.0f) * pos;
					int idx = rand() % 9;
					btTransform trans;
					trans.setIdentity();
					trans.setOrigin(bpos);

					switch (idx)
					{
						case 0:
						case 1:
						case 2:
						{
							float r = 0.5f * (idx + 1);
							btBoxShape* boxShape = new btBoxShape(boxSize * r);
							pdemo->createRigidBody(boxMass * r, trans, boxShape);
						}
						break;

						case 3:
						case 4:
						case 5:
						{
							float r = 0.5f * (idx - 3 + 1);
							btSphereShape* sphereShape = new btSphereShape(sphereRadius * r);
							pdemo->createRigidBody(sphereMass * r, trans, sphereShape);
						}
						break;

						case 6:
						case 7:
						case 8:
						{
							float r = 0.5f * (idx - 6 + 1);
							btCapsuleShape* capsuleShape = new btCapsuleShape(capsuleRadius * r, capsuleHalf * r);
							pdemo->createRigidBody(capsuleMass * r, trans, capsuleShape);
						}
						break;
					}

					numBodies++;
				}
			}
			offset -= 0.05f * spacing * (size - 1);
			spacing *= 1.1f;
			pos[1] += (cubeSize * 2.0f + spacing);
		}
	}

	// OG Cable test
	// Shape
	btCollisionShape* shape = new btBoxShape(btVector3(0.5, 0.5, 0.5));

	// Position / Rotation
	btVector3 positionKinematic(0, 8.1, -3);
	btVector3 positionPhysic(0, 8.1, 4);
	btVector3 positionWall(0, 8, 0);
	btQuaternion rotation(0, 0, 0, 1);

	// Transform
	btTransform transformKinematic;
	transformKinematic.setIdentity();
	transformKinematic.setRotation(rotation);
	transformKinematic.setOrigin(positionKinematic);

	btTransform transformPhysic;
	transformPhysic.setIdentity();
	transformPhysic.setRotation(rotation);
	transformPhysic.setOrigin(positionPhysic);

	btTransform transformWall;
	transformWall.setIdentity();
	transformWall.setOrigin(positionWall);

	// Resolution's cables
	int resolution = 50;
	int iteration = 50;

	// Create the rigidbodys
	btRigidBody* kinematic = pdemo->createRigidBody(0, transformKinematic, shape);
	btRigidBody* physic = pdemo->createRigidBody(100, transformPhysic, shape);

	btRigidBody* wall = pdemo->createCableRigidBody(0, transformWall, new btBoxShape(btVector3(1, 1, 1)));

	wall->getCollisionShape()->setMargin(0);

	// Anchor's positions
	btVector3 anchorPositionKinematic = positionKinematic + btVector3(0, 0, 0);
	btVector3 anchorPositionKinematic2 = positionKinematic + btVector3(0.5, 0, 0);
	btVector3 anchorPositionKinematic3 = positionKinematic + btVector3(-0.5, 0, 0);
	btVector3 anchorPositionKinematic4 = positionKinematic + btVector3(-1, 0, 0);
	btVector3 anchorPositionPhysic = positionPhysic + btVector3(0, 0, 0);
	btVector3 anchorPositionPhysic2 = positionPhysic + btVector3(0.5, 0, 0);
	btVector3 anchorPositionPhysic3 = positionPhysic + btVector3(-0.5, 0, 0);
	btVector3 anchorPositionPhysic4 = positionPhysic + btVector3(1, 0, 0);

	btCable* cable = pdemo->createCable(resolution, iteration, 1.61, anchorPositionKinematic, anchorPositionPhysic, physic, kinematic);

	cable->setUseCollision(true);
	cable->setUseLRA(false);
	cable->getCollisionShape()->setMargin(0.005);
	cable->setCollisionParameters(1,2);
	cable->setCollisionMargin(0.005);
	pdemo->SetCameraPosition(btVector3(0, 10, 0));

	btCable* cable2 = pdemo->createCable(resolution, iteration, 1.61, anchorPositionKinematic2, anchorPositionPhysic2, physic, kinematic);

	cable2->setUseCollision(true);
	cable2->setUseLRA(false);
	cable2->getCollisionShape()->setMargin(0.005);
	cable2->setCollisionParameters(1, 2);
	cable2->setCollisionMargin(0.005);

	btCable* cable3 = pdemo->createCable(resolution, iteration, 1.61, anchorPositionKinematic3, anchorPositionPhysic3, physic, kinematic);

	cable3->setUseCollision(true);
	cable3->setUseLRA(false);
	cable3->getCollisionShape()->setMargin(0.005);
	cable3->setCollisionParameters(1, 2);
	cable3->setCollisionMargin(0.005);

	btCable* cable4 = pdemo->createCable(resolution, iteration, 1.61, anchorPositionKinematic4, anchorPositionPhysic4, physic, kinematic);

	cable4->setUseCollision(true);
	cable4->setUseLRA(false);
	cable4->getCollisionShape()->setMargin(0.005);
	cable4->setCollisionParameters(1, 2);
	cable4->setCollisionMargin(0.005);
}

static void Init_TestBenchmarkSubsteps(CableDemo* pdemo)
{
	// Benchmark
	int tickCount = 1;
	btScalar cumulatedDelta = 0;

	// Setup
	btVector3 boxSize(1.5f, 1.5f, 1.5f);
	float sphereMass = 0.01f;
	float boxMass = 0.01f;
	float sphereRadius = 1.5f;
	float capsuleHalf = 2.0f;
	float capsuleRadius = 1.0f;
	float capsuleMass = 1.0f;

	{
		int size = 5;
		int height = 5;

		const float cubeSize = boxSize[0];
		float spacing = 0.5f;
		btVector3 pos(0.0f, 10, 0.0f);
		float offset = -size * (cubeSize * 1.0f + spacing) * 0.5f;

		int numBodies = 0;

		for (int k = 0; k < height; k++)
		{
			for (int j = 0; j < size; j++)
			{
				pos[2] = offset + (float)j * (cubeSize * 1.0f + spacing);
				for (int i = 0; i < size; i++)
				{
					pos[0] = offset + (float)i * (cubeSize * 1.0f + spacing);
					btVector3 bpos = btVector3(0, 0, 0) + btVector3(5.0f, 1.0f, 5.0f) * pos;
					int idx = rand() % 9;
					btTransform trans;
					trans.setIdentity();
					trans.setOrigin(bpos);

					switch (idx)
					{
						case 0:
						case 1:
						case 2:
						{
							float r = 0.5f * (idx + 1);
							btBoxShape* boxShape = new btBoxShape(boxSize * r);
							pdemo->createRigidBody(boxMass * r, trans, boxShape)->setSleepingThresholds(0, 0);
						}
						break;

						case 3:
						case 4:
						case 5:
						{
							float r = 0.5f * (idx - 3 + 1);
							btSphereShape* sphereShape = new btSphereShape(sphereRadius * r);
							pdemo->createRigidBody(sphereMass * r, trans, sphereShape)->setSleepingThresholds(0, 0);
						}
						break;

						case 6:
						case 7:
						case 8:
						{
							float r = 0.5f * (idx - 6 + 1);
							btCapsuleShape* capsuleShape = new btCapsuleShape(capsuleRadius * r, capsuleHalf * r);
							pdemo->createRigidBody(capsuleMass * r, trans, capsuleShape)->setSleepingThresholds(0, 0);


						}
						break;
					}

					numBodies++;
				}
			}
			offset -= 0.05f * spacing * (size - 1);
			spacing *= 0.5f;
			pos[1] += (cubeSize * 1.0f + spacing);
		}
	}

	// OG Cable test
	// Shape
	btCollisionShape* shape = new btBoxShape(btVector3(0.5, 0.5, 0.5));

	// Position / Rotation
	btVector3 positionKinematic(0, 8.1, -3);
	btVector3 positionPhysic(0, 8.1, 4);
	btVector3 positionWall(0, 7, 0);
	btVector3 positionAnotherWall(0, 9, 0);
	btQuaternion rotation(0, 0, 0, 1);

	// Transform
	btTransform transformKinematic;
	transformKinematic.setIdentity();
	transformKinematic.setRotation(rotation);
	transformKinematic.setOrigin(positionKinematic);

	btTransform transformPhysic;
	transformPhysic.setIdentity();
	transformPhysic.setRotation(rotation);
	transformPhysic.setOrigin(positionPhysic);

	btTransform transformWall;
	transformWall.setIdentity();
	transformWall.setOrigin(positionWall);

	btTransform transformAnotherWall;
	transformAnotherWall.setIdentity();
	transformAnotherWall.setOrigin(positionAnotherWall);

	// Resolution's cables
	int resolution = 50;
	int iteration = 50;

	// Create the rigidbodys
	btRigidBody* kinematic = pdemo->createRigidBody(0, transformKinematic, shape);
	btRigidBody* physic = pdemo->createRigidBody(100, transformPhysic, shape);

	btRigidBody* wall = pdemo->createCableRigidBody(0, transformWall, new btBoxShape(btVector3(10, 1, 1)));
	wall->getCollisionShape()->setMargin(0);

	btRigidBody* anotherWall = pdemo->createCableRigidBody(0, transformAnotherWall, new btBoxShape(btVector3(5, 0.1, 10)));
	anotherWall->getCollisionShape()->setMargin(0);
	anotherWall->setSleepingThresholds(0, 0);

	// Anchor's positions
	btVector3 anchorPositionKinematic = positionKinematic + btVector3(0, 0, 0);
	btVector3 anchorPositionKinematic2 = positionKinematic + btVector3(0.5, 0, 0);
	btVector3 anchorPositionKinematic3 = positionKinematic + btVector3(-0.5, 0, 0);
	btVector3 anchorPositionKinematic4 = positionKinematic + btVector3(-1, 0, 0);
	btVector3 anchorPositionKinematic5 = positionKinematic + btVector3(-2, 0, 0);
	btVector3 anchorPositionPhysic = positionPhysic + btVector3(0, 0, 0);
	btVector3 anchorPositionPhysic2 = positionPhysic + btVector3(0.5, 0, 0);
	btVector3 anchorPositionPhysic3 = positionPhysic + btVector3(-0.5, 0, 0);
	btVector3 anchorPositionPhysic4 = positionPhysic + btVector3(1, 0, 0);
	btVector3 anchorPositionPhysic5 = positionPhysic + btVector3(2, 0, 0);

	btCable* cable = pdemo->createCable(resolution, iteration, 1.61, anchorPositionKinematic, anchorPositionPhysic, physic, kinematic);
	cable->setUseCollision(true);
	cable->setUseLRA(false);
	cable->getCollisionShape()->setMargin(0.005);
	cable->setCollisionParameters(1, 2);
	cable->setCollisionMargin(0.005);
	pdemo->SetCameraPosition(btVector3(0, 10, 0));

	btCable* cable2 = pdemo->createCable(resolution, iteration, 1.61, anchorPositionKinematic2, anchorPositionPhysic2, physic, kinematic);
	cable2->setUseCollision(true);
	cable2->setUseLRA(false);
	cable2->getCollisionShape()->setMargin(0.005);
	cable2->setCollisionParameters(1, 2);
	cable2->setCollisionMargin(0.005);

	btCable* cable3 = pdemo->createCable(resolution, iteration, 1.61, anchorPositionKinematic3, anchorPositionPhysic3, physic, kinematic);
	cable3->setUseCollision(true);
	cable3->setUseLRA(false);
	cable3->getCollisionShape()->setMargin(0.005);
	cable3->setCollisionParameters(1, 2);
	cable3->setCollisionMargin(0.005);

	btCable* cable4 = pdemo->createCable(resolution, iteration, 1.61, anchorPositionKinematic4, anchorPositionPhysic4, physic, kinematic);
	cable4->setUseCollision(true);
	cable4->setUseLRA(false);
	cable4->getCollisionShape()->setMargin(0.005);
	cable4->setCollisionParameters(1, 2);
	cable4->setCollisionMargin(0.005);

	btCable* cable5 = pdemo->createCable(resolution, iteration, 1.61, anchorPositionKinematic5, anchorPositionPhysic5, physic, kinematic);
	cable5->setUseCollision(true);
	cable5->setUseLRA(false);
	cable5->getCollisionShape()->setMargin(0.005);
	cable5->setCollisionParameters(1, 2);
	cable5->setCollisionMargin(0.005);
}	


static void initLock(CableDemo* pdemo)
{
	btAlignedObjectArray<btRigidBody*> sphere = btAlignedObjectArray<btRigidBody*>();

	btTransform pos = btTransform();
	pos.setIdentity();
	pos.setOrigin(btVector3(-0.2, 1, 0));

	// Create sphere ring

	btTransform spherePositionB = btTransform();
	spherePositionB.setIdentity();
	spherePositionB.setOrigin(btVector3(0.30, 0, 0.3));

	btTransform spherePositionC = btTransform();
	spherePositionC.setIdentity();
	spherePositionC.setOrigin(btVector3(0.4, 0, 0.0));

	btTransform spherePositionD = btTransform();
	spherePositionD.setIdentity();
	spherePositionD.setOrigin(btVector3(0.30, 0, -0.3));

	btCompoundShape* a18shape = new btCompoundShape();

	btTransform a18 = btTransform();
	a18.setIdentity();
	a18.setOrigin(btVector3(-6, 0, 0));

	// Body
	btBoxShape* a = new btBoxShape(btVector3(6, 1, 1));
	// T center
	btBoxShape* b = new btBoxShape(btVector3(0.3, 0.3, 0.2));
	btCylinderShape* c = new btCylinderShapeZ(btVector3(0.15, 0.15, 0.5));
	btBoxShape* d = new btBoxShape(btVector3(0.3, 0.3, 0.2));

	a->setMargin(0);
	b->setMargin(0);
	c->setMargin(0);
	d->setMargin(0);

	a18shape->addChildShape(a18, a);
	a18shape->addChildShape(spherePositionB, b);
	a18shape->addChildShape(spherePositionC, c);
	a18shape->addChildShape(spherePositionD, d);

	btRigidBody* box = pdemo->createRigidBody(704, pos, a18shape);
	box->setRestitution(0);
	box->setDamping(0.1, 0.1);
	// box->setGravity(btVector3(0, 0, 0));
	box->setFriction(1);
}

static void Init_TestSupportA18(CableDemo* pdemo)
{
	// Resolution's cable
	int resolution = 60;
	int iterations = 100;
	btScalar margin = 0.005;

	// Shape
	//Loading object
	//load our obj mesh

	btConvexHullShape* sphereShape = new btConvexHullShape();
	{
		const char* fileName = "sphere8.obj";
		char relativeFileName[1024];
		if (b3ResourcePath::findResourcePath(fileName, relativeFileName, 1024, 0))
		{
			char pathPrefix[1024];
			b3FileUtils::extractPath(relativeFileName, pathPrefix, 1024);
		}

		b3BulletDefaultFileIO fileIO;
		GLInstanceGraphicsShape* glmesh = LoadMeshFromObj(relativeFileName, "", &fileIO);
		printf("[INFO] Obj loaded: Extracted %d verticed from obj file [%s]\n", glmesh->m_numvertices, fileName);

		for (int i = 0; i < glmesh->m_numvertices; i++)
		{
			const GLInstanceVertex& v = glmesh->m_vertices->at(i);
			float temp = v.xyzw[0];
			btVector3 vtx(v.xyzw[0], v.xyzw[1], v.xyzw[2]);
			sphereShape->addPoint(vtx);
		}
		sphereShape->setLocalScaling(btVector3(2, 2, 2));
		sphereShape->optimizeConvexHull();
		sphereShape->initializePolyhedralFeatures();
	}

	btCollisionShape* cylander = new btBoxShape(btVector3(0.2, .3, 0.2));
	btCollisionShape* cylanderB = new btBoxShape(btVector3(0.2, .2, 0.5));

	btTransform AnchorUpPos = btTransform();
	AnchorUpPos.setIdentity();
	AnchorUpPos.setOrigin(btVector3(0, 3, 0));
	btRigidBody* AnchorUp = pdemo->createRigidBody(0, AnchorUpPos, new btBoxShape(btVector3(0.2, 0.2, 0.2)));

	btTransform LestTransform = btTransform();
	LestTransform.setIdentity();
	LestTransform.setOrigin(btVector3(0, -3, 0));
	btRigidBody* Lest = pdemo->createRigidBody(10, LestTransform, cylander);
	Lest->setupMassAtImpact(10, 704, 0, 0.1);
	Lest->activeMassAtImpact(true);

	//btVector3 positionWall(2, 0.8,0);
	//btTransform transformWall;
	//transformWall.setIdentity();
	//transformWall.setOrigin(positionWall);
	//
	//btRigidBody* wall = pdemo->createRigidBody(100, transformWall, new btBoxShape(btVector3(0.1, 0.1, 2)));
	//wall->setGravity(btVector3(0, 0, 0));

	initLock(pdemo);

	//btTransform t;
	//t.setIdentity();
	//t.setOrigin(btVector3(0,1,0));
	//
	//btRigidBody* tb = pdemo->createRigidBody(1000, t, new btSphereShape(0.5));
	//tb->setGravity(btVector3(0, 0, 0));

	btAlignedObjectArray<btVector3> waypointPos = btAlignedObjectArray<btVector3>();
	waypointPos.push_back(LestTransform.getOrigin() + btVector3(0, 0.3, 0));  // point de départ
	waypointPos.push_back(AnchorUpPos.getOrigin() + btVector3(0, -0.3, 0));   // Arrivée

	btCable* cable = pdemo->createCableWaypoint(resolution, iterations, 1, waypointPos, Lest, AnchorUp, true, true);
	//btCable* cable = pdemo->createCable(resolution, iterations, 3, LestTransform.getOrigin() + btVector3(0, 0.5, 0), AnchorUpPos.getOrigin() + btVector3(0, -0.5, 0), AnchorUp, Lest, false, false);
	cable->setFriction(1);
	cable->setUseBending(true);
	cable->setUseCollision(true);
	cable->setUseLRA(true);
	cable->getCollisionShape()->setMargin(margin);
	cable->setCollisionMargin(margin);
	cable->setCollisionResponseActive(true);
	//cable->setCollisionStiffness(0, 1000, 0, 1);
	cable->setCollisionParameters(3, 6);

	cable->m_anchors[0].m_bodyMassRatio = 0.1;
	cable->m_anchors[1].m_bodyMassRatio = 0.1;

	pdemo->SetCameraPosition(btVector3(0, 2, -3));
}

static void Init_Test1000Nodes(CableDemo* pdemo)
{
	// Resolution's cable
	int resolution = 1000;
	int iterations = 100;
	btScalar margin = 0.001;

	// Shape
	btCollisionShape* shape = new btSphereShape(1);

	btTransform t = btTransform();

	btVector3 groundPos = btVector3(0, -5, 0);
	btTransform transformGround = btTransform();
	transformGround.setIdentity();
	transformGround.setOrigin(groundPos);

	btRigidBody* ground = pdemo->createRigidBody(0, transformGround, new btBoxShape(btVector3(500, 2, 500)));

	btTransform transformRight = btTransform();
	transformRight.setIdentity();
	transformRight.setOrigin(btVector3(500, 2, 0));

	btRigidBody* bodyRightAnchor = pdemo->createRigidBody(0, transformRight, new btBoxShape(btVector3(1, 1, 1)));

	btTransform transformLeft = btTransform();
	transformLeft.setIdentity();
	transformLeft.setOrigin(btVector3(-500, 2, 0));

	btRigidBody* bodyLeftAnchor = pdemo->createRigidBody(0, transformLeft, new btBoxShape(btVector3(1, 1, 1)));

	btAlignedObjectArray<btVector3> waypointPos = btAlignedObjectArray<btVector3>();
	waypointPos.push_back(transformRight.getOrigin() + btVector3(0, 0.5, 1.5));  // point de départ
	waypointPos.push_back(btVector3(0, 2.1, 1.5));
	//waypointPos.push_back(transformLeft.getOrigin() + btVector3(0, 0.75, 1.5));
	waypointPos.push_back(transformLeft.getOrigin() + btVector3(0, 0.5, 1.5));  // Arrivée

	btTransform te = btTransform();
	te.setIdentity();
	te.setOrigin(btVector3(0, 1.9, -1));
	auto p = te.getRotation();
	p.setRotation(btVector3(1, 0, 0), 1);
	te.setRotation(p);
	btRigidBody* ta = pdemo->createRigidBody(0, te, new btBoxShape(btVector3(8, 1.5, 1)));

	btCable* cable = pdemo->createCableWaypoint(resolution, iterations, 5, waypointPos, bodyLeftAnchor, bodyRightAnchor, true, true);

	//btCable* cable = pdemo->createCable(resolution, iterations, 1, transformRight.getOrigin() + btVector3(0, 0, 1.1), transformLeft.getOrigin() + btVector3(0, 0, 1.1), bodyLeftAnchor, bodyRightAnchor, false, false);
	//btCable* cable = pdemo->createCable(resolution, iterations, 1, transformRight.getOrigin() + btVector3(0, 1.1, 0), transformLeft.getOrigin() + btVector3(0, 1.1, 0), nullptr, nullptr, false, false);

	cable->setUseCollision(false);
	cable->getCollisionShape()->setMargin(margin);
	cable->setUseLRA(false);
	cable->setCollisionParameters(1, 2);
	cable->setCollisionMargin(margin);
	pdemo->SetCameraPosition(btVector3(0, 0.5, 5));
}

/// ---------------------------------------------------------------------------
/// 1.  Helpers: a tiny latitude/longitude tessellator
/// ---------------------------------------------------------------------------
static void pushTriangle(std::vector<int>& idx,
						 int a, int b, int c)
{
	idx.push_back(a);
	idx.push_back(b);
	idx.push_back(c);
}

// returns vertex & index buffers for a unit-radius sphere
static void buildSphereMesh(std::vector<btScalar>& verts,
							std::vector<int>& indices,
							int stacks = 18,
							int slices = 36)
{
	verts.clear();
	indices.clear();

	// generate vertices
	for (int i = 0; i <= stacks; ++i)
	{
		btScalar v = btScalar(i) / stacks;    // 0 … 1
		btScalar phi = (v - 0.5f) * SIMD_PI;  // -π/2 … π/2

		for (int j = 0; j <= slices; ++j)
		{
			btScalar u = btScalar(j) / slices;  // 0 … 1
			btScalar theta = u * SIMD_2_PI;     // 0 … 2π

			btScalar x = btCos(theta) * btCos(phi);
			btScalar y = btSin(phi);
			btScalar z = btSin(theta) * btCos(phi);

			verts.push_back(y);
			verts.push_back(x);
			verts.push_back(z);
		}
	}

	// generate indices (two tris per quad)
	int vertsPerRow = slices + 1;
	for (int i = 0; i < stacks; ++i)
	{
		for (int j = 0; j < slices; ++j)
		{
			int a = i * vertsPerRow + j;
			int b = i * vertsPerRow + j + 1;
			int c = (i + 1) * vertsPerRow + j + 1;
			int d = (i + 1) * vertsPerRow + j;

			pushTriangle(indices, a, b, c);
			pushTriangle(indices, a, c, d);
		}
	}
}

/// ---------------------------------------------------------------------------
/// 2.  Wrap those raw arrays in a btIndexedMesh
/// ---------------------------------------------------------------------------
static btTriangleIndexVertexArray*
createTIVAfromIndexedMesh(std::vector<btScalar>& verts,
						  std::vector<int>& indices)
{
	btIndexedMesh mesh;
	mesh.m_numTriangles = static_cast<int>(indices.size() / 3);
	mesh.m_triangleIndexBase = reinterpret_cast<const unsigned char*>(indices.data());
	mesh.m_triangleIndexStride = 3 * sizeof(int);
	mesh.m_indexType = PHY_INTEGER;

	mesh.m_numVertices = static_cast<int>(verts.size() / 3);
	mesh.m_vertexBase = reinterpret_cast<const unsigned char*>(verts.data());
	mesh.m_vertexStride = 3 * sizeof(btScalar);
	mesh.m_vertexType = PHY_DOUBLE;

	// Bullet owns the btIndexedMesh values,
	// but *you* must keep the underlying arrays alive!
	auto* tiva = new btTriangleIndexVertexArray();
	tiva->addIndexedMesh(mesh, mesh.m_indexType);

	return tiva;
}

/// ---------------------------------------------------------------------------
/// 3.  Factory: radius  →  ready-to-use GImpact mesh shape
/// ---------------------------------------------------------------------------
btGImpactMeshShape* makeSphereGImpact(float radius,
									  int stacks = 18,
									  int slices = 36)
{
	static std::vector<btScalar> vertices;
	static std::vector<int> indices;

	buildSphereMesh(vertices, indices, stacks, slices);

	// scale vertices in-place (unit sphere → desired radius)
	for (size_t i = 0; i < vertices.size(); ++i) vertices[i] *= radius;

	btTriangleIndexVertexArray* tiva = createTIVAfromIndexedMesh(vertices, indices);

	btGImpactMeshShape* shape = new btGImpactMeshShape(tiva);
	shape->setMargin(0);
	shape->updateBound();

	return shape;
}

static void Init_TestCollisionCableSphere(CableDemo* pdemo)
{
	// Resolution's cable
	int resolution = 100;
	int iterations = 50;
	btScalar margin = 0.01;

	/*
	//btCollisionShape* shape = new btSphereShape(1);
	btCollisionShape* shape = new btBoxShape(btVector3(1,1,1));
	
	btCompoundShape* compound = new btCompoundShape();
	btTransform localA = btTransform();
	localA.setIdentity();
	localA.setOrigin(btVector3(0, 0, 0));
	compound->addChildShape(localA, shape);
	
	
	btTransform t = btTransform();
	t.setIdentity();
	t.setOrigin(btVector3(0, 3.75, 0));
	btRigidBody* spheres = pdemo->createRigidBody(1000, t, compound);
	*/

	// Shape
	//btCollisionShape* shape = new btBoxShape(btVector3(1, 1, 1));
	btCollisionShape* shape = new btSphereShape(1);
	btCompoundShape* compound = new btCompoundShape();

	btTransform localA = btTransform();
	localA.setIdentity();
	localA.setOrigin(btVector3(0, 1, 0.80));

	btTransform localB = btTransform();
	localB.setIdentity();
	localB.setOrigin(btVector3(0, 1, -0.80));

	compound->addChildShape(localA, shape);
	compound->addChildShape(localB, shape);

	btTransform t = btTransform();
	t.setIdentity();
	t.setOrigin(btVector3(0, 4, 0));

	btRigidBody* spheres = pdemo->createCableRigidBody(100000, t, compound);
	spheres->setSleepingThresholds(0, 0);

	btVector3 groundPos = btVector3(0, -15, 0);
	btTransform transformGround = btTransform();
	transformGround.setIdentity();
	transformGround.setOrigin(groundPos);
	//btRigidBody* ground = pdemo->createRigidBody(0, transformGround, new btBoxShape(btVector3(50, 2, 50)));

	// Masses
	btTransform transformRight = btTransform();
	transformRight.setIdentity();
	transformRight.setOrigin(btVector3(4, 3, 0));
	btRigidBody* bodyRightAnchor = pdemo->createRigidBody(0, transformRight, new btBoxShape(btVector3(1, 1, 1)));

	btTransform transformLeft = btTransform();
	transformLeft.setIdentity();
	transformLeft.setOrigin(btVector3(-4, 1, 0));
	btRigidBody* bodyLeftAnchor = pdemo->createRigidBody(0, transformLeft, new btBoxShape(btVector3(1, 1, 1)));

	btCable* cable = pdemo->createCable(resolution, iterations, 3, transformRight.getOrigin(), transformLeft.getOrigin(), bodyLeftAnchor, bodyRightAnchor, true, true);

	//cable->appendAnchor(10, spheres, true);
	spheres->setFriction(1);
	cable->setFriction(0);

	cable->setUseCollision(true);
	cable->getCollisionShape()->setMargin(margin);
	cable->setUseLRA(true);
	//cable->setCollisionStiffness(0, 100000, 0, 1);
	cable->setCollisionParameters(3, 6);
	cable->setCollisionMargin(margin);
	pdemo->SetCameraPosition(btVector3(0, 3.5, 0));
}

static void Init_TestCollisionFallingA18Constraint(CableDemo* pdemo)
{
	// Walls
	btTransform transformBlockA = btTransform();
	transformBlockA.setIdentity();
	transformBlockA.setOrigin(btVector3(6.1, -5, 0));
	btRigidBody* BlockA = pdemo->createRigidBody(0, transformBlockA, new btBoxShape(btVector3(1, 10, 10)));

	btTransform transformBlockB = btTransform();
	transformBlockB.setIdentity();
	transformBlockB.setOrigin(btVector3(-6.1, -5, 0));
	btRigidBody* BlockB = pdemo->createRigidBody(0, transformBlockB, new btBoxShape(btVector3(1, 10, 10)));

	// Pillar
	btTransform Lest = btTransform();
	Lest.setIdentity();
	Lest.setOrigin(btVector3(0, 6.5, 2.8));
	btRigidBody* LestBody = pdemo->createRigidBody(1000000000, Lest, new btBoxShape(btVector3(10, 0.5, 0.5)), 1234);

	// Blue Cube
	btTransform trAnchorUp = btTransform();
	trAnchorUp.setIdentity();
	trAnchorUp.setOrigin(btVector3(0, 10, 0));
	btRigidBody* anchorUp = pdemo->createRigidBody(0, trAnchorUp, new btBoxShape(btVector3(0.5, 0.5, 0.5)));

	// Ground
	btVector3 groundPos = btVector3(0, -15, 0);
	btTransform transformGround = btTransform();
	transformGround.setIdentity();
	transformGround.setOrigin(groundPos);
	btRigidBody* ground = pdemo->createRigidBody(0, transformGround, new btBoxShape(btVector3(50, 2, 50)));

	// A18
	btTransform trA18 = btTransform();
	trA18.setIdentity();
	trA18.setOrigin(btVector3(0, 1, 0));
	btRigidBody* a18 = pdemo->createRigidBody(704, trA18, new btBoxShape(btVector3(4, 5, 0.5)));
	a18->setMaxLinearVelocity(30);
	a18->setSleepingThresholds(0, 0);

	// 1. Geometry ----------------------------------------------------------------
	static const btVector3 kHalfExtents(4.0f, 0.5f, 0.5f);
	btBoxShape* boxClaw = new btBoxShape(kHalfExtents);

	static const btVector3 kCubeVerts[8] = {
		{-kHalfExtents.x(), -kHalfExtents.y(), -kHalfExtents.z()},  // 0
		{kHalfExtents.x(), -kHalfExtents.y(), -kHalfExtents.z()},   // 1
		{kHalfExtents.x(), kHalfExtents.y(), -kHalfExtents.z()},    // 2
		{-kHalfExtents.x(), kHalfExtents.y(), -kHalfExtents.z()},   // 3
		{-kHalfExtents.x(), -kHalfExtents.y(), kHalfExtents.z()},   // 4
		{kHalfExtents.x(), -kHalfExtents.y(), kHalfExtents.z()},    // 5
		{kHalfExtents.x(), kHalfExtents.y(), kHalfExtents.z()},     // 6
		{-kHalfExtents.x(), kHalfExtents.y(), kHalfExtents.z()}     // 7
	};

	// 12 triangles (two per face)
	static const unsigned int kCubeIdx[36] = {
		0, 2, 1, 2, 0, 3,  // −Z
		4, 5, 6, 6, 7, 4,  // +Z
		0, 5, 4, 5, 0, 1,  // −Y
		3, 6, 2, 6, 3, 7,  // +Y
		1, 6, 5, 6, 1, 2,  // +X
		0, 7, 3, 7, 0, 4   // −X
	};

	// 2. Fill a btIndexedMesh
	btIndexedMesh mesh;
	mesh.m_numTriangles = 12;
	mesh.m_triangleIndexBase = reinterpret_cast<const unsigned char*>(kCubeIdx);
	mesh.m_triangleIndexStride = 3 * sizeof(unsigned int);
	mesh.m_numVertices = 8;
	mesh.m_vertexBase = reinterpret_cast<const unsigned char*>(kCubeVerts);
	mesh.m_vertexStride = sizeof(btVector3);
	mesh.m_indexType = PHY_INTEGER;  // int indices
	mesh.m_vertexType = PHY_DOUBLE;

	static btTriangleIndexVertexArray triArray;
	triArray.addIndexedMesh(mesh, PHY_INTEGER);  // stride chosen above

	// 4-m long, 0.5-m diameter
	btTriangleMesh* cylMesh = buildCylinderMesh(8.0f, 1.0f, 12, 1);

	btCompoundShape* compoundCylShape = new btCompoundShape(false);
	compoundCylShape->setMargin(0.0);

	btTransform trCylinder = btTransform::getIdentity();
	btCylinderShapeX* cylShape = new btCylinderShapeX(btVector3(4.0f, 0.5, 0.5));
	cylShape->setMargin(0.0);

	trCylinder.setOrigin(btVector3(-1, 0, 0));
	trCylinder.setRotation(btQuaternion(btVector3(0, 0, 1), 0.25));
	compoundCylShape->addChildShape(trCylinder, cylShape);
	trCylinder.setOrigin(btVector3(1, 0, 0));
	trCylinder.setRotation(btQuaternion(btVector3(0, 0, 1), -0.25));
	compoundCylShape->addChildShape(trCylinder, cylShape);

	// 3. Create the GImpact shape
	btGImpactMeshShape* gimpactShape = new btGImpactMeshShape(cylMesh);
	gimpactShape->updateBound();
	gimpactShape->setMargin(0.0);

	// Claw
	btTransform trClaw = btTransform();
	trClaw.setIdentity();
	trClaw.setOrigin(btVector3(0, 7.5, 0));

	btRigidBody* claw = pdemo->createRigidBody(0, trClaw, cylShape, 159);
	claw->m_redirectionTarget = a18;
	a18->m_Children.push_back(claw);
	claw->m_localTransform = btTransform(btMatrix3x3::getIdentity(), btVector3(0, 6.5, 0));

	btRigidBody* clawCC = pdemo->createCableRigidBody(0, trClaw, cylShape, 123456);
	a18->m_cableCollision = clawCC;
	clawCC->m_localTransform = btTransform(btMatrix3x3::getIdentity(), btVector3(0, 6.5, 0));
	clawCC->m_redirectionTarget = a18;

	// Cable's Waypoints
	btAlignedObjectArray<btVector3> waypointPos = btAlignedObjectArray<btVector3>();
	waypointPos.push_back(Lest.getOrigin() + btVector3(0, 0, 0));
	waypointPos.push_back(Lest.getOrigin() + btVector3(0, 0, -3.5));  // point de départ
	waypointPos.push_back(Lest.getOrigin() + btVector3(0, 3, -3.5));  // Arrivée
	waypointPos.push_back(trAnchorUp.getOrigin());                    // Arrivée

	// Resolution's cable
	int resolution = 80;
	int iterations = 40;
	btScalar margin = 0.01;

	// Cable
	btCable* cable = pdemo->createCableWaypoint(resolution, iterations, 1, waypointPos, LestBody, anchorUp, true, true);
	cable->setFriction(1);
	cable->setUseLRA(true);
	cable->setUseBending(false);
	cable->getCollisionShape()->setMargin(margin);

	cable->setUseCollision(true);
	cable->setCollisionMargin(margin);
	cable->setCollisionParameters(5, 10);
	//cable->setCollisionStiffness(0, 50000, 0, 1);

	//cable->setCollisionMode(1);

	for (int i = 0; i < cable->m_anchors.size(); ++i)
	{
		cable->m_anchors[i].m_bodyMassRatio = 0.000015;
	}
	pdemo->SetCameraPosition(btVector3(0, 10, 0));

	pdemo->m_cable = cable;

	///register GIMPACT algorithm
	btGImpactCollisionAlgorithm::registerAlgorithm(pdemo->m_dispatcher);
}

static void Init_TestCollisionCableConvexHullOnMeshSphere(CableDemo* pdemo)
{
	//load our obj mesh
	const char* fileName = "duck.obj";

	//const char* fileName = "bunny.obj";
	char relativeFileName[1024];
	if (b3ResourcePath::findResourcePath(fileName, relativeFileName, 1024, 0))
	{
		char pathPrefix[1024];
		b3FileUtils::extractPath(relativeFileName, pathPrefix, 1024);
	}

	b3BulletDefaultFileIO fileIO;
	GLInstanceGraphicsShape* glmesh = LoadMeshFromObj(relativeFileName, "", &fileIO);

	// BtGImpact
	// Create arrays to hold the vertex and index data
	btIndexedMesh indexedMesh;
	indexedMesh.m_vertexType = PHY_FLOAT;

	// Set index data
	indexedMesh.m_numTriangles = glmesh->m_numIndices / 3;  // Each triangle has 3 indices
	indexedMesh.m_triangleIndexBase = (const unsigned char*)(&glmesh->m_indices->at(0));
	indexedMesh.m_triangleIndexStride = 3 * sizeof(int);  // Each triangle uses 3 indices

	// Set vertex data
	indexedMesh.m_numVertices = glmesh->m_numvertices;
	indexedMesh.m_vertexBase = (const unsigned char*)(&glmesh->m_vertices->at(0));
	indexedMesh.m_vertexStride = 9 * sizeof(float);  // Each vertex has 9 floats (x, y, z, w, nx, ny, nz, u, v)

	// Create a btTriangleIndexVertexArray and add the indexed mesh to it
	btTriangleIndexVertexArray* triangleArray = new btTriangleIndexVertexArray();
	triangleArray->addIndexedMesh(indexedMesh, PHY_INTEGER);
	btGImpactMeshShape* shape = new btGImpactMeshShape(triangleArray);
	shape->setMargin(0);
	shape->setLocalScaling(btVector3(10, 1.5, 1.5));
	shape->updateBound();

	btCompoundShape* compound = new btCompoundShape();
	compound->addChildShape(btTransform::getIdentity(), shape);

	btVector3 duckPosition = btVector3(0, 3.5, 0);
	btTransform transformDuck = btTransform();
	transformDuck.setIdentity();
	transformDuck.setOrigin(duckPosition);
	transformDuck.setRotation(btQuaternion(btVector3(0, 1, 0), SIMD_PI / 2.0) + btQuaternion(btVector3(0, 0, 1), SIMD_PI / 5.0));
	btRigidBody* obstacle = pdemo->createCableRigidBody(0, transformDuck, shape);
	obstacle->setSleepingThresholds(0, 0);
	obstacle->setDamping(0.1, 0.1);

	btVector3 groundPos = btVector3(0, -22, 0);
	btTransform transformGround = btTransform();
	transformGround.setIdentity();
	transformGround.setOrigin(groundPos);
	btRigidBody* ground = pdemo->createRigidBody(0, transformGround, new btBoxShape(btVector3(50, 20, 50)));
	ground->setRestitution(0);

	btTransform transformRight = btTransform();
	transformRight.setIdentity();
	transformRight.setOrigin(btVector3(5, 7, 0));
	btRigidBody* bodyRightAnchor = pdemo->createRigidBody(0, transformRight, new btBoxShape(btVector3(1, 1, 1)));

	btTransform transformLeft = btTransform();
	transformLeft.setIdentity();
	transformLeft.setOrigin(btVector3(-5, 7, 0));
	btRigidBody* bodyLeftAnchor = pdemo->createRigidBody(10, transformLeft, new btBoxShape(btVector3(1, 1, 1)));
	bodyLeftAnchor->setRestitution(0);

	// Resolution's cable
	int resolution = 75;
	int iterations = 100;
	btScalar margin = 0.01;

	btCable* cable = pdemo->createCable(resolution, iterations, 1, transformRight.getOrigin(), transformLeft.getOrigin() + btVector3(1, 0, 0), bodyLeftAnchor, bodyRightAnchor);
	cable->setUseCollision(true);
	cable->getCollisionShape()->setMargin(margin / 2.0);
	cable->setUseLRA(true);
	cable->setCollisionMargin(margin);
	cable->setCollisionParameters(1, 2);

	pdemo->SetCameraPosition(btVector3(0, 0.5, 0));

	///register GIMPACT algorithm
	btGImpactCollisionAlgorithm::registerAlgorithm(pdemo->m_dispatcher);
}

static void Init_TestCollisionRingBox(CableDemo* pdemo)
{
	// Shape
	btCollisionShape* cubeShape = new btBoxShape(btVector3(1, 1, 1));
	// Resolution's cable
	int resolution = 100;
	int iterations = 100;
	btScalar margin = 0.01;

	btTransform pos = btTransform();
	pos.setIdentity();
	pos.setOrigin(btVector3(0, 0, 0));
	btScalar mass = 1;
	// Create sphere ring

	btTransform a18 = btTransform();
	a18.setIdentity();
	a18.setOrigin(btVector3(-0.80, 0, 0));

	btTransform boxPositionB = btTransform();
	boxPositionB.setIdentity();
	boxPositionB.setOrigin(btVector3(0, 0.80, 0));

	btTransform boxPositionC = btTransform();
	boxPositionC.setIdentity();
	boxPositionC.setOrigin(btVector3(0.80, 0, 0));

	btTransform boxPositionD = btTransform();
	boxPositionD.setIdentity();
	boxPositionD.setOrigin(btVector3(0, -0.80, 0));

	btCompoundShape* ringShape = new btCompoundShape();

	btBoxShape* a = new btBoxShape(btVector3(0.5, 1, 1));
	btBoxShape* b = new btBoxShape(btVector3(1, 0.5, 1));
	btBoxShape* c = new btBoxShape(btVector3(0.5, 1, 1));
	btBoxShape* d = new btBoxShape(btVector3(1, 0.5, 1));

	a->setMargin(0);
	b->setMargin(0);
	c->setMargin(0);
	d->setMargin(0);

	ringShape->addChildShape(a18, a);
	ringShape->addChildShape(boxPositionB, b);
	ringShape->addChildShape(boxPositionC, c);
	ringShape->addChildShape(boxPositionD, d);

	btRigidBody* ring = pdemo->createCableRigidBody(100, pos, ringShape);
	ring->updateInertiaTensor();

	btVector3 groundPos = btVector3(0, -10, 0);
	btTransform transformGround = btTransform();
	transformGround.setIdentity();
	transformGround.setOrigin(groundPos);
	btRigidBody* ground = pdemo->createRigidBody(0, transformGround, new btBoxShape(btVector3(50, 2, 50)));

	btTransform transformRight = btTransform();
	transformRight.setIdentity();
	transformRight.setOrigin(btVector3(0, 0, 5));
	btRigidBody* bodyRightAnchor = pdemo->createRigidBody(0, transformRight, new btBoxShape(btVector3(0.5, 0.5, 0.5)));

	btTransform transformLeft = btTransform();
	transformLeft.setIdentity();
	transformLeft.setOrigin(btVector3(0, 0, -5));
	btRigidBody* bodyLeftAnchor = pdemo->createRigidBody(0, transformLeft, new btBoxShape(btVector3(0.5, 0.5, 0.5)));

	btAlignedObjectArray<btVector3> waypointPos = btAlignedObjectArray<btVector3>();
	waypointPos.push_back(transformRight.getOrigin() + btVector3(0, 0, 0));  // point de départ
	waypointPos.push_back(transformLeft.getOrigin() + btVector3(0, 0, 0));   // Arrivée

	btCable* cable = pdemo->createCableWaypoint(resolution, iterations, 3, waypointPos, bodyRightAnchor, bodyLeftAnchor, true, true);
	//cable->appendAnchor(50, box,true);

	ring->setFriction(1);
	cable->setFriction(1);

	cable->setUseCollision(true);
	cable->getCollisionShape()->setMargin(margin);
	cable->setCollisionMargin(margin);
	cable->setUseLRA(true);
	cable->setCollisionParameters(2, 6);
	pdemo->SetCameraPosition(btVector3(0, -3, 0));
}

static void Init_TestCollisionRingSphere(CableDemo* pdemo)
{
	// Shape
	btCollisionShape* cubeShape = new btBoxShape(btVector3(1, 1, 1));

	// Resolution's cable
	int resolution = 50;
	int iterations = 100;
	btScalar margin = 0.01;

	btAlignedObjectArray<btRigidBody*> sphere = btAlignedObjectArray<btRigidBody*>();

	btTransform pos = btTransform();
	pos.setIdentity();
	pos.setOrigin(btVector3(0, 0, 0));
	btScalar mass = 1;
	// Create sphere ring

	btTransform a18 = btTransform();
	a18.setIdentity();
	a18.setOrigin(btVector3(-0.80, 0, 0));

	btTransform spherePositionB = btTransform();
	spherePositionB.setIdentity();
	spherePositionB.setOrigin(btVector3(0, 0.80, 0));

	btTransform spherePositionC = btTransform();
	spherePositionC.setIdentity();
	spherePositionC.setOrigin(btVector3(0.80, 0, 0));

	btTransform spherePositionD = btTransform();
	spherePositionD.setIdentity();
	spherePositionD.setOrigin(btVector3(0, -0.80, 0));

	btCompoundShape* a18shape = new btCompoundShape();

	btSphereShape* a = new btSphereShape(0.7);
	btSphereShape* b = new btSphereShape(0.7);
	btSphereShape* c = new btSphereShape(0.7);
	btSphereShape* d = new btSphereShape(0.7);

	a->setMargin(0);
	b->setMargin(0);
	c->setMargin(0);
	d->setMargin(0);

	a18shape->addChildShape(a18, a);
	a18shape->addChildShape(spherePositionB, b);
	a18shape->addChildShape(spherePositionC, c);
	a18shape->addChildShape(spherePositionD, d);

	btRigidBody* box = pdemo->createCableRigidBody(100, pos, a18shape);
	box->updateInertiaTensor();

	btVector3 groundPos = btVector3(0, -10, 0);
	btTransform transformGround = btTransform();
	transformGround.setIdentity();
	transformGround.setOrigin(groundPos);
	btRigidBody* ground = pdemo->createRigidBody(0, transformGround, new btBoxShape(btVector3(50, 2, 50)));

	btTransform transformRight = btTransform();
	transformRight.setIdentity();
	transformRight.setOrigin(btVector3(0, 0, 5));
	btRigidBody* bodyRightAnchor = pdemo->createRigidBody(0, transformRight, new btBoxShape(btVector3(0.5, 0.5, 0.5)));

	btTransform transformLeft = btTransform();
	transformLeft.setIdentity();
	transformLeft.setOrigin(btVector3(0, 0, -5));
	btRigidBody* bodyLeftAnchor = pdemo->createRigidBody(0, transformLeft, new btBoxShape(btVector3(0.5, 0.5, 0.5)));

	btAlignedObjectArray<btVector3> waypointPos = btAlignedObjectArray<btVector3>();
	waypointPos.push_back(transformRight.getOrigin() + btVector3(0, 0, 0));  // point de départ
	waypointPos.push_back(transformLeft.getOrigin() + btVector3(0, 0, 0));   // Arrivée

	btCable* cable = pdemo->createCableWaypoint(resolution, iterations, 3, waypointPos, bodyRightAnchor, bodyLeftAnchor, true, true);
	//cable->appendAnchor(50, box,true);

	box->setFriction(1);
	cable->setFriction(1);

	cable->setUseCollision(true);
	cable->getCollisionShape()->setMargin(margin);
	cable->setCollisionMargin(margin);
	cable->setUseLRA(true);
	cable->setCollisionParameters(2, 6);
	pdemo->SetCameraPosition(btVector3(0, -3, 0));
}

static void Init_TestCollisionOn1Node(CableDemo* pdemo)
{
	// Shape
	btCollisionShape* cubeShape = new btBoxShape(btVector3(1, 1, 1));
	//btCollisionShape* cubeShape = new btSphereShape(1.15);
	// Resolution's cable
	int resolution = 40;
	int iterations = 50;
	btScalar margin = 0.005;

	btTransform pos = btTransform();
	pos.setIdentity();
	pos.setOrigin(btVector3(0, 0, 0));

	btQuaternion r = btQuaternion();
	r.setRotation(btVector3(0, 0, 1), SIMD_PI * 0.40);
	//r.setRotation(btVector3(0, 0, 1), SIMD_PI * 0.10);

	btTransform t = btTransform();
	t.setIdentity();
	t.setOrigin(btVector3(0, 2, 0));

	btTransform transformCubeA;
	transformCubeA.setIdentity();
	transformCubeA.setRotation(r);
	transformCubeA.setOrigin(btVector3(0.95, 0, 0));

	r.setRotation(btVector3(0, 0, 1), -SIMD_PI * 0.40);

	//r.setRotation(btVector3(0, 0, 1),- SIMD_PI * 0.10);
	btTransform transformCubeB;
	transformCubeB.setIdentity();
	transformCubeB.setRotation(r);
	transformCubeB.setOrigin(btVector3(-0.95, 0, 0));

	btCompoundShape* compoundCubes = new btCompoundShape();

	//btRigidBody* cubA = pdemo->createRigidBody(1, transformCubeA, cubeA);
	//btRigidBody* cubB = pdemo->createRigidBody(1, transformCubeB, cubeB);

	compoundCubes->addChildShape(transformCubeA, cubeShape);
	compoundCubes->addChildShape(transformCubeB, cubeShape);

	btRigidBody* obj = pdemo->createCableRigidBody(100, t, compoundCubes);
	obj->setFriction(1);

	btTransform transformRight = btTransform();
	transformRight.setIdentity();
	transformRight.setOrigin(btVector3(0, 0, 5));
	btRigidBody* bodyRightAnchor = pdemo->createRigidBody(0, transformRight, new btBoxShape(btVector3(0.5, 0.5, 0.5)));

	btTransform transformLeft = btTransform();
	transformLeft.setIdentity();
	transformLeft.setOrigin(btVector3(0, 0, -5));
	btRigidBody* bodyLeftAnchor = pdemo->createRigidBody(0, transformLeft, new btBoxShape(btVector3(0.5, 0.5, 0.5)));

	btAlignedObjectArray<btVector3> waypointPos = btAlignedObjectArray<btVector3>();
	waypointPos.push_back(transformRight.getOrigin() + btVector3(0, 0, 0));  // point de départ
	waypointPos.push_back(transformLeft.getOrigin() + btVector3(0, 0, 0));   // Arrivée

	btCable* cable = pdemo->createCableWaypoint(resolution, iterations, 1, waypointPos, bodyRightAnchor, bodyLeftAnchor, true, true);
	cable->setFriction(1);
	cable->setUseCollision(true);
	cable->getCollisionShape()->setMargin(margin);
	cable->setCollisionMargin(margin);
	cable->setUseLRA(true);
	cable->setCollisionParameters(1, 2);
	//cable->setCollisionStiffness(100, 10000, 0, 0.01);

	pdemo->SetCameraPosition(btVector3(0, -3, 0));
}

static void Init_TestClaw(CableDemo* pdemo)
{
	// Shape
	btCollisionShape* cubeShape = new btBoxShape(btVector3(0.25, 0.25, 0.25));
	// Resolution's cable
	int resolution = 100;
	int iterations = 80;
	btScalar margin = 0.05;

	// compound
	btTransform blocCompound = btTransform();
	blocCompound.setIdentity();
	blocCompound.setOrigin(btVector3(1.05, 5, -15));
	btCompoundShape* compundShape = new btCompoundShape(false);

	btVector3 kHalfExtentsA = btVector3(1, 1, 5);
	btBoxShape* a1 = new btBoxShape(kHalfExtentsA);
	a1->setMargin(0.0);

	btGImpactMeshShape* a2;
	{
		static const btVector3 kCubeVertsA[8] = {
			{-kHalfExtentsA.x(), -kHalfExtentsA.y(), -kHalfExtentsA.z()},  // 0
			{kHalfExtentsA.x(), -kHalfExtentsA.y(), -kHalfExtentsA.z()},   // 1
			{kHalfExtentsA.x(), kHalfExtentsA.y(), -kHalfExtentsA.z()},    // 2
			{-kHalfExtentsA.x(), kHalfExtentsA.y(), -kHalfExtentsA.z()},   // 3
			{-kHalfExtentsA.x(), -kHalfExtentsA.y(), kHalfExtentsA.z()},   // 4
			{kHalfExtentsA.x(), -kHalfExtentsA.y(), kHalfExtentsA.z()},    // 5
			{kHalfExtentsA.x(), kHalfExtentsA.y(), kHalfExtentsA.z()},     // 6
			{-kHalfExtentsA.x(), kHalfExtentsA.y(), kHalfExtentsA.z()}     // 7
		};

		// 12 triangles (two per face)
		static const unsigned int kCubeIdxA[36] = {
			0, 2, 1, 2, 0, 3,  // −Z
			4, 5, 6, 6, 7, 4,  // +Z
			0, 5, 4, 5, 0, 1,  // −Y
			3, 6, 2, 6, 3, 7,  // +Y
			1, 6, 5, 6, 1, 2,  // +X
			0, 7, 3, 7, 0, 4   // −X
		};

		// 2. Fill a btIndexedMesh
		btIndexedMesh mesh;
		mesh.m_numTriangles = 12;
		mesh.m_triangleIndexBase = reinterpret_cast<const unsigned char*>(kCubeIdxA);
		mesh.m_triangleIndexStride = 3 * sizeof(unsigned int);
		mesh.m_numVertices = 8;
		mesh.m_vertexBase = reinterpret_cast<const unsigned char*>(kCubeVertsA);
		mesh.m_vertexStride = sizeof(btVector3);
		mesh.m_indexType = PHY_INTEGER;  // int indices
		mesh.m_vertexType = PHY_DOUBLE;

		static btTriangleIndexVertexArray triArrayA;
		triArrayA.addIndexedMesh(mesh, PHY_INTEGER);  // stride chosen above

		// 3. Create the GImpact shape
		a2 = new btGImpactMeshShape(&triArrayA);
		a2->updateBound();
		a2->setMargin(0.0);
	}

	btTransform t = btTransform();
	t.setIdentity();
	t.setOrigin(btVector3(0, 0, 0));
	compundShape->addChildShape(t, a1);

	btTransform y = btTransform();
	y.setIdentity();
	y.setOrigin(btVector3(-1.5, 0, 4));

	// Claw
	btVector3 kHalfExtentsB = btVector3(3, 0.25, 0.25);
	btBoxShape* b1 = new btBoxShape(kHalfExtentsB);
	b1->setMargin(0.0);

	btGImpactMeshShape* b2;
	{
		static const btVector3 kCubeVertsB[8] = 
		{
			{-kHalfExtentsB.x(), -kHalfExtentsB.y(), -kHalfExtentsB.z()},  // 0
			{kHalfExtentsB.x(), -kHalfExtentsB.y(), -kHalfExtentsB.z()},   // 1
			{kHalfExtentsB.x(), kHalfExtentsB.y(), -kHalfExtentsB.z()},    // 2
			{-kHalfExtentsB.x(), kHalfExtentsB.y(), -kHalfExtentsB.z()},   // 3
			{-kHalfExtentsB.x(), -kHalfExtentsB.y(), kHalfExtentsB.z()},   // 4
			{kHalfExtentsB.x(), -kHalfExtentsB.y(), kHalfExtentsB.z()},    // 5
			{kHalfExtentsB.x(), kHalfExtentsB.y(), kHalfExtentsB.z()},     // 6
			{-kHalfExtentsB.x(), kHalfExtentsB.y(), kHalfExtentsB.z()}     // 7
		};

		// 12 triangles (two per face)
		static const unsigned int kCubeIdxB[36] = 
		{
			0, 2, 1, 2, 0, 3,  // −Z
			4, 5, 6, 6, 7, 4,  // +Z
			0, 5, 4, 5, 0, 1,  // −Y
			3, 6, 2, 6, 3, 7,  // +Y
			1, 6, 5, 6, 1, 2,  // +X
			0, 7, 3, 7, 0, 4   // −X
		};

		// 2. Fill a btIndexedMesh
		btIndexedMesh mesh;
		mesh.m_numTriangles = 12;
		mesh.m_triangleIndexBase = reinterpret_cast<const unsigned char*>(kCubeIdxB);
		mesh.m_triangleIndexStride = 3 * sizeof(unsigned int);
		mesh.m_numVertices = 8;
		mesh.m_vertexBase = reinterpret_cast<const unsigned char*>(kCubeVertsB);
		mesh.m_vertexStride = sizeof(btVector3);
		mesh.m_indexType = PHY_INTEGER;  // int indices
		mesh.m_vertexType = PHY_DOUBLE;

		static btTriangleIndexVertexArray triArrayB;
		triArrayB.addIndexedMesh(mesh, PHY_INTEGER);  // stride chosen above

		// 3. Create the GImpact shape
		b2 = new btGImpactMeshShape(&triArrayB);
		b2->updateBound();
		b2->setMargin(0.0);
	}

	btQuaternion rotation = btQuaternion(0, 0, 0);
	y.setRotation(rotation);
	compundShape->addChildShape(y, b1);

	btRigidBody* obj = pdemo->createCableRigidBody(100, blocCompound, compundShape);
	obj->setGravity(btVector3(0, 0, 100));
	//obj->setAngularFactor(btVector3(0, 0, 0));
	//obj->setAngularVelocity(btVector3(0, 5, 0));

	btTransform LestTransform = btTransform();
	LestTransform.setIdentity();
	LestTransform.setOrigin(btVector3(0, 0, 0));
	btRigidBody* LestBody = pdemo->createRigidBody(0, LestTransform, new btBoxShape(btVector3(0.1, 0.1, 0.1)));

	btTransform trAnchorUp = btTransform();
	trAnchorUp.setIdentity();
	trAnchorUp.setOrigin(btVector3(0, 10, 0));
	btRigidBody* anchorUp = pdemo->createRigidBody(0, trAnchorUp, new btBoxShape(btVector3(1, 1, 1)));

	btAlignedObjectArray<btVector3> waypointPos = btAlignedObjectArray<btVector3>();
	waypointPos.push_back(LestTransform.getOrigin() + btVector3(0, 0.1, 0));
	waypointPos.push_back(trAnchorUp.getOrigin() + btVector3(0, -1, 0));  // Arrivée

	btCable* cable = pdemo->createCableWaypoint(resolution, iterations, 1, waypointPos, LestBody, anchorUp, true, true);
	cable->setUseLRA(false);
	cable->setUseBending(false);
	cable->setUseCollision(true);
	cable->setCableRadius(margin);
	cable->setCollisionMargin(margin);
	cable->setCollisionParameters(3, 6);
	// cable->setCollisionStiffness(0, 1000, 0, 1);
	cable->getCollisionShape()->setMargin(margin);

	///register GIMPACT algorithm
	btGImpactCollisionAlgorithm::registerAlgorithm(pdemo->m_dispatcher);

	pdemo->m_cable = cable;
}

static void Init_Growth(CableDemo* pdemo)
{
	// Shape
	btCollisionShape* boxShape = new btBoxShape(btVector3(0.5, 0.5, 0.5));

	// Masses
	btScalar massKinematic(0);
	btScalar massPhysic(1);

	// Rotation
	btQuaternion rotation(0, 0, 0, 1);

	// Transform
	btTransform transformKinematic;
	transformKinematic.setIdentity();
	transformKinematic.setRotation(rotation);

	btTransform transformPhysic;
	transformPhysic.setIdentity();
	transformPhysic.setRotation(rotation);

	// Resolution's cable
	int resolution = 20;
	int iteration = 100;

	// Create cube and cable
	massPhysic = 22000;

	// Positions
	btVector3 positionKinematic(0, 10, 0);
	btVector3 positionPhysic(0, 4, 0);
	transformKinematic.setOrigin(positionKinematic);
	transformPhysic.setOrigin(positionPhysic);

	// Create the rigidbodys
	btRigidBody* kinematic = pdemo->createRigidBody(massKinematic, transformKinematic, boxShape);
	btRigidBody* physic = pdemo->createRigidBody(massPhysic, transformPhysic, boxShape);

	// Anchor's positions
	btVector3 anchorPositionKinematic = positionKinematic - btVector3(0, -0.5, 0);
	btVector3 anchorPositionPhysic = positionPhysic + btVector3(0, 0.5, 0);

	btCable* cable = pdemo->createCable(resolution, iteration, 25, anchorPositionKinematic, anchorPositionPhysic, physic, kinematic);
	cable->setCollisionParameters(1, 2);
}

static void Init_DetachA18(CableDemo* pdemo)
{
	/// Create enlacedObject:
	btVector3 enlacedObjectPosition = btVector3(0, 0, -4);
	btTransform enlacedObjectTransform = btTransform::getIdentity();
	enlacedObjectTransform.setOrigin(enlacedObjectPosition);
	btCollisionShape* enlacedObjectShape = new btBoxShape(btVector3(0.5, 0.5, 0.5));
	enlacedObjectShape->setMargin(0.0);
	btRigidBody* enlacedObjectBody = pdemo->createCableRigidBody(0, enlacedObjectTransform, enlacedObjectShape);
	enlacedObjectBody->setSleepingThresholds(0, 0);

	/// Create topBox:
	btVector3 boxPosition = btVector3(0, 0, 0);
	btVector3 halfExtends = btVector3(1, 1, 1);
	btTransform boxTransform = btTransform::getIdentity();
	boxTransform.setOrigin(boxPosition);
	btCollisionShape* boxShape = new btBoxShape(halfExtends);
	boxShape->setMargin(0.0);
	btRigidBody* boxBody = pdemo->createCableRigidBody(0, boxTransform, boxShape);

	/// Create Cable:
	int resolution = 100;
	int iterations = 100;
	btScalar margin = 0.01;
	btAlignedObjectArray<btVector3> waypointPos = btAlignedObjectArray<btVector3>();
	waypointPos.push_back(boxPosition + btVector3(0, halfExtends.getY(), -halfExtends.getZ()));   // Start
	waypointPos.push_back(enlacedObjectPosition + btVector3(0, 1, -1));                           //
	waypointPos.push_back(enlacedObjectPosition + btVector3(0, -1, -1));                          //
	waypointPos.push_back(boxPosition + btVector3(0, -halfExtends.getY(), -halfExtends.getZ()));  // End
	btCable* cable = pdemo->createCableWaypoint(resolution, iterations, 1, waypointPos, boxBody, boxBody, true, true);
	cable->setUseLRA(true);
	cable->setUseCollision(true);
	cable->setCableRadius(margin);
	cable->setCollisionMargin(margin);
	cable->setCollisionParameters(3, 6);
	cable->getCollisionShape()->setMargin(margin);

	// cable->setCollisionParameters(3, 3, 0);
	// cable->setCollisionViscosity(20);
	// double dataX[5] = {0, 0.001, 0.01, 0.5, 1};
	// double dataY[5] = {0, 1, 100000, 5000000, 10000000};
	// cable->updateCurveResponse(dataX, dataY, 5);
	// cable->setCollisionMode(1);
}

static void Init_MCMVCable(CableDemo* pdemo)
{
	/// MCMV
	// Shape: MCMV
	btCompoundShape* mcmvShape = new btCompoundShape(false);
	mcmvShape->setMargin(0);
	btGImpactMeshShape* gImpactShape;
	btBoxShape* mcmvBoxShape;
	{
		// Shape: MCMV - Floor
		{
			static const btVector3 kHalfExtents(2.5, 0.5, 5);

			static const btVector3 kCubeVerts[8] = {
				{-kHalfExtents.x(), -kHalfExtents.y(), -kHalfExtents.z()},  // 0
				{kHalfExtents.x(), -kHalfExtents.y(), -kHalfExtents.z()},   // 1
				{kHalfExtents.x(), kHalfExtents.y(), -kHalfExtents.z()},    // 2
				{-kHalfExtents.x(), kHalfExtents.y(), -kHalfExtents.z()},   // 3
				{-kHalfExtents.x(), -kHalfExtents.y(), kHalfExtents.z()},   // 4
				{kHalfExtents.x(), -kHalfExtents.y(), kHalfExtents.z()},    // 5
				{kHalfExtents.x(), kHalfExtents.y(), kHalfExtents.z()},     // 6
				{-kHalfExtents.x(), kHalfExtents.y(), kHalfExtents.z()}     // 7
			};

			static const unsigned int kCubeIdx[36] = {
				0, 2, 1, 2, 0, 3,  // −Z
				4, 5, 6, 6, 7, 4,  // +Z
				0, 5, 4, 5, 0, 1,  // −Y
				3, 6, 2, 6, 3, 7,  // +Y
				1, 6, 5, 6, 1, 2,  // +X
				0, 7, 3, 7, 0, 4   // −X
			};

			btIndexedMesh mesh;
			mesh.m_numTriangles = 12;
			mesh.m_triangleIndexBase = reinterpret_cast<const unsigned char*>(kCubeIdx);
			mesh.m_triangleIndexStride = 3 * sizeof(unsigned int);
			mesh.m_numVertices = 8;
			mesh.m_vertexBase = reinterpret_cast<const unsigned char*>(kCubeVerts);
			mesh.m_vertexStride = sizeof(btVector3);
			mesh.m_indexType = PHY_INTEGER;
			mesh.m_vertexType = PHY_DOUBLE;

			static btTriangleIndexVertexArray triArray;
			triArray.addIndexedMesh(mesh, PHY_INTEGER);

			btGImpactMeshShape* floorShape = new btGImpactMeshShape(&triArray);
			floorShape->updateBound();
			floorShape->setMargin(0);
			gImpactShape = floorShape;

			btBoxShape* floorBoxShape = new btBoxShape(kHalfExtents);
			floorBoxShape->setMargin(0);
			mcmvBoxShape = floorBoxShape;

			btBvhTriangleMeshShape* floorBvhShape = new btBvhTriangleMeshShape(floorShape->getMeshInterface(), true);
			floorBvhShape->setMargin(0);

			btTransform floorTransform = btTransform();
			floorTransform.setIdentity();
			floorTransform.setOrigin(btVector3(0, 0, 0));
			mcmvShape->addChildShape(floorTransform, floorBoxShape);
		}

		// Shape: MCMV - Wall
		{
			static const btVector3 kHalfExtents(2.5, 2, 0.5);

			static const btVector3 kCubeVerts[8] = 
			{
				{-kHalfExtents.x(), -kHalfExtents.y(), -kHalfExtents.z()},  // 0
				{kHalfExtents.x(), -kHalfExtents.y(), -kHalfExtents.z()},   // 1
				{kHalfExtents.x(), kHalfExtents.y(), -kHalfExtents.z()},    // 2
				{-kHalfExtents.x(), kHalfExtents.y(), -kHalfExtents.z()},   // 3
				{-kHalfExtents.x(), -kHalfExtents.y(), kHalfExtents.z()},   // 4
				{kHalfExtents.x(), -kHalfExtents.y(), kHalfExtents.z()},    // 5
				{kHalfExtents.x(), kHalfExtents.y(), kHalfExtents.z()},     // 6
				{-kHalfExtents.x(), kHalfExtents.y(), kHalfExtents.z()}     // 7
			};

			static const unsigned int kCubeIdx[36] = 
			{
				0, 2, 1, 2, 0, 3,  // −Z
				4, 5, 6, 6, 7, 4,  // +Z
				0, 5, 4, 5, 0, 1,  // −Y
				3, 6, 2, 6, 3, 7,  // +Y
				1, 6, 5, 6, 1, 2,  // +X
				0, 7, 3, 7, 0, 4   // −X
			};

			btIndexedMesh mesh;
			{
				mesh.m_numTriangles = 12;
				mesh.m_triangleIndexBase = reinterpret_cast<const unsigned char*>(kCubeIdx);
				mesh.m_triangleIndexStride = 3 * sizeof(unsigned int);
				mesh.m_numVertices = 8;
				mesh.m_vertexBase = reinterpret_cast<const unsigned char*>(kCubeVerts);
				mesh.m_vertexStride = sizeof(btVector3);
				mesh.m_indexType = PHY_INTEGER;
				mesh.m_vertexType = PHY_DOUBLE;
			}

			static btTriangleIndexVertexArray triArray;
			triArray.addIndexedMesh(mesh, PHY_INTEGER);

			btGImpactMeshShape* wallShape = new btGImpactMeshShape(&triArray);
			wallShape->setMargin(0);
			wallShape->updateBound();

			btBoxShape* wallBoxShape = new btBoxShape(kHalfExtents);
			wallBoxShape->setMargin(0);

			btBvhTriangleMeshShape* wallBvhShape = new btBvhTriangleMeshShape(wallShape->getMeshInterface(), true);
			wallShape->setMargin(0);

			btTransform wallTransform = btTransform();
			wallTransform.setIdentity();
			wallTransform.setOrigin(btVector3(0, 1, 4));
			//wallTransform.setRotation(btQuaternion(0, 10, 0));
			mcmvShape->addChildShape(wallTransform, wallBoxShape);
		}

		// Shape: MCMV - Corner
		{
			static const btVector3 kHalfExtents(2.5, 0.25, 0.25);

			static const btVector3 kCubeVerts[6] = 
			{
				{-kHalfExtents.x(), -kHalfExtents.y(), kHalfExtents.z()},   // 0
				{-kHalfExtents.x(), -kHalfExtents.y(), -kHalfExtents.z()},  // 1
				{kHalfExtents.x(), -kHalfExtents.y(), -kHalfExtents.z()},   // 2
				{kHalfExtents.x(), -kHalfExtents.y(), kHalfExtents.z()},    // 3
				{-kHalfExtents.x(), kHalfExtents.y(), kHalfExtents.z()},    // 4
				{kHalfExtents.x(), kHalfExtents.y(), kHalfExtents.z()}      // 5
			};

			static const unsigned int kCubeIdx[24] = 
			{
				0, 1, 2, 0, 2, 3,
				0, 4, 1, 3, 2, 5,
				1, 4, 5, 1, 5, 2,
				3, 5, 4, 3, 4, 0
			};

			btIndexedMesh mesh;
			mesh.m_numTriangles = 8;
			mesh.m_triangleIndexBase = reinterpret_cast<const unsigned char*>(kCubeIdx);
			mesh.m_triangleIndexStride = 3 * sizeof(unsigned int);
			mesh.m_numVertices = 6;
			mesh.m_vertexBase = reinterpret_cast<const unsigned char*>(kCubeVerts);
			mesh.m_vertexStride = sizeof(btVector3);
			mesh.m_indexType = PHY_INTEGER;  // int indices
			mesh.m_vertexType = PHY_DOUBLE;

			static btTriangleIndexVertexArray triArray;
			triArray.addIndexedMesh(mesh, PHY_INTEGER);  // stride chosen above

			btGImpactMeshShape* gimpactShape = new btGImpactMeshShape(&triArray);
			gimpactShape->setMargin(0);
			gimpactShape->updateBound();

			btTransform cornerTransform = btTransform();
			cornerTransform.setIdentity();
			cornerTransform.setOrigin(btVector3(0, 0.5, 4.0));
			//mcmvShape->addChildShape(cornerTransform, gimpactShape);
		}
	}
	// Transform: MCMV
	btTransform mcmvTransform = btTransform();
	mcmvTransform.setIdentity();
	// Rigid: MCMV
	btRigidBody* mcmvBody = pdemo->createCableRigidBody(0, mcmvTransform, mcmvShape);

	/// Left Arm
	// Shape: Left Arm
	btBoxShape* leftArmShape = new btBoxShape(btVector3(0.1, 0.1, 0.35));
	leftArmShape->setMargin(0);
	// Transform: Left Arm
	btTransform leftArmTransform = btTransform();
	leftArmTransform.setIdentity();
	leftArmTransform.setOrigin(btVector3(2.5, 3, 10));
	// Rigid: Left Arm
	btRigidBody* leftArmBody = pdemo->createRigidBody(0, leftArmTransform, leftArmShape);

	/// Right Arm
	// Shape: Right Arm
	btBoxShape* rightArmShape = new btBoxShape(btVector3(0.1, 0.1, 0.35));
	rightArmShape->setMargin(0);
	// Transform: Right Arm
	btTransform rightArmTransform = btTransform();
	rightArmTransform.setIdentity();
	rightArmTransform.setOrigin(btVector3(0.5, 1.5, 2.5));
	// Rigid: Right Arm
	btRigidBody* rightArmBody = pdemo->createRigidBody(0, rightArmTransform, rightArmShape);

	/// Cable
	// Resolution's cable
	int resolution = 200;
	int iterations = 50;
	btScalar margin = 0.01;
	btAlignedObjectArray<btVector3> waypointPos = btAlignedObjectArray<btVector3>();
	waypointPos.push_back(btVector3(0.5, 1.5, 2.5));
	waypointPos.push_back(btVector3(2.75, 1, 2.75));
	waypointPos.push_back(btVector3(3, 2.5, 9));
	waypointPos.push_back(btVector3(2.5, 3, 10));
	btCable* mcmvCable = pdemo->createCableWaypoint(resolution, iterations, 1, waypointPos, rightArmBody, leftArmBody, true, true);
	// Parameters
	mcmvCable->setUseLRA(true);
	mcmvCable->setUseCollision(true);
	mcmvCable->setCableRadius(margin);
	mcmvCable->setCollisionMargin(margin);
	mcmvCable->getCollisionShape()->setMargin(margin);
	mcmvCable->setCollisionParameters(5, 5);
	
	pdemo->m_cable = mcmvCable;

	// Register GIMPACT algorithm
	btGImpactCollisionAlgorithm::registerAlgorithm(pdemo->m_dispatcher);
}

btScalar posX = 0;
btScalar posY = 0;
btScalar margin = 0;

void OnPosXChanged(float value, void* userPtr)
{
	CableDemo* pdemo = (CableDemo*)userPtr;
	pdemo->posX = btScalar(value);
}

void OnPosYChanged(float value, void* userPtr)
{
	CableDemo* pdemo = (CableDemo*)userPtr;
	pdemo->posY = btScalar(value);
}

void OnMarginChanged(float value, void* userPtr)
{
	CableDemo* pdemo = (CableDemo*)userPtr;
	pdemo->margin = btScalar(value);
}

static void Init_RayCast(CableDemo* pdemo)
{
	btVector3 position = btVector3(2, 9, 0);
	btScalar halfExtend = 0.5;
	btVector3 halfExtends = btVector3(halfExtend, halfExtend, halfExtend);
	btTransform trShape = btTransform::getIdentity();
	btCompoundShape* compoundShape = new btCompoundShape(false);
	compoundShape->setMargin(0.0);

	// BoxShape
	trShape.setOrigin(position);
	{
		btBoxShape* shape1 = new btBoxShape(halfExtends);
		shape1->setMargin(0.0);

		btBoxShape* shape2 = new btBoxShape(halfExtends);
		shape2->setMargin(0.0);
		// trShape.setOrigin(position + btVector3(0, 0.495, 0));

		btCompoundShape* shape = new btCompoundShape(false);
		shape->setMargin(0.0);
		shape->addChildShape(btTransform(btQuaternion(), btVector3(   0, 0.510, 0)), shape1);
		// shape->addChildShape(btTransform(btQuaternion(), btVector3(-0.5, 0.005, 0)), shape2);

		btRigidBody* rb = pdemo->createRigidBody(10, trShape, shape2);
		rb->setGravity(btVector3(0, 0, 0));
		rb->setSleepingThresholds(0, 0);
	}
	position.setY(position.getY() - 2.0);

	// SphereShape
	trShape.setOrigin(position);
	{
		btSphereShape* shape = new btSphereShape(halfExtend);
		shape->setMargin(0.0);

		btRigidBody* rb = pdemo->createRigidBody(10, trShape, shape);
		rb->setGravity(btVector3(0, 0, 0));
		rb->setSleepingThresholds(0, 0);
	}
	position.setY(position.getY() - 2.0);

	// CylinderShape
	trShape.setOrigin(position);
	{
		btCylinderShape* shape = new btCylinderShape(halfExtends);
		shape->setMargin(0.0);

		btRigidBody* rb = pdemo->createRigidBody(10, trShape, shape);
		rb->setGravity(btVector3(0, 0, 0));
		rb->setSleepingThresholds(0, 0);
	}
	position.setY(position.getY() - 2.0);

	// ConvexHullShape
	trShape.setOrigin(position);
	{
		btConvexHullShape* shape = new btConvexHullShape();
		shape->setMargin(0.0);
		const char* fileName = "cube.obj";
		char relativeFileName[1024];
		if (b3ResourcePath::findResourcePath(fileName, relativeFileName, 1024, 0))
		{
			char pathPrefix[1024];
			b3FileUtils::extractPath(relativeFileName, pathPrefix, 1024);
		}

		b3BulletDefaultFileIO fileIO;
		GLInstanceGraphicsShape* glmesh = LoadMeshFromObj(relativeFileName, "", &fileIO);

		for (int i = 0; i < glmesh->m_numvertices; i++)
		{
			const GLInstanceVertex& v = glmesh->m_vertices->at(i);
			float temp = v.xyzw[0];
			btVector3 vtx(v.xyzw[0], v.xyzw[1], v.xyzw[2]);
			shape->addPoint(vtx);
		}
		shape->initializePolyhedralFeatures();
		shape->optimizeConvexHull();

		btRigidBody* rb = pdemo->createRigidBody(10, trShape, shape);
		rb->setGravity(btVector3(0, 0, 0));
		rb->setSleepingThresholds(0, 0);
	}
	position.setY(position.getY() - 2.0);

	// GImpactShape
	trShape.setOrigin(position);
	{
		static const btVector3 kCubeVerts[8] = {
			{-halfExtends.x(), -halfExtends.y(), -halfExtends.z()},  // 0
			{halfExtends.x(), -halfExtends.y(), -halfExtends.z()},   // 1
			{halfExtends.x(), halfExtends.y(), -halfExtends.z()},    // 2
			{-halfExtends.x(), halfExtends.y(), -halfExtends.z()},   // 3
			{-halfExtends.x(), -halfExtends.y(), halfExtends.z()},   // 4
			{halfExtends.x(), -halfExtends.y(), halfExtends.z()},    // 5
			{halfExtends.x(), halfExtends.y(), halfExtends.z()},     // 6
			{-halfExtends.x(), halfExtends.y(), halfExtends.z()}     // 7
		};

		static const unsigned int kCubeIdx[36] = {
			0, 2, 1, 2, 0, 3,  // −Z
			4, 5, 6, 6, 7, 4,  // +Z
			0, 5, 4, 5, 0, 1,  // −Y
			3, 6, 2, 6, 3, 7,  // +Y
			1, 6, 5, 6, 1, 2,  // +X
			0, 7, 3, 7, 0, 4   // −X
		};

		btIndexedMesh mesh;
		mesh.m_numTriangles = 12;
		mesh.m_triangleIndexBase = reinterpret_cast<const unsigned char*>(kCubeIdx);
		mesh.m_triangleIndexStride = 3 * sizeof(unsigned int);
		mesh.m_numVertices = 8;
		mesh.m_vertexBase = reinterpret_cast<const unsigned char*>(kCubeVerts);
		mesh.m_vertexStride = sizeof(btVector3);
		mesh.m_indexType = PHY_INTEGER;
		mesh.m_vertexType = PHY_DOUBLE;

		static btTriangleIndexVertexArray triArray;
		triArray.addIndexedMesh(mesh, PHY_INTEGER);

		btGImpactMeshShape* shape = new btGImpactMeshShape(&triArray);
		shape->updateBound();
		shape->setMargin(0.0);

		btRigidBody* rb = pdemo->createRigidBody(10, trShape, shape);
		rb->setGravity(btVector3(0, 0, 0));
		rb->setSleepingThresholds(0, 0);
	}
	position.setY(position.getY() - 2.0);

	// CompoundShape_BoxShape
	trShape.setOrigin(position);
	{
		btBoxShape* shape = new btBoxShape(halfExtends);
		shape->setMargin(0.0);

		compoundShape->addChildShape(trShape, shape);
	}
	position.setY(position.getY() - 2.0);

	// CompoundShape_SphereShape
	trShape.setOrigin(position);
	{
		btSphereShape* shape = new btSphereShape(halfExtend);
		shape->setMargin(0.0);

		compoundShape->addChildShape(trShape, shape);
	}
	position.setY(position.getY() - 2.0);

	// CompoundShape_CylinderShape
	trShape.setOrigin(position);
	{
		btCylinderShape* shape = new btCylinderShape(halfExtends);
		shape->setMargin(0.0);

		compoundShape->addChildShape(trShape, shape);
	}
	position.setY(position.getY() - 2.0);

	// CompoundShape_ConvexHullShape
	trShape.setOrigin(position);
	{
		btConvexHullShape* shape = new btConvexHullShape();
		shape->setMargin(0.0);
		const char* fileName = "cube.obj";
		char relativeFileName[1024];
		if (b3ResourcePath::findResourcePath(fileName, relativeFileName, 1024, 0))
		{
			char pathPrefix[1024];
			b3FileUtils::extractPath(relativeFileName, pathPrefix, 1024);
		}

		b3BulletDefaultFileIO fileIO;
		GLInstanceGraphicsShape* glmesh = LoadMeshFromObj(relativeFileName, "", &fileIO);

		for (int i = 0; i < glmesh->m_numvertices; i++)
		{
			const GLInstanceVertex& v = glmesh->m_vertices->at(i);
			float temp = v.xyzw[0];
			btVector3 vtx(v.xyzw[0] + trShape.getOrigin().getX(), v.xyzw[1] + trShape.getOrigin().getY(), v.xyzw[2] + trShape.getOrigin().getZ());
			shape->addPoint(vtx);
		}
		shape->initializePolyhedralFeatures();
		shape->optimizeConvexHull();

		compoundShape->addChildShape(btTransform::getIdentity(), shape);
	}
	position.setY(position.getY() - 2.0);

	// CompoundShape_GImpactShape
	trShape.setOrigin(position);
	{
		static const btVector3 kCubeVerts[8] = {
			{-halfExtends.x(), -halfExtends.y(), -halfExtends.z()},  // 0
			{halfExtends.x(), -halfExtends.y(), -halfExtends.z()},   // 1
			{halfExtends.x(), halfExtends.y(), -halfExtends.z()},    // 2
			{-halfExtends.x(), halfExtends.y(), -halfExtends.z()},   // 3
			{-halfExtends.x(), -halfExtends.y(), halfExtends.z()},   // 4
			{halfExtends.x(), -halfExtends.y(), halfExtends.z()},    // 5
			{halfExtends.x(), halfExtends.y(), halfExtends.z()},     // 6
			{-halfExtends.x(), halfExtends.y(), halfExtends.z()}     // 7
		};

		static const unsigned int kCubeIdx[36] = {
			0, 2, 1, 2, 0, 3,  // −Z
			4, 5, 6, 6, 7, 4,  // +Z
			0, 5, 4, 5, 0, 1,  // −Y
			3, 6, 2, 6, 3, 7,  // +Y
			1, 6, 5, 6, 1, 2,  // +X
			0, 7, 3, 7, 0, 4   // −X
		};

		btIndexedMesh mesh;
		mesh.m_numTriangles = 12;
		mesh.m_triangleIndexBase = reinterpret_cast<const unsigned char*>(kCubeIdx);
		mesh.m_triangleIndexStride = 3 * sizeof(unsigned int);
		mesh.m_numVertices = 8;
		mesh.m_vertexBase = reinterpret_cast<const unsigned char*>(kCubeVerts);
		mesh.m_vertexStride = sizeof(btVector3);
		mesh.m_indexType = PHY_INTEGER;
		mesh.m_vertexType = PHY_DOUBLE;

		static btTriangleIndexVertexArray triArray;
		triArray.addIndexedMesh(mesh, PHY_INTEGER);

		btGImpactMeshShape* shape = new btGImpactMeshShape(&triArray);
		shape->setMargin(0.0);
		shape->updateBound();

		compoundShape->addChildShape(trShape, shape);
	}
	position.setY(position.getY() - 2.0);

	btRigidBody* rbCompound = pdemo->createRigidBody(10, btTransform::getIdentity(), compoundShape);
	rbCompound->setGravity(btVector3(0, 0, 0));
	rbCompound->setSleepingThresholds(0, 0);

	SliderParams sliderStartingXRay("Position X", &pdemo->posX);
	sliderStartingXRay.m_userPointer = pdemo;
	sliderStartingXRay.m_minVal = 0;
	sliderStartingXRay.m_maxVal = 4;
	sliderStartingXRay.m_callback = OnPosXChanged;

	SliderParams sliderStartingYRay("Position Y", &pdemo->posY);
	sliderStartingYRay.m_userPointer = pdemo;
	sliderStartingYRay.m_minVal = 0;
	sliderStartingYRay.m_maxVal = 4;
	sliderStartingYRay.m_callback = OnPosYChanged;

	pdemo->getGUIHelper()->getParameterInterface()->registerSliderFloatParameter(sliderStartingXRay);
	pdemo->getGUIHelper()->getParameterInterface()->registerSliderFloatParameter(sliderStartingYRay);

	pdemo->posX = posX;
	pdemo->posY = posY;

	// Register GIMPACT algorithm
	btGImpactCollisionAlgorithm::registerAlgorithm(pdemo->m_dispatcher);
}

static void Init_Collision(CableDemo* pdemo)
{
	btVector3 position = btVector3(2, 9, 0);
	btScalar halfExtend = 0.5;
	btVector3 halfExtends = btVector3(halfExtend, halfExtend, halfExtend);
	btTransform trShape = btTransform::getIdentity();
	btCompoundShape* compoundShape = new btCompoundShape(false);
	compoundShape->setMargin(0.0);

	// BoxShape
	trShape.setOrigin(position);
	{
		btBoxShape* shape = new btBoxShape(halfExtends);
		shape->setMargin(0.0);

		btRigidBody* rb = pdemo->createRigidBody(10, trShape, shape);
		rb->setGravity(btVector3(0, 0, 0));
		rb->setSleepingThresholds(0, 0);
	}
	position.setY(position.getY() - 2.0);

	// SphereShape
	trShape.setOrigin(position);
	{
		btSphereShape* shape = new btSphereShape(halfExtend);
		shape->setMargin(0.0);

		btRigidBody* rb = pdemo->createRigidBody(10, trShape, shape);
		rb->setGravity(btVector3(0, 0, 0));
		rb->setSleepingThresholds(0, 0);
	}
	position.setY(position.getY() - 2.0);

	// CylinderShape
	trShape.setOrigin(position);
	{
		btCylinderShape* shape = new btCylinderShape(halfExtends);
		shape->setMargin(0.0);

		btRigidBody* rb = pdemo->createRigidBody(10, trShape, shape);
		rb->setGravity(btVector3(0, 0, 0));
		rb->setSleepingThresholds(0, 0);
	}
	position.setY(position.getY() - 2.0);

	// ConvexHullShape
	trShape.setOrigin(position);
	{
		btConvexHullShape* shape = new btConvexHullShape();
		shape->setMargin(0.0);
		const char* fileName = "cube.obj";
		char relativeFileName[1024];
		if (b3ResourcePath::findResourcePath(fileName, relativeFileName, 1024, 0))
		{
			char pathPrefix[1024];
			b3FileUtils::extractPath(relativeFileName, pathPrefix, 1024);
		}

		b3BulletDefaultFileIO fileIO;
		GLInstanceGraphicsShape* glmesh = LoadMeshFromObj(relativeFileName, "", &fileIO);

		for (int i = 0; i < glmesh->m_numvertices; i++)
		{
			const GLInstanceVertex& v = glmesh->m_vertices->at(i);
			float temp = v.xyzw[0];
			btVector3 vtx(v.xyzw[0], v.xyzw[1], v.xyzw[2]);
			shape->addPoint(vtx);
		}
		shape->initializePolyhedralFeatures();
		shape->optimizeConvexHull();

		btRigidBody* rb = pdemo->createRigidBody(10, trShape, shape);
		rb->setGravity(btVector3(0, 0, 0));
		rb->setSleepingThresholds(0, 0);
	}
	position.setY(position.getY() - 2.0);

	// GImpactShape
	trShape.setOrigin(position);
	{
		static const btVector3 kCubeVerts[8] = {
			{-halfExtends.x(), -halfExtends.y(), -halfExtends.z()},  // 0
			{halfExtends.x(), -halfExtends.y(), -halfExtends.z()},   // 1
			{halfExtends.x(), halfExtends.y(), -halfExtends.z()},    // 2
			{-halfExtends.x(), halfExtends.y(), -halfExtends.z()},   // 3
			{-halfExtends.x(), -halfExtends.y(), halfExtends.z()},   // 4
			{halfExtends.x(), -halfExtends.y(), halfExtends.z()},    // 5
			{halfExtends.x(), halfExtends.y(), halfExtends.z()},     // 6
			{-halfExtends.x(), halfExtends.y(), halfExtends.z()}     // 7
		};

		static const unsigned int kCubeIdx[36] = {
			0, 2, 1, 2, 0, 3,  // −Z
			4, 5, 6, 6, 7, 4,  // +Z
			0, 5, 4, 5, 0, 1,  // −Y
			3, 6, 2, 6, 3, 7,  // +Y
			1, 6, 5, 6, 1, 2,  // +X
			0, 7, 3, 7, 0, 4   // −X
		};

		btIndexedMesh mesh;
		mesh.m_numTriangles = 12;
		mesh.m_triangleIndexBase = reinterpret_cast<const unsigned char*>(kCubeIdx);
		mesh.m_triangleIndexStride = 3 * sizeof(unsigned int);
		mesh.m_numVertices = 8;
		mesh.m_vertexBase = reinterpret_cast<const unsigned char*>(kCubeVerts);
		mesh.m_vertexStride = sizeof(btVector3);
		mesh.m_indexType = PHY_INTEGER;
		mesh.m_vertexType = PHY_DOUBLE;

		static btTriangleIndexVertexArray triArray;
		triArray.addIndexedMesh(mesh, PHY_INTEGER);

		btGImpactMeshShape* shape = new btGImpactMeshShape(&triArray);
		shape->updateBound();
		shape->setMargin(0.0);

		btRigidBody* rb = pdemo->createRigidBody(10, trShape, shape);
		rb->setGravity(btVector3(0, 0, 0));
		rb->setSleepingThresholds(0, 0);
	}
	position.setY(position.getY() - 2.0);

	// CompoundShape_BoxShape
	trShape.setOrigin(position);
	{
		btBoxShape* shape = new btBoxShape(btVector3(halfExtend, halfExtend * 2, halfExtend));
		shape->setMargin(0.0);

		trShape.setOrigin(btVector3(position.getX(), position.getY(), position.getZ()));
		compoundShape->addChildShape(trShape, shape);

		shape = new btBoxShape(btVector3(halfExtend * 2, halfExtend, halfExtend));
		shape->setMargin(0.0);

		trShape.setOrigin(btVector3(position.getX() - 0.5, position.getY() - 1, position.getZ()));
		// trShape.setRotation(btQuaternion(btVector3(0,0,1), -3.0 / 6.0));
		compoundShape->addChildShape(trShape, shape);
		trShape.setRotation(btQuaternion());
	}
	position.setY(position.getY() - 2.0);

	// CompoundShape_SphereShape
	trShape.setOrigin(position);
	{
		btSphereShape* shape = new btSphereShape(halfExtend);
		shape->setMargin(0.0);

		// compoundShape->addChildShape(trShape, shape);
	}
	position.setY(position.getY() - 2.0);

	// CompoundShape_CylinderShape
	trShape.setOrigin(position);
	{
		btCylinderShape* shape = new btCylinderShape(halfExtends);
		shape->setMargin(0.0);

		// compoundShape->addChildShape(trShape, shape);
	}
	position.setY(position.getY() - 2.0);

	// CompoundShape_ConvexHullShape
	trShape.setOrigin(position);
	{
		btConvexHullShape* shape = new btConvexHullShape();
		shape->setMargin(0.0);
		const char* fileName = "cube.obj";
		char relativeFileName[1024];
		if (b3ResourcePath::findResourcePath(fileName, relativeFileName, 1024, 0))
		{
			char pathPrefix[1024];
			b3FileUtils::extractPath(relativeFileName, pathPrefix, 1024);
		}

		b3BulletDefaultFileIO fileIO;
		GLInstanceGraphicsShape* glmesh = LoadMeshFromObj(relativeFileName, "", &fileIO);

		for (int i = 0; i < glmesh->m_numvertices; i++)
		{
			const GLInstanceVertex& v = glmesh->m_vertices->at(i);
			float temp = v.xyzw[0];
			btVector3 vtx(v.xyzw[0] + trShape.getOrigin().getX(), v.xyzw[1] + trShape.getOrigin().getY(), v.xyzw[2] + trShape.getOrigin().getZ());
			shape->addPoint(vtx);
		}
		shape->initializePolyhedralFeatures();
		shape->optimizeConvexHull();

		// compoundShape->addChildShape(btTransform::getIdentity(), shape);
	}
	position.setY(position.getY() - 2.0);

	// CompoundShape_GImpactShape
	trShape.setOrigin(position);
	{
		static const btVector3 kCubeVerts[8] = {
			{-halfExtends.x(), -halfExtends.y(), -halfExtends.z()},  // 0
			{halfExtends.x(), -halfExtends.y(), -halfExtends.z()},   // 1
			{halfExtends.x(), halfExtends.y(), -halfExtends.z()},    // 2
			{-halfExtends.x(), halfExtends.y(), -halfExtends.z()},   // 3
			{-halfExtends.x(), -halfExtends.y(), halfExtends.z()},   // 4
			{halfExtends.x(), -halfExtends.y(), halfExtends.z()},    // 5
			{halfExtends.x(), halfExtends.y(), halfExtends.z()},     // 6
			{-halfExtends.x(), halfExtends.y(), halfExtends.z()}     // 7
		};

		static const unsigned int kCubeIdx[36] = {
			0, 2, 1, 2, 0, 3,  // −Z
			4, 5, 6, 6, 7, 4,  // +Z
			0, 5, 4, 5, 0, 1,  // −Y
			3, 6, 2, 6, 3, 7,  // +Y
			1, 6, 5, 6, 1, 2,  // +X
			0, 7, 3, 7, 0, 4   // −X
		};

		btIndexedMesh mesh;
		mesh.m_numTriangles = 12;
		mesh.m_triangleIndexBase = reinterpret_cast<const unsigned char*>(kCubeIdx);
		mesh.m_triangleIndexStride = 3 * sizeof(unsigned int);
		mesh.m_numVertices = 8;
		mesh.m_vertexBase = reinterpret_cast<const unsigned char*>(kCubeVerts);
		mesh.m_vertexStride = sizeof(btVector3);
		mesh.m_indexType = PHY_INTEGER;
		mesh.m_vertexType = PHY_DOUBLE;

		static btTriangleIndexVertexArray triArray;
		triArray.addIndexedMesh(mesh, PHY_INTEGER);

		btGImpactMeshShape* shape = new btGImpactMeshShape(&triArray);
		shape->setMargin(0.0);
		shape->updateBound();

		// compoundShape->addChildShape(trShape, shape);
	}
	position.setY(position.getY() - 2.0);

	btRigidBody* rbCompound = pdemo->createRigidBody(10, btTransform::getIdentity(), compoundShape);
	rbCompound->setGravity(btVector3(0, 0, 0));
	rbCompound->setSleepingThresholds(0, 0);

	SliderParams sliderStartingXRay("Position X", &pdemo->posX);
	sliderStartingXRay.m_userPointer = pdemo;
	sliderStartingXRay.m_minVal = 0;
	sliderStartingXRay.m_maxVal = 2;
	sliderStartingXRay.m_callback = OnPosXChanged;

	SliderParams sliderStartingYRay("Position Y", &pdemo->posY);
	sliderStartingYRay.m_userPointer = pdemo;
	sliderStartingYRay.m_minVal = -2;
	sliderStartingYRay.m_maxVal = 2;
	sliderStartingYRay.m_callback = OnPosYChanged;

	SliderParams sliderMargin("Margin", &pdemo->margin);
	sliderMargin.m_userPointer = pdemo;
	sliderMargin.m_minVal = 0;
	sliderMargin.m_maxVal = 1;
	sliderMargin.m_callback = OnMarginChanged;

	pdemo->getGUIHelper()->getParameterInterface()->registerSliderFloatParameter(sliderStartingXRay);
	pdemo->getGUIHelper()->getParameterInterface()->registerSliderFloatParameter(sliderStartingYRay);
	pdemo->getGUIHelper()->getParameterInterface()->registerSliderFloatParameter(sliderMargin);

	pdemo->posX = posX;
	pdemo->posY = posY;
	pdemo->margin = margin;

	// Register GIMPACT algorithm
	btGImpactCollisionAlgorithm::registerAlgorithm(pdemo->m_dispatcher);
}

static btPoint2PointConstraint* ballJoint;
static btRigidBody* rod;
btScalar tau = 0.0f;
btScalar damping = 0.0f;
btScalar impClamp = 0.0f;
btScalar rodAngularDamping = 0.0f;
btScalar rodLinearDamping = 0.0f;

void OnRodAngularDampingChanged(float value, void* userPtr)
{
	if (rod)
	{
		rod->setDamping(rod->getLinearDamping(), value);
	}
}

void OnRodLinearDampingChanged(float value, void* userPtr)
{
	if (rod)
	{
		rod->setDamping(value, rod->getAngularDamping());
	}
}

void OnBallJointTauChanged(float value, void* userPtr)
{
	if (ballJoint)
	{
		ballJoint->m_setting.m_tau = value;
	}
}

void OnBallJointDampingChanged(float value, void* userPtr)
{
	if (ballJoint)
	{
		ballJoint->m_setting.m_damping = value;
	}
}

void OnBallJointImpulseClampChanged(float value, void* userPtr)
{
	if (ballJoint)
	{
		ballJoint->m_setting.m_impulseClamp = value;
	}
}

static void Init_BallJoint(CableDemo* pdemo)
{
	// Shape
	btCollisionShape* boxShape = new btBoxShape(btVector3(0.5, 2, 0.5));
	btCollisionShape* ballShape = new btSphereShape(0.2f);

	// Masses
	btScalar massKinematic(0);
	btScalar massPhysic(100);

	// Rotation
	btQuaternion rotation(0, 0, 0, 1);

	// Transform
	btTransform transformPhysic;
	transformPhysic.setIdentity();
	transformPhysic.setRotation(rotation);

	btTransform transformKinematic;
	transformKinematic.setOrigin(btVector3(0, 4, 0));
	transformKinematic.setRotation(rotation);

	// RB
	rod = pdemo->createRigidBody(massPhysic, transformPhysic, boxShape);
	rod->setRestitution(0);
	rod->setFriction(0.5);
	rod->setSleepingThresholds(0, 0);
	rod->setFlags(0);

	btRigidBody* sphere = pdemo->createRigidBody(massKinematic, transformKinematic, ballShape);
	sphere->setRestitution(0);
	sphere->setFriction(0.5);
	sphere->setSleepingThresholds(0, 0);
	sphere->setFlags(0);

	// Constraint
	//  To calculate the locals points:
	//      MatrixLocalA = Identity * LocalPivot
	//      MatrixWorld = LtWA * MatrixLocalA
	//      MatrixLocalA = WtLA * MatrixWorld
	//      MatrixLocalB = WtLB * MatrixWorld
	btTransform framePivot = btTransform::getIdentity();
	btTransform frameWorld = sphere->getWorldTransform() * framePivot;
	btTransform frameInA = sphere->getWorldTransform().inverse() * frameWorld;
	btTransform frameInB = rod->getWorldTransform().inverse() * frameWorld;
	//btPoint2PointConstraint* ballJoint = new btPoint2PointConstraint(*sphere, *rod, frameInA.getOrigin(), frameInB.getOrigin());
	ballJoint = new btPoint2PointConstraint(*sphere, *rod, frameInA.getOrigin(), frameInB.getOrigin());
	ballJoint->setOverrideNumSolverIterations(256);

	// Constraint settings
	ballJoint->m_setting.m_tau = tau;
	ballJoint->m_setting.m_damping = damping;
	ballJoint->m_setting.m_impulseClamp = impClamp;

	SliderParams sliderTau("tau", &tau);
	sliderTau.m_minVal = 0;
	sliderTau.m_maxVal = 1;
	sliderTau.m_callback = OnBallJointTauChanged;

	SliderParams sliderDamp("damping", &damping);
	sliderDamp.m_minVal = 0;
	sliderDamp.m_maxVal = 1;
	sliderDamp.m_callback = OnBallJointDampingChanged;

	SliderParams sliderImp("impulseClamp", &impClamp);
	sliderImp.m_minVal = 0;
	sliderImp.m_maxVal = 1;
	sliderImp.m_callback = OnBallJointImpulseClampChanged;

	SliderParams sliderRodAngularDamping("rod angular damping", &rodAngularDamping);
	sliderRodAngularDamping.m_minVal = 0;
	sliderRodAngularDamping.m_maxVal = 1;
	sliderRodAngularDamping.m_callback = OnRodAngularDampingChanged;

	SliderParams sliderRodLinearDamping("rod linear damping", &rodLinearDamping);
	sliderRodLinearDamping.m_minVal = 0;
	sliderRodLinearDamping.m_maxVal = 1;
	sliderRodLinearDamping.m_callback = OnRodLinearDampingChanged;

	pdemo->getGUIHelper()->getParameterInterface()->registerSliderFloatParameter(sliderTau);
	pdemo->getGUIHelper()->getParameterInterface()->registerSliderFloatParameter(sliderDamp);
	pdemo->getGUIHelper()->getParameterInterface()->registerSliderFloatParameter(sliderImp);
	pdemo->getGUIHelper()->getParameterInterface()->registerSliderFloatParameter(sliderRodAngularDamping);
	pdemo->getGUIHelper()->getParameterInterface()->registerSliderFloatParameter(sliderRodLinearDamping);

	pdemo->getDynamicsWorld()->addConstraint(ballJoint);
}

static void Init_FixedJoint(CableDemo* pdemo)
{
	// Shape
	btCollisionShape* boxShape = new btCylinderShape(btVector3(0.2, 1, 0.2));
	btCollisionShape* cubeShape = new btBoxShape(btVector3(1, 1, 1));
	btCollisionShape* planeShape = new btBoxShape(btVector3(10, 0.1, 10));

	// Masses
	btScalar massKinematic(0);
	btScalar massRod(100);
	btScalar massCube(513.5f);

	// Rotation
	btQuaternion rotation(0, 0, 0, 1);
	btQuaternion rotationRod(0, 0, 1.5708);

	// Transform
	btTransform transformBox;
	transformBox.setOrigin(btVector3(-2.2f, 2, 0));
	transformBox.setRotation(rotationRod);

	btTransform transformCube;
	transformCube.setOrigin(btVector3(0, 2, 0));
	transformCube.setRotation(rotation);

	btTransform transformPlane;
	transformPlane.setOrigin(btVector3(0, 0, 0));
	transformPlane.setRotation(rotation);

	// RB
	btRigidBody* rod = pdemo->createRigidBody(massRod, transformBox, boxShape);
	rod->setMassProps(massRod, btVector3(5.583, 5.583, 0.5f));
	rod->setRestitution(0);
	rod->setFriction(0.5);
	rod->setSleepingThresholds(0, 0);
	rod->setFlags(0);

	btRigidBody* cube = pdemo->createRigidBody(massCube, transformCube, cubeShape);

	cube->setSleepingThresholds(0, 0);
	cube->setFlags(0);

	btRigidBody* plane = pdemo->createRigidBody(massKinematic, transformPlane, planeShape);
	plane->setSleepingThresholds(0, 0);
	plane->setFriction(0.5);
	plane->setFlags(0);

	// Constraint
	//  To calculate the locals points:
	//      MatrixLocalA = Identity * LocalPivot
	//      MatrixWorld = LtWA * MatrixLocalA
	//      MatrixLocalA = WtLA * MatrixWorld
	//      MatrixLocalB = WtLB * MatrixWorld
	btTransform framePivot = btTransform::getIdentity();
	framePivot.setOrigin(btVector3(0, -0.5, 0));
	btTransform frameWorld = rod->getWorldTransform() * framePivot;
	btTransform frameA = rod->getWorldTransform().inverse() * frameWorld;
	btTransform frameB = cube->getWorldTransform().inverse() * frameWorld;
	btFixedConstraint* fixed = pdemo->createFixedConstraint(*rod, *cube, frameA, frameB, 256);
}

static void Init_Stability(CableDemo* pdemo)
{
	// Shapes
	btCollisionShape* updownBoxShape = new btBoxShape(btVector3(0.4, 0.1, 0.1));
	btCollisionShape* lefrightBoxShape = new btBoxShape(btVector3(0.1, 0.4, 0.1));
	btTransform upBoxTransform(btMatrix3x3::getIdentity(), btVector3(0,0.3,0));
	btTransform downBoxTransform(btMatrix3x3::getIdentity(), btVector3(0,-0.3,0));
	btTransform leftBoxTransform(btMatrix3x3::getIdentity(), btVector3(-0.3,0,0));
	btTransform rightBoxTransform(btMatrix3x3::getIdentity(), btVector3(0.3, 0, 0));

	btCompoundShape* ringShape = new btCompoundShape(false);
	ringShape->addChildShape(upBoxTransform, updownBoxShape);
	ringShape->addChildShape(downBoxTransform, updownBoxShape);
	ringShape->addChildShape(leftBoxTransform, lefrightBoxShape);
	ringShape->addChildShape(rightBoxTransform, lefrightBoxShape);

	btBoxShape* a18Shape = new btBoxShape(btVector3(0.6, 2, 0.6));

	// Masses
	btScalar massRingLest(6200);
	btScalar massRingA18(0);
	btScalar massA18(704);

	// Transform
	btTransform transformAttachPoint(btMatrix3x3::getIdentity(), btVector3(0, 7.7, 0));
	btTransform transformRingLest(btMatrix3x3::getIdentity(), btVector3(0, 5, 0));
	btTransform transformRingA18(btQuaternion(btVector3(0,1,0), SIMD_HALF_PI), btVector3(0, 4.7, 0));
	btTransform transformA18(btMatrix3x3::getIdentity(), btVector3(0, 2.35, 0));

	// Attach Point
	btRigidBody* attachPoint = pdemo->createRigidBody(btScalar(1), transformAttachPoint, new btBoxShape(btVector3(0.1,0.1,0.1)));
	attachPoint->setCollisionFlags(2); // attachPoint->getCollisionFlags();
	attachPoint->setSleepingThresholds(0, 0);
	attachPoint->setMassProps(0, btVector3(0,0,0));
	
	// Objet A (Lest)
	btRigidBody* ringLest = pdemo->createRigidBody(massRingLest, transformRingLest, ringShape);
	ringLest->setSleepingThresholds(0, 0);
	//ringLest->updateMassAtImpact(true, massRingLest, 1000, 0.001, 1);

	// Object B (A18's claws)
	btRigidBody* ringA18 = pdemo->createRigidBody(massRingA18, transformRingA18, ringShape);
	ringA18->setSleepingThresholds(0, 0);
	
	// Objet C (A18)
	btRigidBody* a18 = pdemo->createRigidBody(massA18, transformA18, a18Shape);
	a18->setSleepingThresholds(0, 0);
	 
	// Object B's set up
	ringA18->m_redirectionTarget = a18;
	ringA18->m_localTransform = btTransform(btQuaternion(btVector3(0, 1, 0), 3.14 / 2.0), btVector3(0, 2.35, 0));
	a18->m_Children.push_back(ringA18);

	// Object D (ground)
	//btRigidBody* ground = pdemo->createRigidBody(0, btTransform(btQuaternion::getIdentity(), btVector3(0,4,0)), new btBoxShape(btVector3(10,0.2,10)));

	// Cable's Waypoints
	btAlignedObjectArray<btVector3> waypointPos = btAlignedObjectArray<btVector3>();
	waypointPos.push_back(transformRingLest.getOrigin() + btVector3(0,0.3,0));
	waypointPos.push_back(transformAttachPoint.getOrigin());

	// Cable's Parameters
	int resolution = 20;
	int iteration = 100;
	double linearMass = 10;

	// Cable
	btCable* cable = pdemo->createCableWaypoint(resolution, iteration, linearMass, waypointPos, ringLest, attachPoint, true, true);
	cable->getCollisionShape()->setMargin(0.025);
	cable->setCableRadius(0.05);
	cable->setUseBending(false);
	cable->setUseLRA(false);
	cable->m_anchors[0].m_bodyMassRatio = 0.0;
	cable->m_anchors[1].m_bodyMassRatio = 0;
	cable->m_materials[0]->m_kLST = 1.0;
	cable->setDistanceMode(2);
	cable->setUseAnchorConstraintPlacement(false);

	// Add the current cable to allow the user to debug it
	pdemo->m_cable = cable;
}

static void Init_StabilityTension(CableDemo* pdemo)
{
	// Update shared data
	auto& data = StabilityTensionData;
	pdemo->m_stabilityData = &data;

	// Test defaults
	if (data.AutoResetTensionTest)
	{
		// Rigidbodies
		data.A_mass = 100.0;
		data.B_mass = 0.0;
		data.C_mass = 100.0;
		data.Ground_offset = 0.4;
		data.A_MassImpact = 0.0;

		// Anchors
		data.A_massRatio = 0.02;
		data.D_massRatio = 0.0;

		// Distance
		data.Cable_DistanceMode = 0;

		// Cable
		data.Cable_resolution = 5.0;
		data.Cable_iteration = 100.0;
		data.Cable_linearMass = 1.0;
		data.Cable_length = 0.4;
		data.Cable_targetLength = 1.5;
		data.Cable_growSpeed = 0.5;
		data.Cable_LRA = false;
		data.Cable_AnchorPlacement = true;
		data.Cable_massRatioActivationThreshold = 0.0;
	}

	// Shapes
	btCollisionShape* updownBoxShape = new btBoxShape(btVector3(0.4, 0.1, 0.1));
	btCollisionShape* lefrightBoxShape = new btBoxShape(btVector3(0.1, 0.4, 0.1));
	btTransform upBoxTransform(btMatrix3x3::getIdentity(), btVector3(0, 0.3, 0));
	btTransform downBoxTransform(btMatrix3x3::getIdentity(), btVector3(0, -0.3, 0));
	btTransform leftBoxTransform(btMatrix3x3::getIdentity(), btVector3(-0.3, 0, 0));
	btTransform rightBoxTransform(btMatrix3x3::getIdentity(), btVector3(0.3, 0, 0));

	btCompoundShape* ringShape = new btCompoundShape(false);
	ringShape->addChildShape(upBoxTransform, updownBoxShape);
	ringShape->addChildShape(downBoxTransform, updownBoxShape);
	ringShape->addChildShape(leftBoxTransform, lefrightBoxShape);
	ringShape->addChildShape(rightBoxTransform, lefrightBoxShape);

	btScalar waypointHeightShift = 0.3;
	btScalar attachPointHeight = 5.2;
	btScalar LestHeight = attachPointHeight - waypointHeightShift - data.Cable_length;

	btTransform transformAttachPoint(btMatrix3x3::getIdentity(), btVector3(0, attachPointHeight, 0));
	btTransform transformRingLest(btMatrix3x3::getIdentity(), btVector3(0, LestHeight, 0));

	// Objet A (Lest)
	btRigidBody* ringLest = pdemo->createRigidBody(data.A_mass, transformRingLest, ringShape);
	ringLest->setSleepingThresholds(0, 0);
	ringLest->setupMassAtImpact(data.A_mass, 200, 0.0, 0.2);
	ringLest->activeMassAtImpact(data.A_MassImpact > 0.0 ? true : false);

	// Objet D (Cable attach point)
	btRigidBody* attachPoint = pdemo->createRigidBody(btScalar(1), transformAttachPoint, new btBoxShape(btVector3(0.1, 0.1, 0.1)));
	attachPoint->setCollisionFlags(2);  // attachPoint->getCollisionFlags();
	attachPoint->setSleepingThresholds(0, 0);
	attachPoint->setMassProps(0, btVector3(0, 0, 0));

	// Ground
	btRigidBody* ground = pdemo->createRigidBody(0, btTransform(btQuaternion(btVector3(1,0,0), 0.0),
		                                         btVector3(0, LestHeight - data.Ground_offset - 0.4 - 0.2, 0)), 
		                                         new btBoxShape(btVector3(10, 0.2, 10))
	);

	// Cable
	// Waypoints
	btAlignedObjectArray<btVector3> waypointPos = btAlignedObjectArray<btVector3>();
	waypointPos.push_back(transformRingLest.getOrigin() + btVector3(0, waypointHeightShift, 0));
	waypointPos.push_back(transformAttachPoint.getOrigin());

	// Parameters
	btCable* cable = pdemo->createCableWaypoint(data.Cable_resolution, data.Cable_iteration, data.Cable_linearMass, waypointPos, ringLest, attachPoint, true, true);
	cable->getCollisionShape()->setMargin(0.025);
	cable->setCableRadius(0.05);
	cable->setUseBending(false);
	cable->setUseLRA(data.Cable_LRA);
	cable->m_anchors[0].m_bodyMassRatio = data.A_massRatio;
	cable->m_anchors[1].m_bodyMassRatio = data.D_massRatio;
	cable->m_materials[0]->m_kLST = 1.0;
	cable->setDistanceMode(data.Cable_DistanceMode);
	cable->setUseAnchorConstraintPlacement(data.Cable_AnchorPlacement);
	cable->setMassRatioActivationThreshold(data.Cable_massRatioActivationThreshold);
	pdemo->m_cable = cable;

	// User controls
	SliderParams sliderSubsteps("Substeps count (Global)", &globals.substepsCount);
	sliderSubsteps.m_userPointer = pdemo;
	sliderSubsteps.m_minVal = 1;
	sliderSubsteps.m_maxVal = 8;
	sliderSubsteps.m_clampToIntegers = true;
	sliderSubsteps.m_clampToNotches = true;
	pdemo->getGUIHelper()->getParameterInterface()->registerSliderFloatParameter(sliderSubsteps, 1);

	ButtonParams dataSelector("Lock", 0, true);
	dataSelector.m_userPointer = pdemo;
	dataSelector.m_initialState = data.AutoResetTensionTest < 0.5;
	dataSelector.m_callback = [](int buttonId, bool buttonState, void* userPtr)
	{
		if (buttonId != 0)
		{
			return;
		}

		CableDemo* pdemo = (CableDemo*)userPtr;
		pdemo->m_stabilityData->AutoResetTensionTest = buttonState ? 0 : 1;
	};
	pdemo->getGUIHelper()->getParameterInterface()->registerButtonParameter(dataSelector);

	SliderParams sliderCableLength("Initial Length (Cable)", &data.Cable_length);
	btScalar stepCableLength = 0.2;
	sliderCableLength.m_userPointer = pdemo;
	sliderCableLength.m_minVal = 0.2;
	sliderCableLength.m_maxVal = 10 - stepCableLength;
	sliderCableLength.m_clampToIntegers = false;
	sliderCableLength.m_clampToNotches = true;
	pdemo->getGUIHelper()->getParameterInterface()->registerSliderFloatParameter(sliderCableLength, stepCableLength);

	SliderParams sliderCableResolution("Resolution (Cable)", &data.Cable_resolution);
	btScalar stepCableResolution = 1.0;
	sliderCableResolution.m_userPointer = pdemo;
	sliderCableResolution.m_minVal = 2;
	sliderCableResolution.m_maxVal = 200 - stepCableResolution;
	sliderCableResolution.m_clampToIntegers = true;
	sliderCableResolution.m_clampToNotches = true;
	pdemo->getGUIHelper()->getParameterInterface()->registerSliderFloatParameter(sliderCableResolution, stepCableResolution);

	SliderParams sliderGroundOffset("Offset (Ground)", &data.Ground_offset);
	btScalar stepGroundOffset = 0.05;
	sliderGroundOffset.m_userPointer = ground;
	sliderGroundOffset.m_minVal = 0;
	sliderGroundOffset.m_maxVal = 2 - stepGroundOffset;
	sliderGroundOffset.m_clampToIntegers = true;
	sliderGroundOffset.m_clampToNotches = true;
	pdemo->getGUIHelper()->getParameterInterface()->registerSliderFloatParameter(sliderGroundOffset, stepGroundOffset);

	ButtonParams lraSelector("LRA", 1, true);
	lraSelector.m_userPointer = pdemo;
	lraSelector.m_initialState = data.Cable_LRA;
	lraSelector.m_callback = [](int buttonId, bool buttonState, void* userPtr)
	{
		if (buttonId != 1)
		{
			return;
		}

		CableDemo* pdemo = (CableDemo*)userPtr;
		pdemo->m_stabilityData->Cable_LRA = buttonState;
		pdemo->m_cable->setUseLRA(pdemo->m_stabilityData->Cable_LRA);
	};
	pdemo->getGUIHelper()->getParameterInterface()->registerButtonParameter(lraSelector);

	ButtonParams anchorPlacementSelector("AnchorPlacement", 2, true);
	anchorPlacementSelector.m_userPointer = pdemo;
	anchorPlacementSelector.m_initialState = data.Cable_AnchorPlacement;
	anchorPlacementSelector.m_callback = [](int buttonId, bool buttonState, void* userPtr)
	{
		if (buttonId != 2)
		{
			return;
		}

		CableDemo* pdemo = (CableDemo*)userPtr;
		pdemo->m_stabilityData->Cable_AnchorPlacement = buttonState;
		pdemo->m_cable->setUseAnchorConstraintPlacement(pdemo->m_stabilityData->Cable_AnchorPlacement);
	};
	pdemo->getGUIHelper()->getParameterInterface()->registerButtonParameter(anchorPlacementSelector);

	SliderParams sliderMassA("Mass Lest (A)", &data.A_mass);
	btScalar stepMassA = 10;
	sliderMassA.m_userPointer = ringLest;
	sliderMassA.m_minVal = 10;
	sliderMassA.m_maxVal = 10000 - stepMassA;
	sliderMassA.m_clampToIntegers = true;
	sliderMassA.m_clampToNotches = true;
	sliderMassA.m_callback = [](float value, void* userPtr)
	{
		btRigidBody* lest = (btRigidBody*)userPtr;
		lest->setMassProps(value, lest->getLocalInertia() * value * lest->getInvMass());
		lest->updateInertiaTensor();
	};
	pdemo->getGUIHelper()->getParameterInterface()->registerSliderFloatParameter(sliderMassA, stepMassA);

	SliderParams sliderMassImpactA("MassAtImpact (A)", &data.A_MassImpact);
	sliderMassImpactA.m_userPointer = ringLest;
	sliderMassImpactA.m_minVal = 0;
	sliderMassImpactA.m_maxVal = 1;
	sliderMassImpactA.m_clampToIntegers = true;
	sliderMassImpactA.m_clampToNotches = false;
	sliderMassImpactA.m_callback = [](float value, void* userPtr)
	{
		btRigidBody* lest = (btRigidBody*)userPtr;
		lest->activeMassAtImpact(value > 0.0 ? true : false);
	};
	pdemo->getGUIHelper()->getParameterInterface()->registerSliderFloatParameter(sliderMassImpactA, 1);

	SliderParams sliderAMassRatio("MassRatio (A)", &data.A_massRatio);
	btScalar stepAMassRatio = 0.02;
	sliderAMassRatio.m_userPointer = pdemo;
	sliderAMassRatio.m_minVal = 0;
	sliderAMassRatio.m_maxVal = 1 - stepAMassRatio;
	sliderAMassRatio.m_clampToIntegers = false;
	sliderAMassRatio.m_clampToNotches = true;
	sliderAMassRatio.m_callback = [](float value, void* userPtr)
	{
		CableDemo* demo = (CableDemo*)userPtr;
		demo->m_cable->m_anchors[0].m_bodyMassRatio = value;
	};
	pdemo->getGUIHelper()->getParameterInterface()->registerSliderFloatParameter(sliderAMassRatio, stepAMassRatio);

	SliderParams sliderMassRationActivationThreshold("Mass ratio min (Cable)", &data.Cable_massRatioActivationThreshold);
	btScalar stepMassRationActivationThreshold = 0.02;
	sliderMassRationActivationThreshold.m_userPointer = pdemo;
	sliderMassRationActivationThreshold.m_minVal = 0;
	sliderMassRationActivationThreshold.m_maxVal = 1.0 - stepMassRationActivationThreshold;
	sliderMassRationActivationThreshold.m_clampToIntegers = true;
	sliderMassRationActivationThreshold.m_clampToNotches = true;
	sliderMassRationActivationThreshold.m_callback = [](float value, void* userPtr)
	{
		CableDemo* pdemo = (CableDemo*)userPtr;
		pdemo->m_cable->setMassRatioActivationThreshold(value);
	};
	pdemo->getGUIHelper()->getParameterInterface()->registerSliderFloatParameter(sliderMassRationActivationThreshold, stepMassRationActivationThreshold);

	SliderParams sliderCableLinearMass("Linear Mass (Cable)", &data.Cable_linearMass);
	btScalar stepCableLinearMass = 1.0;
	sliderCableLinearMass.m_userPointer = pdemo;
	sliderCableLinearMass.m_minVal = 1;
	sliderCableLinearMass.m_maxVal = 100 - stepCableLinearMass;
	sliderCableLinearMass.m_clampToIntegers = true;
	sliderCableLinearMass.m_clampToNotches = true;
	sliderCableLinearMass.m_callback = [](float value, void* userPtr)
	{
		CableDemo* pdemo = (CableDemo*)userPtr;
		pdemo->m_cable->setLinearMass(value);
		pdemo->m_cable->updateNodesMass();
	};
	pdemo->getGUIHelper()->getParameterInterface()->registerSliderFloatParameter(sliderCableLinearMass, stepCableLinearMass);

	SliderParams sliderCableTargetLength("Target Length (Cable)", &data.Cable_targetLength);
	btScalar stepCableTargetLength = 0.25;
	sliderCableTargetLength.m_userPointer = pdemo;
	sliderCableTargetLength.m_minVal = 0.25;
	sliderCableTargetLength.m_maxVal = 20 - stepCableTargetLength;
	sliderCableTargetLength.m_clampToIntegers = false;
	sliderCableTargetLength.m_clampToNotches = true;
	sliderCableTargetLength.m_callback = [](float value, void* userPtr)
	{
		CableDemo* demo = (CableDemo*)userPtr;
		demo->SetCableTargetLength(value);
	};
	pdemo->getGUIHelper()->getParameterInterface()->registerSliderFloatParameter(sliderCableTargetLength, stepCableTargetLength);

	SliderParams sliderCableGrowSpeed("Change Speed (Cable)", &data.Cable_growSpeed);
	btScalar stepCableGrowSpeed = 0.25;
	sliderCableGrowSpeed.m_userPointer = pdemo;
	sliderCableGrowSpeed.m_minVal = -5.0;
	sliderCableGrowSpeed.m_maxVal = 5.0 - stepCableGrowSpeed;
	sliderCableGrowSpeed.m_clampToIntegers = false;
	sliderCableGrowSpeed.m_clampToNotches = true;
	sliderCableGrowSpeed.m_callback = [](float value, void* userPtr)
	{
		CableDemo* demo = (CableDemo*)userPtr;
		demo->SetCableGrowSpeed(value);
	};
	pdemo->getGUIHelper()->getParameterInterface()->registerSliderFloatParameter(sliderCableGrowSpeed, stepCableGrowSpeed);

	SliderParams sliderCableSolverIteration("Solver iteration (Cable)", &data.Cable_iteration);
	btScalar stepCableSolverIteration = 1.0;
	sliderCableSolverIteration.m_userPointer = pdemo;
	sliderCableSolverIteration.m_minVal = 5;
	sliderCableSolverIteration.m_maxVal = 200 - stepCableSolverIteration;
	sliderCableSolverIteration.m_clampToIntegers = true;
	sliderCableSolverIteration.m_clampToNotches = true;
	sliderCableSolverIteration.m_callback = [](float value, void* userPtr)
	{
		CableDemo* pdemo = (CableDemo*)userPtr;
		pdemo->m_cable->m_cfg.piterations = value;
	};
	pdemo->getGUIHelper()->getParameterInterface()->registerSliderFloatParameter(sliderCableSolverIteration, stepCableSolverIteration);
}

static void Init_StabilityA18(CableDemo* pdemo)
{
	// Update shared data
	auto& data = StabilityA18Data;
	pdemo->m_stabilityData = &data;

	// Test defaults
	if (data.AutoResetTensionTest)
	{
		// Rigidbodies
		data.A_mass = 100;
		data.C_mass = 100;
		data.Ground_offset = 10.0;
		data.A_MassImpact = 0.0;

		// Anchors
		data.A_massRatio = 0.0;
		data.D_massRatio = 0.0;

		// Distance
		data.Cable_DistanceMode = 0;

		// Cable
		data.Cable_resolution = 20;
		data.Cable_iteration = 100;
		data.Cable_linearMass = 10;
		data.Cable_length = 0.4;
		data.Cable_targetLength = 1.0;
		data.Cable_growSpeed = 1.0;
		data.Cable_LRA = false;
		data.Cable_AnchorPlacement = false;
		data.Cable_massRatioActivationThreshold = 0.0;
	}

	// Shapes
	btCollisionShape* updownBoxShape = new btBoxShape(btVector3(0.4, 0.1, 0.1));
	btCollisionShape* lefrightBoxShape = new btBoxShape(btVector3(0.1, 0.4, 0.1));
	btTransform upBoxTransform(btMatrix3x3::getIdentity(), btVector3(0, 0.3, 0));
	btTransform downBoxTransform(btMatrix3x3::getIdentity(), btVector3(0, -0.3, 0));
	btTransform leftBoxTransform(btMatrix3x3::getIdentity(), btVector3(-0.3, 0, 0));
	btTransform rightBoxTransform(btMatrix3x3::getIdentity(), btVector3(0.3, 0, 0));

	btCompoundShape* ringShape = new btCompoundShape(false);
	ringShape->addChildShape(upBoxTransform, updownBoxShape);
	ringShape->addChildShape(downBoxTransform, updownBoxShape);
	ringShape->addChildShape(leftBoxTransform, lefrightBoxShape);
	ringShape->addChildShape(rightBoxTransform, lefrightBoxShape);

	btBoxShape* a18Shape = new btBoxShape(btVector3(0.6, 2, 0.6));

	btScalar waypointHeightShift = 0.3;
	btScalar attachPointHeight = 5;
	btScalar LestHeight = attachPointHeight - waypointHeightShift - data.Cable_length;

	btTransform transformAttachPoint(btMatrix3x3::getIdentity(), btVector3(0, attachPointHeight, 0));
	btTransform transformRingLest(btMatrix3x3::getIdentity(), btVector3(0, LestHeight, 0));
	btTransform transformRingA18(btQuaternion(btVector3(0, 1, 0), SIMD_HALF_PI), btVector3(0, LestHeight - 0.3, 0));
	btTransform transformA18(btMatrix3x3::getIdentity(), btVector3(0, transformRingA18.getOrigin().getY() - 2.0 - 0.3, 0));

	// Objet A (Lest)
	btRigidBody* ringLest = pdemo->createRigidBody(data.A_mass, transformRingLest, ringShape);
	ringLest->setSleepingThresholds(0, 0);
	ringLest->setupMassAtImpact(data.A_mass, 200, 0.0, 0.2);
	ringLest->activeMassAtImpact(data.A_MassImpact > 0.0 ? true : false);

	// Objet C (A18)
	btRigidBody* a18 = pdemo->createRigidBody(data.C_mass, transformA18, a18Shape);
	a18->setSleepingThresholds(0, 0);

	// Object B (Claw)
	btRigidBody* ringA18 = pdemo->createRigidBody(data.B_mass, transformRingA18, ringShape);
	ringA18->m_localTransform = btTransform(btQuaternion(btVector3(0, 1, 0), 3.14 / 2.0), btVector3(0, 2.3, 0));
	ringA18->m_redirectionTarget = a18;
	a18->m_Children.push_back(ringA18);

	// Objet D (Cable attach point)
	btRigidBody* attachPoint = pdemo->createRigidBody(btScalar(1), transformAttachPoint, new btBoxShape(btVector3(0.1, 0.1, 0.1)));
	attachPoint->setCollisionFlags(2);  
	attachPoint->setSleepingThresholds(0, 0);
	attachPoint->setMassProps(0, btVector3(0, 0, 0));

	// Ground
	btRigidBody* ground = pdemo->createRigidBody(0, btTransform(btQuaternion::getIdentity(), 
		                                         btVector3(0, LestHeight - data.Ground_offset - 4 - 0.6 - 0.2, 0)),
												 new btBoxShape(btVector3(10, 0.2, 10)));

	// Cable
	// Waypoints
	btAlignedObjectArray<btVector3> waypointPos = btAlignedObjectArray<btVector3>();
	waypointPos.push_back(transformRingLest.getOrigin() + btVector3(0, waypointHeightShift, 0));
	waypointPos.push_back(transformAttachPoint.getOrigin());

	// Parameters
	btCable* cable = pdemo->createCableWaypoint(data.Cable_resolution, data.Cable_iteration, data.Cable_linearMass, waypointPos, ringLest, attachPoint, true, true);
	cable->getCollisionShape()->setMargin(0.025);
	cable->setCableRadius(0.05);
	cable->setUseBending(false);
	cable->setUseLRA(data.Cable_LRA);
	cable->m_anchors[0].m_bodyMassRatio = data.A_massRatio;
	cable->m_anchors[1].m_bodyMassRatio = data.D_massRatio;
	cable->m_materials[0]->m_kLST = 1.0;
	cable->setDistanceMode(data.Cable_DistanceMode);
	cable->setUseAnchorConstraintPlacement(data.Cable_AnchorPlacement);
	cable->setMassRatioActivationThreshold(data.Cable_massRatioActivationThreshold);
	pdemo->m_cable = cable;

	// User controls
	SliderParams sliderSubsteps("Substeps count (Global)", &globals.substepsCount);
	sliderSubsteps.m_userPointer = pdemo;
	sliderSubsteps.m_minVal = 1;
	sliderSubsteps.m_maxVal = 8;
	sliderSubsteps.m_clampToIntegers = true;
	sliderSubsteps.m_clampToNotches = true;
	pdemo->getGUIHelper()->getParameterInterface()->registerSliderFloatParameter(sliderSubsteps, 1);

	ButtonParams dataSelector("Lock", 0, true);
	dataSelector.m_userPointer = pdemo;
	dataSelector.m_initialState = data.AutoResetTensionTest < 0.5;
	dataSelector.m_callback = [](int buttonId, bool buttonState, void* userPtr)
	{
		if (buttonId != 0) { return; }

		CableDemo* pdemo = (CableDemo*)userPtr;
		pdemo->m_stabilityData->AutoResetTensionTest = buttonState ? 0 : 1;
	};
	pdemo->getGUIHelper()->getParameterInterface()->registerButtonParameter(dataSelector);

	SliderParams sliderCableResolution("Resolution (Cable)", &data.Cable_resolution);
	sliderCableResolution.m_userPointer = pdemo;
	sliderCableResolution.m_minVal = 5;
	sliderCableResolution.m_maxVal = 200;
	sliderCableResolution.m_clampToIntegers = true;
	sliderCableResolution.m_clampToNotches = true;
	pdemo->getGUIHelper()->getParameterInterface()->registerSliderFloatParameter(sliderCableResolution, 5);

	SliderParams sliderCableLength("Initial Length (Cable)", &data.Cable_length);
	sliderCableLength.m_userPointer = pdemo;
	sliderCableLength.m_minVal = 0.2;
	sliderCableLength.m_maxVal = 8;
	sliderCableLength.m_clampToIntegers = false;
	sliderCableLength.m_clampToNotches = true;
	pdemo->getGUIHelper()->getParameterInterface()->registerSliderFloatParameter(sliderCableLength, 0.2);

	SliderParams sliderGroundOffset("Offset (Ground)", &data.Ground_offset);
	sliderGroundOffset.m_userPointer = ground;
	sliderGroundOffset.m_minVal = 0;
	sliderGroundOffset.m_maxVal = 200;
	sliderGroundOffset.m_clampToIntegers = true;
	sliderGroundOffset.m_clampToNotches = true;
	pdemo->getGUIHelper()->getParameterInterface()->registerSliderFloatParameter(sliderGroundOffset, 2);

	ButtonParams lraSelector("LRA", 1, true);
	lraSelector.m_userPointer = pdemo;
	lraSelector.m_initialState = data.Cable_LRA;
	lraSelector.m_callback = [](int buttonId, bool buttonState, void* userPtr)
	{
		if (buttonId != 1)
		{
			return;
		}

		CableDemo* pdemo = (CableDemo*)userPtr;
		pdemo->m_stabilityData->Cable_LRA = buttonState;
		pdemo->m_cable->setUseLRA(pdemo->m_stabilityData->Cable_LRA);
	};
	pdemo->getGUIHelper()->getParameterInterface()->registerButtonParameter(lraSelector);

	ButtonParams anchorPlacementSelector("AnchorPlacement", 2, true);
	anchorPlacementSelector.m_userPointer = pdemo;
	anchorPlacementSelector.m_initialState = data.Cable_AnchorPlacement;
	anchorPlacementSelector.m_callback = [](int buttonId, bool buttonState, void* userPtr)
	{
		if (buttonId != 2)
		{
			return;
		}

		CableDemo* pdemo = (CableDemo*)userPtr;
		pdemo->m_stabilityData->Cable_AnchorPlacement = buttonState;
		pdemo->m_cable->setUseAnchorConstraintPlacement(pdemo->m_stabilityData->Cable_AnchorPlacement);
	};
	pdemo->getGUIHelper()->getParameterInterface()->registerButtonParameter(anchorPlacementSelector);

	SliderParams sliderMassA("Mass Lest (A)", &data.A_mass);
	sliderMassA.m_userPointer = ringLest;
	sliderMassA.m_minVal = 10;
	sliderMassA.m_maxVal = 20000;
	sliderMassA.m_clampToIntegers = true;
	sliderMassA.m_clampToNotches = true;
	sliderMassA.m_callback = [](float value, void* userPtr)
	{
		btRigidBody* lest = (btRigidBody*)userPtr;
		lest->setMassProps(value, lest->getLocalInertia() * value * lest->getInvMass());
		lest->updateInertiaTensor();
	};
	pdemo->getGUIHelper()->getParameterInterface()->registerSliderFloatParameter(sliderMassA, 100);

	SliderParams sliderMassC("Mass A18 (C)", &data.C_mass);
	sliderMassC.m_userPointer = a18;
	sliderMassC.m_minVal = 10;
	sliderMassC.m_maxVal = 20000;
	sliderMassC.m_clampToIntegers = true;
	sliderMassC.m_clampToNotches = true;
	sliderMassC.m_callback = [](float value, void* userPtr)
	{
		btRigidBody* a18 = (btRigidBody*)userPtr;
		a18->setMassProps(value, a18->getLocalInertia() * value * a18->getInvMass());
		a18->updateInertiaTensor();
	};
	pdemo->getGUIHelper()->getParameterInterface()->registerSliderFloatParameter(sliderMassC, 100);

	SliderParams sliderMassImpactA("MassAtImpact (A)", &data.A_MassImpact);
	sliderMassImpactA.m_userPointer = ringLest;
	sliderMassImpactA.m_minVal = 0;
	sliderMassImpactA.m_maxVal = 1;
	sliderMassImpactA.m_clampToIntegers = true;
	sliderMassImpactA.m_clampToNotches = false;
	sliderMassImpactA.m_callback = [](float value, void* userPtr)
	{
		btRigidBody* lest = (btRigidBody*)userPtr;
		lest->activeMassAtImpact(value > 0.0 ? true : false);
	};
	pdemo->getGUIHelper()->getParameterInterface()->registerSliderFloatParameter(sliderMassImpactA, 1);

	SliderParams sliderAMassRatio("MassRatio (A)", &data.A_massRatio);
	sliderAMassRatio.m_userPointer = pdemo;
	sliderAMassRatio.m_minVal = 0;
	sliderAMassRatio.m_maxVal = 1;
	sliderAMassRatio.m_clampToIntegers = false;
	sliderAMassRatio.m_clampToNotches = true;
	sliderAMassRatio.m_callback = [](float value, void* userPtr)
	{
		CableDemo* demo = (CableDemo*)userPtr;
		demo->m_cable->m_anchors[0].m_bodyMassRatio = value;
	};
	pdemo->getGUIHelper()->getParameterInterface()->registerSliderFloatParameter(sliderAMassRatio, 0.05);

	SliderParams sliderMassRationActivationThreshold("Mass ratio min threshold (Cable)", &data.Cable_massRatioActivationThreshold);
	sliderMassRationActivationThreshold.m_userPointer = pdemo;
	sliderMassRationActivationThreshold.m_minVal = 0;
	sliderMassRationActivationThreshold.m_maxVal = 1.0;
	sliderMassRationActivationThreshold.m_clampToIntegers = true;
	sliderMassRationActivationThreshold.m_clampToNotches = true;
	sliderMassRationActivationThreshold.m_callback = [](float value, void* userPtr)
	{
		CableDemo* pdemo = (CableDemo*)userPtr;
		pdemo->m_cable->setMassRatioActivationThreshold(value);
	};
	pdemo->getGUIHelper()->getParameterInterface()->registerSliderFloatParameter(sliderMassRationActivationThreshold, 0.05);

	SliderParams sliderCableLinearMass("Linear Mass (Cable)", &data.Cable_linearMass);
	sliderCableLinearMass.m_userPointer = pdemo;
	sliderCableLinearMass.m_minVal = 1;
	sliderCableLinearMass.m_maxVal = 50;
	sliderCableLinearMass.m_clampToIntegers = true;
	sliderCableLinearMass.m_clampToNotches = true;
	sliderCableLinearMass.m_callback = [](float value, void* userPtr)
	{
		CableDemo* pdemo = (CableDemo*)userPtr;
		pdemo->m_cable->setLinearMass(value);
		pdemo->m_cable->updateNodesMass();
	};
	pdemo->getGUIHelper()->getParameterInterface()->registerSliderFloatParameter(sliderCableLinearMass, 5);

	SliderParams sliderCableTargetLength("Target Length (Cable)", &data.Cable_targetLength);
	sliderCableTargetLength.m_userPointer = pdemo;
	sliderCableTargetLength.m_minVal = 0.2;
	sliderCableTargetLength.m_maxVal = 8;
	sliderCableTargetLength.m_clampToIntegers = false;
	sliderCableTargetLength.m_clampToNotches = true;
	sliderCableTargetLength.m_callback = [](float value, void* userPtr)
	{
		CableDemo* demo = (CableDemo*)userPtr;
		demo->SetCableTargetLength(value);
	};
	pdemo->getGUIHelper()->getParameterInterface()->registerSliderFloatParameter(sliderCableTargetLength, 0.2);

	SliderParams sliderCableGrowSpeed("Change Speed (Cable)", &data.Cable_growSpeed);
	sliderCableGrowSpeed.m_userPointer = pdemo;
	sliderCableGrowSpeed.m_minVal = -4;
	sliderCableGrowSpeed.m_maxVal = 4;
	sliderCableGrowSpeed.m_clampToIntegers = false;
	sliderCableGrowSpeed.m_clampToNotches = true;
	sliderCableGrowSpeed.m_callback = [](float value, void* userPtr)
	{
		CableDemo* demo = (CableDemo*)userPtr;
		demo->SetCableGrowSpeed(value);
	};
	pdemo->getGUIHelper()->getParameterInterface()->registerSliderFloatParameter(sliderCableGrowSpeed, 0.2);

	SliderParams sliderCableSolverIteration("Solver iteration (Cable)", &data.Cable_iteration);
	sliderCableSolverIteration.m_userPointer = pdemo;
	sliderCableSolverIteration.m_minVal = 10;
	sliderCableSolverIteration.m_maxVal = 200;
	sliderCableSolverIteration.m_clampToIntegers = true;
	sliderCableSolverIteration.m_clampToNotches = true;
	sliderCableSolverIteration.m_callback = [](float value, void* userPtr)
	{
		CableDemo* pdemo = (CableDemo*)userPtr;
		pdemo->m_cable->m_cfg.piterations = value;
	};
	pdemo->getGUIHelper()->getParameterInterface()->registerSliderFloatParameter(sliderCableSolverIteration, 10);
}


void (*demofncs[])(CableDemo*) =
{
	Init_CableForceDown,
	Init_CableForceUp,
	Init_Nodes,
	Init_Weigths,
	Init_Iterations,
	Init_Lengths,
	Init_TestArse,
	Init_TestCollisionFreeCableWithStaticCube,
	Init_TestSupportA18,
	Init_Test1000Nodes,
	Init_TestCollisionCableSphere,
	Init_TestCollisionFallingA18Constraint,
	Init_TestCollisionCableConvexHullOnMeshSphere,
	Init_TestCollisionRingBox,
	Init_TestCollisionRingSphere,
	Init_TestCollisionOn1Node,
	Init_TestClaw,
	Init_Growth,
	Init_TestCableCollisionMt,
	Init_CableHydro,
	Init_CableBending,
	Init_TwoCablesOneCube,
	Init_DetachA18,
	Init_MCMVCable,
	Init_BallJoint,
	Init_FixedJoint,
	Init_RayCast,
	Init_Collision,
	Init_StabilityTension,
	Init_StabilityA18,
	Init_Stability,
	Init_TestBenchmarkSubsteps,
};

////////////////////////////////////
///for mouse picking
void pickingPreTickCallbackCable(btDynamicsWorld* world, btScalar timeStep)
{
	CableDemo* cableDemo = (CableDemo*)world->getWorldUserInfo();

	if (cableDemo->m_drag)
	{
		const int x = cableDemo->m_lastmousepos[0];
		const int y = cableDemo->m_lastmousepos[1];
		float rf[3];
		cableDemo->getGUIHelper()->getRenderInterface()->getActiveCamera()->getCameraPosition(rf);
		float target[3];
		cableDemo->getGUIHelper()->getRenderInterface()->getActiveCamera()->getCameraTargetPosition(target);
		btVector3 cameraTargetPosition(target[0], target[1], target[2]);

		const btVector3 cameraPosition(rf[0], rf[1], rf[2]);
		const btVector3 rayFrom = cameraPosition;

		const btVector3 rayTo = cableDemo->getRayTo(x, y);
		const btVector3 rayDir = (rayTo - rayFrom).normalized();
		const btVector3 N = (cameraTargetPosition - cameraPosition).normalized();
		const btScalar O = btDot(cableDemo->m_impact, N);
		const btScalar den = btDot(N, rayDir);
		if ((den * den) > 0)
		{
			const btScalar num = O - btDot(N, rayFrom);
			const btScalar hit = num / den;
			if ((hit > 0) && (hit < 1500))
			{
				cableDemo->m_goal = rayFrom + rayDir * hit;
			}
		}
		btVector3 delta = cableDemo->m_goal - cableDemo->m_node->m_x;
		static const btScalar maxdrag = 10;
		if (delta.length2() > (maxdrag * maxdrag))
		{
			delta = delta.normalized() * maxdrag;
		}
		cableDemo->m_node->m_v += delta / timeStep;
	}
}

int currentCableDemo = 0;

///
/// btTaskSchedulerManager -- manage a number of task schedulers so we can switch between them
///
class btTaskSchedulerManager
{
	btAlignedObjectArray<btITaskScheduler*> m_taskSchedulers;
	btAlignedObjectArray<btITaskScheduler*> m_allocatedTaskSchedulers;

public:
	btTaskSchedulerManager() {}
	void init()
	{
		addTaskScheduler(btGetSequentialTaskScheduler());
#if BT_THREADSAFE
		if (btITaskScheduler* ts = btCreateDefaultTaskScheduler())
		{
			m_allocatedTaskSchedulers.push_back(ts);
			addTaskScheduler(ts);
		}
		addTaskScheduler(btGetOpenMPTaskScheduler());
		addTaskScheduler(btGetTBBTaskScheduler());
		addTaskScheduler(btGetPPLTaskScheduler());
		if (getNumTaskSchedulers() > 1)
		{
			// prefer a non-sequential scheduler if available
			btSetTaskScheduler(m_taskSchedulers[1]);
		}
		else
		{
			btSetTaskScheduler(m_taskSchedulers[0]);
		}
#endif  // #if BT_THREADSAFE
	}
	void shutdown()
	{
		for (int i = 0; i < m_allocatedTaskSchedulers.size(); ++i)
		{
			delete m_allocatedTaskSchedulers[i];
		}
		m_allocatedTaskSchedulers.clear();
	}

	void addTaskScheduler(btITaskScheduler* ts)
	{
		if (ts)
		{
#if BT_THREADSAFE
			// if initial number of threads is 0 or 1,
			if (ts->getNumThreads() <= 1)
			{
				// for OpenMP, TBB, PPL set num threads to number of logical cores
				ts->setNumThreads(ts->getMaxNumThreads());
			}
#endif  // #if BT_THREADSAFE
			m_taskSchedulers.push_back(ts);
		}
	}
	int getNumTaskSchedulers() const { return m_taskSchedulers.size(); }
	btITaskScheduler* getTaskScheduler(int i) { return m_taskSchedulers[i]; }
};

static btTaskSchedulerManager gTaskSchedulerMgr;

void CableDemo::setDrawClusters(bool drawClusters)
{
	if (drawClusters)
	{
		getSoftDynamicsWorld()->setDrawFlags(getSoftDynamicsWorld()->getDrawFlags() | fDrawFlags::Clusters);
	}
	else
	{
		getSoftDynamicsWorld()->setDrawFlags(getSoftDynamicsWorld()->getDrawFlags() & (~fDrawFlags::Clusters));
	}
}

void CableDemo::initPhysics()
{
	m_currentDemoIndex = currentCableDemo;

	///create concave ground mesh
	m_guiHelper->setUpAxis(1);

	m_dispatcher = 0;

	if (gTaskSchedulerMgr.getNumTaskSchedulers() == 0)
	{
		gTaskSchedulerMgr.init();
	}

	///register some softbody collision algorithms on top of the default btDefaultCollisionConfiguration
	m_collisionConfiguration = new btSoftBodyRigidBodyCollisionConfiguration();

	//m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
	m_dispatcher = new btCollisionDispatcherMt(m_collisionConfiguration);
	m_softBodyWorldInfo.m_dispatcher = m_dispatcher;

	////////////////////////////
	///Register softbody versus softbody collision algorithm

	///Register softbody versus rigidbody collision algorithm
	////////////////////////////

	btVector3 worldAabbMin(-1000, -1000, -1000);
	btVector3 worldAabbMax(1000, 1000, 1000);

	//m_broadphase = new btAxisSweep3(worldAabbMin, worldAabbMax, maxProxies);

	m_broadphase = new btDbvtBroadphase();

	m_softBodyWorldInfo.m_broadphase = m_broadphase;

	m_softBodyWorldInfo.numThread = 4;

	//btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver();
	btSequentialImpulseConstraintSolverMt* solver = new btSequentialImpulseConstraintSolverMt();

	//btDantzigSolver* mlcp = new btDantzigSolver();
	//btSolveProjectedGaussSeidel* mlcp = new btSolveProjectedGaussSeidel;
	//btMLCPSolver* solver = new btMLCPSolver(mlcp);

	m_solver = solver;

	btSoftBodySolver* softBodySolver = 0;
#ifdef USE_AMD_OPENCL

	static bool once = true;
	if (once)
	{
		once = false;
		initCL(0, 0);
	}

	if (g_openCLSIMDSolver)
		delete g_openCLSIMDSolver;
	if (g_softBodyOutput)
		delete g_softBodyOutput;

	if (1)
	{
		g_openCLSIMDSolver = new btOpenCLSoftBodySolverSIMDAware(g_cqCommandQue, g_cxMainContext);
		//	g_openCLSIMDSolver = new btOpenCLSoftBodySolver( g_cqCommandQue, g_cxMainContext);
		g_openCLSIMDSolver->setCLFunctions(new CachingCLFunctions(g_cqCommandQue, g_cxMainContext));
	}

	softBodySolver = g_openCLSIMDSolver;
	g_softBodyOutput = new btSoftBodySolverOutputCLtoCPU;
#endif  //USE_AMD_OPENCL

	//btDiscreteDynamicsWorld* world = new btSoftRigidDynamicsWorld(m_dispatcher, m_broadphase, m_solver, m_collisionConfiguration);

	btSoftRigidDynamicsWorld* world = new btSoftRigidDynamicsWorld(&m_softBodyWorldInfo, m_solver, m_collisionConfiguration, softBodySolver);
	m_dynamicsWorld = world;
	m_dynamicsWorld->setInternalTickCallback(pickingPreTickCallbackCable, this, true);

	m_dynamicsWorld->getSolverInfo().m_numIterations = 256;

	m_dynamicsWorld->getDispatchInfo().m_enableSPU = true;
	m_dynamicsWorld->setGravity(btVector3(0, -9.81, 0));
	m_softBodyWorldInfo.m_gravity.setValue(0, -9.81, 0);
	m_guiHelper->createPhysicsDebugDrawer(world);
	//	clientResetScene();

	m_softBodyWorldInfo.m_sparsesdf.Initialize();
	//	clientResetScene();

	//create ground object

	int lastDemo = (sizeof(demofncs) / sizeof(demofncs[0])) - 1;

	if (currentCableDemo < 0)
		currentCableDemo = lastDemo;
	if (currentCableDemo > lastDemo)
		currentCableDemo = 0;

	m_softBodyWorldInfo.m_sparsesdf.Reset();

	m_softBodyWorldInfo.air_density = (btScalar)0;
	m_softBodyWorldInfo.water_density = 0;
	m_softBodyWorldInfo.water_offset = 0;
	m_softBodyWorldInfo.water_normal = btVector3(0, 0, 0);
	m_softBodyWorldInfo.m_gravity.setValue(0, -9.81, 0);
	// Sync both asked value and current used value
	substepSolver = globals.substepsCount;
	m_softBodyWorldInfo.numIteration = substepSolver;

	m_autocam = false;
	m_raycast = false;
	m_cutting = false;
	m_results.fraction = 1.f;

	demofncs[currentCableDemo](this);

	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);

	// register GIMPACT algorithm
	btGImpactCollisionAlgorithm::registerAlgorithm((btCollisionDispatcher*)m_dynamicsWorld->getDispatcher());
}

void CableDemo::exitPhysics()
{
	//cleanup in the reverse order of creation/initialization

	//remove the rigidbodies from the dynamics world and delete them
	int i;
	for (i = m_dynamicsWorld->getNumConstraints() - 1; i >= 0; i--)
	{
		btTypedConstraint* constraint = m_dynamicsWorld->getConstraint(i);
		m_dynamicsWorld->removeConstraint(constraint);
		delete constraint;
	}
	for (i = m_dynamicsWorld->getNumCollisionObjects() - 1; i >= 0; i--)
	{
		btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		if (body && body->getMotionState())
		{
			delete body->getMotionState();
		}
		m_dynamicsWorld->removeCollisionObject(obj);
		delete obj;
	}

	//delete collision shapes
	for (int j = 0; j < m_collisionShapes.size(); j++)
	{
		btCollisionShape* shape = m_collisionShapes[j];
		m_collisionShapes[j] = 0;
		delete shape;
	}

	//delete dynamics world
	delete m_dynamicsWorld;
	m_dynamicsWorld = 0;

	//delete solver
	delete m_solver;

	//delete broadphase
	delete m_broadphase;

	//delete dispatcher
	delete m_dispatcher;

	delete m_collisionConfiguration;
}

class CommonExampleInterface* CableDemoCreateFunc(struct CommonExampleOptions& options)
{
	currentCableDemo = options.m_option;
	return new CableDemo(options.m_guiHelper);
}
