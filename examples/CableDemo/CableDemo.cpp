#include "../Utils/b3ResourcePath.h"
#include "Bullet3Common/b3FileUtils.h"
#include "../Importers/ImportObjDemo/LoadMeshFromObj.h"
#include "../OpenGLWindow/GLInstanceGraphicsShape.h"
#include "../Utils/b3BulletDefaultFileIO.h"

#include "../CommonInterfaces/CommonRigidBodyBase.h"

#include "btBulletDynamicsCommon.h"
#include "BulletSoftBody/btSoftRigidDynamicsWorld.h"

#include "BulletCollision/CollisionDispatch/btSphereSphereCollisionAlgorithm.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkEpa2.h"
#include "LinearMath/btQuickprof.h"
#include "LinearMath/btIDebugDraw.h"

#include <stdio.h>  //printf debugging
#include "LinearMath/btConvexHull.h"
#include "BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h"
#include "BulletSoftBody/btSoftBodyHelpers.h"

#include "CableDemo.h"
#include "GL_ShapeDrawer.h"

#include "LinearMath/btAlignedObjectArray.h"
#include "BulletSoftBody/btSoftBody.h"
#include "BulletCable/btCable.h"

#include "BulletDynamics/MLCPSolvers/btDantzigSolver.h"
#include "BulletDynamics/MLCPSolvers/btSolveProjectedGaussSeidel.h"
#include "BulletDynamics/MLCPSolvers/btMLCPSolver.h"
#include <BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolverMt.h>
#include "BulletCollision/CollisionDispatch/btCollisionDispatcherMt.h"
#include <iostream>
#include <chrono>
#include <BulletCollision/CollisionDispatch/btInternalEdgeUtility.h>

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


class CableDemo : public CommonRigidBodyBase
{
public:
	int substepSolver = 4;
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
	bool m_printTens;
	bool m_close = false;
	bool m_grow = false;
	bool m_growWithSpeedAndDistance = false;
	bool m_shrink = false;
	btScalar speed = 0;

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

	CableDemo(struct GUIHelperInterface* helper): CommonRigidBodyBase(helper),
		  m_drag(false)

	{
		m_applyForceOnRigidbody = false;
		m_attachLock = false;
		m_moveBody = false;
		m_printFPS = false;
		m_printTens = false;
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
			if (speed>=0)
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
			m_printFPS = ! m_printFPS;
		}
		if (key == 'n' && state)
		{
			m_attachLock = true;
		}

		if (key == 't' && state)
		{
			m_printTens = !m_printTens;
		}

		return false;
	}

	void mouseFunc(int button, int state, int x, int y);
	void mouseMotionFunc(int x, int y);

	void Grows(float dt,bool test = true)
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
		btScalar radius = cable->getCableData().radius;
		btScalar coefficient = cable->getCableData().normalDragCoefficient;

		for (int i = 1; i < cable->m_nodes.size()-1; ++i)
		{
			btVector3 nodePosition = cable->m_nodes[i].m_x;
			btVector3 nodeVelocity = btVector3(cable->getNodeData()[i].velocity_x, cable->getNodeData()[i].velocity_y, cable->getNodeData()[i].velocity_z);

			const float waterFactor = 1.0f;

			// Initialize the forces/velocities
			btVector3 waterNormalForce = btVector3(0,0,0);

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

			waterNormalForce += computeNormalForce(i,relativeVelWater, linkBefore, linkDirBefore,radius, density, 1.2)* linkBeforeWaterFactor / 2.0f;
			waterNormalForce += computeNormalForce(i,relativeVelWater, linkAfter, linkDirAfter, radius, density, 1.2) * linkAfterWaterFactor / 2.0f;

			float addedMass = density * 3.1415926535f * (radius * radius) * linkBefore.length() * 0.5f;
			addedMass += density * 3.1415926535f * (radius * radius) * linkAfter.length() * 0.5f;
		
		//	cable->m_nodes[i].m_f += waterNormalForce;

			getSoftDynamicsWorld()->m_nodeForces[i].x = waterNormalForce.getX();
			getSoftDynamicsWorld()->m_nodeForces[i].y = waterNormalForce.getY();
			getSoftDynamicsWorld()->m_nodeForces[i].z = waterNormalForce.getZ();
			getSoftDynamicsWorld()->m_nodeForces[i].ma = addedMass;
			
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
			obj->applyCentralForce(btVector3(speed*500, 0, 0));
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
		auto softWorld = getSoftDynamicsWorld()->getCollisionObjectArray();
		btCable* cable = (btCable*)getSoftDynamicsWorld()->getSoftBodyArray().at(0);
		btRigidBody* a18 = (btRigidBody*)softWorld.at(6);

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
		cable->appendAnchor(1,a18);
		cable->appendAnchor(5,a18);
		cable->setUseCollision(false);
		cable->setUseLRA(false);
		m_attachLock = false;
	}

	void printTension()
	{
		auto softWorld = getSoftDynamicsWorld()->getCollisionObjectArray();

		for (int i = 0; i < getSoftDynamicsWorld()->getSoftBodyArray().size(); ++i)
		{
			btCable* cable = (btCable*)getSoftDynamicsWorld()->getSoftBodyArray().at(i);
			for (int i = 0; i < cable->m_anchors.size(); i++)
			{
				if (cable->m_anchors.at(i).m_body->getInvMass() == 0) continue;
				cout << " Tension on Anchor " << i << " - Node " << cable->m_anchors.at(i).m_node->index
					 << " = " << cable->getTensionAt(i).length() << " - Mass " << 1.0 / cable->m_anchors.at(i).m_body->getInvMass() << endl;
			}

		}
	}
	void stepSimulation(float deltaTime) override
	{
		if (m_dynamicsWorld)
		{
			const auto currentTime{Clock::now()};
			if ( m_applyForceOnRigidbody)
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
			if (m_printTens)
				printTension();

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

			current_ticks = clock();

			if (m_currentDemoIndex == 21) // Cable hydro force
			{
				ApplyForcesNodes(m_waterCurrent);
			}

			int subStep = substepSolver;
			m_dynamicsWorld->stepSimulation(deltaTime, subStep, deltaTime / subStep);
			//m_dynamicsWorld->stepSimulation(deltaTime);

			delta_ticks = clock() - current_ticks;
			
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
		for (int i = 0; i < 5 ; i++)
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

	void PrintDistance(int indexCable, btCable * cable)
	{
		btSoftBody::Anchor a0 = cable->m_anchors[0];
		btVector3 worldPositionAnchor0 = a0.m_body->getCenterOfMassPosition() + a0.m_c1;
		btVector3 worldPositionNode0 = a0.m_node->m_x;

		btSoftBody::Anchor a1 = cable->m_anchors[1];
		btVector3 worldPositionAnchor1 = a1.m_body->getCenterOfMassPosition() + a1.m_c1;
		btVector3 worldPositionNode1 = a1.m_node->m_x;

		float distance0 = worldPositionAnchor0.distance(worldPositionNode0) * 100; // Convert in cm
		float distance1 = worldPositionAnchor1.distance(worldPositionNode1) * 100;  // Convert in cm

		b3Printf("Cable : % i - Distance Anchor0-Node %f cm | Distance Anchor1-Node %f cm | - Cable length %f | - Impluse: %f N", indexCable, distance0, distance1, cable->getLength(), cable->getTensionAt(0).length());
		b3Printf("Distance: %f - Distance Reel: %f",  cable->getLength(), cable->getRestLength());
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
		btCable* cable = new btCable(&m_softBodyWorldInfo, getSoftDynamicsWorld(), resolution,0, positionNodes, massNodes);
		cable->setTotalMass(totalMass);
		
		cable->setUseCollision(false);
		if (bodyB != nullptr)
			cable->appendAnchor(0, bodyB, posWorldAnchorBodyB - bodyB->getWorldTransform().getOrigin(),DisableCollisionOnB);
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
		getSoftDynamicsWorld()->addSoftBody(cable);
		cable->setCollisionParameters(1, 1, 0);
		return cable;
	}

	btCable* createCableWaypoint(int resolution, int iteration, btScalar totalMass, btAlignedObjectArray<btVector3> anchorPos, btRigidBody* bodyA = nullptr, btRigidBody* bodyB = nullptr, bool DisableCollisionOnA = true, bool DisableCollisionOnB = true)
	{
		int s = anchorPos.size()-1;
		btScalar totalDist = 0;
		for (int i = 0; i < s; i++)
		{
			totalDist += (anchorPos.at(i) - anchorPos.at(i + 1)).length();
		}

		int numberOfNode = 0;
		for (int i = 0; i < s; i++)
		{
			btScalar dist = (anchorPos.at(i) - anchorPos.at(i + 1)).length();
			numberOfNode += dist/totalDist  *resolution ;
		}
		int resolutionReel = numberOfNode - (anchorPos.size()-2);

		// Nodes' positions
		//resolution = resolutionReel * s;
		btVector3* positionNodes = new btVector3[resolutionReel];
		btScalar* massNodes = new btScalar[resolutionReel];

		int position = 0;
		// j = number of links between anchors
		for (int j = 0; j < s ; j++)
		{
			btScalar dist = (anchorPos.at(j) - anchorPos.at(j + 1)).length();
			int limite = dist / totalDist * resolution;
			for (int i = 0; i < limite; ++i)
			{
				const btScalar t = i / (btScalar)(limite-1);
				btVector3 pos = lerp(anchorPos.at(j), anchorPos.at(j+1), t);
				positionNodes[position] = pos;
				massNodes[position] = 1;
				if (i !=limite-1)
					position++;
			}
		}
		// Cable's creation
		btCable* cable = new btCable(&m_softBodyWorldInfo, getSoftDynamicsWorld(), resolutionReel,0, positionNodes, massNodes);
		cable->setUseCollision(false);
		if (bodyA != nullptr)
			cable->appendAnchor(0, bodyA, anchorPos.at(0) - bodyA->getWorldTransform().getOrigin(), DisableCollisionOnA);
		if (bodyB != nullptr)
			cable->appendAnchor(cable->m_nodes.size() - 1, bodyB, anchorPos.at(s) - bodyB->getWorldTransform().getOrigin(), DisableCollisionOnB);
		// Cable's config
		cable->setTotalMass(totalMass);
		cable->m_cfg.piterations = iteration;
		cable->m_cfg.kAHR = 1;

		cable->setCollisionParameters(1, 1, 0);
		// Add cable to the world
		getSoftDynamicsWorld()->addSoftBody(cable);
		
		return cable;
	}
};

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
		cable->setCollisionParameters(1, 1, 0);
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

		btCable*  cable =pdemo->createCable(resolution, iteration,200, anchorPositionKinematic, anchorPositionPhysic, physic, kinematic);
		cable->setCollisionParameters(1, 1, 0);

		cable->m_anchors[0].BodyMassRatio = 1;
		cable->m_anchors[1].BodyMassRatio = 1;
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
		btVector3 positionKinematic(-5 +( i * 2.5f), 10, 0);
		btVector3 positionPhysic(-5 + (i * 2.5f), 4, 0);
		transformKinematic.setOrigin(positionKinematic);
		transformPhysic.setOrigin(positionPhysic);

		// Create the rigidbodys
		btRigidBody* kinematic = pdemo->createRigidBody(massKinematic, transformKinematic, boxShape);  // mASS kINECMATIC

		btRigidBody* physic = pdemo->createRigidBody(massPhysic, transformPhysic, boxShape);

		// Anchor's positions
		btVector3 anchorPositionKinematic = positionKinematic - btVector3(0, -0.5, 0);
		btVector3 anchorPositionPhysic = positionPhysic + btVector3(0, 0.5, 0);

		pdemo->createCable(resolution, iteration,1, anchorPositionKinematic, anchorPositionPhysic, physic, kinematic);
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

		pdemo->createCable(resolution, iteration, 1,  anchorPositionKinematic, anchorPositionPhysic, physic, kinematic);
	}
}

static void Init_CableBending(CableDemo* pdemo)
{
	// Shape
	btCollisionShape* cubeShape = new btBoxShape(btVector3(0.5, 0.5, 0.5));
	btCollisionShape* emptyShape = new btBoxShape(btVector3(0, 0, 0));

	// Parameters
	btScalar cubeMass(0);
	btVector3 cubeOrigin(0,0,0);
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
	cable->setCollisionParameters(3, 3, 0);
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
	btVector3 positionPhysic(0, 3-6.68, 0);
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
	btScalar linearMass = 0.003; // 3gr per meter
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

	b3Printf("Cable length :%f - Total cable mass: %f - DragCoefficient:%f - Radius:%f ", cableLength, cableTotalMass, cable->getCableData().normalDragCoefficient, cable->getCableData().radius);
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
	int resolution = 50;
	int iteration = 50;

	// Create the rigidbodys
	btRigidBody* kinematic = pdemo->createRigidBody(0, transformKinematic, shape);
	btRigidBody* physic = pdemo->createRigidBody(100, transformPhysic, shape);

	btRigidBody* wall = pdemo->createRigidBody(0, transformWall, new btBoxShape(btVector3(1,1,1)));
	
	wall->getCollisionShape()->setMargin(0);


	// Anchor's positions
	btVector3 anchorPositionKinematic = positionKinematic + btVector3(0, 0, 0);
	btVector3 anchorPositionPhysic = positionPhysic + btVector3(0, 0, 0);

	btCable* cable = pdemo->createCable(resolution, iteration, 1.61, anchorPositionKinematic, anchorPositionPhysic, physic, kinematic);
	cable->setUseCollision(true);
	cable->setUseLRA(false);
	cable->getCollisionShape()->setMargin(0.005);
	cable->setCollisionParameters(1, 1, 0);
	cable->setCollisionMargin(0.005);
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

	btRigidBody* wall = pdemo->createRigidBody(0, transformWall, new btBoxShape(btVector3(1,1,1)));
	
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
	cable->setCollisionParameters(1, 1, 0);
	cable->setCollisionMargin(0.005);
	pdemo->SetCameraPosition(btVector3(0, 10, 0));

	btCable* cable2 = pdemo->createCable(resolution, iteration, 1.61, anchorPositionKinematic2, anchorPositionPhysic2, physic, kinematic);

	cable2->setUseCollision(true);
	cable2->setUseLRA(false);
	cable2->getCollisionShape()->setMargin(0.005);
	cable2->setCollisionParameters(1, 1, 0);
	cable2->setCollisionMargin(0.005);

	btCable* cable3 = pdemo->createCable(resolution, iteration, 1.61, anchorPositionKinematic3, anchorPositionPhysic3, physic, kinematic);

	cable3->setUseCollision(true);
	cable3->setUseLRA(false);
	cable3->getCollisionShape()->setMargin(0.005);
	cable3->setCollisionParameters(1, 1, 0);
	cable3->setCollisionMargin(0.005);

	btCable* cable4 = pdemo->createCable(resolution, iteration, 1.61, anchorPositionKinematic4, anchorPositionPhysic4, physic, kinematic);

	cable4->setUseCollision(true);
	cable4->setUseLRA(false);
	cable4->getCollisionShape()->setMargin(0.005);
	cable4->setCollisionParameters(1, 1, 0);
	cable4->setCollisionMargin(0.005);
}

static void initLock(CableDemo* pdemo)
{
	btAlignedObjectArray<btRigidBody*> sphere =  btAlignedObjectArray<btRigidBody*>();

	btTransform pos =  btTransform();
	pos.setIdentity();
	pos.setOrigin(btVector3(-0.2,1, 0));
	
	// Create sphere ring
	
	
	btTransform spherePositionB = btTransform();
	spherePositionB.setIdentity();
	spherePositionB.setOrigin(btVector3(0.30, 0,0.3));

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
	btBoxShape * a = new btBoxShape(btVector3(6, 1, 1));
	// T center
	btBoxShape* b = new btBoxShape(btVector3(0.3, 0.3, 0.2));
	btCylinderShape* c = new btCylinderShapeZ(btVector3(0.15,0.15, 0.5));
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
	Lest->updateMassAtImpact(true, 10, 704, 0, 0.1);

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
	waypointPos.push_back(LestTransform.getOrigin() + btVector3(0, 0.3, 0)); // point de départ
	waypointPos.push_back(AnchorUpPos.getOrigin() + btVector3(0, -0.3, 0)); // Arrivée
	
	btCable* cable = pdemo->createCableWaypoint(resolution, iterations, 1, waypointPos, Lest, AnchorUp ,true,true);
	//btCable* cable = pdemo->createCable(resolution, iterations, 3, LestTransform.getOrigin() + btVector3(0, 0.5, 0), AnchorUpPos.getOrigin() + btVector3(0, -0.5, 0), AnchorUp, Lest, false, false);
	cable->setFriction(1);
	cable->setUseBending(true);
	cable->setUseCollision(true);
	cable->setUseLRA(true);
	cable->getCollisionShape()->setMargin(margin);
	cable->setCollisionMargin(margin);
	cable->setCollisionResponseActive(true);
	cable->setCollisionViscosity(51);
	// cable->setCollisionViscosity(100);
	cable->setCollisionStiffness(0, 1000, 0, 1);
	cable->setCollisionParameters(3, 3, 0);

	cable->m_anchors[0].BodyMassRatio = 0.1;
	cable->m_anchors[1].BodyMassRatio = 0.1;

	pdemo->SetCameraPosition(btVector3(0,2,-3));
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
	te.setOrigin(btVector3(0, 1.9,- 1));
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
	cable->setCollisionParameters(1,1,0);
	cable->setCollisionMargin(margin);
	pdemo->SetCameraPosition(btVector3(0, 0.5, 5));
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

	btRigidBody* spheres = pdemo->createRigidBody(100000, t, compound);
	

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
	
	btCable* cable = pdemo->createCable(resolution, iterations, 3, transformRight.getOrigin(), transformLeft.getOrigin(), bodyLeftAnchor, bodyRightAnchor,true,true);
	
	//cable->appendAnchor(10, spheres, true);
	spheres->setFriction(1);
	cable->setFriction(0);

	cable->setUseCollision(true);
	cable->getCollisionShape()->setMargin(margin);
	cable->setUseLRA(true);
	cable->setCollisionViscosity(50);
	cable->setCollisionStiffness(0,1000000000,0,1);
	cable->setCollisionParameters(3,3,0);
	cable->setCollisionMargin(margin);
	pdemo->SetCameraPosition(btVector3(0, 3.5, 0));

}

static void Init_TestCollisionFallingA18(CableDemo* pdemo)
{
	// Shape
	btCollisionShape* cubeShape = new btBoxShape(btVector3(0.25, 0.25, 0.25));
	// Resolution's cable
	int resolution = 40;
	int iterations = 70;
	btScalar margin = 0.01;

	// Blockeurs
	btTransform blocCompound = btTransform();
	blocCompound.setIdentity();
	blocCompound.setOrigin(btVector3(0, 8, -6));
	btCompoundShape* c = new btCompoundShape();

	/*
	btTransform t = btTransform();
	t.setIdentity();
	t.setOrigin(btVector3(-1, 0, 0));
	c->addChildShape(t, cubeShape);

	// Blockeurs
	btTransform y = btTransform();
	y.setIdentity();
	y.setOrigin(btVector3(1, 0, 0));
	c->addChildShape(y, cubeShape);

	btTransform z = btTransform();
	z.setIdentity();
	z.setOrigin(btVector3(0, 0, 1));
	c->addChildShape(z, cubeShape);

	btTransform zz = btTransform();
	zz.setIdentity();
	zz.setOrigin(btVector3(0, 0, -1));
	c->addChildShape(zz, cubeShape);
	*/

	btTransform t = btTransform();
	t.setIdentity();
	t.setOrigin(btVector3(0, 0, 6));
	c->addChildShape(t, new btBoxShape(btVector3(5, 0.5, 0.5)));

	btTransform y = btTransform();
	y.setIdentity();
	y.setOrigin(btVector3(0, 0, 0));
	c->addChildShape(y, new btBoxShape(btVector3(5, 0.5, 5)));
	//btRigidBody* cubeBis = pdemo->createRigidBody(100, y, cubeShape);

	btRigidBody* obj = pdemo->createRigidBody(1000, blocCompound, c);
	//obj->updateInertiaTensor();

	btTransform Lest = btTransform();
	Lest.setIdentity();
	//Lest.setOrigin(btVector3(0, 6, 0));
	Lest.setOrigin(btVector3(0, 6.5, 2.8));
	//btRigidBody* LestBody = pdemo->createRigidBody(100, Lest, new btBoxShape(btVector3(5, 0.5, 5)));
	btRigidBody* LestBody = pdemo->createRigidBody(0, Lest, new btBoxShape(btVector3(0.5, 10, 0.5)));

	btTransform trAnchorUp = btTransform();
	trAnchorUp.setIdentity();
	trAnchorUp.setOrigin(btVector3(0, 10, 0));
	btRigidBody* anchorUp = pdemo->createRigidBody(0, trAnchorUp, new btBoxShape(btVector3(0.5, 0.5, 0.5)));

	btVector3 groundPos = btVector3(0, -15, 0);
	btTransform transformGround = btTransform();
	transformGround.setIdentity();
	transformGround.setOrigin(groundPos);
	btRigidBody* ground = pdemo->createRigidBody(0, transformGround, new btBoxShape(btVector3(50, 2, 50)));

	btAlignedObjectArray<btVector3> waypointPos = btAlignedObjectArray<btVector3>();
	waypointPos.push_back(Lest.getOrigin() + btVector3(0, 0, 0));
	waypointPos.push_back(Lest.getOrigin() + btVector3(0, 0, -3.5));  // point de d�part
	waypointPos.push_back(Lest.getOrigin() + btVector3(0, 3, -3.5));  // Arriv�e
	waypointPos.push_back(trAnchorUp.getOrigin());                    // Arriv�e

	// Masses
	btTransform transformBlockA = btTransform();
	transformBlockA.setIdentity();
	transformBlockA.setOrigin(btVector3(6.1, -5, 0));
	btRigidBody* BlockA = pdemo->createRigidBody(0, transformBlockA, new btBoxShape(btVector3(1, 10, 10)));

	btTransform transformBlockB = btTransform();
	transformBlockB.setIdentity();
	transformBlockB.setOrigin(btVector3(-6.1, -5, 0));
	btRigidBody* BlockB = pdemo->createRigidBody(0, transformBlockB, new btBoxShape(btVector3(1, 10, 10)));

	btCable* cable = pdemo->createCableWaypoint(resolution, iterations, 1, waypointPos, LestBody, anchorUp, true, true);

	obj->setFriction(1);
	cable->setFriction(1);

	//btCable* cable = pdemo->createCable(resolution, iterations, 1, transformRight.getOrigin() + btVector3(0, 0.2, 0), transformLeft.getOrigin() + btVector3(0.4, 0, 0), bodyLeftAnchor, bodyRightAnchor, true, false);
	cable->setUseBending(true);
	cable->setUseCollision(true);
	cable->setUseLRA(true);

	cable->getCollisionShape()->setMargin(margin);
	cable->setCollisionMargin(margin);
	cable->setCollisionParameters(1, 1, 0);
	cable->setCollisionMode(1);
	pdemo->SetCameraPosition(btVector3(0, 5, 0));
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
	btRigidBody* LestBody = pdemo->createRigidBody(1000000, Lest, new btBoxShape(btVector3(10, 0.5, 0.5)), 1234);

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

	// Claw
	btTransform trClaw = btTransform();
	trClaw.setIdentity();
	trClaw.setOrigin(btVector3(0, 7.5, 0));
	btRigidBody* claw = pdemo->createRigidBody(0, trClaw, new btBoxShape(btVector3(4, 0.5, 0.5)), 6789);
	claw->setCollisionFlags(claw->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
	claw->m_redirectionTarget = a18;
	claw->m_localTransform = btTransform(btMatrix3x3::getIdentity(), btVector3(0, 6.5, 0));
	a18->m_kinematicChildren.push_back(claw);

	// Cable's Waypoints
	btAlignedObjectArray<btVector3> waypointPos = btAlignedObjectArray<btVector3>();
	waypointPos.push_back(Lest.getOrigin() + btVector3(0, 0, 0));
	waypointPos.push_back(Lest.getOrigin() + btVector3(0, 0, -3.5));  // point de départ
	waypointPos.push_back(Lest.getOrigin() + btVector3(0, 3, -3.5));  // Arrivée
	waypointPos.push_back(trAnchorUp.getOrigin());  // Arrivée
	
	// Resolution's cable
	int resolution = 60;
	int iterations = 20;
	btScalar margin = 0.01;

	// Cable
	btCable* cable = pdemo->createCableWaypoint(resolution, iterations, 1, waypointPos, LestBody, anchorUp, true, true);
	cable->setFriction(1);
	cable->setUseBending(true);
	cable->setUseCollision(true);
	cable->setUseLRA(true);
	cable->getCollisionShape()->setMargin(margin);
	cable->setCollisionMargin(margin);
	cable->setCollisionResponseActive(true);
	cable->setCollisionViscosity(50);
	cable->setCollisionStiffness(0, 10000000, 0, 1);
	cable->setCollisionParameters(3, 3, 0);

	//cable->setCollisionMode(1);

	for (int i = 0; i < cable->m_anchors.size(); ++i)
	{
		cable->m_anchors[i].BodyMassRatio = 0.000015;
	}
	pdemo->SetCameraPosition(btVector3(0, 10, 0));
}

static void Init_TestCollisionCableConvexHullOnMeshSphere(CableDemo* pdemo)
{
	// Resolution's cable
	int resolution = 150;
	int iterations = 100;
	btScalar margin = 0.01;

	// Shape
	btCollisionShape* cubeShape = new btBoxShape(btVector3(1, 1, 1));

	/* Loading object */
	//load our obj mesh
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

	/* btConvexHullShape* shape = new btConvexHullShape();
	for (int i = 0; i < glmesh->m_numvertices; i++)
	{
		const GLInstanceVertex& v = glmesh->m_vertices->at(i);
		float temp = v.xyzw[0];
		btVector3 vtx(v.xyzw[0], v.xyzw[1], v.xyzw[2]);
		shape->addPoint(vtx);
	}
	shape->setLocalScaling(btVector3(2,2,2));
	shape->optimizeConvexHull();
	shape->initializePolyhedralFeatures();
	*/
	
	btTriangleMesh* meshInterface = new btTriangleMesh();
	for (int i = 0; i < glmesh->m_numIndices / 3; i++)
	{
		const GLInstanceVertex& cv0 = glmesh->m_vertices->at(glmesh->m_indices->at(i * 3));
		const btVector3& v0 = btVector3(cv0.xyzw[0], cv0.xyzw[1], cv0.xyzw[2]);
		const GLInstanceVertex& cv1 = glmesh->m_vertices->at(glmesh->m_indices->at(i * 3 + 1));
		const btVector3& v1 = btVector3(cv1.xyzw[0], cv1.xyzw[1], cv1.xyzw[2]);
		const GLInstanceVertex& cv2 = glmesh->m_vertices->at(glmesh->m_indices->at(i * 3 + 2));
		const btVector3& v2 = btVector3(cv2.xyzw[0], cv2.xyzw[1], cv2.xyzw[2]);
		meshInterface->addTriangle(v0, v1, v2);
	}	
	btBvhTriangleMeshShape* shape = new btBvhTriangleMeshShape(meshInterface, true, true);
	shape->setLocalScaling(btVector3(2, 2, 2));
	btTriangleInfoMap* triangleInfoMap = new btTriangleInfoMap();
	btGenerateInternalEdgeInfo(shape, triangleInfoMap);
	
	


	btTransform t = btTransform();
	t.setIdentity();
	t.setOrigin(btVector3(0, 3, 0));
	//btQuaternion r = btQuaternion();
	//r.setRotation(btVector3(0, 1, 0), SIMD_PI * 0.40);
	//t.setRotation(r);
	shape->setMargin(0.0);
	btRigidBody* sphere = pdemo->createRigidBody(0, t, shape);
	

	btVector3 groundPos = btVector3(0, -8, 0);
	btTransform transformGround = btTransform();
	transformGround.setIdentity();
	transformGround.setOrigin(groundPos);
	btRigidBody* ground = pdemo->createRigidBody(0, transformGround, new btBoxShape(btVector3(50, 2, 50)));

	btTransform transformRight = btTransform();
	transformRight.setIdentity();
	transformRight.setOrigin(btVector3(5, 7, 0));
	btRigidBody* bodyRightAnchor = pdemo->createRigidBody(0, transformRight, new btBoxShape(btVector3(1, 1, 1)));

	btTransform transformLeft = btTransform();
	transformLeft.setIdentity();
	transformLeft.setOrigin(btVector3(-5, 7, 0));
	btRigidBody* bodyLeftAnchor = pdemo->createRigidBody(1000, transformLeft, new btBoxShape(btVector3(1, 1, 1)));

	btCable* cable = pdemo->createCable(resolution, iterations, 1, transformRight.getOrigin(), transformLeft.getOrigin() + btVector3(1, 0, 0), bodyLeftAnchor, bodyRightAnchor);
	cable->setUseCollision(true);
	cable->getCollisionShape()->setMargin(margin);
	cable->setUseLRA(true);
	cable->setCollisionMargin(margin);
	cable->setCollisionParameters(1,1,0);

	pdemo->SetCameraPosition(btVector3(0, 0.5, 0));
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

	btRigidBody* ring = pdemo->createRigidBody(100, pos, ringShape);
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
	cable->setCollisionParameters(1, 3, 0);
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

	
	btRigidBody* box = pdemo->createRigidBody(100, pos, a18shape);
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
	waypointPos.push_back(transformLeft.getOrigin() + btVector3(0, 0, 0));  // Arrivée

	btCable* cable = pdemo->createCableWaypoint(resolution, iterations, 3, waypointPos, bodyRightAnchor, bodyLeftAnchor, true, true);
	//cable->appendAnchor(50, box,true);
	
	box->setFriction(1);
	cable->setFriction(1);
	
	cable->setUseCollision(true);
	cable->getCollisionShape()->setMargin(margin);
	cable->setCollisionMargin(margin);
	cable->setUseLRA(true);
	cable->setCollisionParameters(1, 5, 0);
	pdemo->SetCameraPosition(btVector3(0, -3, 0));
}

static void Init_TestCollisionOn1Node (CableDemo* pdemo)
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

	btRigidBody* obj = pdemo->createRigidBody(100, t, compoundCubes);
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
	cable->setCollisionParameters(1, 1, 0);
	cable->setCollisionViscosity(100);
	//cable->setCollisionStiffness(100, 10000, 0, 0.01);
	
	pdemo->SetCameraPosition(btVector3(0, -3, 0));
}

static void Init_TestClaw(CableDemo* pdemo)
{
	// Shape
	btCollisionShape* cubeShape = new btBoxShape(btVector3(0.25, 0.25, 0.25));
	// Resolution's cable
	int resolution = 50;
	int iterations = 100;
	btScalar margin = 0.005;

	// compound
	btTransform blocCompound = btTransform();
	blocCompound.setIdentity();
	blocCompound.setOrigin(btVector3(1.10, 5, -5));
	btCompoundShape* c = new btCompoundShape();

	// body
	auto a = new btBoxShape(btVector3(1, 1, 5));
	a->setMargin(0.01);
	btTransform t = btTransform();
	t.setIdentity();
	t.setOrigin(btVector3(0, 0, 0));
	c->addChildShape(t,a);

	btTransform y = btTransform();
	y.setIdentity();
	y.setOrigin(btVector3(-1.5, 0, 4));
	
	// Claw
	auto b = new btBoxShape(btVector3(3, 0.25, 0.25));
	b->setMargin(0.01);
	btQuaternion rotation = btQuaternion(btVector3(0, 1, 0), SIMD_PI * 0.45);
	y.setRotation(rotation);
	c->addChildShape(y, b);

	btRigidBody* obj = pdemo->createRigidBody(100, blocCompound, c);
	obj->setGravity(btVector3(0, 0, 5));
	obj->getCollisionShape()->setMargin(0);

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
	cable->setCollisionViscosity(100);
	cable->setUseBending(true);
	cable->setUseCollision(true);
	cable->setUseLRA(true);

	cable->getCollisionShape()->setMargin(margin);
	cable->setCollisionMargin(margin);
	cable->setCollisionParameters(1, 3, 0);
}

static void Init_TestConstraintClawA18(CableDemo* pdemo)
{
	// A18
	btTransform transformA18 = btTransform::getIdentity();
	transformA18.setOrigin(btVector3(-4, 0, 0));
	// transformA18.setRotation(btQuaternion(btVector3(0, 1, 0), SIMD_PI / 4.0));
	btRigidBody* a18 = pdemo->createRigidBody(704, transformA18, new btBoxShape(btVector3(3, 0.4, 0.4)));
	a18->setGravity(btVector3(0, 0, 0));

	// A18's claw
	btTransform transformA18Claw = btTransform::getIdentity();
	transformA18Claw.setOrigin(btVector3(-2, 0, 0));
	transformA18Claw.setRotation(btQuaternion(btVector3(0, 1, 0), -SIMD_PI/4.0));
	btRigidBody* a18Claw = pdemo->createRigidBody(1, transformA18Claw, new btBoxShape(btVector3(1, 0.2, 0.2)));
	a18Claw->getCollisionShape()->setMargin(0);
	a18Claw->setGravity(btVector3(0, 0, 0)); 

	// A18's claw FixedConstraint
	btTransform frameInA = btTransform::getIdentity();
	btTransform frameLocalA = a18Claw->getWorldTransform().inverse() * a18->getWorldTransform();
	frameInA.setOrigin(frameLocalA.getOrigin()); // + pivot
	frameInA.setBasis(a18Claw->getWorldTransform().inverse().getBasis());
	btTransform frameInB = btTransform::getIdentity();
	frameInB.setBasis(a18->getWorldTransform().inverse().getBasis());
	btFixedConstraint* fixedA18 = pdemo->createFixedConstraint(*a18Claw, *a18, frameInA, frameInB, 256);
	// fixedA18->setAngularLowerLimit(btVector3(0, -SIMD_PI / 2.0, 0));
	// fixedA18->setAngularUpperLimit(btVector3(0, -SIMD_PI / 2.0, 0));

	// Lest
	btTransform transformLest = btTransform::getIdentity();
	transformLest.setOrigin(btVector3(0, -3, 0));
	btRigidBody* lest = pdemo->createRigidBody(10, transformLest, new btBoxShape(btVector3(0.1, 0.2, 0.11)));
	lest->getCollisionShape()->setMargin(0);

	// Lest's support
	btTransform transformLestSupport = btTransform::getIdentity();
	transformLestSupport.setOrigin(btVector3(0, 3, 0));
	btRigidBody* lestSupport = pdemo->createRigidBody(0, transformLestSupport, new btBoxShape(btVector3(1, 1, 1)));
	lestSupport->getCollisionShape()->setMargin(0);

	// Cable
	int resolution = 75;
	int iterations = 100;
	btScalar margin = 0.005;
	btAlignedObjectArray<btVector3> waypointPos = btAlignedObjectArray<btVector3>();
	waypointPos.push_back(transformLest.getOrigin());
	waypointPos.push_back(transformLestSupport.getOrigin());  // Arrivée
	btCable* cable = pdemo->createCableWaypoint(resolution, iterations, 1, waypointPos, lest, lestSupport, true, true);
	cable->setCollisionViscosity(100);
	cable->setUseBending(true);
	cable->setUseCollision(true);
	cable->setUseLRA(true);
	cable->getCollisionShape()->setMargin(margin);
	cable->setCollisionMargin(margin);
	cable->setCollisionParameters(5, 1, 0);
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
	cable->setCollisionParameters(1, 1, 0);
}

static void Init_DetachA18(CableDemo* pdemo)
{
	/// Create A18:
	// Shapes
	btCollisionShape* a18Shape = new btBoxShape(btVector3(0.4, 0.4, 2));
	btCollisionShape* clawShape = new btCylinderShapeX(btVector3(0.75, 0.15, 0.15));
	// Positions
	btVector3 a18Position = btVector3(0, 0, 0);
	btVector3 clawPosition = btVector3(0, 1, 2.30);
	// Transform
	btTransform a18Transform = btTransform();
	a18Transform.setIdentity();
	a18Transform.setOrigin(a18Position);
	a18Transform.setRotation(btQuaternion(btVector3(1,0,0),75));
	btTransform clawTransform = btTransform();
	clawTransform.setIdentity();
	clawTransform.setOrigin(clawPosition);
	// Rigid bodies
	btRigidBody* a18Body = pdemo->createRigidBody(704, a18Transform, a18Shape);
	a18Body->setGravity(btVector3(0, 0, -9.81));
	a18Body->setLinearVelocity(btVector3(0, 0, -30));
	btRigidBody* clawBody = pdemo->createRigidBody(0, clawTransform, clawShape);
	clawBody->setCollisionFlags(clawBody->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
	clawBody->m_redirectionTarget = a18Body;
	clawBody->m_localTransform = btTransform(btMatrix3x3::getIdentity(), btVector3(0, 0, 2.30));
	a18Body->m_kinematicChildren.push_back(clawBody);


	/// Create Inspector125 + Lest:
	// Shapes
	btCollisionShape* inspector125Shape = new btBoxShape(btVector3(1, 1, 5));
	btCollisionShape* lestShape = new btCylinderShape(btVector3(0.1, 0.1, 1));
	// Positions
	btVector3 inspector125Position = btVector3(0, 3, 7.5);
	btVector3 lestPosition = btVector3(0, -0.5, 3);
	// Transform
	btTransform inspector125Transform = btTransform();
	inspector125Transform.setIdentity();
	inspector125Transform.setOrigin(inspector125Position);
	btTransform lestTransform = btTransform();
	lestTransform.setIdentity();
	lestTransform.setOrigin(lestPosition);
	// Rigid bodies
	btRigidBody* inspector125Body = pdemo->createRigidBody(0, inspector125Transform, inspector125Shape);
	btRigidBody* lestBody = pdemo->createRigidBody(0, lestTransform, lestShape);

	/// Create Cable:
	// Resolution's cable
	int resolution = 50;
	int iterations = 100;
	btScalar margin = 0.01;
	btAlignedObjectArray<btVector3> waypointPos = btAlignedObjectArray<btVector3>();
	waypointPos.push_back(lestPosition + btVector3(0, 0.085, 0));              // Start
	waypointPos.push_back(lestPosition + btVector3(0, 1, -0.95));              // Bot A18
    waypointPos.push_back(lestPosition + btVector3(0, 1.9, -1.4));             // Top A18
	waypointPos.push_back(inspector125Position + btVector3(0, -0.95, -4.95));  // End
	btCable* cable = pdemo->createCableWaypoint(resolution, iterations, 1, waypointPos, lestBody, inspector125Body, true, true);
	cable->setUseCollision(true);
	cable->getCollisionShape()->setMargin(0);
	cable->setCollisionMargin(margin);
	cable->setUseLRA(true);
	cable->setCollisionParameters(1, 1, 0);

	// cable->setCollisionParameters(3, 3, 0);
	// cable->setCollisionViscosity(20);
	// double dataX[5] = {0, 0.001, 0.01, 0.5, 1};
	// double dataY[5] = {0, 1, 100000, 5000000, 10000000};
	// cable->updateCurveResponse(dataX, dataY, 5);
	// cable->setCollisionMode(1);

	/// Setup Scene:
	// Camera
	pdemo->SetCameraPosition(btVector3(-2.5, 0, 2.5));
	pdemo->SetCameraRotation(5,90,0);
	inspector125Body->setLinearVelocity(btVector3(0, 0, 2));
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
		Init_TestCollisionFallingA18,
		Init_TestCollisionFallingA18Constraint,
		Init_TestCollisionCableConvexHullOnMeshSphere,
		Init_TestCollisionRingBox,
		Init_TestCollisionRingSphere,
		Init_TestCollisionOn1Node, 
		Init_TestClaw,
		Init_TestConstraintClawA18,
		Init_Growth,
		Init_TestCableCollisionMt,
		Init_CableHydro,
		Init_CableBending,
		Init_TwoCablesOneCube,
		Init_DetachA18
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
	m_softBodyWorldInfo.numIteration = substepSolver;

	m_autocam = false;
	m_raycast = false;
	m_cutting = false;
	m_results.fraction = 1.f;

	demofncs[currentCableDemo](this);

	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
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
