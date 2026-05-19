/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  https://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include "btSoftRigidDynamicsWorld.h"
#include "LinearMath/btQuickprof.h"

//softbody & helpers
#include "btSoftBody.h"
#include "btSoftBodyHelpers.h"
#include "btSoftBodySolvers.h"
#include "btDefaultSoftBodySolver.h"
#include "BulletCollision/CollisionDispatch/btCollisionDispatcherMt.h"
#include "LinearMath/btSerializer.h"

btSoftRigidDynamicsWorld::btSoftRigidDynamicsWorld(
	btSoftBodyWorldInfo* worldInfo,
	btConstraintSolver* constraintSolver,
	btCollisionConfiguration* collisionConfiguration,
	btSoftBodySolver* softBodySolver) : btDiscreteDynamicsWorld(worldInfo->m_dispatcher, worldInfo->m_broadphase, constraintSolver, collisionConfiguration),
										m_softBodySolver(softBodySolver),
										m_ownsSolver(false)

{
	if (!m_softBodySolver)
	{
		void* ptr = btAlignedAlloc(sizeof(btDefaultSoftBodySolver), 16);
		m_softBodySolver = new (ptr) btDefaultSoftBodySolver();
		m_ownsSolver = true;
	}

	m_drawFlags = fDrawFlags::Std;
	m_drawNodeTree = true;
	m_drawFaceTree = false;
	m_drawClusterTree = false;

	m_sbi = btSoftBodyWorldInfo(worldInfo);
	// Check if it's unecessary
	//m_sbi.m_sparsesdf.Initialize();

	// Set numThread From SoftBodyWorldInfo
	m_softBodySolver->numThread = m_sbi.numThread;
	

	// () used to force 0 initialization
	m_nodeForces = new btSoftBody::NodeForces[m_sbi.maxNodeNumber]();
	m_nodesPos = new btCable::NodePos[m_sbi.maxNodeNumber]();
	m_nodesData = new btCable::NodeData[m_sbi.maxNodeNumber]();
	m_cableIndexesArray = new int[m_sbi.maxNodeNumber]();

	// Pool of max nbr of cable TODO: check if m_sbi value can change after initialization
	m_cablesData = new btCable::CableData[m_sbi.maxCableNumber]();

	m_defaultCableData = btCable::CableData();
	m_defaultNodeData = btCable::NodeData();
	m_defaultNodePos = btCable::NodePos();
	m_defaultNodeForces = btSoftBody::NodeForces();
}


btSoftRigidDynamicsWorld::~btSoftRigidDynamicsWorld()
{
	if (m_ownsSolver)
	{
		m_softBodySolver->~btSoftBodySolver();
		btAlignedFree(m_softBodySolver);
	}

	if (m_nodeForces != nullptr)
	{
		delete[] m_nodeForces;
	}

	if (m_nodesPos != nullptr)
	{
		delete[] m_nodesPos;
	}

	if (m_nodesData != nullptr)
	{
		delete[] m_nodesData;
	}

	if (m_cablesData != nullptr)
	{
		delete[] m_cablesData;
	}

	if (m_cableIndexesArray != nullptr)
	{
		delete[] m_cableIndexesArray;
	}
}

void btSoftRigidDynamicsWorld::predictUnconstraintMotion(btScalar timeStep)
{
	btDiscreteDynamicsWorld::predictUnconstraintMotion(timeStep);
	{
		BT_PROFILE("predictUnconstraintMotionSoftBody");
		m_softBodySolver->predictMotion(float(timeStep));
	}
}

void btSoftRigidDynamicsWorld::prepareSingleStepSimulation()
{
	// Reset arrays in order to avoid garbage values causing wrong calculations
	if (m_indexSubIteration == m_subIteration - 1)
	{
		std::fill_n(m_nodesData, m_sbi.maxNodeNumber, m_defaultNodeData);
		std::fill_n(m_nodesPos, m_sbi.maxNodeNumber, m_defaultNodePos);
		std::fill_n(m_cablesData, m_sbi.maxCableNumber, m_defaultCableData);
		std::fill_n(m_nodeForces, m_sbi.maxNodeNumber, m_defaultNodeForces);
	}

	//Set Arrays
	m_totalNodeCount = 0;
	m_totalActiveNodeCount = 0;
	m_totalPhysicsNodeCount = 0;

	m_totalCableCount = 0;
	m_totalActiveCableCount = 0;
	m_totalPhysicsCableCount = 0;
	
	int nodesIndex = 0;
	int physicCableIndex = 0;
	for (int i = 0; i < m_softBodies.size(); i++)
	{		
		btCable* cable = reinterpret_cast<btCable*>(m_softBodies[i]);
		std::size_t const nodesCount = cable->m_nodes.size();

		// Store each cable data into a single one for GPU computing of Hydro and Aero Forces
		if (cable->getUseHydroAero())
		{
			cable->setStartIndex(nodesIndex);

			// Clean and fast chunks initialization (see https://godbolt.org/z/nWK63GYcc)
			std::fill_n(m_cableIndexesArray + nodesIndex, nodesCount, physicCableIndex);
			memcpy(m_nodesData + nodesIndex, cable->getNodeData(), nodesCount * cable->NodeDataSize);
			memcpy(m_nodesPos + nodesIndex, cable->getNodePos(), nodesCount * cable->NodePosSize);

			// First update `endIndex` before performing the copy
			nodesIndex += nodesCount;
			cable->setEndIndex(nodesIndex - 1);
			//memcpy(m_cablesData + i, cable->getCableData(), cable->CableDataSize); // remplacer i par physicCableIndex
			memcpy(m_cablesData + physicCableIndex, cable->getCableData(), cable->CableDataSize); 


			physicCableIndex += 1;
		}

		if (cable->getActivationState() == ACTIVE_TAG)
		{
			m_totalActiveNodeCount += nodesCount;
			m_totalActiveCableCount += 1;
		}

		m_totalNodeCount += nodesCount;
		m_totalCableCount += 1;
	}

	m_totalPhysicsNodeCount = nodesIndex;
	m_totalPhysicsCableCount = physicCableIndex;
}

void btSoftRigidDynamicsWorld::internalSingleStepSimulation(btScalar timeStep)
{
	// Let the solver grab the soft bodies and if necessary optimize for it
	m_softBodySolver->optimize(getSoftBodyArray());

	if (!m_softBodySolver->checkInitialized())
	{
		btAssert("Solver initialization failed\n");
	}

	btDiscreteDynamicsWorld::internalSingleStepSimulation(timeStep);

	updateCableCollisionColliders(timeStep);

	///solve soft bodies constraints
	solveSoftBodiesConstraints(timeStep);
}

void btSoftRigidDynamicsWorld::internalSingleStepSimulationOneCable(btScalar timeStep, btCable* cable)
{
	// Let the solver grab the soft bodies and if necessary optimize for it
	m_softBodySolver->optimize(getSoftBodyArray());

	if (!m_softBodySolver->checkInitialized())
	{
		btAssert("Solver initialization failed\n");
	}

	btDiscreteDynamicsWorld::internalSingleStepSimulation(timeStep);

	updateCableCollisionColliders(timeStep);

	///solve soft bodies constraints
	solveSoftBodyConstraintsOneCable(timeStep, cable);
}

void btSoftRigidDynamicsWorld::updateCableCollisionColliders(btScalar timeStep)
{
	for (int i = 0; i < m_collisionObjects.size(); ++i)
	{
		btRigidBody* rb = btRigidBody::upcast(m_collisionObjects[i]);
		if (rb) rb->updateCableCollision(timeStep);	
	}
}

void btSoftRigidDynamicsWorld::solveSoftBodiesConstraints(btScalar timeStep)
{
	BT_PROFILE("solveSoftConstraints");

	if (m_softBodies.size())
	{
		btSoftBody::solveClusters(m_softBodies);
	}

	btCollisionDispatcherMt* dispatcher = static_cast<btCollisionDispatcherMt*>(m_dispatcher1);

	dispatcher->m_batchUpdating = true;
	
	// Solve constraints solver-wise
	m_softBodySolver->solveConstraints(timeStep * m_softBodySolver->getTimeScale());

	dispatcher->m_batchUpdating = false;
}

void btSoftRigidDynamicsWorld::solveSoftBodyConstraintsOneCable(btScalar timeStep, btCable* cable)
{
	BT_PROFILE("solveSoftConstraints");

	if (m_softBodies.size())
	{
		btSoftBody::solveClusters(m_softBodies);
	}

	btCollisionDispatcherMt* dispatcher = static_cast<btCollisionDispatcherMt*>(m_dispatcher1);

	dispatcher->m_batchUpdating = true;
	
	// Solve constraints solver-wise
	((btDefaultSoftBodySolver*) m_softBodySolver)->solveConstraintsOneCable(timeStep * m_softBodySolver->getTimeScale(), cable);

	dispatcher->m_batchUpdating = false;
}

void btSoftRigidDynamicsWorld::addSoftBody(btSoftBody* body, int collisionFilterGroup, int collisionFilterMask)
{
	m_softBodies.push_back(body);

	// Set the soft body solver that will deal with this body
	// to be the world's solver
	body->setSoftBodySolver(m_softBodySolver);

	btCollisionWorld::addCollisionObject(body,
										 collisionFilterGroup,
										 collisionFilterMask);
}

void btSoftRigidDynamicsWorld::removeSoftBody(btSoftBody* body)
{
	m_softBodies.remove(body);

	btCollisionWorld::removeCollisionObject(body);
}

void btSoftRigidDynamicsWorld::removeCollisionObject(btCollisionObject* collisionObject)
{
	btSoftBody* body = btSoftBody::upcast(collisionObject);
	if (body)
		removeSoftBody(body);
	else
		btDiscreteDynamicsWorld::removeCollisionObject(collisionObject);
}

void btSoftRigidDynamicsWorld::debugDrawWorld()
{
	btDiscreteDynamicsWorld::debugDrawWorld();

	if (getDebugDrawer())
	{
		int i;
		for (i = 0; i < this->m_softBodies.size(); i++)
		{
			btSoftBody* psb = (btSoftBody*)this->m_softBodies[i];
			if (getDebugDrawer() && (getDebugDrawer()->getDebugMode() & (btIDebugDraw::DBG_DrawWireframe)))
			{
				btSoftBodyHelpers::DrawFrame(psb, m_debugDrawer);
				btSoftBodyHelpers::Draw(psb, m_debugDrawer, m_drawFlags);
			}

			if (m_debugDrawer && (m_debugDrawer->getDebugMode() & btIDebugDraw::DBG_DrawAabb))
			{
				if (m_drawNodeTree) btSoftBodyHelpers::DrawNodeTree(psb, m_debugDrawer);
				if (m_drawFaceTree) btSoftBodyHelpers::DrawFaceTree(psb, m_debugDrawer);
				if (m_drawClusterTree) btSoftBodyHelpers::DrawClusterTree(psb, m_debugDrawer);
			}
		}
	}
}

struct btSoftSingleRayCallback : public btBroadphaseRayCallback
{
	btVector3 m_rayFromWorld;
	btVector3 m_rayToWorld;
	btTransform m_rayFromTrans;
	btTransform m_rayToTrans;
	btVector3 m_hitNormal;

	const btSoftRigidDynamicsWorld* m_world;
	btCollisionWorld::RayResultCallback& m_resultCallback;

	btSoftSingleRayCallback(const btVector3& rayFromWorld, const btVector3& rayToWorld, const btSoftRigidDynamicsWorld* world, btCollisionWorld::RayResultCallback& resultCallback)
		: m_rayFromWorld(rayFromWorld),
		  m_rayToWorld(rayToWorld),
		  m_world(world),
		  m_resultCallback(resultCallback)
	{
		m_rayFromTrans.setIdentity();
		m_rayFromTrans.setOrigin(m_rayFromWorld);
		m_rayToTrans.setIdentity();
		m_rayToTrans.setOrigin(m_rayToWorld);

		btVector3 rayDir = (rayToWorld - rayFromWorld);

		if (rayDir.fuzzyZero()) return;

		rayDir.normalize();
		///what about division by zero? --> just set rayDirection[i] to INF/1e30
		m_rayDirectionInverse[0] = rayDir[0] == btScalar(0.0) ? btScalar(1e30) : btScalar(1.0) / rayDir[0];
		m_rayDirectionInverse[1] = rayDir[1] == btScalar(0.0) ? btScalar(1e30) : btScalar(1.0) / rayDir[1];
		m_rayDirectionInverse[2] = rayDir[2] == btScalar(0.0) ? btScalar(1e30) : btScalar(1.0) / rayDir[2];
		m_signs[0] = m_rayDirectionInverse[0] < 0.0;
		m_signs[1] = m_rayDirectionInverse[1] < 0.0;
		m_signs[2] = m_rayDirectionInverse[2] < 0.0;

		m_lambda_max = rayDir.dot(m_rayToWorld - m_rayFromWorld);
	}

	virtual bool process(const btBroadphaseProxy* proxy)
	{
		///terminate further ray tests, once the closestHitFraction reached zero
		if (m_resultCallback.m_closestHitFraction == btScalar(0.f))
			return false;

		btCollisionObject* collisionObject = (btCollisionObject*)proxy->m_clientObject;

		//only perform raycast if filterMask matches
		if (m_resultCallback.needsCollision(collisionObject->getBroadphaseHandle()))
		{
			//RigidcollisionObject* collisionObject = ctrl->GetRigidcollisionObject();
			//btVector3 collisionObjectAabbMin,collisionObjectAabbMax;
#if 0
#ifdef RECALCULATE_AABB
			btVector3 collisionObjectAabbMin,collisionObjectAabbMax;
			collisionObject->getCollisionShape()->getAabb(collisionObject->getWorldTransform(),collisionObjectAabbMin,collisionObjectAabbMax);
#else
			//getBroadphase()->getAabb(collisionObject->getBroadphaseHandle(),collisionObjectAabbMin,collisionObjectAabbMax);
			const btVector3& collisionObjectAabbMin = collisionObject->getBroadphaseHandle()->m_aabbMin;
			const btVector3& collisionObjectAabbMax = collisionObject->getBroadphaseHandle()->m_aabbMax;
#endif
#endif
			//btScalar hitLambda = m_resultCallback.m_closestHitFraction;
			//culling already done by broadphase
			//if (btRayAabb(m_rayFromWorld,m_rayToWorld,collisionObjectAabbMin,collisionObjectAabbMax,hitLambda,m_hitNormal))
			{
				m_world->rayTestSingle(m_rayFromTrans, m_rayToTrans,
									   collisionObject,
									   collisionObject->getCollisionShape(),
									   collisionObject->getWorldTransform(),
									   m_resultCallback);
			}
		}
		return true;
	}
};

void btSoftRigidDynamicsWorld::rayTest(const btVector3& rayFromWorld, const btVector3& rayToWorld, RayResultCallback& resultCallback) const
{
	BT_PROFILE("rayTest");
	/// use the broadphase to accelerate the search for objects, based on their aabb
	/// and for each object with ray-aabb overlap, perform an exact ray test
	btSoftSingleRayCallback rayCB(rayFromWorld, rayToWorld, this, resultCallback);

#ifndef USE_BRUTEFORCE_RAYBROADPHASE
	m_broadphasePairCache->rayTest(rayFromWorld, rayToWorld, rayCB);
#else
	for (int i = 0; i < this->getNumCollisionObjects(); i++)
	{
		rayCB.process(m_collisionObjects[i]->getBroadphaseHandle());
	}
#endif  //USE_BRUTEFORCE_RAYBROADPHASE
}

void btSoftRigidDynamicsWorld::rayTestSingle(const btTransform& rayFromTrans, const btTransform& rayToTrans,
											 btCollisionObject* collisionObject,
											 const btCollisionShape* collisionShape,
											 const btTransform& colObjWorldTransform,
											 RayResultCallback& resultCallback)
{
	if (collisionShape->isSoftBody())
	{
		btSoftBody* softBody = btSoftBody::upcast(collisionObject);
		if (softBody)
		{
			btSoftBody::sRayCast softResult;
			if (softBody->rayTest(rayFromTrans.getOrigin(), rayToTrans.getOrigin(), softResult))
			{
				if (softResult.fraction <= resultCallback.m_closestHitFraction)
				{
					btCollisionWorld::LocalShapeInfo shapeInfo;
					shapeInfo.m_shapePart = 0;
					shapeInfo.m_triangleIndex = softResult.index;
					// get the normal
					btVector3 rayDir = rayToTrans.getOrigin() - rayFromTrans.getOrigin();
					btVector3 normal = -rayDir;
					normal.normalize();

					if (softResult.feature == btSoftBody::eFeature::Face)
					{
						normal = softBody->m_faces[softResult.index].m_normal;
						if (normal.dot(rayDir) > 0)
						{
							// normal always point toward origin of the ray
							normal = -normal;
						}
					}

					btCollisionWorld::LocalRayResult rayResult(collisionObject,
															   &shapeInfo,
															   normal,
															   softResult.fraction);
					bool normalInWorldSpace = true;
					resultCallback.addSingleResult(rayResult, normalInWorldSpace);
				}
			}
		}
	}
	else
	{
		btCollisionWorld::rayTestSingle(rayFromTrans, rayToTrans, collisionObject, collisionShape, colObjWorldTransform, resultCallback);
	}
}

void btSoftRigidDynamicsWorld::serializeSoftBodies(btSerializer* serializer)
{
	int i;
	//serialize all collision objects
	for (i = 0; i < m_collisionObjects.size(); i++)
	{
		btCollisionObject* colObj = m_collisionObjects[i];
		if (colObj->getInternalType() & btCollisionObject::CO_SOFT_BODY)
		{
			int len = colObj->calculateSerializeBufferSize();
			btChunk* chunk = serializer->allocate(len, 1);
			const char* structType = colObj->serialize(chunk->m_oldPtr, serializer);
			serializer->finalizeChunk(chunk, structType, BT_SOFTBODY_CODE, colObj);
		}
	}
}

void btSoftRigidDynamicsWorld::serialize(btSerializer* serializer)
{
	serializer->startSerialization();

	serializeDynamicsWorldInfo(serializer);

	serializeSoftBodies(serializer);

	serializeRigidBodies(serializer);

	serializeCollisionObjects(serializer);

	serializer->finishSerialization();
}


void btSoftRigidDynamicsWorld::updateCableForces(btSoftBody::NodeForces* co, int size)
{
	memcpy(m_nodeForces, co, btSoftBody::nodeForcesSize * static_cast<std::size_t>(size));
}

void* btSoftRigidDynamicsWorld::getCablesData()
{
	return m_cablesData;
}

void* btSoftRigidDynamicsWorld::getNodesPos()
{
	return m_nodesPos;
}

void* btSoftRigidDynamicsWorld::getNodesData()
{
	return m_nodesData;
}

int* btSoftRigidDynamicsWorld::getCableIndexesArray()
{
	return m_cableIndexesArray;
}



