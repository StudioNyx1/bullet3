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


#ifndef BT_SOFT_RIGID_DYNAMICS_WORLD_H
#define BT_SOFT_RIGID_DYNAMICS_WORLD_H

#include "BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h"
#include "btSoftBody.h"
// add btCable include for sending data
#include "BulletCable/btCable.h"
#include <map>

typedef btAlignedObjectArray<btSoftBody*> btSoftBodyArray;

class btSoftBodySolver;

class btSoftRigidDynamicsWorld : public btDiscreteDynamicsWorld
{
	btSoftBodyArray m_softBodies;
	int m_drawFlags;
	bool m_drawNodeTree;
	bool m_drawFaceTree;
	bool m_drawClusterTree;
	btSoftBodyWorldInfo m_sbi;
	///Solver classes that encapsulate multiple soft bodies for solving
	btSoftBodySolver* m_softBodySolver;
	bool m_ownsSolver;

protected:
	virtual void predictUnconstraintMotion(btScalar timeStep);

	virtual void internalSingleStepSimulation(btScalar timeStep);

	void updateCableCollisionObjects();

	void solveSoftBodiesConstraints(btScalar timeStep);

	void serializeSoftBodies(btSerializer* serializer);

	int m_sizeOfNodeForcesStruct;
	int m_hydroCableNodesNumber;

	// Default structs (with values at 0) used to reset the arrays used by the world
	btCable::NodeData m_defaultNodeData;
	btCable::NodePos m_defaultNodePos;
	btSoftBody::NodeForces m_defaultNodeForces;
	btCable::CableData m_defaultCableData;

public:
	btSoftRigidDynamicsWorld(btSoftBodyWorldInfo* worldInfo, btConstraintSolver* constraintSolver, btCollisionConfiguration* collisionConfiguration, btSoftBodySolver* softBodySolver = 0);

	virtual ~btSoftRigidDynamicsWorld();

	virtual void debugDrawWorld();

	void addSoftBody(btSoftBody* body, int collisionFilterGroup = btBroadphaseProxy::DefaultFilter, int collisionFilterMask = btBroadphaseProxy::AllFilter);

	void removeSoftBody(btSoftBody* body);

	///removeCollisionObject will first check if it is a rigid body, if so call removeRigidBody otherwise call btDiscreteDynamicsWorld::removeCollisionObject
	virtual void removeCollisionObject(btCollisionObject* collisionObject);

	int getDrawFlags() const { return (m_drawFlags); }
	void setDrawFlags(int f) { m_drawFlags = f; }

	btSoftBodyWorldInfo& getWorldInfo()
	{
		return m_sbi;
	}
	const btSoftBodyWorldInfo& getWorldInfo() const
	{
		return m_sbi;
	}

	virtual btDynamicsWorldType getWorldType() const
	{
		return BT_SOFT_RIGID_DYNAMICS_WORLD;
	}

	btSoftBodyArray& getSoftBodyArray()
	{
		return m_softBodies;
	}

	const btSoftBodyArray& getSoftBodyArray() const
	{
		return m_softBodies;
	}

	int getTotalNumNodes() {
		int total = 0;
		for (int i = 0; i < m_softBodies.size(); i++)
		{			
			total += m_softBodies.at(i)->m_nodes.size();
		}
		return total;
	}

	virtual void rayTest(const btVector3& rayFromWorld, const btVector3& rayToWorld, RayResultCallback& resultCallback) const;

	/// rayTestSingle performs a raycast call and calls the resultCallback. It is used internally by rayTest.
	/// In a future implementation, we consider moving the ray test as a virtual method in btCollisionShape.
	/// This allows more customization.
	static void rayTestSingle(const btTransform& rayFromTrans, const btTransform& rayToTrans,
							  btCollisionObject* collisionObject,
							  const btCollisionShape* collisionShape,
							  const btTransform& colObjWorldTransform,
							  RayResultCallback& resultCallback);

	virtual void serialize(btSerializer* serializer);

	int getHydroNodesNumber();

	void updateCableForces(btSoftBody::NodeForces* co, int size);

	void prepareSingleStepSimulation();
	void* btSoftRigidDynamicsWorld::getCablesData();
	void* btSoftRigidDynamicsWorld::getNodesPos();
	void* btSoftRigidDynamicsWorld::getNodesData();
	int* getCableIndexesArray();

	btSoftBody::NodeForces* m_nodeForces;
	btCable::CableData* m_cablesData;
	btCable::NodePos* m_nodesPos;
	btCable::NodeData* m_nodesData;

	int* m_cableIndexesArray;

	map<btCollisionObject*, btCollisionObject*> m_cableCollisionObjects;
};

#endif  //BT_SOFT_RIGID_DYNAMICS_WORLD_H
