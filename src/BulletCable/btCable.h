#ifndef _BT_CABLE_H
#define _BT_CABLE_H
#include "LinearMath/btAlignedObjectArray.h"
#include "LinearMath/btTransform.h"
#include "LinearMath/btIDebugDraw.h"
#include "LinearMath/btVector3.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"

#include "BulletCollision/CollisionShapes/btConcaveShape.h"
#include "BulletCollision/CollisionDispatch/btCollisionCreateFunc.h"
#include "BulletCollision/BroadphaseCollision/btDbvt.h"
#include "BulletDynamics/Featherstone/btMultiBodyLinkCollider.h"
#include "BulletDynamics/Featherstone/btMultiBodyConstraint.h"
#include "BulletSoftBody/btSoftBody.h"
#include "BulletCollision/CollisionDispatch/btCollisionWorld.h"
#include "BulletCollision/BroadphaseCollision/btCollisionAlgorithm.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.h"


using namespace std;

///The btCable is a class that inherits from btSoftBody.
///Its purpose is to be able to create a cable/rope with our own method parameters that Bullet does not implement.
class btCable : public btSoftBody
{
private:
	btVector3* impulses;
	// btVector3* springsForces;

	// Differents forces for the cable:
	void addSpringForces(); // other way to add the spring & damping forces for the links
	void addMoorDyn();
	void addForces();
	btCollisionShape* collisionShapeNode;
	btCollisionWorld* world;
	btPersistentManifold* tempManiforld = new btPersistentManifold();
	btConvexPenetrationDepthSolver* m_pdSolver;

public:
	void predictMotion(btScalar dt) override;
	void solveConstraints() override;
	static btSoftBody::psolver_t getSolver(ePSolver::_ solver);
	static void PSolve_Anchors(btSoftBody* psb, btScalar kst, btScalar ti);
	static void PSolve_Links(btSoftBody* psb, btScalar kst, btScalar ti);
	static void PSolve_RContacts(btSoftBody* psb, btScalar kst, btScalar ti);
	btCable(btSoftBodyWorldInfo* worldInfo, btCollisionWorld* world, int node_count, const btVector3* x, const btScalar* m);

	void removeLink(int index);
	void removeNode(int index);
	void removeAnchor(int index);

	void setRestLenghtLink(int index, btScalar distance);
	btScalar getRestLenghtLink(int index);

	void swapNodes(int index0, int index1);
	void swapAnchors(int index0, int index1);

	btScalar getLength();

	btVector3* getImpulses();

	bool checkContact(const btCollisionObjectWrapper* colObjWrap, const btVector3& x, btScalar margin, btSoftBody::sCti& cti) const override;

	btCollisionShape* getCollisionShapeNode() const;
	void setCollisionShapeNode(btCollisionShape* nodeShape);

	void setWorldRef(btCollisionWorld* colWorld);
	bool checkCollide(btCollisionObject* colObjA, btCollisionObject* colObjB, btCollisionWorld::ContactResultCallback& resultCallback) const;
};

#endif  //_BT_CABLE_H
