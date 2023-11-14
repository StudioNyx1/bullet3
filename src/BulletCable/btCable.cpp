#include "btCable.h"
#include <BulletSoftBody/btSoftBodyInternals.h>
#include <BulletSoftBody/btSoftRigidDynamicsWorld.h>
#include <fstream>
#include <BulletCollision/CollisionShapes/btSphereShape.h>
#include <BulletCollision/CollisionShapes/btCapsuleShape.h>
#include <Bullet3Common/b3Logging.h>
#include <vector>
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <list>
#include "../../Extras/Serialize/BulletWorldImporter/btWorldImporter.h"
#include <BulletCollision/NarrowPhaseCollision/btRaycastCallback.h>

#define SLEEPINGTHRESHOLD 0.0001
using namespace std::chrono;


btCable::btCable(btSoftBodyWorldInfo* worldInfo, btCollisionWorld* world, int node_count, const btVector3* x, const btScalar* m) : btSoftBody(worldInfo, node_count, x, m)
{
	m_world = world;

	// Initialize Data
	m_cableData = new CableData();
	m_nodePos = new NodePos[8192];
	m_nodeData = new NodeData[8192];
	
	for (int i = 0; i < this->m_nodes.size(); i++)
	{
		m_nodes[i].m_battach = 0;
		m_nodes[i].index = i;

		if (i != 0)
		{
			appendLink(i - 1, i);
		}
		
		// Set Node pos Struct
		m_nodePos[i].x = m_nodes[i].m_x.getX();
		m_nodePos[i].y = m_nodes[i].m_x.getY();
		m_nodePos[i].z = m_nodes[i].m_x.getZ();
		
		// Set Node Data Struct
		m_nodeData[i].velocity_x = m_nodes[i].m_v.getX();
        m_nodeData[i].velocity_y = m_nodes[i].m_v.getY();
        m_nodeData[i].velocity_z = m_nodes[i].m_v.getZ();
	}

    // Using second node we set mass cable value
    m_cableData->nodeMass = getMass(1);
    // Using getCollisionShape we set the cable radius
	m_cableData->radius = getCollisionShape()->getMargin();
}

#pragma region Constraints


void btCable::solveConstraints()
{
	int i, ni;
	for (i = 0, ni = m_nodes.size(); i < ni; ++i)
	{
		m_nodes[i].m_nbCollidingObject = 0;
		// debug draw
		m_nodes[i].m_splitv = btVector3(0, 0, 0);
	}

	// Prepare links
	for (i = 0, ni = m_links.size(); i < ni; ++i)
	{
		Link& l = m_links[i];
		l.m_c3 = l.m_n[1]->m_q - l.m_n[0]->m_q;
		l.m_c2 = 1 / (l.m_c3.length2() * l.m_c0);
	}
	
	// Prepare anchors
	for (i = 0, ni = this->m_anchors.size(); i < ni; ++i)
	{
		Anchor& a = this->m_anchors[i];
		
		const btVector3 ra = a.m_body->getWorldTransform().getBasis() * a.m_local;
		a.m_c0 = ImpulseMatrix(m_sst.sdt,
							   a.m_node->m_im,
							   a.m_body->getInvMass(),
							   a.m_body->getInvInertiaTensorWorld(),
							   ra);
		a.m_c1 = ra;
		a.m_c2 = m_sst.sdt * a.m_node->m_im;
		a.m_body->activate();
		a.tension = btVector3(0,0,0);
	}
	
	btAlignedObjectArray<BroadPhasePair *> BroadPhaseOutput = btAlignedObjectArray<BroadPhasePair *>();
	btAlignedObjectArray<NodePairNarrowPhase> nodePairContact = btAlignedObjectArray<NodePairNarrowPhase>();
	
	if (useCollision)
	{
		btScalar marginNode = this->m_collisionShape->getMargin();
		btScalar margin;
		ni = m_nodes.size();

		// Get the overlapping pairs
		auto cache = m_world->getBroadphase()->getOverlappingPairCache();
		auto temp = cache->getOverlappingPairArray();
		// Maybe a faster way is possible using a filter
		for (int w = 0; w < temp.size(); w++)
		{
			if (temp[w].m_pProxy0->m_clientObject == this)
			{
				// Add Object to the potential list
				btCollisionObject* rb = (btCollisionObject*)temp[w].m_pProxy1->m_clientObject;

				// Entity is set with collision
				if (rb->getBroadphaseHandle()->m_collisionFilterMask == 0)
				{
					continue;
				}

				if (rb->hasContactResponse() && this->m_collisionDisabledObjects.findLinearSearch(rb) == m_collisionDisabledObjects.size())
				{
					BroadPhasePair* temp = new BroadPhasePair();
					temp->body = rb;
					BroadPhaseOutput.push_back(temp);
				}
					
			}
			if (temp[w].m_pProxy1->m_clientObject == this)
			{
				// Add the node to the potential list of node collision
				btCollisionObject* rb = (btCollisionObject*)temp[w].m_pProxy0->m_clientObject;

				// Entity is set with collision
				if (rb->getBroadphaseHandle()->m_collisionFilterMask == 0)
				{
					continue;
				}

				if (rb->hasContactResponse() && this->m_collisionDisabledObjects.findLinearSearch(rb) == m_collisionDisabledObjects.size())
				{
					BroadPhasePair* temp = new BroadPhasePair();
					temp->body = rb;
					BroadPhaseOutput.push_back(temp);
				}
			}
		}
		int potentialCollisionObjectListSize = BroadPhaseOutput.size();
		
		if (potentialCollisionObjectListSize > 0)
		{
			for (int i = 0; i < ni; i++)
			{
				if (m_nodes[i].m_battach != 0) continue;

				for (int j = 0; j < potentialCollisionObjectListSize; j++)
				{
					btCollisionObject* obj = BroadPhaseOutput[j]->body;
					Node& n = m_nodes[i];
					m_nodes[i].m_xOut = m_nodes[i].m_q;
					margin = marginNode + 0.05;

					if (obj->getCollisionShape()->getShapeType() != SPHERE_SHAPE_PROXYTYPE)
						margin += obj->getCollisionShape()->getMargin();
					
					// Box Definition
					btVector3 minLink = btVector3(0, 0, 0);
					btVector3 maxLink = btVector3(0, 0, 0);

					minLink.setX(btMin(n.m_x.x() - margin , n.m_q.x() - margin));
					minLink.setY(btMin(n.m_x.y() - margin , n.m_q.y() - margin));
					minLink.setZ(btMin(n.m_x.z() - margin , n.m_q.z() - margin));

					maxLink.setX(btMax(n.m_x.x() + margin , n.m_q.x() + margin ));
					maxLink.setY(btMax(n.m_x.y() + margin , n.m_q.y() + margin ));
					maxLink.setZ(btMax(n.m_x.z() + margin , n.m_q.z() + margin ));

					btVector3 mins, maxs;

					if (obj->getCollisionShape()->getShapeType() == COMPOUND_SHAPE_PROXYTYPE)
					{
						btTransform WorldToLocalMatrix = btTransform(obj->getWorldTransform());
						btCompoundShape* temp = (btCompoundShape*)obj->getCollisionShape();
						recursiveBroadPhase(BroadPhaseOutput.at(j), &n, temp, &nodePairContact, minLink, maxLink, WorldToLocalMatrix);
					}
					else
					{
						obj->getCollisionShape()->getAabb(obj->getWorldTransform(), mins, maxs);
						// Intersect box
						if (minLink.x() <= maxs.x() && maxLink.x() >= mins.x() &&
							minLink.y() <= maxs.y() && maxLink.y() >= mins.y() &&
							minLink.z() <= maxs.z() && maxLink.z() >= mins.z())
						{
							auto temp = NodePairNarrowPhase();
							temp.pair = BroadPhaseOutput.at(j);
							temp.node = &n;
							temp.node->m_nbCollidingObject++;
							temp.m_Xout = PositionStartRayCalculation(&n, BroadPhaseOutput.at(j)->body);
							temp.collisionShape = obj->getCollisionShape();
							nodePairContact.push_back(temp);
						}
					}
				}
			}
		}
	}
	
	for (int i = 0; i < m_cfg.piterations; ++i)
	{
		anchorConstraint();
		distanceConstraint(); 
		if (useLRA) LRAConstraint();
		if (useBending && i%2 == 0) bendingConstraintDistance();
		if (useCollision && i%m_substepDelayCollision == 0) solveContact(&nodePairContact);
	}
	
	if (useCollision) solveContact(&nodePairContact);

	// Remove unused manifold 
	clearManifold(BroadPhaseOutput,false);

	for (i = 0; i < nodePairContact.size(); i++)
	{
		NodePairNarrowPhase nodePair = nodePairContact.at(i);
		if (!nodePair.hit)
			continue;
		btPersistentManifold* manifold; 
		// If the manifold exist, use it
		if (nodePairContact.at(i).pair->haveManifoldsRegister)
			manifold = nodePairContact.at(i).pair->manifold;
		// Else create one 
		else
		{
			manifold = m_world->getDispatcher()->getNewManifold( this,nodePairContact[i].pair->body);
			manifolds.push_back(manifold);
			nodePairContact.at(i).pair->manifold = manifold;
			nodePairContact.at(i).pair->haveManifoldsRegister = true;
		}

		// Obj 0 = Cable
		// Obj 1 = RigidBody

		const btVector3& pointB = nodePairContact.at(i).lastPosition;
		const btVector3& normal = nodePairContact.at(i).normal;
		const btScalar distance = nodePairContact.at(i).distance;

		btManifoldPoint newPoint = btManifoldPoint(btVector3(0, 0, 0), btVector3(0, 0, 0),normal,distance);
		newPoint.m_positionWorldOnA = pointB + normal * distance;
		newPoint.m_positionWorldOnB = pointB;

		newPoint.m_appliedImpulse = nodePairContact.at(i).impulse.length();
		manifold->addManifoldPoint(newPoint, true);
		btManifoldPoint temp = manifold->getContactPoint(manifold->getNumContacts() - 1);
	}
	// Clear manifolds without contact point
	clearManifold(BroadPhaseOutput, true);

	BroadPhaseOutput.clear();
	nodePairContact.clear();

	const btScalar vc = m_sst.isdt * (1 - m_cfg.kDP);

	lenght = 0;

	for (i = 0, ni = m_nodes.size(); i < ni; ++i)
	{
		Node& n = m_nodes[i];
		n.m_v = (n.m_x - n.m_q) * vc;
		n.m_f = btVector3(0, 0, 0);

		// Update NodePos
		m_nodePos[i].x = m_nodes[i].m_x.getX();
		m_nodePos[i].y = m_nodes[i].m_x.getY();
		m_nodePos[i].z = m_nodes[i].m_x.getZ();

		// Update NodeData
		m_nodeData[i].velocity_x = m_nodes[i].m_v.getX();
		m_nodeData[i].velocity_y = m_nodes[i].m_v.getY();
		m_nodeData[i].velocity_z = m_nodes[i].m_v.getZ();

		// Calculate Volume
		float sizeElement = 0;

		if (i == 0)
		{
			sizeElement = (m_nodes[i].m_x - m_nodes[i+1].m_x).length();
			// Calculate cable Lenght
			lenght += sizeElement;
		}
		else if (i == m_nodes.size() - 1)
		{
			sizeElement = (m_nodes[i].m_x - m_nodes[i-1].m_x).length(); 
		}
		else
		{
			sizeElement = (m_nodes[i].m_x - m_nodes[i-1].m_x).length(); 
			// Calculate cable Lenght
			lenght += sizeElement;

			sizeElement += (m_nodes[i].m_x - m_nodes[i+1].m_x).length(); 
		}

		// Using a cylinder volume calculation
		m_nodeData[i].volume = SIMD_PI * m_cableData->radius * m_cableData->radius * sizeElement;
		// Divide it by two cause every node is used twice in the calculation
		m_nodeData[i].volume = m_nodeData[i].volume / 2;
	}

	// Update mass using second node if it was updated in unity side
	m_cableData->nodeMass = getMass(1);

}


void btCable::clearManifold(btAlignedObjectArray<BroadPhasePair *> objs,bool init)
{
	if (manifolds.size() == 0)
	{
		return;
	}
	else
	{
		int count = manifolds.size();
		// Remove body that aren t in the broadphase anymore
		if (!init)
		{
			bool isCreated = false;
			for (int i = 0; i < count; i++)
			{
				for (int j = 0; j < objs.size(); j++)
				{
					// Body 0 is always the cable
					if (manifolds.at(i)->getBody1() == objs.at(j)->body)
					{
						objs.at(j)->haveManifoldsRegister = true;
						objs.at(j)->manifold = manifolds.at(i);
						isCreated = true;
						break;
					}
				}
				// remove unused manifolds
				if (!isCreated)
				{
					manifolds.removeAtIndex(i);
					count--;
				}
				else
				{
					manifolds.at(i)->clearManifold();
				}
			}
		}
		// Remouve the body that have no contact point 
		else
		{
			int total = 0;
			for (int i = 0; i < count; i++)
			{
				if (manifolds.at(i)->getNumContacts() == 0)
				{
					manifolds.removeAtIndex(i);
					count--;
					i--;
				}
				else
				{
					total += manifolds.at(i)->getNumContacts();
				}
			}
		}
		
	}
	
}


void btCable::recursiveBroadPhase(BroadPhasePair* obj, Node* n, btCompoundShape* shape, btAlignedObjectArray<NodePairNarrowPhase>* nodePairContact, btVector3 minLink, btVector3 maxLink, btTransform transformLocal)
{
	int subShapes = shape->getNumChildShapes();
	for (int i = 0; i < subShapes; i++) {
		btCollisionShape* temp = shape->getChildShape(i);
		if (temp->getShapeType() == COMPOUND_SHAPE_PROXYTYPE)
		{
			btCompoundShape* compound = (btCompoundShape*)temp;
			btTransform newTransform = btTransform();
			newTransform.mult(shape->getChildTransform(i), transformLocal);
			recursiveBroadPhase(obj,n,compound, nodePairContact,minLink,maxLink,newTransform);
		}
		else
		{
			btVector3 mins, maxs;
			auto transform = shape->getChildTransform(i);
			transform.mult(transformLocal, transform);
			temp->getAabb(transform, mins, maxs);
			// Intersect box
			if (minLink.x() <= maxs.x() && maxLink.x() >= mins.x() &&
				minLink.y() <= maxs.y() && maxLink.y() >= mins.y() &&
				minLink.z() <= maxs.z() && maxLink.z() >= mins.z())
			{
				
				auto nodePair = NodePairNarrowPhase();
				nodePair.worldToLocal = transform;
				nodePair.pair = obj;
				nodePair.collisionShape = temp;
				nodePair.node = n;
				nodePair.node->m_nbCollidingObject++;
				nodePair.m_Xout = PositionStartRayCalculation(n, obj->body);
				nodePairContact->push_back(nodePair);
				// debug
			}
		}
	}
}


bool btCable::checkCollide(int indexNode)
{
	
	btCollisionObjectArray array = m_world->getCollisionObjectArray();
	Node n = this->m_nodes[indexNode];
	btScalar margin = this->getCollisionShape()->getMargin();
	for (int i = 0; i < array.size(); i++)
	{
		btVector3 minLink = btVector3(0, 0, 0);
		btVector3 maxLink = btVector3(0, 0, 0);

		minLink.setX(n.m_x.x() - margin);
		minLink.setY(n.m_x.y() - margin);
		minLink.setZ(n.m_x.z() - margin);
		
		maxLink.setX(n.m_x.x() + margin);
		maxLink.setY(n.m_x.y() + margin);
		maxLink.setZ(n.m_x.z() + margin);

		btCollisionObject* colObj = array[i];
		int type = colObj->getInternalType();

		// Check if the object is not a softbody
		if (type == 8)
			continue;

		// Check if the object is a rigidbody which havs contacts
		if ((!colObj->hasContactResponse()))
			continue;

		// Cancel detection with ignored body
		if (m_collisionDisabledObjects.findLinearSearch(colObj) != m_collisionDisabledObjects.size())
			continue;

		btVector3 mins, maxs;
		colObj->getCollisionShape()->getAabb(colObj->getWorldTransform(), mins, maxs);

		// Intersect box
		if (minLink.x() <= maxs.x() && maxLink.x() >= mins.x() &&
			minLink.y() <= maxs.y() && maxLink.y() >= mins.y() &&
			minLink.z() <= maxs.z() && maxLink.z() >= mins.z())
		{
			return true;
		}
		
	}
	return false;
}

// todo compute Velocity to move mq out of the box
btVector3 btCable::PositionStartRayCalculation(Node *n, btCollisionObject * obj)
{
	btScalar dt = this->m_sst.sdt;
	btVector3 velocity;
	if (obj->getInternalType() == CO_RIGID_BODY)
	{
		btRigidBody* rb = (btRigidBody*)obj;
		velocity = rb->getVelocityInLocalPoint(n->m_x - rb->getWorldTransform().getOrigin());
	}
	else
		velocity = obj->getInterpolationLinearVelocity();
	if (btFuzzyZero(velocity.length())) {
		return n->m_q ;
	}
	return (n->m_q + velocity*dt );
	
}

// Resolve iteratively all the contact constraint
// Do nbSubStep times the resolution to valid a good collision
void btCable::solveContact(btAlignedObjectArray<NodePairNarrowPhase>* nodePairContact) {
	int nbSubStep = m_subIterationCollision;
	int size = nodePairContact->size();

	btVector3 positionStartRay;
	btVector3 positionEndRay;

	btTransform m_rayFromTrans;
	btTransform m_rayToTrans;
	btScalar marginNode = this->getCollisionShape()->getMargin();
	btScalar marginBody;
	
	btScalar margin;
	Node* n;
	Node* nBefore;
	Node* nAfter;

	btScalar rlLinkBefore;
	btScalar rlLinkAfter;

	btCollisionObject* obj;
	btCollisionShape* shape;

	btTransform t;
	int indexNode;

	for (int j=0;j<nbSubStep;j++)
	{
		for (int i = 0; i < size; i++)
		{
			n = nodePairContact->at(i).node;
			n->m_splitv = btVector3(1, 0, 0);

			// If the node only collide 1 body we don t have to update nbSubStep times
			if (n->m_nbCollidingObject == 1 && j>0)
				continue;

			obj = nodePairContact->at(i).pair->body;
			shape = nodePairContact->at(i).collisionShape;
			indexNode = n->index;
			bool isSphereShape = shape->getShapeType() == SPHERE_SHAPE_PROXYTYPE;
			
			// SphereShape margin is sphereShape Radius, not a safe margin
			if (isSphereShape)
			{
				marginBody = 0.005;
				margin = marginNode + marginBody;
			}
			else
			{
				marginBody = shape->getMargin();
				margin = marginNode + shape->getMargin(); 
			}

			positionStartRay = nodePairContact->at(i).m_Xout;
			positionEndRay = n->m_x;

			btVector3 dir = btVector3(0, 0, 0);

			btScalar len = (positionEndRay - positionStartRay).length();

			if (btFuzzyZero(len))
				continue;

			if (len <= m_collisionSleepingThreshold)
			{
				continue;
			}
				
			dir = (positionEndRay - positionStartRay) / len;
			
			
			btVector3 start = positionStartRay - dir * m_safeDirectionThreshold;

			btVector3 end = positionEndRay;  

			m_rayFromTrans.setIdentity();
			m_rayFromTrans.setOrigin(start);

			m_rayToTrans.setIdentity();
			m_rayToTrans.setOrigin(end);

			btCollisionWorld::ClosestRayResultCallback m_resultCallback(start, end);

			// If the body is not a compound it s the simplest case
			// We use the object collision shape, no need to get the collision shape of the compound component  
			if (!obj->getCollisionShape()->isCompound())
			{
				
				m_world->rayTestSingleWithMargin(m_rayFromTrans, m_rayToTrans,
												 obj,
												 shape,
												 obj->getWorldTransform(),
												 m_resultCallback, marginNode);
			}
			// case of a coumpound shape
			else
			{
				t = nodePairContact->at(i).worldToLocal;
				m_world->rayTestSingleWithMargin(m_rayFromTrans, m_rayToTrans,
												 obj,
												 shape,
												 t,
												 m_resultCallback, marginNode);		
			}
			if (m_resultCallback.hasHit())
			{
				btVector3 contactPoint = m_resultCallback.m_hitPointWorld;

				btVector3 outMouvementPos = m_resultCallback.m_hitNormalWorld * m_correctionNormal;
				btVector3 newPosOut = contactPoint + outMouvementPos;
				btVector3 impulse = btVector3(0,0,0);

				btScalar offset = (n->m_x - contactPoint).length();
				btScalar distPenetration = (m_resultCallback.m_hitNormalWorld * offset).length();

				if (obj->getInternalType() == CO_RIGID_BODY)
				{
					btRigidBody* rb = (btRigidBody*)btRigidBody::upcast(obj);
					if (!obj->isStaticOrKinematicObject())
						impulse = moveBodyCollision(rb, margin, n, m_resultCallback.m_hitNormalWorld, contactPoint);
				}
				
				// Replace node
				n->m_x = contactPoint + outMouvementPos;

				btVector3 correctionBefore = btVector3(0, 0, 0);
				btVector3 correctionAfter = btVector3(0, 0, 0);

				// Link mouvement to avoid node getting stuck
				if (indexNode > 0 && !isSphereShape)
				{
					nBefore = &m_nodes[indexNode - 1];
					rlLinkBefore = m_links[indexNode - 1].m_rl;
					btVector3 linkBefore = nBefore->m_x - n->m_x;
					btScalar len = linkBefore.length();
					if (len > rlLinkBefore)
					{
						btVector3 dirBefore = linkBefore / len;
						btVector3 extraSizeLinkBefore = linkBefore - dirBefore * rlLinkBefore;
						btVector3 tangentDirBefore = dirBefore.cross(m_resultCallback.m_hitNormalWorld);
						tangentDirBefore = m_resultCallback.m_hitNormalWorld.cross(tangentDirBefore).normalized();
						correctionBefore = extraSizeLinkBefore.dot(-m_resultCallback.m_hitNormalWorld) * tangentDirBefore;
					}
				}

				// Link mouvement to avoid node getting stuck
				if (indexNode < this->m_nodes.size() - 1 && !isSphereShape)
				{
					nAfter = &m_nodes[indexNode + 1];
					rlLinkAfter = m_links[indexNode].m_rl;
					btVector3 linkAfter = nAfter->m_x - n->m_x;
					btScalar len = linkAfter.length();
					if (len > rlLinkAfter)
					{
						btVector3 dirAfter = linkAfter / len;
						btVector3 extraSizeLinkAfter = linkAfter - dirAfter * rlLinkAfter;
						btVector3 tangentDirAfter = dirAfter.cross(m_resultCallback.m_hitNormalWorld);
						tangentDirAfter = m_resultCallback.m_hitNormalWorld.cross(tangentDirAfter).normalized();
						correctionAfter = extraSizeLinkAfter.dot(-m_resultCallback.m_hitNormalWorld) * tangentDirAfter;
					}
				}

				// Add correction
				n->m_x = n->m_x + correctionBefore + correctionAfter;

				nodePairContact->at(i).impulse = impulse;
				nodePairContact->at(i).m_Xout = n->m_x;
				nodePairContact->at(i).lastPosition = contactPoint - margin * m_resultCallback.m_hitNormalWorld;
				nodePairContact->at(i).hit = true;
				nodePairContact->at(i).normal = m_resultCallback.m_hitNormalWorld;
				nodePairContact->at(i).distance = distPenetration;
			}
		}
	}
}

btVector3 btCable::moveBodyCollision(btRigidBody* obj, btScalar margin, Node* n, btVector3 normale, btVector3 hitPosition)
{	
	// a = node
	// b = body
	btScalar ima = n->m_im;
	btScalar imb = obj->getInvMass();
	btScalar dt = this->m_sst.sdt;
	if (imb == 0) return btVector3(0,0,0);

	btScalar totalMass = ima + imb;
	btTransform wtr = obj->getWorldTransform();
	btMatrix3x3 iwi = obj->getInvInertiaTensorWorld();

	// Contact hardness
	btScalar coef = obj->isStaticOrKinematicObject() ? this->m_cfg.kKHR : this->m_cfg.kCHR;

	// bodyToNodeVector
	btVector3 ra = n->m_x - wtr.getOrigin();
	btVector3 vBody = obj->getVelocityInLocalPoint(ra)*dt;
	btVector3 vNode = n->m_x - n->m_q;
	btVector3 vRelative = vNode - vBody;

	btScalar dn = btDot(vRelative, normale);
	if (dn <= SIMD_EPSILON)
	{
		btScalar offset = btMin((n->m_x - hitPosition).length(), margin);
		btMatrix3x3 impulseMat = ImpulseMatrix(dt,ima,imb,iwi,ra);

		auto distPenetration = (-normale * (offset * coef));

		btVector3 impulse = impulseMat *  distPenetration;
		obj->applyImpulse(impulse, ra);
		return impulse;
	}
	return btVector3(0, 0, 0);
}


void btCable::LRAConstraint()
{
	btScalar distance = 0;

	Node& a = m_nodes[m_nodes.size() - 1];
	for (int i = 0; i < m_anchors.size(); ++i)
		if (a.index == m_anchors[i].m_node->index)
			a.m_x = m_anchors[i].m_c1 + m_anchors[i].m_body->getCenterOfMassPosition(); 

	for (int i = m_links.size() - 1; i >= 0; --i)
	{
		Link& l = m_links[i];
		Node* b = l.m_n[0];

		distance += l.m_rl;
		if (a.m_x.distance(b->m_x) > distance)
			b->m_x = a.m_x + (b->m_x - a.m_x).normalized() * distance;
	}
}

void btCable::LRAConstraint(int level, int idxAnchor)
{
	int stablePointIndex = m_anchors[idxAnchor].m_node->index;
	m_anchors[idxAnchor].m_node->m_x = m_anchors[idxAnchor].m_body->getCenterOfMassPosition() + m_anchors[idxAnchor].m_c1;

	// Apply LRA left Side
	for (int j = stablePointIndex - 1; j >= 0; j--)
	{
		Node& n = m_nodes[j];
		btScalar distMax = m_links[stablePointIndex - 1].m_rl;
		int indexToCheck = j - 1;
		if (indexToCheck >= 0)
		{
			Node& left = m_nodes[indexToCheck];
			btVector3 AB = left.m_x - n.m_x;
			if (AB.length() > distMax)
			{
				btVector3 ABNormalized = AB.normalized();
				btVector3 PosLocal = ABNormalized * distMax;
				btVector3 newpos = n.m_x + PosLocal;
				left.m_x = newpos;
			}
		}
	}

	// Apply LRA right Side
	for (int j = stablePointIndex; j < m_nodes.size() - 1; j++)
	{
		Node& n = m_nodes[j];
		btScalar distMax = m_links[stablePointIndex].m_rl;
		int indexToCheck = j + 1;
		if (indexToCheck < m_nodes.size())
		{
			Node& right = m_nodes[indexToCheck];
			btVector3 AB = right.m_x - n.m_x;
			if (AB.length() > distMax)
			{
				btVector3 ABNormalized = AB.normalized();
				btVector3 PosLocal = ABNormalized * distMax;
				btVector3 newpos = n.m_x + PosLocal;
				right.m_x = newpos;
			}
		}
	}
}

btVector3 ConstrainDistance(btVector3 point, btVector3 anchor, float distance)
{
	return ((point - anchor).normalize() * distance) + anchor;
}

void btCable::FABRIKChain()
{
	for (int i = 0; i < m_anchors.size(); ++i)
	{
		Anchor& a = m_anchors[i];
		const btVector3 ra = a.m_body->getWorldTransform().getBasis() * a.m_local;

		a.m_node->m_x = m_anchors[i].m_body->getCenterOfMassPosition() + a.m_c1;
		int idx = a.m_node->index;

		for (int i = idx; i > 0; --i)
		{
			//Pull the current segment to the previous one
			if (m_nodes[i - 1].m_x.distance(m_nodes[i].m_x) > m_links[i - 1].m_rl)
				m_nodes[i - 1].m_x = ConstrainDistance(m_nodes[i - 1].m_x, m_nodes[i].m_x, m_links[i - 1].m_rl);
		}

		for (int i = idx + 1; i < m_nodes.size(); i++)
		{
			//Pull the current segment to the previous one
			if (m_nodes[i].m_x.distance(m_nodes[i - 1].m_x) > m_links[i - 1].m_rl)
				m_nodes[i].m_x = ConstrainDistance(m_nodes[i].m_x, m_nodes[i - 1].m_x, m_links[i - 1].m_rl);
		}
	}
}

void btCable::bendingConstraintDistance()
{
	int size = m_nodes.size();
	float stiffness = this->bendingStiffness;
	float iterationFactor = stiffness * stiffness;
	btScalar angleMax = this->maxAngle;


	for (int i = 1; i < this->m_links.size(); ++i)
	{
		Node* before = m_links[i - 1].m_n[0];  // Node before;
		Node* current = m_links[i].m_n[0];     // Current Node
		Node* after = m_links[i].m_n[1];       // Node After
		
		btVector3 delta1 = current->m_x - before->m_x;
		btVector3 delta2 = after->m_x - current->m_x;

		if (btFuzzyZero(delta1.length()) || btFuzzyZero(delta2.length())) continue;

		btScalar dot = delta1.normalized().dot(delta2.normalized());

		if (dot < -1.0f) dot = -1.0f;
		if (dot > 1.0f) dot = 1.0f;
		btScalar phi = acos(dot);
		if (phi > angleMax || angleMax == 0)
			stiffness = stiffness;
		else
			stiffness = phi * stiffness / angleMax;

		// DHat
		{
			btVector3 r = after->m_x - before->m_x;
			btScalar minLength = m_links[i].m_rl / 2;

			btScalar rr = r.length2();
			btScalar d2 = btDot(delta2, r);
			btScalar d1 = btDot(delta1, r);
			btClamp(d2, (btScalar)0.0, rr);
			btClamp(d1, (btScalar)0.0, rr);
			btScalar alpha1 = d2 / rr;
			btScalar alpha2 = d1 / rr;

			btVector3 d = alpha1 * before->m_x + alpha2 * after->m_x - current->m_x;
			btScalar dLen = d.length();

			if (btFuzzyZero(dLen))
			{
				continue;
			}

			btVector3 dNorm = d/dLen;
			btVector3 J1 = alpha1 * dNorm;
			btVector3 J2 = -dNorm;
			btVector3 J3 = alpha2 * dNorm;
			btScalar sum = before->m_im * alpha1 * alpha1 + current->m_im + after->m_im * alpha2 * alpha2;
			if (sum <= DBL_EPSILON)
			{
				continue;
			}
			//btScalar C = dLen;
			btScalar mass = 1.0 / sum;

			btScalar impulse = -stiffness * mass * dLen * iterationFactor;

			before->m_x += (before->m_im * impulse) * J1;
			current->m_x += (current->m_im * impulse) * J2;
			after->m_x += (after->m_im * impulse) * J3;
		}
	}
}

void btCable::bendingConstraintAngle()
{
	int size = m_nodes.size();

	for (int i = 1; i < this->m_links.size(); ++i)
	{
		Node* before = m_links[i - 1].m_n[0];  // Node before;
		Node* current = m_links[i].m_n[0];     // Current Node
		Node* after = m_links[i].m_n[1];       // Node After

		btVector3 p0 = before->m_x;
		btVector3 p1 = after->m_x;
		btVector3 p2 = current->m_x;

		btScalar phiZero = 0;

		float stiffness = 1;

		btVector3 axeTan = (p0 - p2).cross(p1 - p2);
		if (axeTan.length() < DBL_EPSILON)
			continue;
		axeTan = axeTan.normalized();

		btVector3 p3 = p2 + axeTan * min(m_links[i - 1].m_rl, m_links[i].m_rl) * 0.5;

		btVector3 e = p3 - p2;
		btScalar elen = e.length();
		btScalar invElen = 1.0 / elen;

		btVector3 n1 = (p2 - p0).cross(p3 - p0);
		btVector3 n2 = (p3 - p1).cross(p2 - p1);

		n1 /= n1.length2();
		n2 /= n2.length2();

		btVector3 d0 = elen * n1;
		btVector3 d1 = elen * n2;
		btVector3 d2 = (p0 - p3).dot(e) * invElen * n1 + (p1 - p3).dot(e) * invElen * n2;
		btVector3 d3 = (p2 - p0).dot(e) * invElen * n1 + (p2 - p1).dot(e) * invElen * n2;

		n1 = n1.normalized();
		n2 = n2.normalized();

		btScalar dot = n1.dot(n2);

		if (dot < -1.0f) dot = -1.0f;
		if (dot > 1.0f) dot = 1.0f;
		btScalar phi = acos(dot);

		btScalar invMass = current->m_im;

		btScalar lambda = invMass * d0.length2() +
						  invMass * d1.length2() +
						  invMass * d2.length2() +
						  invMass * d3.length2();

		lambda = (phi - phiZero) / lambda * stiffness;

		if (abs(lambda) <= FLT_EPSILON)
			continue;

		if (n1.cross(n2).dot(e) > 0.0f)
			lambda = -lambda;

		before->m_x += -invMass * lambda * d0;
		after->m_x += -invMass * lambda * d1;
		current->m_x += -invMass * lambda * d2;
	}
}

void btCable::distanceConstraint()
{
	BT_PROFILE("PSolve_Links");
	for (int i = 0, ni = m_links.size(); i < ni; ++i)
	{
		Link& l = m_links[i];
		Node& a = *l.m_n[0];
		Node& b = *l.m_n[1];
		btVector3 AB = b.m_x - a.m_x;
		btVector3 ABNormalized = AB.normalized();
		if (ABNormalized.fuzzyZero()) continue;
		btScalar normAB = AB.length();
		btScalar k = m_materials[0]->m_kLST;
		btScalar sumInvMass = a.m_im + b.m_im;
		if (sumInvMass >= 0)
		{
			a.m_x += a.m_im / sumInvMass * (normAB - l.m_rl) * ABNormalized;
			b.m_x -= b.m_im / sumInvMass * (normAB - l.m_rl) * ABNormalized;
		}
	}
}

void btCable::predictMotion(btScalar dt)
{
	cableState = Valid;
	int i, ni;

	/* Update                */
	if (m_bUpdateRtCst)
	{
		m_bUpdateRtCst = false;
		updateLinkConstants();
	}

	/* Prepare                */
	m_sst.sdt = dt * m_cfg.timescale;
	m_sst.isdt = 1 / m_sst.sdt;
	m_sst.velmrg = m_sst.sdt * 3;
	m_sst.radmrg = getCollisionShape()->getMargin();
	m_sst.updmrg = m_sst.radmrg * (btScalar)0.25;

	// Forces
	if (useGravity) addVelocity(m_worldInfo->m_gravity * m_sst.sdt);

	// SoftRigidBody
	NodeForces* nodeForces = static_cast<btSoftRigidDynamicsWorld*>(m_world)->m_nodeForces;
	btVector3 nodeForceToApply = btVector3();

	for (i = 0, ni = m_nodes.size(); i < ni; ++i)
	{
		Node& n = m_nodes[i];
		n.m_q = n.m_x;

		if (isActive() && useHydroAero)
		{  

			// Apply Hydro and Aero forces
			NodeForces currentNodeForces = nodeForces[m_cableData->startIndex + i];

			// We check if currentNodeForces Are Correct, if it's not then we dont apply these forces.
			if (std::isinf(currentNodeForces.x) || std::isnan(currentNodeForces.x) || std::isinf(currentNodeForces.y) || std::isnan(currentNodeForces.y) || std::isinf(currentNodeForces.z) || std::isnan(currentNodeForces.z))
			{
				cableState = InternalForcesError;
			}
			else
			{
				n.m_f.setValue(n.m_f.getX() + currentNodeForces.x, n.m_f.getY() + currentNodeForces.y, n.m_f.getZ() + currentNodeForces.z);
			}
		}

		btVector3 deltaV = n.m_f * n.m_im * m_sst.sdt;
		n.m_v += deltaV;
		n.m_x += n.m_v * m_sst.sdt;
		n.m_f = btVector3(0, 0, 0);
	}
	/* Bounds                */
	updateBounds();

	/* Clear contacts        */
	m_rcontacts.resize(0);
	m_scontacts.resize(0);
}

void btCable::anchorConstraint()
{
	BT_PROFILE("PSolve_Anchors");
	const btScalar kAHR = m_cfg.kAHR * 1;
	const btScalar dt = m_sst.sdt;
	
	for (int i = 0, ni = this->m_anchors.size(); i < ni; ++i)
	{
		Anchor& a =this->m_anchors[i];
		const btTransform& t = a.m_body->getWorldTransform();
		Node& n = *a.m_node;
		const btVector3 wa = t * a.m_local;
		const btVector3 va = a.m_body->getVelocityInLocalPoint(a.m_c1) * dt;
		const btVector3 vb = n.m_x - n.m_q;
		const btVector3 vr = (va - vb) + (wa - n.m_x) * kAHR;
		const btVector3 impulse = a.m_c0 * vr * a.m_influence;
		// n.m_x += impulse * a.m_c2;
		n.m_x = a.m_body->getCenterOfMassPosition() + a.m_c1;
		a.m_body->applyImpulse(-impulse, a.m_c1);

		//impulses[i] += impulse / dt;
		a.tension += impulse / dt;
	}
}

#pragma endregion

#pragma region Getter/Setter

btScalar btCable::getRestLength()
{
	btScalar length = 0;
	for (int i = 0; i < m_links.size(); ++i)
		length += m_links[i].m_rl;
	return length;
}

btScalar btCable::getLength()
{
	return lenght;
}

btVector3 btCable::getTensionAt(int index)
{
	int size = m_anchors.size();
	if (index < size && index >= 0)
		return m_anchors[index].tension;
	else
		return btVector3(0, 0, 0);
}

void btCable::setBendingMaxAngle(btScalar angle)
{
	this->maxAngle = angle;
}

btScalar btCable::getBendingMaxAngle()
{
	return maxAngle;
}

void btCable::setBendingStiffness(btScalar stiffness)
{
	this->bendingStiffness = stiffness;
}

btScalar btCable::getBendingStiffness()
{
	return this->bendingStiffness;
}

void btCable::setUseLRA(bool active)
{
	useLRA = active;
}

bool btCable::getUseLRA()
{
	return useLRA;
}

void btCable::setUseBending(bool active)
{
	useBending = active;
}

bool btCable::getUseBending()
{
	return useBending;
}

void btCable::setUseGravity(bool active)
{
	useGravity = active;
}

bool btCable::getUseGravity()
{
	return useGravity;
}

void btCable::setUseCollision(bool active)
{
	useCollision = active;
}

bool btCable::getUseCollision()
{
	return useCollision;
}


void btCable::setUseHydroAero(bool active) 
{
	useHydroAero = active;
}

bool btCable::getUseHydroAero()
{
	return useHydroAero;
}
    
void btCable::setHorizonDrop(float value)
{
	m_cableData->horizonDrop = value;
}

bool btCable::updateCableData(btCable::CableData &cableData)
{
	memcpy(m_cableData, &cableData, CableDataSize);

	return true;
}

void* btCable::getCableNodesPos() 
{
	return m_nodePos;
}

int btCable::getCableState()
{
	return (int)cableState;
}

void btCable::appendNode(const btVector3& x, btScalar m)
{
	if (m_nodes.capacity() == m_nodes.size())
	{
		pointersToIndices();
		m_nodes.reserve(m_nodes.size() * 2 + 1);
		indicesToPointers();
	}
	const btScalar margin = getCollisionShape()->getMargin();
	m_nodes.push_back(Node());
	Node& n = m_nodes[m_nodes.size() - 1];
	ZeroInitialize(n);
	n.m_x = x;
	n.m_q = n.m_x;
	n.m_im = m > 0 ? 1 / m : 0;
	n.m_material = m_materials[0];
	
	for (int i = m_nodes.size()-3; i < m_nodes.size(); i++)
	{
		// Update NodePos
		m_nodePos[i].x = m_nodes[i].m_x.getX();
		m_nodePos[i].y = m_nodes[i].m_x.getY();
		m_nodePos[i].z = m_nodes[i].m_x.getZ();
	}

	n.index = m_nodes.size() - 1;	
}

void btCable::setTotalMass(btScalar mass, bool fromfaces)
{
	btScalar massNode = mass / m_nodes.size();
	for (int i = 0; i < m_nodes.size(); ++i)
	{
		m_nodes[i].m_im = 1.0 / massNode;
	}
}

void btCable::setCollisionParameters(int substepDelayCollision, int subIterationCollision, btScalar correctionNormal, btScalar safeDirectionThreshold, btScalar collisionSleepingThreshold)
{
	m_substepDelayCollision = substepDelayCollision;
	m_subIterationCollision = subIterationCollision;
	m_correctionNormal = correctionNormal;
	m_safeDirectionThreshold = safeDirectionThreshold;
	m_collisionSleepingThreshold = collisionSleepingThreshold;
}

#pragma endregion
