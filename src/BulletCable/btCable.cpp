//#include "Bullet3Common/b3Logging.h"

#include <omp.h>
#include <vector>
#include <thread>
#include <iostream>
#include <LinearMath/btQuickprof.h>
#include <BulletSoftBody/btSoftBodyInternals.h>
#include <BulletCollision/GImpact/btGImpactShape.h>
#include <BulletSoftBody/btSoftRigidDynamicsWorld.h>
#include <BulletCollision/CollisionShapes/btSphereShape.h>

btCable::btCable(btSoftBodyWorldInfo* worldInfo, btCollisionWorld* world, int node_count, int section_count, const btVector3* x, const btScalar* m) : btSoftBody(worldInfo, node_count, x, m), _nodeContactSphere(m_collisionMargin)
{
	m_world = world;
	m_solverSubStep = worldInfo->numIteration;
	m_cpt = 0;

	// Initialize Data
	m_cableData = new CableData();
	m_nodePos = new NodePos[worldInfo->maxNodeNumberPerCable]();
	m_nodeData = new NodeData[worldInfo->maxNodeNumberPerCable]();

	_candidates = btAlignedObjectArray<BroadPhasePair>();
	_nodePairContact = btAlignedObjectArray<NodePairNarrowPhase>();

	for (int i = 0; i < this->m_nodes.size(); i++)
	{
		Node& node = m_nodes[i];
		node.m_battach = 0;
		node.index = i;
		node.m_xn = x[i];

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

	// Using getCollisionShape we set the cable radius
	m_cableData->radius = getCollisionShape()->getMargin();

	if (section_count > 0)
	{
		m_sectionCount = section_count;
		m_section = new SectionInfo[section_count]();
	}
	else
	{
		m_defaultRestLength = m_links.at(m_links.size() - 1 ).m_rl;
	}

	m_gravity = worldInfo->m_gravity;

	// Cached objects used for contact solving
	_nodeContactSphere.setUnscaledRadius(m_collisionMargin);
	_nodeContactObject = btCollisionObject();
	_nodeContactTransform = btTransform::getIdentity();
	_nodeContactObject.setCollisionShape(&_nodeContactSphere);
	setDistanceMode((int) DistanceMode::Bullet);
	m_collisionMode = CollisionMode::Base;
}

void btCable::setMassRatioActivationThreshold(btScalar offset)
{
	m_minAccumulator = offset;
}

void btCable::updateLength(btScalar dt)
{
	if (WantedSpeed > 0)
	{
		if (WantedDistance > 0)
		{
			if (WantedDistance > getRestLength())
			{
				Grows(dt);
			}
		}
		else if (WantedDistance == 0)
		{
			Grows(dt);
		}
	}
	else if (WantedSpeed < 0)
	{
		if (WantedDistance < getRestLength())
		{
			Shrinks(dt);
		}
	}
	else
	{
		m_growingState = 0;
	}
}

void btCable::beginIterativeSolve()
{
	m_iter.active = true;
	m_iter.total = m_cfg.piterations;
	m_iter.current = 0;
}

bool btCable::stepOneIteration()
{
	if (!m_iter.active || m_iter.current >= m_iter.total)
		return false;

	// Run exactly one internal relaxation/constraint-projection iteration
	solveSingleCableIteration(m_iter.current);

	m_iter.current++;

	//b3Printf("Iteration: %d / %d", m_iter.current, m_iter.total);

	return m_iter.current < m_iter.total;
}

void btCable::endIterativeSolve()
{
	if (!m_iter.active)
		return;

	// Assures we go through all the iterations before finishing
	while (stepOneIteration());

	EndConstraintsSolve();

	// Clear session state
	m_iter = IterativeSolveState();

	updateNodeData();
}

void btCable::PrepareSolver()
{
	int i, ni;

	// Prepare cable
	m_tenseAccumulator = 0.0;

	// Prepare nodes
	for (i = 0, ni = m_nodes.size(); i < ni; ++i)
	{
		Node& node = m_nodes[i];
		node.cptIteration = 0;
		node.computeNodeConstraint = true;
		node.m_splitv = btVector3(0, 0, 0);
		node.m_nbCollidingObjectPotential = 0;

		// XPDB
		node.m_q_sub = node.m_x;  // Reset each substep for damping
	}

	// Prepare links
	for (i = 0, ni = m_links.size(); i < ni; ++i)
	{
		Link& l = m_links[i];
		l.m_c3 = l.m_n[1]->m_q - l.m_n[0]->m_q;
		l.m_c2 = 1.0 / (l.m_c3.length2() * l.m_c0);

		// XPDB
		l.m_lambda = 0.0;
	}

	// Prepare anchors
	for (i = 0, ni = this->m_anchors.size(); i < ni; ++i)
	{
		Anchor& a = this->m_anchors[i];
		Node* n = a.m_node;
		btRigidBody* b = a.m_body; 

		// Relative anchor position
		const btVector3 ra = b->getWorldTransform().getBasis() * a.m_local;

		// Node's masses
		const double invMassNode = n->m_im;
		const double massNode = invMassNode < DBL_EPSILON ? 0.0 : 1.0 / invMassNode;

		// Get mass as normal
		const double invMassBody = b->getInvMass();
		const double massBody = b->getMass();

		// Tweaked mass
		const double tweakedMass = massNode + massBody * a.m_bodyMassRatio;
		const double invTweakedMass = tweakedMass < FLT_EPSILON ? 0.0 : 1.0 / tweakedMass;

		// Masses' matrices
		a.m_c0 = ImpulseMatrix(m_sst.sdt, invMassNode, invMassBody, b->getInvInertiaTensorWorld(), ra);
		a.m_c0_massBalance = ImpulseMatrix(m_sst.sdt, invTweakedMass, invMassBody, b->getInvInertiaTensorWorld(), ra);
		a.m_c1 = ra;
		a.m_c2 = m_sst.sdt * invMassNode;
		a.m_c2_massBalance = m_sst.sdt * invTweakedMass;
		a.m_body->activate();
		a.m_lastTension = btVector3(0, 0, 0);	
		if (m_world->GetIndexSubIteration() == 0)
		{
			a.m_totalTension = btVector3(0, 0, 0);
		}

		// When a node's mass is greater than 5% of the attached body's mass, we cancel out the AnchorConstraintPlacement
		a.m_anchorPlacement = massBody < FLT_EPSILON ? true : (tweakedMass / massBody < 5.0 / 100.0);
	}

	// Prepare contacts
	{
		// SolveConstraint could be called more than once per frame
		// To keep contact manifold during all these iteration we had to them a certain lifetime
		// At the last iteration if the lifeTime = 0 we could remove the manifold
		if (m_cpt == m_solverSubStep)
		{
			m_cpt = 0;
		}
		if (m_cpt == 0)
		{
			resetManifoldLifeTime();
		}
		m_cpt++;
	}

	// Run broadPhase to get candidates
	runBroadPhase();
}

void btCable::solveSingleCableIteration(int currentIter)
{
	bool lastStep = currentIter == m_cfg.piterations - 1;
	bool firstStep = currentIter == 0;
	bool runBending = (currentIter + 1) % 2 == 0 || lastStep;
	bool runCollisionDetection = firstStep || (currentIter + 1) % m_substepDelayCollisionNarrow == 0 || lastStep;
	bool runContactConstraint = (currentIter + 1) % m_substepDelayCollisionSolver == 0 || lastStep;

	updateNodeDeltaPos(currentIter);

	anchorConstraint();

	distanceConstraint(currentIter);

	if (useLRA)
	{
		LRAConstraint();
	}

	if (useBending && runBending)
	{
		bendingConstraint();
	}

	if (!useCollision)
	{
		return;
	}

	if (runCollisionDetection)
	{
		runNarrowPhase();
	}

	if (runContactConstraint)
	{
		contactConstraint();
	}
}

void btCable::EndConstraintsSolve()
{
	// Account for MassAtImpact
	for (int i = 0; i < m_anchors.size(); ++i)
	{
		Anchor& anchor = this->m_anchors[i];
		btRigidBody* body = anchor.m_body;

		if (body->isStaticOrKinematicObject()) continue;

		if (!body->IsMassAtImpactActive()) continue;

		body->storeAnchorLastState(anchor.m_dist);
	}

	if (m_useAnchorConstraintPlacement)
	{
		anchorConstraintPlacement();
	}

	// TODO @BenH: Add better manifolds
	// Clear manifolds all cables manifolds
	for (int i = manifolds.size() - 1; i >= 0; --i)
	{
		m_world->getDispatcher()->releaseManifold(manifolds.at(i).manifold);
		manifolds.removeAtIndex(i);

		for (int i = 0; i < _nodePairContact.size(); i++)
		{
			NodePairNarrowPhase* nodePair = &_nodePairContact.at(i);
			btPersistentManifold* manifold = m_world->getDispatcher()->getNewManifold(this, nodePair->pair->body);
			CableManifolds cm = CableManifolds(manifold, m_solverSubStep);
			manifolds.push_back(cm);
			nodePair->manifold = manifold;
			nodePair->haveManifoldsRegister = true;

			// Obj 0 = Cable
			// Obj 1 = RigidBody
			const btVector3& pointB = nodePair->hitPoint;
			const btVector3& normal = nodePair->normal;
			const btScalar distance = nodePair->distance;

			btManifoldPoint newPoint = btManifoldPoint(btVector3(0, 0, 0), btVector3(0, 0, 0), normal, distance);
			newPoint.m_positionWorldOnA = pointB + normal * distance;
			newPoint.m_positionWorldOnB = pointB;
			manifold->addManifoldPoint(newPoint, true);
		}
	}

	// Grows/Shrinks only in physic
	updateLength(m_sst.sdt);

	_nodePairContact.clear();
}

void btCable::updateNodeData()
{
	const btScalar frameDT = 1.0 / m_sst.fdt;
	const btScalar subFrameDT = 1.0 / m_sst.sdt;
	const btScalar damping = 1.0 - m_cfg.kDP;
	for (int i = 0; i < m_nodes.size(); ++i)
	{
		// Update velocities for the cable
		Node& n = m_nodes[i];
		n.m_vn = n.m_v;
		n.m_v = (n.m_x - n.m_q) * subFrameDT * damping;

		// Only update data for last substep
		if (m_world->GetIndexSubIteration() == m_world->GetSubIteration() - 1)
		{
			btVector3 nodeVelocity = (n.m_x - n.m_xn) * frameDT * damping;

			// Update velocities for the hydro's forces
			n.m_movingAverage[n.m_indexMovingAverage] = nodeVelocity;

			btVector3 average = btVector3(0, 0, 0);
			int currentIndex = (n.m_indexMovingAverage + 1) % n.m_maxSizeMovingAverage;  // start after current value, first has weight of 0.0

			float weight = 0.0f;
			float weightPortion = 1.0f / n.m_maxSizeMovingAverage;
			float totalWeight = 0.0f;
			for (int j = 0; j < n.m_maxSizeMovingAverage; j++)
			{
				average += n.m_movingAverage[currentIndex] * weight;

				totalWeight += weight;

				weight += weightPortion;
				currentIndex = (currentIndex + 1) % n.m_maxSizeMovingAverage;
			}

			average = average / totalWeight;

			n.m_indexMovingAverage = (n.m_indexMovingAverage + 1) % n.m_maxSizeMovingAverage;

			// Update NodePos
			m_nodePos[i].x = n.m_x.getX();
			m_nodePos[i].y = n.m_x.getY();
			m_nodePos[i].z = n.m_x.getZ();

			// Update NodeData
			if (useHydroAero)
			{
				m_nodeData[i].velocity_x = average.getX();
				m_nodeData[i].velocity_y = average.getY();
				m_nodeData[i].velocity_z = average.getZ();
			}
			else
			{
				m_nodeData[i].velocity_x = nodeVelocity.getX();
				m_nodeData[i].velocity_y = nodeVelocity.getY();
				m_nodeData[i].velocity_z = nodeVelocity.getZ();
			}

			n.m_xn = n.m_x;  // Update previous pos with current
							 //n.m_f = btVector3(0, 0, 0);  // reset node total forces
		}

		// Calculate Volume
		float sizeElement = 0;
		if (i == 0)
		{
			sizeElement = (n.m_x - m_nodes[i + 1].m_x).length();
		}
		else if (i == m_nodes.size() - 1)
		{
			sizeElement = (n.m_x - m_nodes[i - 1].m_x).length();
		}
		else
		{
			sizeElement = (n.m_x - m_nodes[i - 1].m_x).length();
			sizeElement += (n.m_x - m_nodes[i + 1].m_x).length();
		}

		// Using a cylinder volume calculation with 2 links and divide by 2
		m_nodeData[i].volume = SIMD_PI * m_cableData->radius * m_cableData->radius * sizeElement * 0.5;
	}
}

void btCable::ResetForceAndVelocity()
{
	int nodeCount = m_nodes.size();
	btVector3 v0 = btVector3(0, 0, 0);
	for (int i = 0; i < nodeCount; i++)
	{
		m_nodes[i].m_v = v0;
		m_nodes[i].m_vn = v0;
		m_nodes[i].m_f = v0;

		ResetVelocityArray(i);
	}
}

void btCable::ResetNodePosition(const int nodeIndex, const btVector3 position)
{
	m_nodes[nodeIndex].m_x = position;
	m_nodes[nodeIndex].m_q = position;
	m_nodes[nodeIndex].m_xn = position;
}

void btCable::predictMotion(btScalar dt)
{
	cableState = Valid;
	int i, ni;

	// Grows/Shrinks the cable
	updateLength(dt);

	/* Update                */
	if (m_bUpdateRtCst)
	{
		m_bUpdateRtCst = false;
		updateLinkConstants();
	}

	/* Prepare                */
	m_sst.sdt = dt * m_cfg.timescale;
	m_sst.fdt = dt * m_cfg.timescale * m_world->GetSubIteration();
	m_sst.isdt = 1 / m_sst.sdt;
	m_sst.velmrg = m_sst.sdt * 3;
	m_sst.radmrg = getCollisionShape()->getMargin();
	m_sst.updmrg = m_sst.radmrg * (btScalar)0.25;

	// SoftRigidBody
	NodeForces* nodeForces = ((btSoftRigidDynamicsWorld*)m_world)->m_nodeForces;
	btVector3 nodeForceToApply = btVector3();
	for (i = 0, ni = m_nodes.size(); i < ni; ++i)
	{
		Node& n = m_nodes[i];
		n.m_q = n.m_x;

		float addedMass = 0.0f;
		if (isActive())
		{
			// Add gravity force
			if (useGravity)
			{
				n.m_f += m_gravity / n.m_im;
			}

			if (useHydroAero)
			{
				// Get the Hydro and Aero forces
				NodeForces& currentNodeForces = nodeForces[m_cableData->startIndex + i];

				// TODO @BenH. : Verif a déplacer dans Unity (appliqué 4 fois)
				// We check if currentNodeForces are correct, if not, then we dont apply these forces.
				if (std::isinf(currentNodeForces.x) || std::isnan(currentNodeForces.x) || std::isinf(currentNodeForces.y) || std::isnan(currentNodeForces.y) || std::isinf(currentNodeForces.z) || std::isnan(currentNodeForces.z))
				{
					cableState = InternalForcesError;
				}
				else
				{
					n.m_f.setValue(n.m_f.getX() + currentNodeForces.x, n.m_f.getY() + currentNodeForces.y, n.m_f.getZ() + currentNodeForces.z);
				}

				// Integrate addedMass for each substep
				addedMass = currentNodeForces.ma;
			}
		}

		const btScalar mass = 1.0f / n.m_im + addedMass;
		btVector3 acceleration = n.m_f / mass;
		n.m_v += acceleration * m_sst.sdt;
		n.m_x += n.m_v * m_sst.sdt;
		n.m_f = btVector3(0, 0, 0);
	}

	/* Bounds                */
	updateBounds();

	/* Clear contacts        */
	m_rcontacts.resize(0);
	m_scontacts.resize(0);
}

void btCable::Grows(float dt)
{
	int sizeNode = m_nodes.size();
	int lastIndexNode = sizeNode - 1;

	int sizeLink = m_links.size();
	int lastIndexLink = sizeLink - 1;
	Link& lastLink = m_links.at(lastIndexLink);

	// Get the all distances needed
	btScalar lengthToAdd = dt * abs(WantedSpeed);
	btScalar currentLinkRL = lastLink.m_rl;
	btScalar newLinkRL = currentLinkRL + lengthToAdd;
	btScalar totalLengthRL = getRestLength();
	btScalar currentCableRL = getLinkRestLength(lastIndexLink);
	btScalar maxRL = currentCableRL * 2.0;

	// If there is a target length we don't extend rl more than necessary
	if (WantedDistance > 0)
	{
		btScalar value = WantedDistance - getRestLength();
		if (value > FLT_EPSILON && value < dt * WantedSpeed)
		{
			newLinkRL = value + currentLinkRL;
			WantedSpeed = 0;
		}
	}

	// if we had to add a node
	while (newLinkRL > maxRL)
	{
		// Limit the node's number
		if (((btSoftRigidDynamicsWorld*)m_world)->getTotalNumNodes() >= m_worldInfo->maxNodeNumber || sizeNode >= m_worldInfo->maxNodeNumberPerCable)
		{
			newLinkRL = maxRL;
			m_growingState = 4;
			WantedSpeed = 0;
			break;
		}			
		
		// Get the affected anchor to modify it after the node and links removing
		Anchor* anchor = nullptr;
		for (int i = 0; i < m_anchors.size(); i++)
		{
			Anchor* currentAnchor = &m_anchors.at(i);
			// Look up for its new node's data
			if (currentAnchor->m_node->index == lastIndexNode)
			{
				anchor = currentAnchor;
				break;
			}
		}

		// Get the nodes
		Node* lastNode = &m_nodes[lastIndexNode];
		Node* newNode = nullptr;
		Node* beforeNewNode = &m_nodes[lastIndexNode - 1];

		// Remove the last link
		m_links.removeAtIndex(lastIndexLink);
		lastIndexLink--;
		sizeLink--;

		// Calculate the new node's position
		btVector3 dirCable = (lastNode->m_x - beforeNewNode->m_x).normalized();
		btVector3 newNodePos = beforeNewNode->m_x + dirCable * currentCableRL;

		// Create the new node again
		appendNode(newNodePos, 1.0);		
		lastIndexNode++;
		sizeNode++;		
		
		// Swap the new node and the node-1
		m_nodes.swap(lastIndexNode - 1, lastIndexNode);

		// Get the nodes
		lastNode = &m_nodes[lastIndexNode];
		newNode = &m_nodes[lastIndexNode - 1];
		beforeNewNode = &m_nodes[lastIndexNode - 2];

		// Set the indexex
		lastNode->index = lastIndexNode;
		newNode->index = lastIndexNode - 1; 
		beforeNewNode->index = lastIndexNode - 2;

		// Re-synchronize the anchor's node
		if (anchor)
		{
			anchor->m_node = lastNode;
		}

		// Calculte the before new node's mass
		btScalar beforelastNodeMass = m_linearMass * 0.5f * (currentCableRL + (lastIndexLink < 0 ? 0.0 : m_links.at(lastIndexLink).m_rl));
		setMass(lastIndexNode - 2, beforelastNodeMass);

		// Update the new node's and before new node's velocities
		newNode->m_v = beforeNewNode->m_v;
		beforeNewNode->m_v = ((lastIndexNode < 3 ? btVector3(0,0,0) : m_nodes[lastIndexNode - 3].m_v) + beforeNewNode->m_v) * 0.5;

		// Create the two links (n and n-1)
		appendLink(lastIndexNode - 2, lastIndexNode - 1, m_materials[0]);
		lastIndexLink++;
		sizeLink++;
		appendLink(lastIndexNode - 1, lastIndexNode, m_materials[0]);
		lastIndexLink++;
		sizeLink++;

		newLinkRL -= currentCableRL;
	}

	// Set the Rest Length
	m_links.at(lastIndexLink).m_rl = newLinkRL;
	m_links.at(lastIndexLink).m_c1 = newLinkRL * newLinkRL;

	// Set the mass for the last-1 node
	btScalar firstNodeMass = m_linearMass * 0.5f * newLinkRL;
	setMass(lastIndexNode, firstNodeMass);
	if (sizeNode > 2)
	{
		btScalar linkMass = firstNodeMass + m_linearMass * 0.5f * (m_links[lastIndexLink - 1].m_rl);
		setMass(lastIndexNode - 1, linkMass);
	}
	else
	{
		setMass(lastIndexNode - 1, firstNodeMass);
	}

	m_growingState = 1;
}

void btCable::Shrinks(float dt)
{
	int sizeNode = m_nodes.size();
	int lastIndexNode = sizeNode - 1;

	int sizeLink = m_links.size();
	int lastIndexLink = sizeLink - 1;
	Link& lastLink = m_links.at(lastIndexLink);

	// Get the all distances needed
	btScalar lengthToRemove = dt * abs(WantedSpeed);
	btScalar currentLinkRL = lastLink.m_rl;
	btScalar newLinkRL = currentLinkRL - lengthToRemove;
	btScalar totalLengthRL = getRestLength();
	btScalar currentCableRL = getLinkRestLength(lastIndexLink);
	btScalar minRL = currentCableRL * 0.5;


	// If there is a target length, we don't extend newRL more than necessary
	if (WantedDistance > 0)
	{
		btScalar value = totalLengthRL - WantedDistance;
		if (value > FLT_EPSILON && value < lengthToRemove)
		{
			newLinkRL = currentLinkRL - value;
			m_growingState = 2;
			WantedSpeed = 0;
		}
	}

	//  Avoid adjusting the length when creating a cable with a rest length shorter than the minimum length
	if (totalLengthRL < minRL)
	{
		newLinkRL = currentLinkRL;
		m_growingState = 2;
		WantedSpeed = 0;
	}
	// To avoid shrink to much
	else if (totalLengthRL - lengthToRemove < minRL)
	{
		newLinkRL = minRL;
		m_growingState = 2;
		WantedSpeed = 0;
	}
	// We cannot remove an anchor
	else if (m_nodes.at(lastIndexNode - 1).m_battach != 0)
	{
		// Minimum shrink lenght
		if (newLinkRL < minRL)
		{
			newLinkRL = minRL;
			m_growingState = 3;
			WantedSpeed = 0;
		}
	}
	else 
	{
		// if we had to delete a node
		while (newLinkRL < minRL)
		{
			// Limit the link size when they are 2 nodes only
			if (sizeNode <= 2)
			{
				newLinkRL = minRL;
				WantedSpeed = 0;
				break;
			}

			// Get the affected anchor to modify it after the node and links removing
			Anchor* anchor = nullptr;
			for (int i = 0; i < m_anchors.size(); i++)
			{
				Anchor* currentAnchor = &m_anchors.at(i);
				// Look up for its new node's data
				if (currentAnchor->m_node->index == lastIndexNode)
				{
					anchor = currentAnchor;
					break;
				}
			}

			// Remove the last link and the last-1 link
			m_links.removeAtIndex(lastIndexLink);
			lastIndexLink--;
			sizeLink--;
			m_links.removeAtIndex(lastIndexLink);
			lastIndexLink--;
			sizeLink--;

			// Remove the last-1 node
			removeNodeAt(lastIndexNode - 1);
			lastIndexNode--;
			sizeNode--;

			// Add the new link between the last-2 node (which currenlty last-1) and the last node
			appendLink(lastIndexNode - 1, lastIndexNode, m_materials[0]);
			lastIndexLink++;
			sizeLink++;

			// Re-synchronize the anchor's node
			if (anchor)
			{
				anchor->m_node = &m_nodes.at(lastIndexNode);
			}

			newLinkRL += currentCableRL;
		}
	}

	// Set the Rest Length
	m_links.at(lastIndexLink).m_rl = newLinkRL;
	m_links.at(lastIndexLink).m_c1 = newLinkRL * newLinkRL;

	// Set the mass for the last-1 node
	btScalar firstNodeMass = m_linearMass * 0.5f * newLinkRL;
	setMass(lastIndexNode, firstNodeMass);
	if (sizeNode > 2)
	{
		btScalar linkMass = firstNodeMass + m_linearMass * 0.5f * (m_links[lastIndexLink - 1].m_rl);
		setMass(lastIndexNode - 1, linkMass);
	}
	else
	{
		setMass(lastIndexNode - 1, firstNodeMass);
	}

	m_growingState = 1;
}

void btCable::synchNodesInfos()
{
	int nodeCount = m_nodes.size();

	for (int i = 0; i < nodeCount; i++)
	{
		// Update NodePos
		m_nodePos[i].x = m_nodes[i].m_x.getX();
		m_nodePos[i].y = m_nodes[i].m_x.getY();
		m_nodePos[i].z = m_nodes[i].m_x.getZ();

		// Update NodeData
		m_nodeData[i].velocity_x = m_nodes[i].m_v.getX();
		m_nodeData[i].velocity_y = m_nodes[i].m_v.getY();
		m_nodeData[i].velocity_z = m_nodes[i].m_v.getZ();
	}
}

void btCable::updateCurveResponse(btScalar* dataX, btScalar* dataY, int size)
{
	vector<double> vectorX;
	vector<double> vectorY;
	for (int i = 0; i < size; ++i)
	{
		vectorX.push_back(dataX[i]);
		vectorY.push_back(dataY[i]);
	}
	setControlPoint(vectorX, vectorY);
}

void btCable::setControlPoint(vector<btScalar> dataX, vector<btScalar> dataY)
{
	if (spline) delete spline;

	for (int i = 0; i < dataX.size(); i++)
	{
		dataX[i] += 1;
	}
	spline = new MonotonicSpline1D(dataX, dataY);
}

#pragma region Constraints

bool btCable::shouldTestObject(btCollisionObject* colObj) const
{
	if (colObj->getInternalType() != CO_RIGID_BODY)
		return false;

	// Check for cable mask
	if (!(colObj->getBroadphaseHandle()->m_collisionFilterMask & (1 << 3)))
		return false;

	if (!colObj->hasContactResponse())
		return false;

	if (m_collisionDisabledObjects.findLinearSearch(colObj) != m_collisionDisabledObjects.size())
		return false;

	return true;
}

void btCable::collectPotentials(btCollisionObjectArray& collisionObjectArray, std::vector<btCollisionObject*>& out) const
{
	out.reserve(collisionObjectArray.size());
	for (int i = 0, n = collisionObjectArray.size(); i < n; ++i)
	{
		btCollisionObject* co = collisionObjectArray[i];
		if (shouldTestObject(co))
			out.push_back(co);
	}
}

// for each potential, cache its AABB + interpolation velocity
void btCable::buildObjData(const std::vector<btCollisionObject*>& pots, std::vector<ObjData>& out) const
{
	out.clear();
	out.reserve(pots.size());
	for (auto* co : pots)
	{
		btVector3 mi, ma;
		co->getCollisionShape()->getAabb(co->getWorldTransform(), mi, ma);
		btVector3 velB = co->getInterpolationLinearVelocity();
		//velB = btVector3(0, 0, 0);
		out.emplace_back(ObjData{co, mi, ma, velB});
	}
}

// parallel AABB test
void btCable::runBroadPhase()
{
	_candidates.clear();
	btCollisionObjectArray objects = m_world->getCollisionObjectArray();
	if (objects.size() < 2)
		return;

	// Filter object we can't hit (disabled, wrong mask etc...)
	std::vector<btCollisionObject*> potentials;
	collectPotentials(objects, potentials);

	if (potentials.empty())
		return;

	// cache their AABBs + velocities
	std::vector<ObjData> objData;
	buildObjData(potentials, objData);

	// Before starting threads, first check if the cable global AABB overlaps any objects
	bool anyHit = false;
	for (auto& od : objData)
	{
		if (m_bounds[0].x() <= od.maxAabb.x() && m_bounds[1].x() >= od.minAabb.x() &&
			m_bounds[0].y() <= od.maxAabb.y() && m_bounds[1].y() >= od.minAabb.y() &&
			m_bounds[0].z() <= od.maxAabb.z() && m_bounds[1].z() >= od.minAabb.z())
		{
			anyHit = true;
			break;
		}
	}

	if (!anyHit)
	{
		// no possible collision this frame, skip threading
		return;
	}

	int nodeCount = m_nodes.size();

	// one bucket per thread
	btAlignedObjectArray<btAlignedObjectArray<BroadPhasePair>> threadBuckets;
	threadBuckets.resize(omp_get_max_threads());

#pragma omp parallel for schedule(static, 1)
	for (int i = 0; i < nodeCount; ++i)
	{
		int tid = omp_get_thread_num();
		Node* n = &m_nodes[i];

		// static node box (no margin first)
		btVector3 lo, hi;
		setNodeBoundingBox(n->m_x, n->m_q, 0, &lo, &hi);
		btVector3 nodeVel = (n->m_x - n->m_q) / m_sst.sdt;
		//btVector3 zero = btVector3(0, 0, 0);

		// test each object
		for (const auto& od : objData)
		{
			if (aabbTestMargin(nodeVel, od.objVelocity, lo, hi, od.minAabb, od.maxAabb)) continue;

			BroadPhasePair bp;
			bp.node = n;
			bp.body = od.obj;
			bp.bodyType = od.obj->getInternalType();
			threadBuckets[tid].push_back(bp);
		}
	}

	// merge results
	for (int i = 0; i < threadBuckets.size(); ++i)
	{
		btAlignedObjectArray<BroadPhasePair>& bucket = threadBuckets[i];
		for (int j = 0; j < bucket.size(); ++j)
		{
			_candidates.push_back(bucket[j]);
		}
	}
}

bool btCable::aabbTestMargin(btVector3 nodeVel, btVector3 objVel, btVector3 nodeMinAabb, btVector3 nodeMaxAabb, btVector3 minAabb, btVector3 maxAabb)
{
	btScalar margin = m_collisionMargin + (objVel - nodeVel).length() * m_sst.sdt;
	// expand and test aabb
	return nodeMinAabb.x() - margin > maxAabb.x() || nodeMaxAabb.x() + margin < minAabb.x() ||
		   nodeMinAabb.y() - margin > maxAabb.y() || nodeMaxAabb.y() + margin < minAabb.y() ||
		   nodeMinAabb.z() - margin > maxAabb.z() || nodeMaxAabb.z() + margin < minAabb.z();
}

void btCable::runNarrowPhase()
{
	//Clear & Pre-allocate outputs
	_nodePairContact.clear();
	_nodePairContact.reserve(_nodePairContact.size() + _candidates.size());

	if (_candidates.size() <= 0)
	{
		return;
	}

	for (int i = 0; i < _candidates.size(); i++)
	{
		BroadPhasePair* c = &_candidates.at(i);
		Node* node = c->node;
		btRigidBody* rb = btRigidBody::upcast(c->body);
		if (!rb) continue;

		btVector3 currentPos = node->m_x;
		btVector3 prevPos = node->m_q;
		btScalar margin = computeCollisionMargin(rb->getCollisionShape());

		// Get transform of the rigid body
		btTransform rbTransform = rb->getWorldTransform();
		btTransform rbPrevTransform = rb->getPreviousWorldTransform();

		btVector3 normalContact;
		btVector3 hitContact;
		bool foundCollision = false;
		btScalar toi = 1.0f;  // Time of impact
		btScalar penetration = 0.0f;

		// btVector3 rayStart = prevPos;
		// btVector3 rayEnd;
		btTransform rbTransformAtTime;

		//Single sweep in relative motion
		// Relative-motion single sweep against rb fixed at rbPrevTransform
		btVector3 nodeStart = prevPos;
		btVector3 nodeEnd = currentPos;

		// Compute the motion of the rigid body's material point that coincides with nodeStart at t0
		btVector3 pLocal = rbPrevTransform.inverse() * nodeStart;
		btVector3 rbPointEnd = rbTransform * pLocal;
		btVector3 deltaNode = nodeEnd - nodeStart;
		btVector3 deltaRbPt = rbPointEnd - nodeStart;
		btVector3 relEnd = nodeStart + (deltaNode - deltaRbPt);

		// Skip if motion is negligible
		btVector3 sweepDelta = relEnd - nodeStart;

		if (sweepDelta.length2() < FLT_EPSILON)
		{
			continue;
		}

		btTransform fromTransform(btQuaternion::getIdentity(), nodeStart);
		btTransform toTransform(btQuaternion::getIdentity(), relEnd);

		btCollisionWorld::ClosestConvexResultCallback callback(nodeStart, relEnd);
		callback.m_collisionFilterGroup = rb->getBroadphaseHandle()->m_collisionFilterMask;
		callback.m_collisionFilterMask = rb->getBroadphaseHandle()->m_collisionFilterGroup;

		btTransform rbStatic = rbPrevTransform;

		btCollisionWorld::objectQuerySingle(
			&_nodeContactSphere, fromTransform, toTransform,
			rb, rb->getCollisionShape(), rbStatic,
			callback, btScalar(0.0f));

		if (callback.hasHit())
		{
			foundCollision = true;

			// Build rb transform at TOI (interpolate pose)
			btVector3 startPos = rbPrevTransform.getOrigin();
			btVector3 endPos = rbTransform.getOrigin();
			rbTransformAtTime.setOrigin(lerp(startPos, endPos, toi));

			btQuaternion startRot = rbPrevTransform.getRotation();
			btQuaternion endRot = rbTransform.getRotation();
			btQuaternion rot = slerp(startRot, endRot, toi);
			rbTransformAtTime.setRotation(rot);

			// Convert hit point and normal from rbPrevTransform world frame -> rb local
			btVector3 hitWorld_prev = callback.m_hitPointWorld;
			btVector3 nWorld_prev = callback.m_hitNormalWorld.normalized();

			rbStatic = rbPrevTransform;
			pLocal = rbStatic.inverse() * hitWorld_prev;

			// Transform to world at time-of-impact
			btVector3 hitWorld_t = rbTransformAtTime * pLocal;
			btVector3 nWorld_t = (rbTransformAtTime.getBasis() * (rbPrevTransform.getBasis().transpose() * nWorld_prev)).normalized();

			// Store contact data at time-of-impact; push out by node margin
			normalContact = nWorld_t;
			hitContact = hitWorld_t + nWorld_t * (m_collisionMargin + FLT_EPSILON);

			// Conservative remaining-trajectory penetration estimate along the normal
			btVector3 centerAtHitRel = nodeStart.lerp(relEnd, callback.m_closestHitFraction);
			btVector3 remaining = relEnd - centerAtHitRel;
			penetration = remaining.dot(nWorld_t);
			penetration = btMax<btScalar>(0, penetration);
		}

		if (!foundCollision)
		{
			continue;  // Skip to next candidate if no collision found
		}

		// Debug
		// node->m_xOut = hitContact;
		// node->m_xOutMargin = margin;
		// node->m_xStartOut = prevPos;
		// node->m_xStartRay = rayStart;
		// node->m_xEndRay = rayEnd;

		// Output results
		NodePairNarrowPhase pair;
		pair.pair = c;
		pair.node = node;
		pair.hitPoint = hitContact;
		pair.timeOfImpact = toi;
		pair.distance = penetration;
		pair.margin = margin;
		pair.worldTransform = rbTransformAtTime;
		pair.normal = normalContact;

		_nodePairContact.push_back(pair);
	}
}

void btCable::solveConstraints()
{
	PrepareSolver();

	// Solve constraints
	for (int i = 0; i < m_cfg.piterations; ++i)
	{
		solveSingleCableIteration(i);
	}

	EndConstraintsSolve();
}

void btCable::resetManifoldLifeTime()
{
	int size = manifolds.size();
	for (int i = 0; i < size; i++)
	{
		manifolds.at(i).lifeTime = m_solverSubStep;
	}
}

void btCable::setNodeBoundingBox(btVector3 mx, btVector3 mq, btScalar margin, btVector3* minLink, btVector3* maxLink)
{
	const btVector3 lower(
		btMin(mx.x(), mq.x()),
		btMin(mx.y(), mq.y()),
		btMin(mx.z(), mq.z()));

	const btVector3 upper(
		btMax(mx.x(), mq.x()),
		btMax(mx.y(), mq.y()),
		btMax(mx.z(), mq.z()));

	const btVector3 expand(margin, margin, margin);
	*minLink = lower - expand;
	*maxLink = upper + expand;
}

void btCable::updateNodeDeltaPos(int iteration)
{
	Node* node;
	btScalar deltaPos;
	for (int i = 0; i < m_nodes.size(); i++)
	{
		node = &m_nodes.at(i);
		deltaPos = (node->m_x - node->posPreviousIteration).length();
		if (deltaPos > 0.0001 || iteration == 0 || node->m_battach != 0)
		{
			node->cptIteration++;
			node->computeNodeConstraint = true;
			node->posPreviousIteration = node->m_x;
		}
		else
		{
			node->computeNodeConstraint = false;
		}
	}
}

void btCable::anchorConstraint()
{
	BT_PROFILE("PSolve_Anchors");
	const btScalar kAHR = m_cfg.kAHR;
	const btScalar dt = m_sst.sdt;
	for (int i = 0, ni = this->m_anchors.size(); i < ni; ++i)
	{
		Anchor& anchor = m_anchors[i];
		btRigidBody& body = *anchor.m_body;
		Node& node = *anchor.m_node;

		const btVector3 wa = body.getWorldTransform() * anchor.m_local;
		const btVector3 va = anchor.m_body->getVelocityInLocalPoint(anchor.m_c1) * dt;
		const btVector3 vb = node.m_x - node.m_q_sub;
		const btVector3 vr = (va - vb) + (wa - node.m_x) * kAHR;
		btVector3 impulse = anchor.m_c0_massBalance * vr;

		btScalar currentTension = anchor.m_lastTension.length();
		anchor.m_lastTension += impulse / dt;
		btScalar finalTension = anchor.m_lastTension.length();
		if (m_maxTension >= 0 && finalTension >= m_maxTension)
		{
			anchor.m_lastTension = anchor.m_lastTension.normalized() * m_maxTension;
			impulse *= (anchor.m_lastTension.length() - currentTension) / (finalTension - currentTension);
		}

		// Account for max tension constraint when updating average tension during this frame
		anchor.m_totalTension += impulse / dt;

		// Update anchor's data
		anchor.m_dist = wa.distance(node.m_x);
		node.m_x += impulse * anchor.m_c2_massBalance;
		anchor.m_body->applyImpulse(-impulse, anchor.m_c1);
	}
}

void btCable::anchorConstraintPlacement()
{
	const btScalar kAHR = m_cfg.kAHR;
	const btScalar dt = m_sst.sdt;

	for (int i = 0, ni = this->m_anchors.size(); i < ni; ++i)
	{
		Anchor& anchor = m_anchors[i];
		if (!anchor.m_anchorPlacement)
		{
			continue;
		}
		btRigidBody& body = *anchor.m_body;
		Node& node = *anchor.m_node;

		const btVector3 wa = body.getWorldTransform() * anchor.m_local;
		const btVector3 va = anchor.m_body->getVelocityInLocalPoint(anchor.m_c1) * dt;
		const btVector3 vb = node.m_x - node.m_q_sub;
		const btVector3 vr = (va - vb) + (wa - node.m_x) * kAHR;
		btVector3 impulse = anchor.m_c0_massBalance * vr;

		btScalar currentTension = anchor.m_lastTension.length();
		anchor.m_lastTension += impulse / dt;
		btScalar finalTension = anchor.m_lastTension.length();
		if (m_maxTension >= 0 && finalTension >= m_maxTension)
		{
			anchor.m_lastTension = anchor.m_lastTension.normalized() * m_maxTension;
			impulse *= (anchor.m_lastTension.length() - currentTension) / (finalTension - currentTension);
		}
		
		// Account for max tension constraint when updating average tension during this frame
		anchor.m_totalTension += impulse / dt;

		// Update anchor's data
		anchor.m_dist = wa.distance(node.m_x);
		node.m_x = wa;
		anchor.m_body->applyImpulse(-impulse, anchor.m_c1);
	}
}

void btCable::distanceConstraint(int currentIter)
{
	(this->*m_distanceFunction)();
}

void btCable::distanceConstraintBullet()
{
	BT_PROFILE("PSolve_Links");
	const btScalar stiffness = m_materials[0]->m_kLST;
	for (int i = 0, ni = m_links.size(); i < ni; ++i)
	{
		Link& l = m_links[i];
		if (l.m_c0 > 0)
		{
			Node& a = *l.m_n[0];
			Node& b = *l.m_n[1];
			const btVector3 del = b.m_x - a.m_x;
			const btScalar len2 = del.length2();
			if (l.m_c1 + len2 > SIMD_EPSILON)
			{
				const btScalar k = ((l.m_c1 - len2) / (l.m_c0 * (l.m_c1 + len2))) * stiffness;
				a.m_x -= del * (k * a.m_im);
				b.m_x += del * (k * b.m_im);
				m_tenseAccumulator = max(m_tenseAccumulator, (len2 - l.m_c1) / l.m_c1);
			}
		}
	}
	m_tenseAccumulator = min(m_tenseAccumulator, m_maxAccumulator);
}

void btCable::distanceConstraintXPBD()
{
	BT_PROFILE("PSolve_Links");

	Link* l;
	Node* a;
	Node* b;

	const btScalar dt = m_sst.sdt;
	const btScalar invDt2 = 1.0 / (dt * dt);
	const btScalar stiffness = m_materials[0]->m_kLST;

	//// @TEST(jeremy) Physical based stiffness
	//// https://en.wikipedia.org/wiki/Young%27s_modulus
	//// https://en.wikipedia.org/wiki/Hooke%27s_law#Derived_formulae
	//const btScalar crossSectionArea = SIMD_PI * m_cableData->radius * m_cableData->radius;  // m²
	//const btScalar youngModulus = 300 * 1e09;  // Pa
	//stiffness = crossSectionArea * youngModulus;
	//// @TEST(jeremy) Physical based stiffness

	// Scaled compliance to be deltaT independant
	// const btScalar alpha_t = 1.0 / cableStiffness * invDt2;
	const btScalar alpha_t = btClamped(1.0 - stiffness, 0.0, 1.0) * invDt2;

	// Scaled damping to be deltaT independant
	const btScalar beta_t = dampingStiffness * dt * dt;
	const btScalar gamma_t = (alpha_t * beta_t) / dt;


	for (int i = 0; i < m_links.size(); ++i)
	{
		l = &m_links[i];
		a = l->m_n[0];
		b = l->m_n[1];
		if (!a->computeNodeConstraint && !b->computeNodeConstraint)
			continue;

		a->computeNodeConstraint = true;
		b->computeNodeConstraint = true;

		// Guard against zero division error
		if (a->m_im < SIMD_EPSILON && b->m_im < SIMD_EPSILON)
		{
			continue;
		}

		// Get cable segment
		btVector3 AB = b->m_x - a->m_x;
		btScalar L = AB.length();
		if (L < SIMD_EPSILON)
		{
			continue;
		}
		btVector3 linkDirection = AB.normalized();

		// XPBD
		// @TODO(jeremy) Use squared distance constraint to be more numerically stable
		// https://matthias-research.github.io/pages/publications/XPBD.pdf

		// Distance constraint --> Cs​(b​, a​) = ∥b​−a∥ − L0
		const btScalar C = L - l->m_rl;
		// dC/d --> Power rule + chain rule --> 0.5 * d^0.5 * 2 * d --> d / ||d||
		const btVector3 Ca_gradient = -linkDirection;
		const btVector3 Cb_gradient = linkDirection;

		// Rayleigh damping constraint -> Project relative motion onto the motion axis
		// If segment is getting longer, the projection is positive and damping correction will pull them back in
		// If segment is getting smaller, the projection is negative and damping correction will pull them apart
		// ∇C·(xi - xn) = linkDirection · [xbi​−xbn, ​xai​−xan​​]
		btScalar damping_constraint = linkDirection.dot((b->m_x - b->m_q) - (a->m_x - a->m_q));

		// XPBD lagrangian
		// ∇C(x) * Transpose(∇C(x)) = ∥∇C(x)∥2 = 1 (because ∥∇C(x)∥ == 1)
		const btScalar invmeff = a->m_im + b->m_im;
		const btScalar numer = -(C + alpha_t * l->m_lambda + gamma_t * damping_constraint);
		const btScalar denom = (1.0 + gamma_t) * invmeff + alpha_t;
		const btScalar dLambda = numer / denom;
		l->m_lambda += dLambda;

		//// @TEST(jeremy) Used to scale / boost lagrangian 
		//const btScalar omegaRef = 1.0;
		//btScalar omegaA = omegaRef;
		//btScalar omegaB = omegaRef;
		//const btVector3 dxA = a->m_im * Ca_gradient * dLambda * omegaA;
		//const btVector3 dxB = b->m_im * Cb_gradient * dLambda * omegaB;
		//a->m_x += dxA;
		//b->m_x += dxB;
		//// @TEST(jeremy) Used to scale / boost lagrangian 

		// Update iteration (n and n-1) positions
		const btVector3 dxA = a->m_im * Ca_gradient * dLambda;
		const btVector3 dxB = b->m_im * Cb_gradient * dLambda;
		a->m_x += dxA;
		b->m_x += dxB;
	}
}

void btCable::LRAConstraint()
{
	LRAHierachique();

	if (invertLRA)
	{
		btScalar distance = 0;
		Node& a = m_nodes[0];
		bool aMove = a.computeNodeConstraint;
		for (int i = 0; i < m_anchors.size(); ++i)
			if (a.index == m_anchors[i].m_node->index)
				a.m_x = m_anchors[i].m_c1 + m_anchors[i].m_body->getCenterOfMassPosition();

		for (int i = 0; i < m_links.size(); ++i)
		{
			Link& l = m_links[i];
			Node* b = l.m_n[1];
			if (!aMove && !b->computeNodeConstraint) continue;
			if (!b->computeNodeConstraint)
			{
				b->computeNodeConstraint = true;
				b->cptIteration++;
			}
			distance += l.m_rl;
			if (a.m_x.distance(b->m_x) > distance)
				b->m_x = a.m_x + (b->m_x - a.m_x).normalized() * distance;
		}
	}
	else
	{
		btScalar distance = 0;
		Node& a = m_nodes[m_nodes.size() - 1];
		bool aMove = a.computeNodeConstraint;
		for (int i = 0; i < m_anchors.size(); ++i)
			if (a.index == m_anchors[i].m_node->index)
				a.m_x = m_anchors[i].m_c1 + m_anchors[i].m_body->getCenterOfMassPosition();

		for (int i = m_links.size() - 1; i >= 0; --i)
		{
			Link& l = m_links[i];
			Node* b = l.m_n[0];
			if (!aMove && !b->computeNodeConstraint) continue;
			if (!b->computeNodeConstraint)
			{
				b->computeNodeConstraint = true;
				b->cptIteration++;
			}
			distance += l.m_rl;
			if (a.m_x.distance(b->m_x) > distance)
				b->m_x = a.m_x + (b->m_x - a.m_x).normalized() * distance;
		}
	}
}

void btCable::LRAHierachique()
{
	int level = 2;
	int size = m_nodes.size();
	// iteration on node

	if (invertLRA)
	{
		for (int i = size - 1; i >= 0; i--)
		{
			int dist = 2;
			for (int levelTemp = 1; levelTemp <= level; levelTemp++)
			{
				int delta = dist / 2;
				// We check if the node exist
				if (i - delta >= 0 && i + delta < size)
				{
					distanceHierachy(i + delta, i - delta);
				}
				dist *= 2;
			}
		}
	}
	else
	{
		for (int i = 0; i < size; i++)
		{
			int dist = 2;
			for (int levelTemp = 1; levelTemp <= level; levelTemp++)
			{
				int delta = dist / 2;
				// We check if the node exist
				if (i - delta >= 0 && i + delta < size)
				{
					distanceHierachy(i - delta, i + delta);
				}
				dist *= 2;
			}
		}
	}
}

void btCable::distanceHierachy(int indexMain, int indexCheck)
{
	Node* main = &m_nodes.at(indexMain);
	Node* altNode = &m_nodes.at(indexCheck);

	if (!main->computeNodeConstraint && !altNode->computeNodeConstraint)
		return;

	if (!main->computeNodeConstraint)
	{
		main->computeNodeConstraint = true;
		main->cptIteration++;
	}
	if (!altNode->computeNodeConstraint)
	{
		altNode->computeNodeConstraint = true;
		altNode->cptIteration++;
	}

	btVector3 deltaPos = altNode->m_x - main->m_x;
	int iterator = 1;
	int LinkIndexDelta = 0;
	if (indexMain > indexCheck)
	{
		LinkIndexDelta = -1;
		iterator = -1;
	}
	btScalar maxDist = 0;

	for (int i = indexMain; i != indexCheck; i += iterator)
	{
		Link l = m_links.at(i + LinkIndexDelta);
		maxDist += l.m_rl;
	}
	btScalar dist = deltaPos.length();
	if (dist > maxDist)
	{
		deltaPos.normalize();

		btScalar k = m_materials[0]->m_kLST;

		main->m_x += (0.5 * (dist - maxDist) * deltaPos);
		altNode->m_x -= (0.5 * (dist - maxDist) * deltaPos);
	}
}

void btCable::bendingConstraint()
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

		if (!before->computeNodeConstraint && !current->computeNodeConstraint && !after->computeNodeConstraint)
			continue;

		before->computeNodeConstraint = true;
		current->computeNodeConstraint = true;
		after->computeNodeConstraint = true;

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
			btScalar rr = r.length2();
			btScalar d2 = btDot(delta2, r);
			btScalar d1 = btDot(delta1, r);
			btClamp(d2, (btScalar)0.0, rr);
			btClamp(d1, (btScalar)0.0, rr);
			btScalar alpha1 = d2 / rr;
			btScalar alpha2 = d1 / rr;
			btVector3 p = (before->m_x + after->m_x) * 0.5f;
			btVector3 d = p - current->m_x;
			btScalar dLen = d.length();

			if (btFuzzyZero(dLen))
			{
				continue;
			}

			btVector3 dNorm = d / dLen;
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

#pragma region Contact Constraint

void btCable::contactConstraint()
{
	int nbContactPairPotential = _nodePairContact.size();
	if (nbContactPairPotential == 0) return;

	for (int i = 0; i < nbContactPairPotential; ++i)
	{
		NodePairNarrowPhase* pair = &_nodePairContact.at(i);
		btCollisionObject* obj = pair->pair->body;
		btRigidBody* rb = btRigidBody::upcast(obj);

		// Get the top rigidBody which contains the mass of the object (otherwise there will be no impulse)
		while (rb->m_redirectionTarget)
		{
			rb = rb->m_redirectionTarget;
		}

		int nIdx = pair->node->index;
		Node* node = &m_nodes[nIdx];

		//node->m_x = pair->hitPoint; // debug, causes nodes to stick to their collision point

		btVector3 n = pair->normal;
		btVector3 x = node->m_x;
		btVector3 p = pair->hitPoint;

		// Signed distance of the node center to the contact plane (positive = outside, negative = penetrating)
		btScalar signedToPlane = (x - p).dot(n);
		btScalar penetration = -signedToPlane;

		if (penetration > 0.0)
		{
			pair->hitPoint = x + n * penetration;

			// Create a SphereShape to simulate the collision between the node and a rigidbody
			_nodeContactTransform.setOrigin(pair->hitPoint);
			_nodeContactObject.setWorldTransform(_nodeContactTransform);

			// Simulate the collision			
			btTransform currentRbTr(obj->getWorldTransform());
			obj->setWorldTransform(pair->worldTransform);
			MyContactResultCallback result(0, &_nodeContactObject, obj);
			m_world->contactPairTest(&_nodeContactObject, obj, result);
			if (result.m_connected)
			{
				// Update the pair for the next step
				pair->hitPoint = result.contactPoint + result.contactNorm * (m_collisionMargin + FLT_EPSILON);
				pair->normal = result.contactNorm;
			}
			obj->setWorldTransform(currentRbTr);

			if (rb && impulseCompute)
			{
				btVector3 imp = calculateBodyImpulse(rb, node, pair->normal, pair->hitPoint);
				rb->applyRedirectionImpulse(imp, pair->hitPoint);
			}

			// Update the node position at last
			node->m_x = pair->hitPoint;
		}
	}
}

btScalar btCable::computeCollisionMargin(const btCollisionShape* shape) const
{
	return m_collisionMargin + shape->getMargin();
}

btVector3 btCable::calculateBodyImpulse(btRigidBody* obj, Node* n, btVector3 normal, btVector3 hitPosition)
{
	// a = node
	// b = body
	btTransform wtr = obj->getWorldTransform();

	btScalar ima = n->m_im;          // inverse mass of node
	btScalar imb = obj->getInvMass(); // inverse mass of body
	if (imb == 0) return btVector3{0, 0, 0};

	// bodyToNodeVector (relative to body COM)
	btVector3 rB = hitPosition - wtr.getOrigin();

	// Velocities at contact
	btVector3 vBody = obj->getVelocityInLocalPoint(rB);
	btVector3 vNode = (n->m_x - n->m_q) / m_sst.sdt;
	btVector3 vRelative = vNode - vBody;

	btScalar vRel_n = btDot(vRelative, normal);

	// Tangent direction (handle degenerate case)
	btVector3 vRel_t = vRelative - normal * vRel_n;
	btScalar vRel_t_len2 = vRel_t.length2();
	btVector3 tangentDir = (vRel_t_len2 > SIMD_EPSILON) ? (vRel_t / btSqrt(vRel_t_len2)) : btVector3{0, 0, 0};

	// Penetration along normal (node relative to hit point)
	btVector3 deltaPosNode = hitPosition - n->m_x;
	btScalar penetrationDistance = deltaPosNode.dot(normal);

	// Effective mass along a direction dir: K = ima + imb + dir · ((rB × dir) × (invInertB * (rB × dir)))
	btMatrix3x3 invInertB = obj->getInvInertiaTensorWorld();
	auto effectiveMass = [&](const btVector3& dir) -> btScalar {
		btVector3 rBxDir = rB.cross(dir);
		btVector3 Iinv_rBxDir = invInertB * rBxDir;
		btScalar ang = dir.dot(rBxDir.cross(Iinv_rBxDir));
		return ima + imb + ang;
	};

	// Friction coefficient
	btScalar mu = obj->getFriction() * getFriction();

	btScalar Kn = effectiveMass(normal);
	if (Kn <= SIMD_EPSILON) return btVector3{0, 0, 0};

	// Normal impulse (non-negative)
	btScalar jn = -vRel_n / Kn;
	jn = btMax(btScalar(0), jn);

	// Tangent impulse with Coulomb clamp
	btScalar jt = 0;
	if (tangentDir.fuzzyZero() == false)
	{
		btScalar Kt = effectiveMass(tangentDir);
		if (Kt > SIMD_EPSILON)
		{
			btScalar vRel_t_scalar = vRelative.dot(tangentDir);
			jt = -vRel_t_scalar / Kt;
			btScalar jt_max = mu * jn;
			btClamp(jt, -jt_max, jt_max);
		}
	}

	// Total impulse
	btVector3 impulse = -(normal * jn + tangentDir * jt) / m_sst.sdt;

	btScalar k = 1.0f;
	if (m_collisionMode == CollisionMode::Linear && penetrationDistance > penetrationMin)
	{
		btScalar distanceTot = penetrationMax - penetrationMin;
		btScalar ratio = (penetrationDistance - penetrationMin) / distanceTot;
		k = Lerp(this->collisionStiffnessMin, this->collisionStiffnessMax, min(1.0, ratio));		
	}
	else if (m_collisionMode == CollisionMode::Curve && spline)
	{
		k = spline->eval(penetrationDistance);

		if (isnan(k))
			k = 1.0f;		
	}

	n->m_SplineEval = k;	
	impulse *= k; // Apply coefficient
	return impulse;
}

void btCable::distanceConstraintLock(int limMin, int limMax)
{
	Link* l;
	Node* a;
	Node* b;
	for (int i = limMin; i < limMax - 1; ++i)
	{
		l = &m_links[i];
		a = l->m_n[0];
		b = l->m_n[1];

		btVector3 AB = b->m_x - a->m_x;
		if (AB.fuzzyZero())
		{
			continue;
		}
		btVector3 ABNormalized = AB.normalized();

		btScalar normAB = AB.length();
		btScalar k = m_materials[0]->m_kLST;

		btScalar sumInvMass = a->m_im + b->m_im;
		if (sumInvMass >= SIMD_EPSILON)
		{
			btVector3 denom = 1 / sumInvMass * (normAB - l->m_rl) * ABNormalized;
			if (i != limMin) a->m_x += (a->m_im * denom) * k;
			if (i != limMax - 2) b->m_x -= (b->m_im * denom) * k;
		}
	}
}

#pragma endregion

#pragma endregion

#pragma region Getter / Setter

void btCable::setMaxTension(btScalar maxTension)
{
	m_maxTension = maxTension;
}

void btCable::setCollisionMode(int mode)
{
	m_collisionMode = (CollisionMode)mode;
}

btScalar btCable::getRestLength()
{
	btScalar length = 0;
	for (int i = 0; i < m_links.size(); ++i)
		length += m_links[i].m_rl;
	return length;
}

btScalar btCable::getLength()
{
	btScalar length = 0;
	for (int i = 0; i < m_links.size(); ++i)
		length += m_links[i].m_n[0]->m_x.distance(m_links[i].m_n[1]->m_x);
	return length;
}

btVector3 btCable::getTensionAt(int index)
{
	btVector3 tension = btVector3(0, 0, 0);
	if (index < m_anchors.size() && index >= 0)
	{
		tension = m_anchors[index].m_totalTension / (btScalar) m_world->GetSubIteration();
	}

	return tension;
}

btVector3 btCable::getLocalAnchorWithNode(int indexNode)
{
	for (int idxAnchor = 0; idxAnchor < m_anchors.size(); ++idxAnchor)
	{
		if (m_anchors[idxAnchor].m_node->index == indexNode)
		{
			return m_anchors[idxAnchor].m_local;
		}
	}

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

void btCable::setInvertLRA(bool invert)
{
	invertLRA = invert;
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

void btCable::addSection(btScalar rl, int start, int end, int nbNodes)
{
	m_section[m_sectionCurrent].RestLength = rl;
	m_section[m_sectionCurrent].StartNodeIndex = start;
	m_section[m_sectionCurrent].EndNodeIndex = end;
	m_section[m_sectionCurrent].NumberOfNodes = nbNodes;
	m_sectionCurrent++;
}

void* btCable::getCableNodesPos()
{
	return m_nodePos;
}

int btCable::getCableState()
{
	return (int)cableState;
}

int btCable::getCollisionMode()
{
	return (int)m_collisionMode;
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
	InitializeNode(&n, x, m);

	n.m_material = m_materials[0];

	for (int i = m_nodes.size() - 3; i < m_nodes.size(); i++)
	{
		// Update NodePos
		m_nodePos[i].x = m_nodes[i].m_x.getX();
		m_nodePos[i].y = m_nodes[i].m_x.getY();
		m_nodePos[i].z = m_nodes[i].m_x.getZ();
	}
	n.index = m_nodes.size() - 1;
}

void btCable::removeNodeAt(const int index)
{
	if (index < m_nodes.size())
	{
		delete[] m_nodes[index].m_movingAverage;
		m_nodes[index].index = -1;
		m_nodes[m_nodes.size() - 1].index = index;

		m_nodes.removeAtIndex(index);
	}
}

void btCable::setTotalMass(btScalar mass, bool fromfaces)
{
	btScalar massNode = mass / m_nodes.size();
	for (int i = 0; i < m_nodes.size(); ++i)
	{
		m_nodes[i].m_im = 1.0 / massNode;
	}
}

void btCable::updateNodesMass()
{
	// Reset all masses
	for (int i = 0; i < m_nodes.size(); ++i)
	{
		Node* node0 = &m_nodes[i];
		node0->m_im = 0;
	}

	// Set all masses
	for (int i = 0; i < m_links.size(); ++i)
	{
		btScalar mass0 = 0;
		btScalar mass1 = 0;

		Node* node0 = &m_nodes[i];
		Node* node1 = &m_nodes[i+1];

		if (node0->m_im > 0)
		{
			mass0 = 1.0f / node0->m_im;
		}

		if (node1->m_im > 0)
		{
			mass1 = 1.0f / node1->m_im;
		}

		btScalar currentHalfMass = (0.5 * m_linearMass * m_links[i].m_rl);

		mass0 += currentHalfMass;
		mass1 += currentHalfMass;

		node0->m_im = 1.0f / mass0;
		node1->m_im = 1.0f / mass1;
	}
	m_bUpdateRtCst = true;
}

void btCable::setCollisionParameters(int substepSolverCollisionDelay, int substepNarrowCollisionDelay)
{
	m_substepDelayCollisionSolver = substepSolverCollisionDelay;
	m_substepDelayCollisionNarrow = substepNarrowCollisionDelay;
}

void btCable::setCollisionMargin(float colMargin)
{
	this->m_collisionMargin = colMargin;

	_nodeContactSphere.setUnscaledRadius(colMargin);
	_nodeContactObject.setCollisionShape(&_nodeContactSphere);
}

float btCable::getCollisionMargin()
{
	return this->m_collisionMargin;
}

btScalar btCable::getLinkRestLength(int indexLink)
{
	// If no section set
	if (m_sectionCount < 1)
	{
		return m_defaultRestLength;
	}

	// Check if the node is in a current section
	for (int i = 0; i < m_sectionCount; i++)
	{
		if (indexLink < m_section[i].EndNodeIndex)
		{
			return m_section[i].RestLength;
		}
	}
	// If the node isn't in a section we use the last section restLength
	return m_section[m_sectionCount - 1].RestLength;
}

void btCable::setDefaultRestLength(btScalar rl)
{
	m_defaultRestLength = rl;
}

void btCable::setMinLength(btScalar value)
{
	m_minLength = value;
}

void btCable::setWantedGrowSpeedAndDistance(btScalar speed, btScalar distance)
{
	WantedDistance = distance;
	WantedSpeed = speed;
}

void btCable::setLinearMass(btScalar mass)
{
	m_linearMass = mass;
}

void btCable::setCollisionStiffness(btScalar stiffnessMin, btScalar stiffnessMax, btScalar distMin, btScalar distMax)
{
	this->collisionStiffnessMin = stiffnessMin;
	this->collisionStiffnessMax = stiffnessMax;
	this->penetrationMin = distMin;
	this->penetrationMax = distMax;
}

void btCable::setCollisionResponseActive(bool active)
{
	this->impulseCompute = active;
}

int btCable::getGrowingState()
{
	return m_growingState;
}

void btCable::setUseAnchorConstraintPlacement(bool status)
{
	m_useAnchorConstraintPlacement = status;
}

void btCable::setDistanceMode(int mode)
{
	m_distanceMode = (DistanceMode)mode;
	switch(m_distanceMode)
	{
		case DistanceMode::Bullet:
			m_distanceFunction = &btCable::distanceConstraintBullet;
			break;
		case DistanceMode::XPBD:
			m_distanceFunction = &btCable::distanceConstraintXPBD;
			break;
	}
}

int btCable::getDistanceMode()
{
	return (int)m_distanceMode;
}

#pragma endregion
