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
#include <BulletCollision/NarrowPhaseCollision/btRaycastCallback.h>

btCable::btCable(btSoftBodyWorldInfo* worldInfo, btCollisionWorld* world, int node_count, int section_count, const btVector3* x, const btScalar* m) : btSoftBody(worldInfo, node_count, x, m)
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
		m_defaultRestLength = m_links.at(0).m_rl;
	}

	m_gravity = worldInfo->m_gravity;
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

	// Prepare nodes
	for (i = 0, ni = m_nodes.size(); i < ni; ++i)
	{
		Node& node = m_nodes[i];
		node.cptIteration = 0;
		node.computeNodeConstraint = true;
		node.m_splitv = btVector3(0, 0, 0);
		node.m_nbCollidingObjectPotential = 0;

		// Reset drawings
		node.m_xOut = btVector3(FLT_MAX, FLT_MAX, FLT_MAX);
		node.m_xStartOut = btVector3(FLT_MAX, FLT_MAX, FLT_MAX);
		node.m_xOutNormal = btVector3(FLT_MAX, FLT_MAX, FLT_MAX);
		node.m_xOutMargin = FLT_MAX;
		node.m_xStartRay = btVector3(FLT_MAX, FLT_MAX, FLT_MAX);
		node.m_xEndRay = btVector3(FLT_MAX, FLT_MAX, FLT_MAX);
	}

	// Prepare links
	for (i = 0, ni = m_links.size(); i < ni; ++i)
	{
		Link& l = m_links[i];
		l.m_c3 = l.m_n[1]->m_q - l.m_n[0]->m_q;
		l.m_c2 = 1.0 / (l.m_c3.length2() * l.m_c0);
	}

	// Prepare anchors
	for (i = 0, ni = this->m_anchors.size(); i < ni; ++i)
	{
		Anchor& a = this->m_anchors[i];
		const btVector3 ra = a.m_body->getWorldTransform().getBasis() * a.m_local;

		const double invMassBody = a.m_body->getInvMass();
		const double invMassNode = a.m_node->m_im;
		const auto& invInertiaTensorWorld = a.m_body->getInvInertiaTensorWorld();

		// Compute the real impulse matrix to be able to later compute the cable tension
		a.m_c0 = ImpulseMatrix(m_sst.sdt,
							   invMassNode,
							   invMassBody,
							   invInertiaTensorWorld,
							   ra);

		// Compute a tweaked impulse matrix used to stabilized distance body / anchor
		const double nodeMass = (1.0 / invMassNode);
		a.impacted = false;

		const double tweakedMass = nodeMass + a.m_body->getMass() * a.BodyMassRatio * (1.0 / a.m_body->m_anchorsCount);
		a.m_c0_massBalance = ImpulseMatrix(m_sst.sdt,
										   1.0 / tweakedMass,
										   invMassBody,
										   invInertiaTensorWorld,
										   ra);

		a.m_c1 = ra;
		a.m_c2 = m_sst.sdt * a.m_node->m_im;
		a.m_body->activate();
		a.tension = btVector3(0, 0, 0);
	}
	_impacted = false;

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
	bool runContactConstraint  = (currentIter + 1) % m_substepDelayCollisionSolver == 0 || lastStep;

	updateNodeDeltaPos(currentIter);

	anchorConstraint(_impacted);

	distanceConstraint();

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
	if (_impacted)
	{
		anchorConstraint(_impacted);
	}

	for (int i = 0; i < m_anchors.size(); ++i)
	{
		Anchor& a = this->m_anchors[i];
		if (a.m_body->canChangedMassAtImpact() && !a.m_body->isStaticOrKinematicObject())
		{
			if (a.impacted)
			{
				btScalar limit = a.m_body->getUpperLimitDistanceImpact() - a.m_body->getLowerLimitDistanceImpact();
				btScalar ratio = (a.m_dist - a.m_body->getLowerLimitDistanceImpact()) / limit;
				btScalar func = 1.0 - pow(max(0.0, abs(ratio - 1.0) * 1.1 - 0.1), 3);
				btScalar clampRatio = Clamp(func, 0.0, 1.0);
				btScalar newMass = Lerp(a.m_body->getLowerLimitMassImpact(), a.m_body->getUpperLimitMassImpact(), clampRatio);
				a.m_body->setMassProps(newMass, newMass * a.m_body->getLocalInertia() * a.m_body->getInvMass());
				a.m_body->setGravity(m_worldInfo->m_gravity * (a.m_body->getLowerLimitMassImpact() / newMass));
			}
			else
			{
				a.m_body->setMassProps(a.m_body->getLowerLimitMassImpact(), a.m_body->getLowerLimitMassImpact() * a.m_body->getLocalInertia() * a.m_body->getInvMass());
				a.m_body->setGravity(m_worldInfo->m_gravity);
			}
		}
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

	_nodePairContact.clear();
}

void btCable::updateNodeData()
{
	const btScalar frameDT = (1.0 / m_sst.fdt) * (1.0 - m_cfg.kDP);
	const btScalar subFrameDT = (1.0 / m_sst.sdt) * (1.0 - m_cfg.kDP);
	for (int i = 0; i < m_nodes.size(); ++i)
	{
		// Update velocities for the cable
		Node& n = m_nodes[i];
		n.m_vn = n.m_v;
		n.m_v = (n.m_x - n.m_q) * subFrameDT;

		// Only update data for last substep
		if (m_world->GetIndexSubIteration() == m_world->GetSubIteration() - 1)
		{
			btVector3 nodeVelocity = (n.m_x - n.m_xn) * frameDT;

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

			n.m_xn = n.m_x;              // Update previous pos with current
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

	// Forces
	// if (useGravity) addVelocity(m_gravity * m_sst.sdt);

	// SoftRigidBody
	NodeForces* nodeForces = ((btSoftRigidDynamicsWorld*) m_world)->m_nodeForces;
	btVector3 nodeForceToApply = btVector3();
	for (i = 0, ni = m_nodes.size(); i < ni; ++i)
	{
		Node& n = m_nodes[i];
		n.m_q = n.m_x;

		float addedMass = 0.0f;
		if (isActive())
		{
			// Get the Hydro and Aero forces
			NodeForces currentNodeForces = nodeForces[m_cableData->startIndex + i];

			// Integrate once (first sub step)
			if (m_world->GetIndexSubIteration() == 0)
			{
				// Add gravity force
				if (useGravity)
				{
					n.m_f += m_gravity / n.m_im;
				}

				// Integrate forces only on first iteration
				if (useHydroAero)
				{
					// We check if currentNodeForces are correct, if not, then we dont apply these forces.
					if (std::isinf(currentNodeForces.x) || std::isnan(currentNodeForces.x) || std::isinf(currentNodeForces.y) || std::isnan(currentNodeForces.y) || std::isinf(currentNodeForces.z) || std::isnan(currentNodeForces.z))
					{
						cableState = InternalForcesError;
					}
					else
					{
						n.m_f.setValue(n.m_f.getX() + currentNodeForces.x, n.m_f.getY() + currentNodeForces.y, n.m_f.getZ() + currentNodeForces.z);
					}
				}
			}

			// Integrate addedMass for each substep
			addedMass = currentNodeForces.ma;
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
	if (!m_world)
	{
		return;
	}

	int totalNumNodes = ((btSoftRigidDynamicsWorld*)m_world)->getTotalNumNodes();
	double rl = m_links.at(m_links.size() - 1).m_rl;
	double distance = dt * WantedSpeed + rl;
	int nodeSize = m_nodes.size();

	// If there is a target length we don t extend rl more than necessary
	if (WantedDistance > 0)
	{
		btScalar value = WantedDistance - getRestLength();
		if (value > FLT_EPSILON && value < dt * WantedSpeed)
		{
			distance = value + rl;
			WantedSpeed = 0;
		}
	}

	// base restLength on the link
	double linkRestLength = getLinkRestLength(m_links.size() - 1);

	// Check if we had to had a node
	if (distance > linkRestLength * 2)
	{
		// Node number limits
		if (totalNumNodes >= m_worldInfo->maxNodeNumber || nodeSize >= m_worldInfo->maxNodeNumberPerCable)
		{
			m_growingState = 4;
			return;
		}
	}

	m_links.at(m_links.size() - 1).m_rl = distance;
	m_links.at(m_links.size() - 1).m_c1 = distance * distance;

	btScalar firstNodeMass = m_linearMass * 0.5f * distance;

	// Update mass
	setMass(nodeSize - 1, firstNodeMass);
	if (nodeSize > 2)
	{
		btScalar linkMass = firstNodeMass + m_linearMass * 0.5f * (m_links[m_links.size() - 2].m_rl);
		setMass(nodeSize - 2, linkMass);
	}
	else
	{
		setMass(nodeSize - 2, firstNodeMass);
	}

	// Case when we need to add at least 1 node
	while (distance > linkRestLength * 2)
	{
		if (totalNumNodes >= m_worldInfo->maxNodeNumber || nodeSize >= m_worldInfo->maxNodeNumberPerCable) {
			m_growingState = 4;
			return;
		}

		
		Node* node0 = &m_nodes[nodeSize - 1];
		Node* node1 = &m_nodes[nodeSize - 2];

		btVector3 dir = node0->m_x - node1->m_x;
		dir.normalize();
		dir *= linkRestLength;

		// Save last node position (will be new node's position)
		btVector3 positionLastNode = node0->m_x;

		// Set the node at the right place
		btVector3 positionPreviousNode = node1->m_x + dir;
		ResetNodePosition(nodeSize - 1, positionPreviousNode);

		node0->m_battach = 0;

		// Create the new node
		this->appendNode(positionLastNode, 1 / node0->m_im);
		nodeSize++;

		// Set the velocity
		Node* newNode = &m_nodes[nodeSize - 1];
		newNode->m_v = m_nodes.at(nodeSize - 2).m_v;
		m_nodes.at(nodeSize - 2).m_v = (m_nodes.at(nodeSize - 3).m_v + m_nodes.at(nodeSize - 2).m_v) * 0.5;
		appendLink(m_nodes.size() - 2, m_nodes.size() - 1, m_materials[0]);

		// Split rest length onto the 2 last links
		m_links[m_links.size() - 2].m_rl = linkRestLength;
		m_links[m_links.size() - 2].m_c1 = linkRestLength * linkRestLength;

		m_links[m_links.size() - 1].m_rl = distance - linkRestLength;
		m_links[m_links.size() - 1].m_c1 = (distance - linkRestLength) * (distance - linkRestLength);

		// Swap anchor if needed
		for (int i = 0; i < m_anchors.size(); i++)
		{
			Anchor* a = &m_anchors.at(i);
			if (a->m_node != nullptr)
			{
				// If its the anchor on the previous last node
				if (a->m_node->index == m_nodes.size() - 2)
				{
					newNode->m_battach = -1;
					a->m_node = &m_nodes.at(m_nodes.size() - 1);
				}
			}
		}
		distance -= linkRestLength;

		// Update Mass
		btScalar firstNodeMass = m_linearMass * 0.5f * distance;
		setMass(nodeSize - 1, firstNodeMass);
		btScalar LinkMassWithRl = 0.5 * m_links[m_links.size() - 2].m_rl * m_linearMass;
		setMass(nodeSize - 2, LinkMassWithRl + firstNodeMass);
		// If there is only 2 links
		if (nodeSize == 3)
		{
			setMass(nodeSize - 3, LinkMassWithRl);
		}
		// Normal case
		else
		{
			btScalar LinkMassBefore = 0.5 * m_links[m_links.size() - 3].m_rl * m_linearMass;
			setMass(nodeSize - 3, LinkMassWithRl + LinkMassBefore);
		}
	}
	m_growingState = 1;
}

void btCable::Shrinks(float dt)
{
	int linkSize = m_links.size();
	int nodesSize = m_nodes.size();

	double rl = m_links.at(linkSize - 1).m_rl;
	double distance = dt * WantedSpeed + m_links.at(linkSize - 1).m_rl;

	// To avoid shrink to much
	btScalar totalRl = getRestLength();
	if (totalRl + distance <= m_minLength)
	{
		m_growingState = 2;
		return;
	}

	// We can t shrinks over an anchor
	if (m_nodes.at(nodesSize - 2).m_battach != 0)
	{
		// Minimum shrink lenght
		if (distance < m_minLength)
		{
			m_links.at(linkSize - 1).m_rl = m_minLength;
			m_links.at(linkSize - 1).m_c1 = m_minLength * m_minLength;

			m_growingState = 3;
			return;
		}
	}

	// set the Rest Length and set the mass
	m_links.at(linkSize - 1).m_rl = distance;
	m_links.at(linkSize - 1).m_c1 = distance * distance;
	btScalar linkRestLength = getLinkRestLength(linkSize - 1);
	btScalar firstNodeMass = m_linearMass * 0.5f * distance;
	setMass(nodesSize - 1, firstNodeMass);
	if (nodesSize > 2)
	{
		btScalar linkMass = firstNodeMass + m_linearMass * 0.5f * (m_links[m_links.size() - 2].m_rl);
		setMass(nodesSize - 2, linkMass);
	}
	else
	{
		setMass(nodesSize - 2, firstNodeMass);
	}

	// if we had to delete a node
	while (distance < abs(dt * WantedSpeed))
	{
		btVector3 nodePos = m_nodes.at(nodesSize - 1).m_x;
		btVector3 nodeVel = m_nodes.at(nodesSize - 1).m_v;

		if (nodesSize > 2)
		{
			// Remove the last node and the last link
			m_links.removeAtIndex(linkSize - 1);
			removeNodeAt(nodesSize - 1);
			nodesSize--;
			linkSize--;
		}

		int indexNode = nodesSize - 1;

		for (int i = 0; i < m_anchors.size(); i++)
		{
			Anchor* a = &m_anchors.at(i);
			// If the node deleted was an anchor
			if (a->m_node->index == nodesSize)
			{
				a->m_node = &m_nodes.at(indexNode);
				a->m_node->m_x = nodePos;
				a->m_node->m_v = nodeVel;
				a->m_node->m_battach = -1;
			}
		}

		btScalar dist = (nodePos - m_nodes.at(nodesSize - 2).m_x).length();

		// Set the new restLength and mass
		m_links.at(linkSize - 1).m_rl = dist;
		m_links.at(linkSize - 1).m_c1 = dist * dist;

		firstNodeMass = m_linearMass * 0.5f * dist;
		setMass(nodesSize - 1, firstNodeMass);

		if (nodesSize > 2)
		{
			btScalar linkMass = firstNodeMass + m_linearMass * 0.5f * (m_links[linkSize - 2].m_rl);
			setMass(nodesSize - 2, linkMass);
		}
		else
		{
			setMass(nodesSize - 2, firstNodeMass);
		}

		distance += linkRestLength;
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

void btCable::collectPotentials(btCollisionObjectArray &collisionObjectArray, std::vector<btCollisionObject*>& out) const 
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
		out.emplace_back(ObjData { co, mi, ma, velB });
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
		if ( m_bounds[0].x() <= od.maxAabb.x() && m_bounds[1].x() >= od.minAabb.x() &&
		     m_bounds[0].y() <= od.maxAabb.y() && m_bounds[1].y() >= od.minAabb.y() &&
		     m_bounds[0].z() <= od.maxAabb.z() && m_bounds[1].z() >= od.minAabb.z() )
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

	// Notify the dispatcher to pass into threaded mode
	btCollisionDispatcherMt* disp = static_cast<btCollisionDispatcherMt*>(m_world->getDispatcher());
	disp->m_batchUpdating = true;
	
	#pragma omp parallel for schedule(static, 1)
	for (int i = 0; i < nodeCount; ++i)
	{
		int tid = omp_get_thread_num();
		Node* n = &m_nodes[i];
		n->m_xOut = btVector3(0, 0, 0);

		// static node box (no margin first)
		btVector3 lo, hi;
		setNodeBoundingBox(n->m_x, n->m_q, 0, &lo, &hi);
		btVector3 nodeVel = (n->m_x - n->m_q) / m_sst.sdt;
		btVector3 zero = btVector3(0, 0, 0);

		// test each object
		for (const auto& od : objData)
		{
			if (aabbTestMargin(nodeVel, od.objVelocity, lo, hi, od.minAabb, od.maxAabb)) continue;
		
			BroadPhasePair bp;
			bp.node = n;
			bp.minLink = lo;
			bp.maxLink = hi;
			bp.body = od.obj;
			bp.bodyType = od.obj->getInternalType();
			threadBuckets[tid].push_back(bp);		
		}
	}

	// Notify the dispatcher that we don't use threads anymore
	disp->m_batchUpdating = false;

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
    
    // Reusable collision objects
    btSphereShape nodeCollisionShape(m_collisionMargin);
    btCollisionObject nodeCollisionObject;

	for (int i = 0; i < _candidates.size(); i++ )
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
		btScalar toi = 1.0f; // Time of impact
		btScalar penetration = 0.0f;

		btVector3 rayStart = prevPos;

		btVector3 rayEnd;
		btTransform rbTransformAtTime;

		//Single sweep in relative motion
		// Relative-motion single sweep against rb fixed at rbPrevTransform
		btVector3 nodeStart = prevPos;
		btVector3 nodeEnd   = currentPos;
  
		// Compute the motion of the rigid body's material point that coincides with nodeStart at t0
		btVector3 pLocal      = rbPrevTransform.inverse() * nodeStart;
		btVector3 rbPointEnd  = rbTransform * pLocal;
		btVector3 deltaNode   = nodeEnd - nodeStart;
		btVector3 deltaRbPt   = rbPointEnd - nodeStart;
		btVector3 relEnd      = nodeStart + (deltaNode - deltaRbPt);
  
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
        callback.m_collisionFilterMask  = rb->getBroadphaseHandle()->m_collisionFilterGroup;
  
        btTransform rbStatic = rbPrevTransform;
  
        btCollisionWorld::objectQuerySingle(
            &nodeCollisionShape, fromTransform, toTransform,
            rb, rb->getCollisionShape(), rbStatic,
            callback, btScalar(0.0f)
        );
  
        if (callback.hasHit())
        {
        	foundCollision = true;
        	fromTransform.setOrigin(callback.m_hitPointWorld + normalContact * (m_collisionMargin + FLT_EPSILON));
        	toTransform.setOrigin(callback.m_hitPointWorld + normalContact * (m_collisionMargin + FLT_EPSILON));
	  
        	btCollisionWorld::ClosestConvexResultCallback second(fromTransform.getOrigin(), toTransform.getOrigin());
        	callback.m_collisionFilterGroup = rb->getBroadphaseHandle()->m_collisionFilterMask;
        	callback.m_collisionFilterMask  = rb->getBroadphaseHandle()->m_collisionFilterGroup;
        	
        	btCollisionWorld::objectQuerySingle(
				&nodeCollisionShape, fromTransform, toTransform,
				rb, rb->getCollisionShape(), rbStatic,
				second, 0.0f
			);
	  
        	if (second.hasHit())
        	{
        		callback = second;
        	}
  
        	// Build rb transform at TOI (interpolate pose)
        	btVector3 startPos = rbPrevTransform.getOrigin();
        	btVector3 endPos   = rbTransform.getOrigin();
        	rbTransformAtTime.setOrigin(lerp(startPos, endPos, toi));

        	btQuaternion startRot = rbPrevTransform.getRotation();
        	btQuaternion endRot   = rbTransform.getRotation();
        	btQuaternion rot      = slerp(startRot, endRot, toi);
        	rbTransformAtTime.setRotation(rot);

        	// Convert hit point and normal from rbPrevTransform world frame -> rb local
        	btVector3 hitWorld_prev = callback.m_hitPointWorld;
        	btVector3 nWorld_prev   = callback.m_hitNormalWorld.normalized();

        	btTransform rbStatic = rbPrevTransform;
        	btVector3 pLocal = rbStatic.inverse() * hitWorld_prev;

        	// For pure rotation/rigid transform, local normal = R^T * n
        	btMatrix3x3 Rprev = rbStatic.getBasis();
        	btVector3 nLocal = (Rprev.transpose() * nWorld_prev).normalized();

        	// Transform to world at time-of-impact
        	btVector3 hitWorld_t = rbTransformAtTime * pLocal;
        	btMatrix3x3 Rtoi = rbTransformAtTime.getBasis();
        	btVector3 nWorld_t = (Rtoi * nLocal).normalized();

        	// Store contact data at time-of-impact; push out by node margin
        	normalContact = nWorld_t;
        	hitContact    = hitWorld_t + nWorld_t * (m_collisionMargin + FLT_EPSILON);

        	// Conservative remaining-trajectory penetration estimate along the normal
        	btVector3 centerAtHitRel = nodeStart.lerp(relEnd, callback.m_closestHitFraction);
        	penetration = (relEnd - centerAtHitRel).dot(nWorld_t);
        }
        
        if (!foundCollision) 
        {
            continue; // Skip to next candidate if no collision found
        }

		// Debug
		node->m_xOut = hitContact;
		node->m_xOutMargin = margin;
		node->m_xStartOut = prevPos;
		node->m_xStartRay = rayStart;
		node->m_xEndRay = rayEnd;

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

void btCable::anchorConstraint(bool& impact)
{
	BT_PROFILE("PSolve_Anchors");
	const btScalar kAHR = m_cfg.kAHR;
	const btScalar dt = m_sst.sdt;
	for (int i = 0, ni = this->m_anchors.size(); i < ni; ++i)
	{
		Anchor& a = this->m_anchors[i];
		Node& n = *a.m_node;

		const btVector3 wa = a.m_body->getCenterOfMassPosition() + a.m_c1;
		const btVector3 va = a.m_body->getVelocityInLocalPoint(a.m_c1) * dt;
		const btVector3 vb = n.m_x - n.m_q;
		const btVector3 vr = (va - vb) + (wa - n.m_x) * kAHR;

		const btVector3 vectAnchorNode = (wa - n.m_x);
		const btScalar distAnchorNode = vectAnchorNode.length();
		btScalar ratio = distAnchorNode / 0.1;
		ratio = Clamp(ratio, 0.0, 1.0);

		if (a.m_body->canChangedMassAtImpact() && !a.m_body->isStaticOrKinematicObject())
		{
			// distance Anchor-Node
			if (wa.distance(n.m_x) > a.m_body->getLowerLimitDistanceImpact())
			{
				impact = true;
				a.impacted = true;
				a.m_dist = wa.distance(n.m_x);
			}
		}

		const btVector3 impulse = a.m_c0 * vr * a.m_influence;
		const btVector3 impulseMassBalance = a.m_c0_massBalance * vr * a.m_influence;
		btVector3 finalImpulse = lerp(impulse, impulseMassBalance, ratio);
		btScalar currentTension = a.tension.length();
		a.tension += finalImpulse / dt;
		btScalar finalTension = a.tension.length();
		if (m_maxTension >= 0 && finalTension >= m_maxTension)
		{
			a.tension = a.tension.normalized() * m_maxTension;
			finalImpulse *= (a.tension.length() - currentTension) / (finalTension - currentTension);
		}

		a.m_body->applyImpulse(-finalImpulse, a.m_c1);

		n.m_x = wa;
	}
}

void btCable::distanceConstraint()
{
	BT_PROFILE("PSolve_Links");

	Link* l;
	Node* a;
	Node* b;
	for (int i = 0; i < m_links.size(); ++i)
	{
		l = &m_links[i];
		a = l->m_n[0];
		b = l->m_n[1];
		if (!a->computeNodeConstraint && !b->computeNodeConstraint)
			continue;

		a->computeNodeConstraint = true;
		b->computeNodeConstraint = true;

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
			a->m_x += (a->m_im * denom) * k;
			b->m_x -= (b->m_im * denom) * k;
		}
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

		btScalar corr = penetration;

		if (rb && impulseCompute)
		{
			//@Unsure : Apply the response parameters on the reversed node impulse instead of calculating a new one
			btVector3 imp = calculateBodyImpulse(rb, m_collisionMargin, node, pair->normal, pair->hitPoint);
			rb->applyRedirectionImpulse(imp, pair->hitPoint);
		}

		node->m_x = x + n * corr;
		node->m_n = pair->normal;

		// Refresh hit positions for the next solver step
		pair->hitPoint = node->m_x;
		pair->normal = node->m_n;
	}
}

btScalar btCable::computeCollisionMargin(const btCollisionShape* shape) const
{
	return shape->getShapeType() == SPHERE_SHAPE_PROXYTYPE ? m_collisionMargin + 0.001 : m_collisionMargin + shape->getMargin();
}

btVector3 btCable::calculateBodyImpulse(btRigidBody* obj, btScalar margin, Node* n, btVector3 normal, btVector3 hitPosition)
{
	// a = node
	// b = body
	btScalar viscosityCoef = this->collisionViscosity;
	btScalar dt = this->m_sst.sdt;
	btTransform wtr = obj->getWorldTransform();

	btScalar ima = n->m_im;
	btScalar imb = obj->getInvMass();
	if (imb == 0) return btVector3(0, 0, 0);
	btScalar totalMass = ima + imb;

	// bodyToNodeVector
	btVector3 ra = hitPosition - wtr.getOrigin();
	btVector3 vBody = obj->getVelocityInLocalPoint(ra);
	btVector3 vNode = (n->m_x - n->m_q) / dt;
	btVector3 vRelative = vNode - vBody;
	btScalar vRelativeOnNormal = btDot(vRelative, normal);

	// Friction value
	btVector3 vRelativeTangent = vRelative - (normal * vRelativeOnNormal);
	btVector3 tangentDir = vRelativeTangent.normalized();
	
	int frictionCoef = obj->getFriction() * getFriction();
	
	// Part of vRelativeOnTangentDir
	btScalar jt = -vRelative.dot(tangentDir) * frictionCoef;
	jt = jt / totalMass;

	btVector3 deltaPosNode = hitPosition - n->m_x;
	btScalar penetrationDistance = deltaPosNode.dot(normal);
	if (collisionMode == CollisionMode::Linear)
	{
		btScalar k;

		if (penetrationDistance < penetrationMin)
		{
			return btVector3{0, 0, 0};
		}

		if (penetrationDistance > penetrationMax)
		{
			k = this->collisionStiffnessMax;
		}
		else
		{
			btScalar distanceTot = penetrationMax - penetrationMin;
			btScalar ratio = (penetrationDistance - penetrationMin) / distanceTot;
			k = Lerp(this->collisionStiffnessMin, this->collisionStiffnessMax, ratio);
		}

		btScalar responseVector = -k * penetrationDistance + viscosityCoef * vRelativeOnNormal;

		const btVector3 impulse = ((responseVector * normal) - (tangentDir * jt * m_sst.isdt)) * dt;
		return impulse;
	}

	if (collisionMode == CollisionMode::Curve)

	{
		if (!spline) return btVector3(0, 0, 0);

		penetrationDistance *= 1.0 / m_substepDelayCollisionSolver;
		btScalar k = spline->eval(penetrationDistance);
		n->m_SplineEval = k;
		if (isnan(k))
		{
			return btVector3(0, 0, 0);
		}
		btScalar responseVector = -k * penetrationDistance + viscosityCoef * vRelativeOnNormal;

		const btVector3 impulse = ((responseVector * normal) - (tangentDir * jt * m_sst.isdt)) * dt;
		return impulse * m_substepDelayCollisionSolver;
	}

	return btVector3(0, 0, 0);
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
	collisionMode = (CollisionMode)mode;
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

void btCable::setCollisionParameters(int substepSolverCollisionDelay, int substepNarrowCollisionDelay)
{
	m_substepDelayCollisionSolver = substepSolverCollisionDelay;
	m_substepDelayCollisionNarrow = substepNarrowCollisionDelay;
}

void btCable::setCollisionMargin(float colMargin)
{
	this->m_collisionMargin = colMargin;
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

void btCable::setCollisionViscosity(btScalar viscosity)
{
	this->collisionViscosity = viscosity;
}

void btCable::setCollisionResponseActive(bool active)
{
	this->impulseCompute = active;
}

int btCable::getGrowingState()
{
	return m_growingState;
}

#pragma endregion
