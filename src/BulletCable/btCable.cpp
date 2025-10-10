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
	m_nodes.reserve(m_worldInfo->maxNodeNumberPerCable);
	m_links.reserve(m_worldInfo->maxNodeNumberPerCable - 1);
	m_world = world;
	m_solverSubStep = worldInfo->numIteration;
	m_cpt = 0;

	// Initialize Data
	m_cableData = new CableData();
	m_nodePos = new NodePos[worldInfo->maxNodeNumberPerCable]();
	m_nodeData = new NodeData[worldInfo->maxNodeNumberPerCable]();

	initSecondaryPool(worldInfo->maxNodeNumberPerCable);

	for (int i = 0; i < this->m_nodes.size(); i++)
	{
		Node& node = m_nodes[i];
		node.m_battach = 0;
		node.index = i;
		node.m_xn = x[i];

		if (i != 0)
		{
			appendLink(i - 1, i);
			m_linkedList.addTail(&node);

			m_linkedListLinks.addTail(&m_links[i-1]);
			onLinkInserted(&m_links[i-1]);
		}
		else
		{
			m_linkedList.addHead(&node);
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

	// Cached objects used for contact solving
	_nodeContactSphere.setUnscaledRadius(m_collisionMargin);
	_nodeContactObject = btCollisionObject();
	_nodeContactTransform = btTransform::getIdentity();
	_nodeContactObject.setCollisionShape(&_nodeContactSphere);
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
		const auto& invInertiaTensorWorld = a.m_body->getInvInertiaTensorWorld();

		// Node's masses
		const double invMassNode = a.m_node->m_im;
		const double massNode = invMassNode < FLT_EPSILON ? 0.0 : 1.0 / invMassNode;

		// Body's masses
		const double invMassBody = a.m_body->getInvMass();
		const double massBody = a.m_body->getMass();

		// // Tweaked mass
		const double tweakedMass = massNode + massBody * a.BodyMassRatio;
		const double invTweakedMass = tweakedMass < FLT_EPSILON ? 0.0 : 1.0 / tweakedMass;

		// Masses' matrices
		a.m_c0 = ImpulseMatrix(m_sst.sdt, invMassNode, invMassBody, invInertiaTensorWorld, ra);
		a.m_c0_massBalance = ImpulseMatrix(m_sst.sdt, invTweakedMass, invMassBody, invInertiaTensorWorld, ra);
		a.m_c1 = ra;
		a.m_c2 = m_sst.sdt * invMassNode;
		a.m_c2_massBalance = m_sst.sdt * invTweakedMass;
		a.m_body->activate();
		a.tension = btVector3(0, 0, 0);
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
	runBroadPhase(m_nodes, _candidates);
}

void btCable::solveSingleCableIteration(int currentIter)
{
	bool lastStep = currentIter == m_cfg.piterations - 1;
	bool firstStep = currentIter == 0;
	bool runBending = (currentIter + 1) % 2 == 0 || lastStep;
	bool runCollisionDetection = firstStep || (currentIter + 1) % m_substepDelayCollisionNarrow == 0 || lastStep;
	bool runContactConstraint = (currentIter + 1) % m_substepDelayCollisionSolver == 0 || lastStep;
	bool shouldAddBackupNodes = (currentIter + 1) == m_substepDelayCollisionSolver;

	updateNodeDeltaPos(currentIter);

	anchorConstraint();

	restorePrimaryMasses();
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
		runNarrowPhase(_candidates, _nodePairContact);
	}

	if (runContactConstraint)
	{
		restoreChangedMasses();
		
		contactConstraint(_nodePairContact);
		secondaryNodesContact(_anchorBackupCandidates, false); // not applying position change on nodes (acts like a wall)
		
		updateBackupNodes();
		secondaryNodesContact(_secondPairContact, true); // applies position change on nodes (backups primary collisions)

		if (shouldAddBackupNodes)
		{
			if (collisionBackupEnabled)
			{
				addBackupNodes();
			}
			if (anchorBackupEnabled)
			{
				addAnchorBackup();
				runBroadPhase(m_anchorBackups, _anchorBackupCandidates);
			}
		}
	}
}

void btCable::EndConstraintsSolve()
{
	for (int i = 0; i < m_anchors.size(); ++i)
	{
		Anchor& anchor = this->m_anchors[i];
		btRigidBody* body = anchor.m_body;
		Node* node = anchor.m_node;

		bool isImpacted = body->isImpacted();
		bool canChangeMass = body->canChangedMassAtImpact();
		bool isStaticOrKinematic = body->isStaticOrKinematicObject();

		if (isStaticOrKinematic) continue;

		if (canChangeMass)
		{
			btScalar limit = body->getUpperLimitDistanceImpact() - body->getLowerLimitDistanceImpact();
			btScalar ratio = Clamp((anchor.m_dist - body->getLowerLimitDistanceImpact()) / limit, 0.0, 1.0);
			btScalar func = 1.0 - pow(1.0 - ratio, 3);									// easeOutCubic
			// btScalar func = sqrt(1.0 - pow(ratio - 1.0, 2.0));						// easeOutCirc
			// btScalar func = ratio * ratio;											// easeInQuad
			// btScalar func = ratio * ratio * ratio * ratio;							// easeInQuart
			btScalar newMass = Lerp(body->getLowerLimitMassImpact(), body->getUpperLimitMassImpact(), func);
			body->setMassProps(newMass, newMass * body->getLocalInertia() * body->getInvMass());
			body->updateInertiaTensor();
			body->setGravity(m_worldInfo->m_gravity * (body->getLowerLimitMassImpact() / newMass));
		}
		else if (isImpacted)
		{
			body->setMassProps(body->getLowerLimitMassImpact(), body->getLowerLimitMassImpact() * body->getLocalInertia() * body->getInvMass());
			body->updateInertiaTensor();
			body->changeImpacted(false);
			body->setGravity(m_worldInfo->m_gravity);
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

	removeAllBackupNodes();
	_nodePairContact.clear();
}

void btCable::rememberPrimariesMass()
{
	for (int i = 0; i < m_nodes.size(); ++i)
	{
		Node& n = m_nodes.at(i);
		SavedMass saved_mass;
		saved_mass.originalInvMass = n.m_im;
		saved_mass.changedInvMass = -1;
		saved_mass.hasChanged = false;
		m_massOverrides[&n] = saved_mass;
	}	
}

void btCable::restorePrimaryMasses()
{
	for (int i = 0; i < m_nodes.size(); ++i)
	{
		Node& n = m_nodes.at(i);
		SavedMass& saved_mass = m_massOverrides[&n];
		if (saved_mass.hasChanged)
		{
			n.m_im = saved_mass.originalInvMass;
		}
	}
}

void btCable::restoreChangedMasses()
{
	for (int i = 0; i < m_nodes.size(); ++i)
	{
		Node& n = m_nodes.at(i);
		SavedMass& saved_mass = m_massOverrides[&n];
		if (saved_mass.hasChanged)
		{
			n.m_im = saved_mass.changedInvMass;
		}
	}
}

void btCable::resetOverridesState()
{
	for (auto& kv : m_massOverrides) {
		kv.second.changedInvMass = -1;
		kv.second.hasChanged = false;
	}
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
	NodeForces* nodeForces = ((btSoftRigidDynamicsWorld*)m_world)->m_nodeForces;
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
		if (totalNumNodes >= m_worldInfo->maxNodeNumber || nodeSize >= m_worldInfo->maxNodeNumberPerCable)
		{
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

		m_linkedList.addTail(newNode);
		m_linkedListLinks.addTail(&m_links[m_links.size() - 1]);
		onLinkInserted(&m_links[m_links.size() - 1]);

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
	double distance = dt * WantedSpeed + rl;

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
			onLinkRemoved(m_linkedListLinks.getTail()->getValue());
			m_linkedListLinks.remove(m_linkedListLinks.getTail());
			m_links.removeAtIndex(linkSize - 1);

			m_linkedList.remove(m_linkedList.getTail());
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
void btCable::runBroadPhase(btAlignedObjectArray<Node>& nodesArray, btAlignedObjectArray<BroadPhasePair>& outCandidates)
{
	outCandidates.clear();
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

	// one bucket per thread
	btAlignedObjectArray<btAlignedObjectArray<BroadPhasePair>> threadBuckets;
	threadBuckets.resize(omp_get_max_threads());
	int nodeCount = nodesArray.size();

	#pragma omp parallel for schedule(static, 1)
	for (int i = 0; i < nodeCount; ++i)
	{
		int tid = omp_get_thread_num();
		Node* n = &nodesArray[i];

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
			outCandidates.push_back(bucket[j]);
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

void btCable::runNarrowPhase(btAlignedObjectArray<BroadPhasePair>& candidates, btAlignedObjectArray<NodePairNarrowPhase>& outPairContacts)
{
	//Clear & Pre-allocate outputs
	outPairContacts.clear();
	outPairContacts.reserve(outPairContacts.size() + candidates.size());

	if (candidates.size() <= 0)
	{
		return;
	}

	for (int i = 0; i < candidates.size(); i++)
	{
		BroadPhasePair* c = &candidates.at(i);
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

		outPairContacts.push_back(pair);
	}
}

btSoftBody::Node* btCable::createPreparedSpare(btVector3 aPos, btVector3 bPos, btVector3 aVel, btVector3 bVel, int j, int segments, NodePairNarrowPhase* pair, btScalar newNodeMass)
{
	Node* spare = acquireSecondaryNode();
	if (!spare) return nullptr;

	// Interpolate along a -> b
	btScalar t  = btScalar(j) / btScalar(segments);
	btScalar it = btScalar(1) - t;

	btVector3 bLerpPos = t * bPos;
	btVector3 aLerpPos = it * aPos;

	spare->m_x = aLerpPos + bLerpPos;
	spare->m_v = it * aVel + t * bVel;
	spare->m_q = spare->m_x - bVel * m_sst.sdt;
	spare->m_im = 1.0f / newNodeMass;

	if (!pair)
	{
		return spare;
	}

	BroadPhasePair sparePair;
	sparePair.body = pair->pair->body;
	sparePair.node = spare;
	sparePair.bodyType = pair->pair->bodyType;
	
	_secondPairContact.push_back(sparePair);

	return spare;
}

void btCable::insertInterpolatedNodes(btLink<Node*>* anchor,
									  Node* a,
									  Node* b,
									  btScalar restBeforeA,
									  btScalar restAB,
									  btScalar restAfterB,
									  NodePairNarrowPhase* pair)
{
	if (!anchor || !a || !b || restAB <= btScalar(0)) return;

	btScalar dist = btDistance(a->m_x, b->m_x);
	if (dist <= m_backupAddThreshold * restAB) return;

	BackupNodesRun* backupRun = new BackupNodesRun();
	backupRun->leftPrimary = a;
	backupRun->rightPrimary = b;

	// Decide segments (policy). Ensure at least 2 if we insert.
	int segments = std::max(2, (int)std::ceil(dist / restAB));
	int inserts  = segments - 1;

	backupRun->secondaryRun.reserve(inserts);

	removeLinkBetween(a, b);

	btScalar rlSeg = restAB / (btScalar)segments;
	btScalar halfLinear = 0.5f * m_linearMass;

	// Update mass for 'a' immediately: adjacent rests (restBeforeA, rlSeg)
	btScalar massA = halfLinear * (btMax(restBeforeA, btScalar(0)) + rlSeg);
	if (a->isSecondary) a->m_im = 1.0f / massA; else setMass(a->index, massA);

	m_massOverrides[a].changedInvMass = a->m_im;
	m_massOverrides[a].hasChanged = true;

	Node* prev = a;
	btLink<Node*>* cursor = anchor;

	for (int j = 1; j <= inserts; ++j)
	{
		// Mass for the inserted node: (rlSeg, rlSeg)
		btScalar secondaryMass = halfLinear * (rlSeg + rlSeg);
		Node* spare = createPreparedSpare(a->m_x, b->m_x, a->m_v, b->m_v, j, segments, pair, secondaryMass);
		if (!spare) break;

		// Insert node into node list right after cursor
		btLink<Node*>* newNode = new btLink<Node*>();
		newNode->setValue(spare);
		newNode->insertAfter(cursor);
		cursor = cursor->getNext();

		// Connect prev -> spare with uniform rest
		addLinkBetweenConsecutiveNodes(prev, spare, rlSeg);

		backupRun->secondaryRun.push_back(spare);
		
		prev = spare;
	}

	// Final link prev -> b
	addLinkBetweenConsecutiveNodes(prev, b, rlSeg);

	// Mass for 'b': (rlSeg, restAfterB)
	btScalar massB = halfLinear * (rlSeg + btMax(restAfterB, btScalar(0)));
	if (b->isSecondary) b->m_im = 1.0f / massB; else setMass(b->index, massB);

	m_massOverrides[b].changedInvMass = b->m_im;
	m_massOverrides[b].hasChanged = true;

	m_backupNodesRun.push_back(backupRun);
}

void btCable::addBackupNodes()
{	
    // Iterate original contact pairs, but do not re-walk newly created links.
    for (int i = 0, nPairs = _nodePairContact.size(); i < nPairs; ++i)
    {
	    NodePairNarrowPhase* pair = &_nodePairContact.at(i);
    	Node* n = pair->node;

    	btLink<Node*>* linkN = m_linkedList.findByValue(n);
    	if (!linkN) continue;

    	// Next neighbor gap handling (n -> n_next)
    	btLink<Node*>* linkNext = linkN->getNext();
    	if (linkNext && !linkNext->isTail())
    	{
    		Node* a = linkN->getValue(); // n
    		Node* b = linkNext->getValue();

    		btScalar rest = 0;
    		btScalar restBefore = 0;
    		if (a->index > 0 && a->index - 1 < m_links.size())
    		{
    			Link* Lleft = nullptr;
    			Link* Lright = nullptr;
    			auto it = m_nodeAdj.find(n);
    			if (it != m_nodeAdj.end()) {
    				if (it->second.left)  Lleft  = it->second.left;
    				if (it->second.right) Lright = it->second.right;
    			}
    			
    			rest = Lright->m_rl;
    			restBefore = Lleft->m_rl;
    		}

    		btScalar restAfterB = 0;
    		if (b->index > 0 && b->index - 1 < m_links.size())
    		{
    			Link* Lright = nullptr;
    			auto it = m_nodeAdj.find(n);
    			if (it != m_nodeAdj.end()) {
    				if (it->second.right) Lright = it->second.right;
    			}
    			
    			restAfterB = Lright->m_rl;
    		}

    		insertInterpolatedNodes(linkN, a, b, restBefore, rest, restAfterB, pair);
    	}
    }
}

void btCable::addAnchorBackup()
{
	// Insert backup nodes between:
	// - the first anchor and the first node
	// - the last node and the last anchor
	// using a fixed number of nodes chosen based on distance.
	
	// Head anchor -> first node
	btLink<Node*>* firstNodeLink = m_linkedList.getHead(); // first node 
	if (firstNodeLink && !firstNodeLink->isTail())
	{
		Node* a = firstNodeLink->getValue(); // first node

		// Get rest to the right (firstNode -> secondNode)
		btScalar restAB = 0.0f;

		// Try to read from adjacency of anchor->a link. Expect m_nodeAdj[a].right to be the link (firstNode -> secondNode)
		Link* LRightOfA = nullptr;
		auto ita = m_nodeAdj.find(a);
		if (ita != m_nodeAdj.end()) 
		{
			LRightOfA = ita->second.right; // link: (firstNode -> secondNode)
		}

		if (LRightOfA)
		{
			restAB = LRightOfA->m_rl;
		}
		
		if (LRightOfA && restAB > FLT_EPSILON)
		{
			// Compute current distance between anchor and first node
			// Obtain anchor world position
			int idxAnchor = 0;
			for (int idx = 0; idx < m_anchors.size(); ++idx)
				idxAnchor = a->index == m_anchors[idx].m_node->index ? idx : idxAnchor;

			btVector3 xAnchorLocal = m_anchors[idxAnchor].m_local;
			btVector3 xAnchorWorld = m_anchors[idxAnchor].m_body->getWorldTransform() * xAnchorLocal;
			btVector3 xFirst  = a->m_x;
			btScalar dist = btDistance(xAnchorWorld, xFirst);

			if (dist > m_backupAnchorAddThreshold * restAB)
			{
				// Decide number of nodes to insert based on distance.
				// Compute desired inserts purely from distance
				int inserts = (int)floor(dist / m_anchorBackupSpacing);

				// Ensure we still insert something if over threshold
				inserts = btMax(inserts, 1);

				// We'll insert after anchor (before the first node).
				btLink<Node*>* cursor = firstNodeLink;

				// Insert from the node closest to 'a' down to the one closest to anchor
				for (int j = inserts; j >= 1; --j)
				{
					// j indicates which fraction between anchor (0) and a (1) this node lies at when distributing evenly by count
					// We reuse createPreparedSpare with j and (inserts+1) to place by fraction of dist
					Node* spare = createPreparedSpare(xAnchorWorld, a->m_x, btVector3{0,0,0}, a->m_v, j, inserts + 1, nullptr, m_anchorBackupMass);
					if (!spare) break;

					// Insert before current cursor
					btLink<Node*>* newNode = new btLink<Node*>();
					newNode->setValue(spare);
					newNode->insertBefore(cursor);

					m_anchorBackups.push_back(*spare);
					
					cursor = newNode;    // next insert goes before this one
				}
			}
		}
	}
	
	// Last node -> tail anchor
	// Find last node (node before tail)
	btLink<Node*>* lastNodeLink = m_linkedList.getTail(); // last node
	
	if (lastNodeLink && !lastNodeLink->isHead())
	{
		Node* b = lastNodeLink->getValue(); // last node
	
		// Get rest to the left (prevB -> B)
		btScalar restAB = 0.0f;
	
		// From adjacency, the left link of b should be (prevB -> B)
		Link* LleftOfB = nullptr;

		auto itb = m_nodeAdj.find(b);
		if (itb != m_nodeAdj.end()) 
		{
			LleftOfB = itb->second.left; // link: (prevB -> B)
		}

		if (LleftOfB)
		{
			restAB = LleftOfB->m_rl;
		}
			
		if (LleftOfB && restAB > FLT_EPSILON) 
		{
			// Compute current distance between b and anchor		
			int idxAnchor = 0;
			for (int idx = 0; idx < m_anchors.size(); ++idx)
				idxAnchor = b->index == m_anchors[idx].m_node->index ? idx : idxAnchor;

			btVector3 xAnchorLocal = m_anchors[idxAnchor].m_local;
			btVector3 xAnchorWorld = m_anchors[idxAnchor].m_body->getWorldTransform() * xAnchorLocal;
			btVector3 xLast = b->m_x;
			btScalar dist = btDistance(xLast, xAnchorWorld);
	
			if (dist > m_backupAnchorAddThreshold * restAB)
			{
				// Decide number of nodes to insert based on distance (same policy as head side)
				int inserts = (int)floor(dist / m_anchorBackupSpacing);
				inserts = btMax(inserts, 1);
	
				// Insert before tail anchor (after last node)
				btLink<Node*>* cursor = lastNodeLink;
	
				for (int j = 1; j <= inserts; ++j)
				{
					// Place by even fractions across the distance segment
					Node* spare = createPreparedSpare(b->m_x, xAnchorWorld, b->m_v, btVector3{0,0,0}, j, inserts + 1, nullptr, m_anchorBackupMass);
					if (!spare) break;
	
					btLink<Node*>* newNode = new btLink<Node*>();
					newNode->setValue(spare);
					newNode->insertAfter(cursor);
					cursor = cursor->getNext();
	
					m_anchorBackups.push_back(*spare);
				}
			}
		}
	}
}

void btCable::updateBackupNodes()
{
	for (int i = 0; i < m_backupNodesRun.size(); ++i)
	{
		BackupNodesRun* run = m_backupNodesRun.at(i);

		if (!run || !run->leftPrimary || !run->rightPrimary) { continue; }
		if (run->leftPrimary->isSecondary || run->rightPrimary->isSecondary) { continue; }
		
		// Interpolate secondaries uniformly between a and b,
		// matching the creation in insertInterpolatedNodes where rlSeg was uniform.
		btVector3 xa = run->leftPrimary->m_x;
		btVector3 xb = run->rightPrimary->m_x;

		int count = run->secondaryRun.size();
		
		for (int k = 0; k < count; ++k)
		{
			Node* s = run->secondaryRun[k];
			if (!s) { continue; }
			
			// assume s is secondary by construction
			const btScalar t = btScalar(k + 1) / btScalar(count + 1);
			s->m_x = (1.0f - t) * xa + t * xb;
		}
	}
}

void btCable::secondaryNodesContact(btAlignedObjectArray<BroadPhasePair>& candidates, bool applyNodeChange)
{
	for (int i = 0; i < candidates.size(); i++)
	{
		BroadPhasePair pair = candidates.at(i);
		Node *n = &m_secondaryPool.at(pair.node->poolIndex).node;		

		// First contact test
		_nodeContactObject.setWorldTransform(btTransform(btQuaternion::getIdentity(), n->m_x));
		MyContactResultCallback cb1(0, &_nodeContactObject, pair.body);
		m_world->contactPairTest(&_nodeContactObject, pair.body, cb1);

		if (cb1.m_connected && cb1.minDist < 0)
		{
			// Compute first projection (do not apply yet)
			btVector3 proj1 = cb1.contactPoint + cb1.contactNorm * (m_collisionMargin + FLT_EPSILON);

			// Second contact test from projected position (corner case)
			_nodeContactObject.setWorldTransform(btTransform(btQuaternion::getIdentity(), proj1));
			MyContactResultCallback cb2(0, &_nodeContactObject, pair.body);
			m_world->contactPairTest(&_nodeContactObject, pair.body, cb2);

			// Choose the deepest penetration if both hit; otherwise keep the first
			bool hit2 = (cb2.m_connected && cb2.minDist < 0);
			bool use2 = hit2 && (btFabs(cb2.minDist) > btFabs(cb1.minDist));

			btVector3& finalPoint = use2 ? cb2.contactPoint : cb1.contactPoint;
			btVector3& finalNorm  = use2 ? cb2.contactNorm  : cb1.contactNorm;

			// Final depenetration using the chosen contact (apply once at the end if requested)
			btVector3 finalPos = finalPoint + finalNorm * (m_collisionMargin + FLT_EPSILON);

			// Apply impulse once, using the chosen (deepest) contact
			if (impulseCompute)
			{
				btRigidBody* rb = btRigidBody::upcast(pair.body);
				while (rb->m_redirectionTarget)
				{
					rb = rb->m_redirectionTarget;
				}
				btVector3 impulse = calculateBodyImpulse(rb, n, finalNorm, finalPoint);
				rb->applyRedirectionImpulse(impulse, finalPoint);
			}

			if (applyNodeChange)
			{
				n->m_x = finalPos;
				n->m_n = finalNorm;
			}
		}
	}
}

void btCable::depenetrateBackups(btAlignedObjectArray<BroadPhasePair>& candidates)
{
	for (int i = 0; i < candidates.size(); i++)
	{
		BroadPhasePair pair = candidates.at(i);
		Node *n = pair.node;

		// First contact test
		_nodeContactObject.setWorldTransform(btTransform(btQuaternion::getIdentity(), n->m_x));
		MyContactResultCallback cb1(0, &_nodeContactObject, pair.body);
		m_world->contactPairTest(&_nodeContactObject, pair.body, cb1);

		if (cb1.m_connected && cb1.minDist < 0)
		{
			// Compute first projection (do not apply yet)
			btVector3 proj1 = cb1.contactPoint + cb1.contactNorm * (m_collisionMargin + FLT_EPSILON);

			// Second contact test from projected position (corner case)
			_nodeContactObject.setWorldTransform(btTransform(btQuaternion::getIdentity(), proj1));
			MyContactResultCallback cb2(0, &_nodeContactObject, pair.body);
			m_world->contactPairTest(&_nodeContactObject, pair.body, cb2);

			// Choose the deepest penetration if both hit; otherwise keep the first
			bool hit2 = (cb2.m_connected && cb2.minDist < 0);
			bool use2 = hit2 && (btFabs(cb2.minDist) > btFabs(cb1.minDist));

			btVector3& finalPoint = use2 ? cb2.contactPoint : cb1.contactPoint;
			btVector3& finalNorm  = use2 ? cb2.contactNorm  : cb1.contactNorm;

			// Final depenetration using the chosen contact (apply once at the end if requested)
			btVector3 finalPos = finalPoint + finalNorm * (m_collisionMargin + FLT_EPSILON);

			n->m_x = finalPos;
            n->m_n = finalNorm;
		}
	}
}

void btCable::removeBackupNodes()
{
	btLink<Node*>* cur = m_linkedList.getHead();
	while (cur && !cur->isTail())
	{
		// Find a primary node that has at least one secondary following
		Node* left = cur->getValue();
		if (!left || left->isSecondary)
		{
			cur = cur->getNext();
			continue;
		}

		btLink<Node*>* runStart = cur->getNext();
		if (!runStart || runStart->isTail() || !runStart->getValue() || !runStart->getValue()->isSecondary)
		{
			cur = cur->getNext();
			continue;
		}

		// Gather the whole run of consecutive secondaries
		btLink<Node*>* runEnd = runStart;
		int secondaryCount = 0;
		while (runEnd && !runEnd->isTail())
		{
			Node* n = runEnd->getValue();
			if (!n || !n->isSecondary) break;
			secondaryCount++;
			runEnd = runEnd->getNext();
		}

		// runEnd is first non-secondary (or tail). Must be a primary on the right to consider removal/trim.
		if (!runEnd || runEnd->isTail())
		{
			// No right primary, skip past the run
			cur = runEnd; // could be tail
			continue;
		}

		Node* right = runEnd->getValue();
		if (!right || right->isSecondary)
		{
			// Unexpected, but advance to be safe
			cur = runEnd;
			continue;
		}

		// Compute primary-to-primary span
		btScalar ppDist2 = btDistance2(left->m_x, right->m_x);
		// Use authoritative primary-to-primary link rest length
		const int linkIndex = left->index; // link between left(i) and right(i+1) is m_links[i]
		if (linkIndex < 0 || linkIndex >= m_links.size())
		{
			cur = runEnd;
			continue;
		}
		const btScalar linkRest = m_links[linkIndex].m_rl;
		btScalar rest2 = linkRest * linkRest;

		// If P-P distance fits in one segment, remove the entire secondary run
		if (ppDist2 <= rest2)
		{
			btLink<Node*>* it = runStart;
			while (it != runEnd)
			{
				btLink<Node*>* toRemove = it;
				it = it->getNext();

				Node* n = toRemove->getValue();
				toRemove->remove();
				if (n) releaseSecondaryNode(n);
				delete toRemove;
			}

			// Rebuild single edge left->right
			removeLinkBetween(left, right);
			addLinkBetweenConsecutiveNodes(left, right, linkRest);
			
			// Continue after right primary
			cur = runEnd;
			continue;
		}

		// Otherwise, compute required segments and trim extras if any
		// segments k = ceil(dist / rest) -> required secondaries = k - 1
		const btScalar ppDist = btDistance(left->m_x, right->m_x);
		int segments = (int)ceil(ppDist / linkRest);
		if (segments < 1) segments = 1;
		int requiredSecondaries = segments - 1;

		if (secondaryCount > requiredSecondaries)
		{
			// Remove extras, keep uniformly spaced ones to approximate even subdivision.
			int toRemove = secondaryCount - requiredSecondaries;

			// Strategy: keep first requiredSecondaries nodes, remove the rest from the tail of the run.
			// This is simple and avoids reindexing math. If better distribution is needed, pick every Nth.
			btLink<Node*>* it = runStart;
			for (int kept = 0; kept < requiredSecondaries && it != runEnd; ++kept)
			{
				it = it->getNext();
			}
			// Now 'it' points to first extra to remove
			while (toRemove > 0 && it != runEnd)
			{
				btLink<Node*>* toRemoveLink = it;
				it = it->getNext();

				Node* n = toRemoveLink->getValue();
				toRemoveLink->remove();
				if (n) releaseSecondaryNode(n);
				delete toRemoveLink;

				--toRemove;
			}

			removeLinkBetween(left, right);
			Node* prev = left;
			btLink<Node*>* walk = cur->getNext();
			while (walk != runEnd)
			{
				Node* mid = walk->getValue();
				addLinkBetweenConsecutiveNodes(prev, mid);
				prev = mid;
				walk = walk->getNext();
			}
			
			addLinkBetweenConsecutiveNodes(prev, right);
			
			// Continue after right primary
			cur = runEnd;
			continue;
		}

		// Nothing to remove/trim; move forward
		cur = runEnd;
	}
}

void btCable::removeAllBackupNodes()
{
	btLink<Node*>* cur = m_linkedList.getHead();
	while (cur && !cur->isTail())
	{
		btLink<Node*>* thisLink = cur;
		cur = cur->getNext(); // advance first to keep iterator valid

		Node* n = thisLink->getValue();
		if (!n || !n->isSecondary)
		{
			continue;
		}

		// Unlink from list
		thisLink->remove();

		// Return the node to the pool
		releaseSecondaryNode(n);

		// Free the list link if heap-allocated
		delete thisLink;
	}

	// Recreate link list to mirror existing primary links in m_links
	// 1) clear current link-edge list
	btLink<Link*>* ecur = m_linkedListLinks.getHead();
	while (ecur && !ecur->isTail())
	{
		btLink<Link*>* rm = ecur;
		ecur = ecur->getNext();
		m_linkedListLinks.remove(rm);
		onLinkRemoved(rm->getValue());
		delete rm;
	}

	// 2) repopulate with pointers to m_links entries (primary edges only)
	for (int i = 0; i < m_links.size(); ++i)
	{
		m_linkedListLinks.addTail(&m_links[i]);
		onLinkInserted(&m_links[i]);
	}

	updateNodesMass();
	m_backupNodesRun.clear();
	_secondPairContact.clear();
	_anchorBackupCandidates.clear();
	m_anchorBackups.clear();
	resetOverridesState();
}

void btCable::resetNodesAndLinks()
{
	// Rebuild the linked lists from the primary arrays: m_nodes and m_links.

	// Clear lists
	m_linkedList.clear();
	m_linkedListLinks.clear();
	m_nodeAdj.clear();

	// Recreate node list in order
	// Insert each Node* between head and tail, preserving order of m_nodes.
	for (size_t i = 0; i < m_nodes.size(); ++i)
	{
		Node* n = &m_nodes[i];
		if (!n) continue;

		btLink<Node*>* link = new btLink<Node*>();
		link->setValue(n);
		m_linkedList.addTail(link);
	}

	// Recreate link list in order
	// Insert each Link* similarly, assuming m_links is ordered along the cable.
	for (size_t i = 0; i < m_links.size(); ++i)
	{
		Link* L = &m_links[i];
		if (!L) continue;

		btLink<Link*>* link = new btLink<Link*>();
		link->setValue(L);
		m_linkedListLinks.addTail(link);
		onLinkInserted(L);
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
		Anchor& a = m_anchors[i];
		Node& n = *a.m_node;

		bool useMassBalance = a.BodyMassRatio > 0;
		const btVector3 wa = a.m_body->getWorldTransform() * a.m_local;
		const btVector3 va = a.m_body->getVelocityInLocalPoint(a.m_c1) * dt;
		const btVector3 vb = n.m_x - n.m_q;
		const btVector3 vr = (va - vb) + (wa - n.m_x) * kAHR;
		btVector3 impulse = (!useMassBalance ? a.m_c0 : a.m_c0_massBalance) * vr * a.m_influence;

		// Limit the impulse
		btScalar currentTension = a.tension.length();
		a.tension += impulse / dt;
		btScalar finalTension = a.tension.length();
		if (m_maxTension >= 0 && finalTension >= m_maxTension)
		{
			a.tension = a.tension.normalized() * m_maxTension;
			impulse *= (a.tension.length() - currentTension) / (finalTension - currentTension);
		}

		// Update anchor's data
		a.m_dist = wa.distance(n.m_x);
		a.m_body->applyImpulse(-impulse, a.m_c1);

		// Update node's position
		// n.m_x += impulse * (!useMassBalance ? a.m_c2 : a.m_c2_massBalance);
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

void btCable::contactConstraint(btAlignedObjectArray<NodePairNarrowPhase> pairContacts)
{
	int nbContactPairPotential = pairContacts.size();
	if (nbContactPairPotential == 0) return;

	for (int i = 0; i < nbContactPairPotential; ++i)
	{
		NodePairNarrowPhase* pair = &pairContacts.at(i);
		btCollisionObject* obj = pair->pair->body;
		btRigidBody* rb = btRigidBody::upcast(obj);

		// Get the top rigidBody which contains the mass of the object (otherwise there will be no impulse)
		while (rb->m_redirectionTarget)
		{
			rb = rb->m_redirectionTarget;
		}
		
		Node* node = pair->node;

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
			node->m_n = pair->normal;
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
	if (collisionMode == CollisionMode::Linear && penetrationDistance > penetrationMin)
	{
		btScalar distanceTot = penetrationMax - penetrationMin;
		btScalar ratio = (penetrationDistance - penetrationMin) / distanceTot;
		k = Lerp(this->collisionStiffnessMin, this->collisionStiffnessMax, min(1.0, ratio));		
	}
	else if (collisionMode == CollisionMode::Curve && spline)
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

int btCable::getCollisionMode()
{
	return (int)collisionMode;
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

	rememberPrimariesMass();
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

void btCable::setBackupInsertionThreshold(btScalar multiplier)
{
	m_backupAddThreshold = multiplier;
}

void btCable::setAnchorBackupInsertionThreshold(btScalar multiplier)
{
	m_backupAnchorAddThreshold = multiplier;
}

void btCable::setAnchorBackupMass(btScalar mass)
{
	m_anchorBackupMass = mass;
}

void btCable::setAnchorBackupSpacing(btScalar spacing)
{
	m_anchorBackupSpacing = spacing;
}


void btCable::setCollisionBackupActivation(bool active)
{
	collisionBackupEnabled = active;
}

void btCable::setAnchorBackupActivation(bool active)
{
	anchorBackupEnabled = active;
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

#pragma endregion
