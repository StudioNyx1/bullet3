#include "BulletCollision/GImpact/btGImpactShape.h"
#include <tuple>
#include <chrono>
#include <vector>
#include <thread>
#include <LinearMath/btQuickprof.h>
#include <BulletSoftBody/btSoftBodyInternals.h>
#include <BulletSoftBody/btSoftRigidDynamicsWorld.h>
#include <BulletCollision/CollisionShapes/btSphereShape.h>

btCable::btCable(btSoftBodyWorldInfo* worldInfo, btCollisionWorld* world, int node_count, int section_count, const btVector3* x, const btScalar* m) : btSoftBody(worldInfo, node_count, x, m)
{
	m_world = world;
	m_solverSubStep = worldInfo->numIteration;
	m_cpt = 0;

	// Initialize Data
	m_cableData = new CableData();
	m_nodePos = new NodePos[worldInfo->maxNodeNumberPerCable]();
	m_nodeData = new NodeData[worldInfo->maxNodeNumberPerCable]();

	for (int i = 0; i < this->m_nodes.size(); i++)
	{
		m_nodes[i].m_battach = 0;
		m_nodes[i].index = i;
		m_nodes[i].m_xn = x[i];

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
			n.m_f = btVector3(0, 0, 0);  // reset node total forces
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
	NodeForces* nodeForces = static_cast<btSoftRigidDynamicsWorld*>(m_world)->m_nodeForces;
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

void btCable::collectPotentials(btCollisionObjectArray &collisionObjectArray, std::vector<btCollisionObject*>& out) const {
	out.reserve(collisionObjectArray.size());
	for (int i = 0, n = collisionObjectArray.size(); i < n; ++i) {
		btCollisionObject* co = collisionObjectArray[i];
		if (shouldTestObject(co))
			out.push_back(co);
	}
}

// for each potential, cache its AABB + interpolation velocity
void btCable::buildObjData(const std::vector<btCollisionObject*>& pots,
                           std::vector<ObjData>& out) const {
	out.clear();
	out.reserve(pots.size());
	for (auto* co : pots) {
		btVector3 mi, ma;
		co->getCollisionShape()->getAabb(co->getWorldTransform(), mi, ma);
		btVector3 velB = co->getInterpolationLinearVelocity();
		out.emplace_back(ObjData { co, mi, ma, velB });
	}
}

// parallel AABB test
void btCable::runBroadPhase(std::vector<ObjData>& data, std::vector<BroadPhasePair>* candidates)
{

	// Before starting threads, first check if the cable global AABB overlaps any objects
	bool anyHit = false;
	for (auto& od : data) {
		if ( m_bounds[0].x() <= od.maxAabb.x() && m_bounds[1].x() >= od.minAabb.x() &&
		     m_bounds[0].y() <= od.maxAabb.y() && m_bounds[1].y() >= od.minAabb.y() &&
		     m_bounds[0].z() <= od.maxAabb.z() && m_bounds[1].z() >= od.minAabb.z() )
		{
			anyHit = true;
			break;
		}
	}
	if (!anyHit) {
		// no possible collision this frame, skip threading
		return;
	}

	int nodeCount = m_nodes.size();

	// one bucket per thread
	std::vector<std::vector<BroadPhasePair>> threadBuckets(omp_get_num_threads());

	// Notify the dispatcher to pass into threaded mode
	btCollisionDispatcherMt* disp = static_cast<btCollisionDispatcherMt*>(m_world->getDispatcher());
	disp->m_batchUpdating = true;
	
	#pragma omp parallel for schedule(static, 1)
	for (int i = 0; i < nodeCount; ++i)
	{
		int tid = omp_get_thread_num();
		Node* n = &m_nodes[i];
		
		btVector3 nodeVel = n->m_v / m_sst.sdt;

		// static node box (no margin first)
		btVector3 lo, hi;
		setNodeBoundingBox(n->m_x, n->m_q, 0, &lo, &hi);

		// test each object
		for (const auto& od : data)
		{
			if (aabbTestMargin(nodeVel, od.objVelocity, lo, hi, od.minAabb, od.maxAabb)) {
				continue;
			}
			
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
	for (std::vector<BroadPhasePair>& bucket : threadBuckets)
		candidates->insert(candidates->end(), bucket.begin(), bucket.end());
}

bool btCable::aabbTestMargin(btVector3 nodeVel, btVector3 objVel, btVector3 nodeMinAabb, btVector3 nodeMaxAabb, btVector3 minAabb, btVector3 maxAabb) {
	btScalar margin = m_collisionMargin
					+ (objVel - nodeVel).length() * m_sst.sdt
					+ btScalar(0.05);
	// expand and test aabb
	return nodeMinAabb.x() - margin > maxAabb.x() || nodeMaxAabb.x() + margin < minAabb.x() ||
	    nodeMinAabb.y() - margin > maxAabb.y() || nodeMaxAabb.y() + margin < minAabb.y() ||
	    nodeMinAabb.z() - margin > maxAabb.z() || nodeMaxAabb.z() + margin < minAabb.z();
}

// single‐threaded narrow phase
void btCable::runNarrowPhase(std::vector<BroadPhasePair>& cands,
                             btAlignedObjectArray<int>*     idxOut,
                             btAlignedObjectArray<NodePairNarrowPhase>* npOut)
{
	for (BroadPhasePair& c : cands)
	{		
		btCollisionObject* co = c.body;
		btRigidBody* rb = btRigidBody::upcast(co);
		
		if (!rb)
			continue;
		
		btVector3 rbVel = rb->getVelocityInLocalPoint(c.node->m_q - rb->getWorldTransform().getOrigin());
		btVector3 nodePosition = c.node->m_q + rbVel * m_sst.sdt;
		
		btSphereShape nodeShape(m_collisionMargin);
		btCollisionObject tmp;
		tmp.setCollisionShape(&nodeShape);
		tmp.setWorldTransform(btTransform(btQuaternion::getIdentity(), nodePosition));
		
		MyContactResultCallback result(0, &tmp, co);
		m_world->contactPairTest(&tmp, co, result);
		
		if (result.numContacts == 0)
			continue;

		c.node->m_nbCollidingObjectPotential++;

		NodePairNarrowPhase pair;
		pair.node           = c.node;
		pair.numContacts    = result.numContacts;
		pair.pair           = &c;
		pair.worldTransform = co->getWorldTransform();

		pair.m_Xouts.resize(result.numContacts);
		pair.normals.resize(result.numContacts);
		pair.distances.resize(result.numContacts);

		for (int i = 0; i < result.numContacts; i++) {
			pair.m_Xouts[i] = result.contacts[i].point + result.contacts[i].normal * m_collisionMargin;
			pair.normals[i] = result.contacts[i].normal;
		}
		
		npOut->push_back(pair);
		idxOut->push_back(c.node->index);
	}
}

void btCable::detectCollisionsThreaded(
    btAlignedObjectArray<int>*     indexNodeContact,
    btAlignedObjectArray<NodePairNarrowPhase>* nodePairContact)
{
	btCollisionObjectArray objects = m_world->getCollisionObjectArray();
	if (objects.size() < 2)
		return;

	// find which objects we could hit
	std::vector<btCollisionObject*> potentials;
	collectPotentials(objects, potentials);
	
	if (potentials.empty())
		return;

	// cache their AABBs + velocities
	std::vector<ObjData> objData;
	buildObjData(potentials, objData);

	// run the threaded broad phase
	std::vector<BroadPhasePair>* candidates = new std::vector<BroadPhasePair>;
	runBroadPhase(objData, candidates);
	
	if (candidates->empty())
		return;

	// run the serial narrow phase
	runNarrowPhase(*candidates, indexNodeContact, nodePairContact);
}

// void btCable::detectCollisions(btAlignedObjectArray<int>* indexNodeContact,
// 				       btAlignedObjectArray<NodePairNarrowPhase>* nodePairContact, btAlignedObjectArray<BroadPhasePair*>*  broadPhaseOutput)
// {
// 	btScalar marginNode = m_collisionMargin;
// 	btScalar margin;
// 	int ni = m_nodes.size();
//
// 	// BroadPhase (quick check to determine which collisions can collide with a cable)
// 	// TODO @benH: Nettoyer code, fusionner Broad&Narrow (uillser les aabbs), faire un meilleur check de collision masks
// 	auto objects = m_world->getCollisionObjectArray();
// 	for (int w = 0; w < objects.size(); w++)
// 	{
// 		// Add Object to the potential list
// 		btCollisionObject* colObj = objects[w];
//
// 		if (colObj->getInternalType() != CO_RIGID_BODY)
//
// 		{
// 			continue;
// 		}
//
// 		// The 4nd bit of the m_collisionFilterMask needs to be 1
// 		std::string byte = std::bitset<8>(colObj->getBroadphaseHandle()->m_collisionFilterMask).to_string();
// 		int bitCable = byte.size() - 4;
// 		if (byte[bitCable] == '0')
// 		{
// 			continue;
// 		}
//
// 		if (colObj->hasContactResponse() && this->m_collisionDisabledObjects.findLinearSearch(colObj) == m_collisionDisabledObjects.size())
// 		{
// 			BroadPhasePair* temp = new BroadPhasePair();
// 			temp->body = colObj;
// 			temp->bodyType = colObj->getInternalType();
// 			broadPhaseOutput->push_back(temp);
// 		}
// 	}
//
// 	// NarrowPhase
// 	int potentialCollisionObjectListSize = broadPhaseOutput->size();
// 	if (potentialCollisionObjectListSize > 0)
// 	{
// 		for (int i = 0; i < ni; i++)
// 		{
// 			Node& n = m_nodes[i];
// 			n.collideInAllIteration = false;
// 			btVector3 velA = (n.m_x - n.m_q) / m_sst.sdt;
//
// 			for (int j = 0; j < potentialCollisionObjectListSize; j++)
// 			{
// 				btCollisionObject* co = (*broadPhaseOutput)[j]->body;
// 				btVector3 velB = co->getInterpolationLinearVelocity();
//
// 				btScalar dVel = (velB - velA).length();
// 				margin = marginNode + dVel * m_sst.sdt + 0.05;
//
// 				// Box Definition
// 				btVector3 minLink = btVector3(0, 0, 0);
// 				btVector3 maxLink = btVector3(0, 0, 0);
//
// 				setNodeBoundingBox(n.m_x, n.m_q, margin, &minLink, &maxLink);
// 				recursiveBroadPhase(broadPhaseOutput->at(j), &n, co->getCollisionShape(), nodePairContact, minLink, maxLink, co->getWorldTransform());
// 			}
//
// 			if (n.m_nbCollidingObjectPotential > 0)
// 			{
// 				indexNodeContact->push_back(n.index);
// 			}
// 		}
// 	}
// }

void btCable::solveConstraints()
{
	int i, ni;

	// Prepare nodes
	for (i = 0, ni = m_nodes.size(); i < ni; ++i)
	{
		m_nodes[i].cptIteration = 0;
		m_nodes[i].computeNodeConstraint = true;
		//m_nodes[i].posPreviousIteration = m_nodes[i].m_x;
		//m_nodes[i].collideInAllIteration = false;
		m_nodes[i].m_nbCollidingObjectPotential = 0;
		m_nodes[i].m_splitv = btVector3(0, 0, 0);
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
	btAlignedObjectArray<int> indexNodeContact = btAlignedObjectArray<int>();
	btAlignedObjectArray<NodePairNarrowPhase> nodePairContact = btAlignedObjectArray<NodePairNarrowPhase>();
	btAlignedObjectArray<CollisionState> collisionState = btAlignedObjectArray<CollisionState>();
	
	if (useCollision)
	{
		auto start = std::chrono::high_resolution_clock::now();
		detectCollisionsThreaded(&indexNodeContact, &nodePairContact);
		auto end = std::chrono::high_resolution_clock::now();
		std::chrono::duration<double, std::milli> duration = end - start;
		std::cout << "[detectCollisions] Took " << duration.count() << " ms" << std::endl;
	}

	// Solve constraints
	bool impacted = false;
	for (i = 0; i < m_cfg.piterations; ++i)
	{
		bool lastStep = i == m_cfg.piterations - 1;

		updateNodeDeltaPos(i);

		anchorConstraint(impacted);

		distanceConstraint();

		if (useLRA)
		{
			LRAConstraint();
		}

		if (useBending && (i % 2 == 0 || lastStep))
		{
			bendingConstraint();
		}

		if (useCollision && (i % m_substepDelayCollision == 0 || lastStep))
		{
			contactConstraint(&nodePairContact, &collisionState, &indexNodeContact);
		}
	}

	if (useCollision)
	{
		resolveConflitZone(&nodePairContact, &collisionState);
	}

	if (impacted)
	{
		anchorConstraint(impacted);
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

	for (i = 0; i < nodePairContact.size(); i++)
	{
		NodePairNarrowPhase* nodePair = &nodePairContact.at(i);
		btPersistentManifold* manifold;
		// If the manifold exist, use it
		if (nodePair->haveManifoldsRegister)
			manifold = nodePair->manifold;
		// Else create a new one
		else
		{
			manifold = m_world->getDispatcher()->getNewManifold(this, nodePair->pair->body);
			CableManifolds cm = CableManifolds(manifold, m_solverSubStep);
			manifolds.push_back(cm);
			nodePair->manifold = manifold;
			nodePair->haveManifoldsRegister = true;
		}
		// Contact point

		// Obj 0 = Cable
		// Obj 1 = RigidBody
		for (int j = 0; j < nodePair->numContacts; j++) {
			const btVector3& pointB = nodePair->m_Xouts[j];
			const btVector3& normal = nodePair->normals[j];
			const btScalar distance = nodePair->distances[j];

			btManifoldPoint newPoint = btManifoldPoint(btVector3{0, 0, 0}, btVector3{0, 0, 0}, normal, distance);
			newPoint.m_positionWorldOnA = pointB + normal * distance;
			newPoint.m_positionWorldOnB = pointB;
			manifold->addManifoldPoint(newPoint, true);
		}
	}

	// Clear manifolds without contact point
	clearManifoldContact();

	nodePairContact.clear();
	indexNodeContact.clear();
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
	btMin(mx.z(), mq.z())
	 );

	const btVector3 upper(
	    btMax(mx.x(), mq.x()),
	    btMax(mx.y(), mq.y()),
	    btMax(mx.z(), mq.z())
	);

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
	btScalar distAnchor = 0.0;
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

void btCable::contactConstraint(btAlignedObjectArray<NodePairNarrowPhase>* nodePairContact, btAlignedObjectArray<CollisionState>* collisionStates, btAlignedObjectArray<int>* indexNodeContact) {
	int nbSubStep = m_subIterationCollision;
	int nbContactPairPotential = nodePairContact->size();
	if (nbContactPairPotential == 0) return;

	auto shouldCast = [&](int nodeIdx, int step, const CollisionState& S) -> bool {
		const Node& n = m_nodes[nodeIdx];
		if (m_collisionMargin <= 0 || n.m_battach != 0) return false;
		if (step > 0 && n.m_nbCollidingObjectPotential == 1) return false;
		return (step == 0) || S.collided;
	};

	auto computeShift = [&](const btVector3& start,
			    const btVector3& hit,
			    const btVector3& normal) -> btVector3 {
		const btScalar backOff = 0.0005f;
		btVector3 outShift = normal * backOff;
		btVector3 touchDir = (start.distance(hit) > FLT_EPSILON)
		    ? (start - hit).normalized()
		    : btVector3(0,0,0);
		btScalar cosTheta = touchDir.dot(normal);
		btScalar theta    = acos(cosTheta);
		btScalar deltaPos = theta * backOff;
		btVector3 oppose = (start.distance(hit + outShift) > FLT_EPSILON)
		    ? (start - (hit + outShift)).normalized()
		    : btVector3(0,0,0);
		btVector3 correction = oppose * (deltaPos * 0.5f);
		btVector3 newPos = hit + outShift + correction;
		return newPos - start;
	};

	auto twoContactSolve = [&](const std::array<btVector3,2>& norms,
				   const std::array<btVector3,2>& hits,
				   btScalar topMargin) -> btVector3 {
		const btScalar PI = SIMD_PI;
		btVector3 n0 = norms[0], n1 = norms[1];
		btVector3 mid = (hits[0] + hits[1]) * 0.5f;
		btScalar dotP = btClamped(n0.dot(n1), -1.0, 1.0);
		btScalar angle = PI - acos(dotP);
		btScalar sinA  = sin(angle);
		if (btFuzzyZero(sinA)) return hits[0];
		btVector3 dir = (n0 + n1).normalized();
		btScalar B = 0.5f * PI - 0.5f * angle;
		btScalar b = (topMargin * 0.5f) / (sinA * 0.5f) * sin(B);
		return mid + dir * b;
	};

	// 4) Allocate and initialize CollisionState for each node
	for (int idx = 0; idx < indexNodeContact->size(); idx++)
	{
		int id = indexNodeContact->at(idx);
		collisionStates->at(idx).posBefore = m_nodes[id].m_x;
		// phaseIdx initialized to {-1,-1}, everCollided stays false
	}

	// 5) Main substep loop
	for (int step = 0; step < nbSubStep; ++step) {
		// Reset per-node state for this iteration
		for (int idx = 0; idx < indexNodeContact->size(); idx++)
		{
			int id = indexNodeContact->at(idx);
			CollisionState& state = collisionStates->at(id);
			state.accumulatedShift = btVector3(0,0,0);
			state.hitCount         = 0;
			state.topMargin        = 0;
			state.collided         = false;
		}

		// 5A) Gather all ray-cast jobs into one flat vector
		std::vector<RayJob> jobs;
		jobs.reserve(nbContactPairPotential * 2);  // assume ≤2 rays per pair
		
		for (int i = 0; i < nbContactPairPotential; ++i) {
			NodePairNarrowPhase* pair = &nodePairContact->at(i);
			int nIdx = pair->node->index;  // assume each node has an index
			for (int k = 0; k < pair->numContacts; ++k) {
	            	
				if (!shouldCast(nIdx, step, collisionStates->at(nIdx))) continue;
				jobs.push_back({
				    nIdx, pair, k,
				    pair->m_Xouts[k],
				    m_nodes[nIdx].m_x,
				    computeCollisionMargin(pair->pair->body->getCollisionShape())
				});
	            	
				// record which contact‐pair slots belong to this node
				if (step == 0) {
					CollisionState& state = collisionStates->at(nIdx);
					for (int p = 0; p < 2; ++p) {
						if (state.phaseIdx[p] < 0) { state.phaseIdx[p] = i; break; }
					}
				}
			}
		}

		// 5B) Cast each ray and accumulate results
		for (auto& J : jobs) {
			auto result = castRay(J.from, J.to, J.pair, J.margin);
			if (!result.hasHit()) continue;

			CollisionState& state = collisionStates->at(J.nodeIdx);
			state.collided     = state.everCollided = true;
			state.topMargin    = btMax(state.topMargin, J.margin);
			int idx        = state.hitCount++;
			state.normals[idx] = result.m_hitNormalWorld;
			state.hits[idx]    = result.m_hitPointWorld;
			state.accumulatedShift += computeShift(J.from,
							  result.m_hitPointWorld,
							  result.m_hitNormalWorld);

			// apply impulse if this is a rigid body
			auto* obj = J.pair->pair->body;
			if (obj->getInternalType() == CO_RIGID_BODY && impulseCompute) {
				auto* rb = btRigidBody::upcast(obj);
				auto imp = calculateBodyImpulse(
				    rb, m_collisionMargin,
				    &m_nodes[J.nodeIdx],
				    result.m_hitNormalWorld,
				    result.m_hitPointWorld
				);
				rb->applyRedirectionImpulse(imp, m_nodes[J.nodeIdx].m_x);
			}
		}

		// 5C) Update node positions based on accumulated shifts
		bool anyContact = false;
		for (int idx = 0; idx < indexNodeContact->size(); idx++)
		{
			int id = indexNodeContact->at(idx);
			CollisionState& state = collisionStates->at(id);
			if (state.hitCount == 0) continue;
			anyContact = true;

			btVector3 newPos;
			if (state.hitCount == 1) {
				newPos = state.posBefore + state.accumulatedShift;
			} else {
				newPos = twoContactSolve(state.normals, state.hits, state.topMargin);
			}

			auto& node = m_nodes[id];
			node.m_x = newPos;
			node.collideInAllIteration = true;
		}

		// If no collisions this iteration, stop early
		if (!anyContact) break;
	}

	// 6) Refresh ray start positions for the next solver step
	for (int i = 0; i < nbContactPairPotential; ++i) {
		NodePairNarrowPhase* pair = &nodePairContact->at(i);
		int nIdx = pair->node->index;
		for (int k = 0; k < pair->numContacts; ++k) {
			pair->m_Xouts[k] = m_nodes[nIdx].m_x;
		}
	}
}

// void btCable::contactConstraint(btAlignedObjectArray<NodePairNarrowPhase>* nodePairContact, btAlignedObjectArray<int>* indexNodeContact)
// {
// 	int nbSubStep = m_subIterationCollision;
// 	int nbContactPairPotential = nodePairContact->size();
// 	if (nbContactPairPotential == 0) return;
//
// 	btVector3 positionStart;
// 	btVector3 positionEnd;
//
// 	Node* n;
// 	btScalar marginNode = m_collisionMargin;
// 	int indexNode;
//
// 	btCollisionObject* obj;
// 	btCollisionShape* shape;
// 	btScalar margin;
//
// 	bool finish = false;
//
// 	setupNodeForCollision(indexNodeContact);
//
// 	// Compute force application
// 	for (int j = 0; j < nbSubStep; j++)
// 	{
// 		bool has1contact = false;
//
// 		// Setup structures
// 		resetNormalAndHitPosition(indexNodeContact);
//
// 		// Collision Resolution
// 		for (int i = 0; i < nbContactPairPotential; i++)
// 		{
// 			NodePairNarrowPhase* temp = &nodePairContact->at(i);
// 			temp->hitInIteration = false;
// 			n = temp->node;
//
// 			// Update contact index for in the node structure
// 			updateContactPos(n, i, j);
//
// 			// Check if collision is possible
// 			if (!n->computeNodeConstraint || !checkCondition(n, j))
// 			{
// 				continue;
// 			}
//
// 			obj = temp->pair->body;
// 			shape = temp->collisionShape;
// 			indexNode = n->index;
//
// 			// SphereShape margin is sphereShape Radius, not a safe margin
// 			margin = computeCollisionMargin(shape);
//
// 			positionStart = temp->m_Xout;
// 			positionEnd = n->m_x;
// 			btScalar len = positionStart.distance(positionEnd);
// 			if (len < m_collisionSleepingThreshold + FLT_EPSILON)
// 			{
// 				continue;
// 			}
//
// 			btCollisionWorld::ClosestRayResultCallback m_resultCallback = castRay(positionStart, positionEnd, temp, margin);
// 			if (m_resultCallback.hasHit())
// 			{
// 				n->collide = true;
// 				if (n->topMargin < margin)
// 					n->topMargin = margin;
//
// 				btVector3 contact = m_resultCallback.m_hitPointWorld;
// 				btVector3 normal = m_resultCallback.m_hitNormalWorld;
//
// 				setRayResult(positionStart, contact, normal, n, temp);
//
// 				// Calculate & Apply the impulse for colllided object
// 				if (obj->getInternalType() == CO_RIGID_BODY && impulseCompute)
// 				{
// 					btRigidBody* rb = btRigidBody::upcast(obj);
// 					temp->impulse = calculateBodyImpulse(rb, marginNode, n, normal, contact);
// 				}
// 			}
// 		}
//
// 		// Update position
// 		Node* node;
// 		for (int i = 0; i < indexNodeContact->size(); i++)
// 		{
// 			float nbCorrection = m_nodes.at(indexNodeContact->at(i)).m_nbCollidingObjectInFrame;
// 			if (nbCorrection > 0)
// 			{
// 				has1contact = true;
// 				node = &m_nodes.at(indexNodeContact->at(i));
// 				btVector3 newPos = node->positionCollision / nbCorrection;
//
// 				if (nbCorrection == 1)
// 				{
// 					if (node->m_nbCollidingObjectPotential == 2)
// 					{
// 						int contactIndex;
// 						int position = -1;
// 						if (nodePairContact->at(node->narrowPhaseIndex[0]).hitInIteration == false)
// 						{
// 							contactIndex = node->narrowPhaseIndex[0];
// 							position = 1;
// 						}
// 						else
// 						{
// 							contactIndex = node->narrowPhaseIndex[1];
// 							position = 0;
// 						}
//
// 						NodePairNarrowPhase* contact = &nodePairContact->at(contactIndex);
//
// 						margin = computeCollisionMargin(shape);
// 						btCollisionWorld::ClosestRayResultCallback m_resultCallback = castRay(node->hitPosition[0], newPos, contact, margin);
//
// 						// Cast ray from m_x to newPos;
// 						if (m_resultCallback.hasHit())
// 						{
// 							// That means we replace the node in an other body
// 							btVector3 contactPoint = m_resultCallback.m_hitPointWorld;
// 							btVector3 normal = m_resultCallback.m_hitNormalWorld;
//
// 							node->normals[1] = normal;
// 							node->hitPosition[1] = contactPoint;
//
// 							btVector3 temp = fastTrigoPositionCompute(node);
// 							newPos = temp;
// 						}
// 					}
// 				}
// 				else
// 				{
// 					if (nbCorrection == 2)
// 					{
// 						btScalar dist = (node->hitPosition[0] - node->hitPosition[1]).length();
//
// 						btVector3 temp = fastTrigoPositionCompute(node);
// 						newPos = temp;
// 					}
// 				}
//
// 				node->m_x = newPos;
// 				node->positionCollision = btVector3(0, 0, 0);
// 				node->collide = true;
// 				node->collideInAllIteration = true;
// 			}
// 		}
//
// 		// Force Apply
// 		if (impulseCompute)
// 		{
// 			for (int i = 0; i < nbContactPairPotential; i++)
// 			{
// 				NodePairNarrowPhase* temp = &nodePairContact->at(i);
// 				if (temp->node->m_nbCollidingObjectInFrame > 0)
// 				{
// 					obj = temp->pair->body;
// 					btTransform wtr = obj->getWorldTransform();
// 					btVector3 impulse = temp->impulse;
// 					temp->impulse = btVector3(0, 0, 0);
// 					if (obj->getInternalType() == CO_RIGID_BODY)
// 					{
// 						btRigidBody* rb = btRigidBody::upcast(obj);
// 						rb->applyRedirectionImpulse(impulse, temp->node->m_x);
// 					}
// 				}
// 			}
// 		}
//
// 		if (!has1contact)
// 		{
// 			break;
// 		}
// 	}
//
// 	// Update the new ray starting position for the next solver step
// 	NodePairNarrowPhase* temp;
// 	for (int i = 0; i < nbContactPairPotential; i++)
// 	{
// 		temp = &nodePairContact->at(i);
// 		if (nodePairContact->at(i).node->collide)
// 		{
// 			temp->m_Xout = temp->node->m_x;
// 		}
// 	}
// }

// void btCable::setupNodeForCollision(btAlignedObjectArray<int>* indexNodeContact)
// {
// 	Node* n;
// 	for (int i = 0; i < indexNodeContact->size(); i++)
// 	{
// 		n = &m_nodes.at(indexNodeContact->at(i));
// 		n->collide = false;
// 		n->posBeforeCollision = n->m_x;
// 		for (int nbCollide = 0; nbCollide < n->m_nbCollidingObjectPotential; nbCollide++)
// 		{
// 			n->narrowPhaseIndex[nbCollide] = -1;
// 		}
// 		n->topMargin = 0;
// 	}
// }

void btCable::solveContactLimited(btAlignedObjectArray<NodePairNarrowPhase>* nodePairContact, btAlignedObjectArray<CollisionState>* collisionStates, int limitLow, int limitHigh)
{
	int nbSubStep = m_subIterationCollision;
	int nbContactPairPotential = nodePairContact->size();
	if (nbContactPairPotential == 0) return;

	btVector3 positionStart;
	btVector3 positionEnd;

	btCollisionShape* shape;
	btScalar margin;

	for (int i = limitLow; i < limitHigh; i++)
	{
		CollisionState* state = &collisionStates->at(i);
		Node* n = &m_nodes.at(i);
		// n->collide = false;
		state->posBefore  = n->m_x;
		// if (n->m_nbCollidingObjectPotential > 0)
		// {
		// 	for (int nbCollide = 0; nbCollide < n->m_nbCollidingObjectPotential; nbCollide++)
		// 		n->narrowPhaseIndex[nbCollide] = -1;
		// }

		state->topMargin = 0;
	}

	// Compute force application
	for (int j = 0; j < nbSubStep; j++)
	{
		bool has1contact = false;

		// Setup structures
		Node* n;
		for (int i = limitLow; i < limitHigh; i++)
		{
			CollisionState * state = &collisionStates->at(i);
			n = &m_nodes.at(i);
			n->m_nbCollidingObjectInFrame = 0;
			for (int nbCollide = 0; nbCollide < n->m_nbCollidingObjectPotential; nbCollide++)
			{
				state->normals[nbCollide] = btVector3{0, 0, 0};
				state->hits[nbCollide] = btVector3{0, 0, 0};
			}
		}

		// Collision Resolution
		for (int i = 0; i < nbContactPairPotential; i++)
		{
			NodePairNarrowPhase* temp = &nodePairContact->at(i);
			temp->hitInIteration = false;
			n = temp->node;

			if (n->index < limitLow || n->index >= limitHigh)
				continue;
			// Update contact index for in the node structure
			updateContactPos(n, i, j);

			// Check if collision is possible
			if (!checkCondition(n, j))
				continue;
			
			shape = temp->pair->body->getCollisionShape();

			// SphereShape margin is sphereShape Radius, not a safe margin
			margin = computeCollisionMargin(shape);

			positionStart = temp->m_Xout;
			positionEnd = n->m_x;
			btScalar len = positionStart.distance(positionEnd);
			if (len < m_collisionSleepingThreshold + FLT_EPSILON)
			{
				continue;
			}

			btCollisionWorld::ClosestRayResultCallback m_resultCallback = castRay(positionStart, positionEnd, temp, TODO,margin);
			if (m_resultCallback.hasHit())
			{
				n->collide = true;
				if (n->topMargin < margin)
					n->topMargin = margin;

				btVector3 contact = result.m_hitPointWorld;
				btVector3 normal = result.m_hitNormalWorld;

				setRayResult(positionStart, contact, normal, n, temp,TODO);
			}
		}

		// Update position
		Node* node;
		for (int i = limitLow; i < limitHigh; i++)
		{
			float nbCorrection = m_nodes.at(i).m_nbCollidingObjectInFrame;
			if (nbCorrection > 0)
			{
				has1contact = true;
				node = &m_nodes.at(i);
				btVector3 newPos = node->positionCollision / nbCorrection;

				if (nbCorrection == 1)
				{
					if (node->m_nbCollidingObjectPotential == 2)
					{
						int contactIndex;
						int position = -1;
						if (nodePairContact->at(node->narrowPhaseIndex[0]).hitInIteration == false)
						{
							contactIndex = node->narrowPhaseIndex[0];
							position = 1;
						}
						else
						{
							contactIndex = node->narrowPhaseIndex[1];
							position = 0;
						}

						NodePairNarrowPhase* contact = &nodePairContact->at(contactIndex);

						margin = computeCollisionMargin(shape);
						btCollisionWorld::ClosestRayResultCallback m_resultCallback = castRay(node->hitPosition[0], newPos, contact, TODO,margin);

						// Cast ray from m_x to newPos;
						if (result.hasHit())
						{
							// That means we replace the node in an other body
							btVector3 contact = result.m_hitPointWorld;
							btVector3 normal = result.m_hitNormalWorld;

							node->normals[1] = normal;
							node->hitPosition[1] = contact;

							btVector3 temp = fastTrigoPositionCompute(node);
							newPos = temp;
						}
					}
				}
				else
				{
					if (nbCorrection == 2)
					{
						//btScalar dist = (node->hitPosition[0] - node->hitPosition[1]).length();

						btVector3 temp = fastTrigoPositionCompute(node);
						newPos = temp;
					}
				}

				node->m_x = newPos;
				//node->positionCollision = btVector3(0, 0, 0);
				//node->collide = true;
				node->collideInAllIteration = true;
			}
		}

		if (!has1contact)
		{
			break;
		}
	}

	// Update the new ray starting position for the next solver step
	NodePairNarrowPhase* temp;
	for (int i = 0; i < nbContactPairPotential; i++)
	{
		temp = &nodePairContact->at(i);
		for (int j = 0; j < temp->numContacts; j++) {
			if (nodePairContact->at(i).node->index >= limitLow && nodePairContact->at(i).node->index < limitHigh)
			{
				temp->m_Xouts[j] = temp->node->m_x;
			}
		}
	}
}

void btCable::resolveConflitZone(btAlignedObjectArray<NodePairNarrowPhase>* nodePairContact, btAlignedObjectArray<CollisionState>* collisionStates)
{
	Node* node;
	int distSectorMax = 5;
	Node* nodeAfter;

	for (int i = 0; i < m_nodes.size() - 1; i++)
	{
		node = &m_nodes.at(i);

		if (node->collideInAllIteration)
		{
			int limitMin = btMax(i - distSectorMax, 0);
			int limitMax = i;
			int indexDist = distSectorMax;

			bool continousSector = true;

			while (continousSector && i < m_nodes.size() - 1)
			{
				i++;
				nodeAfter = &m_nodes.at(i);

				if (!nodeAfter->collideInAllIteration)
				{
					indexDist--;
				}
				else
				{
					indexDist = distSectorMax;	
				}

				if (indexDist == 0)
				{
					continousSector = false;
					limitMax = i;
				}
			}

			for (int j = 0; j < 5; j++)
			{
				distanceConstraintLock(limitMin, limitMax);
				solveContactLimited(nodePairContact, collisionStates, limitMin,limitMax);
			}
		}
	}
}

void btCable::clearManifoldContact()
{
	int count = manifolds.size();
	// Remove the body that have no contact point and lifeTime = 0
	for (int i = 0; i < count; i++)
	{
		if (manifolds.at(i).manifold->getNumContacts() <= 0)
		{
			manifolds.at(i).lifeTime--;

			if (manifolds.at(i).lifeTime <= 0)
			{
				m_world->getDispatcher()->releaseManifold(manifolds.at(i).manifold);
				manifolds.removeAtIndex(i);

				count--;
				i--;
			}
		}
	}
}

// void btCable::recursiveBroadPhase(BroadPhasePair* obj, Node* n, btCollisionShape* shape, btAlignedObjectArray<NodePairNarrowPhase>* nodePairContact, btAlignedObjectArray<int>* nodeIndexOut,
// 								  btVector3 minLink, btVector3 maxLink, btTransform* worldTransform)
// {
// 	if (shape->getShapeType() == COMPOUND_SHAPE_PROXYTYPE)
// 	{
// 		btCompoundShape* compound = static_cast<btCompoundShape*>(shape);
// 		for (int i = 0; i < compound->getNumChildShapes(); i++)
// 		{
// 			btTransform newTransform = btTransform();
// 			newTransform.mult(*worldTransform, compound->getChildTransform(i));
// 			recursiveBroadPhase(obj, n, compound->getChildShape(i), nodePairContact, nodeIndexOut, minLink, maxLink, &newTransform);
// 		}
// 	}
// 	else
// 	{
// 		btVector3 mins, maxs;
// 		shape->getAabb(*worldTransform, mins, maxs);
// 		// Intersect box
// 		if (minLink.x() <= maxs.x() && maxLink.x() >= mins.x() &&
// 			minLink.y() <= maxs.y() && maxLink.y() >= mins.y() &&
// 			minLink.z() <= maxs.z() && maxLink.z() >= mins.z())
// 		{
// 			// Pre-calculate the position for the contact pair test
// 			btRigidBody* rb = static_cast<btRigidBody*>(obj->body);
// 			btVector3 rbVelocity = rb->getVelocityInLocalPoint(n->m_q - rb->getWorldTransform().getOrigin());
// 			btVector3 nodePosition = n->m_q + rbVelocity * m_sst.sdt;
//
// 			// Create a SphereShape to simulate the collision between the node and a rigidbody
// 			btSphereShape nodeShape = btSphereShape(m_collisionMargin);
// 			btCollisionObject nodeCollision = btCollisionObject();
// 			btTransform nodeTransform = btTransform::getIdentity();
// 			nodeTransform.setOrigin(nodePosition);
// 			nodeCollision.setWorldTransform(nodeTransform);
// 			nodeCollision.setCollisionShape(&nodeShape);
//
// 			// Simulate the collision
// 			MyContactResultCallback result(0, &nodeCollision, obj->body);
// 			m_world->contactPairTest(&nodeCollision, obj->body, result);
//
// 			if (!result.m_connected)
// 			{
// 				return;
// 			}
//
// 			n->m_nbCollidingObjectPotential++;
//
// 			// Register the collisions' results
// 			NodePairNarrowPhase nodePair = NodePairNarrowPhase();
// 			nodePair.node = n;
// 			nodePair.pair = obj;
// 			nodePair.collisionShape = shape;
// 			nodePair.worldTransform = *worldTransform;
// 			nodePair.node->m_nbCollidingObjectPotential++;
// 			nodePair.m_Xout = result.m_connected ? result.contactPoint + result.contactNorm * m_collisionMargin : nodePosition;
// 			nodePair.normal = result.m_connected ? result.contactNorm : btVector3(0, 0, 0);
// 			nodePair.haveManifoldsRegister = false;
// 			nodePairContact->push_back(nodePair);
//
// 			nodeIndexOut->push_back(n->index);
//
// 			//m_world->getDebugDrawer()->drawSphere(nodePair.m_Xout, m_collisionMargin * 2.0, btVector3(1, 0, 1));
// 		}
// 	}
// }

// void btCable::resetNormalAndHitPosition(btAlignedObjectArray<int>* indexNodeContact)
// {
// 	Node* n;
// 	for (int i = 0; i < indexNodeContact->size(); i++)
// 	{
// 		n = &m_nodes.at(indexNodeContact->at(i));
// 		n->m_nbCollidingObjectInFrame = 0;
// 		for (int nbCollide = 0; nbCollide < n->m_nbCollidingObjectPotential; nbCollide++)
// 		{
// 			n->normals[nbCollide] = btVector3(0, 0, 0);
// 			n->hitPosition[nbCollide] = btVector3(0, 0, 0);
// 		}
// 	}
// }

// void btCable::updateContactPos(Node* n, int position, int step)
// {
// 	if (step != 0) return;
// 	for (int place = 0; place < n->m_nbCollidingObjectPotential; place++)
// 	{
// 		if (n->narrowPhaseIndex[place] == -1)
// 		{
// 			n->narrowPhaseIndex[place] = position;
// 			break;
// 		}
// 	}
// }

// bool btCable::checkCondition(Node* n, int step)
// {
// 	if (m_collisionMargin <= 0 || n->m_battach != 0)
// 	{
// 		return false;
// 	}
//
// 	// If the node only collide 1 body we don t have to update nbSubStep times
//
// 	if (step != 0)
// 	{
// 		if (n->m_nbCollidingObjectPotential == 1) return false;
// 		return n->collide;
// 	}
// 	return true;
// }

btCollisionWorld::ClosestRayResultCallback btCable::castRay(btVector3 positionStart, btVector3 positionEnd, NodePairNarrowPhase* contact, btScalar margin)
{
	btScalar distanceOut = 0.003;
	btVector3 dir = positionStart.distance(positionEnd) > FLT_EPSILON ? (positionEnd - positionStart).normalized() : btVector3(0, 0, 0);
	btVector3 startRay = positionStart - dir * distanceOut;
	btVector3 endRay = positionEnd;

	btTransform rayFrom = btTransform::getIdentity();
	rayFrom.setOrigin(startRay);
	btTransform rayTo   = btTransform::getIdentity();
	rayTo.setOrigin(endRay);

	btCollisionWorld::ClosestRayResultCallback result(startRay, endRay);
	if (startRay.distance(endRay) > FLT_EPSILON) {
		m_world->rayTestSingleWithMargin(
		    rayFrom, rayTo,
		    contact->pair->body,
		    contact->pair->body->getCollisionShape(),
		    contact->worldTransform,
		    result,
		    margin
		);
	}
	return result;
}

btScalar btCable::computeCollisionMargin(btCollisionShape* shape)
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

	// btScalar ima = n->m_im;
	// btScalar imb = obj->getInvMass();
	// if (imb == 0) return btVector3(0, 0, 0);
	// btScalar totalMass = ima + imb;

	// bodyToNodeVector
	btVector3 ra = hitPosition - wtr.getOrigin();
	btVector3 vBody = obj->getVelocityInLocalPoint(ra);
	btVector3 vNode = (n->m_x - n->m_q) * 0.75 / dt;
	btVector3 vRelative = vNode - vBody;
	btScalar vRelativeOnNormal = btDot(vRelative, normal);

	// Friction value
	// btVector3 vRelativeTangent = vRelative - (normal * vRelativeOnNormal);
	// btVector3 tangentDir = vRelativeTangent.normalized();
	//
	// int frictionCoef = obj->getFriction() * getFriction();
	//
	// // Part of vRelativeOnTangentDir
	// btScalar jt = -vRelative.dot(tangentDir) * frictionCoef;
	// jt = jt / totalMass;

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

		const btVector3 impulse = ((responseVector * normal) /*- (tangentDir * jt * m_sst.isdt)*/) * dt;
		return impulse;
	}

	if (collisionMode == CollisionMode::Curve)

	{
		if (!spline) return btVector3(0, 0, 0);

		penetrationDistance *= 1.0 / m_substepDelayCollision;
		btScalar k = spline->eval(penetrationDistance);
		if (isnan(k))
		{
			return btVector3(0, 0, 0);
		}
		btScalar responseVector = -k * penetrationDistance + viscosityCoef * vRelativeOnNormal;

		const btVector3 impulse = ((responseVector * normal) /*- (tangentDir * jt * m_sst.isdt)*/) * dt;
		return impulse * m_substepDelayCollision;
	}

	return btVector3(0, 0, 0);
}

btVector3 btCable::fastTrigoPositionCompute(CollisionState* state)
{
	btScalar PI = SIMD_PI;
	btScalar angle;
	btVector3 n0, n1;
	btVector3 newPos = (state->hits[0] + state->hits[1]) * 0.5;

	n0 = state->normals[0];
	n1 = state->normals[1];

	btScalar dotProduct = Clamp(n0.dot(n1), -1.0, 1.0);
	angle = PI - acos(dotProduct);
	btScalar sinA = sin(angle);
	// if sinA is 0, the correction cannot be calculated with sinA
	if (abs(sinA) < FLT_EPSILON)
	{
		return state->hits[0];
	}

	btVector3 moyDirection = ((n0 + n1) * 0.5);
	if (moyDirection.fuzzyZero())
	{
		return newPos;
	}
	moyDirection = moyDirection.normalized();

	btScalar a = state->topMargin;
	btScalar B = 0.5 * PI - (angle * 0.5);
	btScalar b = (a * 0.5) / (sinA * 0.5) * sin(B);

	btVector3 correction = b * moyDirection;
	newPos += correction;

	return newPos;
}

// btVector3 btCable::fastTrigoPositionCompute(Node* n)
// {
// 	btScalar PI = SIMD_PI;
// 	btScalar angle;
// 	btVector3 n0, n1;
// 	btVector3 newPos = (n->hitPosition[0] + n->hitPosition[1]) * 0.5;
//
// 	n0 = n->normals[0];
// 	n1 = n->normals[1];
//
// 	btScalar dotProduct = Clamp(n0.dot(n1), -1.0, 1.0);
// 	angle = PI - acos(dotProduct);
// 	btScalar sinA = sin(angle);
// 	// if sinA is 0, the correction cannot be calculated with sinA
// 	if (abs(sinA) < FLT_EPSILON)
// 	{
// 		return n->hitPosition[0];
// 	}
//
// 	btVector3 moyDirection = ((n0 + n1) * 0.5);
// 	if (moyDirection.fuzzyZero())
// 	{
// 		return newPos;
// 	}
// 	moyDirection = moyDirection.normalized();
//
// 	btScalar a = n->topMargin;
// 	btScalar B = 0.5 * PI - (angle * 0.5);
// 	btScalar b = (a * 0.5) / (sinA * 0.5) * sin(B);
//
// 	btVector3 correction = b * moyDirection;
// 	newPos += correction;
//
// 	return newPos;
// }

// void btCable::setRayResult(const btVector3 &positionStart,
//     btVector3 contactPoint,
//     const btVector3 &normal,
//     Node* n,
//     NodePairNarrowPhase* temp,
//     int rayIndex)
// {
// 	btScalar backOff = 0.0005;
// 	btVector3 outShift = normal * backOff;
//
// 	btVector3 touchDir = (positionStart.distance(contactPoint) > FLT_EPSILON)
// 	    ? (positionStart - contactPoint).normalized()
// 	    : btVector3(0, 0, 0);
// 	btScalar cosTheta = touchDir.dot(normal);
// 	btScalar theta    = acos(cosTheta);
// 	btScalar deltaPos = theta * backOff;
//
// 	btVector3 oppose = (positionStart.distance(contactPoint + outShift) > FLT_EPSILON)
// 	    ? (positionStart - (contactPoint + outShift)).normalized()
// 	    : btVector3(0, 0, 0);
// 	btVector3 correction = oppose * (deltaPos * 0.5);
// 	btVector3 newPos     = contactPoint + outShift + correction;
//
// 	btScalar offset   = (n->m_x - contactPoint).length();
// 	btScalar penDepth = (contactPoint * offset).length();
//
// 	// Update the Node
// 	n->positionCollision += newPos;
// 	int idx = n->m_nbCollidingObjectInFrame++;
// 	n->normals[idx] = normal;
// 	n->hitPosition[idx] = contactPoint;
//
// 	// Update the per-ray data
// 	temp->hit            = true;
// 	temp->hitInIteration = true;
// 	temp->normals[rayIndex] = normal;
// 	temp->lastPosition[rayIndex] = contactPoint;
// 	temp->distances[rayIndex] = penDepth;
// }

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

void btCable::setCollisionParameters(int substepDelayCollision, int subIterationCollision, btScalar collisionSleepingThreshold)
{
	m_substepDelayCollision = substepDelayCollision;
	m_subIterationCollision = subIterationCollision;
	m_collisionSleepingThreshold = collisionSleepingThreshold;
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