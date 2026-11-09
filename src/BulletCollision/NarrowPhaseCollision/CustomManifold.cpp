#include "CustomManifold.h"

CustomManifoldPoint::CustomManifoldPoint(btManifoldPoint* point)
{
	m_positionWorldOnA = point->getPositionWorldOnA();
	m_positionWorldOnB = point->getPositionWorldOnB();
	m_impulse = point->m_frameImpulse; // whole frame (sum over sub-steps), not only the last sub-step
	m_depth = point->getDistance();
	m_normal = point->getNormalWorldOnB();
}

bool CustomManifoldPoint::isSameContact(const CustomManifoldPoint& other, bool swapped) const
{
	// Tolerances: 0.1 mm on the positions, ~2.5 degrees on the normal
	const btScalar maxDist2 = btScalar(1e-4) * btScalar(1e-4);
	const btScalar minNormalDot = btScalar(0.999);

	const btVector3& otherOnA = swapped ? other.m_positionWorldOnB : other.m_positionWorldOnA;
	const btVector3& otherOnB = swapped ? other.m_positionWorldOnA : other.m_positionWorldOnB;
	const btVector3 otherNormal = swapped ? -other.m_normal : other.m_normal;

	return (m_positionWorldOnA - otherOnA).length2() < maxDist2
		&& (m_positionWorldOnB - otherOnB).length2() < maxDist2
		&& m_normal.dot(otherNormal) > minNormalDot;
}

void CustomManifoldPoint::merge(const CustomManifoldPoint& other)
{
	m_impulse += other.m_impulse;
	if (other.m_depth < m_depth)
	{
		m_depth = other.m_depth;
	}
}

CustomManifold::CustomManifold(btPersistentManifold* manifold)
{
	m_body0 = manifold->getBody0();
	m_body1 = manifold->getBody1();

	m_points = {};
	addPoint(manifold);
}
	

void CustomManifold::addPoint(btPersistentManifold* manifold)
{
	// Body A/B of this manifold may be inverted compared to the cached pair
	const bool swapped = manifold->getBody0() != m_body0;

	int count = manifold->getNumContacts();
	for (int i = 0; i < count; i++)
	{
		CustomManifoldPoint manifoldPoint = CustomManifoldPoint(&manifold->getContactPoint(i));

		// Compound vs compound: one manifold per child shape pair, so overlapping (or duplicated)
		// child shapes produce the same contact several times. Keep it once, with the summed impulse.
		bool merged = false;
		for (size_t j = 0; j < m_points.size(); j++)
		{
			if (m_points[j].isSameContact(manifoldPoint, swapped))
			{
				m_points[j].merge(manifoldPoint);
				merged = true;
				break;
			}
		}

		if (!merged)
		{
			m_points.push_back(manifoldPoint);
		}
	}

	removeZeroImpulsePoints();
}

// Keep only the points that received an impulse during the frame. If none did (e.g. bodies only
// touching or separating), keep the deepest one so the pair is never empty.
// Called after each addPoint: the points of all the manifolds of the pair are taken into account.
void CustomManifold::removeZeroImpulsePoints()
{
	// Below this, the impulse is solver noise (e.g. 1e-10)
	const btScalar minImpulse = btScalar(1e-6);

	std::vector<CustomManifoldPoint> kept;
	int deepest = -1;
	for (size_t i = 0; i < m_points.size(); i++)
	{
		if (btFabs(m_points[i].GetImpulse()) > minImpulse)
		{
			kept.push_back(m_points[i]);
		}
		else if (deepest < 0 || m_points[i].GetDepth() < m_points[deepest].GetDepth())
		{
			deepest = int(i);
		}
	}

	if (kept.empty() && deepest >= 0)
	{
		kept.push_back(m_points[deepest]);
	}

	m_points.swap(kept);
}
