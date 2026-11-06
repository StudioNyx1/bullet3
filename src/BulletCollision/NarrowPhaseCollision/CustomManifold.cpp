#include "CustomManifold.h"

CustomManifoldPoint::CustomManifoldPoint(btManifoldPoint* point)
{
	m_positionWorldOnA = point->getPositionWorldOnA();
	m_positionWorldOnB = point->getPositionWorldOnB();
	m_impulse = point->m_frameImpulse; // whole frame (sum over sub-steps), not only the last sub-step
	m_depth = point->getDistance();
	m_normal = point->getNormalWorldOnB();
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
	int count = manifold->getNumContacts();
	for (int i = 0; i < count; i++)
	{
		CustomManifoldPoint manifoldPoint = CustomManifoldPoint(&manifold->getContactPoint(i));
		m_points.push_back(manifoldPoint);
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
