#include "CustomManifold.h"

CustomManifoldPoint::CustomManifoldPoint(btManifoldPoint* point)
{
	m_positionWorldOnA = point->getPositionWorldOnA();
	m_positionWorldOnB = point->getPositionWorldOnB();
	m_impulse = point->getAppliedImpulse();
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
}
