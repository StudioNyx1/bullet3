#pragma once

#include "LinearMath/btVector3.h"
#include <btBulletCollisionCommon.h>
#include <vector>

class CustomManifoldPoint
{
private:
	btVector3 m_positionWorldOnA;
	btVector3 m_positionWorldOnB;
	btVector3 m_normal;
	btScalar m_impulse;
	btScalar m_depth;
	
public:
	CustomManifoldPoint(btManifoldPoint* point);

	btVector3 GetPositionWorldOnA() const
	{
		return m_positionWorldOnA;
	}

	btVector3 GetPositionWorldOnB() const
	{
		return m_positionWorldOnB;
	}

	btScalar GetImpulse() const
	{
		return m_impulse;
	}

	btScalar GetDepth() const
	{
		return m_depth;
	}

	btVector3 GetNormal() const
	{
		return m_normal;
	}
};

class CustomManifold
{
private:
	const btCollisionObject* m_body0;
	const btCollisionObject* m_body1;
	std::vector<CustomManifoldPoint> m_points;

public:
	CustomManifold(btPersistentManifold* manifold);

	void addPoint(btPersistentManifold* point);

	int getCount()
	{
		return m_points.size();
	}

	CustomManifoldPoint* getManifoldPoint(int index)
	{
		return &m_points[index];
	}

	const btCollisionObject* getBody0()
	{
		return m_body0;
	}

	const btCollisionObject* getBody1()
	{
		return m_body1;
	}
};

