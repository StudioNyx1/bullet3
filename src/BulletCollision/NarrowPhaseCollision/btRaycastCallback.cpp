/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  https://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

//#include <stdio.h>

#include "BulletCollision/CollisionShapes/btConvexShape.h"
#include "BulletCollision/CollisionShapes/btTriangleShape.h"
#include "BulletCollision/NarrowPhaseCollision/btSubSimplexConvexCast.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkConvexCast.h"
#include "BulletCollision/NarrowPhaseCollision/btContinuousConvexCollision.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.h"
#include "btRaycastCallback.h"

btTriangleRaycastCallback::btTriangleRaycastCallback(const btVector3& from, const btVector3& to, unsigned int flags)
	: m_from(from), m_to(to), m_flags(flags), m_hitFraction(1.0), m_margin(0.0) { }

void btTriangleRaycastCallback::processTriangle(btVector3* triangle,
                                                int partId, int triangleIndex)
{
    const btVector3& v0 = triangle[0];
    const btVector3& v1 = triangle[1];
    const btVector3& v2 = triangle[2];

    btVector3 v10 = v1 - v0;
    btVector3 v20 = v2 - v0;
    btVector3 n   = v10.cross(v20);
    const btScalar nLen   = n.length();
    const btScalar planeD = v0.dot(n);

    btScalar distA = n.dot(m_from) - planeD;
    btScalar distB = n.dot(m_to) - planeD;

    // Margin for the plane test
    const btScalar planeMargin = (m_margin + FLT_EPSILON) * nLen;

    if ((distA >  planeMargin && distB >  planeMargin) ||
        (distA < -planeMargin && distB < -planeMargin))
        return;

    // distance along the segment to where we ENTER the slab
    const btScalar denom = distA - distB;
    if (btFabs(denom) < SIMD_EPSILON) return;

    btScalar target = 0.0f;
    if (distA >  planeMargin)
        target =  planeMargin;
    else
        if (distA < -planeMargin)
            target = -planeMargin;

    const btScalar distance = (distA - target) / denom;
    if (distance >= m_hitFraction) return;

    // original inside-triangle tests
    btVector3 point = m_from.lerp(m_to, distance);

    btScalar edge_tolerance = n.length2();
    edge_tolerance *= btScalar(-FLT_EPSILON);

    btVector3 v0p = v0 - point;
    btVector3 v1p = v1 - point;
    btVector3 cp0 = v0p.cross(v1p);
    if (cp0.dot(n) < edge_tolerance) return;

    btVector3 v2p = v2 - point;
    btVector3 cp1 = v1p.cross(v2p);
    if (cp1.dot(n) < edge_tolerance) return;

    btVector3 cp2 = v2p.cross(v0p);
    if (cp2.dot(n) < edge_tolerance) return;

    // normalize for reporting
    if (nLen > SIMD_EPSILON) n /= nLen;

    if (((m_flags & kF_KeepUnflippedNormal) == 0) && (distA <= btScalar(0.0)))
        m_hitFraction = reportHit(-n, distance, partId, triangleIndex);
    else
        m_hitFraction = reportHit( n, distance, partId, triangleIndex);
}

btTriangleConvexcastCallback::btTriangleConvexcastCallback(const btConvexShape* convexShape, const btTransform& convexShapeFrom, const btTransform& convexShapeTo, const btTransform& triangleToWorld, const btScalar triangleCollisionMargin)
{
	m_convexShape = convexShape;
	m_convexShapeFrom = convexShapeFrom;
	m_convexShapeTo = convexShapeTo;
	m_triangleToWorld = triangleToWorld;
	m_hitFraction = 1.0f;
	m_triangleCollisionMargin = triangleCollisionMargin;
	m_allowedPenetration = 0.f;
}

void btTriangleConvexcastCallback::processTriangle(btVector3* triangle, int partId, int triangleIndex)
{
	btTriangleShape triangleShape(triangle[0], triangle[1], triangle[2]);
	triangleShape.setMargin(m_triangleCollisionMargin);

	btVoronoiSimplexSolver simplexSolver;
	btGjkEpaPenetrationDepthSolver gjkEpaPenetrationSolver;

//#define  USE_SUBSIMPLEX_CONVEX_CAST 1
//if you reenable USE_SUBSIMPLEX_CONVEX_CAST see commented out code below
#ifdef USE_SUBSIMPLEX_CONVEX_CAST
	btSubsimplexConvexCast convexCaster(m_convexShape, &triangleShape, &simplexSolver);
#else
	//btGjkConvexCast	convexCaster(m_convexShape,&triangleShape,&simplexSolver);
	btContinuousConvexCollision convexCaster(m_convexShape, &triangleShape, &simplexSolver, &gjkEpaPenetrationSolver);
#endif  //#USE_SUBSIMPLEX_CONVEX_CAST

	btConvexCast::CastResult castResult;
	castResult.m_fraction = btScalar(1.);
	castResult.m_allowedPenetration = m_allowedPenetration;
	if (convexCaster.calcTimeOfImpact(m_convexShapeFrom, m_convexShapeTo, m_triangleToWorld, m_triangleToWorld, castResult))
	{
		//add hit
		if (castResult.m_normal.length2() > btScalar(0.0001))
		{
			if (castResult.m_fraction < m_hitFraction)
			{
				/* btContinuousConvexCast's normal is already in world space */
				/*
#ifdef USE_SUBSIMPLEX_CONVEX_CAST
				//rotate normal into worldspace
				castResult.m_normal = m_convexShapeFrom.getBasis() * castResult.m_normal;
#endif //USE_SUBSIMPLEX_CONVEX_CAST
*/
				castResult.m_normal.normalize();

				reportHit(castResult.m_normal,
						  castResult.m_hitPoint,
						  castResult.m_fraction,
						  partId,
						  triangleIndex);
			}
		}
	}
}
