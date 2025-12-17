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

#include "btRigidBody.h"
#include "BulletCollision/CollisionShapes/btConvexShape.h"
#include "LinearMath/btMinMax.h"
#include "LinearMath/btTransformUtil.h"
#include "LinearMath/btMotionState.h"
#include "BulletDynamics/ConstraintSolver/btTypedConstraint.h"
#include "LinearMath/btSerializer.h"
#include <btBulletDynamicsCommon.h>

//'temporarily' global variables
btScalar gDeactivationTime = btScalar(2.);
bool gDisableDeactivation = false;
static int uniqueId = 0;


btScalar btLerp(const btScalar& a, const btScalar& b, btScalar t)
{
	return a + (b - a) * t;
}

btRigidBody::btRigidBody(const btRigidBody::btRigidBodyConstructionInfo& constructionInfo)
{
	setupRigidBody(constructionInfo);
}

btRigidBody::btRigidBody(btScalar mass, btMotionState* motionState, btCollisionShape* collisionShape, const btVector3& localInertia)
{
	btRigidBodyConstructionInfo cinfo(mass, motionState, collisionShape, localInertia);
	setupRigidBody(cinfo);
}

void btRigidBody::setupRigidBody(const btRigidBody::btRigidBodyConstructionInfo& constructionInfo)
{
	m_internalType = CO_RIGID_BODY;

	m_linearVelocity.setValue(btScalar(0.0), btScalar(0.0), btScalar(0.0));
	m_lastAcceleration.setValue(btScalar(0.0), btScalar(0.0), btScalar(0.0));
	m_angularVelocity.setValue(btScalar(0.), btScalar(0.), btScalar(0.));
	m_lastAngularAcceleration.setValue(btScalar(0.0), btScalar(0.0), btScalar(0.0));
	m_angularFactor.setValue(1, 1, 1);
	m_linearFactor.setValue(1, 1, 1);
	m_gravity.setValue(btScalar(0.0), btScalar(0.0), btScalar(0.0));
	m_gravity_acceleration.setValue(btScalar(0.0), btScalar(0.0), btScalar(0.0));
	m_totalForce.setValue(btScalar(0.0), btScalar(0.0), btScalar(0.0));
	m_lastTotalForce.setValue(btScalar(0.0), btScalar(0.0), btScalar(0.0));
	m_totalTorque.setValue(btScalar(0.0), btScalar(0.0), btScalar(0.0)),
	m_lastTotalTorque.setValue(btScalar(0.0), btScalar(0.0), btScalar(0.0));
	setDamping(constructionInfo.m_linearDamping, constructionInfo.m_angularDamping);

	m_addedMass = 0.0f;

	m_linearSleepingThreshold = constructionInfo.m_linearSleepingThreshold;
	m_angularSleepingThreshold = constructionInfo.m_angularSleepingThreshold;
	m_optionalMotionState = constructionInfo.m_motionState;
	m_contactSolverType = 0;
	m_frictionSolverType = 0;
	m_additionalDamping = constructionInfo.m_additionalDamping;
	m_additionalDampingFactor = constructionInfo.m_additionalDampingFactor;
	m_additionalLinearDampingThresholdSqr = constructionInfo.m_additionalLinearDampingThresholdSqr;
	m_additionalAngularDampingThresholdSqr = constructionInfo.m_additionalAngularDampingThresholdSqr;
	m_additionalAngularDampingFactor = constructionInfo.m_additionalAngularDampingFactor;

	if (m_optionalMotionState)
	{
		m_optionalMotionState->getWorldTransform(m_worldTransform);
	}
	else
	{
		m_worldTransform = constructionInfo.m_startWorldTransform;
	}

	m_previousWorldTransform = m_worldTransform;
	m_interpolationWorldTransform = m_worldTransform;
	m_interpolationLinearVelocity.setValue(0, 0, 0);
	m_interpolationAngularVelocity.setValue(0, 0, 0);

	//moved to btCollisionObject
	m_friction = constructionInfo.m_friction;
	m_rollingFriction = constructionInfo.m_rollingFriction;
	m_spinningFriction = constructionInfo.m_spinningFriction;

	m_restitution = constructionInfo.m_restitution;

	setCollisionShape(constructionInfo.m_collisionShape);
	m_debugBodyId = uniqueId++;

	setMassProps(constructionInfo.m_mass, constructionInfo.m_localInertia);
	updateInertiaTensor();

	m_rigidbodyFlags = BT_ENABLE_GYROSCOPIC_FORCE_IMPLICIT_BODY;

	m_deltaLinearVelocity.setZero();
	m_deltaAngularVelocity.setZero();
	m_invMass = m_inverseMass * m_linearFactor;
	m_pushVelocity.setZero();
	m_turnVelocity.setZero();

	m_Children = {};
	m_cableCollision = nullptr;
	m_redirectionTarget = nullptr;
	m_localTransform = btTransform::getIdentity();

	// Init massAtImpact
	backupMassProps();
	setupMassAtImpact(m_massImpactData.MassBackup, m_massImpactData.MassBackup, 0.0, 1.0);
}

void btRigidBody::predictIntegratedTransform(btScalar timeStep, btTransform& predictedTransform)
{
	clampVelocity();
	btTransformUtil::integrateTransform(m_worldTransform, m_linearVelocity, m_angularVelocity, timeStep, predictedTransform);
}

void btRigidBody::updateBulletChildren(btScalar timeStep, unsigned int currentFrame)
{
	// Ignore static or kinematic objects as they will be updated by their parent
	if (this->isStaticOrKinematicObject())
	{
		return;
	}

	// 1. Traverse up to find the top-most dynamic parent
	btRigidBody* root = this;
	while (root->m_parent != nullptr && !root->m_parent->isStaticOrKinematicObject())
	{
		root = root->m_parent;
	}

	// 2. Check if the root has already been updated this frame.
	// If the root is updated, it means the whole hierarchy below it is also done.
	if (root->m_lastUpdateFrame == currentFrame)
	{
		return;
	}

	// 3. Start the recursive update
	root->updateBulletChildrenRecursive(timeStep, currentFrame);
}

void btRigidBody::updateBulletChildrenRecursive(btScalar timeStep, unsigned int currentFrame)
{
	// Mark this body as updated for this frame
	m_lastUpdateFrame = currentFrame;
	
	for (auto kinematic : m_Children)
	{
		if (!kinematic->isStaticOrKinematicObject()) continue;

		// World transform kinematic = WorldTransform Parent * LocalTransform Kinematic	
		// Note: 'getWorldTransform()' here refers to 'this' (the parent), which is already updated.
		btTransform res = getWorldTransform() * kinematic->m_localTransform;
		
		kinematic->setInterpolationWorldTransform(kinematic->getWorldTransform());
		kinematic->setWorldTransform(res);
		kinematic->getMotionState()->setWorldTransform(res);

		// Recurse down the tree using the internal helper
		kinematic->updateBulletChildrenRecursive(timeStep, currentFrame);

		// Need velocity update later
		kinematic->m_hasMovedWithChildrenUpdate = true;
	}
}

void btRigidBody::updateBulletChildrenInterpolated(btScalar timeStep, unsigned int currentFrame)
{
	// Ignore static or kinematic objects as they will be updated by their parent
	if (this->isStaticOrKinematicObject())
	{
		return;
	}

	// 1. Traverse up to find the top-most dynamic parent
	btRigidBody* root = this;
	while (root->m_parent != nullptr && !root->m_parent->isStaticOrKinematicObject())
	{
		root = root->m_parent;
	}
	
	// 2. Check if the root has already been updated this frame.
	// If the root is updated, it means the whole hierarchy below it is also done.
	if (root->m_lastInterpolatedUpdateFrame == currentFrame)
	{
		return;
	}

	// 3. Start the recursive update from the root
	root->updateBulletChildrenInterpolatedRecursive(timeStep, currentFrame);
}

void btRigidBody::updateBulletChildrenInterpolatedRecursive(btScalar timeStep, unsigned int currentFrame)
{
	// Mark this body as updated for this frame
	m_lastInterpolatedUpdateFrame = currentFrame;
	
	for (auto kinematic : m_Children)
	{
		if (!kinematic->isStaticOrKinematicObject()) continue;

		// World transform kinematic = WorldTransform Parent * LocalTransform Kinematic	
		// Note: 'getWorldTransform()' here refers to 'this' (the parent), which is already updated.
		btTransform res = getWorldTransform() * kinematic->m_localTransform;
		
		kinematic->setInterpolationWorldTransform(kinematic->getWorldTransform());
		kinematic->setWorldTransform(res);
		kinematic->getMotionState()->setWorldTransform(res);

		// Recurse down the tree using the internal helper
		kinematic->updateBulletChildrenInterpolatedRecursive(timeStep, currentFrame);

		// Need velocity update later
		kinematic->m_hasMovedWithChildrenUpdate = true;
	}
}

void btRigidBody::addBulletChild(btRigidBody* child)
{
	// Ensure we don't add null
	if (!child) return;

	m_Children.push_back(child);
    
	// Link the child to the parent
	child->m_parent = this;
}

void btRigidBody::removeBulletChild(btRigidBody* child)
{
	for (int index = 0; index < m_Children.size(); index++)
	{
		if (m_Children[index] == child) 
		{
			// Unlink the child from the parent
			child->m_parent = nullptr;

			m_Children.erase(m_Children.begin() + index);
			return;
		}
	}
}

void btRigidBody::updateCableCollision(btScalar timeStep)
{
	if (m_cableCollision != NULL)
	{
		// World transform kinematic = WordlTransform Parent * LocalTransform Kinematic
		btTransform res = getWorldTransform() * m_cableCollision->m_localTransform;
		m_cableCollision->setInterpolationWorldTransform(m_cableCollision->getWorldTransform());
		m_cableCollision->setWorldTransform(res);
		m_cableCollision->getMotionState()->setWorldTransform(res);

		m_cableCollision->setLinearVelocity(m_linearVelocity);
		m_cableCollision->setAngularVelocity(m_angularVelocity);
	}
}


void btRigidBody::saveKinematicVelocity(btScalar timeStep)
{
	bool needUpdate;
	if (m_parent == nullptr)
	{
		// If this body has no parent, we need to recalculate velocity because it may have moved
		needUpdate = true;
	}
	else
	{
		// If this body is considered has child of dynamic body and moved by his parent
		needUpdate = m_hasMovedWithChildrenUpdate;
	}

	//todo: clamp to some (user definable) safe minimum timestep, to limit maximum angular/linear velocities
	if (timeStep != btScalar(0.) && needUpdate)
	{
		btVector3 linVel, angVel;
		btTransformUtil::calculateVelocity(m_interpolationWorldTransform, m_worldTransform, timeStep, linVel, angVel);
		m_linearVelocity = linVel;
		m_angularVelocity = angVel;

		m_hasMovedWithChildrenUpdate = false;
	}
}

void btRigidBody::saveKinematicState(btScalar timeStep)
{
	// //todo: clamp to some (user definable) safe minimum timestep, to limit maximum angular/linear velocities
	// if (timeStep != btScalar(0.))
	// {
	//		//if we use motionstate to synchronize world transforms, get the new kinematic/animated world transform
	//		if (getMotionState())
	//			getMotionState()->getWorldTransform(m_worldTransform);
	//		btVector3 linVel, angVel;
	//		
	//		btTransformUtil::calculateVelocity(m_interpolationWorldTransform, m_worldTransform, timeStep, m_linearVelocity, m_angularVelocity);
	//		m_interpolationLinearVelocity = m_linearVelocity;
	//		m_interpolationAngularVelocity = m_angularVelocity;
	//		m_interpolationWorldTransform = m_worldTransform;
	//		//printf("angular = %f %f %f\n",m_angularVelocity.getX(),m_angularVelocity.getY(),m_angularVelocity.getZ());
	// }

	// In Unity, we update the render of the rigibody, so we need to sync the motion state
	if (getMotionState()) getMotionState()->getWorldTransform(m_worldTransform);
}

void btRigidBody::syncKinematicState()
{
	m_previousWorldTransform = m_worldTransform;
	m_interpolationWorldTransform = m_previousWorldTransform;
	m_interpolationLinearVelocity = m_linearVelocity;
	m_interpolationAngularVelocity = m_angularVelocity;
}

void btRigidBody::getAabb(btVector3& aabbMin, btVector3& aabbMax) const
{
	getCollisionShape()->getAabb(m_worldTransform, aabbMin, aabbMax);
}

void btRigidBody::setGravity(const btVector3& acceleration)
{
	if (m_inverseMass != btScalar(0.0))
	{
		m_gravity = acceleration * (btScalar(1.0) / m_inverseMass);
	}
	m_gravity_acceleration = acceleration;
}

void btRigidBody::setDamping(btScalar lin_damping, btVector3 ang_damping)
{
#ifdef BT_USE_OLD_DAMPING_METHOD
	m_linearDamping = btMax(lin_damping, btScalar(0.0));
	m_angularDamping = btMax(ang_damping, btScalar(0.0));
#else
	m_linearDamping = btClamped(lin_damping, btScalar(0.0), btScalar(1.0));
	m_angularDamping.setX(btClamped(ang_damping.getX(), btScalar(0.0), btScalar(1.0)));
	m_angularDamping.setY(btClamped(ang_damping.getY(), btScalar(0.0), btScalar(1.0)));
	m_angularDamping.setZ(btClamped(ang_damping.getZ(), btScalar(0.0), btScalar(1.0)));
#endif
}

void btRigidBody::setDamping(btScalar lin_damping, btScalar ang_damping)
{
#ifdef BT_USE_OLD_DAMPING_METHOD
	m_linearDamping = btMax(lin_damping, btScalar(0.0));
	m_angularDamping = btMax(ang_damping, btScalar(0.0));
#else
	m_linearDamping = btClamped(lin_damping, btScalar(0.0), btScalar(1.0));
	m_angularDamping.setX(btClamped(ang_damping, btScalar(0.0), btScalar(1.0)));
	m_angularDamping.setY(btClamped(ang_damping, btScalar(0.0), btScalar(1.0)));
	m_angularDamping.setZ(btClamped(ang_damping, btScalar(0.0), btScalar(1.0)));


#endif
}

void btRigidBody::setAddedMass(btScalar addedMass)
{
	m_addedMass = addedMass;
}

///applyDamping damps the velocity, using the given m_linearDamping and m_angularDamping
void btRigidBody::applyDamping(btScalar timeStep)
{
	//On new damping: see discussion/issue report here: http://code.google.com/p/bullet/issues/detail?id=74
	//todo: do some performance comparisons (but other parts of the engine are probably bottleneck anyway

#ifdef BT_USE_OLD_DAMPING_METHOD
	m_linearVelocity *= btMax((btScalar(1.0) - timeStep * m_linearDamping), btScalar(0.0));
	m_angularVelocity *= btMax((btScalar(1.0) - timeStep * m_angularDamping), btScalar(0.0));
#else
	m_linearVelocity *= btPow(btScalar(1) - m_linearDamping, timeStep); 

	// m_angularDamping is LCS
	// m_angularVelocity is ECEF
	// Convert angular velocity to LCS, apply damping, convert it back to ECEF
	btTransform tr = getWorldTransform();
	btDefaultMotionState* mo = (btDefaultMotionState*)m_optionalMotionState;
	if (mo)
	{
		tr = mo->getGraphicsWorldTransform();
	}
	btVector3 angularVelocityLCS = tr.getBasis().inverse() * m_angularVelocity;
	angularVelocityLCS.setX(angularVelocityLCS.getX() * btPow(btScalar(1) - m_angularDamping.getX(), timeStep));
	angularVelocityLCS.setY(angularVelocityLCS.getY() * btPow(btScalar(1) - m_angularDamping.getY(), timeStep));
	angularVelocityLCS.setZ(angularVelocityLCS.getZ() * btPow(btScalar(1) - m_angularDamping.getZ(), timeStep));

	m_angularVelocity = tr.getBasis() * angularVelocityLCS;

#endif

	if (m_additionalDamping)
	{
		//Additional damping can help avoiding lowpass jitter motion, help stability for ragdolls etc.
		//Such damping is undesirable, so once the overall simulation quality of the rigid body dynamics system has improved, this should become obsolete
		if ((m_angularVelocity.length2() < m_additionalAngularDampingThresholdSqr) &&
			(m_linearVelocity.length2() < m_additionalLinearDampingThresholdSqr))
		{
			m_angularVelocity *= m_additionalDampingFactor;
			m_linearVelocity *= m_additionalDampingFactor;
		}

		btScalar speed = m_linearVelocity.length();
		if (speed < m_linearDamping)
		{
			btScalar dampVel = btScalar(0.005);
			if (speed > dampVel)
			{
				btVector3 dir = m_linearVelocity.normalized();
				m_linearVelocity -= dir * dampVel;
			}
			else
			{
				m_linearVelocity.setValue(btScalar(0.), btScalar(0.), btScalar(0.));
			}
		}

		btScalar angDampVel = btScalar(0.005);
		btVector3 dir = m_angularVelocity.normalized();

		btScalar angSpeedX = m_angularVelocity.getX();
		if (angSpeedX < m_angularDamping.getX())
		{
			if (angSpeedX > angDampVel)
			{
				m_angularVelocity.setX(angSpeedX - dir.getX() * angDampVel);
			}
			else
			{
				m_angularVelocity.setX(btScalar(0.));
			}
		}

		btScalar angSpeedY = m_angularVelocity.getY();
		if (angSpeedY < m_angularDamping.getY())
		{
			if (angSpeedY > angDampVel)
			{
				m_angularVelocity.setY(angSpeedY - dir.getY() * angDampVel);
			}
			else
			{
				m_angularVelocity.setY(btScalar(0.));
			}
		}

		btScalar angSpeedZ = m_angularVelocity.getZ();
		if (angSpeedZ < m_angularDamping.getZ())
		{
			if (angSpeedZ > angDampVel)
			{
				m_angularVelocity.setZ(angSpeedZ - dir.getZ() * angDampVel);
			}
			else
			{
				m_angularVelocity.setZ(btScalar(0.));
			}
		}
	}
}

void btRigidBody::applyGravity()
{
	if (isStaticOrKinematicObject())
		return;

	applyCentralForce(m_gravity);
}

void btRigidBody::clearGravity()
{
    if (isStaticOrKinematicObject())
        return;
    
    applyCentralForce(-m_gravity);
}

void btRigidBody::proceedToTransform(const btTransform& newTrans)
{
	setCenterOfMassTransform(newTrans);
}

void btRigidBody::setMassProps(btScalar mass, const btVector3& inertia)
{
	setMassPropsNoGravity(mass, inertia);
	
	//Fg = m * a
	m_gravity = mass * m_gravity_acceleration;

	if (m_massImpactData.IsActive)
	{
		backupMassProps();
		syncMassAtImpact();
	}
}

void btRigidBody::setMassPropsNoGravity(btScalar mass, const btVector3& inertia)
{
	if (mass == btScalar(0.))
	{
		m_collisionFlags |= btCollisionObject::CF_STATIC_OBJECT;
		m_inverseMass = btScalar(0.);
	}
	else
	{
		m_collisionFlags &= (~btCollisionObject::CF_STATIC_OBJECT);
		m_inverseMass = btScalar(1.0) / mass;
	}

	m_invInertiaLocal.setValue(inertia.x() != btScalar(0.0) ? btScalar(1.0) / inertia.x() : btScalar(0.0),
							   inertia.y() != btScalar(0.0) ? btScalar(1.0) / inertia.y() : btScalar(0.0),
							   inertia.z() != btScalar(0.0) ? btScalar(1.0) / inertia.z() : btScalar(0.0));

	m_invMass = m_linearFactor * m_inverseMass;
}

void btRigidBody::setupMassAtImpact(btScalar lowerMass, btScalar upperMass, btScalar lowerLimitDistance, btScalar upperLimitDistance)
{
	m_massImpactData.LowerUserMass = lowerMass;
	m_massImpactData.UpperUserMass = upperMass;
	m_massImpactData.LowerLimitDistance = btMax(lowerLimitDistance, 0.0);
	m_massImpactData.UpperLimitDistance = btMax(upperLimitDistance, m_massImpactData.LowerLimitDistance);
}

void btRigidBody::syncMassAtImpact()
{
	m_massImpactData.LowerMass = btMax(m_massImpactData.MassBackup, m_massImpactData.LowerUserMass);
	m_massImpactData.UpperMass = btMax(m_massImpactData.UpperUserMass, m_massImpactData.LowerMass);

	// Force reset to avoid mixing different setups
	m_massImpactData.lastNodeAnchorDistance = 0.0;
}

void btRigidBody::activeMassAtImpact(bool isActive)
{
	// Update mass properties on state change only
	if (m_massImpactData.IsActive == isActive)
	{
		return;
	}

	m_massImpactData.IsActive = isActive;

	if (m_massImpactData.IsActive)
	{
		// On activation make sure to sync true mass
		backupMassProps();
		syncMassAtImpact();
	}
	else
	{
		// On deactivation make sure to restore mass
		restoreMassProps();
	}
}

void btRigidBody::backupMassProps()
{
	m_massImpactData.MassBackup = getMass();
	m_massImpactData.InertiaLocalBackup = getLocalInertia();
}

void btRigidBody::storeAnchorLastState(btScalar anchorNodeDistance)
{
	// Keep track of the last anchor node distance
	// In case of multiple anchor on the same body we assume the worst case (max required)
	m_massImpactData.lastNodeAnchorDistance = btMax(anchorNodeDistance, m_massImpactData.lastNodeAnchorDistance);
}

void btRigidBody::applyMassAtImpact()
{
	// Safety net
	if (m_anchorsCount < 1.0)
	{
		return;
	}

	// @TODO(BenH) add new bool to enable/disable this mechanic (cable tendu ou non)

	// Match massRatio application
	btScalar ratio = 0.0;
	if (m_massImpactData.lastNodeAnchorDistance > m_massImpactData.LowerLimitDistance)
	{
		// Compute mass change based on the distance between anchor and closest node
		btScalar anchorNodeDistanceRange = m_massImpactData.UpperLimitDistance - m_massImpactData.LowerLimitDistance;
		btScalar rawRatio = btClamped((m_massImpactData.lastNodeAnchorDistance - m_massImpactData.LowerLimitDistance) / anchorNodeDistanceRange, 0.0, 1.0);
		
		// Select transition dynamic
		ratio = rawRatio;
		// ratio = 1.0 - pow(max(0.0, abs(rawRatio - 1.0) * 1.1 - 0.1), 3);
		// ratio = 1.0 - pow(1.0 - rawRatio, 3);								// easeOutCubic
		// ratio = sqrt(1.0 - pow(rawRatio - 1.0, 2.0));						// easeOutCirc
		// ratio = rawRatio * rawRatio;										    // easeInQuad
		// ratio = rawRatio * rawRatio * rawRatio * rawRatio;					// easeInQuart
	}

	// Change only the body mass and inertia without altering the resulting gravity force
	btScalar newMass = btLerp(m_massImpactData.LowerMass, m_massImpactData.UpperMass, ratio);
	if (btFabs(newMass - getMass()) > DBL_EPSILON)
	{
		setMassPropsNoGravity(newMass, newMass * getLocalInertia() * m_inverseMass);
		updateInertiaTensor();
	}

	// Reset to make sure data is not persistent
	m_massImpactData.lastNodeAnchorDistance = 0.0;
}

void btRigidBody::restoreMassProps()
{
	setMassProps(m_massImpactData.MassBackup, m_massImpactData.InertiaLocalBackup);
	updateInertiaTensor();
}

void btRigidBody::updateInertiaTensor()
{
	m_invInertiaTensorWorld = m_worldTransform.getBasis().scaled(m_invInertiaLocal) * m_worldTransform.getBasis().transpose();
}

btVector3 btRigidBody::getLocalInertia() const
{
	btVector3 inertiaLocal;
	const btVector3 inertia = m_invInertiaLocal;
	inertiaLocal.setValue(inertia.x() != btScalar(0.0) ? btScalar(1.0) / inertia.x() : btScalar(0.0),
						  inertia.y() != btScalar(0.0) ? btScalar(1.0) / inertia.y() : btScalar(0.0),
						  inertia.z() != btScalar(0.0) ? btScalar(1.0) / inertia.z() : btScalar(0.0));
	return inertiaLocal;
}

inline btVector3 evalEulerEqn(const btVector3& w1, const btVector3& w0, const btVector3& T, const btScalar dt,
							  const btMatrix3x3& I)
{
	const btVector3 w2 = I * w1 + w1.cross(I * w1) * dt - (T * dt + I * w0);
	return w2;
}

inline btMatrix3x3 evalEulerEqnDeriv(const btVector3& w1, const btVector3& w0, const btScalar dt,
									 const btMatrix3x3& I)
{
	btMatrix3x3 w1x, Iw1x;
	const btVector3 Iwi = (I * w1);
	w1.getSkewSymmetricMatrix(&w1x[0], &w1x[1], &w1x[2]);
	Iwi.getSkewSymmetricMatrix(&Iw1x[0], &Iw1x[1], &Iw1x[2]);

	const btMatrix3x3 dfw1 = I + (w1x * I - Iw1x) * dt;
	return dfw1;
}

btVector3 btRigidBody::computeGyroscopicForceExplicit(btScalar maxGyroscopicForce) const
{
	btVector3 inertiaLocal = getLocalInertia();
	btMatrix3x3 inertiaTensorWorld = getWorldTransform().getBasis().scaled(inertiaLocal) * getWorldTransform().getBasis().transpose();
	btVector3 tmp = inertiaTensorWorld * getAngularVelocity();
	btVector3 gf = getAngularVelocity().cross(tmp);
	btScalar l2 = gf.length2();
	if (l2 > maxGyroscopicForce * maxGyroscopicForce)
	{
		gf *= btScalar(1.) / btSqrt(l2) * maxGyroscopicForce;
	}
	return gf;
}

btVector3 btRigidBody::computeGyroscopicImpulseImplicit_Body(btScalar step) const
{
	btVector3 idl = getLocalInertia();
	btVector3 omega1 = getAngularVelocity();
	btQuaternion q = getWorldTransform().getRotation();

	// Convert to body coordinates
	btVector3 omegab = quatRotate(q.inverse(), omega1);
	btMatrix3x3 Ib;
	Ib.setValue(idl.x(), 0, 0,
				0, idl.y(), 0,
				0, 0, idl.z());

	btVector3 ibo = Ib * omegab;

	// Residual vector
	btVector3 f = step * omegab.cross(ibo);

	btMatrix3x3 skew0;
	omegab.getSkewSymmetricMatrix(&skew0[0], &skew0[1], &skew0[2]);
	btVector3 om = Ib * omegab;
	btMatrix3x3 skew1;
	om.getSkewSymmetricMatrix(&skew1[0], &skew1[1], &skew1[2]);

	// Jacobian
	btMatrix3x3 J = Ib + (skew0 * Ib - skew1) * step;

	//	btMatrix3x3 Jinv = J.inverse();
	//	btVector3 omega_div = Jinv*f;
	btVector3 omega_div = J.solve33(f);

	// Single Newton-Raphson update
	omegab = omegab - omega_div;  //Solve33(J, f);
	// Back to world coordinates
	btVector3 omega2 = quatRotate(q, omegab);
	btVector3 gf = omega2 - omega1;
	return gf;
}

btVector3 btRigidBody::computeGyroscopicImpulseImplicit_World(btScalar step) const
{
	// use full newton-euler equations.  common practice to drop the wxIw term. want it for better tumbling behavior.
	// calculate using implicit euler step so it's stable.

	const btVector3 inertiaLocal = getLocalInertia();
	const btVector3 w0 = getAngularVelocity();

	btMatrix3x3 I;

	I = m_worldTransform.getBasis().scaled(inertiaLocal) *
		m_worldTransform.getBasis().transpose();

	// use newtons method to find implicit solution for new angular velocity (w')
	// f(w') = -(T*step + Iw) + Iw' + w' + w'xIw'*step = 0
	// df/dw' = I + 1xIw'*step + w'xI*step

	btVector3 w1 = w0;

	// one step of newton's method
	{
		const btVector3 fw = evalEulerEqn(w1, w0, btVector3(0, 0, 0), step, I);
		const btMatrix3x3 dfw = evalEulerEqnDeriv(w1, w0, step, I);

		btVector3 dw;
		dw = dfw.solve33(fw);
		//const btMatrix3x3 dfw_inv = dfw.inverse();
		//dw = dfw_inv*fw;

		w1 -= dw;
	}

	btVector3 gf = (w1 - w0);
	return gf;
}

void btRigidBody::integrateVelocities(btScalar step)
{
	if (isStaticOrKinematicObject())
		return;

	m_linearVelocity += updateAcceleration() * step;
	m_angularVelocity += updateAngularAcceleration() * step;


	/*#define MAX_ANGVEL SIMD_HALF_PI
		/// clamp angular velocity. collision calculations will fail on higher angular velocities
		btScalar angvel = m_angularVelocity.length();
		if (angvel * step > MAX_ANGVEL)
		{
			m_angularVelocity *= (MAX_ANGVEL / step) / angvel;
		}
		#if defined(BT_CLAMP_VELOCITY_TO) && BT_CLAMP_VELOCITY_TO > 0
		clampVelocity(m_angularVelocity);
		#endif*/
}

void btRigidBody::clampVelocity()
{
	if (m_maxLinearVelocity > 0 && m_linearVelocity.length() > m_maxLinearVelocity)
		m_linearVelocity = m_linearVelocity.normalized() * m_maxLinearVelocity;

	if (m_maxAngularVelocity > 0)
	{
		btScalar x = m_angularVelocity.getX();
		btClamp(x, -m_maxAngularVelocity, m_maxAngularVelocity);
		m_angularVelocity.setX(x);

		btScalar y = m_angularVelocity.getY();
		btClamp(y, -m_maxAngularVelocity, m_maxAngularVelocity);
		m_angularVelocity.setY(y);

		btScalar z = m_angularVelocity.getZ();
		btClamp(z, -m_maxAngularVelocity, m_maxAngularVelocity);
		m_angularVelocity.setZ(z);
	}
}

btQuaternion btRigidBody::getOrientation() const
{
	btQuaternion orn;
	m_worldTransform.getBasis().getRotation(orn);
	return orn;
}

void btRigidBody::setCenterOfMassTransform(const btTransform& xform)
{
	if (isKinematicObject())
	{
		m_interpolationWorldTransform = m_worldTransform;
	}
	else
	{
		m_interpolationWorldTransform = xform;
	}
	m_interpolationLinearVelocity = getLinearVelocity();
	m_interpolationAngularVelocity = getAngularVelocity();
	m_previousWorldTransform = m_worldTransform;
	m_worldTransform = xform;
	updateInertiaTensor();
}

void btRigidBody::addConstraintRef(btTypedConstraint* c)
{
	///disable collision with the 'other' body

	int index = m_constraintRefs.findLinearSearch(c);
	//don't add constraints that are already referenced
	//btAssert(index == m_constraintRefs.size());
	if (index == m_constraintRefs.size())
	{
		m_constraintRefs.push_back(c);
		btCollisionObject* colObjA = &c->getRigidBodyA();
		btCollisionObject* colObjB = &c->getRigidBodyB();
		if (colObjA == this)
		{
			colObjA->setIgnoreCollisionCheck(colObjB, true);
		}
		else
		{
			colObjB->setIgnoreCollisionCheck(colObjA, true);
		}
	}
}

void btRigidBody::removeConstraintRef(btTypedConstraint* c)
{
	int index = m_constraintRefs.findLinearSearch(c);
	//don't remove constraints that are not referenced
	if (index < m_constraintRefs.size())
	{
		m_constraintRefs.remove(c);
		btCollisionObject* colObjA = &c->getRigidBodyA();
		btCollisionObject* colObjB = &c->getRigidBodyB();
		if (colObjA == this)
		{
			colObjA->setIgnoreCollisionCheck(colObjB, false);
		}
		else
		{
			colObjB->setIgnoreCollisionCheck(colObjA, false);
		}
	}
}

int btRigidBody::calculateSerializeBufferSize() const
{
	int sz = sizeof(btRigidBodyData);
	return sz;
}

///fills the dataBuffer and returns the struct name (and 0 on failure)
const char* btRigidBody::serialize(void* dataBuffer, class btSerializer* serializer) const
{
	btRigidBodyData* rbd = (btRigidBodyData*)dataBuffer;

	btCollisionObject::serialize(&rbd->m_collisionObjectData, serializer);

	m_invInertiaTensorWorld.serialize(rbd->m_invInertiaTensorWorld);
	m_linearVelocity.serialize(rbd->m_linearVelocity);
	m_angularVelocity.serialize(rbd->m_angularVelocity);
	rbd->m_inverseMass = m_inverseMass;
	m_angularFactor.serialize(rbd->m_angularFactor);
	m_linearFactor.serialize(rbd->m_linearFactor);
	m_gravity.serialize(rbd->m_gravity);
	m_gravity_acceleration.serialize(rbd->m_gravity_acceleration);
	m_invInertiaLocal.serialize(rbd->m_invInertiaLocal);
	m_totalForce.serialize(rbd->m_totalForce);
	m_totalTorque.serialize(rbd->m_totalTorque);
	rbd->m_linearDamping = m_linearDamping;
	m_angularDamping.serialize(rbd->m_angularDamping);
	rbd->m_additionalDamping = m_additionalDamping;
	rbd->m_additionalDampingFactor = m_additionalDampingFactor;
	rbd->m_additionalLinearDampingThresholdSqr = m_additionalLinearDampingThresholdSqr;
	rbd->m_additionalAngularDampingThresholdSqr = m_additionalAngularDampingThresholdSqr;
	rbd->m_additionalAngularDampingFactor = m_additionalAngularDampingFactor;
	rbd->m_linearSleepingThreshold = m_linearSleepingThreshold;
	rbd->m_angularSleepingThreshold = m_angularSleepingThreshold;

	// Fill padding with zeros to appease msan.
#ifdef BT_USE_DOUBLE_PRECISION
	memset(rbd->m_padding, 0, sizeof(rbd->m_padding));
#endif

	return btRigidBodyDataName;
}

void btRigidBody::serializeSingleObject(class btSerializer* serializer) const
{
	btChunk* chunk = serializer->allocate(calculateSerializeBufferSize(), 1);
	const char* structType = serialize(chunk->m_oldPtr, serializer);
	serializer->finalizeChunk(chunk, structType, BT_RIGIDBODY_CODE, (void*)this);
}
