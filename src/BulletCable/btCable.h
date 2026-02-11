#ifndef _BT_CABLE_H
#define _BT_CABLE_H

#include "cubic_spline.hpp"
#include "LinearMath/btVector3.h"
#include "LinearMath/btTransform.h"
#include "BulletSoftBody/btSoftBody.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "BulletCollision/CollisionShapes/btTriangleShape.h"
#include "BulletCollision/CollisionDispatch/btCollisionWorld.h"
#include "BulletCollision/CollisionDispatch/btCollisionDispatcherMt.h"
#include "BulletCollision/CollisionShapes/btSphereShape.h"

#include <vector>
#include <BulletCollision/CollisionShapes/btCompoundShape.h>

using namespace std;

class CableManifolds
{
public:
	btPersistentManifold* manifold;
	int lifeTime;
	CableManifolds(btPersistentManifold* mani, int time)
	{
		manifold = mani;
		lifeTime = time;
	}
};

///The btCable is a class that inherits from btSoftBody.
///Its purpose is to be able to create a cable/rope with our own method parameters that Bullet does not implement.
class btCable : public btSoftBody
{
	typedef void (btCable::* DistanceFunction)();

	enum class CollisionMode
	{
		Base = 0, // No additional coefficient applied
		Linear,	// Linear coefficient depending on penetration
		Curve // depending on user given curve
	};

	enum class DistanceMode
	{
		PBD = 0, // Original distance constraint (Original)
		XPBD // Modified distance constraint
	};
	
	enum class StretchRatioMode
	{
		None = 0, // Assume mass ratio is always needed at max
	    Cable,    // Tension ratio is computed based on the whole cable length (same accross anchors)
		Link,     // Tension ration is computed based on the more stretched link (same accross anchors)
		Anchor    // Tension ration is computed only at the anchor level (unique to each anchor)
	};

	enum class StretchRatioCurve
	{
		Linear = 0,     
		Quadratic,      
		QuadraticInverse,
		Quartic,
		QuarticInverse
	};

	//
	~btCable()
	{
		int size = manifolds.size();
		for (int i = 0; i < size; i++)
		{
			m_world->getDispatcher()->releaseManifold(manifolds.at(i).manifold);
		}
		manifolds.clear();

		// Release allocated data
		if (m_nodePos != nullptr)
		{
			delete[] m_nodePos;
		}

		if (m_nodeData != nullptr)
		{
			delete[] m_nodeData;
		}

		if (m_section != nullptr)
		{
			delete[] m_section;
		}

		delete m_cableData;
		delete spline;
	}

	btAlignedObjectArray<CableManifolds> manifolds;

public:
	struct CableData
	{
		float radius;
		float tangentDragCoefficient;
		float normalDragCoefficient;
		int startIndex;
		int endIndex;
	};
	static const std::size_t CableDataSize = sizeof(CableData);

	struct NodePos
	{
		double x;
		double y;
		double z;
	};
	static const std::size_t NodePosSize = sizeof(NodePos);

	struct NodeData
	{
		float velocity_x;
		float velocity_y;
		float velocity_z;
		float volume;
	};
	static const std::size_t NodeDataSize = sizeof(NodeData);

	struct BroadPhasePair
	{
		Node* node;
		btCollisionObject* body;
		int bodyType;
	};

	struct NodePairNarrowPhase
	{
		Node* node;
		btVector3 hitPoint;
		btVector3 normal;
		btScalar timeOfImpact;
		btScalar margin;
		btScalar distance;
		BroadPhasePair* pair;
		btTransform worldTransform;

		btPersistentManifold* manifold = nullptr;
		bool haveManifoldsRegister = false;
	};

	struct RayJob
	{
		int nodeIdx;
		NodePairNarrowPhase* pair;
		int rayIdx;
		btVector3 from;
		btVector3 to;
		btScalar margin;
	};

	struct ObjData
	{
		btCollisionObject* obj;
		btVector3 minAabb, maxAabb;
		btVector3 objVelocity;
	};

	struct ContactInfo
	{
		btVector3 point;
		btVector3 normal;
		btScalar distance;
	};

	struct MyContactResultCallback : public btCollisionWorld::ContactResultCallback
	{
		bool m_connected;
		btScalar minDist = FLT_MAX;
		btScalar m_margin;
		btVector3 contactPoint;
		btVector3 contactNorm;

		btCollisionObject* A;
		btCollisionObject* B;

		MyContactResultCallback(btScalar pMargin, btCollisionObject* pA, btCollisionObject* pB) : m_connected(false), m_margin(pMargin), A(pA), B(pB) {}

		virtual btScalar addSingleResult(btManifoldPoint& cp, const btCollisionObjectWrapper* colliderA, int partId0, int index0, const btCollisionObjectWrapper* colliderB, int partId1, int index1)
		{
			btScalar dist = cp.getDistance();
			// Filter back face depending on collider order
			bool orderIsAB = false;
			if (colliderA->m_collisionObject == A && colliderB->m_collisionObject == B)
			{
				orderIsAB = true;
			}
			else  // A and B are inverted
			{
				orderIsAB = false;
			}

			// Triangle Shape ?
			bool isTriangleShape = colliderB->m_shape->getShapeType() == TRIANGLE_SHAPE_PROXYTYPE;
			// Back face ?
			if (isTriangleShape)
			{
				const btTriangleShape* triangleShape = (const btTriangleShape*)(colliderB->getCollisionShape());
				btVector3 triangleNormal;
				triangleShape->calcNormal(triangleNormal);
				if (triangleNormal.dot(cp.m_normalWorldOnB) < 0)
				{
					// Back face is touched, filter it out or handle accordingly
					return 1.0;
				}
			}

			if (dist <= btScalar(0) || (dist > btScalar(0) && dist <= m_margin))
			{
				// Compute an always-negative "effectiveDist" to store in minDist
				// - If penetrating, use actual dist (negative or zero).
				// - If just near-contact (positive dist), convert to a small negative proxy.
				btScalar effectiveDist = dist <= btScalar(0) ? dist : -dist;
				
				//cout << "dist: " << dist << endl;
				m_connected = true;
				if (effectiveDist < minDist)
				{
					// A=A, B=B
					// A is the node, B is the target collider
					if (orderIsAB)
					{
						contactPoint = cp.getPositionWorldOnB();
						contactNorm = cp.m_normalWorldOnB;
					}
					// A and B are inverted
					else
					{
						contactPoint = cp.getPositionWorldOnA();
						contactNorm = -cp.m_normalWorldOnB;
					}
					minDist = effectiveDist;
				}
			}
			return 1.0;
		}
	};

private:
	// Growing state for Unity control
	// 0: cable length isn't changing
	// 1: cable currently grows
	// 2: cable shinks at the minimal value
	// 3: cable shrinks on an anchor
	// 4: cable grow but cant add node

	int m_growingState = 0;
	// Number of solverIteration for 1 deltaTime passed
	int m_solverSubStep = 1;
	// Actual iteration
	int m_cpt;
	int m_sectionCount = 0;
	int m_sectionCurrent = 0;
	btScalar m_minLength;
	bool useLRA = true;
	bool invertLRA = false;
	bool useBending = true;
	bool useGravity = true;
	bool useCollision = true;
	btScalar m_linearMass = 1.0;
	btScalar m_maxTension = -1.0;

	vector<btScalar> collisionFonctionPointX;
	vector<btScalar> collisionFonctionPointY;

	//
	// Mass balance control
	//

	// How mass balance is influenced by cable stretch
	struct StretchRatioBehavior
	{
		StretchRatioMode mode{StretchRatioMode::None};
		StretchRatioCurve curveLowSpeed{StretchRatioCurve::Linear};
		StretchRatioCurve curveHighSpeed{StretchRatioCurve::Linear};
		btScalar speedThreshold{4.0};
		btScalar min{0.0};
		btScalar max{1.0};
	};
	StretchRatioBehavior m_stretchBehavior{};
	btScalar m_lengthAccumulator = 0.0;
	btScalar m_restLengthAccumulator = 0.0;
	btScalar m_stretchRatio = 0.0;
	btScalar m_stretchRatioDamped = 0.0;
	btScalar m_cableStretchRatio = 0.0;
	btScalar m_linkStretchRatio = 0.0;

	// Schmitt hysteresis to avoid on/off activation
	struct StretchRatioHysteresis
	{
		// Window used to avoid on/off effect arount the mass balance activation
		btScalar amount{0.0};

		// Precomputer bounds based on amount and mass balance activation minimal value
		btScalar lowerBound{0.0};
		btScalar upperBound{0.0};

		// Percentage of iterations at which the state can no longer change
		btScalar threshold{1.0};
		bool enabled{0.0};
		bool frozen{0.0};
	};
	StretchRatioHysteresis m_stretchHysteresis{};

	// EMA filtering to filter out noise
	// https://en.wikipedia.org/wiki/Exponential_smoothing
	struct StretchRatioDamping
	{
		// Damping stored as it counterpart (1-damping) for efficiency
		btScalar amount{1.0};  // The bigger the more damping [0, 1]

		// Percentage of iterations at which the damping attenuation is applied
		btScalar threshold{0.0};  // The larger the later [0, 1]

		// Used to scale damping after reaching threshold
		btScalar attenuation{1.0};  // The smaller the more attenuation [0, 1]
	};
	StretchRatioDamping m_stretchDamping{};
	btScalar m_cableStretchRatioDamped = 0.0;
	btScalar m_linkStretchRatioDamped = 0.0;
	btScalar m_massBalanceRatio = 0.0;



	MonotonicSpline1D* spline;

	// Node forces members
	bool impulseCompute = true;

	btScalar penetrationMin = 0;
	btScalar penetrationMax = 0.1;
	btScalar collisionStiffnessMin = 100;
	btScalar collisionStiffnessMax = 10000;

	btScalar collisionViscosity = 10;

	btScalar m_defaultRestLength;

	btScalar maxAngle = 0;
	btScalar bendingStiffness = 0;

	// disabled collision detection if this movement
	btScalar m_collisionSleepingThreshold = 0.0;

	// number of iteration step between each iteration of the collision constraint
	int m_substepDelayCollisionSolver = 1;

	// number of iteration of the resolution on multi-collision node
	int m_substepDelayCollisionNarrow = 1;

	// Node forces members
	bool useHydroAero = true;

	// XPDB
	btScalar cableStiffness = 500 * 1e9;
	btScalar dampingStiffness = 0;

	float m_collisionMargin = 0;

	btCable::CableData* m_cableData;
	btCable::NodeData* m_nodeData;
	btCable::NodePos* m_nodePos;

	void distanceConstraint(int currentIter);
	void distanceConstraintPBD();
	void distanceConstraintXPBD();

	btScalar computeMassBalanceRatio(Anchor& anchor);

	void distanceConstraintLock(int limMin, int limMax);
	void LRAConstraint();
	void LRAHierachique();
	void distanceHierachy(int indexStart, int indexEnd);

	void predictMotion(btScalar dt) override;
	void solveConstraints() override;
	static void setNodeBoundingBox(btVector3 mx, btVector3 mq, btScalar margin, btVector3* minLink, btVector3* maxLink);
	void anchorConstraint();
	void anchorConstraintPlacement();

	void updateMassRatioDamping(int currentIter);

	void contactConstraint();
	btVector3 calculateBodyImpulse(btRigidBody* obj, Node* n, btVector3 normal, btVector3 hitPosition);
	btScalar computeCollisionMargin(const btCollisionShape* shape) const;
	void resetManifoldLifeTime();

	void updateNodeDeltaPos(int iteration);

	btScalar getLinkRestLength(int index);

	void collectPotentials(btCollisionObjectArray& collisionObjectArray, std::vector<btCollisionObject*>& out) const;
	void buildObjData(const std::vector<btCollisionObject*>& pots,
					  std::vector<ObjData>& out) const;
	void runBroadPhase();
	void runNarrowPhase();
	bool aabbTestMargin(btVector3 nodeVel, btVector3 objVel, btVector3 nodeMinAabb, btVector3 nodeMaxAabb, btVector3 minAabb, btVector3 maxAabb);

	// Internal: run one 'constraint/projection' iteration of your existing cable solver
	void solveSingleCableIteration(int currentIter);

	void EndConstraintsSolve();

	struct IterativeSolveState
	{
		bool active = false;
		int total = 0;
		int current = 0;

		// Add caches you may need across iterations here (e.g., predicted positions)
		// Example:
		// btAlignedObjectArray<btVector3> m_predictedPos;
	};

	IterativeSolveState m_iter;

	btAlignedObjectArray<BroadPhasePair> _candidates;
	btAlignedObjectArray<NodePairNarrowPhase> _nodePairContact;
	bool _impacted = false;

	// Cached object used to resolve contacts
	btSphereShape _nodeContactSphere;
	btCollisionObject _nodeContactObject;
	btTransform _nodeContactTransform;

public:
	btCable(btSoftBodyWorldInfo* worldInfo, btCollisionWorld* world, int node_count, int section_count, const btVector3* x, const btScalar* m);

	CollisionMode m_collisionMode;
	DistanceMode m_distanceMode;
	DistanceFunction m_distanceFunction;  // avoid a lot of "if/switch" statements
	bool m_useAnchorConstraintPlacement;

	btScalar WantedDistance = 0;
	btScalar WantedSpeed = 0;
	btScalar forceResponseCoef;
	btVector3 m_gravity;

	void updateLength(btScalar dt);

	void Grows(float dt);

	void Shrinks(float dt);

	void updateNodeData();

	void ResetForceAndVelocity();

	void ResetNodePosition(const int nodeIndex, const btVector3 position);

	void PrepareSolver();

	// Begin an iterative solve session for visual stepping
	void beginIterativeSolve();

	// Perform exactly one solver iteration; returns true if more remain
	bool stepOneIteration();

	// Finalize and clean up after the iterative solve session
	void endIterativeSolve();

	// Query helpers
	bool isIterativeSolveActive() const { return m_iter.active; }
	int currentIteration() const { return m_iter.current; }
	int totalIterations() const { return m_iter.total; }

	enum CableState
	{
		Valid = 0,
		InternalForcesError = 1,
		ExternalForcesError = 2
	};

	btCable::CableState cableState = Valid;

	struct SectionInfo
	{
		double RestLength;
		int StartNodeIndex;
		int EndNodeIndex;
		int NumberOfNodes;
	};

	btCable::SectionInfo* m_section;

#pragma region Use methods
public:
	btScalar getLength();
	btScalar getRestLength();

	btVector3 getTensionAt(int index);
	btVector3 getLocalAnchorWithNode(int index);

	void bendingConstraint();

	void setUseBending(bool active);
	bool getUseBending();

	void setBendingMaxAngle(btScalar angle);
	btScalar getBendingMaxAngle();

	void setBendingStiffness(btScalar stiffness);
	btScalar getBendingStiffness();

	void setUseLRA(bool active);
	bool getUseLRA();
	void setInvertLRA(bool invert);

	void setUseGravity(bool active);
	bool getUseGravity();

	void setUseCollision(bool active);
	bool getUseCollision();

	void setCollisionParameters(int substepSolverCollisionDelay, int substepNarrowCollisionDelay);

	bool getUseHydroAero();
	void setUseHydroAero(bool active);

	btCable::CableData* getCableData()
	{
		return m_cableData;
	}

	void setStartIndex(int start)
	{
		m_cableData->startIndex = start;
	}

	void setEndIndex(int end)
	{
		m_cableData->endIndex = end;
	}

	void setCableRadius(float radius)
	{
		m_cableData->radius = radius;
	}

	void setCableNormalDragCoefficient(float normalDragCoefficient)
	{
		m_cableData->normalDragCoefficient = normalDragCoefficient;
	}

	void setCableTangentDragCoefficient(float tangentDragCoefficient)
	{
		m_cableData->tangentDragCoefficient = tangentDragCoefficient;
	}

	btCable::NodeData* getNodeData()
	{
		return m_nodeData;
	}

	btCable::NodePos* getNodePos()
	{
		return m_nodePos;
	}

	void* getCableNodesPos();

	int getCableState();

	int getCollisionMode();

	void appendNode(const btVector3& x, btScalar m) override;

	void removeNodeAt(const int index);

	void setTotalMass(btScalar mass, bool fromfaces = false) override;

	void setCollisionMargin(float colMargin);

	float getCollisionMargin();

	void addSection(btScalar rl, int start, int end, int nbNodes);

	void setDefaultRestLength(btScalar rl);
	void setMinLength(btScalar value);

	int getGrowingState();

	void setWantedGrowSpeedAndDistance(btScalar speed, btScalar distance);
	void setLinearMass(btScalar mass);
	void updateNodesMass();

	void setCollisionStiffness(btScalar stiffnessMin, btScalar stiffnessMax, btScalar distMin, btScalar distMax);
	void setCollisionResponseActive(bool active);
	void setCollisionMode(int mode);
	void setControlPoint(vector<btScalar> dataX, vector<btScalar> dataY);
	void updateCurveResponse(btScalar* dataX, btScalar* dataY, int size);

	void synchNodesInfos();
	void setMaxTension(btScalar maxTension);

	bool shouldTestObject(btCollisionObject* colObj) const;

	void setDistanceMode(int mode);
	int getDistanceMode();

	void setUseAnchorConstraintPlacement(bool status);

	btScalar getStretchRatio() { return m_stretchRatio; }
	btScalar getStretchRatioDamped() { return m_stretchRatioDamped; }
	btScalar getMassBalanceRatio() { return m_massBalanceRatio; }
	bool getIsMassBalanceEnabled() { return m_stretchHysteresis.enabled; }

	void setStretchRatioMinThreshold(btScalar value);
	btScalar getStretchRatioMaxThreshold() { return m_stretchBehavior.max; }

	void setStretchRatioMaxThreshold(btScalar value);
	btScalar getStretchRatioMinThreshold() { return m_stretchBehavior.min; }

	void setStretchRatioHysteresis(btScalar value);
	btScalar getStretchRatioHysteresis() { return m_stretchHysteresis.amount; }

	void setStretchRatioDamping(btScalar value);
	btScalar getStretchRatioDamping() { return m_stretchDamping.amount; }

	void setStretchRatioMode(int modeId) { m_stretchBehavior.mode = static_cast<StretchRatioMode>(modeId); }
	int getStretchRatioMode() { return static_cast<int>(m_stretchBehavior.mode); }

	void setStretchRatioLowSpeedCurve(int curveId) { m_stretchBehavior.curveLowSpeed = static_cast<StretchRatioCurve>(curveId); }
	int getStretchRatioLowSpeedCurve() { return static_cast<int>(m_stretchBehavior.curveLowSpeed); }

	void setStretchRatioHighSpeedCurve(int curveId) { m_stretchBehavior.curveHighSpeed = static_cast<StretchRatioCurve>(curveId); }
	int getStretchRatioHighSpeedCurve() { return static_cast<int>(m_stretchBehavior.curveHighSpeed); }

	void setStretchRatioSpeedThreshold(btScalar threshold) { m_stretchBehavior.speedThreshold = threshold; }
	btScalar getStretchRatioSpeedThreshold() { return m_stretchBehavior.speedThreshold; }

	void setStretchRatioHysteresisThreshold(btScalar threshold) { m_stretchHysteresis.threshold = threshold; }
	btScalar getStretchRatioHysteresisThreshold() { return m_stretchHysteresis.threshold; }

	void setStretchRatioDampingThreshold(btScalar threshold) { m_stretchDamping.threshold = threshold; }
	btScalar getStretchRatioDampingThreshold() { return m_stretchDamping.threshold; }

#pragma endregion
};
#endif  //_BT_CABLE_H
