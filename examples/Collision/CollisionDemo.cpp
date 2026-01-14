#include "CollisionDemo.h"

#include "../CommonInterfaces/CommonGraphicsAppInterface.h"
#include "../CommonInterfaces/CommonRenderInterface.h"
#include "../CommonInterfaces/CommonGUIHelperInterface.h"
#include "../CommonInterfaces/CommonExampleInterface.h"
#include "../CommonInterfaces/CommonRigidBodyBase.h"

#include "LinearMath/btTransform.h"

#include <BulletSoftBody/btSoftBody.h>
#include <BulletDynamics/Dynamics/btDiscreteDynamicsWorldMt.h>
#include <BulletSoftBody/btSoftRigidDynamicsWorld.h>
#include "BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h"
#include <BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolverMt.h>
#include <BulletCollision/NarrowPhaseCollision/CustomManifold.h>

class CollisionDemo : public CommonRigidBodyBase
{
public:
	CollisionDemo(GUIHelperInterface* guiHelper) : CommonRigidBodyBase(guiHelper)
	{}
private:

	class btTaskSchedulerManager
	{
		btAlignedObjectArray<btITaskScheduler*> m_taskSchedulers;
		btAlignedObjectArray<btITaskScheduler*> m_allocatedTaskSchedulers;

	public:
		btTaskSchedulerManager() {}
		void init()
		{
			addTaskScheduler(btGetSequentialTaskScheduler());
#if BT_THREADSAFE
			if (btITaskScheduler* ts = btCreateDefaultTaskScheduler())
			{
				m_allocatedTaskSchedulers.push_back(ts);
				addTaskScheduler(ts);
			}
			addTaskScheduler(btGetOpenMPTaskScheduler());
			addTaskScheduler(btGetTBBTaskScheduler());
			addTaskScheduler(btGetPPLTaskScheduler());
			if (getNumTaskSchedulers() > 1)
			{
				// prefer a non-sequential scheduler if available
				btSetTaskScheduler(m_taskSchedulers[1]);
			}
			else
			{
				btSetTaskScheduler(m_taskSchedulers[0]);
			}
#endif  // #if BT_THREADSAFE
		}
		void shutdown()
		{
			for (int i = 0; i < m_allocatedTaskSchedulers.size(); ++i)
			{
				delete m_allocatedTaskSchedulers[i];
			}
			m_allocatedTaskSchedulers.clear();
		}

		void addTaskScheduler(btITaskScheduler* ts)
		{
			if (ts)
			{
#if BT_THREADSAFE
				// if initial number of threads is 0 or 1,
				if (ts->getNumThreads() <= 1)
				{
					// for OpenMP, TBB, PPL set num threads to number of logical cores
					ts->setNumThreads(ts->getMaxNumThreads());
				}
#endif  // #if BT_THREADSAFE
				m_taskSchedulers.push_back(ts);
			}
		}
		int getNumTaskSchedulers() const { return m_taskSchedulers.size(); }
		btITaskScheduler* getTaskScheduler(int i) { return m_taskSchedulers[i]; }
	};

	btTaskSchedulerManager gTaskSchedulerMgr;

	CommonGraphicsApp* m_app;

	// World
	btSoftBodyWorldInfo m_softBodyWorldInfo;
	
	btCollisionAlgorithmCreateFunc* m_boxBoxCF;

	// Inherited via CommonExampleInterface
	void initPhysics() override
	{
		///create concave ground mesh
		m_guiHelper->setUpAxis(1);

		m_dispatcher = 0;

		if (gTaskSchedulerMgr.getNumTaskSchedulers() == 0)
		{
			gTaskSchedulerMgr.init();
		}

		///register some softbody collision algorithms on top of the default btDefaultCollisionConfiguration
		m_collisionConfiguration = new btSoftBodyRigidBodyCollisionConfiguration();

		//m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
		m_dispatcher = new btCollisionDispatcherMt(m_collisionConfiguration);
		m_softBodyWorldInfo.m_dispatcher = m_dispatcher;

		////////////////////////////
		///Register softbody versus softbody collision algorithm

		///Register softbody versus rigidbody collision algorithm
		////////////////////////////

		btVector3 worldAabbMin(-1000, -1000, -1000);
		btVector3 worldAabbMax(1000, 1000, 1000);

		//m_broadphase = new btAxisSweep3(worldAabbMin, worldAabbMax, maxProxies);

		m_broadphase = new btDbvtBroadphase();

		m_softBodyWorldInfo.m_broadphase = m_broadphase;

		m_softBodyWorldInfo.numThread = 4;

		//btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver();
		btSequentialImpulseConstraintSolverMt* solver = new btSequentialImpulseConstraintSolverMt();

		//btDantzigSolver* mlcp = new btDantzigSolver();
		//btSolveProjectedGaussSeidel* mlcp = new btSolveProjectedGaussSeidel;
		//btMLCPSolver* solver = new btMLCPSolver(mlcp);

		m_solver = solver;

		btSoftBodySolver* softBodySolver = 0;
#ifdef USE_AMD_OPENCL

		static bool once = true;
		if (once)
		{
			once = false;
			initCL(0, 0);
		}

		if (g_openCLSIMDSolver)
			delete g_openCLSIMDSolver;
		if (g_softBodyOutput)
			delete g_softBodyOutput;

		if (1)
		{
			g_openCLSIMDSolver = new btOpenCLSoftBodySolverSIMDAware(g_cqCommandQue, g_cxMainContext);
			//	g_openCLSIMDSolver = new btOpenCLSoftBodySolver( g_cqCommandQue, g_cxMainContext);
			g_openCLSIMDSolver->setCLFunctions(new CachingCLFunctions(g_cqCommandQue, g_cxMainContext));
		}

		softBodySolver = g_openCLSIMDSolver;
		g_softBodyOutput = new btSoftBodySolverOutputCLtoCPU;
#endif  //USE_AMD_OPENCL

		//btDiscreteDynamicsWorld* world = new btSoftRigidDynamicsWorld(m_dispatcher, m_broadphase, m_solver, m_collisionConfiguration);

		btSoftRigidDynamicsWorld* world = new btSoftRigidDynamicsWorld(&m_softBodyWorldInfo, m_solver, m_collisionConfiguration, softBodySolver);
		m_dynamicsWorld = world;
		//m_dynamicsWorld->setInternalTickCallback(pickingPreTickCallbackCable, this, true);

		m_dynamicsWorld->getSolverInfo().m_numIterations = 256;

		m_dynamicsWorld->getDispatchInfo().m_enableSPU = true;
		m_dynamicsWorld->setGravity(btVector3(0, -9.81, 0));
		m_softBodyWorldInfo.m_gravity.setValue(0, -9.81, 0);
		m_guiHelper->createPhysicsDebugDrawer(world);
		//	clientResetScene();

		m_softBodyWorldInfo.m_sparsesdf.Initialize();
		//	clientResetScene();


		m_softBodyWorldInfo.m_sparsesdf.Reset();

		m_softBodyWorldInfo.air_density = (btScalar)0;
		m_softBodyWorldInfo.water_density = 0;
		m_softBodyWorldInfo.water_offset = 0;
		m_softBodyWorldInfo.water_normal = btVector3(0, 0, 0);
		m_softBodyWorldInfo.m_gravity.setValue(0, -9.81, 0);
		//m_softBodyWorldInfo.numIteration = substepSolver;

		//Init_PhysicFallDemo();
		Init_KinematicFallDemo();

		m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
	}

	void exitPhysics() override
	{
		//cleanup in the reverse order of creation/initialization

	//remove the rigidbodies from the dynamics world and delete them
		int i;
		for (i = m_dynamicsWorld->getNumConstraints() - 1; i >= 0; i--)
		{
			btTypedConstraint* constraint = m_dynamicsWorld->getConstraint(i);
			m_dynamicsWorld->removeConstraint(constraint);
			delete constraint;
		}
		for (i = m_dynamicsWorld->getNumCollisionObjects() - 1; i >= 0; i--)
		{
			btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
			btRigidBody* body = btRigidBody::upcast(obj);
			if (body && body->getMotionState())
			{
				delete body->getMotionState();
			}
			m_dynamicsWorld->removeCollisionObject(obj);
			delete obj;
		}

		//delete collision shapes
		for (int j = 0; j < m_collisionShapes.size(); j++)
		{
			btCollisionShape* shape = m_collisionShapes[j];
			m_collisionShapes[j] = 0;
			delete shape;
		}

		//delete dynamics world
		delete m_dynamicsWorld;
		m_dynamicsWorld = 0;

		//delete solver
		delete m_solver;

		//delete broadphase
		delete m_broadphase;

		//delete dispatcher
		delete m_dispatcher;

		delete m_collisionConfiguration;
	}


	std::string GetPrintState(int colFlag, int colType, int activationState)
	{
		std::string s = "";
		s += "CollisionFlag: ";
		switch (colFlag)
		{
			case 0: s += "CF_DYNAMIC_OBJECT"; break;
			case 1: s += "CF_STATIC_OBJECT"; break;
			case 2: s += "CF_KINEMATIC_OBJECT"; break;
			case 4: s += "CF_NO_CONTACT_RESPONSE"; break;
			case 8: s += "CF_CUSTOM_MATERIAL_CALLBACK"; break;
			case 16: s += "CF_CHARACTER_OBJECT"; break;
			case 32: s += "CF_DISABLE_VISUALIZE_OBJECT"; break;
			default: s += "UNKNOW";	break;
		}
		
		s += " | CollisionObjectTypes: ";
		switch (colType)
		{
			case 1: s += "CO_COLLISION_OBJECT"; break;
			case 2: s += "CO_RIGID_BODY"; break;
			case 4: s += "CO_GHOST_OBJECT"; break;
			case 8: s += "CO_SOFT_BODY"; break;
			default: s += "UNKNOW";	break;
		}

		s += " | ActivationState: ";
		switch (activationState)
		{
		case 1: s += "ACTIVE_TAG"; break;
		case 2: s += "ISLAND_SLEEPING"; break;
		case 3: s += "WANTS_DEACTIVATION"; break;
		case 4: s += "DISABLE_DEACTIVATION"; break;
		case 5: s += "DISABLE_SIMULATION"; break;
		case 6: s += "FIXED_BASE_MULTI_BODY"; break;
		default: s += "UNKNOW";	break;
		}

		return s;
	}

	void MoveKinematic(int index, btVector3 vel, float dt)
	{
		btCollisionObjectArray collisionArray = m_dynamicsWorld->getCollisionObjectArray();
		btRigidBody* rbKinematic = (btRigidBody*)collisionArray[index];

		btTransform mat = rbKinematic->getWorldTransform();
		btVector3 pos = mat.getOrigin();
		pos += vel* dt;
		mat.setOrigin(pos);
		rbKinematic->setWorldTransform(mat);
		rbKinematic->getMotionState()->setWorldTransform(mat);
	}

	void stepSimulation(float deltaTime) override
	{
		btCollisionObjectArray collisionArray = m_dynamicsWorld->getCollisionObjectArray();
		int count = m_dynamicsWorld->getNumCollisionObjects();

		// Move kinematic
		MoveKinematic(1, btVector3(0, -1, 0), deltaTime);
		
		for (int i = 0; i < count; i++)
		{
			btRigidBody* rb = (btRigidBody*)collisionArray[i];
			int colfFlag = rb->getCollisionFlags();
			int colfType = rb->getInternalType();
			int activationState = rb->getActivationState();

			rb->activate(true);

			b3Printf("Object %i - %s", i, GetPrintState(colfFlag, colfType, activationState).c_str());
		}

		//PrintVelocities("BEFORE");
		if (m_dynamicsWorld)
		{
			m_dynamicsWorld->stepSimulation(deltaTime);
		}
		PrintManifoldInformation();
		b3Printf("Cache");
		PrintManifoldCacheInformation();
		//PrintVelocities("AFTER");

		//b3Printf("Object count: %i", count);
		//b3Printf("Manifold number: %i",m_dispatcher->getNumManifolds());
		//b3Printf("Manifold cache number: %i", m_dispatcher->getNumManifoldsCache());
	}

	void PrintVelocities(string frameMoment)
	{
		btCollisionObjectArray collisionArray = m_dynamicsWorld->getCollisionObjectArray();
		int count = m_dynamicsWorld->getNumCollisionObjects();
		for (int i = 0; i < count; i++)
		{
			btRigidBody* rb = (btRigidBody*)collisionArray[i];
			b3Printf(" %s : - Object %i - vel.y %f - pos %f", frameMoment.c_str(), i, rb->getLinearVelocity().getY(), rb->getWorldTransform().getOrigin().getY());
		}
	}

	void PrintManifoldInformation()
	{
		int manifoldCount = m_dispatcher->getNumManifolds();
		for (int i = 0; i < manifoldCount; i++)
		{
			btPersistentManifold* manifold = m_dispatcher->getManifoldByIndexInternal(i);
			int numContact = manifold->getNumContacts();
			btRigidBody* rb0 = (btRigidBody*)manifold->getBody0();
			btRigidBody* rb1 = (btRigidBody*)manifold->getBody1();

			b3Printf("Manifold %i - body0 : mass %f - vel %f - pos %f ", i, rb0->getMass(), rb0->getLinearVelocity().getY(), rb0->getWorldTransform().getOrigin().getY() );
			b3Printf("Manifold %i - body1 : mass %f - vel %f - pos %f ", i, rb1->getMass(), rb1->getLinearVelocity().getY(), rb1->getWorldTransform().getOrigin().getY() );

			for (int j = 0; j < numContact; j++)
			{
				btManifoldPoint point = manifold->getContactPoint(j);
				btScalar impulse = point.getAppliedImpulse();
				b3Printf("Manifold %i - point %i - impulse %f", i, j, impulse);
			}
		}
	}

	void PrintManifoldCacheInformation()
	{
		int manifoldCount = m_dispatcher->getNumManifoldsCache();
		for (int i = 0; i < manifoldCount; i++)
		{
			CustomManifold* manifold = m_dispatcher->getManifoldsCacheByIndexInternal(i);
			int numContact = manifold->getCount();
			btRigidBody* rb0 = (btRigidBody*)manifold->getBody0();
			btRigidBody* rb1 = (btRigidBody*)manifold->getBody1();

			b3Printf("Manifold %i - body0 : mass %f - vel %f - pos %f ", i, rb0->getMass(), rb0->getLinearVelocity().getY(), rb0->getWorldTransform().getOrigin().getY());
			b3Printf("Manifold %i - body1 : mass %f - vel %f - pos %f ", i, rb1->getMass(), rb1->getLinearVelocity().getY(), rb1->getWorldTransform().getOrigin().getY());

			for (int j = 0; j < numContact; j++)
			{
				CustomManifoldPoint* point = manifold->getManifoldPoint(j);
				btScalar impulse = point->GetImpulse();
				b3Printf("Manifold %i - point %i - impulse %f", i, j, impulse);
			}
		}
	}


	// Physics cube fall on kinematic one
	void Init_PhysicFallDemo()
	{
		// Shape
		btCollisionShape* boxShape = new btBoxShape(btVector3(0.25, 0.25, 0.25));

		// Masses
		btScalar massKinematic(0);
		btScalar massPhysic(10);

		// Rotation
		btQuaternion rotation(0, 0, 0, 1);

		// Transform
		btTransform transformKinematic;
		transformKinematic.setIdentity();
		transformKinematic.setRotation(rotation);

		btTransform transformPhysic;
		transformPhysic.setIdentity();
		transformPhysic.setRotation(rotation);

		// Positions
		btVector3 positionKinematic(0, 0, 0);
		btVector3 positionPhysic(0, 3, 0);
		transformKinematic.setOrigin(positionKinematic);
		transformPhysic.setOrigin(positionPhysic);

		// Create the rigidbodys
		btRigidBody* kinematic = createRigidBody(massKinematic, transformKinematic, boxShape);
		btRigidBody* physic = createRigidBody(massPhysic, transformPhysic, boxShape);

		physic->setGravity(btVector3(0, -9.81, 0));
	}

	// Kinematic cube fall on physic one
	void Init_KinematicFallDemo()
	{
		// Shape
		btCollisionShape* boxShape = new btBoxShape(btVector3(0.5, 0.5, 0.5));

		// Masses
		btScalar massKinematic(0);
		btScalar massPhysic(15400);

		// Rotation
		btQuaternion rotation(0, 0, 0, 1);

		// Transform
		btTransform transformKinematic;
		transformKinematic.setIdentity();
		transformKinematic.setRotation(rotation);

		btTransform transformPhysic;
		transformPhysic.setIdentity();
		transformPhysic.setRotation(rotation);

		// Positions
		//btVector3 positionKinematic(0, 6378137 + 3, 0);
		//btVector3 positionPhysic(0, 6378137, 0);
		btVector3 positionKinematic(0,  3, 0);
		btVector3 positionPhysic(0, 0, 0);
		transformKinematic.setOrigin(positionKinematic);
		transformPhysic.setOrigin(positionPhysic);

		// Create the rigidbodys
		btRigidBody* physic = createRigidBody(massPhysic, transformPhysic, boxShape, btVector4(0,0,1,1));
		btRigidBody* kinematic = createRigidBody(massKinematic, transformKinematic, boxShape, btVector4(1, 0, 0, 1));
		
		physic->setGravity(btVector3(0, 0, 0));

		kinematic->setCollisionFlags(btCollisionObject::CF_KINEMATIC_OBJECT);
	}
};

class CommonExampleInterface* CubeCollisionCreateFunc(struct CommonExampleOptions& options)
{
	return new CollisionDemo(options.m_guiHelper);
}