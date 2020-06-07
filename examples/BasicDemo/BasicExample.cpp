/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2015 Google Inc. http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include "BasicExample.h"

#include "btBulletDynamicsCommon.h"
#define ARRAY_SIZE_Y 5
#define ARRAY_SIZE_X 5
#define ARRAY_SIZE_Z 5

#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h"

#include "../CommonInterfaces/CommonRigidBodyBase.h"
#include "BulletDynamics/Character/btKinematicCharacterController.h"
#include "BulletCollision\CollisionDispatch\btGhostObject.h"

struct BasicExample : public CommonRigidBodyBase
{
	BasicExample(struct GUIHelperInterface* helper)
		: CommonRigidBodyBase(helper)
	{
	}
	virtual ~BasicExample() {}
	virtual void initPhysics();
	virtual void renderScene();
	void resetCamera()
	{
		float dist = 4;
		float pitch = -35;
		float yaw = 52;
		float targetPos[3] = {0, 0, 0};
		m_guiHelper->resetCamera(dist, yaw, pitch, targetPos[0], targetPos[1], targetPos[2]);
	}
};

void bullet3_eventFunCall(void* m_param, btKinematicCharacterController::EVENT_CONTROL ec, btVector3 start, btVector3 end, btScalar angle) {

	int a = 0;
}

void BasicExample::initPhysics()
{
	m_guiHelper->setUpAxis(1);

	createEmptyDynamicsWorld();
	//m_dynamicsWorld->setGravity(btVector3(0,0,0));
	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);

	if (m_dynamicsWorld->getDebugDrawer())
		m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawWireframe + btIDebugDraw::DBG_DrawContactPoints);

	///create a few basic rigid bodies
	btBoxShape* groundShape = createBoxShape(btVector3(btScalar(150.), btScalar(50.), btScalar(150.)));

	//groundShape->initializePolyhedralFeatures();
	//btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0,1,0),50);

	m_collisionShapes.push_back(groundShape);

	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0, -50, 0));

	{

		btScalar mass(0.);
		btRigidBody* brb = createRigidBody(mass, groundTransform, groundShape, btVector4(0, 0, 1, 1));
	}
	/*
	{
		//create a few dynamic rigidbodies
		// Re-using the same collision is better for memory usage and performance

		btBoxShape* colShape = createBoxShape(btVector3(.1, .1, .1));

		//btCollisionShape* colShape = new btSphereShape(btScalar(1.));
		m_collisionShapes.push_back(colShape);

		/// Create Dynamic Objects
		btTransform startTransform;
		startTransform.setIdentity();

		btScalar mass(1.f);

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0, 0, 0);
		if (isDynamic)
			colShape->calculateLocalInertia(mass, localInertia);

		for (int k = 0; k < ARRAY_SIZE_Y; k++)
		{
			for (int i = 0; i < ARRAY_SIZE_X; i++)
			{
				for (int j = 0; j < ARRAY_SIZE_Z; j++)
				{
					startTransform.setOrigin(btVector3(
						btScalar(0.2 * i),
						btScalar(2 + .2 * k),
						btScalar(0.2 * j)));

					createRigidBody(mass, startTransform, colShape);
				}
			}
		}
	}*/

		{
			btKinematicCharacterController* m_character;
			btPairCachingGhostObject* m_ghostObject;

			btTransform m_trans;
			m_trans.setIdentity();
			m_trans.setOrigin(btVector3(5.0, 2.0, 5.0));

			m_ghostObject = new btPairCachingGhostObject();
			m_ghostObject->setWorldTransform(m_trans);
			((btAxisSweep3*)m_dynamicsWorld->getBroadphase())->getOverlappingPairCache()->setInternalGhostPairCallback(new btGhostPairCallback());
			//this->m_overlappingPairCache->getOverlappingPairCache()->setInternalGhostPairCallback(new btGhostPairCallback());
			btScalar characterHeight = 1.0f;
			btScalar characterWidth = 1.0f;
			btConvexShape* capsule = new btCapsuleShape(characterWidth, characterHeight);
			m_ghostObject->setCollisionShape(capsule);
			m_ghostObject->setCollisionFlags(btCollisionObject::CF_CHARACTER_OBJECT | btCollisionObject::CF_KINEMATIC_OBJECT);

			btScalar stepHeight = btScalar(0.0);
			m_character = new btKinematicCharacterController(m_ghostObject, capsule, stepHeight);
			m_character->setWalkDirection(btVector3(0.0, 0.0, 0.0));
			m_character->m_eventFunCall = bullet3_eventFunCall;
			//向世界中添加碰撞对象   
			m_dynamicsWorld->addCollisionObject(
				m_ghostObject,
				btBroadphaseProxy::CharacterFilter,
				btBroadphaseProxy::StaticFilter | btBroadphaseProxy::DefaultFilter);
			m_dynamicsWorld->addAction(m_character);
			
			btBoxShape* colShape = createBoxShape(btVector3(1, 1, 1));
			//btCollisionShape* colShape = new btSphereShape(btScalar(1.));
			m_collisionShapes.push_back(colShape);

			btTransform startTransform;
			startTransform.setIdentity();
			startTransform.setOrigin(btVector3(
				btScalar(10),
				btScalar(3),
				btScalar(0)));
			btScalar mass(1.0f);
			//createRigidBody(mass, startTransform, colShape);

			m_character->moveDirection(btVector3(
				btScalar(10),
				btScalar(0),
				btScalar(0)));
		}
		/*
	{
		btKinematicCharacterController* m_character;
		btPairCachingGhostObject* m_ghostObject;

		btTransform m_trans;
		m_trans.setIdentity();
		m_trans.setOrigin(btVector3(
			btScalar(10),
			btScalar(2.0),
			btScalar(0)));

		m_ghostObject = new btPairCachingGhostObject();
		m_ghostObject->setWorldTransform(m_trans);
		//this->m_overlappingPairCache->getOverlappingPairCache()->setInternalGhostPairCallback(new btGhostPairCallback());
		btScalar characterHeight = 1.0f;
		btScalar characterWidth = 1.0f;
		btConvexShape* capsule = new btCapsuleShape(characterWidth, characterHeight);
		m_ghostObject->setCollisionShape(capsule);
		//m_ghostObject->setCollisionFlags(btCollisionObject::CF_CHARACTER_OBJECT | btCollisionObject::CF_KINEMATIC_OBJECT);
		btScalar stepHeight = btScalar(0.0);
		m_character = new btKinematicCharacterController(m_ghostObject, capsule, stepHeight);
		m_character->setWalkDirection(btVector3(0.0, 0.0, 0.0));

		//向世界中添加碰撞对象   
		m_dynamicsWorld->addCollisionObject(
			m_ghostObject,
			btBroadphaseProxy::CharacterFilter,
			btBroadphaseProxy::StaticFilter | btBroadphaseProxy::DefaultFilter);
		m_dynamicsWorld->addAction(m_character);
		}*/
	
	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}

void BasicExample::renderScene()
{
	CommonRigidBodyBase::renderScene();
}

CommonExampleInterface* BasicExampleCreateFunc(CommonExampleOptions& options)
{
	return new BasicExample(options.m_guiHelper);
}

B3_STANDALONE_EXAMPLE(BasicExampleCreateFunc)
