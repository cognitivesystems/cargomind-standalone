/*
Bullet Continuous Collision Detection and Physics Library
*/

#include "bullet/PhysicsBullet.h"
#include "BulletCollision/NarrowPhaseCollision/btRaycastCallback.h"
#include "LinearMath/btHashMap.h"

#include <stdio.h>
#include <vector>
#include <iostream>

// unit is meter in bullet
PhysicsBullet::PhysicsBullet()
{
    m_defaultContactProcessingThreshold = BT_LARGE_FLOAT;
    initPhysics();
}

Eigen::Vector3d PhysicsBullet::castRays(Eigen::Vector3d &point, Eigen::Vector3d &direction)
{
    Eigen::Vector3d projection;
    projection = point;

    if (m_dynamicsWorld)
	{
        // first hit
        btVector3 from = btVector3( (float)(point(0)), (float)(point(1)), (float)(point(2)) );
        btVector3 to   = btVector3( (float)(point(0) + 5 * direction(0)), (float)(point(1) + 5 * direction(1)), (float)(point(2) + 5 * direction(2)) );       

        ////////////////////////////////////////////////////////////////////////////////
        /// \brief Problem:  If this point already contacts with a box, then the rayTest will get next point instead of this contact point.
        /// \brief Solution: check if this direction is contacted. If it is contacted, don't need projection!
        bool needProj = true;
        std::shared_ptr<btRigidBody> pointBall (localCreatePointSphere(from));
        ContactResultCallback resultCallback;
        m_dynamicsWorld->contactTest(pointBall.get(), resultCallback);
        size_t oldNum = resultCallback.collisionVec.size();

        std::shared_ptr<btRigidBody> pointTemp (localCreatePointSphere(btVector3( (float)(point(0)-0.01*direction(0)), (float)(point(1)-0.01*direction(1)), (float)(point(2)-0.01*direction(2)))));
        resultCallback.clearDatas();
        m_dynamicsWorld->contactTest(pointTemp.get(), resultCallback);

        if(resultCallback.collisionVec.size() < oldNum)
        {
            needProj = false;
        }
        /////////////////////////////////////////////////////////////////////////

        btCollisionWorld::ClosestRayResultCallback	closestResults(from,to);
        m_dynamicsWorld->rayTest(from,to,closestResults);

        if (closestResults.hasHit() && needProj)
        {
            btVector3 p = closestResults.m_hitPointWorld;
            projection << p.getX(), p.getY(), p.getZ();

            //check for 0.0
            for(size_t i = 0; i<3; ++i)
            {
                if(projection(i) < 1.0e-04){ projection(i) = 0.0;}
            }

            if( floatEqual(point(0), projection(0)) && floatEqual(point(1), projection(1)) && floatEqual(point(2), projection(2)))
            {
                projection = point;
            }
            // cut for 0.00
            for(size_t i = 0; i<3; ++i)
            {
                projection(i) = ((int)( floor((projection(i)+1e-3) * 100)))/ 100.0;
            }
        }
	}
    return projection;
}

void PhysicsBullet::addNewBoxToPhysics(bpa::Box &new_box)
{
    btScalar mass(0.0);
    btVector3 size;
    btVector3 origin;
    if(new_box.is_rotated)
    {
        size = btVector3(new_box.m_width, new_box.m_length, new_box.m_height);
        // COM is the original one, depend on 0 rotation
        origin = btVector3(new_box.position.position(0) + new_box.center_of_mass.position(1),
                           new_box.position.position(1) + new_box.center_of_mass.position(0),
                           new_box.position.position(2) + new_box.center_of_mass.position(2));
    }
    else
    {
        size = btVector3(new_box.m_length, new_box.m_width, new_box.m_height);
        origin = btVector3(new_box.position.position(0) + new_box.center_of_mass.position(0),
                           new_box.position.position(1) + new_box.center_of_mass.position(1),
                           new_box.position.position(2) + new_box.center_of_mass.position(2));
    }

    this->localCreateRigidBox(mass, size, origin);
}

void PhysicsBullet::addNewBoxToPhysicsNoRot(bpa::Box &new_box)
{
    btScalar mass(0.0);
    btVector3 size(new_box.m_length, new_box.m_width, new_box.m_height);
    btVector3 origin(new_box.position.position(0) + new_box.center_of_mass.position(0),
                     new_box.position.position(1) + new_box.center_of_mass.position(1),
                     new_box.position.position(2) + new_box.center_of_mass.position(2));
    this->localCreateRigidBox(mass, size, origin);
}

void PhysicsBullet::addNewBoxesToPhysics(std::vector<bpa::Box> &packed_boxes)
{
    for(bpa::Box packed_box : packed_boxes)
        addNewBoxToPhysics(packed_box);
}

// Suppose every time have 4 contacts point between two boxes
double PhysicsBullet::getSupportArea(bpa::Box &new_box, bpa::Box &old_box)
{
    double diag = -1.0;
    double length = -1.0;
    double width = -1.0;
    double area = 0.0;

    ContactResultCallback resultCallback;
    if(this->isBoxContact(new_box, old_box, resultCallback))
    {
        for(size_t i = 1; i < resultCallback.collisionVec.size(); ++i)
        {
           //get the down side contact area
           double dist = (resultCallback.posWorldOnAVec[i].getX()-resultCallback.posWorldOnAVec[0].getX())*(resultCallback.posWorldOnAVec[i].getX()-resultCallback.posWorldOnAVec[0].getX())
                        + (resultCallback.posWorldOnAVec[i].getY()-resultCallback.posWorldOnAVec[0].getY())*(resultCallback.posWorldOnAVec[i].getY()-resultCallback.posWorldOnAVec[0].getY());

            dist = std::sqrt(dist);

            if(floatGreaterThan(dist, diag))
            {
                length = width;
                width = diag;
                diag = dist;
            }
            else if(floatGreaterThan(dist, width))
            {
                length = width;
                width = dist;
            }
            else /*if(floatGreaterThan(dist, length))*/
                length = dist;
        }

        if(length > 0 && width > 0)
            area = length*width;
        else
            area = 0.0;
    }
    else
    {
         area = 0.0;
    }
    return area;
}

double PhysicsBullet::getContactArea(bpa::Box &new_box, bpa::Box &old_box)
{
    double diag = -1.0;
    double length = -1.0;
    double width = -1.0;
    double area = 0.0;

    ContactResultCallback resultCallback;
    if(this->isBoxContact(new_box, old_box, resultCallback))
    {
        for(size_t i = 1; i < resultCallback.collisionVec.size(); ++i)
        {
            double dist = 0.0;
/*            if(floatEqual(resultCallback.posWorldOnAVec[1].getZ(), resultCallback.posWorldOnAVec[0].getZ()) && floatEqual(resultCallback.posWorldOnAVec[2].getZ(), resultCallback.posWorldOnAVec[0].getZ()) && floatEqual(resultCallback.posWorldOnAVec[3].getZ(), resultCallback.posWorldOnAVec[0].getZ()))
            {
                //get the down contact area
                dist = (resultCallback.posWorldOnAVec[i].getX()-resultCallback.posWorldOnAVec[0].getX())*(resultCallback.posWorldOnAVec[i].getX()-resultCallback.posWorldOnAVec[0].getX())
                        + (resultCallback.posWorldOnAVec[i].getY()-resultCallback.posWorldOnAVec[0].getY())*(resultCallback.posWorldOnAVec[i].getY()-resultCallback.posWorldOnAVec[0].getY());

            }
            //get side contact area
            else */if(floatEqual(resultCallback.posWorldOnAVec[1].getX(), resultCallback.posWorldOnAVec[0].getX()) && floatEqual(resultCallback.posWorldOnAVec[2].getX(), resultCallback.posWorldOnAVec[0].getX()) && floatEqual(resultCallback.posWorldOnAVec[3].getX(), resultCallback.posWorldOnAVec[0].getX()))
            {
                dist = (resultCallback.posWorldOnAVec[i].getZ()-resultCallback.posWorldOnAVec[0].getZ())*(resultCallback.posWorldOnAVec[i].getZ()-resultCallback.posWorldOnAVec[0].getZ())
                          + (resultCallback.posWorldOnAVec[i].getY()-resultCallback.posWorldOnAVec[0].getY())*(resultCallback.posWorldOnAVec[i].getY()-resultCallback.posWorldOnAVec[0].getY());
            }
            else
            {
                dist = (resultCallback.posWorldOnAVec[i].getX()-resultCallback.posWorldOnAVec[0].getX())*(resultCallback.posWorldOnAVec[i].getX()-resultCallback.posWorldOnAVec[0].getX())
                          + (resultCallback.posWorldOnAVec[i].getZ()-resultCallback.posWorldOnAVec[0].getZ())*(resultCallback.posWorldOnAVec[i].getZ()-resultCallback.posWorldOnAVec[0].getZ());
            }

            dist = std::sqrt(dist);

            if(floatGreaterThan(dist, diag))
            {
                length = width;
                width = diag;
                diag = dist;
            }
            else if(floatGreaterThan(dist, width))
            {
                length = width;
                width = dist;
            }
            else
                length = dist;
        }

        if(length > 0 && width > 0)
            area = length*width;
        else
            area = 0.0;
    }
    else
    {
         area = 0.0;
    }
    return area;
}

double PhysicsBullet::getMinimumDistance(bpa::Box &new_box)
{
    btVector3 size(new_box.m_length, new_box.m_width, new_box.m_height);
    btVector3 origin(new_box.position.position(0) + new_box.center_of_mass.position(0),
                     new_box.position.position(1) + new_box.center_of_mass.position(1),
                     new_box.position.position(2) + new_box.center_of_mass.position(2));
    std::shared_ptr<btConvexShape> colShape (new btBoxShape(btVector3(size.getX()/2.0, size.getY()/2.0, size.getZ()/2.0)));
    btTransform new_box_transform;
    new_box_transform.setIdentity();
    new_box_transform.setOrigin(btVector3(origin.getX(), origin.getY(), origin.getZ()));  // center of mass

    double min_distance = std::numeric_limits<float>::max();

    for(size_t body_id = 0; body_id < m_convexShapes.size(); ++body_id)
    {
        btVoronoiSimplexSolver sGjkSimplexSolver;
        btGjkEpaPenetrationDepthSolver epa;
        btGjkPairDetector convexConvex(colShape.get(), m_convexShapes.at(body_id), &sGjkSimplexSolver, &epa);

        btPointCollector gjkOutput;
        btGjkPairDetector::ClosestPointInput input;
        input.m_transformA = new_box_transform;
        input.m_transformB = m_collisionBodies.at(body_id)->getWorldTransform();
        convexConvex.getClosestPoints(input, gjkOutput, 0);
        if(gjkOutput.m_distance < min_distance)
        {
            min_distance = gjkOutput.m_distance;
        }
    }
    return min_distance;
}

double PhysicsBullet::getMinimumDistance(bpa::Box &new_box, bpa::Box &old_box)
{
    btVector3 size_new(new_box.m_length, new_box.m_width, new_box.m_height);
    btVector3 origin_new(new_box.position.position(0) + new_box.center_of_mass.position(0),
                         new_box.position.position(1) + new_box.center_of_mass.position(1),
                         new_box.position.position(2) + new_box.center_of_mass.position(2));
    std::shared_ptr<btConvexShape> col_shape_new (new btBoxShape(btVector3(size_new.getX()/2.0, size_new.getY()/2.0, size_new.getZ()/2.0)));
    btTransform new_box_transform;
    new_box_transform.setIdentity();
    new_box_transform.setOrigin(btVector3(origin_new.getX(), origin_new.getY(), origin_new.getZ()));

    btVector3 size_old(old_box.m_length, old_box.m_width, old_box.m_height);
    btVector3 origin_old(old_box.position.position(0) + old_box.center_of_mass.position(0),
                         old_box.position.position(1) + old_box.center_of_mass.position(1),
                         old_box.position.position(2) + old_box.center_of_mass.position(2));
    std::shared_ptr<btConvexShape> col_shape_old (new btBoxShape(btVector3(size_old.getX()/2.0, size_old.getY()/2.0, size_old.getZ()/2.0)));
    btTransform old_box_transform;
    old_box_transform.setIdentity();
    old_box_transform.setOrigin(btVector3(origin_old.getX(), origin_old.getY(), origin_old.getZ()));

    btVoronoiSimplexSolver sGjkSimplexSolver;
    btGjkEpaPenetrationDepthSolver epa;
    btGjkPairDetector convexConvex(col_shape_new.get(), col_shape_old.get(), &sGjkSimplexSolver, &epa);

    btPointCollector gjkOutput;
    btGjkPairDetector::ClosestPointInput input;
    input.m_transformA = new_box_transform;
    input.m_transformB = old_box_transform;
    convexConvex.getClosestPoints(input, gjkOutput, 0);
    return gjkOutput.m_distance;
}

void PhysicsBullet::initPhysics()
{
    /* 1. Dynamic worlld ***********************************************************************************************/
	///collision configuration contains default setup for memory, collision setup
	m_collisionConfiguration = new btDefaultCollisionConfiguration();

	///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
    m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
	m_broadphase = new btDbvtBroadphase();

	///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
	btSequentialImpulseConstraintSolver* sol = new btSequentialImpulseConstraintSolver;
	m_solver = sol;

	m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher,m_broadphase,m_solver,m_collisionConfiguration);
//    m_dynamicsWorld->setDebugDrawer(&sDebugDraw);
    m_dynamicsWorld->setGravity(btVector3(0, 0, -10));
}

void PhysicsBullet::addBinBoundingBox()
{
    /* 2. create: groundShapes ******************************************************************************************/
    ///create a few basic rigid bodies
    btScalar Mass(0.0);
    btVector3 Size(100, 100, 0);
    btVector3 Origin(0, 0, -0);
    this->localCreateRigidBox(Mass, Size, Origin);

    //side bound
    btVector3 xSize(0, 4, 5);
    btVector3 ySize(5, 0, 5);
    this->localCreateRigidBox(Mass, xSize, btVector3(0.0, 1.59, 2.0)); //x
    this->localCreateRigidBox(Mass, xSize, btVector3(2.44, 1.59, 2.0));
    this->localCreateRigidBox(Mass, ySize, btVector3(1.22, 0.0, 2.0)); //y
    this->localCreateRigidBox(Mass, ySize, btVector3(1.22, 3.18, 2.0));
}

void PhysicsBullet::exitPhysics()
{
	//cleanup in the reverse order of creation/initialization
	//remove the rigidbodies from the dynamics world and delete them
    int i;
    for (i=m_dynamicsWorld->getNumCollisionObjects()-1; i>=0 ;i--)
    {
        btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
        btRigidBody* body = btRigidBody::upcast(obj);
        if (body && body->getMotionState())
        {
            delete body->getMotionState();
        }
        m_dynamicsWorld->removeCollisionObject( obj );
        delete obj;
    }
    m_collisionBodies.clear();
    m_convexShapes.clear();
	delete m_dynamicsWorld;
	delete m_solver;
	delete m_broadphase;
	delete m_dispatcher;
	delete m_collisionConfiguration;	
}

void PhysicsBullet::localCreateRigidBox(btScalar mass, btVector3 size, btVector3 origin)
{
    btCollisionShape* colShape = new btBoxShape(btVector3(size.getX()/2.0, size.getY()/2.0, size.getZ()/2.0));
    colShape->setMargin(-0.00001);  // the margin affect the contact distance and normal.
    btConvexShape* convexShape = new btBoxShape(btVector3(size.getX()/2.0, size.getY()/2.0, size.getZ()/2.0));
    btTransform startTransform;
    startTransform.setIdentity();
    startTransform.setOrigin(btVector3(origin.getX(), origin.getY(), origin.getZ()));  // center of mass

    //rigidbody is dynamic if and only if mass is non zero, otherwise static
    bool isDynamic = (mass != 0.f);

    btVector3 localInertia(0,0,0);
    if (isDynamic)
        colShape->calculateLocalInertia(mass,localInertia);

    //using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
#define USE_MOTIONSTATE 1
#ifdef USE_MOTIONSTATE
    btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
    btRigidBody::btRigidBodyConstructionInfo cInfo(mass,myMotionState,colShape,localInertia);
    btRigidBody* body = new btRigidBody(cInfo);
    body->setContactProcessingThreshold(m_defaultContactProcessingThreshold);

#else
    btRigidBody* body = new btRigidBody(mass,0,colShape,localInertia);
    body->setWorldTransform(startTransform);
#endif//

    body->setRollingFriction(0.05);
    body->setFriction(1);
    m_dynamicsWorld->addCollisionObject(body);
	m_convexShapes.push_back(convexShape);
    m_collisionBodies.push_back(body);

    m_dynamicsWorld->performDiscreteCollisionDetection();
    m_dynamicsWorld->updateAabbs();
    m_dynamicsWorld->computeOverlappingPairs();
}

btRigidBody* PhysicsBullet::localCreateRigidBody(btScalar mass, btCollisionShape* shape, btVector3 origin)
{
    btAssert((!shape || shape->getShapeType() != INVALID_SHAPE_PROXYTYPE));
    shape->setMargin(0.0);

    btTransform startTransform;
    startTransform.setIdentity();
    startTransform.setOrigin(btVector3(origin.getX(), origin.getY(), origin.getZ()));

    bool isDynamic = (mass != 0.f);

    btVector3 localInertia(0,0,0);
    if (isDynamic)
        shape->calculateLocalInertia(mass,localInertia);

    btDefaultMotionState *myMotionState = new btDefaultMotionState(startTransform);
    btRigidBody::btRigidBodyConstructionInfo cInfo(mass, myMotionState, shape, localInertia);
    btRigidBody *body = new btRigidBody(cInfo);
    body->setContactProcessingThreshold(m_defaultContactProcessingThreshold);

//    m_dynamicsWorld->addCollisionObject(body);
//    m_dynamicsWorld->addRigidBody(body);
//    m_collisionBodies.push_back(body);

    return body;
}

btRigidBody *PhysicsBullet::localCreateRigidBodyRot(btScalar mass, btCollisionShape *shape, btVector3 origin, btQuaternion q)
{
    btAssert((!shape || shape->getShapeType() != INVALID_SHAPE_PROXYTYPE));
    shape->setMargin(0.0);

    btTransform startTransform;
    startTransform.setIdentity();
    startTransform.setOrigin(btVector3(origin.getX(), origin.getY(), origin.getZ()));
    startTransform.setRotation(q);

    bool isDynamic = (mass != 0.f);

    btVector3 localInertia(0,0,0);
    if (isDynamic)
        shape->calculateLocalInertia(mass,localInertia);

    btDefaultMotionState *myMotionState = new btDefaultMotionState(startTransform);
    btRigidBody::btRigidBodyConstructionInfo cInfo(mass, myMotionState, shape, localInertia);
    btRigidBody *body = new btRigidBody(cInfo);
    body->setContactProcessingThreshold(m_defaultContactProcessingThreshold);

    return body;
}

btRigidBody *PhysicsBullet::localCreatePointSphere(btVector3 origin)
{
    btCollisionShape* shape = new btSphereShape(0.0);
    shape->setMargin(0.0);
    btScalar mass = 0.0f;

    btTransform startTransform;
    startTransform.setIdentity();
    startTransform.setOrigin(btVector3(origin.getX(), origin.getY(), origin.getZ()));

    bool isDynamic = (mass != 0.f);

    btVector3 localInertia(0,0,0);
    if (isDynamic)
        shape->calculateLocalInertia(mass,localInertia);

    btDefaultMotionState *myMotionState = new btDefaultMotionState(startTransform);
    btRigidBody::btRigidBodyConstructionInfo cInfo(mass, myMotionState, shape, localInertia);
    btRigidBody *body = new btRigidBody(cInfo);
    body->setContactProcessingThreshold(m_defaultContactProcessingThreshold);

    return body;
}

bool PhysicsBullet::isPointContact(Eigen::Vector3d &point)
{           
    std::shared_ptr<btRigidBody> pointBall (localCreatePointSphere( btVector3(point(0),point(1),point(2))));
    /************** 1: point & world, setMargin = -0.0001*/
    ContactResultCallback resultCallback;
    m_dynamicsWorld->contactTest(pointBall.get(), resultCallback);

    if(resultCallback.collisionVec.size() > 0) {return true;}
    else {return false;}
}

bool PhysicsBullet::isBoxContact(bpa::Box new_box, bpa::Box &old_box, ContactResultCallback &resultCallback)
{
    bool result=true;
    // if box and other boxes are touch contact
    if (m_dynamicsWorld)
    {
        btScalar mass(0.0);
        btVector3 size(new_box.m_length, new_box.m_width, new_box.m_height);
        btVector3 origin(new_box.position.position(0) + new_box.center_of_mass.position(0),
                         new_box.position.position(1) + new_box.center_of_mass.position(1),
                         new_box.position.position(2) + new_box.center_of_mass.position(2));
        std::shared_ptr<btCollisionShape> colShape (new btBoxShape(btVector3(size.getX()/2.0, size.getY()/2.0, size.getZ()/2.0)));
        std::shared_ptr<btRigidBody> body (this->localCreateRigidBody(mass, colShape.get(), origin));

        //TODO:  Some bugs for box and box contact, without number 0.00001?
        btVector3 sizeOld(old_box.m_length + 0.0001, old_box.m_width + 0.0001, old_box.m_height);
        btVector3 originOld(old_box.position.position(0) + old_box.center_of_mass.position(0),
                         old_box.position.position(1) + old_box.center_of_mass.position(1),
                         old_box.position.position(2) + old_box.center_of_mass.position(2) + 0.001);
        std::shared_ptr<btCollisionShape> shapeOld (new btBoxShape(btVector3(sizeOld.getX()/2.0, sizeOld.getY()/2.0, sizeOld.getZ()/2.0)));
        std::shared_ptr<btRigidBody> bodyOld (this->localCreateRigidBody(mass, shapeOld.get(), originOld));

#pragma omp critical
        {
        m_dynamicsWorld->contactPairTest(body.get(), bodyOld.get(), resultCallback);
        }

        double minDistance = 0.0;
        for(size_t i = 0; i< resultCallback.collisionVec.size(); ++i)
        {   // distance are negtive
            if( floatLessThan(resultCallback.distanceVec[i], minDistance) )
            { minDistance = resultCallback.distanceVec[i]; }
        }
//        if( (resultCallback.collisionVec.size() >0) && (floatEqual(minDistance, 0.0)) )
        if( (resultCallback.collisionVec.size() >0) && (floatLessEqual(std::fabs(minDistance), 0.01)) )
        {
            result=true;
        }
        else
        {
            result=false;
        }
    }
    else
    {
        result=false;
    }
    return result;
}

bool PhysicsBullet::isColliding(bpa::Box &new_box)
{
    // if box and other boxes are touch contact, should not be consided as collided !!!
    if (m_dynamicsWorld)
    {
        btScalar mass(0.0);
        btVector3 size(new_box.m_length-0.01, new_box.m_width-0.01, new_box.m_height-0.01);
        btVector3 origin(new_box.position.position(0) + new_box.center_of_mass.position(0),
                         new_box.position.position(1) + new_box.center_of_mass.position(1),
                         new_box.position.position(2) + new_box.center_of_mass.position(2));
        std::shared_ptr<btCollisionShape> colShape (new btBoxShape(btVector3(size.getX()/2.0, size.getY()/2.0, size.getZ()/2.0)));
        std::shared_ptr<btRigidBody> body (this->localCreateRigidBody(mass, colShape.get(), origin));

        ContactResultCallback resultCallback;
#pragma omp critical
{
        m_dynamicsWorld->contactTest(body.get(), resultCallback);
}
        // check all contact distances, and exclude the surface contact points
        double maxDistance = 0.0;
        for(size_t i = 0; i< resultCallback.collisionVec.size(); ++i)
        {   // distance are negtive
            if( floatLessThan(resultCallback.distanceVec[i], maxDistance) )
            { maxDistance = resultCallback.distanceVec[i]; /*std::cout << "collide distance is = " << maxDistance << std::endl;*/}
        }
//        if( (resultCallback.collisionVec.size() >0) && (!floatEqual(maxDistance, 0.0)) )
        if( (resultCallback.collisionVec.size() >0) && (floatGreaterThan(std::fabs(maxDistance), 0.01)) )
            return true;
        else
            return false;

    }
    else
        return false;
}

bool PhysicsBullet::isCollidingBox(bpa::Box &new_box, bpa::Box &old_box)
{
    // if box and other boxes are touch contact, should not be consided as collided !!!
    if (m_dynamicsWorld)
    {
        //new box
        Eigen::Vector3d bbox, old_bbox;
        if(new_box.is_rotated)
           bbox << new_box.m_width, new_box.m_length, new_box.m_height;
        else
           bbox << new_box.m_length, new_box.m_width, new_box.m_height;
        btScalar mass(0.0);       
        btVector3 size(btVector3(bbox(0)-0.01, bbox(1)-0.01, bbox(2)-0.01));
        btVector3 origin(new_box.position.position(0) + bbox(0)/2.0,
                         new_box.position.position(1) + bbox(1)/2.0,
                         new_box.position.position(2) + bbox(2)/2.0);
        std::shared_ptr<btCollisionShape> colShape(new btBoxShape(btVector3(size.getX()/2.0, size.getY()/2.0, size.getZ()/2.0)));
        std::shared_ptr<btRigidBody> body(this->localCreateRigidBody(mass, colShape.get(), origin));

        //old box
        if(old_box.is_rotated)
            old_bbox << old_box.m_width, old_box.m_length, old_box.m_height;
         else
            old_bbox << old_box.m_length, old_box.m_width, old_box.m_height;
        btVector3 sizeOld(btVector3(old_bbox(0), old_bbox(1), old_bbox(2)));
        btVector3 originOld(old_box.position.position(0) + old_bbox(0)/2.0,
                            old_box.position.position(1) + old_bbox(1)/2.0,
                            old_box.position.position(2) + old_bbox(2)/2.0);
        std::shared_ptr<btCollisionShape> shapeOld(new btBoxShape(btVector3(sizeOld.getX()/2.0, sizeOld.getY()/2.0, sizeOld.getZ()/2.0)));
        std::shared_ptr<btRigidBody> bodyOld(this->localCreateRigidBody(mass, shapeOld.get(), originOld));

        //checking collistion
        ContactResultCallback resultCallback;
#pragma omp critical
{
        m_dynamicsWorld->contactPairTest(body.get(), bodyOld.get(), resultCallback);
}
        // check all contact distances, and exclude the surface contact points
        double maxDistance = 0.0;
        for(size_t i = 0; i< resultCallback.collisionVec.size(); ++i)
        {
            if( floatLessThan(resultCallback.distanceVec[i], maxDistance) )
            { maxDistance = resultCallback.distanceVec[i]; }
        }
//        if( (resultCallback.collisionVec.size() >0) && (!floatEqual(maxDistance, 0.0)) )
        if( (resultCallback.collisionVec.size() >0) && (floatGreaterThan(std::fabs(maxDistance), 0.01)) )
            return true;
        else
            return false;
    }
    else
        return false;
}

bool PhysicsBullet::isCollidingActor(bpp_actor::Actor& new_actor, bpp_actor::Actor& old_actor)
{
    // if box and other boxes are touch contact, should not be consided as collided !!!
    double offset = 0.1; // 0.01; for numerical error (contact case)
    if (m_dynamicsWorld)
    {
        //new box
        btScalar mass(0.0);
        btVector3 size(new_actor.bbox.x-offset, new_actor.bbox.y-offset, new_actor.bbox.z-offset);
        btVector3 origin(new_actor.targetPoseVec[0].position.x,
                         new_actor.targetPoseVec[0].position.y,
                         new_actor.targetPoseVec[0].position.z);
        btQuaternion q(new_actor.targetPoseVec[0].orientation.x, new_actor.targetPoseVec[0].orientation.y,
                       new_actor.targetPoseVec[0].orientation.z, new_actor.targetPoseVec[0].orientation.w);
        std::shared_ptr<btCollisionShape> colShape(new btBoxShape(btVector3(size.getX()/2.0, size.getY()/2.0, size.getZ()/2.0)));
        std::shared_ptr<btRigidBody> body(this->localCreateRigidBodyRot(mass, colShape.get(), origin, q));

        //old box
        btVector3 sizeOld(btVector3(old_actor.bbox.x, old_actor.bbox.y, old_actor.bbox.z));
        btVector3 originOld(old_actor.targetPoseVec[0].position.x,
                            old_actor.targetPoseVec[0].position.y,
                            old_actor.targetPoseVec[0].position.z);
        btQuaternion q_old(old_actor.targetPoseVec[0].orientation.x, old_actor.targetPoseVec[0].orientation.y,
                          old_actor.targetPoseVec[0].orientation.z, old_actor.targetPoseVec[0].orientation.w);
        std::shared_ptr<btCollisionShape> shapeOld(new btBoxShape(btVector3(sizeOld.getX()/2.0, sizeOld.getY()/2.0, sizeOld.getZ()/2.0)));
        std::shared_ptr<btRigidBody> bodyOld(this->localCreateRigidBodyRot(mass, shapeOld.get(), originOld, q_old));

        //checking collistion
        ContactResultCallback resultCallback;
#pragma omp critical
{
        m_dynamicsWorld->contactPairTest(body.get(), bodyOld.get(), resultCallback);
}
        // check all contact distances, and exclude the surface contact points
        double maxDistance = 0.0;
        for(size_t i = 0; i< resultCallback.collisionVec.size(); ++i)
        {
            if( floatLessThan(resultCallback.distanceVec[i], maxDistance) )
            { maxDistance = resultCallback.distanceVec[i]; }
        }
//        if( (resultCallback.collisionVec.size() >0) && (!floatEqual(maxDistance, 0.0)) )
        if( (resultCallback.collisionVec.size() >0) && (floatGreaterThan(std::fabs(maxDistance), 0.02)) )
            return true;
        else
            return false;
    }
    else
        return false;
}

bool PhysicsBullet::isCollidingActorDesiredPose(bpp_actor::Actor& new_actor, bpp_actor::Actor& old_actor)
{
    // if box and other boxes are touch contact, should not be consided as collided !!!
    double offset = 0.01; //0.001;for numerical error (contact case)
    if (m_dynamicsWorld)
    {
        //new box
        btScalar mass(0.0);
        btVector3 size(new_actor.bbox.x-offset, new_actor.bbox.y-offset, new_actor.bbox.z-offset);
        btVector3 origin(new_actor.desiredPoseVec[0].position.x,
                         new_actor.desiredPoseVec[0].position.y,
                         new_actor.desiredPoseVec[0].position.z);
        btQuaternion q(new_actor.desiredPoseVec[0].orientation.x, new_actor.desiredPoseVec[0].orientation.y,
                       new_actor.desiredPoseVec[0].orientation.z, new_actor.desiredPoseVec[0].orientation.w);
        std::shared_ptr<btCollisionShape> colShape(new btBoxShape(btVector3(size.getX()/2.0, size.getY()/2.0, size.getZ()/2.0)));
        std::shared_ptr<btRigidBody> body(this->localCreateRigidBodyRot(mass, colShape.get(), origin, q));

        //old box
        btVector3 sizeOld(btVector3(old_actor.bbox.x, old_actor.bbox.y, old_actor.bbox.z));
        btVector3 originOld(old_actor.desiredPoseVec[0].position.x,
                            old_actor.desiredPoseVec[0].position.y,
                            old_actor.desiredPoseVec[0].position.z);
        btQuaternion q_old(old_actor.desiredPoseVec[0].orientation.x, old_actor.desiredPoseVec[0].orientation.y,
                          old_actor.desiredPoseVec[0].orientation.z, old_actor.desiredPoseVec[0].orientation.w);
        std::shared_ptr<btCollisionShape> shapeOld(new btBoxShape(btVector3(sizeOld.getX()/2.0, sizeOld.getY()/2.0, sizeOld.getZ()/2.0)));
        std::shared_ptr<btRigidBody> bodyOld(this->localCreateRigidBodyRot(mass, shapeOld.get(), originOld, q_old));

        //checking collistion
        ContactResultCallback resultCallback;
#pragma omp critical
{
        m_dynamicsWorld->contactPairTest(body.get(), bodyOld.get(), resultCallback);
}
        // check all contact distances, and exclude the surface contact points
        double maxDistance = 0.0;
        for(size_t i = 0; i< resultCallback.collisionVec.size(); ++i)
        {
            if( floatLessThan(resultCallback.distanceVec[i], maxDistance) )
            { maxDistance = resultCallback.distanceVec[i]; }
        }
//        if( (resultCallback.collisionVec.size() >0) && (!floatEqual(maxDistance, 0.0)) )
        if( (resultCallback.collisionVec.size() >0) && (floatGreaterThan(std::fabs(maxDistance), 0.02)) )
            return true;
        else
            return false;
    }
    else
        return false;
}

// ////////////////////////////////////////////////////////////////////////////////////////////////
PhysicsBullet::ContactResultCallback::ContactResultCallback():
    collision(false),
    distance(0),
    positionWorldOnA(0.0, 0.0, 0.0),
    positionWorldOnB(0.0, 0.0, 0.0),
    localPointA(0.0, 0.0, 0.0),
    localPointB(0.0, 0.0, 0.0),
    normalWorldOnB(0.0, 0.0, 0.0)
{
}

void PhysicsBullet::ContactResultCallback::clearDatas()
{
    this->collision = false;
    this->distance = 0;
    this->positionWorldOnA.setValue(0.0, 0.0, 0.0);
    this->positionWorldOnB.setValue(0.0, 0.0, 0.0);
    this->normalWorldOnB.setValue(0.0, 0.0, 0.0);
    this->localPointA.setValue(0.0, 0.0, 0.0);
    this->localPointB.setValue(0.0, 0.0, 0.0);

    this->collisionVec.clear();
    this->distanceVec.clear();
    this->posWorldOnAVec.clear();
    this->posWorldOnBVec.clear();
    this->normalWorldOnBVec.clear();
}

btScalar PhysicsBullet::ContactResultCallback::addSingleResult(btManifoldPoint& cp, const btCollisionObjectWrapper* colObj0, int partId0, int index0, const btCollisionObjectWrapper* colObj1, int partId1, int index1)
{
    this->collision = true;
    this->distance = cp.getDistance();
    this->positionWorldOnA = cp.getPositionWorldOnA();
    this->positionWorldOnB = cp.getPositionWorldOnB();
    this->normalWorldOnB = cp.m_normalWorldOnB;
    this->localPointA = cp.m_localPointA;
    this->localPointB = cp.m_localPointB;

    this->collisionVec.push_back(collision);
    this->distanceVec.push_back(distance);
    this->posWorldOnAVec.push_back(positionWorldOnA);
    this->posWorldOnBVec.push_back(positionWorldOnB);
    this->normalWorldOnBVec.push_back(normalWorldOnB);

    return 0;
}
