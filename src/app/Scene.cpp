//
// Created by faliszewskii on 16.06.24.
//

#include "Scene.h"
#include "../interface/camera/CameraAnchor.h"
#include "btBulletCollisionCommon.h"
#include "BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h"
#include "BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h"

glm::mat3 extractMat3FromBtTransform(const btTransform& transform) {
    const btMatrix3x3& basis = transform.getBasis();

    // Convert the btMatrix3x3 basis into a glm::mat3
    glm::mat3 result;
    result[0][0] = basis[0][0]; result[0][1] = basis[1][0]; result[0][2] = basis[2][0];
    result[1][0] = basis[0][1]; result[1][1] = basis[1][1]; result[1][2] = basis[2][1];
    result[2][0] = basis[0][2]; result[2][1] = basis[1][2]; result[2][2] = basis[2][2];

    return result;
}

Scene::Scene(AppContext &appContext) : appContext(appContext) {
    appContext.camera = std::make_unique<CameraAnchor>(1920, 1080, glm::vec3(0.0f, 3.0f, 3.0f), glm::vec3(0.f), glm::vec3(-M_PI / 4, 0, 0));
    appContext.frameBufferManager = std::make_unique<FrameBufferManager>();
    appContext.frameBufferManager->create_buffers(appContext.camera->screenWidth, appContext.camera->screenHeight);

    // TODO --- Initialization of the app state goes here.

    appContext.phongShader = std::make_unique<Shader>(Shader::createTraditionalShader(
            "../res/shaders/phong/phong.vert", "../res/shaders/phong/phong.frag"));
    appContext.pointShader = std::make_unique<Shader>(Shader::createTraditionalShader(
            "../res/shaders/point/point.vert", "../res/shaders/point/point.frag"));

    appContext.quad = std::make_unique<Quad>();
    appContext.light = std::make_unique<PointLight>();
    appContext.light->position = {-0.5f , 0.25f, 0.25f};
    appContext.lightBulb = std::make_unique<Point>();

    appContext.bunny = std::make_unique<Model>("../res/models/stanfordBunny.obj");

    //-----includes_end-----

    int i;
    appContext.collisionConfiguration = new btDefaultCollisionConfiguration();
    appContext.dispatcher = new btCollisionDispatcher(appContext.collisionConfiguration);
    appContext.overlappingPairCache = new btDbvtBroadphase();
    appContext.solver = new btSequentialImpulseConstraintSolver;
    appContext.dynamicsWorld = new btDiscreteDynamicsWorld(appContext.dispatcher, appContext.overlappingPairCache, appContext.solver, appContext.collisionConfiguration);
    appContext.dynamicsWorld->setGravity(btVector3(0, -9.81, 0));


    // Create ground plane
    btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0, 1, 0), 0);
    appContext.collisionShapes.push_back(groundShape);

    btDefaultMotionState* groundMotionState = new btDefaultMotionState(btTransform(
            btQuaternion(0, 0, 0, 1), btVector3(0, -1, 0)
    ));
    btRigidBody::btRigidBodyConstructionInfo groundRigidBodyCI(
            0, groundMotionState, groundShape, btVector3(0, 0, 0)
    );
    btRigidBody* groundRigidBody = new btRigidBody(groundRigidBodyCI);
    appContext.dynamicsWorld->addRigidBody(groundRigidBody);

    // Create the chain of cylinders
    const int numLinks = 5;
    const btScalar linkMass = 1.0f;
    const btVector3 linkHalfExtents(0.5, 0.2, 0.5);  // Dimensions of each link
    btVector3 startPosition(0, 5, 0);

    btRigidBody* previousBody = groundRigidBody;

    for (int i = 0; i < numLinks; ++i) {
        // Create collision shape for the link
        btCollisionShape* linkShape = new btCylinderShape(linkHalfExtents);
        appContext.collisionShapes.push_back(linkShape);

        // Create motion state and rigid body
        btTransform linkTransform;
        linkTransform.setIdentity();
        linkTransform.setOrigin(startPosition + btVector3(0, -i * 0.5, 0));

        btDefaultMotionState* linkMotionState = new btDefaultMotionState(linkTransform);

        btVector3 linkInertia(0, 0, 0);
        linkShape->calculateLocalInertia(linkMass, linkInertia);

        btRigidBody::btRigidBodyConstructionInfo linkRigidBodyCI(
                linkMass, linkMotionState, linkShape, linkInertia
        );
        btRigidBody* linkBody = new btRigidBody(linkRigidBodyCI);
        appContext.dynamicsWorld->addRigidBody(linkBody);

        // Add a hinge constraint between the previous and current link
        btVector3 pivotInA(0, -0.25, 0);  // Bottom of the previous link
        btVector3 pivotInB(0, 0.25, 0);   // Top of the current link
        btHingeConstraint* hinge = new btHingeConstraint(
                *previousBody, *linkBody, pivotInA, pivotInB, btVector3(0, 0, 1), btVector3(0, 0, 1)
        );
        appContext.dynamicsWorld->addConstraint(hinge, true);

        previousBody = linkBody;
    }
}

void Scene::update() {
    // TODO --- Here goes scene data update.
    appContext.lightBulb->position = appContext.light->position;
    appContext.lightBulb->color = glm::vec4(appContext.light->color, 1);


    appContext.dynamicsWorld->stepSimulation(1.f / 60.f, 10);

}

void Scene::render() {
    appContext.frameBufferManager->bind();

    auto quadModel = glm::identity<glm::mat4>();
    quadModel = glm::rotate(quadModel, -float(std::numbers::pi / 2), glm::vec3(1, 0, 0));

    auto bunnyModel = glm::identity<glm::mat4>();
    bunnyModel = glm::scale(bunnyModel, glm::vec3(2));

    // TODO --- Here goes scene render.
    appContext.phongShader->use();
    appContext.phongShader->setUniform("model", quadModel);
    appContext.phongShader->setUniform("viewPos", appContext.camera->getViewPosition());
    appContext.phongShader->setUniform("view", appContext.camera->getViewMatrix());
    appContext.phongShader->setUniform("projection", appContext.camera->getProjectionMatrix());
    appContext.phongShader->setUniform("material.hasTexture", false);
    appContext.phongShader->setUniform("material.albedo", glm::vec4(1.0f, 1.0f, 0.0f, 1.0f));
    appContext.phongShader->setUniform("material.shininess", 256.f);
    appContext.light->setupPointLight(*appContext.phongShader);
    appContext.quad->render();
    appContext.phongShader->setUniform("material.albedo", glm::vec4(1.0f, 0.5f, 0.5f, 1.0f));
    appContext.phongShader->setUniform("model", bunnyModel);
//    appContext.bunny->render();

    for (int i = 0; i < appContext.dynamicsWorld->getNumCollisionObjects(); ++i) {
        btCollisionObject* obj = appContext.dynamicsWorld->getCollisionObjectArray()[i];
        btRigidBody* body = btRigidBody::upcast(obj);

        if (body && body->getMotionState()) {
            btTransform transform;
            body->getMotionState()->getWorldTransform(transform);
            glm::mat3 model = extractMat3FromBtTransform(transform);
            appContext.phongShader->setUniform("model", model);
            appContext.bunny->render();
        }
    }

    appContext.pointShader->use();
    appContext.pointShader->setUniform("view", appContext.camera->getViewMatrix());
    appContext.pointShader->setUniform("projection", appContext.camera->getProjectionMatrix());
    appContext.lightBulb->render(*appContext.pointShader);

    appContext.frameBufferManager->unbind();
}
