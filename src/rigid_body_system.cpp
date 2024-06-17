#include "../include/rigid_body_system.h"

#include <cmath>
#include <assert.h>
#include <chrono>

physics_engine::RigidBodySystem::RigidBodySystem() {
    m_odeSolveMicroseconds = new long long[ProfilingSamples];
    m_constraintSolveMicroseconds = new long long[ProfilingSamples];
    m_forceEvalMicroseconds = new long long[ProfilingSamples];
    m_constraintEvalMicroseconds = new long long[ProfilingSamples];
    m_frameIndex = 0;

    for (int i = 0; i < ProfilingSamples; i++) {
        m_odeSolveMicroseconds[i] = -1;
        m_constraintSolveMicroseconds[i] = -1;
        m_forceEvalMicroseconds[i] = -1;
        m_constraintEvalMicroseconds[i] = -1;
    }
}

physics_engine::RigidBodySystem::~RigidBodySystem() {
    delete[] m_odeSolveMicroseconds;
    delete[] m_constraintEvalMicroseconds;
    delete[] m_forceEvalMicroseconds;
    delete[] m_constraintEvalMicroseconds;

    m_state.destroy();
}

void physics_engine::RigidBodySystem::reset() {
    m_rigidBodies.clear();
    m_constraints.clear();
    m_forceGenerators.clear();
}

void physics_engine::RigidBodySystem::process(double dt, int steps) {
    /* void */
}

void physics_engine::RigidBodySystem::addRigidBody(RigidBody *body) {
    m_rigidBodies.push_back(body);
    body->index = (int)m_rigidBodies.size() - 1;
}

void physics_engine::RigidBodySystem::removeRigidBody(RigidBody *body) {
    m_rigidBodies[body->index] = m_rigidBodies.back(); // replace with last element
    m_rigidBodies[body->index]->index = body->index; // update index
    m_rigidBodies.resize(m_rigidBodies.size() - 1); // deallocate last space
}