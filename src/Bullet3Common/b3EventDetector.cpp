#include "b3EventDetector.h"
#include "BulletSoftBody/btSoftBody.h"
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"
#include "../examples/SharedMemory/PhysicsServerCommandProcessor.h"
#include "../examples/SharedMemory/PhysicsServerSharedMemory.h"
#include <sstream>
#include <cmath>
#include <iostream>

b3EventDetector gEventDetector; // define global instance

b3EventDetector::b3EventDetector() {}

void b3EventDetector::resetCurrentWholeBodyDeformation(const btSoftBody* sb) {
    m_bodyDeformationRecord[sb].currentWholeBodyDeformation = 0.0;
}

void b3EventDetector::updateDeformationEvent(const btSoftBody* sb, int tetIndex, const btMatrix3x3& F) {
    m_bodyDeformationRecord[sb].currentWholeBodyDeformation += calculateTetDeformation(F);
}

btScalar b3EventDetector::calculateTetDeformation(const btMatrix3x3& F) const {
    /* 
    C: Right Cauchy-Green tensor from deformation gradient, 
    captures total deformation and removes rigid body rotation.
    E: Green-Lagrange strain tensor: non-linear strain from undeformed state.
    */
    btMatrix3x3 C = F.transposeTimes(F);
    btMatrix3x3 I;
    I.setIdentity();
    btMatrix3x3 E = (C - I) * 0.5;

    // Frobenius norm of E (magnitude)
    btScalar eNormSquared =
        E[0].dot(E[0]) +
        E[1].dot(E[1]) +
        E[2].dot(E[2]);

    btScalar eNorm = std::sqrt(eNormSquared);

    return eNorm;
}

DeformationLevel b3EventDetector::classifyDeformation(btScalar eNorm) const {

    /*if (eNorm > 0.1) {
    printf("Deformation norm: %.6f\n", eNorm);
    }*/

    // TODO: tune buckets empirically
    if (eNorm < 0.02) return DeformationLevel::NO;
    if (eNorm < 0.1) return DeformationLevel::VERY_LOW;
    if (eNorm < 0.2) return DeformationLevel::LOW;
    if (eNorm < 0.3) return DeformationLevel::MEDIUM;
    if (eNorm < 0.4) return DeformationLevel::HIGH;
    return DeformationLevel::VERY_HIGH;
}

std::string b3EventDetector::makeDeformationString(DeformationLevel level) const {
    switch (level) {
    case DeformationLevel::NO: return "no";
    case DeformationLevel::VERY_LOW: return "very low";
    case DeformationLevel::LOW: return "low";
    case DeformationLevel::MEDIUM: return "medium";
    case DeformationLevel::HIGH: return "high";
    case DeformationLevel::VERY_HIGH: return "very high";
    default: return "unknown";
    }
}

void b3EventDetector::setContacting(const btSoftBody* sb, bool isContacting) {
    m_bodyDeformationRecord[sb].isContacting = isContacting;
}

ContactState b3EventDetector::detectContactEvent(const btSoftBody* sb) {
    ContactState contactState;
    auto& record = m_bodyDeformationRecord[sb];

    if (record.isContacting && !record.wasContacting) { // new contact begins
        contactState = ContactState::STARTS;
    }
    else if (!record.isContacting && record.wasContacting) { // contact ends
        contactState = ContactState::ENDS;
    }
    else {
        contactState = ContactState::CONTINUES;
    }

    record.wasContacting = record.isContacting;

    return contactState;
}

extern "C" const btScalar b3GetDeformationEventString()
{
    static btScalar result = gEventDetector.checkForEvent();
    return result;
}

btScalar b3EventDetector::checkForEvent()
{
    btScalar deformationAmount = 0.0;

    for (auto& pair : m_bodyDeformationRecord)
    {
        const ObjectRecord& rec = pair.second;
        // We’ll just report the max of current vs previous
        deformationAmount = std::max(deformationAmount, rec.currentWholeBodyDeformation);
    }

    return deformationAmount;
}


void b3EventDetector::setCommandProcessor(PhysicsServerCommandProcessor* proc) {
    m_commandProcessor = proc;
}

void b3EventDetector::setObjectName(const btCollisionObject* obj, const std::string& name) {
    gObjectNames[obj] = name;
}