#include "b3EventDetector.h"
#include "BulletSoftBody/btSoftBody.h"
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"
#include "BulletDynamics/Dynamics/btDynamicsWorld.h"
#include "BulletSoftBody/btDeformableMultiBodyDynamicsWorld.h"
#include "../examples/SharedMemory/PhysicsServerCommandProcessor.h"
#include "../examples/SharedMemory/PhysicsServerSharedMemory.h"
#include <sstream>
#include <cmath>
#include <iostream>
#include <unordered_set>

b3EventDetector gEventDetector; // define global instance

b3EventDetector::b3EventDetector() {}

void b3EventDetector::resetCurrentWholeBodyDeformation(const btSoftBody* sb) {
    m_objectRecords[sb].currentWholeBodyDeformation = 0.0;
}

void b3EventDetector::setContacting(const btSoftBody* sb, bool isContacting) {
    m_objectRecords[sb].isContacting = isContacting;
}

ContactState b3EventDetector::detectContactEvent(const btSoftBody* sb) {
    ContactState contactState;
    auto& record = m_objectRecords[sb];

    if (record.isContacting && !record.wasContacting) { // new contact begins
        contactState = ContactState::STARTS;
    }
    else if (!record.isContacting && record.wasContacting) { // contact ends
        contactState = ContactState::ENDS;
    }
    else if (record.isContacting && record.wasContacting) {
        contactState = ContactState::CONTINUES;
    }
    else {
        contactState = ContactState::NO_CONTACT;
    }

    record.wasContacting = record.isContacting;

    return contactState;
}

void b3EventDetector::updateDeformationEvent(const btSoftBody* sb, int tetIndex, const btMatrix3x3& F) {
    m_objectRecords[sb].currentWholeBodyDeformation += calculateTetDeformation(F);
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

std::string b3EventDetector::checkForEvent() {
    std::string output;

    // First handle contact events using manifolds (more reliable than contactTest)
    btDispatcher* dispatcher = m_dynamicsWorld->getDispatcher();
    int numManifolds = dispatcher->getNumManifolds();

    // Map to track which objects are currently contacting
    std::unordered_map<const btSoftBody*, std::unordered_set<const btCollisionObject*>> currentContacts;

    for (int i = 0; i < numManifolds; i++) {
        const btPersistentManifold* manifold = dispatcher->getInternalManifoldPointer()[i];
        const btCollisionObject* objA = static_cast<const btCollisionObject*>(manifold->getBody0());
        const btCollisionObject* objB = static_cast<const btCollisionObject*>(manifold->getBody1());

        // Check if either is a tracked soft body
        const btSoftBody* sb = nullptr;
        const btCollisionObject* other = nullptr;

        if ((sb = btSoftBody::upcast(objA))) {
            other = objB;
        } else if ((sb = btSoftBody::upcast(objB))) {
            other = objA;
        } else {
            continue; // Skip non-softbody contacts
        }

        // Only count manifolds with actual contacts
        if (manifold->getNumContacts() > 0) {
            currentContacts[sb].insert(other);
        }
    }

    // Process each tracked soft body
    for (auto it = m_objectRecords.begin(); it != m_objectRecords.end(); ++it) { // iterates through tracked objects
        const btCollisionObject* obj = it->first;

        if (const btSoftBody* sb = btSoftBody::upcast(obj)) { // process events from pov of soft bodies only
            auto& record = m_objectRecords[obj]; // retrieve this body's record
            std::string objName = gObjectNames[obj];
            objName.erase(std::remove(objName.begin(), objName.end(), '\0'), objName.end()); // sanitize null chars

            ContactState contactState = detectContactEvent(sb);

            // Process all detected contacts
            for (const btCollisionObject* other : currentContacts[sb]) {
                std::string otherName = getObjectName(other);
                DeformationLevel peakLevel = classifyDeformation(record.peakDeformation);

                if (contactState == ContactState::STARTS) {
                    output += "[Event] <" + objName + "> enters contact with <" + otherName + ">.\n";
                }
                else if (contactState == ContactState::ENDS) {
                    output += "[Event] <" + objName + "> separates from <" + otherName + "> after contact with "
                        + makeDeformationString(peakLevel) + " deformation.\n";
                    record.peakDeformation = 0.0;
                }
                else if (contactState == ContactState::CONTINUES && record.stableCount > 3) {
                    output += "[Event] <" + objName + "> comes to rest on <" + otherName + "> with "
                        + makeDeformationString(peakLevel) + " deformation.\n";
                }
            }

            // 3. Update deformation peak
            if (record.currentWholeBodyDeformation > record.peakDeformation
                && record.isContacting) {
                record.peakDeformation = record.currentWholeBodyDeformation;
            }

            if (abs(record.currentWholeBodyDeformation - record.lastWholeBodyDeformation) < 0.1
                && record.isContacting) {
                record.stableCount++;
            }
            else {
                record.stableCount = 0; // reset
            }

            record.lastWholeBodyDeformation = record.currentWholeBodyDeformation;
        }
    }
    return output;
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

void b3EventDetector::setCommandProcessor(PhysicsServerCommandProcessor* proc) {
    m_commandProcessor = proc;
}

void b3EventDetector::setObjectName(const btCollisionObject* obj, const std::string& name) {
    gObjectNames[obj] = name;
}

std::string b3EventDetector::getObjectName(const btCollisionObject* obj) const {
    auto it = gObjectNames.find(obj);
    if (it != gObjectNames.end()) {
        return it->second;
    }
    else {
        return "?";
    }
}

void b3EventDetector::setDynamicsWorld(btDeformableMultiBodyDynamicsWorld* world) {
    m_dynamicsWorld = world;
}