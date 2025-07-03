#include "b3EventDetector.h"
#include "BulletSoftBody/btSoftBody.h"
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"
#include "BulletDynamics/Dynamics/btDynamicsWorld.h"
#include "../examples/SharedMemory/PhysicsServerCommandProcessor.h"
#include "../examples/SharedMemory/PhysicsServerSharedMemory.h"
#include <sstream>
#include <cmath>
#include <iostream>

b3EventDetector gEventDetector; // define global instance

b3EventDetector::b3EventDetector() {}

//b3EventDetector::b3EventDetector(const PhysicsServerCommandProcessor* physicsServer) 
    //: m_physicsServer(physicsServer)
//{}


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
    else {
        contactState = ContactState::CONTINUES;
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

    for (auto it = m_objectRecords.begin(); it != m_objectRecords.end(); ++it) { // iterates through tracked objects
        const btCollisionObject* obj = it->first;
        auto& record = m_objectRecords[obj]; // retrieve this body's record

        std::string objName = gObjectNames[obj];
        objName.erase(std::remove(objName.begin(), objName.end(), '\0'), objName.end()); // sanitize null chars


        if (const btSoftBody* sb = btSoftBody::upcast(obj)) { // process events from pov of soft bodies only

            /*
            // 1. Find other contacting object
            std::string otherObj = "";
            btDispatcher* dispatcher = m_dynamicsWorld->getDispatcher();
            int numManifolds = dispatcher->getNumManifolds();

            //std::cout << "numManifolds: " << numManifolds << "\n" ;

            for (int i = 0; i < numManifolds; i++) {
                btPersistentManifold* manifold = dispatcher->getManifoldByIndexInternal(i);

                const btCollisionObject* bodyA = static_cast<const btCollisionObject*>(manifold->getBody0());
                const btCollisionObject* bodyB = static_cast<const btCollisionObject*>(manifold->getBody1());

                std::cout << "bodyA!! " << bodyA;

                if (bodyA) { printf("bodyA IS FOUND\n"); }
                if (bodyB) { printf("bodyB IS FOUND\n"); }

                const btCollisionObject* other = nullptr;
                if (bodyA == obj) {
                    other = bodyB;
                } else if (bodyB == obj) {
                    other = bodyA;
                }

                if (other) {
                    printf("OTHER IS FOUND\n");
                    auto found = gObjectNames.find(other);
                    if (found != gObjectNames.end()) {
                        otherObj = found->second;
                        otherObj.erase(std::remove(otherObj.begin(), otherObj.end(), '\0'), otherObj.end());
                    }
                    break; // stop on first contact found
                }
            }
            */
            /*
                    for (int i = 0; i < objects.size(); ++i) {
            btCollisionObject* obj = objects[i];
            */

            const auto& contacts = getSoftBodyContacts(sb);
            //for (const auto& contact : contacts) {
            for (int i = 0; i < contacts.size(); ++i) {
                btSoftBody::RContact contact = contacts[i];
                const btCollisionObject* otherObj = contact.m_cti.m_colObj;
                std::string otherName = getObjectName(otherObj);


                // 2. Check for event
                DeformationLevel peakLevel = classifyDeformation(record.peakDeformation);
                ContactState contactState = detectContactEvent(sb);

                if (contactState == ContactState::STARTS) {
                    output += "[Event] <" + objName + "> enters contact with <" + otherName + ">.\n";
                }
                else if (contactState == ContactState::ENDS) {
                    //std::cout << "PEAK DEFORMATION: " << record.peakDeformation << "\n";

                    output += "[Event] <" + objName + "> separates from <" + otherName + "> after contact with "
                        + makeDeformationString(peakLevel)
                        + " deformation.\n";

                    record.peakDeformation = 0.0; // reset
                }
                else if (contactState == ContactState::CONTINUES && record.stableCount > 3) { // determine stable count threshold empiricallyS
                    output += "[Event] <" + objName + "> comes to a rest on <" + otherName + "> with "
                        + makeDeformationString(peakLevel)
                        + " deformation.\n";
                }

                // 3. Update deformation peak
                if (record.currentWholeBodyDeformation > record.peakDeformation
                    && record.isContacting) {

                    record.peakDeformation = record.currentWholeBodyDeformation;
                }

                /*if (record.currentWholeBodyDeformation < record.lastWholeBodyDeformation && record.isContacting) {
                    std::cout << "Deformation is decreasing!\n";
                    std::cout << "current deform: " << record.currentWholeBodyDeformation << "\n";
                    std::cout << "last deform: " << record.lastWholeBodyDeformation << "\n";
                }*/

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

/*
std::string b3EventDetector::getObjectName(int bodyUniqueId) const {
    const PhysicsServerCommandProcessorInternalData* data = m_commandProcessor->getInternalData();

    if (!data) {
        return "unknown";
    }

    const UserDataHandleMap& 


    
    // auto it = gObjectNames.find(obj);
    // if (it != gObjectNames.end()) {
    //    return it->second;
    // }
    // return "?";
    
}
*/

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

void b3EventDetector::setDynamicsWorld(btDynamicsWorld* world) {
    m_dynamicsWorld = world;
}

const btSoftBody::tRContactArray& b3EventDetector::getSoftBodyContacts(const btSoftBody* softBody) const {
    return softBody->m_rcontacts;
}