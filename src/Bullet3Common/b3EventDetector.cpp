#include "b3EventDetector.h"
#include "BulletSoftBody/btSoftBody.h"
#include <sstream>
#include <cmath>
#include <iostream>

b3EventDetector gEventDetector; // define global instance

b3EventDetector::b3EventDetector() {}

void b3EventDetector::resetCurrentWholeBodyDeformation(const btSoftBody* sb) {
    m_bodyDeformationRecord[sb].currentWholeBodyDeformation = 0.0;
}

//ContactState b3EventDetector::checkContactState(const btSoftBody* sb, bool inContactNow) {
//    ContactState contactState;
//    auto& record = m_bodyDeformationRecord[sb];
//
//    if (inContactNow && !record.inContact) { // new contact begins
//        contactState = ContactState::STARTS;
//    }
//    else if (!inContactNow && record.inContact) { // contact ends
//        contactState = ContactState::ENDS;
//    }
//    else {
//        contactState = ContactState::CONTINUES;
//    }
//
//    record.inContact = inContactNow;
//
//    return contactState;
//}

void b3EventDetector::setContacting(const btSoftBody* sb, bool isContacting) {
    m_bodyDeformationRecord[sb].isContacting = isContacting;
}

ContactState b3EventDetector::detectContactEvent(const btSoftBody* sb) {
    ContactState contactState;
    auto& record = m_bodyDeformationRecord[sb];

    //if (isContacting && !record.isContacting) { // new contact begins
    if (record.isContacting && !record.wasContacting) { // new contact begins
        contactState = ContactState::STARTS;
        record.hasLogged = false;

        printf("CONTACT BEGINS!\n");
    }
    //else if (!isContacting && record.isContacting) { // contact ends
    else if (!record.isContacting && record.wasContacting) { // contact ends
        contactState = ContactState::ENDS;
        record.hasLogged = false;

        printf("CONTACT ENDS!\n");
    }
    else {
        contactState = ContactState::CONTINUES;

        //printf("CONTACT CONTINUES!\n");

        // HOW TO UPDATE record.hasLogged HERE??
    }

    record.wasContacting = record.isContacting;

    return contactState;
}

void b3EventDetector::updateDeformationEvent(const btSoftBody* sb, int tetIndex, const btMatrix3x3& F) {
    //m_currentWholeBodyDeformation[sb] += calculateTetDeformation(F);
    m_bodyDeformationRecord[sb].currentWholeBodyDeformation += calculateTetDeformation(F);

    //m_bodyDeformationRecord[sb].isContacting = true; // TODO: placeholder for now, remove once contact detection implemented!
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
    // TODO:
    // - finish event detection logic (stable deformation, contact)
    // - decide on templates
    // - get object names to include in trace

    std::string output;

    // IS THE LOOP NECESSARY??
    // iterates through bodies in database
    for (auto it = m_bodyDeformationRecord.begin(); it != m_bodyDeformationRecord.end(); ++it) {
        const btSoftBody* body = it->first;
        auto& record = m_bodyDeformationRecord[body]; // retrieve this body's record
        btScalar currentWholeBodyDeformation = record.currentWholeBodyDeformation;

        //std::cout << "currentWholeBodyDeformation: " << currentWholeBodyDeformation << "\n";
        //std::cout << "lastWholeBodyDeformation: " << record.lastWholeBodyDeformation << "\n";

        //if (!record.isContacting) continue; // TODO: check contact detection logic

        // detect peak deformation to log event
        if (currentWholeBodyDeformation < record.lastWholeBodyDeformation && !record.hasLogged) {
            btScalar peak = record.lastWholeBodyDeformation;
            DeformationLevel level = classifyDeformation(peak);

            output += "[Event] <object> undergoes "
                + makeDeformationString(level)
                + " deformation.\n";

            record.hasLogged = true;
        }

        record.lastWholeBodyDeformation = currentWholeBodyDeformation;

        // TODO: change logic below to reset after a deformation peak when contact ends!!
        // (not just when deformation returns to 0, since that may not happen)
        // 
        // reset for next event
        if (currentWholeBodyDeformation < 1e-4) {
            record.hasLogged = false;
            record.peakDeformation = 0.0;
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