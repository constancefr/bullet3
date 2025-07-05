#include "b3EventDetector.h"
#include "BulletSoftBody/btSoftBody.h"
#include <sstream>
#include <cmath>
#include <iostream>

b3EventDetector gEventDetector; // define global instance

b3EventDetector::b3EventDetector() {}

void b3EventDetector::resetCurrentWholeBodyDeformation(const btSoftBody* sb) {
    m_bodyDeformationMap[sb] = 0.0;
}

void b3EventDetector::updateDeformationEvent(const btSoftBody* sb, const btMatrix3x3& F) {
    m_bodyDeformationMap[sb] += calculateTetDeformation(F);
}

float b3EventDetector::calculateTetDeformation(const btMatrix3x3& F) const {
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

    //std::cout << "eNorm: " << static_cast<float>(eNorm) << "\n";

    return static_cast<float>(eNorm);
}

float b3EventDetector::returnDeformation(btSoftBody* sb) {
    float wholeBodyDeformation = m_bodyDeformationMap.at(sb);

    return wholeBodyDeformation;
}