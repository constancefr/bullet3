#include "b3EventDetector.h"
#include "BulletSoftBody/btSoftBody.h"
#include <sstream>
#include <cmath>

b3EventDetector::b3EventDetector() {}

DeformationLevel b3EventDetector::classifyDeformation(const btMatrix3x3& F) const {
    /* 
    C: Right Cauchy-Green tensor from deformation gradient, 
    captures total deformation and removes rigid body rotation.
    E: Green-Lagrange strain tensor: non-linear strain from undeformed state.
    */
    btMatrix3x3 C = F.transposeTimes(F);
    btMatrix3x3 I; I.setIdentity();
    btMatrix3x3 E = (C - I) * 0.5;

    // Frobenius norm of E (magnitude)
    btScalar eNormSquared =
        E[0].dot(E[0]) +
        E[1].dot(E[1]) +
        E[2].dot(E[2]);

    btScalar eNorm = std::sqrt(eNormSquared);

    // TODO: tune buckets empirically
    if (eNorm < 0.02) return DeformationLevel::VERY_LOW;
    if (eNorm < 0.05) return DeformationLevel::LOW;
    if (eNorm < 0.12) return DeformationLevel::MEDIUM;
    if (eNorm < 0.25) return DeformationLevel::HIGH;
    return DeformationLevel::VERY_HIGH;
}

std::string b3EventDetector::makeDeformationString(DeformationLevel level) const {
    switch (level) {
    case DeformationLevel::VERY_LOW: return "very low";
    case DeformationLevel::LOW: return "low";
    case DeformationLevel::MEDIUM: return "medium";
    case DeformationLevel::HIGH: return "high";
    case DeformationLevel::VERY_HIGH: return "very high";
    default: return "unknown";
    }
}

void b3EventDetector::updateDeformationEvent(const btSoftBody* sb, int tetIndex, const btMatrix3x3& F) {
    TetKey key{sb, tetIndex};
    DeformationLevel current = classifyDeformation(F);

    auto& record = m_deformationHistory[key];
    if (record.lastLevel == current) { // or if change is very small?
        record.stableCount++;
    } else {
        record.stableCount = 0;
    }

    record.lastF = F;
    record.lastLevel = current;
}

std::string b3EventDetector::checkForEvent() {
    for (const auto& pair : m_deformationHistory) {
        const auto& record = pair.second;

        // TODO:
        // - event detection logic
        // - decide on templates
        // - get object names

        // deformation is high and stable
        if (record.stableCount >= 3 && record.lastLevel >= DeformationLevel::HIGH) {
            std::ostringstream oss;
            oss << "<object> stabilises with "
                << makeDeformationString(record.lastLevel)
                << " deformation.";
            return oss.str();
        }
    }
    return "";
}
