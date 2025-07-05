#ifndef B3_EVENT_DETECTOR_H
#define B3_EVENT_DETECTOR_H

#include "LinearMath/btMatrix3x3.h" // for deformation matrix F
#include <string>
#include <unordered_map> // store key-value pairs
#include <utility>

class btSoftBody;

enum class DeformationLevel {
	NO,
	VERY_LOW,
	LOW,
	MEDIUM,
	HIGH,
	VERY_HIGH
};

enum class ContactState {
	STARTS,
	CONTINUES,
	ENDS
};

class b3EventDetector {
public:
	b3EventDetector();

	void resetCurrentWholeBodyDeformation(const btSoftBody* sb);
	void updateDeformationEvent(const btSoftBody* sb, const btMatrix3x3& F); // called in btSoftBody::updateDeformation()
	float returnDeformation(btSoftBody* sb);

private:
	std::unordered_map<const btSoftBody*, float> m_bodyDeformationMap;

	float calculateTetDeformation(const btMatrix3x3& F) const;
};


extern b3EventDetector gEventDetector; // declare global object


#endif