#ifndef B3_EVENT_DETECTOR_H
#define B3_EVENT_DETECTOR_H

#include "LinearMath/btMatrix3x3.h" // for deformation matrix F
#include <string>
#include <unordered_map> // store key-value pairs
#include <utility>

class btSoftBody;

enum class DeformationLevel {
	VERY_LOW,
	LOW,
	MEDIUM,
	HIGH,
	VERY_HIGH
};

class b3EventDetector {
/*
Track deformation of soft body tetrahedra
Detect events based on deformation
Return a NL sequence upon event
*/
public:
	b3EventDetector();

	// call once per tet in btSoftBody::updateDeformation()
	void updateDeformationEvent(const btSoftBody* sb, int tetIndex, const btMatrix3x3& F);
	
	// returns a NL description if event detected
	std::string checkForEvent();

private:
	struct TetKey {
		const btSoftBody* body;
		int index; // tet index within body

		bool operator=(const TetKey& other) const { // to check whether two keys refer to same tet
			return body == other.body && index == other.index;
		}
	};

	struct TetKeyHash { // allows use of TetKey as a key in the unordered_map
		std::size_t operator()(const TetKey& key) const {
			return std::hash<const void*>()(key.body) ^ std::hash<int>()(key.index);
		}
	};

	struct DeformationRecord {
		btMatrix3x3 lastF;
		DeformationLevel lastLevel;
		int stableCount = 0; // how many steps in a row has deformation stayed at the same level
	};

	// main database
	std::unordered_map<TetKey, DeformationRecord, TetKeyHash> m_deformationHistory;

	DeformationLevel classifyDeformation(const btMatrix3x3& F) const;

	std::string makeDeformationString(DeformationLevel level) const;
};


#endif