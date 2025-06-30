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
/*
Track deformation of soft body tetrahedra
Detect events based on deformation
Return a NL sequence upon event
*/
public:
	b3EventDetector();

	//std::unordered_map<const btSoftBody*, bool> gContactFlagsThisStep;
	void setContacting(const btSoftBody* sb, bool isContacting);

	//ContactState checkContactState(const btSoftBody* sb, bool inContact);
	ContactState detectContactEvent(const btSoftBody* sb);

	// call once per tet in btSoftBody::updateDeformation()
	void updateDeformationEvent(const btSoftBody* sb, int tetIndex, const btMatrix3x3& F);

	// returns a NL description if event detected
	std::string checkForEvent();

	void resetCurrentWholeBodyDeformation(const btSoftBody* sb);

private:
	struct TetKey {
		const btSoftBody* body;
		int index; // tet index within body

		bool operator==(const TetKey& other) const { // to check whether two keys refer to same tet
			return body == other.body && index == other.index;
		}
	};

	struct TetKeyHash { // allows use of TetKey as a key in the unordered_map
		std::size_t operator()(const TetKey& key) const {
			return std::hash<const void*>()(key.body) ^ std::hash<int>()(key.index);
		}
	};

	// Can probably delete this:
	//struct TetDeformationRecord {
	//	btMatrix3x3 lastF;
	//	DeformationLevel lastLevel;
	//	int stableCount = 0; // how many steps in a row has deformation stayed at the same level
	//};
	// main tet database
	//std::unordered_map<TetKey, TetDeformationRecord, TetKeyHash> m_tetDeformationRecord;

	struct ObjectDeformationRecord {
		btScalar currentWholeBodyDeformation = 0.0;
		btScalar lastWholeBodyDeformation = 0.0;
		btScalar peakDeformation = 0.0;
		bool isContacting = false; // TODO: add contact detection logic
		bool wasContacting = false;
		bool hasLogged = false;
		bool stableCount = 0; // TODO: implement stabilising detection
	};

	/*struct ObjectContactRecord {
		bool 
	};*/

	// main object deformation database
	std::unordered_map<const btSoftBody*, ObjectDeformationRecord> m_bodyDeformationRecord;

	//std::unordered_map<const btSoftBody*, ObjectContactRecord> m_bodyContactRecord;

	btScalar calculateTetDeformation(const btMatrix3x3& F) const;

	DeformationLevel classifyDeformation(btScalar eNorm) const;

	std::string makeDeformationString(DeformationLevel level) const;
};


extern b3EventDetector gEventDetector; // declare global object


#endif