#ifndef B3_EVENT_DETECTOR_H
#define B3_EVENT_DETECTOR_H

#include "LinearMath/btMatrix3x3.h" // for deformation matrix F
#include <string>
#include <unordered_map> // store key-value pairs
#include <utility>

class btSoftBody;
class btCollisionObject;
class PhysicsServerCommandProcessor;

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

	bool firstStep = true;

	void setContacting(const btSoftBody* sb, bool isContacting);

	ContactState detectContactEvent(const btSoftBody* sb);

	// call once per tet in btSoftBody::updateDeformation()
	void updateDeformationEvent(const btSoftBody* sb, int tetIndex, const btMatrix3x3& F);

	// returns a NL description if event detected
	std::string checkForEvent();

	void resetCurrentWholeBodyDeformation(const btSoftBody* sb);

	void setCommandProcessor(PhysicsServerCommandProcessor* proc);

	std::unordered_map<const btCollisionObject*, std::string> gObjectNames;

	//std::string getObjectName(int bodyUniqueId) const;

	void setObjectName(const btCollisionObject* obj, const std::string& name);

	//const PhysicsServerCommandProcessor* m_physicsServer;



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

	struct ObjectRecord {
		btScalar currentWholeBodyDeformation = 0.0;
		btScalar lastWholeBodyDeformation = 0.0;
		btScalar peakDeformation = 0.0;
		bool isContacting = false;
		bool wasContacting = false;
		bool stableCount = 0;
	};

	// main object deformation database
	std::unordered_map<const btSoftBody*, ObjectRecord> m_bodyDeformationRecord;

	btScalar calculateTetDeformation(const btMatrix3x3& F) const;

	DeformationLevel classifyDeformation(btScalar eNorm) const;

	std::string makeDeformationString(DeformationLevel level) const;

	PhysicsServerCommandProcessor* m_commandProcessor = nullptr;

};


extern b3EventDetector gEventDetector; // declare global object


#endif