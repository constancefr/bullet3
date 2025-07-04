#ifndef B3_EVENT_DETECTOR_H
#define B3_EVENT_DETECTOR_H

#include "LinearMath/btMatrix3x3.h" // for deformation matrix F
#include "LinearMath/btScalar.h"
#include "BulletCollision/CollisionDispatch/btCollisionWorld.h" // to access members of btCollisionWorld
#include <string>
#include <unordered_map> // store key-value pairs
#include <utility>

class btSoftBody;
class btCollisionObject;
class PhysicsServerCommandProcessor;
class btDeformableMultiBodyDynamicsWorld;

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
	ENDS,
	NO_CONTACT
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

	void updateDeformationEvent(const btSoftBody* sb, int tetIndex, const btMatrix3x3& F); // call in btSoftBody::updateDeformation()

	void resetCurrentWholeBodyDeformation(const btSoftBody* sb);

	void setContacting(const btSoftBody* sb, bool isContacting);

	ContactState detectContactEvent(const btSoftBody* sb);

	btScalar checkForEvent();

	std::unordered_map<const btCollisionObject*, std::string> gObjectNames;

	void setObjectName(const btCollisionObject* obj, const std::string& name);

	std::string getObjectName(const btCollisionObject* obj) const;

	void setCommandProcessor(PhysicsServerCommandProcessor* proc);

	void setDynamicsWorld(btDeformableMultiBodyDynamicsWorld* world);

	btDeformableMultiBodyDynamicsWorld* m_dynamicsWorld;


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
		bool isDeformable = false;
		btScalar currentWholeBodyDeformation = 0.0;
		btScalar lastWholeBodyDeformation = 0.0;
		btScalar peakDeformation = 0.0;
		bool isContacting = false;
		bool wasContacting = false;
		int stableCount = 0;
	};

	// main object deformation database
	std::unordered_map<const btSoftBody*, ObjectRecord> m_bodyDeformationRecord;
	//std::unordered_map<const btCollisionObject*, ObjectRecord> m_objectRecords;

	btScalar calculateTetDeformation(const btMatrix3x3& F) const;

	DeformationLevel classifyDeformation(btScalar eNorm) const;

	std::string makeDeformationString(DeformationLevel level) const;

	PhysicsServerCommandProcessor* m_commandProcessor = nullptr;

};

extern b3EventDetector gEventDetector; // declare global object


#endif