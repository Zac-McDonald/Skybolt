/* Copyright 2012-2020 Matthew Reid
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

#pragma once

#include "UdpCommunicator.h"
#include <SkyboltEngine/EngineRoot.h>
#include <SkyboltSim/Entity.h>
#include <SkyboltSim/SimMath.h>

#include <dis6/EntityID.h>
#include <dis6/EntityType.h>
#include <utils/IPacketProcessor.h>
#include <utils/IncomingMessage.h>

#include <map>
#include <memory>

namespace skybolt {
	struct DisClientConfig
	{
		std::string host = "localhost";
		int hostPort = 8001;
		int localPort = 8002;
	};

	struct DisEntityIDCompare
	{
		bool operator()(const DIS::EntityID &a, const DIS::EntityID &b) const {
			return ((a.getSite()        < b.getSite()) ||
					(a.getApplication() < b.getApplication()) ||
					(a.getEntity()      < b.getEntity()));
		}
	};

	struct DisEntityTypeCompare
	{
		bool operator()(const DIS::EntityType &a, const DIS::EntityType &b) const {
			return ((a.getEntityKind()  < b.getEntityKind()) ||
					(a.getDomain()      < b.getDomain()) ||
					(a.getCountry()     < b.getCountry()) ||
					(a.getCategory()    < b.getCategory()) ||
					(a.getSubcategory() < b.getSubcategory()) ||
					(a.getSpecific()    < b.getSpecific()) ||
					(a.getExtra()       < b.getExtra()));
		}
	};

	enum DeadReckoningModel
	{
		Other = 0,
		Static = 1,
		// Rotation:      Fixed    or Rotating
		// Constant rate: Position or Velocity
		// Coordinates:   World    or Body
		// For algorithms, see: https://github.com/open-dis/dis-tutorial/wiki/Dead-Reckoning
		FPW = 2,
		RPW = 3,
		RVW = 4,
		FVW = 5,
		FPB = 6,
		RPB = 7,
		RVB = 8,
		FVB = 9
	};

	struct DisEntity
	{
		sim::EntityPtr simEntity;
		bool updatedThisFrame;
		// sim::TimeReal lastUpdated; // TODO: See note in .cpp

		DeadReckoningModel deadReckoning;
		// NOTE: These are all 32-bit in DIS
		sim::Vector3 linearVelocity;
		sim::Vector3 linearAcceleration;
		sim::Vector3 angularVelocity; // radians/sec about entity local axes (YawPitchRoll)
	};

	// NOTE: Actual interaction with DIS protocol and remote host
	class DisClient : public DIS::IPacketProcessor
	{
	public:
		DisClient(const DisClientConfig &config, EngineRoot *engineRoot, sim::Entity *disGateway);
		~DisClient();

		void update(sim::TimeReal dt);
		void Process(const DIS::Pdu &p); // DIS::IPacketProcessor


		// TODO: Find a better home for entity type mapping (multiple gateways will need access)
		// Maybe try on the Plugin itself?
		std::vector<std::pair<DIS::EntityType, std::string>> mDisTemplates;
		std::string DisClient::TemplateLookup(DIS::EntityType *type, DIS::EntityType *altType);
	private:
		std::unique_ptr<UdpCommunicator> mSocket;
		DIS::IncomingMessage mIncoming;
		char mBuffer[1500+1];

		EngineRoot *mEngineRoot;
		sim::Entity *mDisGateway;

		std::map<DIS::EntityID, DisEntity, DisEntityIDCompare> mEntities;
	};
} // namespace skybolt
