/* Copyright 2012-2020 Matthew Reid
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

#include "DisClient.h"

#include <SkyboltSim/SimMath.h>
#include <SkyboltSim/Components/ProceduralLifetimeComponent.h>
#include <SkyboltSim/Components/ParentReferenceComponent.h>
#include <SkyboltSim/Spatial/Geocentric.h>
#include <SkyboltSim/Spatial/GreatCircle.h>
#include <SkyboltSim/Spatial/Orientation.h>
#include <SkyboltSim/World.h>

#include <utils/PDUType.h>
#include <dis6/EntityStatePDU.h>

#include <boost/log/trivial.hpp>

/* TODO for first-pass implementation:
 * Goals
 * - Support EntityStatePDU used to update remote entities transforms
 * - For entity identification, have some mapping of DIS identity to Skybolt entity
 * - Articulation
 * - Entity appearance
 *
 * - Other PDU types
 * - Remote entity interface (later, maybe)
 */

namespace skybolt {
	DisClient::DisClient(const DisClientConfig &config, EngineRoot *engineRoot, sim::Entity *disGateway) :
		mEngineRoot(engineRoot), mDisGateway(disGateway)
	{
		UdpCommunicatorConfig socketConfig;
		socketConfig.localAddress  = "localhost";
		socketConfig.localPort     = config.localPort;
		socketConfig.remoteAddress = config.host;
		socketConfig.remotePort    = config.hostPort;
		mSocket = std::make_unique<UdpCommunicator>(socketConfig);

	    mBuffer[1500] = '\0';

		// TODO: Move processors to a mapping, not neccessarily as we can switch on PDU type?
		mIncoming.AddProcessor(DIS::PDUType::PDU_ENTITY_STATE, this);
		// mIncoming.AddProcessor(DIS::PDUType::PDU_DETONATION, this);
	}

	DisClient::~DisClient()
	{
		mIncoming.RemoveProcessor(DIS::PDUType::PDU_ENTITY_STATE, this);
	}

	std::string DisClient::TemplateLookup(DIS::EntityType *type, DIS::EntityType *altType = nullptr)
	{
		DIS::EntityType typeCopy;
		typeCopy.setEntityKind(  type->getEntityKind()  );
		typeCopy.setDomain(      type->getDomain()      );
		typeCopy.setCountry(     type->getCountry()     );
		typeCopy.setCategory(    type->getCategory()    );
		typeCopy.setSubcategory( type->getSubcategory() );
		typeCopy.setSpecific(    type->getSpecific()    );
		typeCopy.setExtra(       type->getExtra()       );
		type = &typeCopy;

		// TODO: Copy code for alt-type, if we end up using it and it exists

		// Binary search to find closest matching entity
		size_t low = 0;
		size_t high = mDisTemplates.size() - 1;

		// Broadening progression:
		//   Search exact (specific)
		//   Search subcategory
		//   Search category
		//   Try variation?
		//   Search domain
		//   Search kind
		//   Unknown

		DisEntityTypeCompare cmpLt;
		int retryIteration = 0;
		bool keepRetrying = true;
		while (keepRetrying) {
			while (low <= high) {
				size_t mid = (low + high) / 2;
				auto templateEntry = mDisTemplates[mid];

				if (cmpLt(templateEntry.first, *type)) {
					low = mid + 1;
				} else if (cmpLt(*type, templateEntry.first)) {
					high = mid - 1;
				} else {
					return templateEntry.second;
				}
			}

			switch (retryIteration++) {
			case 0: {
				type->setExtra(0);
			} break;
			case 1: {
				type->setSpecific(0);
			} break;
			case 2: {
				type->setSubcategory(0);
			} break;
			case 3: {
				// TODO: Try variation instead
				// TODO: Decide on order at this point -
				//       Is a generic tank acceptable? or must it be a generic "country" tank?
				type->setCountry(0);
			} break;
			case 4: {
				type->setCategory(0);
			} break;
			case 5: {
				type->setDomain(0);
			} break;
			case 6: {
				type->setEntityKind(0);
			} break;
			default: {
				keepRetrying = false;
			}
			}

			// NOTE: Because the array is sorted, we know the next target won't be after high
			//       If we change resolution order too much, this may not be true.
			low = 0;
		}

		return mDisTemplates[0].second;
	}

	void DisClient::update(sim::TimeReal dt)
	{
		size_t bytesRead = mSocket->receive(*((unsigned char *)mBuffer), 1500);

		// NOTE: DIS Packets can be packed (OpenDIS example does this), so multiple PDUs may be read.
		if (bytesRead)
		{
			// BOOST_LOG_TRIVIAL(info) << "DIS read " << bytesRead << " bytes";

			// TODO: Audit simulators we use that communicate with DIS, do they all use big endian?
			mIncoming.Process((char *)mBuffer, bytesRead, DIS::BIG);
		}

		// Update entities
		for (auto it = mEntities.begin(); it != mEntities.end(); ++it)
		{
			DisEntity *entity = &(*it).second;

			// TODO: Entity heartbeat timeouts and (soft) delete if dead

			if (!entity->updatedThisFrame)
			{
				auto geoPosOpt = sim::getPosition(*entity->simEntity);

				if (entity->deadReckoning != DeadReckoningModel::Static && geoPosOpt) {
					// NOTE: Will we need to store last PDU position?
					// Yes if we want additional smoothing (to hide stutters from overshooting),
					// but dead reckoning algos should be equivalent on timestep deltas (assure this is True)
					sim::Vector3 geoPosition = *geoPosOpt;

					// NOTE: Using fallthroughs to reuse position formulas
					switch (entity->deadReckoning) {
					case DeadReckoningModel::RPW: {
						// TODO: Angular velocity
					}
					case DeadReckoningModel::FPW: {
						geoPosition.x += entity->linearVelocity.x * dt;
						geoPosition.y += entity->linearVelocity.y * dt;
						geoPosition.z += entity->linearVelocity.z * dt;
					} break;
					case DeadReckoningModel::RVW: {
						// TODO: Angular velocity
					}
					case DeadReckoningModel::FVW: {
						sim::TimeReal halfDtSqr = dt * dt * 0.5;
						geoPosition.x += entity->linearVelocity.x * dt + (entity->linearAcceleration.x * halfDtSqr);
						geoPosition.y += entity->linearVelocity.y * dt + (entity->linearAcceleration.y * halfDtSqr);
						geoPosition.z += entity->linearVelocity.z * dt + (entity->linearAcceleration.z * halfDtSqr);
					} break;
						// TODO: Body centered algorithms
					}

					sim::setPosition(*entity->simEntity, geoPosition);
				}
			}
			entity->updatedThisFrame = false;
		}
	}

	void DisClient::Process(const DIS::Pdu &p)
	{
		const DIS::EntityStatePdu &espdu = static_cast<const DIS::EntityStatePdu&>(p);

		// Check header information
		// TODO: Check exercise (and add DIS params to config)
		// TODO: Does OpenDIS handle PDU time (out of order + reliability)


		// Do we have this entity already?
		// No:
		//   Extract type
		//   Lookup entity or use fallback
		//   Add attachment to gateway
		// Yes: continue (later: update visuals)
		DisEntity *entity;
		DIS::EntityID entityId = espdu.getEntityID();
		auto it = mEntities.find(entityId);
		if (it == mEntities.end()) {
			// Make Skybolt entity
			DIS::EntityType entityType = espdu.getEntityType();
			std::string templateName = TemplateLookup(&entityType);

			sim::EntityPtr simEntity = mEngineRoot->entityFactory->createEntity(templateName);
			simEntity->addComponent(std::make_shared<sim::ParentReferenceComponent>(mDisGateway));
			simEntity->addComponent(std::make_shared<sim::ProceduralLifetimeComponent>());
			simEntity->setDynamicsEnabled(false);

			mEngineRoot->simWorld->addEntity(simEntity);

			DisEntity disEntity = {};
			disEntity.simEntity = simEntity;
			auto entry = mEntities.insert(std::make_pair(entityId, disEntity));
			entity = &entry.first->second;
		} else {
			entity = &it->second;
		}

		if (entity) {
			entity->updatedThisFrame = true;
			// TODO: Heartbeats (10 second, default, or by type. Remove entity after 3 missed beats)
			//       Need a way of getting wallclock time from Skybolt?

			// Dead reckoning
			DIS::Vector3Float disLinearVelocity = espdu.getEntityLinearVelocity();
			entity->linearVelocity.x = disLinearVelocity.getX();
			entity->linearVelocity.y = disLinearVelocity.getY();
			entity->linearVelocity.z = disLinearVelocity.getZ();

			DIS::DeadReckoningParameter deadReckoningParams = espdu.getDeadReckoningParameters();
			entity->deadReckoning = (DeadReckoningModel)deadReckoningParams.getDeadReckoningAlgorithm();
			// MISSING: "Other" parameters - for custom/extension algorithms
			DIS::Vector3Float disLinearAcceleration = deadReckoningParams.getEntityLinearAcceleration();
			entity->linearAcceleration.x = disLinearAcceleration.getX();
			entity->linearAcceleration.y = disLinearAcceleration.getY();
			entity->linearAcceleration.z = disLinearAcceleration.getZ();
			DIS::Vector3Float disAngularVelocity = deadReckoningParams.getEntityAngularVelocity();
			entity->angularVelocity.x = disAngularVelocity.getX();
			entity->angularVelocity.y = disAngularVelocity.getY();
			entity->angularVelocity.z = disAngularVelocity.getZ();

			// Get position, no convert needed and set
			DIS::Vector3Double disPosition = espdu.getEntityLocation();
			sim::Vector3 geoPosition(disPosition.getX(), disPosition.getY(), disPosition.getZ());
			sim::setPosition(*entity->simEntity, geoPosition);

			// Get orientation convert and set
			DIS::Orientation disOrientation = espdu.getEntityOrientation();
			sim::Vector3 orientationEuler(disOrientation.getPsi(),
										  disOrientation.getTheta(),
										  disOrientation.getPhi());
			sim::LtpNedOrientation orientation(math::quatFromEuler(orientationEuler));
			sim::Quaternion orientationQuat = sim::toGeocentric(orientation, sim::geocentricToLatLon(geoPosition)).orientation;
			sim::setOrientation(*entity->simEntity, orientationQuat);
		}

		/*
		BOOST_LOG_TRIVIAL(info) << "DIS PDU received: " <<
			(int)p.getPduType() << " (type) " <<
			p.getTimestamp()    << " (timestamp)";
		*/
	}
} // namespace skybolt
