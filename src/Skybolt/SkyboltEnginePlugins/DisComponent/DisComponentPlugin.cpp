/* Copyright 2012-2020 Matthew Reid
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

#include "DisClient.h"

#include <SkyboltEngine/EngineRoot.h>
#include <SkyboltEngine/Plugin/Plugin.h>

#include <dis6/EntityType.h>

#include <boost/config.hpp>
#include <boost/dll/alias.hpp>
#include <boost/log/trivial.hpp>
#include <assert.h>

namespace skybolt {
	using namespace sim;

	typedef std::shared_ptr<DisClient> DisClientPtr;

	// This belongs on the DIS Gateway entity, root for each remote DIS host
	class DisComponent : public sim::Component
	{
	public:
		DisComponent(const DisClientPtr &client) : mClient(client)
		{
			assert(mClient);
		};

		void updatePreDynamics(sim::TimeReal dt, sim::TimeReal dtWallClock) override
		{
			mClient->update(dtWallClock);
		}
	private:
		DisClientPtr mClient;
	};

	// TODO: Needs to be implemented properly, and perhaps elsewhere. Here to remove test clutter from constructor
	void PopulateDisTemplates(std::vector<std::pair<DIS::EntityType, std::string>> *templateList)
	{
		// TODO: This templates mapping should not be per gateway
		DIS::EntityType unknownEntity;
		templateList->push_back(std::make_pair(unknownEntity, "DisUnknownEntity"));

		// nocheckin Remove this fake data
		DIS::EntityType genericPlatform;
		genericPlatform.setEntityKind(1);
		templateList->push_back(std::make_pair(genericPlatform, "Frigate"));

		DIS::EntityType genericTank;
		genericTank.setEntityKind(1);
		genericTank.setDomain(1);
		genericTank.setCategory(1);
		templateList->push_back(std::make_pair(genericTank, "Shuttle"));

		DIS::EntityType genericAttackHelicopter;
		genericAttackHelicopter.setEntityKind(1);
		genericAttackHelicopter.setDomain(2);
		genericAttackHelicopter.setCategory(20);
		templateList->push_back(std::make_pair(genericAttackHelicopter, "SepecatJaguar"));

		DIS::EntityType specificAttackHelicopter;
		specificAttackHelicopter.setEntityKind(1);
		specificAttackHelicopter.setDomain(2);
		specificAttackHelicopter.setCountry(225);
		specificAttackHelicopter.setCategory(20);
		specificAttackHelicopter.setSubcategory(2);
		specificAttackHelicopter.setSpecific(10);
		templateList->push_back(std::make_pair(specificAttackHelicopter, "UH60"));
	}

	const std::string disComponentName = "dis";

	class DisComponentPlugin : public Plugin {
	public:
		DisComponentPlugin(const PluginConfig& config) :
			mComponentFactoryRegistry(config.simComponentFactoryRegistry)
		{
			EngineRoot *engineRoot = config.engineRoot;

			// Function to take an entity and DIS component JSON and configure the DIS component
			auto factory = std::make_shared<ComponentFactoryFunctionAdapter>([engineRoot](Entity *entity, const ComponentFactoryContext &context, const nlohmann::json &json) {
				// This exists on the DIS Gateway entity, entities controlled by this DIS host will be children of this entity.
				DisClientConfig clientConfig;
				clientConfig.host = json.at("hostAddress").get<std::string>();
				clientConfig.hostPort = json.at("hostPort").get<int>();
				clientConfig.localPort = json.at("localPort").get<int>();

				auto disClient = std::make_shared<DisClient>(clientConfig, engineRoot, entity);

				PopulateDisTemplates(&disClient->mDisTemplates);

				DisEntityTypeCompare entityTypeCmpLt;
				std::sort(disClient->mDisTemplates.begin(), disClient->mDisTemplates.end(),
						  [entityTypeCmpLt](const std::pair<DIS::EntityType, std::string> &a,
											const std::pair<DIS::EntityType, std::string> &b) {
							  return entityTypeCmpLt(a.first, b.first);
						  });

				return std::make_shared<DisComponent>(disClient);
			});

			mComponentFactoryRegistry->insert(std::make_pair(disComponentName, factory));
		}

		~DisComponentPlugin()
		{
			mComponentFactoryRegistry->erase(disComponentName);
		}
	private:
		ComponentFactoryRegistryPtr mComponentFactoryRegistry;
	};

	namespace plugins {
		std::shared_ptr<Plugin> createEnginePlugin(const PluginConfig& config)
		{
			return std::make_shared<DisComponentPlugin>(config);
		}

		BOOST_DLL_ALIAS(plugins::createEnginePlugin, createEnginePlugin)
	}
} // namespace skybolt {
