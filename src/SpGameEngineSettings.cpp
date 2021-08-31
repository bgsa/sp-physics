#include "SpGameEngineSettings.h"

namespace NAMESPACE_PHYSICS
{

	SpGameEngineSettings* SpGameEngineSettingsInstance = nullptr;

	void SpGameEngineSettings::init()
	{
		if (SpGameEngineSettingsInstance == nullptr)
			SpGameEngineSettingsInstance = sp_mem_new(SpGameEngineSettings)();

		sp_char filename[SP_DIRECTORY_MAX_LENGTH];
		gameEngineSettingsFilename(filename);

		if (fileExists(filename))
			SpGameEngineSettingsInstance->load();
		else
		{
			SpGameEngineSettingsInstance->createDefaultSettings();
			SpGameEngineSettingsInstance->save();
		}
	}

	void SpGameEngineSettings::release()
	{
		if (SpGameEngineSettingsInstance != nullptr)
		{
			sp_mem_delete(SpGameEngineSettingsInstance, SpGameEngineSettings);
			SpGameEngineSettingsInstance = nullptr;
		}
	}

}