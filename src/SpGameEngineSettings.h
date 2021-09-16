#ifndef SP_GAME_ENGINE_SETTINGS_HEADER
#define SP_GAME_ENGINE_SETTINGS_HEADER

#include "SpectrumPhysics.h"
#include "FileSystem.h"
#include "nlohmann/json.hpp"

#define SP_FILENAME_GAME_ENGINE_SETTINGS "settings.sps"
#define SP_LAST_PROJECTS_MAX (10)

namespace NAMESPACE_PHYSICS
{

	inline void gameEngineSettingsFilename(sp_char* filename)
	{
		sp_char currentDir[SP_DIRECTORY_MAX_LENGTH];
		currentDirectory(currentDir);

		directoryAddPath(currentDir, std::strlen(currentDir),
			SP_FILENAME_GAME_ENGINE_SETTINGS, std::strlen(SP_FILENAME_GAME_ENGINE_SETTINGS),
			filename);
	}

	class SpGameEngineSettings
	{
	private:
		sp_int _latestProjectsLength;
		sp_char _latestProjects[SP_LAST_PROJECTS_MAX][SP_DIRECTORY_MAX_LENGTH];

		inline void createDefaultSettings()
		{
		}

	public:

		/// <summary>
		/// Default constructor
		/// </summary>
		/// <returns></returns>
		API_INTERFACE inline SpGameEngineSettings()
		{
			_latestProjectsLength = 0;
			std::memset(_latestProjects, 0, sizeof(sp_char) * SP_LAST_PROJECTS_MAX * SP_DIRECTORY_MAX_LENGTH);
		}

		/// <summary>
		/// Get the length of last projects loaded
		/// </summary>
		/// <returns></returns>
		API_INTERFACE inline sp_int lastestProjectsLength() const
		{
			return _latestProjectsLength;
		}

		/// <summary>
		/// Get the lastest projects loaded
		/// </summary>
		/// <returns></returns>
		API_INTERFACE inline sp_char* lastestProjects() const
		{
			return (sp_char*)&_latestProjects[0];
		}

		/// <summary>
		/// Set the last loaded project
		/// </summary>
		/// <param name="filename">Full filename</param>
		/// <returns></returns>
		API_INTERFACE inline void addLastProject(const sp_char* filename, const sp_size filenameLength)
		{
			sp_assert(filenameLength < SP_DIRECTORY_MAX_LENGTH, "FilenameLengthExceeded");

			// copy all projects to end
			std::memcpy(_latestProjects[1], _latestProjects, sizeof(sp_char) * 9 * SP_DIRECTORY_MAX_LENGTH);

			// set the last project at first
			std::memcpy(_latestProjects[0], filename, filenameLength);
			_latestProjects[0][filenameLength] = END_OF_STRING;

			if (_latestProjectsLength < SP_LAST_PROJECTS_MAX)
				_latestProjectsLength++;
		}

		/// <summary>
		/// Save the game engine settings
		/// </summary>
		/// <returns></returns>
		API_INTERFACE inline void save()
		{
			nlohmann::ordered_json json;
			json["version"] = "1.0.0";

			nlohmann::json lastProjectsAsJson = nlohmann::json::array();
			for (sp_int i = 0; i < _latestProjectsLength; i++)
			{
				nlohmann::json json = nlohmann::json
				{
					{ "name", _latestProjects[i] }
				};

				lastProjectsAsJson.push_back(json);
			}
			json["last-projects"] = lastProjectsAsJson;

			std::string jsonAsString = json.dump(4);

			sp_char filename[SP_DIRECTORY_MAX_LENGTH];
			gameEngineSettingsFilename(filename);

			writeTextFile(filename, jsonAsString.c_str(), jsonAsString.length());
		}

		/// <summary>
		/// Load the game engine settings from disk
		/// </summary>
		/// <returns></returns>
		API_INTERFACE inline void load()
		{
			sp_char filename[SP_DIRECTORY_MAX_LENGTH];
			gameEngineSettingsFilename(filename);

			const sp_size size = fileSize(filename);

			if (size == 0)
				return;

			sp_char* content = (sp_char*)ALLOC_SIZE(size);

			readTextFile(filename, content, size);
			
			nlohmann::json json = nlohmann::json::parse(content);

			std::string version;

			if (json.find("version") != json.end())
				version = json["version"].get<std::string>();

			if (json.find("last-projects") != json.end() && json["last-projects"].is_array())
			{
				_latestProjectsLength = (sp_int)json["last-projects"].size();

				for (sp_int i = 0; i < _latestProjectsLength; i++)
				{
					const std::string name = json["last-projects"][i]["name"].get<std::string>();
					std::memcpy(_latestProjects[i], name.c_str(), name.length());
				}
			}
		}

		/// <summary>
		/// Initialize the game engine settings
		/// </summary>
		/// <returns></returns>
		API_INTERFACE static void init();

		/// <summary>
		/// Release the game engine settings
		/// </summary>
		/// <returns></returns>
		API_INTERFACE static void release();

	};

	extern SpGameEngineSettings* SpGameEngineSettingsInstance;

}

#endif // SP_GAME_ENGINE_SETTINGS_HEADER