	project "CloudSeed"

	language "C++"
				
	kind "StaticLib"
		
	includedirs {"."}

	files {
		"*.cpp",
		"*.h",
		"AudioLib/*.cpp",
		"AudioLib/*.h",
		"Utils/*.cpp",
		"Utils/*.h",
	}

