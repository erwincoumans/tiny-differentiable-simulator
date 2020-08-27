
ADD_LIBRARY(TinyAudio 
	b3ADSR.cpp
	b3ADSR.h
	b3AudioListener.cpp
	b3AudioListener.h
	RtAudio.cpp
	b3ReadWavFile.cpp
	b3ReadWavFile.h
#	b3SoundEngine.cpp
#	b3SoundEngine.h
	b3SoundSource.cpp
	b3SoundSource.h
	b3WriteWavFile.cpp
	b3WriteWavFile.h
	)

IF(WIN32)
    	target_compile_definitions(TinyAudio 
	    PUBLIC __STK_REALTIME__ WIN32 __WINDOWS_MM__ __WINDOWS_DS__
	    PRIVATE __STK_REALTIME__ WIN32 __WINDOWS_MM__ __WINDOWS_DS__
	    )
	    target_link_libraries(TinyAudio PUBLIC winmm dsound PRIVATE winmm dsound)
ELSE()
IF(APPLE)
	
ELSE()
  #linux
	#links {"asound","pthread"}
	target_compile_definitions(TinyAudio 
	    PUBLIC __STK_REALTIME__ __OS_LINUX__ __LINUX_ALSA__
	    PRIVATE __STK_REALTIME__ __OS_LINUX__ __LINUX_ALSA__
	    )
	    target_link_libraries(TinyAudio PUBLIC asound pthread PRIVATE asound pthread)
	    
ENDIF()

ENDIF()

target_include_directories(TinyAudio PUBLIC . PRIVATE .)


