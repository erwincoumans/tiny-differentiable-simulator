
ADD_LIBRARY(CloudSeed 
  FastSin.cpp 
  Exports.cpp 
  AudioLib/Biquad.cpp
  AudioLib/ShaRandom.cpp
  AudioLib/ValueTables.cpp
  AudioLib/Biquad.h
  AudioLib/Hp1.h
  AudioLib/Lp1.h
  AudioLib/MathDefs.h
  AudioLib/ShaRandom.h
  AudioLib/ValueTables.h
  Utils/Sha256.cpp
  Utils/Sha256.h
  )

target_include_directories(CloudSeed PUBLIC . PRIVATE .)


