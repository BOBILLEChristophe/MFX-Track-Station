/*

   BuildInfo.h

   Date and time of the build, printed at start-up. BUILD_DATE is defined in
   src/BuildInfo.cpp, written by tools/build_info.py before each build, so
   that it changes at every build (not only when main.cpp is compiled).

*/

#pragma once

extern const char BUILD_DATE[];
