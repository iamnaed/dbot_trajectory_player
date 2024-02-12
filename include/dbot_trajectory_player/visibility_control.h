#ifndef DBOT_TRAJECTORY_PLAYER_CPP__VISIBILITY_CONTROL_H_
#define DBOT_TRAJECTORY_PLAYER_CPP__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define DBOT_TRAJECTORY_PLAYER_CPP_EXPORT __attribute__ ((dllexport))
    #define DBOT_TRAJECTORY_PLAYER_CPP_IMPORT __attribute__ ((dllimport))
  #else
    #define DBOT_TRAJECTORY_PLAYER_CPP_EXPORT __declspec(dllexport)
    #define DBOT_TRAJECTORY_PLAYER_CPP_IMPORT __declspec(dllimport)
  #endif
  #ifdef DBOT_TRAJECTORY_PLAYER_CPP_BUILDING_DLL
    #define DBOT_TRAJECTORY_PLAYER_CPP_PUBLIC DBOT_TRAJECTORY_PLAYER_CPP_EXPORT
  #else
    #define DBOT_TRAJECTORY_PLAYER_CPP_PUBLIC DBOT_TRAJECTORY_PLAYER_CPP_IMPORT
  #endif
  #define DBOT_TRAJECTORY_PLAYER_CPP_PUBLIC_TYPE DBOT_TRAJECTORY_PLAYER_CPP_PUBLIC
  #define DBOT_TRAJECTORY_PLAYER_CPP_LOCAL
#else
  #define DBOT_TRAJECTORY_PLAYER_CPP_EXPORT __attribute__ ((visibility("default")))
  #define DBOT_TRAJECTORY_PLAYER_CPP_IMPORT
  #if __GNUC__ >= 4
    #define DBOT_TRAJECTORY_PLAYER_CPP_PUBLIC __attribute__ ((visibility("default")))
    #define DBOT_TRAJECTORY_PLAYER_CPP_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define DBOT_TRAJECTORY_PLAYER_CPP_PUBLIC
    #define DBOT_TRAJECTORY_PLAYER_CPP_LOCAL
  #endif
  #define DBOT_TRAJECTORY_PLAYER_CPP_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // DBOT_TRAJECTORY_PLAYER_CPP__VISIBILITY_CONTROL_H_