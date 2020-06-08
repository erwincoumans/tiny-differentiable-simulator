
#ifndef TINY_LOGGING_H
#define TINY_LOGGING_H

#ifdef __cplusplus
extern "C" {
#endif

/// We add the do/while so that the statement "if (condition)
/// TinyPrintf("test"); else {...}" would fail You can also customize the
/// message by uncommenting out a different line below
#define TinyPrintf(...) TinyOutputPrintfVarArgsInternal(__VA_ARGS__)
//#define TinyPrintf(...) do
//{TinyOutputPrintfVarArgsInternal("TinyPrintf[%s,%d]:",__FILE__,__LINE__);TinyOutputPrintfVarArgsInternal(__VA_ARGS__);
//} while(0) #define TinyPrintf TinyOutputPrintfVarArgsInternal #define
// TinyPrintf(...) printf(__VA_ARGS__) #define TinyPrintf(...)
#define TinyWarning(...)                                                       \
  do {                                                                         \
    TinyOutputWarningMessageVarArgsInternal("TinyWarning[%s,%d]:\n", __FILE__, \
                                            __LINE__);                         \
    TinyOutputWarningMessageVarArgsInternal(__VA_ARGS__);                      \
  } while (0)
#define TinyError(...)                                                     \
  do {                                                                     \
    TinyOutputErrorMessageVarArgsInternal("TinyError[%s,%d]:\n", __FILE__, \
                                          __LINE__);                       \
    TinyOutputErrorMessageVarArgsInternal(__VA_ARGS__);                    \
  } while (0)
#ifndef B3_NO_PROFILE

void TinyEnterProfileZone(const char* name);
void TinyLeaveProfileZone();
#ifdef __cplusplus

class TinyProfileZone {
 public:
  TinyProfileZone(const char* name) { TinyEnterProfileZone(name); }

  ~TinyProfileZone() { TinyLeaveProfileZone(); }
};

#define B3_PROFILE(name) TinyProfileZone __profile(name)
#endif

#else  // B3_NO_PROFILE

#define B3_PROFILE(name)
#define TinyStartProfile(a)
#define TinyStopProfile

#endif  //#ifndef B3_NO_PROFILE

typedef void(TinyPrintfFunc)(const char* msg);
typedef void(TinyWarningMessageFunc)(const char* msg);
typedef void(TinyErrorMessageFunc)(const char* msg);
typedef void(TinyEnterProfileZoneFunc)(const char* msg);
typedef void(TinyLeaveProfileZoneFunc)();

/// The developer can route TinyPrintf output using their own implementation
void TinySetCustomPrintfFunc(TinyPrintfFunc* printfFunc);
void TinySetCustomWarningMessageFunc(TinyWarningMessageFunc* warningMsgFunc);
void TinySetCustomErrorMessageFunc(TinyErrorMessageFunc* errorMsgFunc);

/// Set custom profile zone functions (zones can be nested)
void TinySetCustomEnterProfileZoneFunc(TinyEnterProfileZoneFunc* enterFunc);
void TinySetCustomLeaveProfileZoneFunc(TinyLeaveProfileZoneFunc* leaveFunc);

/// Don't use those internal functions directly, use the TinyPrintf or
/// TinySetCustomPrintfFunc instead (or warning/error version)
void TinyOutputPrintfVarArgsInternal(const char* str, ...);
void TinyOutputWarningMessageVarArgsInternal(const char* str, ...);
void TinyOutputErrorMessageVarArgsInternal(const char* str, ...);

#ifdef __cplusplus
}
#endif

#endif  // TINY_LOGGING_H