/*
 * RBDL - Rigid Body Dynamics Library
 * Copyright (c) 2011-2018 Martin Felis <martin@fysx.org>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */

#ifndef RBDL_LOGGING_H
#define RBDL_LOGGING_H

#include <sstream>
#include <rbdl/rbdl_config.h>

class LoggingGuard;

/** \def RBDL_ENABLE_LOGGING
 *
 * Enables/Disables logging
 *
 * \warning Logging has a huge impact on performance.
 */
#ifndef RBDL_ENABLE_LOGGING
#define LOG if (false) LogOutput 
#define SUPPRESS_LOGGING ;
#else
#define LOG LogOutput
#define SUPPRESS_LOGGING LoggingGuard _nolog
#endif

extern RBDL_DLLAPI std::ostringstream LogOutput;
RBDL_DLLAPI void ClearLogOutput ();

/** \brief Helper object to ignore any logs that happen during its lifetime
 *
 * If an instance of this class exists all logging gets suppressed. This
 * allows to disable logging for a certain scope or a single function call,
 * e.g.
 *
 * \code
 * {
 *   // logging will be active
 *   do_some_stuff();
 *  
 *   // now create a new scope in which a LoggingGuard instance exists
 *   {
 *     LoggingGuard ignore_logging;
 *    
 *     // as a _Nologging instance exists, all logging will be discarded
 *     do_some_crazy_stuff();
 *   }
 *
 *   // logging will be active again
 *   do_some_more_stuff();
 * }
 * \endcode
 *
 */
class RBDL_DLLAPI LoggingGuard {
  public:
    LoggingGuard() {
      log_backup.str("");
      log_backup << LogOutput.str();
    }
    ~LoggingGuard() {
      LogOutput.str("");
      LogOutput << log_backup.str();
    }

  private:
    std::ostringstream log_backup;
};

/* RBDL_LOGGING_H */
#endif
