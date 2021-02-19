#ifndef RBDL_URDFREADER_H
#define RBDL_URDFREADER_H

#include <rbdl/rbdl_config.h>

namespace RigidBodyDynamics {

struct Model;

namespace Addons {
  RBDL_DLLAPI bool URDFReadFromFile (const char* filename, Model* model, bool floating_base, bool verbose = false);
  RBDL_DLLAPI bool URDFReadFromString (const char* model_xml_string, Model* model, bool floating_base, bool verbose = false);
}

}

/* _RBDL_URDFREADER_H */
#endif
