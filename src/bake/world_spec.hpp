#pragma once

// Description of a world of robots and objects, from which world_model.hpp
// builds a simulation and world_codegen.hpp generates batched kernels. The
// world is read from JSON, such as the "resolved" world that the tds_bake
// Python package (python/tds_bake) writes: all bodies are URDF files with
// absolute paths, and all joint values are given per joint name.
//
// {
//   "name": "g1",
//   "dt": 0.002777,                 time step of the step kernel
//   "substeps": 6,                  steps per control step (for the user)
//   "up_axis": "z",                 "z" or "y": the normal of the ground
//   "solver": "tds",                "tds": the kernels solve the contacts,
//                                   "external": another engine finds and
//                                   solves them (see WorldModel)
//   "gravity": [0, 0, -9.81],
//   "contact": {"friction": 0.75, "restitution": 0, "erp": 0.2,
//               "cfm": 1e-5, "iterations": 50, "split": true},
//   "ground": {"height": 0},        or null for no ground plane
//   "bodies": [{
//     "name": "g1",
//     "urdf": "/path/to/g1.urdf",
//     "floating": true,
//     "position": [0, 0, 0.793], "orientation": [0, 0, 0, 1],  (xyzw)
//     "joint_positions": {"left_knee_joint": 0.3},
//     "control": {"mode": "pd",      "pd", "torque" or "none"
//                 "kp": 1000, "kd": 10,
//                 "gains": {"left_.*_ankle.*": [40, 2]},  regex of joints
//                 "armature": 0.01, "implicit": true,
//                 "effort_limits": {"left_knee_joint": 139}},
//     "collision": {"links": ["left_ankle_roll_link"],  or "all" / "none"
//                   "spheres": [{"link": "left_hand_palm_link",
//                                "radius": 0.02, "position": [0, 0, 0],
//                                "with": ["ball"]}],   optional
//                   "with": "all"}   or a list of body names, "ground"
//   }]
// }

#include <map>
#include <stdexcept>
#include <string>
#include <vector>

#include "nlohmann/json.hpp"

namespace tds {

struct WorldSpec {
  struct ContactSettings {
    double friction{0.75};
    double restitution{0};
    // Baumgarte error reduction and constraint force mixing
    double erp{0.2};
    double cfm{1e-5};
    // projected Gauss-Seidel iterations
    int iterations{50};
    // separate kernels for the contact setup, the solver iterations and the
    // integration, see WorldModel
    bool split{true};
  };

  struct Control {
    // "pd": position targets, kp and kd of each actuated joint are inputs
    // "torque": joint torques are inputs
    // "none": passive joints
    std::string mode{"none"};
    double kp{0};
    double kd{0};
    // per-joint gains (kp, kd), the keys are regular expressions of joint
    // names, the first match is used
    std::vector<std::pair<std::string, std::pair<double, double>>> gains;
    // rotor inertia added to the actuated joints
    double armature{0};
    // implicit PD (stable for stiff gains and large time steps)
    bool implicit{true};
    // maximum torque of a joint, by joint name
    std::map<std::string, double> effort_limits;
  };

  struct CollisionSphere {
    std::string link;  // "" for the base
    double radius{0.01};
    double position[3]{0, 0, 0};
    // the bodies the sphere collides with, if has_with, otherwise the
    // bodies of the body (Collision::with)
    bool has_with{false};
    bool with_all{true};
    std::vector<std::string> with;
  };

  struct Collision {
    // "all", "none" or "list" (links)
    std::string mode{"all"};
    std::vector<std::string> links;
    std::vector<CollisionSphere> spheres;
    // the bodies it collides with ("ground" is the ground plane), all if
    // with_all
    bool with_all{true};
    std::vector<std::string> with;
  };

  struct Body {
    std::string name;
    std::string urdf;
    bool floating{true};
    double position[3]{0, 0, 0};
    double orientation[4]{0, 0, 0, 1};
    std::map<std::string, double> joint_positions;
    Control control;
    Collision collision;
    // friction coefficient of the body, the contact friction is the square
    // root of the product of the two bodies, -1 uses the world friction
    double friction{-1};
  };

  std::string name{"world"};
  // "tds": the kernels solve the contacts of the collision geometry,
  // "external": another engine finds and solves all contacts, and the
  // kernels only compute the dynamics of the bodies
  std::string solver{"tds"};
  double dt{1. / 360.};
  int substeps{1};
  // the normal of the ground plane: 1 (y) or 2 (z)
  int up_axis{2};
  double gravity[3]{0, 0, -9.81};
  ContactSettings contact;
  bool has_ground{true};
  double ground_height{0};
  double ground_friction{-1};
  std::vector<Body> bodies;

  static WorldSpec from_json(const std::string& text) {
    return from_json(nlohmann::json::parse(text));
  }

  static WorldSpec from_json(const nlohmann::json& j) {
    WorldSpec spec;
    spec.name = j.value("name", spec.name);
    spec.dt = j.value("dt", spec.dt);
    spec.substeps = j.value("substeps", spec.substeps);
    spec.solver = j.value("solver", spec.solver);
    if (spec.solver != "tds" && spec.solver != "external") {
      throw std::runtime_error(
          "solver must be \"tds\" or \"external\", not \"" + spec.solver +
          "\".");
    }
    const std::string up_axis = j.value("up_axis", std::string("z"));
    if (up_axis != "y" && up_axis != "z") {
      throw std::runtime_error("up_axis must be \"z\" or \"y\", not \"" +
                               up_axis + "\".");
    }
    spec.up_axis = up_axis == "y" ? 1 : 2;
    read_array(j, "gravity", spec.gravity, 3);
    if (j.contains("contact")) {
      const auto& c = j.at("contact");
      spec.contact.friction = c.value("friction", spec.contact.friction);
      spec.contact.restitution =
          c.value("restitution", spec.contact.restitution);
      spec.contact.erp = c.value("erp", spec.contact.erp);
      spec.contact.cfm = c.value("cfm", spec.contact.cfm);
      spec.contact.iterations = c.value("iterations", spec.contact.iterations);
      spec.contact.split = c.value("split", spec.contact.split);
    }
    if (j.contains("ground")) {
      const auto& g = j.at("ground");
      spec.has_ground = !g.is_null() && !(g.is_boolean() && !g.get<bool>());
      if (g.is_object()) {
        spec.ground_height = g.value("height", 0.);
        spec.ground_friction = g.value("friction", -1.);
      }
    }
    for (const auto& b : j.at("bodies")) {
      Body body;
      body.name = b.at("name").get<std::string>();
      body.urdf = b.at("urdf").get<std::string>();
      body.floating = b.value("floating", body.floating);
      read_array(b, "position", body.position, 3);
      read_array(b, "orientation", body.orientation, 4);
      body.friction = b.value("friction", body.friction);
      if (b.contains("joint_positions")) {
        for (const auto& [name, value] : b.at("joint_positions").items()) {
          body.joint_positions[name] = value.get<double>();
        }
      }
      if (b.contains("control")) {
        const auto& c = b.at("control");
        Control& control = body.control;
        control.mode = c.value("mode", std::string("pd"));
        if (control.mode != "pd" && control.mode != "torque" &&
            control.mode != "none") {
          throw std::runtime_error("Body \"" + body.name +
                                   "\": unknown control mode \"" +
                                   control.mode + "\".");
        }
        control.kp = c.value("kp", control.kp);
        control.kd = c.value("kd", control.kd);
        control.armature = c.value("armature", control.armature);
        control.implicit = c.value("implicit", control.implicit);
        if (c.contains("gains")) {
          for (const auto& [pattern, gains] : c.at("gains").items()) {
            control.gains.push_back(
                {pattern, {gains.at(0).get<double>(), gains.at(1).get<double>()}});
          }
        }
        if (c.contains("effort_limits")) {
          for (const auto& [name, value] : c.at("effort_limits").items()) {
            control.effort_limits[name] = value.get<double>();
          }
        }
      }
      if (b.contains("collision")) {
        const auto& c = b.at("collision");
        Collision& collision = body.collision;
        if (c.contains("links")) {
          const auto& links = c.at("links");
          if (links.is_string()) {
            collision.mode = links.get<std::string>();
            if (collision.mode != "all" && collision.mode != "none") {
              throw std::runtime_error("Body \"" + body.name +
                                       "\": collision links must be \"all\", "
                                       "\"none\" or a list of link names.");
            }
          } else {
            collision.mode = "list";
            collision.links = links.get<std::vector<std::string>>();
          }
        }
        if (c.contains("spheres")) {
          for (const auto& s : c.at("spheres")) {
            CollisionSphere sphere;
            sphere.link = s.value("link", std::string());
            sphere.radius = s.at("radius").get<double>();
            read_array(s, "position", sphere.position, 3);
            if (s.contains("with")) {
              sphere.has_with = true;
              read_with(s.at("with"), sphere.with_all, sphere.with);
            }
            collision.spheres.push_back(sphere);
          }
        }
        if (c.contains("with")) {
          read_with(c.at("with"), collision.with_all, collision.with);
        }
      }
      spec.bodies.push_back(body);
    }
    return spec;
  }

 private:
  static void read_with(const nlohmann::json& with, bool& with_all,
                        std::vector<std::string>& names) {
    if (with.is_string()) {
      with_all = with.get<std::string>() == "all";
      names.clear();
      if (!with_all) names = {with.get<std::string>()};
    } else {
      with_all = false;
      names = with.get<std::vector<std::string>>();
    }
  }

  static void read_array(const nlohmann::json& j, const char* key,
                         double* values, int size) {
    if (!j.contains(key)) return;
    const auto& a = j.at(key);
    if (!a.is_array() || static_cast<int>(a.size()) != size) {
      throw std::runtime_error(std::string("\"") + key + "\" must have " +
                               std::to_string(size) + " numbers.");
    }
    for (int i = 0; i < size; ++i) values[i] = a.at(i).get<double>();
  }
};

}  // namespace tds
