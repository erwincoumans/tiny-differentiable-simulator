#pragma once

// Generates the batched kernels of a world (world_model.hpp) from its
// description (world_spec.hpp) with tds::KernelCodegen, as run time sources
// (kernels_cpu.cpp, kernels_cuda.cu and kernels.json, see kernel_loader.hpp).
// kernels.json describes the layout of the arrays in "metadata": the bodies,
// their joints, actuators, links and contacts.
//
// The generated functions are
//   contact, pgs, integrate  a time step with contacts (WorldSpec contact
//                            "split", the default), see world_model.hpp
//   step                     a whole time step, if the world has no contacts
//                            or "split" is false
//   poses                    state -> world poses of the visual shapes
//   links                    state -> world poses of the bases and links

#include <cctype>
#include <string>
#include <vector>

#include "math/eigen_algebra.hpp"
#include "nlohmann/json.hpp"
#include "utils/kernel_codegen.hpp"
#include "world_model.hpp"

namespace tds {

// the world name as a C++ identifier, for the namespace of the kernels
inline std::string world_kernel_name(const std::string& name) {
  std::string id;
  for (char c : name) {
    id += std::isalnum(static_cast<unsigned char>(c)) ? c : '_';
  }
  if (id.empty() || std::isdigit(static_cast<unsigned char>(id[0]))) {
    id = "w" + id;
  }
  return id;
}

template <typename Algebra>
nlohmann::json world_layout(const WorldModel<Algebra>& model) {
  using nlohmann::json;
  json layout;
  layout["state_dim"] = model.state_dim();
  layout["control_dim"] = model.control_dim();
  layout["velocity_dim"] = model.velocity_dim();
  layout["num_contacts"] = model.num_contacts;
  layout["num_visuals"] = model.num_visuals();
  layout["num_links"] = model.num_links();
  layout["split"] = model.num_contacts > 0 && model.spec.contact.split;
  layout["iterations"] = model.spec.contact.iterations;
  layout["dt"] = model.spec.dt;
  layout["substeps"] = model.spec.substeps;
  layout["default_state"] = model.default_state();
  layout["default_control"] = model.default_control();
  layout["warnings"] = model.warnings;
  json bodies = json::array();
  for (const auto& b : model.bodies) {
    if (b.is_ground) continue;
    json body;
    body["name"] = b.name;
    body["floating"] = b.mb->is_floating();
    body["q_offset"] = b.q_offset;
    body["q_dim"] = b.mb->dof();
    body["qd_offset"] = b.qd_offset;
    body["qd_dim"] = b.mb->dof_qd();
    body["base_link"] = b.base_link;
    body["control_mode"] = b.control_mode;
    body["control_offset"] = b.control_offset;
    body["control_dim"] = b.control_dim();
    body["visual_offset"] = b.visual_offset;
    body["num_visuals"] = b.num_visuals;
    body["link_offset"] = b.link_offset;
    json joints = json::array();
    std::vector<int> actuator_of_link(b.mb->num_links(), -1);
    for (size_t i = 0; i < b.actuated_links.size(); ++i) {
      actuator_of_link[b.actuated_links[i]] = static_cast<int>(i);
    }
    json links = json::array();
    links.push_back(b.base_link);
    for (const auto& link : *b.mb) {
      links.push_back(link.link_name);
      if (link.joint_type == JOINT_FIXED) continue;
      json joint;
      joint["name"] = link.joint_name;
      joint["link"] = link.link_name;
      joint["type"] = link.joint_type == JOINT_SPHERICAL ? "spherical"
                      : (link.joint_type == JOINT_PRISMATIC_X ||
                         link.joint_type == JOINT_PRISMATIC_Y ||
                         link.joint_type == JOINT_PRISMATIC_Z ||
                         link.joint_type == JOINT_PRISMATIC_AXIS)
                          ? "prismatic"
                          : "revolute";
      joint["q"] = b.q_offset + link.q_index;
      joint["qd"] = b.qd_offset + link.qd_index;
      joint["actuator"] = actuator_of_link[link.index];
      joints.push_back(joint);
    }
    body["joints"] = joints;
    body["links"] = links;
    json actuators = json::array();
    for (size_t i = 0; i < b.actuated_links.size(); ++i) {
      const auto& link = b.mb->links()[b.actuated_links[i]];
      json a;
      a["joint"] = link.joint_name;
      a["kp"] = b.default_kp[i];
      a["kd"] = b.default_kd[i];
      a["effort_limit"] = b.effort_limits[i];
      actuators.push_back(a);
    }
    body["actuators"] = actuators;
    bodies.push_back(body);
  }
  layout["bodies"] = bodies;
  json contacts = json::array();
  int first = 0;
  for (const auto& pair : model.pairs) {
    const auto& a = model.bodies[pair.body_a];
    const auto& b = model.bodies[pair.body_b];
    auto link_name = [](const auto& body, int link) {
      return link < 0 ? body.base_link : body.mb->links()[link].link_name;
    };
    json c;
    c["body_a"] = a.name;
    c["link_a"] = a.is_ground ? std::string() : link_name(a, pair.link_a);
    c["body_b"] = b.name;
    c["link_b"] = link_name(b, pair.link_b);
    c["first"] = first;
    c["count"] = pair.num_points;
    c["friction"] = pair.friction;
    contacts.push_back(c);
    first += pair.num_points;
  }
  layout["contacts"] = contacts;
  return layout;
}

// Traces the functions of the world and writes the run time sources of its
// kernels to the directory. Returns the layout of the world.
inline nlohmann::json generate_world_kernels(
    const WorldSpec& spec, const std::string& directory,
    const KernelCodegen::Options& options = KernelCodegen::Options()) {
  using ADScalar = KernelCodegen::ADScalar;
  using ADAlgebra = EigenAlgebraT<ADScalar>;
  using X = std::vector<ADScalar>;
  WorldModel<ADAlgebra> model(spec);
  // the double model computes the values at which the functions are traced
  WorldModel<EigenAlgebra> double_model(spec);
  for (const auto& w : model.warnings) {
    std::cout << "Warning: " << w << std::endl;
  }

  KernelCodegen codegen(world_kernel_name(spec.name));
  codegen.options() = options;
  codegen.add_constant("dt", spec.dt);
  codegen.add_constant("substeps", spec.substeps);
  codegen.add_constant("iterations", spec.contact.iterations);
  codegen.add_constant("num_contacts", model.num_contacts);
  codegen.add_string("world", spec.name);

  const int ns = model.state_dim();
  const int nc = model.control_dim();
  const int nv = model.velocity_dim();
  const int m = model.mlcp_rows();
  const std::vector<double> state = double_model.default_state();
  const std::vector<double> control = double_model.default_control();
  auto concat = [](std::vector<double> a, const std::vector<double>& b) {
    a.insert(a.end(), b.begin(), b.end());
    return a;
  };
  const bool split = model.num_contacts > 0 && spec.contact.split;
  if (split) {
    const int nm = model.mlcp_dim();
    const int nj = model.jacobian_dim();
    const int nf = model.factor_dim();
    std::vector<double> velocity(nv), mlcp(nm), jacobian(nj), factor(nf),
        distance(model.num_contacts), lambda(m, 0.);
    double_model.contact(state.data(), control.data(), velocity.data(),
                         mlcp.data(), jacobian.data(), factor.data(),
                         distance.data());
    codegen.add_function(
        "contact", {{"state", ns}, {"control", nc}},
        {{"velocity", nv},
         {"mlcp", nm},
         {"jacobian", nj},
         {"factor", nf},
         {"contact_distance", model.num_contacts}},
        concat(state, control), [&](const X& x, X& y) {
          model.contact(&x[0], &x[ns], &y[0], &y[nv], &y[nv + nm],
                        &y[nv + nm + nj], &y[nv + nm + nj + nf]);
        });
    codegen.add_loop_function(
        "pgs", {{"mlcp", nm}, {"lambda_init", m}}, {"lambda", m},
        "lambda_init", spec.contact.iterations, concat(mlcp, lambda),
        [&](const X& x, X& y) {
          model.pgs_iteration(&x[0], &x[nm], &y[0]);
        });
    codegen.add_function(
        "integrate",
        {{"state", ns},
         {"velocity", nv},
         {"jacobian", nj},
         {"factor", nf},
         {"lambda", m}},
        {{"next_state", ns}},
        concat(concat(concat(concat(state, velocity), jacobian), factor),
               lambda),
        [&](const X& x, X& y) {
          model.integrate(&x[0], &x[ns], &x[ns + nv], &x[ns + nv + nj],
                          &x[ns + nv + nj + nf], &y[0]);
        });
  } else {
    codegen.add_function("step", {{"state", ns}, {"control", nc}},
                         {{"next_state", ns}}, concat(state, control),
                         [&](const X& x, X& y) {
                           model.step(&x[0], &x[ns], &y[0]);
                         });
  }
  codegen.add_function("poses", {{"state", ns}}, {{"poses", model.poses_dim()}},
                       state, [&](const X& x, X& y) {
                         model.poses(&x[0], &y[0]);
                       });
  codegen.add_function("links", {{"state", ns}}, {{"links", model.links_dim()}},
                       state, [&](const X& x, X& y) {
                         model.links(&x[0], &y[0]);
                       });
  nlohmann::json layout = world_layout(double_model);
  codegen.set_metadata_json(layout.dump());
  codegen.write_runtime_sources(directory);
  return layout;
}

}  // namespace tds
