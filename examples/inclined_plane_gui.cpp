#include <chrono>  // std::chrono::seconds
#include <fstream>
#include <iostream>
#include <thread>  // std::this_thread::sleep_for
#include <unordered_map>
#include <vector>

// #define TDS_USE_LEFT_ASSOCIATIVE_TRANSFORMS

#define USE_RIGID_BODY
// #define USE_FLOATING_BASE
// #define USE_SPHERICAL_JOINT

#include "dynamics/forward_dynamics.hpp"
#include "dynamics/integrator.hpp"
#include "geometry.hpp"
#include "math/eigen_algebra.hpp"
#include "math/tiny/tiny_algebra.hpp"
#include "math/tiny/tiny_float_utils.h"
#include "utils/conversion.hpp"
#include "utils/sdf_to_mesh_converter.hpp"
#include "visualizer/opengl/tiny_opengl3_app.h"
#include "visualizer/opengl/utils/tiny_chrome_trace_util.h"
#include "visualizer/opengl/utils/tiny_logging.h"
#include "visualizer/opengl/utils/tiny_mesh_utils.h"
#include "world.hpp"


const double SIM_DT = 0.005;

template <typename Algebra>
struct Scene {
  using Vector3 = typename Algebra::Vector3;

  std::vector<std::shared_ptr<tds::Geometry<Algebra>>> geoms;
  std::vector<tds::Transform<Algebra>> transforms;
  std::vector<Vector3> colors;

  size_t add_geom(std::shared_ptr<tds::Geometry<Algebra>> geom,
                  const tds::Transform<Algebra>& transform,
                  const Vector3& color) {
    geoms.push_back(geom);
    transforms.push_back(transform);
    colors.push_back(color);
    return geoms.size() - 1;
  }
};

struct Visualizer {
  using Scalar = double;
  using Algebra = tds::EigenAlgebraT<Scalar>;
  using Vector3 = typename Algebra::Vector3;

 private:
  TinyOpenGL3App* app{nullptr};
  DrawGridData grid_data;
  std::map<std::size_t, std::pair<int, int>> geom_ids;

 public:
  int num_cells{50};

  Visualizer(const char* title = "Visualizer") {
    app = new TinyOpenGL3App(title, 1024, 768);
    app->m_renderer->init();
    app->set_up_axis(2);
    app->m_renderer->get_active_camera()->set_camera_distance(4);
    app->m_renderer->get_active_camera()->set_camera_pitch(-30);
    app->m_renderer->get_active_camera()->set_camera_target_position(0, 0, 0);
    app->m_renderer->update_camera(2);
    grid_data.upAxis = 2;
    grid_data.drawAxis = true;
    app->set_up_axis(2);
  }

  virtual ~Visualizer() {}  // delete app; }

  void setup_scene(const Scene<Algebra>& scene) {
    TINY::TinyVector3f pos(0, 0, 0);
    TINY::TinyQuaternionf orn(0, 0, 0, 1);
    TINY::TinyVector3f scaling(1, 1, 1);
    for (std::size_t gi = geom_ids.size(); gi < scene.geoms.size(); ++gi) {
      tds::RenderShape shape =
          tds::convert_sdf_to_mesh<Algebra>(*scene.geoms[gi], num_cells);
      if (shape.num_triangles == 0 || shape.num_vertices == 0) {
        throw std::runtime_error(
            "Visualizer: SDF for geom " + std::to_string(gi) + " (" +
            std::to_string(tds::GeometryTypes(scene.geoms[gi]->get_type())) +
            ") resulted in an empty mesh");
      }
      int shape_id = app->m_renderer->register_shape(
          &shape.vertices[0].x, shape.vertices.size(), &shape.indices[0],
          static_cast<int>(shape.num_triangles * 3));
      TINY::TinyVector3f color;
      color[0] = static_cast<float>(Algebra::to_double(scene.colors[gi][0]));
      color[1] = static_cast<float>(Algebra::to_double(scene.colors[gi][1]));
      color[2] = static_cast<float>(Algebra::to_double(scene.colors[gi][2]));
      int instance_id = app->m_renderer->register_graphics_instance(
          shape_id, pos, orn, color, scaling, 1.f);
      geom_ids[gi] = std::make_pair(shape_id, instance_id);
    }
  }

  void update_transforms(const Scene<Algebra>& scene) {
    using namespace TINY;
    std::size_t count = std::max(scene.geoms.size(), geom_ids.size());
    for (std::size_t gi = 0; gi < count; ++gi) {
      auto [shape_id, instance_id] = geom_ids[gi];
      const auto& geom_tf = scene.transforms[gi];
      TinyVector3f base_pos(static_cast<float>(geom_tf.translation[0]),
                            static_cast<float>(geom_tf.translation[1]),
                            static_cast<float>(geom_tf.translation[2]));
      auto rot = Algebra::matrix_to_quat(geom_tf.rotation);
      TinyQuaternionf base_orn(static_cast<float>(Algebra::quat_x(rot)),
                               static_cast<float>(Algebra::quat_y(rot)),
                               static_cast<float>(Algebra::quat_z(rot)),
                               static_cast<float>(Algebra::quat_w(rot)));
      app->m_renderer->write_single_instance_transform_to_cpu(
          base_pos, base_orn, instance_id);
    }
    app->m_renderer->write_transforms();
  }

  bool requested_exit() const { return app->m_window->requested_exit(); }
  void write_transforms() { app->m_renderer->write_transforms(); }
  void draw_grid() { app->draw_grid(grid_data); }
  void render_scene() { app->m_renderer->render_scene(); }
  void swap_buffer() { app->swap_buffer(); }
  void run_main_loop() {
    write_transforms();
    while (!requested_exit()) {
      draw_grid();
      render_scene();
      swap_buffer();
    }
  }
  void sleep(double seconds) const {
    std::this_thread::sleep_for(std::chrono::duration<double>(seconds));
  }
};

template <typename Scalar>
struct Simulator {
  using Algebra = tds::EigenAlgebraT<Scalar>;
  using Vector3 = typename Algebra::Vector3;
  using Matrix3 = typename Algebra::Matrix3;

  Scalar dt{Algebra::from_double(SIM_DT)};

  Scene<Algebra> scene;

  tds::World<Algebra> world;
#ifdef USE_RIGID_BODY
  tds::RigidBody<Algebra>* ramp{nullptr};
  tds::RigidBody<Algebra>* rb{nullptr};
#else
  tds::MultiBody<Algebra>* ramp{nullptr};
  tds::MultiBody<Algebra>* mb{nullptr};
#endif

  std::shared_ptr<tds::Plane<Algebra>> geom_plane;
  std::shared_ptr<tds::Geometry<Algebra>> geom_box;

  int geom_link_id;

  void create_system() {
    world.default_friction = Algebra::from_double(0.4);
    geom_box = std::make_shared<tds::Box<Algebra>>();
    // geom_box = std::make_shared<tds::Sphere<Algebra>>(0.5);
    // geom_box = std::make_shared<tds::Capsule<Algebra>>(
    //     Algebra::from_double(0.2), Algebra::from_double(1.0));

    geom_plane = std::make_shared<tds::Plane<Algebra>>(
        Vector3(Algebra::from_double(0.4), Algebra::zero(), Algebra::one()));
    // geom_plane = std::make_shared<tds::Plane<Algebra>>(Vector3(0.0,
    // 0.0, 1.0));
    Algebra::print("plane normal:", geom_plane->get_normal());
    Matrix3 rot = Algebra::rotation_x_matrix(35 * Algebra::pi() / 180);
    rot *= Algebra::rotation_y_matrix(25 * Algebra::pi() / 180);
    rot *= Algebra::rotation_z_matrix(-15 * Algebra::pi() / 180);

#ifdef USE_RIGID_BODY
    rb = world.create_rigid_body(Algebra::one(), geom_box.get());
    rb->world_pose().position_[2] = Algebra::from_double(2.5);
    ramp = world.create_rigid_body(Algebra::zero(), geom_plane.get());
#else
    ramp = world.create_multi_body();
    ramp->collision_geometries().push_back(geom_plane.get());
    ramp->collision_transforms().push_back(tds::Transform<Algebra>());

#ifdef USE_FLOATING_BASE
    bool floating_base = true;
    mb = world.create_multi_body();
    mb->set_floating_base(floating_base);
    mb->collision_geometries().push_back(geom_box.get());
    mb->collision_transforms().push_back(tds::Transform<Algebra>());
    mb->base_rbi().mass = Algebra::one();
    mb->initialize();
    mb->set_position(Algebra::unit3_z() * Algebra::from_double(4.0));
    mb->set_orientation(rot);
    geom_link_id = mb->num_links() - 1;
#else
    bool floating_base = false;
    tds::RigidBodyInertia<Algebra> rbi;
    rbi.mass = Algebra::from_double(0.1);
    mb = world.create_multi_body(floating_base);
    mb->attach(tds::Link<Algebra>(tds::JOINT_PRISMATIC_X,
                                  tds::Transform<Algebra>(), rbi));
    mb->attach(tds::Link<Algebra>(tds::JOINT_PRISMATIC_Y,
                                  tds::Transform<Algebra>(), rbi));
    mb->attach(tds::Link<Algebra>(tds::JOINT_PRISMATIC_Z,
                                  tds::Transform<Algebra>(), rbi));
#ifdef USE_SPHERICAL_JOINT
    mb->attach(tds::Link<Algebra>(tds::JOINT_SPHERICAL,
                                  tds::Transform<Algebra>(), rbi));
#else
    mb->attach(tds::Link<Algebra>(tds::JOINT_REVOLUTE_X,
                                  tds::Transform<Algebra>(), rbi));
    mb->attach(tds::Link<Algebra>(tds::JOINT_REVOLUTE_Y,
                                  tds::Transform<Algebra>(), rbi));
    mb->attach(tds::Link<Algebra>(tds::JOINT_REVOLUTE_Z,
                                  tds::Transform<Algebra>(), rbi));
#endif
    geom_link_id = mb->num_links() - 1;
    mb->collision_geometries(geom_link_id).push_back(geom_box.get());
    mb->collision_transforms(geom_link_id).push_back(tds::Transform<Algebra>());
    mb->initialize();
    mb->q(2) = Algebra::from_double(4.0);
#ifdef USE_SPHERICAL_JOINT
    auto rot_quat = Algebra::matrix_to_quat(rot);
    mb->q(3) = Algebra::quat_x(rot_quat);
    mb->q(4) = Algebra::quat_y(rot_quat);
    mb->q(5) = Algebra::quat_z(rot_quat);
    mb->q(6) = Algebra::quat_w(rot_quat);
#else
    mb->q(3) = 45 * Algebra::pi() / 180;
// mb->q(4) = 45 * Algebra::pi() / 180;
#endif
#endif
#endif

    // mb->qd(0) = -9.0;
    // mb->qd(1) = 10.0;
    // mb->qd(2) = 11.0;
  }

  static tds::Transform<Algebra> to_tf(const tds::Pose<Algebra>& pose) {
    return tds::Transform<Algebra>(pose.position_,
                                   Algebra::quat_to_matrix(pose.orientation_));
  }

  Simulator() {
    create_system();

#ifdef USE_RIGID_BODY
    scene.add_geom(geom_plane, to_tf(ramp->world_pose()),

                   Vector3(Algebra::from_double(0.5), Algebra::from_double(0.5),
                           Algebra::from_double(0.5)));
    scene.add_geom(
        geom_box, to_tf(rb->world_pose()),
        Vector3(Algebra::one(), Algebra::from_double(0.5), Algebra::zero()));
#else
    scene.add_geom(geom_plane, ramp->collision_transforms()[0],
                   Vector3(Algebra::from_double(0.5), Algebra::from_double(0.5),
                           Algebra::from_double(0.5)));
    scene.add_geom(
        geom_box, mb->collision_transforms(geom_link_id)[0],
        Vector3(Algebra::one(), Algebra::from_double(0.5), Algebra::zero()));
#endif
  }

  void step() {
#ifdef USE_RIGID_BODY
    world.step(dt);
    scene.transforms[1] = to_tf(rb->world_pose());
#else
    tds::forward_dynamics(*mb, world.get_gravity());
    // mb->print_state(false);
    tds::integrate_euler_qdd(*mb, dt);
    world.step(dt);
    tds::integrate_euler(*mb, dt);
#ifdef TDS_USE_LEFT_ASSOCIATIVE_TRANSFORMS
    scene.transforms[1] = mb->collision_transforms(geom_link_id)[0] *
                          mb->get_world_transform(geom_link_id);
#else
    scene.transforms[1] = mb->get_world_transform(geom_link_id) *
                          mb->collision_transforms(geom_link_id)[0];
#endif
#endif
  }
};

int main(int argc, char** argv) {
  Simulator<double> sim;
  Visualizer viz;
  viz.num_cells = 30;
  viz.setup_scene(sim.scene);
  viz.update_transforms(sim.scene);
  viz.write_transforms();
  while (!viz.requested_exit()) {
    sim.step();
    viz.update_transforms(sim.scene);
    viz.draw_grid();
    viz.render_scene();
    viz.swap_buffer();
    viz.sleep(SIM_DT);
  }
  return 0;
}
