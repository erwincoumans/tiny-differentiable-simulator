#include <chrono>
#include <thread>
#include "geometry.hpp"
#include "world.hpp"
#include "visualizer/opengl/tiny_opengl3_app.h"
#include "opengl_urdf_visualizer.h"
#include "dynamics/forward_dynamics.hpp"
#include "dynamics/integrator.hpp"


#include <iostream>

using namespace tds;
#include "math/tiny/tiny_algebra.hpp"
using namespace TINY;


#ifdef USE_TINY
#include "math/tiny/tiny_double_utils.h"
#else
#include "math/eigen_algebra.hpp"
#endif



int main(int argc, char* argv[]) {
#ifdef USE_TINY
  std::cout << "Using TinyAlgebra" << std::endl;
  typedef TinyAlgebra<double, DoubleUtils> Algebra;
#else
  std::cout << "Using EigenAlgebra" << std::endl;
  typedef EigenAlgebra Algebra;
#endif
  typedef Algebra::Scalar Scalar;
  typedef Geometry<Algebra> TinyGeometry;


  double radius = 0.5;
	
  tds::World<Algebra> world;

  

  const TinyGeometry* white_geom = world.create_sphere(radius);
  
  auto mb = world.create_multi_body("sphere");
  bool is_floating= true;
  double sphere_mass= 1;
  double sphere_inertia = 0.4*sphere_mass*radius*radius;;

  mb->base_rbi().mass = sphere_mass;
  mb->base_rbi().com = Algebra::Vector3(0,0,0);
  for (int i=0;i<3;i++)
	  for (int j=0;j<3;j++)
		  mb->base_rbi().inertia(i,j)=0.;
  mb->base_rbi().inertia(0,0)=sphere_inertia;
  mb->base_rbi().inertia(1,1)=sphere_inertia;
  mb->base_rbi().inertia(2,2)=sphere_inertia;

  mb->set_floating_base(is_floating);
  mb->collision_geometries().push_back(white_geom);
  auto tr = tds::Transform<Algebra>();
  tr.set_identity();
  mb->collision_transforms().push_back(tr);
  mb->initialize();
  mb->set_position(Algebra::Vector3(0,0,3));

  OpenGLUrdfVisualizer<Algebra> visualizer(1024,768,"soft contact example");

  //ground plane
  int plane_shape = visualizer.m_opengl_app.register_cube_shape(10,10,0.1);
  int plane_id = visualizer.m_opengl_app.m_instancingRenderer->register_graphics_instance(plane_shape,TinyVector3f(0,0,-0.1), TinyQuaternionf(0,0,0,1),TinyVector3f(0,0,1),TinyVector3f(1,1,1));
  
  const TinyGeometry* plane_geom = world.create_plane();
  auto plane_mb = world.create_multi_body("plane");
  plane_mb->initialize();
  plane_mb->collision_geometries().push_back(plane_geom);
  plane_mb->collision_transforms().push_back(tr);

  int sphere_shape = visualizer.m_opengl_app.register_graphics_unit_sphere_shape(SPHERE_LOD_HIGH);
  TinyVector3f pos(0,0,0);
  TinyQuaternionf orn(0,0,0,1);
  TinyVector3f color(1,0,0);
  TinyVector3f scaling(1,1,1);
  int sphere_id = visualizer.m_opengl_app.m_instancingRenderer->register_graphics_instance(sphere_shape,pos, orn,color,scaling);
  
  double t = 0;
  double dt = 1./240.;

  /////////////
  Scalar combined_contact_stiffness = 0.5;//depends on object masses, try 0.1 to 10000
  Scalar combined_contact_damping = .05;//try 10% of stiffness

  Scalar denom = (dt * combined_contact_stiffness + combined_contact_damping);
  denom = Algebra::max(denom, 1e-6);
  
  Scalar cfm = Scalar(1) / denom;
  Scalar erp = (dt * combined_contact_stiffness) / denom;

  world.get_mb_constraint_solver()->cfm_ = cfm;
  world.get_mb_constraint_solver()->erp_ = erp;
  /////////////


  while (!visualizer.m_opengl_app.m_window->requested_exit())
  {
	  tds::forward_dynamics(*mb, world.get_gravity());
      
	  mb->clear_forces();

      integrate_euler_qdd(*mb, dt);

      world.step(dt);

      tds::integrate_euler(*mb, dt);

	  pos = TinyVector3f(mb->base_X_world().translation.x(),mb->base_X_world().translation.y(),
		  mb->base_X_world().translation.z());
	  visualizer.m_opengl_app.m_instancingRenderer->write_single_instance_transform_to_cpu(pos, orn, sphere_id);
	  visualizer.m_opengl_app.m_instancingRenderer->write_transforms();
	  visualizer.render();
	  
      std::this_thread::sleep_for(std::chrono::duration<double>(dt));

  }
}