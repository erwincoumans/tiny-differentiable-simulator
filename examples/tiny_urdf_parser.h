/*
 * Copyright 2020 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef TINY_URDF_PARSER_H
#define TINY_URDF_PARSER_H

#include <tinyxml2.h>

#include <algorithm>
#include <fstream>
#include <iostream>
#include <iterator>
#include <map>
#include <numeric>
#include <sstream>  // std::istringstream
#include <streambuf>
#include <string>
#include <vector>

#include "tiny_urdf_structures.h"

struct TinyLogger {
  virtual ~TinyLogger() {}
  virtual void report_error(const std::string& txt) = 0;
  virtual void report_warning(const std::string& txt) = 0;
  virtual void print_message(const std::string& msg) = 0;
};

struct StdLogger : public TinyLogger {
  virtual void report_error(const std::string& txt) {
    std::cout << std::string("Error:") << txt << std::endl;
  }
  virtual void report_warning(const std::string& txt) {
    std::cout << std::string("Warning:") << txt << std::endl;
  }
  virtual void print_message(const std::string& txt) {
    std::cout << txt << std::endl;
  }
};

template <typename TinyScalar, typename TinyConstants>
struct TinyUrdfParser {
  typedef ::TinyUrdfStructures<TinyScalar, TinyConstants> TinyUrdfStructures;
  typedef ::TinyUrdfLink<TinyScalar, TinyConstants> TinyUrdfLink;
  typedef ::TinyUrdfJoint<TinyScalar, TinyConstants> TinyUrdfJoint;
  typedef ::TinyUrdfInertial<TinyScalar, TinyConstants> TinyUrdfInertial;
  typedef ::TinyVector3<TinyScalar, TinyConstants> TinyVector3;
  typedef ::TinyUrdfVisual<TinyScalar, TinyConstants> TinyUrdfVisual;
  typedef ::TinyUrdfCollision<TinyScalar, TinyConstants> TinyUrdfCollision;
  typedef ::TinyUrdfGeometry<TinyScalar, TinyConstants> TinyUrdfGeometry;

  static bool parse_vector3(TinyVector3& vec3, const std::string& vector_str,
                            TinyLogger& logger) {
    vec3.set_zero();
    std::vector<TinyScalar> scalars;
    std::istringstream iss(vector_str);
    std::vector<std::string> pieces((std::istream_iterator<std::string>(iss)),
                                    std::istream_iterator<std::string>());

    for (int i = 0; i < pieces.size(); ++i) {
      if (!pieces[i].empty()) {
        scalars.push_back(TinyConstants::scalar_from_string(pieces[i].c_str()));
      }
    }
    if (scalars.size() < 3) {
      logger.report_error("Couldn't parse vector3, need at least 3 values.");
      return false;
    }
    vec3.setValue(scalars[0], scalars[1], scalars[2]);
    return true;
  }

  static bool parse_transform(TinyVector3& xyz, TinyVector3& rpy,
                              const tinyxml2::XMLElement* xml,
                              TinyLogger& logger) {
    xyz.set_zero();
    rpy.set_zero();
    bool result = true;

    const char* xyz_str = xml->Attribute("xyz");
    if (xyz_str) {
      result = result && parse_vector3(xyz, std::string(xyz_str), logger);
    }

    const char* rpy_str = xml->Attribute("rpy");
    if (rpy_str != NULL) {
      result = result && parse_vector3(rpy, std::string(rpy_str), logger);
    }
    return result;
  }

  static bool parse_inertia(TinyUrdfInertial& urdf_inertial,
                            const tinyxml2::XMLElement* config,
                            TinyLogger& logger) {
    urdf_inertial.origin_xyz.set_zero();
    urdf_inertial.origin_rpy.set_zero();
    urdf_inertial.mass = TinyConstants::zero();
    // Origin
    const tinyxml2::XMLElement* o = config->FirstChildElement("origin");
    if (o) {
      if (!parse_transform(urdf_inertial.origin_xyz, urdf_inertial.origin_rpy,
                           o, logger)) {
        return false;
      }
    }

    const tinyxml2::XMLElement* mass_xml = config->FirstChildElement("mass");
    if (!mass_xml) {
      logger.report_error("Inertial element must have a mass element");
      return false;
    }
    if (!mass_xml->Attribute("value")) {
      logger.report_error("Inertial: mass element must have value attribute");
      return false;
    }
    urdf_inertial.mass =
        TinyConstants::scalar_from_string(mass_xml->Attribute("value"));

    const tinyxml2::XMLElement* inertia_xml =
        config->FirstChildElement("inertia");
    if (!inertia_xml) {
      logger.report_error("Inertial element must have inertia element");
      return false;
    }

    if ((inertia_xml->Attribute("ixx") && inertia_xml->Attribute("iyy") &&
         inertia_xml->Attribute("izz"))) {
      urdf_inertial.inertia_xxyyzz.m_x =
          TinyConstants::scalar_from_string(inertia_xml->Attribute("ixx"));
      urdf_inertial.inertia_xxyyzz.m_y =
          TinyConstants::scalar_from_string(inertia_xml->Attribute("iyy"));
      urdf_inertial.inertia_xxyyzz.m_z =
          TinyConstants::scalar_from_string(inertia_xml->Attribute("izz"));
    } else {
      logger.report_error(
          "Inertial: inertia element must have ixx,iyy,izz attributes");
      return false;
    }

    return true;
  }

  static bool parse_geometry(TinyUrdfGeometry& geom,
                             const tinyxml2::XMLElement* vis_xml,
                             TinyLogger& logger) {
    if (vis_xml == 0) return false;

    const tinyxml2::XMLElement* shape = vis_xml->FirstChildElement();
    if (!shape) {
      logger.report_error("Geometry tag contains no child element.");
      return false;
    }

    // const std::string type_name = shape->ValueTStr().c_str();
    const std::string type_name = shape->Value();
    if (type_name == "sphere") {
      geom.geom_type = TINY_SPHERE_TYPE;

      if (!shape->Attribute("radius")) {
        logger.report_error("Sphere shape must have a radius attribute");
        return false;
      } else {
        geom.m_sphere.m_radius =
            TinyConstants::scalar_from_string(shape->Attribute("radius"));
      }
    } else if (type_name == "box") {
      geom.geom_type = TINY_BOX_TYPE;

      if (!shape->Attribute("size")) {
        logger.report_error("box requires a size attribute");
        return false;
      } else {
        parse_vector3(geom.m_box.m_extents, shape->Attribute("size"), logger);
      }
    } else if (type_name == "cylinder") {
      geom.geom_type = TINY_CAPSULE_TYPE;
      if (!shape->Attribute("length") || !shape->Attribute("radius")) {
        logger.report_error(
            "Cylinder shape must have both length and radius attributes");
        return false;
      }
      geom.m_capsule.m_radius =
          TinyConstants::scalar_from_string(shape->Attribute("radius"));
      geom.m_capsule.m_length =
          TinyConstants::scalar_from_string(shape->Attribute("length"));
    } else if (type_name == "capsule") {
      geom.geom_type = TINY_CAPSULE_TYPE;
      if (!shape->Attribute("length") || !shape->Attribute("radius")) {
        logger.report_error(
            "Capsule shape must have both length and radius attributes");
        return false;
      }
      geom.m_capsule.m_radius =
          TinyConstants::scalar_from_string(shape->Attribute("radius"));
      geom.m_capsule.m_length =
          TinyConstants::scalar_from_string(shape->Attribute("length"));
    } else if ((type_name == "mesh") || (type_name == "cdf")) {
      geom.geom_type = TINY_MESH_TYPE;

      geom.m_mesh.m_scale.setValue(TinyConstants::one(), TinyConstants::one(),
                                   TinyConstants::one());
      std::string fn;

      // URDF
      if (shape->Attribute("filename")) {
        fn = shape->Attribute("filename");
      }
      if (shape->Attribute("scale")) {
        if (!parse_vector3(geom.m_mesh.m_scale, shape->Attribute("scale"),
                           logger)) {
          logger.report_warning(
              "Scale should be a vector3, not single scalar. Workaround "
              "activated.\n");
          std::string scalar_str = shape->Attribute("scale");
          TinyScalar scale_factor =
              TinyConstants::scalar_from_string(scalar_str.c_str());
          geom.m_mesh.m_scale.setValue(scale_factor, scale_factor,
                                       scale_factor);
        }
      }

      if (fn.empty()) {
        logger.report_error("Mesh filename is empty");
        return false;
      }

      geom.m_mesh.m_file_name = fn;
    } else {
      if (type_name == "plane") {
        geom.geom_type = TINY_PLANE_TYPE;

        if (!shape->Attribute("normal")) {
          logger.report_error("plane requires a normal attribute");
          return false;
        } else {
          parse_vector3(geom.m_plane.m_normal, shape->Attribute("normal"),
                        logger);
        }
      } else {
        logger.report_error("Unknown geometry type:");
        logger.report_error(type_name);
        return false;
      }
    }

    return true;
  }

  static bool parse_material(TinyUrdfStructures& urdf_structures,
                             TinyUrdfVisual& visual,
                             const tinyxml2::XMLElement* vis_xml,
                             TinyLogger& logger) {
    if (!vis_xml->Attribute("name")) {
      logger.report_error("Material must contain a name attribute");
      return false;
    }
    TinyVisualMaterial<TinyScalar, TinyConstants> material;
    std::string material_m_name = vis_xml->Attribute("name");

    // texture
    const tinyxml2::XMLElement* t = vis_xml->FirstChildElement("texture");
    if (t) {
      if (t->Attribute("filename")) {
        material.texture_filename = t->Attribute("filename");
      }
    }

    // color (todo: alpha )
    {
      const tinyxml2::XMLElement* c = vis_xml->FirstChildElement("color");
      if (c) {
        if (c->Attribute("rgba")) {
          if (!parse_vector3(material.material_rgb, c->Attribute("rgba"),
                             logger)) {
            std::string msg = material_m_name + " has no rgba";
            logger.report_warning(msg);
          }
        }
      }
    }

    urdf_structures.m_materials[material_m_name] = material;
    return true;
  }

  static bool parse_collision(TinyUrdfCollision& collision,
                              const tinyxml2::XMLElement* config,
                              TinyLogger& logger) {
    collision.origin_xyz.set_zero();
    collision.origin_rpy.set_zero();

    // Origin
    const tinyxml2::XMLElement* o = config->FirstChildElement("origin");
    if (o) {
      if (!parse_transform(collision.origin_xyz, collision.origin_rpy, o,
                           logger))
        return false;
    }
    // Geometry
    const tinyxml2::XMLElement* geom = config->FirstChildElement("geometry");
    if (!parse_geometry(collision.geometry, geom, logger)) {
      return false;
    }

    {
      const char* group_char = config->Attribute("group");
      if (group_char) {
        collision.collision_group =
            TinyConstants::scalar_from_string(group_char);
      }
    }

    {
      const char* mask_char = config->Attribute("mask");
      if (mask_char) {
        collision.collision_mask = TinyConstants::scalar_from_string(mask_char);
      }
    }

    const char* name_char = config->Attribute("name");
    if (name_char) collision.collision_name = name_char;

    // const char* concave_char = config->Attribute("concave");
    // if (concave_char)
    //	collision.flags |= URDF_FORCE_CONCAVE_TRIMESH;

    return true;
  }

  static bool parse_visual(TinyUrdfStructures& urdf_structures,
                           TinyUrdfVisual& visual,
                           const tinyxml2::XMLElement* vis_xml,
                           TinyLogger& logger) {
    visual.origin_xyz.set_zero();
    visual.origin_rpy.set_zero();
    // Origin
    const tinyxml2::XMLElement* o = vis_xml->FirstChildElement("origin");
    if (o) {
      if (!parse_transform(visual.origin_xyz, visual.origin_rpy, o, logger)) {
        return false;
      }
    }
    // Geometry
    const tinyxml2::XMLElement* geom = vis_xml->FirstChildElement("geometry");
    if (!parse_geometry(visual.geometry, geom, logger)) {
      return false;
    }

    const char* name_char = vis_xml->Attribute("name");
    if (name_char) visual.visual_name = name_char;

    visual.has_local_material = false;

    // Material
    const tinyxml2::XMLElement* mat = vis_xml->FirstChildElement("material");
    // todo(erwincoumans) skip materials in SDF for now (due to complexity)
    if (mat) {
      // get material name
      if (!mat->Attribute("name")) {
        logger.report_error("Visual material must contain a name attribute");
        return false;
      }
      visual.material_name = mat->Attribute("name");

      // try to parse material element in place

      const tinyxml2::XMLElement* t = mat->FirstChildElement("texture");
      const tinyxml2::XMLElement* c = mat->FirstChildElement("color");
      const tinyxml2::XMLElement* s = mat->FirstChildElement("specular");
      if (t || c || s) {
        parse_material(urdf_structures, visual, mat, logger);
      }
    }
    return true;
  }

  static bool parse_link(TinyUrdfLink& link,
                         TinyUrdfStructures& urdf_structures,
                         const tinyxml2::XMLElement* config,
                         TinyLogger& logger) {
    const char* linkName = config->Attribute("name");
    if (!linkName) {
      logger.report_error("Link with no name");
      return false;
    }
    link.link_name = linkName;

    // optional 'contact' parameters
    const tinyxml2::XMLElement* ci = config->FirstChildElement("contact");
    if (ci) {
      const tinyxml2::XMLElement* friction_xml =
          ci->FirstChildElement("lateral_friction");
      if (friction_xml) {
        if (!friction_xml->Attribute("value")) {
          logger.report_error(
              "Link/contact: lateral_friction element must have value "
              "attribute");
          return false;
        }
        link.contact_info.lateral_friction =
            TinyConstants::scalar_from_string(friction_xml->Attribute("value"));
      }
      {
        const tinyxml2::XMLElement* restitution_xml =
            ci->FirstChildElement("restitution");
        if (restitution_xml) {
          if (!restitution_xml->Attribute("value")) {
            logger.report_error(
                "Link/contact: restitution element must have value attribute");
            logger.report_error(linkName);
            return false;
          }
          link.contact_info.restitution = TinyConstants::scalar_from_string(
              restitution_xml->Attribute("value"));
        }
      }
      {
        const tinyxml2::XMLElement* stiffness_xml =
            ci->FirstChildElement("stiffness");
        if (stiffness_xml) {
          if (!stiffness_xml->Attribute("value")) {
            logger.report_error(
                "Link/contact: stiffness element must have value attribute");
            logger.report_error(linkName);
            return false;
          }
          link.contact_info.stiffness = TinyConstants::scalar_from_string(
              stiffness_xml->Attribute("value"));
        }
      }
      {
        const tinyxml2::XMLElement* damping_xml =
            ci->FirstChildElement("damping");
        if (damping_xml) {
          if (!damping_xml->Attribute("value")) {
            logger.report_error(
                "Link/contact: damping element must have value attribute");
            logger.report_error(linkName);
            return false;
          }
          link.contact_info.damping = TinyConstants::scalar_from_string(
              damping_xml->Attribute("value"));
        }
      }
    }

    // Inertial (optional)
    const tinyxml2::XMLElement* i = config->FirstChildElement("inertial");
    if (i) {
      if (!parse_inertia(link.urdf_inertial, i, logger)) {
        logger.report_error("Could not parse inertial element for Link:");
        logger.report_error(link.link_name);
        return false;
      }
    }

    // Multiple Visuals (optional)
    for (const tinyxml2::XMLElement* vis_xml =
             config->FirstChildElement("visual");
         vis_xml; vis_xml = vis_xml->NextSiblingElement("visual")) {
      TinyUrdfVisual visual;

      if (parse_visual(urdf_structures, visual, vis_xml, logger)) {
        link.urdf_visual_shapes.push_back(visual);
      } else {
        logger.report_error("Could not parse visual element for Link:");
        logger.report_error(link.link_name);
        return false;
      }
    }

    // Multiple Collisions (optional)
    for (const tinyxml2::XMLElement* col_xml =
             config->FirstChildElement("collision");
         col_xml; col_xml = col_xml->NextSiblingElement("collision")) {
      TinyUrdfCollision col;

      if (parse_collision(col, col_xml, logger)) {
        link.urdf_collision_shapes.push_back(col);
      } else {
        logger.report_error("Could not parse collision element for Link:");
        logger.report_error(link.link_name);
        return false;
      }
    }

    return true;
  }

  static bool parse_joint(TinyUrdfStructures& urdf_structures,
                          TinyUrdfJoint& joint, tinyxml2::XMLElement* config,
                          TinyLogger& logger) {
    // Get Joint Name
    const char* name = config->Attribute("name");
    if (!name) {
      logger.report_error("unnamed joint found");
      return false;
    }
    joint.joint_name = name;
    const tinyxml2::XMLElement* o = config->FirstChildElement("origin");
    if (o) {
      if (!parse_transform(joint.joint_origin_xyz, joint.joint_origin_rpy, o,
                           logger)) {
        logger.report_error("Malformed parent origin element for joint:");
        logger.report_error(joint.joint_name);
        return false;
      }
    }

    // Get Parent Link
    tinyxml2::XMLElement* parent_xml = config->FirstChildElement("parent");
    if (parent_xml) {
      const char* pname = parent_xml->Attribute("link");
      if (!pname) {
        logger.report_error(
            "no parent link name specified for Joint link. this might be the "
            "root?");
        logger.report_error(joint.joint_name);
        return false;
      } else {
        joint.parent_name = std::string(pname);
      }
    }

    // Get Child Link
    tinyxml2::XMLElement* child_xml = config->FirstChildElement("child");
    if (child_xml) {
      const char* pname = child_xml->Attribute("link");
      if (!pname) {
        logger.report_error(
            "no child link name specified for Joint link [%s].");
        logger.report_error(joint.joint_name);
        return false;
      } else {
        joint.child_name = std::string(pname);
      }
    }

    // Get Joint type
    const char* type_char = config->Attribute("type");
    if (!type_char) {
      logger.report_error(
          "joint [%s] has no type, check to see if it's a reference.");
      logger.report_error(joint.joint_name);
      return false;
    }

    std::string type_str = type_char;
    if (type_str == "spherical") {
      joint.joint_type = JOINT_INVALID;
      logger.report_error("spherical joints not supported");
      return false;
    } else if (type_str == "planar") {
      joint.joint_type = JOINT_INVALID;
      logger.report_error("planar joints not supported");
      return false;
    } else if (type_str == "floating") {
      joint.joint_type = JOINT_INVALID;
      logger.report_error("floating joints not supported");
      return false;
    } else if (type_str == "revolute") {
      joint.joint_type = JOINT_REVOLUTE_AXIS;
    } else if (type_str == "continuous") {
      joint.joint_type = JOINT_REVOLUTE_AXIS;
    } else if (type_str == "prismatic") {
      joint.joint_type = JOINT_PRISMATIC_AXIS;
    } else if (type_str == "fixed") {
      joint.joint_type = JOINT_FIXED;
    } else {
      logger.report_error("Joint ");
      logger.report_error(joint.joint_name);
      logger.report_error("has unknown type:");
      logger.report_error(type_str.c_str());
      return false;
    }

    {
      // Get Joint Axis
      if (joint.joint_type != JOINT_FIXED) {
        // axis
        tinyxml2::XMLElement* axis_xml = config->FirstChildElement("axis");
        if (!axis_xml) {
          logger.report_warning(
              "urdfdom: no axis elemement for Joint, defaulting to (1,0,0) "
              "axis");
          logger.report_warning(joint.joint_name);
          joint.joint_axis_xyz =
              TinyVector3(TinyConstants::zero(), TinyConstants::zero(),
                          TinyConstants::one());
        } else {
          if (axis_xml->Attribute("xyz")) {
            if (!parse_vector3(joint.joint_axis_xyz, axis_xml->Attribute("xyz"),
                               logger)) {
              logger.report_error("Malformed axis element:");
              logger.report_error(joint.joint_name);
              logger.report_error(" for joint:");
              logger.report_error(axis_xml->Attribute("xyz"));
              return false;
            }
          }
        }
      }
    }
#if 0
		//todo
		// Get limit
		tinyxml2::XMLElement* limit_xml = config->FirstChildElement("limit");
		if (limit_xml)
		{
			if (!parseJointLimits(joint, limit_xml, logger))
			{
				logger.report_error("Could not parse limit element for joint:");
				logger.report_error(joint.m_name.c_str());
				return false;
			}
		}
		else if (joint.m_type == URDFRevoluteJoint)
		{
			logger.report_error("Joint is of type REVOLUTE but it does not specify limits");
			logger.report_error(joint.m_name.c_str());
			return false;
		}
		else if (joint.m_type == URDFPrismaticJoint)
		{
			logger.report_error("Joint is of type PRISMATIC without limits");
			logger.report_error(joint.m_name.c_str());
			return false;
		}
#endif

#if 0
		//todo
		//joint.m_jointDamping = 0;
		//joint.m_jointFriction = 0;

		// Get Dynamics
		XMLElement* prop_xml = config->FirstChildElement("dynamics");
		if (prop_xml)
		{
			// Get joint damping
			const char* damping_str = prop_xml->Attribute("damping");
			if (damping_str)
			{
				joint.m_jointDamping = TinyConstants::scalar_from_string(damping_str);
			}

			// Get joint friction
			const char* friction_str = prop_xml->Attribute("friction");
			if (friction_str)
			{
				joint.m_jointFriction = TinyConstants::scalar_from_string(friction_str);
			}

			if (damping_str == NULL && friction_str == NULL)
			{
				logger.report_error("joint dynamics element specified with no damping and no friction");
				return false;
			}
		}
#endif

    return true;
  }

  static void assign_links(
      const std::string& link_name,
      std::map<std::string, TinyUrdfJoint>& joints_by_name,
      std::map<std::string, std::string>& link_to_joint_name,
      std::map<std::string, std::string>& joint_to_parent_name,
      std::map<std::string, int>& link_name_to_index, int level) {
    std::cout << std::string(level, '-') << link_name << "["
              << link_name_to_index[link_name] << "]" << std::endl;
    // Iterate through the map
    for (auto it = joint_to_parent_name.begin();
         it != joint_to_parent_name.end(); it++) {
      // Check if value of this entry matches with given value
      if (it->second == link_name) {
        std::string joint_name = it->first;
        TinyUrdfJoint joint = joints_by_name[joint_name];
        int index = (int)link_name_to_index.size() -
                    1;  // compensate for parent link at -1
        link_name_to_index[joint.child_name] = index;
        assign_links(joint.child_name, joints_by_name, link_to_joint_name,
                     joint_to_parent_name, link_name_to_index, level + 1);
      }
    }
  }

  TinyUrdfStructures load_urdf(const std::string& file_name) {
    std::ifstream ifs(file_name);
    std::string urdf_string;

    if (!ifs.is_open()) {
      std::cout << "Error, cannot open file_name: " << file_name << std::endl;
      exit(-1);
    }

    urdf_string = std::string((std::istreambuf_iterator<char>(ifs)),
                              std::istreambuf_iterator<char>());

    StdLogger logger;
    int flags = 0;
    TinyUrdfStructures urdf_structures;
    load_urdf_from_string(urdf_string, flags, logger, urdf_structures);
    return urdf_structures;
  }

  static bool load_urdf_from_string(const std::string& urdf_text, int flags,
                                    TinyLogger& logger,
                                    TinyUrdfStructures& urdf_structures) {
    tinyxml2::XMLDocument xml_doc;
    xml_doc.Parse(urdf_text.c_str());
    if (xml_doc.Error()) {
      logger.report_error(xml_doc.ErrorStr());
      xml_doc.ClearError();
      return false;
    }
    logger.print_message("Opened XML file");

    tinyxml2::XMLElement* robot_xml = xml_doc.FirstChildElement("robot");
    if (!robot_xml) {
      logger.report_error("expected a robot element");
      return false;
    }

    // Get robot name
    const char* name = robot_xml->Attribute("name");
    if (!name) {
      logger.report_error("Expected a name for robot");
      return false;
    }
    urdf_structures.m_robot_name = name;

#if 0
		// Get all Material elements
		for (XMLElement* material_xml = robot_xml->FirstChildElement("material"); 
			material_xml; material_xml = material_xml->NextSiblingElement("material"))
		{
			UrdfMaterial* material = new UrdfMaterial;

			parseMaterial(*material, material_xml, logger);

			UrdfMaterial** mat = m_urdf2Model.m_materials.find(material->m_name.c_str());
			if (mat)
			{
				delete material;
				logger.report_warning("Duplicate material");
			}
			else
			{
				m_materials.insert(material->m_name.c_str(), material);
			}
		}
#endif
    std::map<std::string, TinyUrdfJoint> joints_by_name;
    std::map<std::string, std::string> link_to_joint_name;
    std::map<std::string, std::string> joint_to_parent_name;
    std::vector<TinyUrdfJoint> tmp_joints;

    // Get all Joint elements
    for (tinyxml2::XMLElement* joint_xml =
             robot_xml->FirstChildElement("joint");
         joint_xml; joint_xml = joint_xml->NextSiblingElement("joint")) {
      TinyUrdfJoint joint;
      if (parse_joint(urdf_structures, joint, joint_xml, logger)) {
        if (joint_to_parent_name.find(joint.joint_name) !=
            joint_to_parent_name.end()) {
          logger.report_error("joint '%s' is not unique.");
          logger.report_error(joint.joint_name);
          return false;
        } else {
          joint_to_parent_name[joint.joint_name] = joint.parent_name;
          link_to_joint_name[joint.child_name] = joint.joint_name;
          joints_by_name[joint.joint_name] = joint;
          tmp_joints.push_back(joint);
        }
      } else {
        logger.report_error("joint xml is not initialized correctly");
        return false;
      }
    }

    std::vector<std::string> parent_links;

    for (tinyxml2::XMLElement* link_xml = robot_xml->FirstChildElement("link");
         link_xml; link_xml = link_xml->NextSiblingElement("link")) {
      TinyUrdfLink link;

      const char* name = link_xml->Attribute("name");
      if (!name) {
        logger.report_error("Link with no name");
        return false;
      }
      std::string link_name = name;
      if (link_to_joint_name.find(link_name) == link_to_joint_name.end()) {
        parent_links.push_back(link_name);
      }
    }
    if (parent_links.size() > 1) {
      logger.report_error("multiple parent links");
      for (int i = 0; i < parent_links.size(); i++) {
        logger.report_error(parent_links[i]);
      }
      return false;
    } else {
      if (parent_links.size() < 1) {
        logger.report_error("No parent link");
        return false;
      }
    }

    logger.print_message(std::string("base link=") + parent_links[0]);
    // now order joints and links

    urdf_structures.m_name_to_link_index[parent_links[0]] = -1;
    int link_index = 0;
    // recursively assign links

    assign_links(parent_links[0], joints_by_name, link_to_joint_name,
                 joint_to_parent_name, urdf_structures.m_name_to_link_index, 0);

    if (urdf_structures.m_name_to_link_index.size() !=
        (link_to_joint_name.size() + 1)) {
      logger.report_error("Inconsistent joint/link connections");
      return false;
    }

    urdf_structures.m_links.resize(link_to_joint_name.size());

    for (tinyxml2::XMLElement* link_xml = robot_xml->FirstChildElement("link");
         link_xml; link_xml = link_xml->NextSiblingElement("link")) {
      TinyUrdfLink link;

      if (parse_link(link, urdf_structures, link_xml, logger)) {
        if (urdf_structures.m_name_to_link_index.find(link.link_name) ==
            urdf_structures.m_name_to_link_index.end()) {
          logger.report_error("Link inconsistency.");
          logger.report_error(link.link_name);
          return false;
        }

        link.m_parent_index = -2;  // no parent
        int link_index = urdf_structures.m_name_to_link_index[link.link_name];
        if (link_index >= 0) {
          link.m_parent_index =
              urdf_structures.m_name_to_link_index
                  [joint_to_parent_name[link_to_joint_name[link.link_name]]];
        }

#if 0
				//copy model material into link material, if link has no local material
				for (int i = 0; i < link->m_visualArray.size(); i++)
				{
					UrdfVisual& vis = link->m_visualArray.at(i);
					if (!vis.m_geometry.m_hasLocalMaterial && vis.m_materialName.c_str())
					{
						UrdfMaterial** mat = m_urdf2Model.m_materials.find(vis.m_materialName.c_str());
						if (mat && *mat)
						{
							vis.m_geometry.m_localMaterial = **mat;
						}
						else
						{
							//logger.report_error("Cannot find material with name:");
							//logger.report_error(vis.m_materialName);
						}
					}
				}
#endif
        // root/base or child link?
        if (link_index >= 0) {
          urdf_structures.m_links[link_index] = link;
        } else {
          urdf_structures.m_base_links.push_back(link);
        }
      }
    }

    urdf_structures.m_joints.resize(tmp_joints.size());
    for (int i = 0; i < tmp_joints.size(); i++) {
      const TinyUrdfJoint& joint = tmp_joints[i];
      int link_index = urdf_structures.m_name_to_link_index[joint.child_name];
      urdf_structures.m_joints[link_index] = tmp_joints[i];
    }

#if 0
		if (flags & URDF_FORCE_FIXED_BASE)
		{
			
			for (int i = 0; i < urdf_structures.m_base_links.size(); i++)
			{
				urdf_structures.m_base_links[i].
				link->m_inertia.m_mass = 0.0;
				link->m_inertia.m_ixx = 0.0;
				link->m_inertia.m_ixy = 0.0;
				link->m_inertia.m_ixz = 0.0;
				link->m_inertia.m_iyy = 0.0;
				link->m_inertia.m_iyz = 0.0;
				link->m_inertia.m_izz = 0.0;
			}
		}
#endif

    return true;
  }
};

#endif  // TINY_URDF_PARSER_H
