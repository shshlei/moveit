#include <moveit/collision_detection_bullet/bullet_integration/bullet_discrete_bvh_manager.h>

std::string toString(const Eigen::MatrixXd& a)
{
  std::stringstream ss;
  ss << a;
  return ss.str();
}

std::string toString(bool b)
{
  return b ? "true" : "false";
}

int main(int /*argc*/, char** /*argv*/)
{
  collision_detection_bullet::BulletDiscreteBVHManager checker;

  // Create a box
  shapes::ShapePtr box = std::make_shared<shapes::Box>(1, 1, 1);
  Eigen::Isometry3d box_pose;
  box_pose.setIdentity();
  box_pose.translation()(0) = 1.2;
  box_pose.translation()(1) = 1.1;

  std::vector<shapes::ShapeConstPtr> obj1_shapes;
  EigenSTL::vector_Isometry3d obj1_poses;
  obj1_shapes.push_back(box);
  obj1_poses.push_back(box_pose);

  std::vector<collision_detection::CollisionObjectType> collision_object_types(
      1, collision_detection::CollisionObjectType::USE_SHAPE_TYPE);

  checker.addCollisionObject("box_link", collision_detection::BodyType::ROBOT_LINK, obj1_shapes, obj1_poses,
                             collision_object_types, true);

  // Create a thin box
  shapes::ShapePtr sphere = std::make_shared<shapes::Sphere>(0.1);
  Eigen::Isometry3d sphere_pose;
  sphere_pose.setIdentity();

  std::vector<shapes::ShapeConstPtr> obj2_shapes;
  EigenSTL::vector_Isometry3d obj2_poses;
  obj2_shapes.push_back(sphere);
  obj2_poses.push_back(sphere_pose);

  checker.addCollisionObject("sphere", collision_detection::BodyType::WORLD_OBJECT, obj2_shapes, obj2_poses,
                             collision_object_types, false);

  collision_detection::CollisionRequest req;
  collision_detection::CollisionResult res;
  req.distance = true;

  collision_detection::ContactTestData cdata(0.0, 0.0, 1.0, nullptr, &req, &res);

  checker.contactTest(cdata, false);

  std::cout << "Has collision: " << toString(cdata.res->collision).c_str() << std::endl;
  std::cout << "Distance: " << cdata.res->distance << std::endl;

  return 0;
}
