// Copyright 2026 rcllf Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>

#include <cstdio>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "ros2_lf_binding/node_registry.h"
#include "node_registry_internal.h"  // NOLINT(build/include_subdir)

class NodeRegistryTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite()
  {
    // Clear all registry entries so no Node SharedPtrs outlive rclcpp::shutdown().
    ros2_lf_clear_node_registry();
    rclcpp::shutdown();
  }
};

TEST_F(NodeRegistryTest, RegisterAndGet)
{
  auto node = std::make_shared<rclcpp::Node>("test_node");
  ros2_lf_register_node("MyClass", node);
  auto result = ros2_lf_get_node("MyClass");
  EXPECT_EQ(result, node->get_node_base_interface());
}

TEST_F(NodeRegistryTest, RegisterAndGetLifecycleNode)
{
  // A managed lifecycle node is a sibling of rclcpp::Node, not a subclass;
  // registration must go through the type-erased template overload.
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("lc_test_node");
  ros2_lf_register_node("MyLifecycleClass", node);
  auto result = ros2_lf_get_node("MyLifecycleClass");
  ASSERT_NE(result, nullptr);
  EXPECT_EQ(result, node->get_node_base_interface());
}

TEST_F(NodeRegistryTest, GetUnknownReturnsNull)
{
  EXPECT_EQ(ros2_lf_get_node("DoesNotExist"), nullptr);
}

TEST_F(NodeRegistryTest, OverwriteRegistration)
{
  auto n1 = std::make_shared<rclcpp::Node>("n1");
  auto n2 = std::make_shared<rclcpp::Node>("n2");
  ros2_lf_register_node("Same", n1);
  ros2_lf_register_node("Same", n2);
  EXPECT_EQ(ros2_lf_get_node("Same"), n2->get_node_base_interface());
}

TEST_F(NodeRegistryTest, ParseNonComponentJsonEntry)
{
  // Simulate what node_factories.cpp does: pre-register a node, then
  // verify ros2_lf_get_node retrieves it (as ros2_lf_bind_node will do).
  auto fake_node = std::make_shared<rclcpp::Node>("fake");
  ros2_lf_register_node("PublisherNode", fake_node);
  auto found = ros2_lf_get_node("PublisherNode");
  EXPECT_EQ(found.get(), fake_node->get_node_base_interface().get());
}

// A class with two or more instances is registered only under its
// per-instance keys `<Class>__<suffix>`; the startup "not in registry" check
// must count it as registered, and a class without any node must still miss.
TEST_F(NodeRegistryTest, ClassWithTwoInstancesCountsAsRegistered)
{
  auto a = std::make_shared<rclcpp::Node>("take_writer_a");
  auto b = std::make_shared<rclcpp::Node>("take_writer_b");
  ros2_lf_register_node("TwoInstWriter__take_writer_a", "TwoInstWriter", a);
  ros2_lf_register_node("TwoInstWriter__take_writer_b", "TwoInstWriter", b);
  EXPECT_EQ(ros2_lf_get_node("TwoInstWriter"), nullptr);
  EXPECT_NE(ros2_lf_get_node("TwoInstWriter__take_writer_a"), nullptr);
  EXPECT_TRUE(ros2_lf_class_has_registered_node("TwoInstWriter"))
    << "class registered only under '<Class>__<suffix>' keys reported as not in registry";
}

TEST_F(NodeRegistryTest, ClassWithBareKeyCountsAsRegistered)
{
  auto n = std::make_shared<rclcpp::Node>("single_instance");
  ros2_lf_register_node("SingleInstClass", n);
  EXPECT_TRUE(ros2_lf_class_has_registered_node("SingleInstClass"));
}

TEST_F(NodeRegistryTest, MissingClassStillCountsAsUnregistered)
{
  auto n = std::make_shared<rclcpp::Node>("other_writer");
  ros2_lf_register_node("OtherWriter__w", "OtherWriter", n);
  EXPECT_FALSE(ros2_lf_class_has_registered_node("NeverRegisteredClass"));
  // A class name that is only a prefix of another class is not that class.
  EXPECT_FALSE(ros2_lf_class_has_registered_node("Other"));
  EXPECT_FALSE(ros2_lf_class_has_registered_node("OtherWrite"));
}

// The class comes from the registration, not from the key: a class whose own
// name is `Foo__bar`, registered under that bare key, does not stand in for a
// missing class `Foo`.
TEST_F(NodeRegistryTest, KeyShapedClassNameDoesNotHideMissingClass)
{
  auto n = std::make_shared<rclcpp::Node>("foo_bar");
  ros2_lf_register_node("Foo__bar", n);
  EXPECT_TRUE(ros2_lf_class_has_registered_node("Foo__bar"));
  EXPECT_FALSE(ros2_lf_class_has_registered_node("Foo"))
    << "class 'Foo' counted as registered because class 'Foo__bar' is";
}

// A registration with a null node holds nothing to bind, so its class still
// counts as missing and the startup warning still fires (as on main, where
// the lookup returned a null base).
TEST_F(NodeRegistryTest, NullNodeRegistrationCountsAsMissing)
{
  ros2_lf_register_node("Broken", std::shared_ptr<rclcpp::Node>{});
  EXPECT_FALSE(ros2_lf_class_has_registered_node("Broken"))
    << "class 'Broken' counted as registered although its node is null";
}

// Launch parameters=[{'robot': 'r1'}, file, {'rate': 8}] where the
// file sets rate: 7 under the node's own key. launch_ros gives rate 8 because
// each dict becomes a params file keyed by the node, applied in list order.
TEST_F(NodeRegistryTest, InlineLaunchParamsKeepTheirPlaceAmongFiles)
{
  const std::string file = "/tmp/rcllf_test_node_registry_params.yaml";
  {
    std::ofstream f(file);
    f << "/ns1/sub:\n  ros__parameters:\n    rate: 7\n    robot: from_file\n";
  }
  const std::vector<std::string> args = ros2_lf_materialize_param_args(
    {"--ros-args", "-r", "__node:=sub",
      "--rcllf-params-yaml", "{/ns1/sub: {ros__parameters: {robot: r1}}}",
      "--params-file", file,
      "--rcllf-params-yaml", "{/ns1/sub: {ros__parameters: {rate: 8}}}"});
  ASSERT_EQ(args.size(), 9u);
  EXPECT_EQ(args[3], "--params-file");
  EXPECT_EQ(args[6], file);
  EXPECT_EQ(args[7], "--params-file");
  for (const auto & a : args) {
    EXPECT_NE(a, "--rcllf-params-yaml");
  }

  rclcpp::NodeOptions options;
  options.arguments(args);
  auto node = std::make_shared<rclcpp::Node>("sub", "ns1", options);
  EXPECT_EQ(node->declare_parameter<int64_t>("rate", 0), 8)
    << "the later inline value must win over the earlier node-keyed file";
  EXPECT_EQ(node->declare_parameter<std::string>("robot", ""), "from_file")
    << "the node-keyed file must win over the earlier inline value";
  std::remove(file.c_str());
  std::remove(args[4].c_str());
  std::remove(args[8].c_str());
}

TEST_F(NodeRegistryTest, InlineLaunchParamsFlagWithoutTextIsAnError)
{
  EXPECT_THROW(
    ros2_lf_materialize_param_args({"--ros-args", "--rcllf-params-yaml"}),
    std::runtime_error);
}
