// Smoke test: instantiate the rviz panel exported by jsk_interactive_marker
// through pluginlib. Catches missing symbols, moc issues and broken
// plugin_description.xml entries without needing a GUI.
//
// The panel is constructed and destroyed without calling onInitialize(),
// which is safe for TransformableMarkerOperatorAction (it only builds Qt
// widgets in its constructor).

#include <memory>
#include <string>

#include <gtest/gtest.h>

#include <QApplication>

#include <pluginlib/class_loader.hpp>
#include <rviz_common/panel.hpp>

TEST(JskInteractiveMarkerPluginLoad, transformable_marker_operator_panel)
{
  pluginlib::ClassLoader<rviz_common::Panel> loader(
    "rviz_common", "rviz_common::Panel");

  const std::string clz = "jsk_rviz_plugin/TransformableMarkerOperatorAction";
  ASSERT_EQ("jsk_interactive_marker", loader.getClassPackage(clz));

  rviz_common::Panel * panel = nullptr;
  EXPECT_NO_THROW(panel = loader.createUnmanagedInstance(clz)) << clz;
  ASSERT_NE(nullptr, panel) << clz;
  EXPECT_NO_THROW(delete panel) << clz;
}

int main(int argc, char ** argv)
{
  QApplication app(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
