// Smoke test: instantiate every rviz plugin exported by jsk_rviz_plugins
// through pluginlib. Catches missing symbols, moc issues and broken
// plugin_description.xml entries without needing a GUI.

#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <QApplication>

#include <pluginlib/class_loader.hpp>
#include <rviz_common/display.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/tool.hpp>

template<typename BaseT>
void instantiate_all(const std::string & base_class)
{
  pluginlib::ClassLoader<BaseT> loader("rviz_common", base_class);
  std::vector<std::string> classes = loader.getDeclaredClasses();
  size_t tested = 0;
  for (const auto & clz : classes) {
    if (loader.getClassPackage(clz) != "jsk_rviz_plugins") {
      continue;
    }
    SCOPED_TRACE(clz);
    std::shared_ptr<BaseT> instance;
    EXPECT_NO_THROW(instance = loader.createSharedInstance(clz)) << clz;
    EXPECT_NE(nullptr, instance) << clz;
    ++tested;
  }
  SUCCEED() << tested << " " << base_class << " plugins instantiated";
}

TEST(JskRvizPluginLoad, displays)
{
  instantiate_all<rviz_common::Display>("rviz_common::Display");
}

TEST(JskRvizPluginLoad, panels)
{
  instantiate_all<rviz_common::Panel>("rviz_common::Panel");
}

TEST(JskRvizPluginLoad, tools)
{
  instantiate_all<rviz_common::Tool>("rviz_common::Tool");
}

int main(int argc, char ** argv)
{
  QApplication app(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
