#ifndef TRANSFORMABLE_MARKER_OPERATOR_H
#define TRANSFORMABLE_MARKER_OPERATOR_H

#ifndef Q_MOC_RUN
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <QtGlobal>
#if QT_VERSION >= QT_VERSION_CHECK(5, 0, 0)
#  include <QtWidgets>
#else
#  include <QtGui>
#endif
#include <jsk_interactive_marker_msgs/srv/set_marker_dimensions.hpp>
#include <jsk_interactive_marker_msgs/srv/get_marker_dimensions.hpp>
#include <jsk_interactive_marker_msgs/srv/get_transformable_marker_focus.hpp>
#include <jsk_recognition_msgs/msg/object_array.hpp>
#include <jsk_rviz_plugins_msgs/msg/transformable_marker_operate.hpp>
#include <jsk_rviz_plugins_msgs/srv/request_marker_operate.hpp>
#endif

class QLineEdit;
class QPushButton;

namespace jsk_interactive_marker
{
  class TransformableMarkerOperatorAction: public rviz_common::Panel
    {
      Q_OBJECT
      public:
      TransformableMarkerOperatorAction( QWidget* parent = 0 );

      virtual void onInitialize();
      virtual void load( const rviz_common::Config& config );
      virtual void save( rviz_common::Config config ) const;

    protected Q_SLOTS:
      void update();
      void updateServerName();
      void updateObjectArrayTopic();
      void updateFocusMarkerDimensions();
      void updateDimensionsService();
      void updateFrameId();
      void updateName();

      void callRequestMarkerOperateService(jsk_rviz_plugins_msgs::msg::TransformableMarkerOperate operate);
      void insertBoxService();
      void insertCylinderService();
      void insertTorusService();
      void insertMeshService();
      void eraseWithIdService();
      void eraseAllService();
      void eraseFocusService();
      void objectArrayCb(const jsk_recognition_msgs::msg::ObjectArray::ConstSharedPtr obj_array_msg);

    protected:
      // (re)create the persistent service clients when the server name changes
      void ensureServiceClients();

      QPushButton* insert_box_button_;
      QPushButton* insert_cylinder_button_;
      QPushButton* insert_torus_button_;
      QPushButton* insert_mesh_button_;

      QPushButton* erase_with_id_button_;
      QPushButton* erase_all_button_;
      QPushButton* erase_focus_button_;

      QVBoxLayout* layout;

      QLineEdit* server_name_editor_;
      QLineEdit* topic_name_editor_;
      QLineEdit* transform_name_editor_;
      QLineEdit* dimension_x_editor_;
      QLineEdit* dimension_y_editor_;
      QLineEdit* dimension_z_editor_;
      QComboBox* object_editor_;
      QLineEdit* dimension_radius_editor_;
      QLineEdit* dimension_sm_radius_editor_;
      QLineEdit* name_editor_;
      QLineEdit* description_editor_;
      QLineEdit* frame_editor_;
      QLineEdit* id_editor_;

      QTimer* update_timer_;

      std::vector<jsk_recognition_msgs::msg::Object> objects_;

      rclcpp::Node::SharedPtr nh_;
      rclcpp::Subscription<jsk_recognition_msgs::msg::ObjectArray>::SharedPtr sub_obj_array_;
      rclcpp::Client<jsk_rviz_plugins_msgs::srv::RequestMarkerOperate>::SharedPtr request_marker_operate_client_;
      rclcpp::Client<jsk_interactive_marker_msgs::srv::SetMarkerDimensions>::SharedPtr set_dimensions_client_;
      rclcpp::Client<jsk_interactive_marker_msgs::srv::GetMarkerDimensions>::SharedPtr get_dimensions_client_;
      rclcpp::Client<jsk_interactive_marker_msgs::srv::GetTransformableMarkerFocus>::SharedPtr get_focus_client_;
      std::string client_server_name_;
      bool focus_request_pending_;
      bool dimensions_request_pending_;
    };
}  // namespace jsk_interactive_marker

#endif
