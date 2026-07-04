#ifndef JSK_RVIZ_PLUGINS_IMAGE_TRANSPORT_HINTS_PROPERTY_H
#define JSK_RVIZ_PLUGINS_IMAGE_TRANSPORT_HINTS_PROPERTY_H

#include <rviz_common/properties/property.hpp>
#include <rviz_common/properties/editable_enum_property.hpp>

#include <string>

namespace jsk_rviz_plugins {

class ImageTransportHintsProperty : public rviz_common::properties::EditableEnumProperty
{
  Q_OBJECT
 public:
  ImageTransportHintsProperty(const char* name, const char* description,
                              rviz_common::properties::Property* parent,
                              const char* changed_slot);
  ~ImageTransportHintsProperty();

  // In ROS 2 image_transport::TransportHints requires a node,
  // so this returns the transport name instead.
  std::string getTransportHints();
};
}
#endif
