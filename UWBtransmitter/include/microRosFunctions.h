#ifndef UWB_MICROROSFUNCTIONS_H
#define UWB_MICROROSFUNCTIONS_H

#include <Arduino.h>
#include "wheelchair_sensor_msgs/msg/uwb.h"

bool create_entities();
void destroy_entities();
bool is_agent_connected();
void publishUwbMessage(const wheelchair_sensor_msgs__msg__Uwb &msg);

#endif // UWB_MICROROSFUNCTIONS_H
