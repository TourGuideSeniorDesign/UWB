#ifndef UWB_MICROROSFUNCTIONS_H
#define UWB_MICROROSFUNCTIONS_H

#include <Arduino.h>
#include "wheelchair_sensor_msgs/msg/uwb.h"

/**
 * Initializes the micro-ROS node and publisher for UWB distances.
 */
void microRosSetup();

/**
 * Publishes the distance message to the ROS topic.
 * @param msg A populated wheelchair_sensor_msgs__msg__Uwb message.
 */
void transmitUwbDistances(const wheelchair_sensor_msgs__msg__Uwb &msg);

#endif // UWB_MICROROSFUNCTIONS_H
