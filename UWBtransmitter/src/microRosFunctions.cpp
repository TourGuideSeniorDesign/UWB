#include "microRosFunctions.h"
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/publisher.h>

rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_publisher_t publisher;

void microRosSetup() {
    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);

    rclc_node_init_default(&node, "uwb_transmitter_node", "", &support);

    rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(wheelchair_sensor_msgs, msg, Uwb),
        "uwb"
    );
}

void transmitUwbDistances(const wheelchair_sensor_msgs__msg__Uwb &msg) {
    rcl_publish(&publisher, &msg, NULL);
}

