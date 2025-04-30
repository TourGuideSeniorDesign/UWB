#include "microRosFunctions.h"
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>

rclc_support_t support;
rcl_node_t node;
rcl_allocator_t allocator;
rcl_publisher_t publisher;

void microRosTick(){
    switch (state) {
        case WAITING_AGENT:
            EXECUTE_EVERY_N_MS(500,
                               state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
            break;
        case AGENT_AVAILABLE:
            state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
            if (state == WAITING_AGENT) {
                destroy_entities();
            };
            break;
        case AGENT_CONNECTED:
            EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED
                                                                                        : AGENT_DISCONNECTED;);
            if (state == AGENT_CONNECTED) {
                rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
            }
            break;
        case AGENT_DISCONNECTED:
            destroy_entities();
            state = WAITING_AGENT;
            break;
        default:
            break;
    }
}

bool create_entities() {
    allocator = rcl_get_default_allocator();
    if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) {
        return false;
    }
    if (rclc_node_init_default(&node, "uwb_transmitter_node", "", &support) != RCL_RET_OK) {
        return false;
    }
    if (rclc_publisher_init_best_effort(
            &publisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(wheelchair_sensor_msgs, msg, Uwb),
            "uwb") != RCL_RET_OK) {
        return false;
    }
    return true;
}

void destroy_entities() {
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    rcl_publisher_fini(&publisher, &node);
    rcl_node_fini(&node);
    rclc_support_fini(&support);
}

bool is_agent_connected() {
    return (RMW_RET_OK == rmw_uros_ping_agent(100, 1));
}

void publishUwbMessage(const wheelchair_sensor_msgs__msg__Uwb &msg) {
    rcl_publish(&publisher, &msg, NULL);
}

