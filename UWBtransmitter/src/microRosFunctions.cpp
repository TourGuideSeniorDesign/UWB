#if defined(ROS) || defined(ROS_DEBUG)

#include "dw3000Functions.h"
#include "Arduino.h"
#include "DW3000.h"
#include "microRosFunctions.h"
#include <micro_ros_platformio.h>
#include <wheelchair_sensor_msgs/msg/UWB.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

// number of receivers
#define MAX_RESPONDERS 5

// ROS messages
#ifdef ROS_DEBUG
#include <wheelchair_sensor_msgs/msg/UWB.h>

#elif ROS

#endif

// Executor
rclc_executor_t executor;

// Subscriber
rcl_subscription_t subscriber;
wheelchair_sensor_msgs__msg__UWB UWBdistance;


rcl_subscription_t uwb_subscriber;
wheelchair_sensor_msgs__msg__UWB UWBdistance;

// UWB publisher and timer
rcl_publisher_t uwb_Publisher;
rcl_timer_t uwbTimer;
wheelchair_sensor_msgs__msg__UWB UWBdistance;


#ifdef ROS_DEBUG

#endif

rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

// error handling
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){} }

// Error handle loop
void error_loop() {
    while(1) {
        delay(100);
    }
}

// Callback to trigger TWR and publish result
void uwb_timer_callback(rcl_timer_t *input_timer, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);
    if (input_timer == NULL) return;

    for (int i = 0; i < MAX_RESPONDERS; i++) {
        float distance = rangingManager.rangeWithResponder(i);
        if (distance > 0.0f) {
            uwbMsg.responder_id = i;
            uwbMsg.distance_m = distance;
            RCSOFTCHECK(rcl_publish(&uwbPublisher, &uwbMsg, NULL));
        }
    }
}

#ifdef ROS_DEBUG
void timer_callback(rcl_timer_t * inputTimer, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);
    if (inputTimer != NULL) {
        RCSOFTCHECK(rcl_publish(&uwbPublisher, &uwbMsg, NULL));
    }
}


void subscription_callback(const void *msgin)
{
    const wheelchair_sensor_msgs__msg__RefSpeed *msg = (const wheelchair_sensor_msgs__msg__RefSpeed *)msgin;
    refSpeedMsg = *msg;
#ifdef ROS_DEBUG


// UWB setup
void uwbSetup(uint16_t timer_interval_ms, const char* pubTopicName) {
    if (!DW3000.begin()) {
        error_loop();  // Failed to initialize
    }

    rangingManager.init(&DW3000);

    // ROS: Init publisher
    RCCHECK(rclc_publisher_init_default(
        &uwbPublisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(wheelchair_sensor_msgs, msg, UwbDistance),
        pubTopicName));

    // Timer for TWR loop
    RCCHECK(rclc_timer_init_default(
        &uwbTimer,
        &support,
        RCL_MS_TO_NS(timer_interval_ms),
        uwb_timer_callback));

    // Add to executor
    RCCHECK(rclc_executor_add_timer(&executor, &uwbTimer));
}

#endif


    void microRosSetup(unsigned int timer_timeout, const char* nodeName, const char* subTopicName, const char* pubTopicName){

    set_microros_serial_transports(Serial);
    delay(2000);
    allocator = rcl_get_default_allocator();

    const size_t domain_id = 7;
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, nodeName, "", &support));

    // create uwb subscriber
    RCCHECK(rclc_subscription_init_best_effort(
            &uwb_subscriber1,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(wheelchair_sensor_msgs, msg, distance1),
            "distance1"));

    RCCHECK(rclc_subscription_init_best_effort(
            &uwb_subscriber2,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(wheelchair_sensor_msgs, msg, distance2),
            "distance2"));

    RCCHECK(rclc_subscription_init_best_effort(
            &uwb_subscriber3,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(wheelchair_sensor_msgs, msg, distance3),
            "distance3"));

    RCCHECK(rclc_subscription_init_best_effort(
            &uwb_subscriber4,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(wheelchair_sensor_msgs, msg, distance4),
            "distance4"));
    RCCHECK(rclc_subscription_init_best_effort(
            &uwb_subscriber4,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(wheelchair_sensor_msgs, msg, distance5),
            "distance5"));


    // uwb: init publisher and timer
    RCCHECK(rclc_publisher_init_default(
        &uwbPublisher1,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(wheelchair_sensor_msgs, msg, distance1),
        "uwb_distance1"));

    // uwb: init publisher and timer
    RCCHECK(rclc_publisher_init_default(
        &uwbPublisher2,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(wheelchair_sensor_msgs, msg, distance2),
        "uwb_distance2"));

    // uwb: init publisher and timer
    RCCHECK(rclc_publisher_init_default(
        &uwbPublisher3,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(wheelchair_sensor_msgs, msg, distance3),
        "uwb_distance3"));

    // uwb: init publisher and timer
    RCCHECK(rclc_publisher_init_default(
        &uwbPublisher4,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(wheelchair_sensor_msgs, msg, distance4),
        "uwb_distance4"));

    // uwb: init publisher and timer
    RCCHECK(rclc_publisher_init_default(
        &uwbPublisher4,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(wheelchair_sensor_msgs, msg, distance5),
        "uwb_distance5"));


    RCCHECK(rclc_timer_init_default(
        &uwbTimer,
        &support,
        RCL_MS_TO_NS(1000)
        uwb_timer_callback));


#ifdef ROS


#elif ROS_DEBUG

    RCCHECK(rclc_publisher_init_best_effort(
        &uwbPublisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(wheelchair_sensor_msgs, msg, uwbValues),
        "uwb_value"));


#endif

#ifdef ROS_DEBUG
    RCCHECK(rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(timer_timeout),
        timer_callback));
#endif

#endif