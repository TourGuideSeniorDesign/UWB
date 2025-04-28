/* MicroROS Publisher code for the UWB transmitter
*/

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
#endif

// Executor
rclc_executor_t executor;

// Subscriber
/* rcl_subscription_t subscriber;
wheelchair_sensor_msgs__msg__UWB UWBdistance; */

// UWB publisher and timer
rcl_publisher_t uwb_Publisher;
rcl_timer_t uwbTimer;
wheelchair_sensor_msgs__msg__UWB UWBdistance;

rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

// error handling
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){} }

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){} }

// Error handle loop
void error_loop() {
    while(1) {
        delay(100);
    }
}

// Create Entities
void createEntities(const char* nodeName, const char* pubTopicName, uint16_t timer_interval_ms) {
    allocator = rcl_get_default_allocator();
    set_microros_serial_transports(Serial);
    delay(2000);

    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, nodeName, "", &support));

    RCCHECK(rclc_publisher_init_default(
        &uwbPublisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(wheelchair_sensor_msgs, msg, UWB),
        pubTopicName));

    RCCHECK(rclc_timer_init_default(
        &uwbTimer,
        &support,
        RCL_MS_TO_NS(timer_interval_ms),
        uwb_timer_callback));

    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &uwbTimer));
}

// Destroy Entities
void destroyEntities() {
    rcl_publisher_fini(&uwbPublisher, &node);
    rcl_timer_fini(&uwbTimer);
    rcl_node_fini(&node);
    rclc_executor_fini(&executor);
    rclc_support_fini(&support);
}

// Timer callback to publish UWB message
void uwb_timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);
    if (timer == NULL) return;

    for (int i = 0; i < MAX_RESPONDERS; i++) {
        float distance; // get the actual reading from the sensor
        if (distance > 0.0f) {
            switch(i) {
                case 0: uwbMsg.distance1 = distance; break;
                case 1: uwbMsg.distance2 = distance; break;
                case 2: uwbMsg.distance3 = distance; break;
                case 3: uwbMsg.distance4 = distance; break;
                case 4: uwbMsg.distance5 = distance; break;
            }
        }
    }
    RCSOFTCHECK(rcl_publish(&uwbPublisher, &uwbMsg, NULL));
}

#ifdef ROS_DEBUG
void timer_callback(rcl_timer_t * inputTimer, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);
    if (inputTimer != NULL) {
        RCSOFTCHECK(rcl_publish(&uwbPublisher, &uwbMsg, NULL));
    }
}
#endif

// Loop Tick
void microRosTick() {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}


void subscription_callback(const void *msgin){
    const wheelchair_sensor_msgs__msg__UWB *msg = (const wheelchair_sensor_msgs__msg__UWB *)msgin;
    UWBdistance= *msg;
#ifdef ROS_DEBUG
#elif ROS
#endif
}


UWBdistance getUWBDistance(){
  UWBdistance d;
  d.distance1=uwbMsg.distance1;
  d.distance2=uwbMsg.distance2;
  d.distance3=uwbMsg.distance3;
  d.distance4=uwbMsg.distance4;
  d.distance5=uwbMsg.distance5;
  return d;
}

#endif