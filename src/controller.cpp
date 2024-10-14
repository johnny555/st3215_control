#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>


#include <sensor_msgs/msg/joint_state.h>
#include "jointstate.h"
#include "syslog.h"


#define AGENT_IP { 192, 168, 1, 108 }  // eg IP of the desktop computer
#define AGENT_PORT 8888
// Enable WiFi with null terminated list of multiple APs SSID and password
#define WIFI_AP_LIST {{"The Hutt_EXT", "castleravenloft"}, {"The Hutt","castleravenloft"} , {NULL}}




#ifndef RCCHECK
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){rclErrorLoop();}}
#endif
#ifndef RCSOFTCHECK
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
#endif

#ifndef CONTROL_TIMER
#define CONTROL_TIMER 20 // 50Hz
#endif

#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)


// the uart used to control servos.
// GPIO 18 - S_RXD, GPIO 19 - S_TXD, as default.
#define S_RXD 18
#define S_TXD 19
#include <SCServo.h>

void rclErrorLoop()
{
    while(true)
    {
        //Serial.println("ROS Error setup");
    }
}

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t control_timer;

rcl_publisher_t joint_state_publisher;
sensor_msgs__msg__JointState joint_msg;
JointState joint_state;


enum states
{
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;



SMS_STS sms_sts;


bool syncTime()
{
    const int timeout_ms = 1000;
    if (rmw_uros_epoch_synchronized()) return true; // synchronized previously
    // get the current time from the agent
    RCCHECK(rmw_uros_sync_session(timeout_ms));
    if (rmw_uros_epoch_synchronized()) {
        #if (_POSIX_TIMERS > 0)
                // Get time in milliseconds or nanoseconds
                int64_t time_ns = rmw_uros_epoch_nanos();
            timespec tp;
            tp.tv_sec = time_ns / 1000000000;
            tp.tv_nsec = time_ns % 1000000000;
            clock_settime(CLOCK_REALTIME, &tp);
        #else
            unsigned long long ros_time_ms = rmw_uros_epoch_millis();
            // now we can find the difference between ROS time and uC time
            time_offset = ros_time_ms - millis();
        #endif
        return true;
    }
    return false;
}


struct timespec getTime()
{
    struct timespec tp = {0};
#if (_POSIX_TIMERS > 0)
    clock_gettime(CLOCK_REALTIME, &tp);
#else
    // add time difference between uC time and ROS time to
    // synchronize time with ROS
    unsigned long long now = millis() + time_offset;
    tp.tv_sec = now / 1000;
    tp.tv_nsec = (now % 1000) * 1000000;
#endif
    return tp;
}


void readServoIntoJointMsg()
{
    int num_joints = 1;
    JointData joint_data[num_joints];

    for (int i = 0; i < num_joints; i++) {
        joint_data[i].position = sms_sts.ReadPos(i + 1);
        joint_data[i].velocity = sms_sts.ReadSpeed(i + 1);
        joint_data[i].effort = sms_sts.ReadCurrent(i + 1);
    }

    joint_state.update(joint_data, num_joints);
}

void publishData()
{
    struct timespec time_stamp = getTime();

    joint_msg = joint_state.getData();


    joint_msg.header.stamp.sec = time_stamp.tv_sec;
    joint_msg.header.stamp.nanosec = time_stamp.tv_nsec;

    RCSOFTCHECK(rcl_publish(&joint_state_publisher, &joint_msg, NULL));
}


void controlCallback(rcl_timer_t * timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL)
    {
        readServoIntoJointMsg();
        publishData();
    }
}


bool createEntities()
{

    allocator = rcl_get_default_allocator();
    //create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    // create node
    RCCHECK(rclc_node_init_default(&node, "st3215_control", "", &support));


    RCCHECK(rclc_publisher_init_default(
    &joint_state_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    "joint_state"
    ))

    const unsigned int control_timeout = CONTROL_TIMER;
    RCCHECK(rclc_timer_init_default(
        &control_timer,
        &support,
        RCL_MS_TO_NS(control_timeout),
        controlCallback
    ));


    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
 
    RCCHECK(rclc_executor_add_timer(&executor, &control_timer));

    syncTime();

    return true;
}

bool destroyEntities()
{
    syslog(LOG_INFO, "%s %lu", __FUNCTION__, millis());
    rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
    (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    RCSOFTCHECK(rcl_publisher_fini(&joint_state_publisher, &node));
    RCSOFTCHECK(rcl_timer_fini(&control_timer));
    RCSOFTCHECK(rclc_executor_fini(&executor));
    RCSOFTCHECK(rcl_node_fini(&node))
    RCSOFTCHECK(rclc_support_fini(&support));

    return true;
}


void setup()
{
    Serial1.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
    Serial.begin(115200);
    sms_sts.pSerial = &Serial1;

    set_microros_serial_transports(Serial);

    delay(1000);
}

void loop () {
    switch (state)
    {
        case WAITING_AGENT:
            EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
            break;
        case AGENT_AVAILABLE:
            syslog(LOG_INFO, "%s agent available %lu", __FUNCTION__, millis());
            state = (true == createEntities()) ? AGENT_CONNECTED : WAITING_AGENT;
            if (state == WAITING_AGENT)
            {
                destroyEntities();
            }
            break;
        case AGENT_CONNECTED:
            EXECUTE_EVERY_N_MS(1000, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
            if (state == AGENT_CONNECTED)
            {
                rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
            }
            break;
        case AGENT_DISCONNECTED:
            syslog(LOG_INFO, "%s agent disconnected %lu", __FUNCTION__, millis());
            destroyEntities();
            state = WAITING_AGENT;
            break;
        default:
            break;
    }
}