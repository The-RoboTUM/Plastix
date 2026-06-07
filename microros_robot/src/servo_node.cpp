#include <micro_ros_arduino.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float32_multi_array.h>

#include <ESP32Servo.h>

#define NUM_SERVOS 4

// Servo pins
const int servoPins[NUM_SERVOS] = {18, 19, 21, 22};

Servo servos[NUM_SERVOS];

rcl_subscription_t subscriber;
std_msgs__msg__Float32MultiArray msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

float clamp(float x, float min_val, float max_val)
{
    if (x < min_val) return min_val;
    if (x > max_val) return max_val;
    return x;
}

// Convert radians [-pi/4, pi/4] to servo angle [0,180]
int radToServo(float rad)
{
    rad = clamp(rad, -M_PI/4.0f, M_PI/4.0f);

    float normalized =
        (rad + M_PI/4.0f) / (M_PI/2.0f);

    return (int)(normalized * 180.0f);
}

void subscription_callback(const void *msgin)
{
    const std_msgs__msg__Float32MultiArray *msg =(const std_msgs__msg__Float32MultiArray *)msgin;

    if (msg->data.size < NUM_SERVOS)
    {
        return;
    }

    for (int i = 0; i < NUM_SERVOS; i++)
    {
        float position = msg->data.data[i];
        int servoAngle = radToServo(position);
        servos[i].write(servoAngle);
    }
}

void setup()
{
    set_microros_transports();
    for (int i = 0; i < NUM_SERVOS; i++)
    {
        servos[i].attach(servoPins[i]);
        servos[i].write(90); // center position
    }
    delay(2000);

    allocator = rcl_get_default_allocator();
    rclc_support_init(&support,0,NULL,&allocator);

    rclc_node_init_default(&node,"servo_controller_node","",&support);

    rclc_subscription_init_default(
        &subscriber,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg,Float32MultiArray),"servo_positions");

    rclc_executor_init(&executor,&support.context,1,&allocator);

    rclc_executor_add_subscription(&executor,&subscriber,&msg,&subscription_callback,ON_NEW_DATA);
}

void loop()
{
    rclc_executor_spin_some(&executor,RCL_MS_TO_NS(10));
    delay(10);
}