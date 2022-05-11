#include <stdio.h>    
#include <math.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include "pico/stdlib.h"
#include "pico_uart_transports.h"

#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/int32.h>

#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"

/* DEFINE */
#define PI 3.14159265
#define PWM_LEFT_1 15  
#define PWM_RIGHT_1 14           
#define PWM_LEFT_2 11 
#define PWM_RIGHT_2 10             
#define PWM_LEFT_3 13
#define PWM_RIGHT_3 12
#define PWM_LEFT_4 9 
#define PWM_RIGHT_4 8 

// 0 to max 65025
int PWM_MIN = 10;
int PWMRANGE = 60000;
int min_speed = 4;
int max_speed = 31.42;
bool running;
const uint LED_PIN = 25;

uint32_t u1Pwm, u2Pwm, u3Pwm, u4Pwm;

/* ROBOT PARAMS */
float l = 0.46;
float w = 0.375;
float wheel_radius = 0.1;


rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;


/* FUNCTIONS */
float mapPwm(float u, float out_min, float out_max)  
{
    u = abs(u);
    if(u<=1){
        return 0;
    }
    else if(out_max*((u+min_speed)/max_speed) < PWMRANGE){
        return out_max*((u+min_speed)/max_speed);
    }
    else{
      return PWMRANGE;
    }
}

void pwm_write(int pin,int pwm) {
    pwm_clear_irq(pwm_gpio_to_slice_num(pin));
    pwm_set_gpio_level(pin, pwm);
}

float idle(int x){
    switch (x){
        case 1:
            pwm_write(PWM_LEFT_1, 0);
            pwm_write(PWM_RIGHT_1, 0);
            break;
        case 2:
            pwm_write(PWM_LEFT_2, 0);            
            pwm_write(PWM_RIGHT_2, 0);
            break;
        case 3:
            pwm_write(PWM_LEFT_3, 0);
            pwm_write(PWM_RIGHT_3, 0);
            break;
        case 4:
            pwm_write(PWM_LEFT_4, 0);
            pwm_write(PWM_RIGHT_4, 0);  
            break;  
        default:
            break;    
    }
} 

void velocityCb(const void * msgin){
    const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
    
    float x = msg->linear.x;
    float y = msg->linear.y;
    float z = msg->angular.z;

    float u1 = (z * (-l - w) + x - y)/wheel_radius;
    float u2 = (z * (l + w) + x + y)/wheel_radius;
    float u3 = (z * (l + w) + x - y)/wheel_radius;
    float u4 = (z * (-l - w) + x + y)/wheel_radius;

  
    u1Pwm = mapPwm(u1, PWM_MIN, PWMRANGE);
    u2Pwm = mapPwm(u2, PWM_MIN, PWMRANGE);
    u3Pwm = mapPwm(u3, PWM_MIN, PWMRANGE);
    u4Pwm = mapPwm(u4, PWM_MIN, PWMRANGE);

    (u1==0) ? false : pwm_write((u1 > 0) ? PWM_RIGHT_1:PWM_LEFT_1, u1Pwm);
    (u2==0) ? false : pwm_write((u2 > 0) ? PWM_RIGHT_2:PWM_LEFT_2, u2Pwm);
    (u3==0) ? false : pwm_write((u3 > 0) ? PWM_RIGHT_3:PWM_LEFT_3, u3Pwm);
    (u4==0) ? false : pwm_write((u4 > 0) ? PWM_RIGHT_4:PWM_LEFT_4, u4Pwm);
}

int main() {
    stdio_init_all();

    gpio_set_function(PWM_LEFT_1, GPIO_FUNC_PWM);
    gpio_set_function(PWM_RIGHT_2, GPIO_FUNC_PWM);
    gpio_set_function(PWM_LEFT_3, GPIO_FUNC_PWM);
    gpio_set_function(PWM_RIGHT_4, GPIO_FUNC_PWM);
    gpio_set_function(PWM_LEFT_1, GPIO_FUNC_PWM);
    gpio_set_function(PWM_RIGHT_2, GPIO_FUNC_PWM);
    gpio_set_function(PWM_LEFT_3, GPIO_FUNC_PWM);
    gpio_set_function(PWM_RIGHT_4, GPIO_FUNC_PWM);

    uint slice_num_1 = pwm_gpio_to_slice_num(PWM_LEFT_1);
    uint slice_num_2 = pwm_gpio_to_slice_num(PWM_LEFT_2);
    uint slice_num_3 = pwm_gpio_to_slice_num(PWM_LEFT_3);
    uint slice_num_4 = pwm_gpio_to_slice_num(PWM_LEFT_4);
    uint slice_num_5 = pwm_gpio_to_slice_num(PWM_RIGHT_1);
    uint slice_num_6 = pwm_gpio_to_slice_num(PWM_RIGHT_2);
    uint slice_num_7 = pwm_gpio_to_slice_num(PWM_RIGHT_3);
    uint slice_num_8 = pwm_gpio_to_slice_num(PWM_RIGHT_4);          

    
    pwm_clear_irq(slice_num_1);
    pwm_clear_irq(slice_num_2);
    pwm_clear_irq(slice_num_3);
    pwm_clear_irq(slice_num_4);
    pwm_clear_irq(slice_num_5);
    pwm_clear_irq(slice_num_6);
    pwm_clear_irq(slice_num_7);
    pwm_clear_irq(slice_num_8);
    pwm_clear_irq(slice_num_9);


    pwm_set_irq_enabled(slice_num_1, true);
    pwm_set_irq_enabled(slice_num_2, true);
    pwm_set_irq_enabled(slice_num_3, true);
    pwm_set_irq_enabled(slice_num_4, true);
    pwm_set_irq_enabled(slice_num_5, true);
    pwm_set_irq_enabled(slice_num_6, true);
    pwm_set_irq_enabled(slice_num_7, true);
    pwm_set_irq_enabled(slice_num_8, true);
    pwm_set_irq_enabled(slice_num_9, true);



    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 4.f);

    pwm_init(slice_num_1, &config, true);
    pwm_init(slice_num_2, &config, true);
    pwm_init(slice_num_3, &config, true);
    pwm_init(slice_num_4, &config, true);
    pwm_init(slice_num_5, &config, true);
    pwm_init(slice_num_6, &config, true);
    pwm_init(slice_num_7, &config, true);
    pwm_init(slice_num_8, &config, true);
    pwm_init(slice_num_9, &config, true);

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    gpio_init(SERVO_BTN);
    gpio_set_dir(SERVO_BTN, GPIO_OUT);

    gpio_init(PWM_ANGLE_LEFT);
    gpio_set_dir(PWM_ANGLE_LEFT, GPIO_OUT);
    
    gpio_init(PWM_ANGLE_RIGHT);
    gpio_set_dir(PWM_ANGLE_RIGHT, GPIO_OUT);

    gpio_init(PWM_FEEDER_LEFT);
    gpio_set_dir(PWM_FEEDER_LEFT, GPIO_OUT);	
    
    gpio_init(PWM_FEEDER_RIGHT);
    gpio_set_dir(PWM_FEEDER_RIGHT, GPIO_OUT);

    gpio_init(IN_ROTOR);
    gpio_set_dir(IN_ROTOR, GPIO_OUT);	

    gpio_init(DOWN_SWITCH);
    gpio_set_dir(DOWN_SWITCH, false);
	gpio_pull_up(DOWN_SWITCH);
	
	gpio_init(UP_SWITCH);
	gpio_set_dir(UP_SWITCH, false);
	gpio_pull_up(UP_SWITCH);

    //Check if micro-ROS Agent answers to micro-ROS client
    rmw_uros_set_custom_transport(
        true,
        NULL,
        pico_serial_transport_open,
        pico_serial_transport_close,
        pico_serial_transport_write,
        pico_serial_transport_read
    );

    // rcl_timer_t timer_a;
    // rcl_timer_t timer_b;
    // rcl_timer_t timer_c;
    // rcl_timer_t timer_d;

    rcl_node_t node;
    rcl_allocator_t allocator;
    rclc_support_t support;
    rclc_executor_t executor;

    allocator = rcl_get_default_allocator();

    //Wait for agent successful ping for 2 minutes.
    const int timeout_ms = 1000; 
    const uint8_t attempts = 120;
    rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);
    if (ret != RCL_RET_OK){return ret;} // Unreachable agent, exiting program.

    //create init_options
    rclc_support_init(&support, 0, NULL, &allocator);

    // create node
    rclc_node_init_default(&node, "cmd_vel_node", "", &support);

    running = false;

    // create subscriber
    rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel");

    rclc_subscription_init_default(
        &subscriber_twist,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "twisto");

    // //create publisher u1
    // rclc_publisher_init_default(
	// 	&publisher_u1,
	// 	&node,
	// 	ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
	// 	"cmd_vel_pub_u1");

    // //create publisher u2
    // rclc_publisher_init_default(
	// 	&publisher_u2,
	// 	&node,
	// 	ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
	// 	"cmd_vel_pub_u2");    
    
    // //create publisher u3
    // rclc_publisher_init_default(
	// 	&publisher_u3,
	// 	&node,
	// 	ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
	// 	"cmd_vel_pub_u3");

    // //create publisher u4
    // rclc_publisher_init_default(
	// 	&publisher_u4,
	// 	&node,
	// 	ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
	// 	"cmd_vel_pub_u4");

    // //timer callback for each publisher
	// rclc_timer_init_default(
	// 	&timer_a,
	// 	&support,
	// 	RCL_MS_TO_NS(1000),
	// 	timer_callback_u1);

    // rclc_timer_init_default(
	// 	&timer_b,
	// 	&support,
	// 	RCL_MS_TO_NS(1000),
	// 	timer_callback_u2);

    // rclc_timer_init_default(
	// 	&timer_c,
	// 	&support,
	// 	RCL_MS_TO_NS(1000),
	// 	timer_callback_u3);

    // rclc_timer_init_default(
	// 	&timer_d,
	// 	&support,
	// 	RCL_MS_TO_NS(1000),
	// 	timer_callback_u4);

    // create executor
    rclc_executor_init(&executor, &support.context, 6, &allocator);
    // rclc_executor_add_timer(&executor, &timer_a);
    // rclc_executor_add_timer(&executor, &timer_b);
    // rclc_executor_add_timer(&executor, &timer_c);
    // rclc_executor_add_timer(&executor, &timer_d);
    rclc_executor_add_subscription(&executor, &subscriber, &msg, &velocityCb, ON_NEW_DATA);
    rclc_executor_add_subscription(&executor, &subscriber_twist, &msg_int, &twistoCb, ON_NEW_DATA);

    //gpio_put(LED_PIN, 1); 

    // msg_float_u1.data = 0;
    // msg_float_u2.data = 0;
    // msg_float_u3.data = 0; 
    // msg_float_u4.data = 0;  

    while (true){
        rclc_executor_spin_some(&executor, RCL_S_TO_NS(1));
    }
    
    return 0;
}