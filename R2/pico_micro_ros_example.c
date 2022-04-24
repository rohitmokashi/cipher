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
#define ENA 15               
#define ENB 14               
#define ENC 13                  
#define IN_1 12              
#define IN_2 11               
#define IN_3 10                

#define S_1 16
#define S_1_PWM 17 

#define S_2 18
#define S_2_PWM 19

#define S_3 20
#define S_3_PWM 21 

#define G_1 9
#define G_1_PWM 8

#define G_2 7
#define G_2_PWM 6

#define G_3 5
#define G_3_PWM 4

// 0 to 65025.

/* CONFIGURATIONS */
int PWM_MIN = 10;
int PWMRANGE = 65025;
int min_speed = 0;
int max_speed = 31.42;
#define pwm_val 20000

const uint LED_PIN = 25;

uint32_t u1Pwm, u2Pwm, u3Pwm;

/* ROBOT PARAMS */
float wheel_radius = 0.1;
float d = 0.36;

/* DEFINE SUB PUB */
rcl_subscription_t subscriber_cmd_vel;
rcl_subscription_t subscriber_int;

// rcl_publisher_t publisher_u1;
// rcl_publisher_t publisher_u2;
// rcl_publisher_t publisher_u3;

// std_msgs__msg__Float32 msg_float_u1;
// std_msgs__msg__Float32 msg_float_u2;
// std_msgs__msg__Float32 msg_float_u3;

geometry_msgs__msg__Twist msg;
std_msgs__msg__Int32 msg_int;

/* FUNCTIONS */

// PUBLISHER CALLBACKS
// void timer_callback_u1(rcl_timer_t * timerx, int64_t last_call_time)
// {
// 	rcl_publish(&publisher_u1, &msg_float_u1, NULL);
// }

// void timer_callback_u2(rcl_timer_t * timery, int64_t last_call_time)
// {
// 	rcl_publish(&publisher_u2, &msg_float_u2, NULL);
// }

// void timer_callback_u3(rcl_timer_t * timerz, int64_t last_call_time)
// {
// 	rcl_publish(&publisher_u3, &msg_float_u3, NULL);
// }

//USER_DEFINED FUNCTIONS
float mapPwm(float x, float out_min, float out_max)  
{
    x = (x<0)? -x : x ;
    if(x==0){
        return 0;
    }
    else if(x>0){
        if(out_max*((x+min_speed)/max_speed) <= PWMRANGE){
            return out_max*((x+min_speed)/max_speed);
        }
        else{
            return PWMRANGE;
        }
    }
}

void pwm_write(int pin,int pwm) {
    pwm_clear_irq(pwm_gpio_to_slice_num(pin));
    pwm_set_gpio_level(pin, pwm);
}

void stop(){
    pwm_write(S_1_PWM, 0);
    pwm_write(S_2_PWM, 0);
    pwm_write(S_3_PWM, 0);
    

    gpio_put(G_1_PWM, 0);
    gpio_put(G_1, 0);
    gpio_put(G_2_PWM, 0);
    gpio_put(G_2, 0);
    gpio_put(G_3_PWM, 0);
    gpio_put(G_3, 0);
}

void stepper(int s, int dir, int pwm){
    switch(s){
        case 1:
            if(dir == 1){
                // high s1
                gpio_put(S_1, 1);
                pwm_write(S_1_PWM, pwm);
            }
            else if(dir == 0){
                //low s1
                gpio_put(S_1, 0);
                pwm_write(S_1_PWM, pwm);
            }
            else {
                stop();
            }
            break;
        case 2:
            if(dir == 1){
                // high s2
                gpio_put(S_2, 1);
                pwm_write(S_2_PWM, pwm);
            }
            else if(dir == 0){
                //low s2
                gpio_put(S_2, 0);
                pwm_write(S_2_PWM, pwm);
            }
            else {
                stop();
            }
            break;
        case 3:
            if(dir == 1){
                // high s3
                gpio_put(S_3, 1);
                pwm_write(S_3_PWM, pwm);
            }
            else if(dir == 0){
                //low s3
                gpio_put(S_3, 0);
                pwm_write(S_3_PWM, pwm);
            }
            else {
                stop();
            }
            break;
        default:
            stop();
    }
}

void gripper(int g, int dir, int pwm){
    switch(g){
        case 1:
            if(dir == 1){
                // open g1
                gpio_put(G_1, 1);
                gpio_put(G_1_PWM, 0);
                
            }
            else if(dir == 0){
                //close g1
                gpio_put(G_1_PWM, 1);
                gpio_put(G_1, 0);
                
            }
            else {
                gpio_put(G_1, 0);
            gpio_put(G_1_PWM, 0);
            gpio_put(G_2, 0);
            gpio_put(G_2_PWM, 0);
            gpio_put(G_3, 0);
            gpio_put(G_3_PWM, 0);
            }
            break;
        case 2:
            if(dir == 1){
                // open g2
                gpio_put(G_2, 1);
                gpio_put(G_2_PWM, 0);
            }
            else if(dir == 0){
                //close g2
                gpio_put(G_2_PWM, 1);
                gpio_put(G_2, 0);
            }
            else {
            gpio_put(G_1, 0);
            gpio_put(G_1_PWM, 0);
            gpio_put(G_2, 0);
            gpio_put(G_2_PWM, 0);
            gpio_put(G_3, 0);
            gpio_put(G_3_PWM, 0);
            }
            break;
        case 3:
            if(dir == 1){
                // open g3
                gpio_put(G_3, 1);
                gpio_put(G_3_PWM, 0);
                
            }
            else if(dir == 0){
                //close g3
                gpio_put(G_3_PWM, 1);
                gpio_put(G_3, 0);
                
            }
            else {
                gpio_put(G_1, 0);
            gpio_put(G_1_PWM, 0);
            gpio_put(G_2, 0);
            gpio_put(G_2_PWM, 0);
            gpio_put(G_3, 0);
            gpio_put(G_3_PWM, 0);
                
            }
            break;
        default:
            gpio_put(G_1, 0);
            gpio_put(G_1_PWM, 0);
            gpio_put(G_2, 0);
            gpio_put(G_2_PWM, 0);
            gpio_put(G_3, 0);
            gpio_put(G_3_PWM, 0);
            gpio_put(LED_PIN, 0);
            break;
    }
}

// SUBSCRIBER CALLBACK FUNCTIONS
void velocityCb(const void * msgin){
    const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;

    float u1 = (d*(msg->angular.z) - (msg->linear.y/2) + (msg->linear.x)*sin(PI/3))/wheel_radius;
    float u2 = (d*(msg->angular.z) - (msg->linear.y/2) - (msg->linear.x)*sin(PI/3))/wheel_radius;
    float u3 = (d*(msg->angular.z) + (msg->linear.y))/wheel_radius;
 
    // Then map those values to PWM intensities. PWMRANGE = full speed, while PWM_MIN = the minimal amount of power at which the motors begin moving.
    u1Pwm = mapPwm(u1, PWM_MIN, PWMRANGE);
    u2Pwm = mapPwm(u2, PWM_MIN, PWMRANGE);
    u3Pwm = mapPwm(u3, PWM_MIN, PWMRANGE);

    // Set direction pins and PWM
    gpio_put(IN_1, ((u1>0) ? 0 : 1)); //LF 
    pwm_write(ENA, u1Pwm);
    gpio_put(IN_2, ((u2>0) ? 1 : 0)); //LB
    pwm_write(ENB, u2Pwm);
    gpio_put(IN_3, ((u3>0) ? 0 : 1)); //RF
    pwm_write(ENC, u3Pwm);
    
    // msg_float_u1.data = u1Pwm;
    // msg_float_u2.data = u2Pwm;
    // msg_float_u3.data = u3Pwm;  
}

void intCb(const void * msgin){

    /* 5 digit received number
    digit               use                
    5          stepper/gripper number      1, 2, 3 - step/grip, 4 = idle
    4              up                      1 - on               0- idle
    3             down                     1 - on               0- idle
    2             open                     1 - on               0- idle
    1             close                    1 - on               0- idle
    
    int *ptr = parse_int(r_num);
    int num=ptr[4], up=ptr[3], down=ptr[2], open=ptr[1], close=ptr[0];
    
    11 buttons
        S_1, G_1 axis[7]  1
        S_2, G_2 axis[6]  1
        S_3, G_3 axis[6] -1

        A B X Y L1 R1 SELECT START HOME J1 J2
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    */
    const std_msgs__msg__Int32 * state = (const std_msgs__msg__Int32 *)msgin;
    int r_num = state->data;

    switch(r_num)
    {
    case 10010:
        gripper(1,1,pwm_val);
        gpio_put(LED_PIN, 1); 
        break;
    case 10001:
        gripper(1,0,pwm_val);
        gpio_put(LED_PIN, 1); 
        break;

    case 20010:
        gripper(2,1,pwm_val);
        gpio_put(LED_PIN, 1); 
        break;
    case 20001:
        gripper(2,0,pwm_val);
        gpio_put(LED_PIN, 1); 
        break;

    case 30010:
        gripper(3,1,pwm_val); 
        gpio_put(LED_PIN, 1); 
        break;
    case 30001:
        gripper(3,0,pwm_val);
        gpio_put(LED_PIN, 1); 
        break;

    case 11000:
        stepper(1,1,pwm_val);
        gpio_put(LED_PIN, 1); 
        break;
    case 10100:
        stepper(1,0,pwm_val); 
        gpio_put(LED_PIN, 1);  
        break; 

    case 21000:
        stepper(2,1,pwm_val);
        gpio_put(LED_PIN, 1); 
        break;
    case 20100:
        stepper(2,0,pwm_val);  
        gpio_put(LED_PIN, 1); 
        break;  

    case 31000:
        stepper(3,1,pwm_val);
        gpio_put(LED_PIN, 1); 
        break;
    case 30100:
        stepper(3,0,pwm_val);
        gpio_put(LED_PIN, 1);   
        break;
    case 11010:
        stepper(1,1,pwm_val);
        gripper(1,1,pwm_val);
         gpio_put(LED_PIN, 1);    
        break;
    case 11001:
        stepper(1,1,pwm_val);
        gripper(1,0,pwm_val);
        gpio_put(LED_PIN, 1);
        break;
    case 10110:
        stepper(1,0,pwm_val);
        gripper(1,1,pwm_val); 
        gpio_put(LED_PIN, 1); 
        break;  
    case 10101:
        stepper(1,0,pwm_val);
        gripper(1,0,pwm_val);  
        gpio_put(LED_PIN,pwm_val);
        break;  

    case 21010:
        stepper(2,1,pwm_val);
        gripper(2,1,pwm_val);
         gpio_put(LED_PIN, 1);    
        break;
    case 21001:
        stepper(2,1,pwm_val);
        gripper(2,0,pwm_val);
        gpio_put(LED_PIN, 1);
        break;
    case 20110:
        stepper(2,0,pwm_val);
        gripper(2,1,pwm_val); 
       gpio_put(LED_PIN, 1); 
        break;  
    case 20101:
        stepper(2,0,pwm_val);
        gripper(2,0,pwm_val);  
       gpio_put(LED_PIN,pwm_val);
        break;

    case 31010:
        stepper(3,1,pwm_val);
        gripper(3,1,pwm_val);
       gpio_put(LED_PIN, 1);    
        break;
    case 31001:
        stepper(3,1,pwm_val);
        gripper(3,0,pwm_val);
       gpio_put(LED_PIN, 1);
        break;
    case 30110:
        stepper(3,0,pwm_val);
        gripper(3,1,pwm_val); 
        gpio_put(LED_PIN, 1); 
        break;  
    case 30101:
        stepper(3,0,pwm_val);
        gripper(3,0,pwm_val);  
        gpio_put(LED_PIN, 1);
        break;

    default:
        gpio_put(LED_PIN, 0); 
        stop();
        break;
    }
    
}

int main() {
    stdio_init_all();

    /* PIN CONFIGURATIONS PWM & GPIO */
    gpio_set_function(ENA, GPIO_FUNC_PWM);
    gpio_set_function(ENB, GPIO_FUNC_PWM);
    gpio_set_function(ENC, GPIO_FUNC_PWM); 
    gpio_set_function(S_1_PWM, GPIO_FUNC_PWM); 
    gpio_set_function(S_2_PWM, GPIO_FUNC_PWM); 
    gpio_set_function(S_3_PWM, GPIO_FUNC_PWM); 


    uint slice_num_1 = pwm_gpio_to_slice_num(ENA);
    uint slice_num_2 = pwm_gpio_to_slice_num(ENB);
    uint slice_num_3 = pwm_gpio_to_slice_num(ENC);
    uint slice_num_4 = pwm_gpio_to_slice_num(S_1_PWM);
    uint slice_num_5 = pwm_gpio_to_slice_num(S_2_PWM);
    uint slice_num_6 = pwm_gpio_to_slice_num(S_3_PWM);

    pwm_clear_irq(slice_num_1);
    pwm_clear_irq(slice_num_2);
    pwm_clear_irq(slice_num_3);
    pwm_clear_irq(slice_num_4);
    pwm_clear_irq(slice_num_5);
    pwm_clear_irq(slice_num_6);

    pwm_set_irq_enabled(slice_num_1, true);
    pwm_set_irq_enabled(slice_num_2, true);
    pwm_set_irq_enabled(slice_num_3, true);
    pwm_set_irq_enabled(slice_num_4, true);
    pwm_set_irq_enabled(slice_num_5, true);
    pwm_set_irq_enabled(slice_num_6, true);

    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 4.f);

    // Setup Pins for PWM
    pwm_init(slice_num_1, &config, true);
    pwm_init(slice_num_2, &config, true);
    pwm_init(slice_num_3, &config, true);
    pwm_init(slice_num_4, &config, true);
    pwm_init(slice_num_5, &config, true);
    pwm_init(slice_num_6, &config, true);


    // Setup Pins for GP i/o
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
	gpio_init(IN_1);
    gpio_set_dir(IN_1, GPIO_OUT);
    gpio_init(IN_2);
    gpio_set_dir(IN_2, GPIO_OUT);
    gpio_init(IN_3);
    gpio_set_dir(IN_3, GPIO_OUT);
    
	gpio_init(S_1);
    gpio_set_dir(S_1, GPIO_OUT);
    gpio_init(S_2);
    gpio_set_dir(S_2, GPIO_OUT);
    gpio_init(S_3);
    gpio_set_dir(S_3, GPIO_OUT);

	gpio_init(G_1);
    gpio_set_dir(G_1, GPIO_OUT);
    gpio_init(G_2);
    gpio_set_dir(G_2, GPIO_OUT);
    gpio_init(G_3);
    gpio_set_dir(G_3, GPIO_OUT);

	gpio_init(G_1_PWM);
    gpio_set_dir(G_1_PWM, GPIO_OUT);
    gpio_init(G_2_PWM);
    gpio_set_dir(G_2_PWM, GPIO_OUT);
    gpio_init(G_3_PWM);
    gpio_set_dir(G_3_PWM, GPIO_OUT);

    // NODE TIMER ALLOCATOR SUPPORT EXECUTOR
    rcl_node_t node;

    // rcl_timer_t timerx;
    // rcl_timer_t timery;
    // rcl_timer_t timerz;

    rcl_allocator_t allocator;
    rclc_support_t support;
    rclc_executor_t executor;

    allocator = rcl_get_default_allocator();

    // Check if micro-ROS Agent answers to micro-ROS client
    rmw_uros_set_custom_transport(
        true,
        NULL,
        pico_serial_transport_open,
        pico_serial_transport_close,
        pico_serial_transport_write,
        pico_serial_transport_read
    );

    // Wait for agent successful ping for 2 minutes.
    const int timeout_ms = 1000; 
    const uint8_t attempts = 120;
    rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);
    if (ret != RCL_RET_OK){return ret;} // Unreachable agent, exiting program.

    //create init_options
    rclc_support_init(&support, 0, NULL, &allocator);

    // create node
    rclc_node_init_default(&node, "cmd_vel_node", "", &support);

    // create subscriber - cmd_vel
    rclc_subscription_init_default(
        &subscriber_cmd_vel,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel");

    // create subscriber - int_state
    rclc_subscription_init_default(
        &subscriber_int,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "int_state");

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

    // //timer callback for each publisher
	// rclc_timer_init_default(
	// 	&timerx,
	// 	&support,
	// 	RCL_MS_TO_NS(10),
	// 	timer_callback_u1);

    // rclc_timer_init_default(
	// 	&timery,
	// 	&support,
	// 	RCL_MS_TO_NS(10),
	// 	timer_callback_u2);

    // rclc_timer_init_default(
	// 	&timerz,
	// 	&support,
	// 	RCL_MS_TO_NS(10),
	// 	timer_callback_u3);    
   
    // create executor
    rclc_executor_init(&executor, &support.context, 8, &allocator);
    // rclc_executor_add_timer(&executor, &timerx);
    // rclc_executor_add_timer(&executor, &timery);
    // rclc_executor_add_timer(&executor, &timerz);
    rclc_executor_add_subscription(&executor, &subscriber_cmd_vel, &msg, &velocityCb, ON_NEW_DATA);
    rclc_executor_add_subscription(&executor, &subscriber_int, &msg, &intCb, ON_NEW_DATA);    

    //gpio_put(LED_PIN, 1);

    // msg_float_u1.data = 0;
    // msg_float_u2.data = 0;
    // msg_float_u3.data = 0;  

    while (true)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }
    
    return 0;

}