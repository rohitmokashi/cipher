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
//#include <std_msgs/msg/float32.h>
//#include <std_msgs/msg/int32.h>

// Custom Messages
#include <lagori_robot_msgs/msg/gripper_msg.h>
#include <lagori_robot_msgs/msg/stepper_msg.h>

#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"


/* DEFINE */
#define PI 3.14159265
#define ENA //15               
#define ENB //14               
#define ENC //13                  
#define IN_1 //12              
#define IN_2 //11               
#define IN_3 //10                

#define S_1 //16
#define S_1_PWM //17 
#define S_2 //18
#define S_2_PWM //19
#define S_3 //20
#define S_3_PWM //21 

#define G_1 //9
#define G_1_PWM //8
#define G_2 //7
#define G_2_PWM //6
#define G_3 //5
#define G_3_PWM //4

// 0 to 65025

/* Configurations */
int PWM_MIN = 10;
int PWMRANGE = 65025;
int min_speed = 1;
int max_speed = 31.42;

#define pwm_val 55000

const uint LED_PIN = 25;

uint32_t u1Pwm, u2Pwm, u3Pwm;

/* Robots Params */
float wheel_radius = 0.1;
float d = 0.36;

/* Define Sub Pub */
rcl_subscription_t cmd_vel_sub;
rcl_subscription_t gripper_sub;
rcl_subscription_t stepper_sub;

geometry_msgs__msg__Twist cmd_vel_msg;
lagori_robot_msgs__msg__GripperMsg gripper_msg;
lagori_robot_msgs__msg__StepperMsg stepper_msg;

// rcl_publisher_t publisher_u1;
// rcl_publisher_t publisher_u2;
// rcl_publisher_t publisher_u3;

// std_msgs__msg__Float32 msg_float_u1;
// std_msgs__msg__Float32 msg_float_u2;
// std_msgs__msg__Float32 msg_float_u3;

/* Publisher Callbacks */

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

// void timer_callback_check(rcl_timer_t * timer_check, int64_t last_call_time)
// {
// 	rcl_publish(&publisher_check, &int_check, NULL);
// }

// User Defined Functions
float mapPwm(float x, float out_min, float out_max)  
{
    x = (x<0)? -x : x ;
    if(x==0){
        return 0;
    }
    else {
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

void idle(){}

/* dir 1 = up/open, dir 0 = down/close */
void stepper1(int dir){}
void stepper2(int dir){}
void stepper3(int dir){}

void gripper1(int dir){}
void gripper2(int dir){}
void gripper3(int dir){}

/* Subscriber Callbacks */
void velocityCb(const void * msgin){
    const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;

    float u1 = (d*(msg->angular.z) - (msg->linear.y/2) + (msg->linear.x)*sin(PI/3))/wheel_radius;
    float u2 = (d*(msg->angular.z) - (msg->linear.y/2) - (msg->linear.x)*sin(PI/3))/wheel_radius;
    float u3 = (d*(msg->angular.z) + (msg->linear.y))/wheel_radius;
 
    // Then map those values to PWM intensities. PWMRANGE = full speed, while PWM_MIN = the minimal amount of power
    u1Pwm = mapPwm(u1, PWM_MIN, PWMRANGE);
    u2Pwm = mapPwm(u2, PWM_MIN, PWMRANGE);
    u3Pwm = mapPwm(u3, PWM_MIN, PWMRANGE);

    // Set direction pins and PWM
    gpio_put(IN_1, ((u1>0) ? 0 : 1)); 
    pwm_write(ENA, u1Pwm);
    gpio_put(IN_2, ((u2>0) ? 1 : 0)); 
    pwm_write(ENB, u2Pwm);
    gpio_put(IN_3, ((u3>0) ? 0 : 1)); 
    pwm_write(ENC, u3Pwm);
    
    //debug purpose
    // msg_float_u1.data = u1Pwm;
    // msg_float_u2.data = u2Pwm;
    // msg_float_u3.data = u3Pwm;  
}

void gripperCb(const void * msgin){
    const lagori_robot_msgs__msg__GripperMsg * msg = (const lagori_robot_msgs__msg__GripperMsg *)msgin;
    //declare array of 1 2 3 gripper 
    //on button press 1, else 0
    int one[2] = {msg->one_open, msg->one_close};
    int two[2] = {msg->two_open, msg->two_close};
    int three[2] = {msg->three_open, msg->three_close};

    (one[0] == 1) ? gripper1(1) : (one[1] == 1) ? gripper1(0) : idle();

    (two[0] == 1) ? gripper2(1) : (two[1] == 1) ? gripper2(0) : idle();

    (three[0] == 1) ? gripper3(1) : (three[1] == 1) ? gripper3(0) : idle();    
}

void stepperCb(const void * msgin){
    const lagori_robot_msgs__msg__StepperMsg * msg = (const lagori_robot_msgs__msg__StepperMsg *)msgin;
    //declare array of 1 2 3 stepper
    //on button press 1, else 0
    int one[2] = {msg->one_up, msg->one_down};
    int two[2] = {msg->two_up, msg->two_down};
    int three[2] = {msg->three_up, msg->three_down};

    (one[0] == 1) ? stepper1(1) : (one[1] == 1) ? stepper1(0) : idle();

    (two[0] == 1) ? stepper2(1) : (two[1] == 1) ? stepper2(0) : idle();

    (three[0] == 1) ? stepper3(1) : (three[1] == 1) ? stepper3(0) : idle();
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

    //creating node, allocator, support, executor 
    rcl_node_t node;
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

    // create subscriber - cmd_vel_sub
    rclc_subscription_init_default(
        &cmd_vel_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel");

    // create subscriber - gripper_sub
    rclc_subscription_init_default(
        &gripper_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(lagori_robot_msgs, msg, GripperMsg),
        "r2_gripper");

    // create subscriber - stepper_sub
    rclc_subscription_init_default(
        &stepper_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(lagori_robot_msgs, msg, StepperMsg),
        "r2_stepper");    


    rclc_executor_init(&executor, &support.context, 8, &allocator);
    rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel_msg, &velocityCb, ON_NEW_DATA);
    rclc_executor_add_subscription(&executor, &gripper_sub, &gripper_msg, &gripperCb, ON_NEW_DATA);  
    rclc_executor_add_subscription(&executor, &stepper_sub, &stepper_msg, &stepperCb, ON_NEW_DATA);    

    //gpio_put(LED_PIN, 1);
    // msg_float_u1.data = 0;
    // msg_float_u2.data = 0;
    // msg_float_u3.data = 0;  
    //run the executor in loop 
    
    while (true)
    {
        // spins every 1 ms, 0.001 s, frequency = 100 Hz
        rclc_executor_spin_some(&executor, RCL_S_TO_NS(1)); 
    }
    return 0;
}



/* Unnecessary */
/*
const std_msgs__msg__Int32 * state = (const std_msgs__msg__Int32 *)msgin;
    int r_num = state->data;

    switch(r_num)
    {
    case 10010:
        gripper(1,1,pwm_val);
        //gpio_put(LED_PIN, 1); 
        break;
    case 10001:
        gripper(1,0,pwm_val);
        //gpio_put(LED_PIN, 1); 
        break;

    case 20010:
        gripper(2,1,pwm_val);
        //gpio_put(LED_PIN, 1); 
        break;
    case 20001:
        gripper(2,0,pwm_val);
        //gpio_put(LED_PIN, 1); 
        break;

    case 30010:
        gripper(3,1,pwm_val); 
        //gpio_put(LED_PIN, 1); 
        break;
    case 30001:
        gripper(3,0,pwm_val);
        //gpio_put(LED_PIN, 1); 
        break;

    case 11000:
        stepper(1,1,pwm_val);
        //gpio_put(LED_PIN, 1); 
        break;
    case 10100:
        stepper(1,0,pwm_val); 
        //gpio_put(LED_PIN, 1);  
        break; 

    case 21000:
        stepper(2,1,pwm_val);
        //gpio_put(LED_PIN, 1); 
        break;
    case 20100:
        stepper(2,0,pwm_val);  
        gpio_put(LED_PIN, 1); 
        break;  

    case 31000:
        stepper(3,1,pwm_val);
        //gpio_put(LED_PIN, 1); 
        break;
    case 30100:
        stepper(3,0,pwm_val);
        gpio_put(LED_PIN, 1);   
        break;
    case 11010:
        stepper(1,1,pwm_val);
        gripper(1,1,pwm_val);
         //gpio_put(LED_PIN, 1);    
        break;
    case 11001:
        stepper(1,1,pwm_val);
        gripper(1,0,pwm_val);
        //gpio_put(LED_PIN, 1);
        break;
    case 10110:
        stepper(1,0,pwm_val);
        gripper(1,1,pwm_val); 
        //gpio_put(LED_PIN, 1); 
        break;  
    case 10101:
        stepper(1,0,pwm_val);
        gripper(1,0,pwm_val);  
        //gpio_put(LED_PIN,pwm_val);
        break;  

    case 21010:
        stepper(2,1,pwm_val);
        gripper(2,1,pwm_val);
         //gpio_put(LED_PIN, 1);    
        break;
    case 21001:
        stepper(2,1,pwm_val);
        gripper(2,0,pwm_val);
        //gpio_put(LED_PIN, 1);
        break;
    case 20110:
        stepper(2,0,pwm_val);
        gripper(2,1,pwm_val); 
       //gpio_put(LED_PIN, 1); 
        break;  
    case 20101:
        stepper(2,0,pwm_val);
        gripper(2,0,pwm_val);  
       //gpio_put(LED_PIN,pwm_val);
        break;

    case 31010:
        stepper(3,1,pwm_val);
        gripper(3,1,pwm_val);
       //gpio_put(LED_PIN, 1);    
        break;
    case 31001:
        stepper(3,1,pwm_val);
        gripper(3,0,pwm_val);
       //gpio_put(LED_PIN, 1);
        break;
    case 30110:
        stepper(3,0,pwm_val);
        gripper(3,1,pwm_val); 
        //gpio_put(LED_PIN, 1); 
        break;  
    case 30101:
        stepper(3,0,pwm_val);
        gripper(3,0,pwm_val);  
        //gpio_put(LED_PIN, 1);
        break;

    default:
        gpio_put(LED_PIN, 0); 
        stop();
        break;
    }


///////////////////////////////////////////////////////////
void stepper(int s, int dir, int pwm){
    switch(s){
        case 1:
            if(dir == 1){
                // high s1
                gpio_put(S_1, 1);
                pwm_write(S_1_PWM, pwm);
                gpio_put(LED_PIN, 1);
            }
            else if(dir == 0){
                //low s1
                if(gpio_get(U1_PIN) !=0){
                    gpio_put(S_1, 0);
                    pwm_write(S_1_PWM, pwm);
                    gpio_put(LED_PIN, 1);
                }else {
                    pwm_write(S_1_PWM, 0);
                    gpio_put(LED_PIN, 0);
                }   
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
                gpio_put(LED_PIN, 1);
            }
            else if(dir == 0){
                //low s2
                gpio_put(S_2, 0);
                pwm_write(S_2_PWM, pwm);
                gpio_put(LED_PIN, 1); 
            }
            else {
                stop();
                gpio_put(LED_PIN, 0);
            }
            break;
        case 3:
            if(dir == 1){
                // high s3
                gpio_put(S_3, 1);
                pwm_write(S_3_PWM, pwm);
                gpio_put(LED_PIN, 1);
            }
            else if(dir == 0){
                //low s3
                
                    gpio_put(S_3, 0);
                    pwm_write(S_3_PWM, pwm);
                    gpio_put(LED_PIN, 1);
                
            }
            else {
                stop();
                gpio_put(LED_PIN, 0);
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
                // close g1
                gpio_put(G_1, 1);
                gpio_put(G_1_PWM, 0);
                gpio_put(LED_PIN, 1);      
            }
            else if(dir == 0){
                //open g1
                if(gpio_get(R1_PIN) !=0){
                    gpio_put(G_1_PWM, 1);
                    gpio_put(G_1, 0);
                    gpio_put(LED_PIN, 1);
                }else{
                    gpio_put(G_1, 0);
                    gpio_put(G_1_PWM, 0);
                    gpio_put(LED_PIN, 0);
                    sleep_ms(50);
                }
            }
            else {
                gpio_put(G_1, 0);
                gpio_put(G_1_PWM, 0);
                gpio_put(G_2, 0);
                gpio_put(G_2_PWM, 0);
                gpio_put(G_3, 0);
                gpio_put(G_3_PWM, 0);
                gpio_put(LED_PIN, 0);
            }
            break;
        case 2:
            if(dir == 1){
                // close g2
                gpio_put(G_2, 1);
                gpio_put(G_2_PWM, 0);
                gpio_put(LED_PIN, 1);
            }
            else if(dir == 0){
                //open g2
                if(gpio_get(R2_PIN) !=0){
                    gpio_put(G_2_PWM, 1);
                    gpio_put(G_2, 0);
                    gpio_put(LED_PIN, 1);
                }else {
                    gpio_put(G_2, 0);
                    gpio_put(G_2_PWM, 0);
                    gpio_put(LED_PIN, 0);
                    sleep_ms(50);
                }    
            }
            else {
                gpio_put(G_1, 0);
                gpio_put(G_1_PWM, 0);
                gpio_put(G_2, 0);
                gpio_put(G_2_PWM, 0);
                gpio_put(G_3, 0);
                gpio_put(G_3_PWM, 0);
                gpio_put(LED_PIN, 0);
            }
            break;
        case 3:
            if(dir == 1){
                // close g3
                gpio_put(G_3, 1);
                gpio_put(G_3_PWM, 0);
                gpio_put(LED_PIN, 1);
            }
            else if(dir == 0){
                //open g3
                if(gpio_get(R3_PIN) !=0){
                    gpio_put(G_3_PWM, 1);
                    gpio_put(G_3, 0);
                    gpio_put(LED_PIN, 1);
                }else {
                    gpio_put(G_3, 0);
                    gpio_put(G_3_PWM, 0);
                    gpio_put(LED_PIN, 0);
                    sleep_ms(50);
                }   
            }
            else {
                gpio_put(G_1, 0);
                gpio_put(G_1_PWM, 0);
                gpio_put(G_2, 0);
                gpio_put(G_2_PWM, 0);
                gpio_put(G_3, 0);
                gpio_put(G_3_PWM, 0);
                gpio_put(LED_PIN, 0);
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
*/