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

/*
M1      M2
  |    |

  |    |
M3      M4
*/


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

#define PWM_FEEDER_LEFT 17
#define PWM_FEEDER_RIGHT 16

#define PWM_ANGLE_LEFT 19
#define PWM_ANGLE_RIGHT 18

#define IN_ROTOR 20
#define PWM_ROTOR 21

#define DOWN_SWITCH 0
#define UP_SWITCH 1

#define SERVO_BTN 3

/* angle, rollers, feeder, flap */

/* CONFIGURATIONS */
int rotor_pwm = 65000;
int feeder_pwm = 15000;
int angle_pwm = 20000;

// 0 to max 65025.
int PWM_MIN = 10;
int PWMRANGE = 50000;
int min_speed = 0;
int max_speed = 31.42;
bool run_once = false;
const uint LED_PIN = 25;

uint32_t u1Pwm, u2Pwm, u3Pwm, u4Pwm;

/* ROBOT PARAMS */
float l = 0.46;
float w = 0.375;
float wheel_radius = 0.1;


rcl_subscription_t subscriber;
rcl_subscription_t subscriber_twist;

// rcl_publisher_t publisher_u1;
// rcl_publisher_t publisher_u2;
// rcl_publisher_t publisher_u3;
// rcl_publisher_t publisher_u4;

// std_msgs__msg__Float32 msg_float_u1;
// std_msgs__msg__Float32 msg_float_u2;
// std_msgs__msg__Float32 msg_float_u3;
// std_msgs__msg__Float32 msg_float_u4;

geometry_msgs__msg__Twist msg;
std_msgs__msg__Int32 msg_int;

/* FUNCTIONS */
// void timer_callback_u1(rcl_timer_t * timer, int64_t last_call_time)
// {
// 	rcl_publish(&publisher_u1, &msg_float_u1, NULL);
// }
// void timer_callback_u2(rcl_timer_t * timer, int64_t last_call_time)
// {
// 	rcl_publish(&publisher_u2, &msg_float_u2, NULL);
// }
// void timer_callback_u3(rcl_timer_t * timer, int64_t last_call_time)
// {
// 	rcl_publish(&publisher_u3, &msg_float_u3, NULL);
// }
// void timer_callback_u4(rcl_timer_t * timer, int64_t last_call_time)
// {
// 	rcl_publish(&publisher_u4, &msg_float_u4, NULL);
// }

float mapPwm(float u, float out_min, float out_max)  
{
    u = abs(u);
    if(u==0){
        return 0;
    }
    else if(out_max*((u+min_speed)/max_speed) < PWMRANGE-1000){
        return out_max*((u+min_speed)/max_speed);
    }
    else{
      return PWMRANGE-1000;
    }
}
void pwm_write(int pin,int pwm) {
    pwm_clear_irq(pwm_gpio_to_slice_num(pin));
    pwm_set_gpio_level(pin, pwm);
}

float idle(){
	pwm_write(PWM_LEFT_1, 0);
	pwm_write(PWM_LEFT_2, 0);
	pwm_write(PWM_LEFT_3, 0);
	pwm_write(PWM_LEFT_4, 0);
	pwm_write(PWM_RIGHT_1, 0);
	pwm_write(PWM_RIGHT_2, 0);
	pwm_write(PWM_RIGHT_3, 0);
	pwm_write(PWM_RIGHT_4, 0);
    gpio_put(LED_PIN, 0);
} 
// forward = 1 , backward = -1, left = -2, right = -2, clocl = 3, anticlock = -3

void backward(int pwm1, int pwm2, int pwm3, int pwm4){
	pwm_write(PWM_LEFT_1, pwm1);
	pwm_write(PWM_LEFT_2, 0);
	pwm_write(PWM_LEFT_3, 0);
	pwm_write(PWM_LEFT_4, pwm4);
	pwm_write(PWM_RIGHT_1, 0);
	pwm_write(PWM_RIGHT_2, pwm2);
	pwm_write(PWM_RIGHT_3, pwm3);
	pwm_write(PWM_RIGHT_4, 0);
    gpio_put(LED_PIN, 1);

}

void forward(int pwm1, int pwm2, int pwm3, int pwm4){
	pwm_write(PWM_LEFT_1, 0);
	pwm_write(PWM_LEFT_2, pwm2);
	pwm_write(PWM_LEFT_3, pwm3);
	pwm_write(PWM_LEFT_4, 0);
	pwm_write(PWM_RIGHT_1, pwm1);
	pwm_write(PWM_RIGHT_2, 0);
	pwm_write(PWM_RIGHT_3, 0);
	pwm_write(PWM_RIGHT_4, pwm4);
    gpio_put(LED_PIN, 1);

}

void left(int pwm1, int pwm2, int pwm3, int pwm4){
	pwm_write(PWM_LEFT_1, pwm1);
	pwm_write(PWM_LEFT_2, pwm2);
	pwm_write(PWM_LEFT_3, pwm3);
	pwm_write(PWM_LEFT_4, pwm4);
	pwm_write(PWM_RIGHT_1, 0);
	pwm_write(PWM_RIGHT_2, 0);
	pwm_write(PWM_RIGHT_3, 0);
	pwm_write(PWM_RIGHT_4, 0);
    gpio_put(LED_PIN, 1);

}
void right(int pwm1, int pwm2, int pwm3, int pwm4){
	pwm_write(PWM_LEFT_1, 0);
	pwm_write(PWM_LEFT_2, 0);
	pwm_write(PWM_LEFT_3, 0);
	pwm_write(PWM_LEFT_4, 0);
	pwm_write(PWM_RIGHT_1, pwm1);
	pwm_write(PWM_RIGHT_2, pwm2);
	pwm_write(PWM_RIGHT_3, pwm3);
	pwm_write(PWM_RIGHT_4, pwm4);
    gpio_put(LED_PIN, 1);

}
void clock(int pwm1, int pwm2, int pwm3, int pwm4){
	pwm_write(PWM_LEFT_1, pwm1);
	pwm_write(PWM_LEFT_2, pwm2);
	pwm_write(PWM_LEFT_3, 0);
	pwm_write(PWM_LEFT_4, 0);
	pwm_write(PWM_RIGHT_1, 0);
	pwm_write(PWM_RIGHT_2, 0);
	pwm_write(PWM_RIGHT_3, pwm3);
	pwm_write(PWM_RIGHT_4, pwm4);
    gpio_put(LED_PIN, 1);

}
void anticlock(int pwm1, int pwm2, int pwm3, int pwm4){
	pwm_write(PWM_LEFT_1, 0);
	pwm_write(PWM_LEFT_2, 0);
	pwm_write(PWM_LEFT_3, pwm3);
	pwm_write(PWM_LEFT_4, pwm4);
	pwm_write(PWM_RIGHT_1, pwm1);
	pwm_write(PWM_RIGHT_2, pwm2);
	pwm_write(PWM_RIGHT_3, 0);
	pwm_write(PWM_RIGHT_4, 0);
    gpio_put(LED_PIN, 1);

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


    if(x==0){
      	if(y==0){
        	if(z==0){
          		idle();
        	}else if(z<0){
		  		//clock
				clock(u1Pwm, u2Pwm, u3Pwm, u4Pwm);
	    	}else if(z>0){
				//anticlock
				anticlock(u1Pwm, u2Pwm, u3Pwm, u4Pwm);
			}else{idle();}
		}else if(y<0){
			//left
			left(u1Pwm, u2Pwm, u3Pwm, u4Pwm);
		}else if(y>0){
			//right
			right(u1Pwm, u2Pwm, u3Pwm, u4Pwm);
		}else{idle();}	
    }else if(x>0){
		//forward
		forward(u1Pwm, u2Pwm, u3Pwm, u4Pwm);
	}else if(x<0){
		//backward
		backward(u1Pwm, u2Pwm, u3Pwm, u4Pwm);
	}else{idle();}

  	// msg_float_u1.data = u1;
  	// msg_float_u2.data = u2;
  	// msg_float_u3.data = u3;   
  	// msg_float_u4.data = u4;

}

void feeder(int i){

    do{
        if(gpio_get(DOWN_SWITCH) == 0){
            gpio_put(PWM_FEEDER_LEFT, 0);
            gpio_put(PWM_FEEDER_RIGHT, 1);
        }
        else if(gpio_get(UP_SWITCH) == 0){
            gpio_put(PWM_FEEDER_RIGHT, 0);
            gpio_put(PWM_ANGLE_LEFT, 1);
        }
    }while(gpio_get(DOWN_SWITCH) == 0);

}
void feeder_up(){
    
    if(gpio_get(UP_SWITCH)!=0){
        gpio_put(PWM_FEEDER_RIGHT, 1);
	    gpio_put(LED_PIN, 1);
    }
    else{
        gpio_put(LED_PIN, 0);
        gpio_put(PWM_FEEDER_RIGHT, 0);
        gpio_put(PWM_FEEDER_LEFT, 0);
        sleep_ms(500);
    }

}
void feeder_down(){    
    if(gpio_get(DOWN_SWITCH)!=0){
        gpio_put(PWM_FEEDER_RIGHT, 0);
        gpio_put(PWM_FEEDER_LEFT, 1);
	    gpio_put(LED_PIN, 1);
    }
    else{
        gpio_put(LED_PIN, 0);
        gpio_put(PWM_FEEDER_LEFT, 0);
        gpio_put(PWM_FEEDER_RIGHT, 0);
        sleep_ms(500);
    }
}
void rotors_on(){
    gpio_put(IN_ROTOR, 1);
    pwm_write(PWM_ROTOR, rotor_pwm);
	gpio_put(LED_PIN, 1);
}
void rotors_off(){
    pwm_write(PWM_ROTOR, 0);
	gpio_put(LED_PIN, 1);
}
void angle_down(){
    gpio_put(PWM_ANGLE_LEFT, 0);
    gpio_put(PWM_ANGLE_RIGHT, 1);
    gpio_put(LED_PIN, 1);
}
void angle_up(){
    gpio_put(PWM_ANGLE_RIGHT, 0);
    gpio_put(PWM_ANGLE_LEFT, 1);
    gpio_put(LED_PIN, 1);
}
void servo(){
    gpio_put(SERVO_BTN, 0);
	gpio_put(LED_PIN, 1);
}
void twistoCb(const void * msgin){
    const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
    int r_num = msg->data;
    
    /*
        angle up - 1 , down - 2       nothing - 3
        feeder up - 1 nothing - 0, 
        feeder down - 1 nothing 0,
        rotor on - 1, nothing - 0 
        rotor off - 1, nothing - 0
        servo on - 1, nothing - 0  
    */

    switch(r_num){
        //initial single run
        case 100000:
            //angle up
            angle_up();
            break;
        case 200000:
            //angle down
            angle_down();
            break;
        case 310000:
            //feeder up
            feeder_up();
            break;
        case 301000:
            //feeder down
            feeder_down();
            break;
        case 300100:
            //rotor on
            rotors_on();
            break;
        case 300010:
            //rotor off
            rotors_off();
            break;
        case 300001:
            //servo on off
            servo();
            break;

        //while running angle up
        case 110000:
            angle_up();
            feeder_up();
            //feeder up
            break;
        case 101000:
            angle_up();
            feeder_down();
            //feeder down
            break;
        case 100100:
            angle_up();
            rotors_on();
            //rotor on
            break;
        case 100010:
            angle_up();
            rotors_off();
            //rotor off
            break;
        case 100001:
            angle_up();
            servo();
            //servo
            break;

        //while running angle down
        case 210000:
            angle_down();
            feeder_up();
            //feeder up
            break;
        case 201000:
            angle_down();
            feeder_down();
            //feeder down
            break;
        case 200100:
            angle_down();
            rotors_on();
            //rotor on
            break;
        case 200010:
            angle_down();
            rotors_off();
            //rotor off
            break;
        case 200001:
            angle_down();
            servo();
            //servo
            break;

        //while running feeder up
        case 310100:
            feeder_up();
            rotors_on();
            //rotor on
            break;
        case 310010:
            feeder_up();
            rotors_off();
            //rotor off
            break;
        case 310001:
            feeder_up();
            servo();
            //servo
            break;

        //while running feeder down
        case 301100:
            feeder_down();
            rotors_on();
            //rotor on
            break;
        case 301010:
            feeder_down();
            rotors_off();
            //rotor off
            break;
        case 301001:
            feeder_down();
            servo();
            //servo 
            break;

        //while running rotor on
        case 300101:
            rotors_on();
            servo();
            //servo
            break;

        //while running rotor off
        case 300011:
            rotors_off();
            servo();
            //servo
            break;

        default:
            gpio_put(PWM_ANGLE_LEFT, 0);
            gpio_put(PWM_ANGLE_RIGHT, 0);
            pwm_write(PWM_ROTOR, 0);
            gpio_put(PWM_FEEDER_LEFT, 0);
            gpio_put(PWM_FEEDER_RIGHT, 0);
            gpio_put(SERVO_BTN, 1);
            gpio_put(LED_PIN, 0);
            break;                          
    }
}


int main() {
    stdio_init_all();

    gpio_set_function(PWM_LEFT_1, GPIO_FUNC_PWM);
    gpio_set_function(PWM_LEFT_2, GPIO_FUNC_PWM);
    gpio_set_function(PWM_LEFT_3, GPIO_FUNC_PWM);
    gpio_set_function(PWM_LEFT_4, GPIO_FUNC_PWM);
    gpio_set_function(PWM_RIGHT_1, GPIO_FUNC_PWM);
    gpio_set_function(PWM_RIGHT_2, GPIO_FUNC_PWM);
    gpio_set_function(PWM_RIGHT_3, GPIO_FUNC_PWM);
    gpio_set_function(PWM_RIGHT_4, GPIO_FUNC_PWM);
    gpio_set_function(PWM_ROTOR, GPIO_FUNC_PWM);

    uint slice_num_1 = pwm_gpio_to_slice_num(PWM_LEFT_1);
    uint slice_num_2 = pwm_gpio_to_slice_num(PWM_LEFT_2);
    uint slice_num_3 = pwm_gpio_to_slice_num(PWM_LEFT_3);
    uint slice_num_4 = pwm_gpio_to_slice_num(PWM_LEFT_4);
    uint slice_num_5 = pwm_gpio_to_slice_num(PWM_RIGHT_1);
    uint slice_num_6 = pwm_gpio_to_slice_num(PWM_RIGHT_2);
    uint slice_num_7 = pwm_gpio_to_slice_num(PWM_RIGHT_3);
    uint slice_num_8 = pwm_gpio_to_slice_num(PWM_RIGHT_4); 
    uint slice_num_9 = pwm_gpio_to_slice_num(PWM_ROTOR);            

    
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
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
    }
    
    return 0;
}