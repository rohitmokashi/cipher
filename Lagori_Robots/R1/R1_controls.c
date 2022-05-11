#include <stdio.h>    
#include <math.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include "pico/stdlib.h"
#include "pico_uart_transports.h"

#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"

#include "lagori_robot_msgs/msg/robot_one_controls.h"

rcl_subscription_t robot_controls_subscriber;
lagori_robot_msgs__msg__RobotOneControls robot_controls_msgs;

/* DEFINE */
#define PWM_FEEDER_LEFT 
#define PWM_FEEDER_RIGHT 

#define PWM_ANGLE_LEFT 
#define PWM_ANGLE_RIGHT 

#define IN_ROTOR 
#define PWM_ROTOR 

#define SERVO_BTN 

#define DOWN_SWITCH 
#define UP_SWITCH 

/* CONFIGURATIONS */
int rotor_pwm = 50000;
int feeder_pwm = 11500;
int angle_pwm = 18500;

// 0 to max 65025.
int PWM_MIN = 10;
int PWMRANGE = 50000;
int min_speed = 4;
int max_speed = 31.42;
bool running;
const uint LED_PIN = 25;

/* FUNCTIONS */
void pwm_write(int pin,int pwm) {
    pwm_clear_irq(pwm_gpio_to_slice_num(pin));
    pwm_set_gpio_level(pin, pwm);
}

void shoot(){
    feeder_down();//feeder down to initial position
    sleep(100);
    servo();
    sleep_ms(500);
    rotors_on();
    sleep_ms(500);
    feeder_up();
    rotors_off();
    sleep_ms(500);
}
void feeder_up(){
    if(gpio_get(UP_SWITCH)!=0){
        gpio_put(PWM_FEEDER_RIGHT, 1);
        gpio_put(LED_PIN, 1);
    }
    else{
        gpio_put(PWM_FEEDER_RIGHT, 0);
        gpio_put(PWM_FEEDER_LEFT, 0);
        gpio_put(LED_PIN, 0);
        sleep_ms(50);
    }
}
void feeder_down(){    
    if(gpio_get(DOWN_SWITCH)!=0){
        gpio_put(PWM_FEEDER_RIGHT, 0);
        gpio_put(PWM_FEEDER_LEFT, 1);
        gpio_put(LED_PIN, 1);
    }
    else{
        gpio_put(PWM_FEEDER_LEFT, 0);
        gpio_put(PWM_FEEDER_RIGHT, 0);
        gpio_put(LED_PIN, 0);          
        sleep_ms(50);
    }
}
void rotors_on(){
    if(!running){
        for(int i=0; i<=rotor_pwm;i++){    
            pwm_write(PWM_ROTOR, i);       
        }    
        gpio_put(LED_PIN, 1);
        running = true;
        sleep_ms(200);
    }
    else {
        pwm_write(PWM_ROTOR, rotor_pwm);
        gpio_put(LED_PIN, 1); 
    }  
}
void rotors_off(){
    for(int i=rotor_pwm; i>=0; i--){
        pwm_write(PWM_ROTOR, i);
    }
    gpio_put(LED_PIN, 1);
    running = false;
}
void angle_down(){
    if(gpio_get(DOWN_SWITCH)!=0){
        gpio_put(PWM_ANGLE_LEFT, 0);
        gpio_put(PWM_ANGLE_RIGHT, 1);
        gpio_put(LED_PIN, 1);
    }else{
        gpio_put(PWM_ANGLE_LEFT, 0);
        gpio_put(PWM_ANGLE_RIGHT, 0);
    }    
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

void r1_controls_callback(const void * msgin){
    const lagori_robot_msgs__msg__RobotOneControls * msg = (const lagori_robot_msgs__msg__RobotOneControls *)msgin;
}

int main() {
    stdio_init_all();

    gpio_set_function(PWM_ROTOR, GPIO_FUNC_PWM);

    uint slice_num_1 = pwm_gpio_to_slice_num(PWM_ROTOR);            

    pwm_clear_irq(slice_num_1);

    pwm_set_irq_enabled(slice_num_1, true);

    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 4.f);

    pwm_init(slice_num_1, &config, true);

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

    running = false;

    // create subscriber - r1_controls
    rclc_subscription_init_default(
        &robot_controls_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(),
        "r1_controls");

    // create executor
    rclc_executor_init(&executor, &support.context, 6, &allocator);
    rclc_executor_add_subscription(&executor, &robot_controls_subscriber, &robot_controls_msgs, &r1_controls_callback, ON_NEW_DATA);

    while (true){
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
    }
    return 0;
}