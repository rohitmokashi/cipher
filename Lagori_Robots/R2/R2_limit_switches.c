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

/* Define Pins of Limit switches */
//stepper 1
#define gripper1_open
#define gripper1_close
#define stepper1_level1
#define stepper1_level2
#define stepper1_level3
#define stepper1_level4
#define stepper1_level5

//setpper 2
#define gripper2_open
#define gripper2_close
#define stepper2_level1
#define stepper2_level2
#define stepper2_level3
#define stepper2_level4
#define stepper2_level5

//stepper 3
#define gripper3_open
#define gripper3_close
#define stepper3_level1
#define stepper3_level2
#define stepper3_level3
#define stepper3_level4
#define stepper3_level5

const uint LED_PIN = 25;

void pwm_write(int pin,int pwm) {
    pwm_clear_irq(pwm_gpio_to_slice_num(pin));
    pwm_set_gpio_level(pin, pwm);
}

if(gpio_get(SWITCH) == 0){ //NOT PRESSED
    }
else {// PRESSED

}
                           
int main() {
    stdio_init_all();

    /*gpio_set_function(ENA, GPIO_FUNC_PWM);
    uint slice_num_1 = pwm_gpio_to_slice_num(ENA);
    pwm_clear_irq(slice_num_1);
    pwm_set_irq_enabled(slice_num_1, true);
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 4.f);
    // Setup Pins for PWM
    pwm_init(slice_num_1, &config, true);*/

    /* PIN CONFIGURATIONS GPIO */
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, false);

    //Pins for S1
    gpio_init(gripper1_open);
    gpio_set_dir(gripper1_open, false);
    gpio_init(gripper1_close);
    gpio_set_dir(gripper1_close, false);
    gpio_init(stepper1_level1);
    gpio_set_dir(stepper1_level1, false);
    gpio_init(stepper1_level2);
    gpio_set_dir(stepper1_level2, false);
    gpio_init(stepper1_level3);
    gpio_set_dir(S1_3, false);
    gpio_init(stepper1_level4);
    gpio_set_dir(stepper1_level4, false);
    gpio_init(stepper1_level5);
    gpio_set_dir(stepper1_level5, false);
    
    //Pins for S2
    gpio_init(gripper2_open);
    gpio_set_dir(gripper2_open, false);
    gpio_init(gripper2_close);
    gpio_set_dir(gripper2_close, false);
    gpio_init(stepper2_level1);
    gpio_set_dir(stepper2_level1, false);
    gpio_init(stepper2_level2);
    gpio_set_dir(stepper2_level2, false);
    gpio_init(stepper2_level3);
    gpio_set_dir(stepper2_level3, false);
    gpio_init(stepper2_level4);
    gpio_set_dir(stepper2_level4, false);
    gpio_init(stepper2_level5);
    gpio_set_dir(stepper2_level5, false);

    //Pins for S3
    gpio_init(gripper3_open);
    gpio_set_dir(gripper3_open, false);
    gpio_init(gripper3_close);
    gpio_set_dir(gripper3_close, false);
    gpio_init(stepper3_level1);
    gpio_set_dir(stepper3_level1, false);
    gpio_init(stepper3_level2);
    gpio_set_dir(stepper3_level2, false);
    gpio_init(stepper3_level3);
    gpio_set_dir(stepper3_level3, false);
    gpio_init(stepper3_level4);
    gpio_set_dir(stepper3_level4, false);
    gpio_init(stepper3_level5);
    gpio_set_dir(stepper3_level5, false);

    // NODE TIMER ALLOCATOR SUPPORT EXECUTOR
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

    gpio_put(LED_PIN, 1);

    while (true)
    {
        rclc_executor_spin_some(&executor, RCL_S_TO_NS(1));
    }
    return 0;
}