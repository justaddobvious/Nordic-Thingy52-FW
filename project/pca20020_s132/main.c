/*
  Copyright (c) 2010 - 2017, Nordic Semiconductor ASA
  All rights reserved.

  Redistribution and use in source and binary forms, with or without modification,
  are permitted provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright notice, this
     list of conditions and the following disclaimer.

  2. Redistributions in binary form, except as embedded into a Nordic
     Semiconductor ASA integrated circuit in a product or a software update for
     such product, must reproduce the above copyright notice, this list of
     conditions and the following disclaimer in the documentation and/or other
     materials provided with the distribution.

  3. Neither the name of Nordic Semiconductor ASA nor the names of its
     contributors may be used to endorse or promote products derived from this
     software without specific prior written permission.

  4. This software, with or without modification, must only be used with a
     Nordic Semiconductor ASA integrated circuit.

  5. Any software provided in binary form under this license must not be reverse
     engineered, decompiled, modified and/or disassembled.

  THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
  OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
  OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
  GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
  OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/** @file
 *
 * @brief    Thingy application main file.
 *
 * This file contains the source code for the Thingy application that uses the Weather Station service.
 */

#include <stdint.h>
#include <float.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
//#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "softdevice_handler.h"
#include "app_scheduler.h"
#include "app_util_platform.h"
#include "m_ble.h"
//#include "m_environment.h"
//#include "m_sound.h"
//#include "m_motion.h"
//#include "m_ui.h"
#include "m_batt_meas.h"
#include "drv_ext_light.h"
#include "drv_ext_gpio.h"
#include "nrf_delay.h"
#include "nrf_drv_rng.h"
#include "twi_manager.h"
#include "support_func.h"
#include "pca20020.h"
#include "app_error.h"
#include "m_ocelot.h"

#define  NRF_LOG_MODULE_NAME "main          "
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#define DEAD_BEEF   0xDEADBEEF          /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */
#define SCHED_MAX_EVENT_DATA_SIZE   MAX(APP_TIMER_SCHED_EVENT_DATA_SIZE, BLE_STACK_HANDLER_SCHED_EVT_SIZE) /**< Maximum size of scheduler events. */
#define SCHED_QUEUE_SIZE            60  /**< Maximum number of events in the scheduler queue. */

static const nrf_drv_twi_t     m_twi_sensors = NRF_DRV_TWI_INSTANCE(TWI_SENSOR_INSTANCE);
static m_ble_service_handle_t  m_ble_service_handles[THINGY_SERVICES_MAX];
APP_TIMER_DEF(m_led_timer_id);
static drv_ext_light_rgb_sequence_t m_led_sequence =
{
    .sequence_vals.on_time_ms = 35,
    .sequence_vals.on_intensity = 20 * 2.55,
    .sequence_vals.off_intensity = 0,
    .sequence_vals.fade_in_time_ms = 2000,
    .sequence_vals.fade_out_time_ms = 500,
    .sequence_vals.off_time_ms = 0
};


void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info)
{
    #if NRF_LOG_ENABLED
        error_info_t * err_info = (error_info_t*)info;
        NRF_LOG_ERROR(" id = %d, pc = %d, file = %s, line number: %d, error code = %d = %s \r\n", \
        id, pc, nrf_log_push((char*)err_info->p_file_name), err_info->line_num, err_info->err_code, nrf_log_push((char*)nrf_strerror_find(err_info->err_code)));
    #endif
   
#if 0 
    (void)m_ui_led_set_event(M_UI_ERROR);
#endif
    NRF_LOG_FINAL_FLUSH();
    nrf_delay_ms(5);
    
    // On assert, the system can only recover with a reset.
    #ifndef DEBUG
        NVIC_SystemReset();
    #endif

    app_error_save_and_stop(id, pc, info);
}


/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

#if 0
/**@brief Function for putting Thingy into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    uint32_t err_code;

    NRF_LOG_INFO("Entering sleep mode \r\n");
#if 0
    err_code = m_motion_sleep_prepare(true);
    APP_ERROR_CHECK(err_code);
#endif

    err_code = support_func_configure_io_shutdown();
    APP_ERROR_CHECK(err_code);
    
    // Enable wake on button press.
    nrf_gpio_cfg_sense_input(BUTTON, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);
    // Enable wake on low power accelerometer.
    nrf_gpio_cfg_sense_input(LIS_INT1, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_SENSE_HIGH);
   
    NRF_LOG_FLUSH();
    nrf_delay_ms(5);
    // Go to system-off (sd_power_system_off() will not return; wakeup will cause a reset). When debugging, this function may return and code execution will continue.
    err_code = sd_power_system_off();
    NRF_LOG_WARNING("sd_power_system_off() returned. -Probably due to debugger being used. Instructions will still run. \r\n");
    NRF_LOG_FLUSH();
    
    #ifdef DEBUG
        if(!support_func_sys_halt_debug_enabled())
        {
            APP_ERROR_CHECK(err_code); // If not in debug mode, return the error and the system will reboot.
        }
        else
        {
            NRF_LOG_WARNING("Exec stopped, busy wait \r\n");
            NRF_LOG_FLUSH();
            
            while(true) // Only reachable when entering emulated system off.
            {
                // Infinte loop to ensure that code stops in debug mode.
            }
        }
    #else
        APP_ERROR_CHECK(err_code);
    #endif
}
#endif


/**@brief Function for placing the application in low power state while waiting for events.
 */
#define FPU_EXCEPTION_MASK 0x0000009F
static void power_manage(void)
{
    __set_FPSCR(__get_FPSCR()  & ~(FPU_EXCEPTION_MASK));
    (void) __get_FPSCR();
    NVIC_ClearPendingIRQ(FPU_IRQn);

    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}


/**@brief Battery module data handler.
 */
static void m_batt_meas_handler(m_batt_meas_event_t const * p_batt_meas_event)
{
    NRF_LOG_INFO("Voltage: %d V, Charge: %d %%, Event type: %d \r\n",
                p_batt_meas_event->voltage_mv, p_batt_meas_event->level_percent, p_batt_meas_event->type);
   
    if (p_batt_meas_event != NULL)
    {
        if( p_batt_meas_event->type == M_BATT_MEAS_EVENT_LOW)
        {
            uint32_t err_code;

            err_code = support_func_configure_io_shutdown();
            APP_ERROR_CHECK(err_code);
            
            // Enable wake on USB detect only.
            nrf_gpio_cfg_sense_input(USB_DETECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_SENSE_HIGH);

            NRF_LOG_WARNING("Battery voltage low, shutting down Thingy. Connect USB to charge \r\n");
            NRF_LOG_FINAL_FLUSH();
            // Go to system-off mode (This function will not return; wakeup will cause a reset).
            err_code = sd_power_system_off();

            #ifdef DEBUG
                if(!support_func_sys_halt_debug_enabled())
                {
                    APP_ERROR_CHECK(err_code); // If not in debug mode, return the error and the system will reboot.
                }
                else
                {
                    NRF_LOG_WARNING("Exec stopped, busy wait \r\n");
                    NRF_LOG_FLUSH();
                    while(true) // Only reachable when entering emulated system off.
                    {
                        // Infinte loop to ensure that code stops in debug mode.
                    }
                }
            #else
                APP_ERROR_CHECK(err_code);
            #endif
        }
    }
}


/**@brief Function for handling BLE events.
 */
static void thingy_ble_evt_handler(m_ble_evt_t * p_evt)
{
    switch (p_evt->evt_type)
    {
        case thingy_ble_evt_connected:
            NRF_LOG_INFO(NRF_LOG_COLOR_CODE_GREEN "Thingy_ble_evt_connected \r\n");
            break;

        case thingy_ble_evt_disconnected:
            NRF_LOG_INFO(NRF_LOG_COLOR_CODE_YELLOW "Thingy_ble_evt_disconnected \r\n");
            NRF_LOG_FINAL_FLUSH();
            nrf_delay_ms(5);
            NVIC_SystemReset();
            break;

        case thingy_ble_evt_timeout:
            NRF_LOG_INFO(NRF_LOG_COLOR_CODE_YELLOW "Thingy_ble_evt_timeout \r\n");
#if  0
            sleep_mode_enter();
            NVIC_SystemReset();
#endif
            break;
    }
}


/**@brief Function for initializing the leds.
 */
static uint32_t led_init(const nrf_drv_twi_t* p_twi_instance)
{
    uint32_t                        err_code;
    static drv_sx1509_cfg_t         sx1509_cfg;
    drv_ext_light_init_t            led_init;
    //lint --e{651} Potentially confusing initializer
    static const drv_ext_light_conf_t led_conf[DRV_EXT_LIGHT_NUM] = DRV_EXT_LIGHT_CFG;

    static const nrf_drv_twi_config_t twi_config =
    {
        .scl                = TWI_SCL,
        .sda                = TWI_SDA,
        .frequency          = NRF_TWI_FREQ_100K,
        .interrupt_priority = APP_IRQ_PRIORITY_LOW
    };

    sx1509_cfg.twi_addr       = SX1509_ADDR;
    sx1509_cfg.p_twi_instance = p_twi_instance;
    sx1509_cfg.p_twi_cfg      = &twi_config;

    led_init.p_light_conf        = led_conf;
    led_init.num_lights          = DRV_EXT_LIGHT_NUM;
    led_init.clkx_div            = DRV_EXT_LIGHT_CLKX_DIV_8;
    led_init.p_twi_conf          = &sx1509_cfg;
    led_init.resync_pin          = SX_RESET;

    err_code = drv_ext_light_init(&led_init, false);
    APP_ERROR_CHECK(err_code);

    (void)drv_ext_light_off(DRV_EXT_RGB_LED_SENSE);
    (void)drv_ext_light_off(DRV_EXT_RGB_LED_LIGHTWELL);
    
    nrf_gpio_cfg_output(MOS_1);
    nrf_gpio_cfg_output(MOS_2);
    nrf_gpio_cfg_output(MOS_3);
    nrf_gpio_cfg_output(MOS_4);
    nrf_gpio_pin_clear(MOS_1);
    nrf_gpio_pin_clear(MOS_2);
    nrf_gpio_pin_clear(MOS_3);
    nrf_gpio_pin_clear(MOS_4);

    return NRF_SUCCESS;
}


/**@brief Function for getting random number.
 */
static uint16_t get_random_number(void)
{
    uint32_t err_code;
    uint8_t  bytes_available = 0;
    uint8_t retries = 0;
    uint16_t retval;
    uint8_t data[sizeof(retval)];

    nrf_drv_rng_bytes_available(&bytes_available);
    
    while (bytes_available < sizeof(retval))
    {
        retries++;
        NRF_LOG_WARNING("Too few random bytes available. Trying again \r\n");
        nrf_drv_rng_bytes_available(&bytes_available);
        nrf_delay_ms(5);
        
        if (retries > 5)    // Return after n attempts.
        {
            return NRF_ERROR_TIMEOUT;
        }
    }
    
    NRF_LOG_INFO("Available random bytes: %d \r\n", bytes_available);

    err_code = nrf_drv_rng_rand(&data[0], sizeof(retval));
    APP_ERROR_CHECK(err_code);

    retval = data[0] | (((uint16_t) data[1]) << 8);
    NRF_LOG_INFO("Random value (hex): %04x\r\n", retval);
    return retval;
}


/**@brief Function for getting random color mix.
 */
static drv_ext_light_color_mix_t get_random_color_mix(void)
{
    return (drv_ext_light_color_mix_t) ((unsigned) get_random_number() * ((unsigned) DRV_EXT_LIGHT_COLOR_WHITE - (unsigned) DRV_EXT_LIGHT_COLOR_GREEN + 1) / (UINT16_MAX + 1) + (unsigned) DRV_EXT_LIGHT_COLOR_GREEN);
}

static uint32_t get_random_on_time_ms(void)
{
   return (uint32_t) ((unsigned) get_random_number() * (2500 - 35 + 1) / (UINT16_MAX + 1) + 35);
}

static uint32_t get_random_off_time_ms(void)
{
   return (uint32_t) ((unsigned) get_random_number() * (2500 - 2000 + 1) / (UINT16_MAX + 1) + 2000);
}

/**@brief Function for setting random color sequence.
 */
static void m_led_timer_handler(void* context)
{
    uint32_t err_code;
    uint32_t on_time_ms;

    (void) context;

    err_code = drv_ext_light_off(DRV_EXT_RGB_LED_LIGHTWELL);
    APP_ERROR_CHECK(err_code);

    on_time_ms = get_random_on_time_ms();
    m_led_sequence.sequence_vals.on_time_ms = on_time_ms;
    m_led_sequence.color = get_random_color_mix();
    err_code = drv_ext_light_rgb_sequence(DRV_EXT_RGB_LED_LIGHTWELL, &m_led_sequence);
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_start(m_led_timer_id, APP_TIMER_TICKS(get_random_off_time_ms() + on_time_ms), NULL);
}


/**@brief Function for initializing the Thingy.
 */
static void thingy_init(void)
{
    uint32_t                 err_code;
#if 0
    m_ui_init_t              ui_params;
    m_environment_init_t     env_params;
    m_motion_init_t          motion_params;
#endif
    m_ble_init_t             ble_params;
    batt_meas_init_t         batt_meas_init = BATT_MEAS_PARAM_CFG;

    /**@brief Initialize the TWI manager. */
    err_code = twi_manager_init(APP_IRQ_PRIORITY_THREAD);
    APP_ERROR_CHECK(err_code);

#if 0
    /**@brief Initialize LED and button UI module. */
    ui_params.p_twi_instance = &m_twi_sensors;
    err_code = m_ui_init(&m_ble_service_handles[THINGY_SERVICE_UI],
                         &ui_params);
#endif
    err_code = led_init(&m_twi_sensors);
    APP_ERROR_CHECK(err_code);

#if 0
    /**@brief Initialize environment module. */
    env_params.p_twi_instance = &m_twi_sensors;
    err_code = m_environment_init(&m_ble_service_handles[THINGY_SERVICE_ENVIRONMENT],
                                  &env_params);
    APP_ERROR_CHECK(err_code);

    /**@brief Initialize motion module. */
    motion_params.p_twi_instance = &m_twi_sensors;

    err_code = m_motion_init(&m_ble_service_handles[THINGY_SERVICE_MOTION],
                             &motion_params);
    APP_ERROR_CHECK(err_code);

    err_code = m_sound_init(&m_ble_service_handles[THINGY_SERVICE_SOUND]);
    APP_ERROR_CHECK(err_code);
#endif

    /**@brief Initialize the battery measurement. */
    batt_meas_init.evt_handler = m_batt_meas_handler;
    err_code = m_batt_meas_init(&m_ble_service_handles[THINGY_SERVICE_BATTERY], &batt_meas_init);
    APP_ERROR_CHECK(err_code);

    err_code = m_batt_meas_enable(BATT_MEAS_INTERVAL_MS);
    APP_ERROR_CHECK(err_code);

    if (!m_ocelot_init(&m_ble_service_handles[THINGY_SERVICE_OCELOT]))
      APP_ERROR_HANDLER(NRF_ERROR_INTERNAL);

    /**@brief Initialize BLE handling module. */
    ble_params.evt_handler       = thingy_ble_evt_handler;
    ble_params.p_service_handles = m_ble_service_handles;
    ble_params.service_num       = THINGY_SERVICES_MAX;

    err_code = m_ble_init(&ble_params);
    APP_ERROR_CHECK(err_code);

#if 0
    err_code = m_ui_led_set_event(M_UI_BLE_DISCONNECTED);
    APP_ERROR_CHECK(err_code);
#endif

    err_code = app_timer_create(&m_led_timer_id, APP_TIMER_MODE_SINGLE_SHOT, m_led_timer_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_start(m_led_timer_id, APP_TIMER_TICKS(3500), NULL);
    APP_ERROR_CHECK(err_code);
}


static void board_init(void)
{
    uint32_t            err_code;
    drv_ext_gpio_init_t ext_gpio_init;

    #if defined(THINGY_HW_v0_7_0)
        #error   "HW version v0.7.0 not supported."
    #elif defined(THINGY_HW_v0_8_0)
        NRF_LOG_WARNING("FW compiled for depricated Thingy HW v0.8.0 \r\n");
    #elif defined(THINGY_HW_v0_9_0)
        NRF_LOG_WARNING("FW compiled for depricated Thingy HW v0.9.0 \r\n");
    #endif

    static const nrf_drv_twi_config_t twi_config =
    {
        .scl                = TWI_SCL,
        .sda                = TWI_SDA,
        .frequency          = NRF_TWI_FREQ_400K,
        .interrupt_priority = APP_IRQ_PRIORITY_LOW
    };

    static const drv_sx1509_cfg_t sx1509_cfg =
    {
        .twi_addr       = SX1509_ADDR,
        .p_twi_instance = &m_twi_sensors,
        .p_twi_cfg      = &twi_config
    };

    ext_gpio_init.p_cfg = &sx1509_cfg;
    
    err_code = support_func_configure_io_startup(&ext_gpio_init);
    APP_ERROR_CHECK(err_code);

    nrf_delay_ms(100);
}


/**@brief Application main function.
 */
int main(void)
{
    uint32_t err_code;
    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO(NRF_LOG_COLOR_CODE_GREEN"===== Thingy started! =====\r\n");
    
    // Initialize.
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    board_init();
    thingy_init();

    for (;;)
    {
        app_sched_execute();
        
        if (!NRF_LOG_PROCESS()) // Process logs
        { 
            power_manage();
        }
    }
}
