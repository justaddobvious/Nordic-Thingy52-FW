///////////////////////////////////////////////////////////////////////
// Copyright © 2020,
// 4iiii Innovations Inc.,
// Cochrane, Alberta, Canada.
// All rights reserved.
// Redistribution and use in source and binary forms, with or without
// modification, are not permitted without express written approval of
// 4iiii Innovations Inc.
///////////////////////////////////////////////////////////////////////

#include "app_timer.h"
#include "fstorage.h"
#include "m_ble.h"
#include "nrf.h"
#define NRF_LOG_MODULE_NAME "m_ocelot      "
#include "nrf_log.h"
#include "nrf_section.h"
#include "ocelot.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h> 


static uint32_t m_ocelot_evt_handler(enum OCELOT_EVT evt, union OCELOT_EVT_ARGS* args);
static bool m_ocelot_flash_erase_page(uint32_t address);
static uint32_t m_ocelot_flash_write(void const* data, uint32_t address, uint32_t size);
static void m_ocelot_fs_cb(fs_evt_t const* const evt, fs_ret_t result);
static void m_ocelot_timer_handler(void* context);
static uint32_t m_ocelot_service_init(bool major_minor_fw_ver_changed);


APP_TIMER_DEF(m_ocelot_timer_id);
FS_REGISTER_CFG(fs_config_t m_ocelot_fs_config) =
{
   .p_start_addr = (uint32_t*) 0x7B000,
   .p_end_addr = (uint32_t*) 0x7E000,
   .callback = m_ocelot_fs_cb,
   .num_pages = 3,
   .priority =  0xFE
};
static const struct OCELOT_INIT m_ocelot_init_info =
{
   .application_version =
   {
      .release = 1,
      .major = 3,
      .minor = 1
   },
   .evt_handler = m_ocelot_evt_handler
};


bool m_ocelot_init(m_ble_service_handle_t* p_handle)
{
   if (p_handle == NULL)
      return false;

   p_handle->ble_evt_cb = ocelot_on_ble_evt;
   p_handle->init_cb = m_ocelot_service_init;
   return true;
}


static uint32_t m_ocelot_evt_handler(enum OCELOT_EVT evt, union OCELOT_EVT_ARGS* args)
{
   uint32_t retval = 0;

   switch (evt)
   {
      case OCELOT_EVT_RESET_PREPARE:
         break;

      case OCELOT_EVT_FLASH_ERASE_PAGE:
         retval = (uint32_t) m_ocelot_flash_erase_page(args->flash_erase_page_args.address);
         break;

      case OCELOT_EVT_FLASH_WRITE:
         retval = m_ocelot_flash_write(args->flash_write_args.data, args->flash_write_args.address, args->flash_write_args.size);
         break;

      case OCELOT_EVT_REQUEST_FAST_CONN:
         break;

      case OCELOT_EVT_SERVICE_DATA_UPDATE:
         break;

      default:
         break;
   }

   return retval;
}

static bool m_ocelot_flash_erase_page(uint32_t address)
{
   return fs_erase(&m_ocelot_fs_config, (uint32_t*) (uintptr_t) address, 1, NULL) == FS_SUCCESS;
}

static uint32_t m_ocelot_flash_write(void const* data, uint32_t address, uint32_t size)
{
   return fs_store(&m_ocelot_fs_config, (uint32_t*) address, (uint32_t*) data, size / sizeof(uint32_t), NULL) == FS_SUCCESS ? size : 0;
}

static void m_ocelot_fs_cb(fs_evt_t const* const evt, fs_ret_t result)
{
   switch (evt->id)
   {
      case FS_EVT_ERASE:
      case FS_EVT_STORE:
         ocelot_flash_operation_complete(result == FS_SUCCESS);
         break;

      default:
         break;
   }
}

static void m_ocelot_timer_handler(void* context)
{
   (void) context;
   ocelot_tick1hz();
}

static uint32_t m_ocelot_service_init(bool major_minor_fw_ver_changed)
{
   uint32_t err_code;

   (void) major_minor_fw_ver_changed;

   if (fs_init() != FS_SUCCESS)
      return NRF_ERROR_INTERNAL;

   if (ocelot_init(&m_ocelot_init_info) != OCELOT_SUCCESS)
       return NRF_ERROR_INTERNAL;

   err_code = app_timer_create(&m_ocelot_timer_id, APP_TIMER_MODE_REPEATED, m_ocelot_timer_handler);

   if (err_code != NRF_SUCCESS)
      return NRF_ERROR_INTERNAL;

   err_code = app_timer_start(m_ocelot_timer_id, APP_TIMER_TICKS(1000), NULL);

   if (err_code != NRF_SUCCESS)
      return NRF_ERROR_INTERNAL;

   if (ocelot_ble_init() != OCELOT_SUCCESS)
      return NRF_ERROR_INTERNAL;

   return NRF_SUCCESS;
}

