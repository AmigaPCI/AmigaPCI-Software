/**
  ******************************************************************************
  * @file    usbh_core.c
  * @author  MCD Application Team
  * @brief   This file implements the functions for the core state machine process
  *          the enumeration and the control transfer process
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2015 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                      www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usbh_core.h"

/*
 * Root port reset recovery before the first request. USB 2.0 7.1.7.5
 * requires at least 10 ms; Linux waits about 50 ms. Was 100 ms.
 */
#ifndef USBH_RESET_RECOVERY_MS
#define USBH_RESET_RECOVERY_MS 50U
#endif


/** @addtogroup USBH_LIB
  * @{
  */

/** @addtogroup USBH_LIB_CORE
  * @{
  */

/** @defgroup USBH_CORE
  * @brief This file handles the basic enumeration when a device is connected
  *          to the host.
  * @{
  */


/** @defgroup USBH_CORE_Private_Defines
  * @{
  */
#define USBH_ADDRESS_DEFAULT                     0x00U
#define USBH_ADDRESS_ASSIGNED                    0x01U
#define USBH_MPS_DEFAULT                         0x40U
/**
  * @}
  */

/** @defgroup USBH_CORE_Private_Macros
  * @{
  */
/**
  * @}
  */


/** @defgroup USBH_CORE_Private_Variables
  * @{
  */
#if (USBH_USE_OS == 1U)
#if (osCMSIS >= 0x20000U)
osThreadAttr_t USBH_Thread_Atrr;
#endif
#endif


/**
  * @}
  */


/** @defgroup USBH_CORE_Private_Functions
  * @{
  */
static USBH_StatusTypeDef USBH_HandleEnum(USBH_HandleTypeDef *phost);
static void USBH_HandleSof(USBH_HandleTypeDef *phost);
static USBH_StatusTypeDef DeInitStateMachine(USBH_HandleTypeDef *phost);

#if (USBH_USE_OS == 1U)
#if (osCMSIS < 0x20000U)
static void USBH_Process_OS(void const *argument);
#else
static void USBH_Process_OS(void *argument);
#endif
#endif


/**
  * @brief  HCD_Init
  *         Initialize the HOST Core.
  * @param  phost: Host Handle
  * @param  pUsrFunc: User Callback
  * @retval USBH Status
  */
USBH_StatusTypeDef  USBH_Init(USBH_HandleTypeDef *phost,
                              void (*pUsrFunc)(USBH_HandleTypeDef *phost,
                              uint8_t id), uint8_t id)
{
  /* Check whether the USB Host handle is valid */
  if (phost == NULL)
  {
    USBH_ErrLog("Invalid Host handle");
    return USBH_FAIL;
  }

  /* Set DRiver ID */
  phost->id = id;

  /* Unlink class*/
  phost->pActiveClass = NULL;
  phost->ClassNumber = 0U;

  /* Restore default states and prepare EP0 */
  DeInitStateMachine(phost);

  /* Restore default Device connection states */
  phost->device.PortEnabled = 0U;
  phost->device.is_connected = 0U;
  phost->device.is_disconnected = 0U;
  phost->device.is_ReEnumerated = 0U;

  /* Assign User process */
  if (pUsrFunc != NULL)
  {
    phost->pUser = pUsrFunc;
  }

#if (USBH_USE_OS == 1U)
#if (osCMSIS < 0x20000U)

  /* Create USB Host Queue */
  osMessageQDef(USBH_Queue, MSGQUEUE_OBJECTS, uint16_t);
  phost->os_event = osMessageCreate(osMessageQ(USBH_Queue), NULL);

  /* Create USB Host Task */
#if defined (USBH_PROCESS_STACK_SIZE)
  osThreadDef(USBH_Thread, USBH_Process_OS, USBH_PROCESS_PRIO, 0U, USBH_PROCESS_STACK_SIZE);
#else
  osThreadDef(USBH_Thread, USBH_Process_OS, USBH_PROCESS_PRIO, 0U, 8U * configMINIMAL_STACK_SIZE);
#endif /* defined (USBH_PROCESS_STACK_SIZE) */

  phost->thread = osThreadCreate(osThread(USBH_Thread), phost);

#else

  /* Create USB Host Queue */
  phost->os_event = osMessageQueueNew(MSGQUEUE_OBJECTS, sizeof(uint32_t), NULL);

  /* Create USB Host Task */
  USBH_Thread_Atrr.name = "USBH_Queue";

#if defined (USBH_PROCESS_STACK_SIZE)
  USBH_Thread_Atrr.stack_size = USBH_PROCESS_STACK_SIZE;
#else
  USBH_Thread_Atrr.stack_size = (8U * configMINIMAL_STACK_SIZE);
#endif /* defined (USBH_PROCESS_STACK_SIZE) */

  USBH_Thread_Atrr.priority = USBH_PROCESS_PRIO;
  phost->thread = osThreadNew(USBH_Process_OS, phost, &USBH_Thread_Atrr);

#endif /* (osCMSIS < 0x20000U) */
#endif /* (USBH_USE_OS == 1U) */

  /* Initialize low level driver */
  USBH_LL_Init(phost);

  return USBH_OK;
}


/**
  * @brief  HCD_Init
  *         De-Initialize the Host portion of the driver.
  * @param  phost: Host Handle
  * @retval USBH Status
  */
USBH_StatusTypeDef USBH_DeInit(USBH_HandleTypeDef *phost)
{
  DeInitStateMachine(phost);

  /* Restore default Device connection states */
  phost->device.PortEnabled = 0U;
  phost->device.is_connected = 0U;
  phost->device.is_disconnected = 0U;
  phost->device.is_ReEnumerated = 0U;

  if (phost->pData != NULL)
  {
    USBH_LL_Stop(phost);
  }

  return USBH_OK;
}


/**
  * @brief  DeInitStateMachine
  *         De-Initialize the Host state machine.
  * @param  phost: Host Handle
  * @retval USBH Status
  */
static USBH_StatusTypeDef DeInitStateMachine(USBH_HandleTypeDef *phost)
{
  uint32_t i = 0U;

  /* Clear Pipes flags*/
  for (i = 0U; i < USBH_MAX_PIPES_NBR; i++)
  {
    phost->Pipes[i] = 0U;
  }

  for (i = 0U; i < USBH_MAX_DATA_BUFFER; i++)
  {
    phost->device.Data[i] = 0U;
  }

  phost->gState = HOST_IDLE;
  phost->EnumState = ENUM_IDLE;
  phost->RequestState = CMD_SEND;
  phost->Timer = 0U;

  phost->Control.state = CTRL_SETUP;
  phost->Control.pipe_size = USBH_MPS_DEFAULT;
  phost->Control.errorcount = 0U;

  phost->device.address = USBH_ADDRESS_DEFAULT;
  phost->device.speed = USBH_SPEED_FULL;

  return USBH_OK;
}


/**
  * @brief  USBH_RegisterClass
  *         Link class driver to Host Core.
  * @param  phost : Host Handle
  * @param  pclass: Class handle
  * @retval USBH Status
  */
USBH_StatusTypeDef USBH_RegisterClass(USBH_HandleTypeDef *phost, USBH_ClassTypeDef *pclass)
{
  USBH_StatusTypeDef status = USBH_OK;

  if (pclass != NULL)
  {
    if (phost->ClassNumber < USBH_MAX_NUM_SUPPORTED_CLASS)
    {
      /* link the class to the USB Host handle */
      phost->pClass[phost->ClassNumber++] = pclass;
      status = USBH_OK;
    }
    else
    {
      USBH_ErrLog("Max Class Number reached");
      status = USBH_FAIL;
    }
  }
  else
  {
    USBH_ErrLog("Invalid Class handle");
    status = USBH_FAIL;
  }

  return status;
}


static int getInterfaceIdxFromNum(USBH_HandleTypeDef *phost, uint8_t num)
{
    int i = 0;
    for (; i < phost->device.CfgDesc.bNumInterfaces; ++i) {
        if (phost->device.CfgDesc.Itf_Desc[i].bInterfaceNumber == num)
            return i;
    }
    return -1;
}

/**
  * @brief  USBH_SelectInterface
  *         Select current interface.
  * @param  phost: Host Handle
  * @param  interface: Interface number
  * @retval USBH Status
  */
USBH_StatusTypeDef USBH_SelectInterface(USBH_HandleTypeDef *phost, uint8_t interface)
{
  USBH_StatusTypeDef status = USBH_OK;

  if (phost->device.current_interface == interface)
      return (status);

  USBH_UsrLog ("USB%s Switching to Interface (#%d)", usb_get_path(phost, NULL, -1), interface);
  USBH_UsrLog ("  Class     0x%02x", phost->device.CfgDesc.Itf_Desc[interface].bInterfaceClass );
  USBH_UsrLog ("  SubClass  0x%02x", phost->device.CfgDesc.Itf_Desc[interface].bInterfaceSubClass );
  USBH_UsrLog ("  Protocol  0x%02x", phost->device.CfgDesc.Itf_Desc[interface].bInterfaceProtocol );
  int val = getInterfaceIdxFromNum(phost, interface);
  if (val >= 0 && val < phost->device.CfgDesc.bNumInterfaces) {
    phost->device.current_interface = val; //interface;
  } else {
    USBH_ErrLog ("  Cannot Select Interface (#%d).", interface);
    status = USBH_FAIL;
  }

  return status;
}

#if 0
/**
  * @brief  USBH_GetActiveClass
  *         Return Device Class.
  * @param  phost: Host Handle
  * @param  interface: Interface index
  * @retval Class Code
  */
uint8_t USBH_GetActiveClass(USBH_HandleTypeDef *phost)
{
  return (phost->device.CfgDesc.Itf_Desc[0].bInterfaceClass);
}
#endif


/**
  * @brief  USBH_FindInterface
  *         Find the interface index for a specific class.
  * @param  phost: Host Handle
  * @param  Class: Class code
  * @param  SubClass: SubClass code
  * @param  Protocol: Protocol code
  * @retval interface index in the configuration structure
  * @note : (1)interface index 0xFF means interface index not found
  */
uint8_t  USBH_FindInterface(USBH_HandleTypeDef *phost, uint8_t Class, uint8_t SubClass, uint8_t Protocol)
{
  USBH_InterfaceDescTypeDef *pif;
  USBH_CfgDescTypeDef *pcfg;
  uint8_t if_ix = 0U;

  pif = (USBH_InterfaceDescTypeDef *)0;
  pcfg = &phost->device.CfgDesc;

  while (if_ix < USBH_MAX_NUM_INTERFACES)
  {
    pif = &pcfg->Itf_Desc[if_ix];
    if (((pif->bInterfaceClass == Class) || (Class == 0xFFU)) &&
        ((pif->bInterfaceSubClass == SubClass) || (SubClass == 0xFFU)) &&
        ((pif->bInterfaceProtocol == Protocol) || (Protocol == 0xFFU)))
    {
      return  if_ix;
    }
    if_ix++;
  }
  return 0xFFU;
}


/**
  * @brief  USBH_FindInterfaceIndex
  *         Find the interface index for a specific class interface and alternate setting number.
  * @param  phost: Host Handle
  * @param  interface_number: interface number
  * @param  alt_settings    : alternate setting number
  * @retval interface index in the configuration structure
  * @note : (1)interface index 0xFF means interface index not found
  */
uint8_t  USBH_FindInterfaceIndex(USBH_HandleTypeDef *phost, uint8_t interface_number, uint8_t alt_settings)
{
  USBH_InterfaceDescTypeDef *pif;
  USBH_CfgDescTypeDef *pcfg;
  uint8_t if_ix = 0U;

  pif = (USBH_InterfaceDescTypeDef *)0;
  pcfg = &phost->device.CfgDesc;

  while (if_ix < USBH_MAX_NUM_INTERFACES)
  {
    pif = &pcfg->Itf_Desc[if_ix];
    if ((pif->bInterfaceNumber == interface_number) && (pif->bAlternateSetting == alt_settings))
    {
      return  if_ix;
    }
    if_ix++;
  }
  return 0xFFU;
}


/**
  * @brief  USBH_Start
  *         Start the USB Host Core.
  * @param  phost: Host Handle
  * @retval USBH Status
  */
USBH_StatusTypeDef  USBH_Start(USBH_HandleTypeDef *phost)
{
  /* Start the low level driver  */
  USBH_LL_Start(phost);

  /* Activate VBUS on the port */
  USBH_LL_DriverVBUS(phost, TRUE);

  return USBH_OK;
}


/**
  * @brief  USBH_Stop
  *         Stop the USB Host Core.
  * @param  phost: Host Handle
  * @retval USBH Status
  */
USBH_StatusTypeDef  USBH_Stop(USBH_HandleTypeDef *phost)
{
  /* DeActivate VBUS on the port */
  USBH_LL_DriverVBUS(phost, FALSE);

  /* Stop and cleanup the low level driver  */
  USBH_LL_Stop(phost);

  /* Free Control Pipes */
  USBH_FreePipe(phost, phost->Control.pipe_in);
  USBH_FreePipe(phost, phost->Control.pipe_out);

  return USBH_OK;
}


/**
  * @brief  HCD_ReEnumerate
  *         Perform a new Enumeration phase.
  * @param  phost: Host Handle
  * @retval USBH Status
  */
USBH_StatusTypeDef USBH_ReEnumerate(USBH_HandleTypeDef *phost)
{
  if (USBH_IsPortEnabled(phost))
  {
    phost->device.is_ReEnumerated = 1U;

    /*Stop Host */
    USBH_Stop(phost);

    phost->device.is_disconnected = 1U;
  }

#if (USBH_USE_OS == 1U)
  phost->os_msg = (uint32_t)USBH_PORT_EVENT;
#if (osCMSIS < 0x20000U)
  (void)osMessagePut(phost->os_event, phost->os_msg, 0U);
#else
  (void)osMessageQueuePut(phost->os_event, &phost->os_msg, 0U, NULL);
#endif
#endif

  return USBH_OK;
}

/* Open Control pipes */
void
USBH_OpenControlPipes(USBH_HandleTypeDef *phost)
{
    USBH_OpenPipe(phost,
                  phost->Control.pipe_in,
                  0x80U,
                  phost->device.address,
                  phost->device.speed,
                  USBH_EP_CONTROL,
                  (uint16_t)phost->Control.pipe_size);

    USBH_OpenPipe(phost,
                  phost->Control.pipe_out,
                  0x00U,
                  phost->device.address,
                  phost->device.speed,
                  USBH_EP_CONTROL,
                  (uint16_t)phost->Control.pipe_size);
}


/**
  * @brief  USBH_Process
  *         Background process of the USB Core.
  * @param  phost: Host Handle
  * @retval USBH Status
  */
USBH_StatusTypeDef  USBH_Process(USBH_HandleTypeDef *phost)
{
  __IO USBH_StatusTypeDef status = USBH_FAIL;
  uint8_t idx = 0U;

  /* check for Host pending port disconnect event */
  if (phost->device.is_disconnected == 1U)
  {
    phost->gState = HOST_DEV_DISCONNECTED;
  }

  switch (phost->gState)
  {
    case HOST_IDLE:
      if (!phost->device.is_connected)
        break;
      USBH_UsrLog("USB%s Device Connected", usb_get_path(phost, NULL, -1));
      phost->proc_timer = 0;
      phost->gState = HOST_RESET;
      break;

    case HOST_RESET:
      if (phost->proc_timer++ < 200) {  // 200 ms
        /* Wait between connection and reset (SOF Timer doesn't tick yet) */
        USBH_Delay(1U);  // 1 ms each iteration
        break;
      }

      phost->gState = HOST_DEV_WAIT_FOR_ATTACHMENT;
      USBH_LL_ResetPort(phost);

#if (USBH_USE_OS == 1U)
      phost->os_msg = (uint32_t)USBH_PORT_EVENT;
#if (osCMSIS < 0x20000U)
      (void)osMessagePut(phost->os_event, phost->os_msg, 0U);
#else
      (void)osMessageQueuePut(phost->os_event, &phost->os_msg, 0U, NULL);
#endif
#endif
      break;

    case HOST_DEV_WAIT_FOR_ATTACHMENT: /* Wait for Port Enabled */

      if (phost->device.PortEnabled == 1U)
      {
        USBH_UsrLog("USB%s Device Reset Completed", usb_get_path(phost, NULL, -1));
        phost->gState = HOST_DEV_ATTACHED;
        phost->proc_timer = phost->Timer;
      }
      break;

    case HOST_DEV_ATTACHED:
      /*
       * Reset recovery: wait USBH_RESET_RECOVERY_MS between port reset
       * completion and the first request (USB 2.0 7.1.7.5 requires at
       * least 10 ms). The test was previously inverted (> 100), so there
       * was no wait at all.
       */
      if (phost->Timer - phost->proc_timer < USBH_RESET_RECOVERY_MS)
        break;  // Wait between reset and attach

      if (phost->pUser != NULL)
      {
        phost->pUser(phost, HOST_USER_CONNECTION);
      }

      if (!phost->device.is_connected) {
        USBH_UsrLog ("USB%s DISCONNECTED DURING CONNECTION DELAY",
                     usb_get_path(phost, NULL, -1));
        phost->gState = HOST_DEV_DISCONNECTED;
        return USBH_OK;
      }

      phost->device.speed = USBH_LL_GetSpeed(phost);
      USBH_UsrLog("USB%s %s speed device", usb_get_path(phost, NULL, -1),
                  (phost->device.speed == USBH_SPEED_LOW) ? "low" :
                  (phost->device.speed == USBH_SPEED_HIGH) ? "high" : "full");

      phost->gState = HOST_ENUMERATION;

      phost->Control.pipe_out = USBH_AllocPipe(phost, 0x00U);
      phost->Control.pipe_in  = USBH_AllocPipe(phost, 0x80U);
      phost->busy = 1;  // Paired with -- in HOST_CLASS_REQUEST

      /*
       * EP0 max packet size until the device descriptor says otherwise.
       * 64 for full speed, so the 64-byte first GET_DESCRIPTOR ends on
       * the device's (possibly 8-byte) short packet.
       */
      phost->Control.pipe_size = (phost->device.speed == USBH_SPEED_LOW) ?
                                 USBH_MPS_LOWSPEED : USBH_MPS_DEFAULT;

      USBH_OpenControlPipes(phost);

#if (USBH_USE_OS == 1U)
      phost->os_msg = (uint32_t)USBH_PORT_EVENT;
#if (osCMSIS < 0x20000U)
      (void)osMessagePut(phost->os_event, phost->os_msg, 0U);
#else
      (void)osMessageQueuePut(phost->os_event, &phost->os_msg, 0U, NULL);
#endif
#endif
      break;

    case HOST_ENUMERATION:
      /* Check for enumeration status */
      status = USBH_HandleEnum(phost);
      if (status == USBH_FAIL)
      {
        /*
         * Enumeration failed (e.g. device descriptor request timed out).
         * Previously the handle was left in HOST_ENUMERATION with
         * busy set, forever.
         */
        USBH_UsrLog("USB%s Enumeration failed", usb_get_path(phost, NULL, -1));
        usb_hc_stats_report(phost);
        phost->enum_failed = 1;
        phost->busy = 0;
        phost->gState = HOST_ABORT_STATE;
        if (phost->parent == NULL)
          usb_root_enum_failed(phost);  /* Retry with port re-init */
      }
      else if (status == USBH_OK)
      {
        /* The function shall return USBH_OK when full enumeration is complete */
        USBH_UsrLog("USB%s Enumeration done.", usb_get_path(phost, NULL, -1));
        usb_hc_stats_report(phost);
        if (phost->parent == NULL)
          usb_root_enum_ok(phost);
        phost->device.current_interface = 0U;

        if (phost->device.DevDesc.bNumConfigurations == 1U)
        {
          USBH_UsrLog("USB%s This device has only 1 configuration.", usb_get_path(phost, NULL, -1));
          phost->gState = HOST_SET_CONFIGURATION;
        }
        else
        {
          phost->gState = HOST_INPUT;
        }

      }
      break;

    case HOST_INPUT:
    {
      /* user callback for end of device basic enumeration */
      if (phost->pUser != NULL)
      {
        phost->pUser(phost, HOST_USER_SELECT_CONFIGURATION);
        phost->gState = HOST_SET_CONFIGURATION;

#if (USBH_USE_OS == 1U)
        phost->os_msg = (uint32_t)USBH_STATE_CHANGED_EVENT;
#if (osCMSIS < 0x20000U)
        (void)osMessagePut(phost->os_event, phost->os_msg, 0U);
#else
        (void)osMessageQueuePut(phost->os_event, &phost->os_msg, 0U, NULL);
#endif
#endif
      }
    }
    break;

    case HOST_SET_CONFIGURATION:
      /* set configuration */
      status = USBH_SetCfg(phost, (uint16_t)phost->device.CfgDesc.bConfigurationValue);
      if (status == USBH_OK)
      {
        phost->gState = HOST_SET_WAKEUP_FEATURE;
        USBH_UsrLog("USB%s Default configuration set.", usb_get_path(phost, NULL, -1));
      }
      else if (status != USBH_BUSY)
      {
        /* Previously retried forever on failure */
        USBH_UsrLog("USB%s Set configuration failed", usb_get_path(phost, NULL, -1));
        phost->enum_failed = 1;
        phost->busy = 0;
        phost->gState = HOST_ABORT_STATE;
        if (phost->parent == NULL)
          usb_root_enum_failed(phost);  /* Retry with port re-init */
      }

#if (USBH_USE_OS == 1U)
      phost->os_msg = (uint32_t)USBH_PORT_EVENT;
#if (osCMSIS < 0x20000U)
      (void)osMessagePut(phost->os_event, phost->os_msg, 0U);
#else
      (void)osMessageQueuePut(phost->os_event, &phost->os_msg, 0U, NULL);
#endif
#endif
      break;

    case  HOST_SET_WAKEUP_FEATURE:

      if ((phost->device.CfgDesc.bmAttributes) & (1U << 5))
      {
        status = USBH_SetFeature(phost, FEATURE_SELECTOR_REMOTEWAKEUP);
        if (status == USBH_OK)
        {
#if 0
          USBH_UsrLog("USB%s Device remote wakeup enabled", usb_get_path(phost, NULL, -1));
#endif
          phost->gState = HOST_CHECK_CLASS;
        }
        else if (status != USBH_BUSY)
        {
          /*
           * Remote wakeup is optional; some devices STALL it. This used to
           * retry forever. Carry on without it.
           */
          USBH_UsrLog("USB%s Remote wakeup not enabled (%d)",
                      usb_get_path(phost, NULL, -1), status);
          phost->gState = HOST_CHECK_CLASS;
        }
      }
      else
      {
        phost->gState = HOST_CHECK_CLASS;
      }

#if (USBH_USE_OS == 1U)
      phost->os_msg = (uint32_t)USBH_PORT_EVENT;
#if (osCMSIS < 0x20000U)
      (void)osMessagePut(phost->os_event, phost->os_msg, 0U);
#else
      (void)osMessageQueuePut(phost->os_event, &phost->os_msg, 0U, NULL);
#endif
#endif
      break;

    case HOST_CHECK_CLASS:

      if (phost->ClassNumber == 0U)
      {
        USBH_UsrLog("USB%s No Class has been registered.", usb_get_path(phost, NULL, -1));
        phost->gState = HOST_ABORT_STATE;
      }
      else
      {
        phost->pActiveClass = NULL;

        /*
         * XXX CDH: Only looking at Itf_Desc[0] here.
         *          Maybe need to walk through CfgDesc.bNumInterfaces and
         *          call the class for each one
         *          phost->ClassNumber is used to index phost->pClass[] to locate
         *          the appropriate class to manage this device.
         *          Does phost->pActiveClass need to become an array?
         */
        for (idx = 0U; idx < USBH_MAX_NUM_SUPPORTED_CLASS; idx++)
        {
          if ((phost->pClass[idx] != NULL) &&
              (phost->pClass[idx]->ClassCode == phost->device.CfgDesc.Itf_Desc[0].bInterfaceClass))
          {
            if ((phost->pClass[idx]->Probe != NULL) &&
                (phost->pClass[idx]->Probe(phost) != USBH_OK)) {
                continue;  // Probe did not find a match
            }
            phost->pActiveClass = phost->pClass[idx];
            break;
          }
        }

        if (phost->pActiveClass != NULL)
        {
          status = phost->pActiveClass->Init(phost);
          if (status == USBH_OK)
          {
            phost->gState = HOST_CLASS_REQUEST;
            USBH_UsrLog("USB%s %s class started.", usb_get_path(phost, NULL, -1), phost->pActiveClass->Name);

            /* Inform user that a class has been activated */
            phost->pUser(phost, HOST_USER_CLASS_SELECTED);
          }
          else if (status != USBH_BUSY)
          {
            phost->gState = HOST_ABORT_STATE;
            USBH_UsrLog("USB%s Device not supporting %s class.", usb_get_path(phost, NULL, -1), phost->pActiveClass->Name);
            phost->busy--;  // Paired with = 1 in HOST_DEV_ATTACHED
          }
          else
          {
            USBH_UsrLog("USB%s Device busy.", usb_get_path(phost, NULL, -1));
          }
        }
        else
        {
          phost->gState = HOST_ABORT_STATE;
          USBH_UsrLog("USB%s No registered class (%x) for this device.",
                      usb_get_path(phost, NULL, -1),
                      phost->device.CfgDesc.Itf_Desc[0].bInterfaceClass);
          phost->busy--;  // Paired with = 1 in HOST_DEV_ATTACHED
        }
      }

#if (USBH_USE_OS == 1U)
      phost->os_msg = (uint32_t)USBH_STATE_CHANGED_EVENT;
#if (osCMSIS < 0x20000U)
      (void)osMessagePut(phost->os_event, phost->os_msg, 0U);
#else
      (void)osMessageQueuePut(phost->os_event, &phost->os_msg, 0U, NULL);
#endif
#endif
      break;

    case HOST_CLASS_REQUEST:
      /* process class standard control requests state machine */
      if (phost->pActiveClass != NULL)
      {
        status = phost->pActiveClass->Requests(phost);

        if (status != USBH_BUSY)
        {
          phost->gState = HOST_CLASS;
          phost->busy--;  // Paired with = 1 in HOST_DEV_ATTACHED
        }
      }
      else
      {
        phost->gState = HOST_ABORT_STATE;
        USBH_ErrLog("Invalid Class Driver.");
        phost->busy--;  // Paired with = 1 in HOST_DEV_ATTACHED

#if (USBH_USE_OS == 1U)
        phost->os_msg = (uint32_t)USBH_STATE_CHANGED_EVENT;
#if (osCMSIS < 0x20000U)
        (void)osMessagePut(phost->os_event, phost->os_msg, 0U);
#else
        (void)osMessageQueuePut(phost->os_event, &phost->os_msg, 0U, NULL);
#endif
#endif
      }
      break;

    case HOST_CLASS:
      /* process class state machine */
      if (phost->pActiveClass != NULL)
      {
        phost->pActiveClass->BgndProcess(phost);
      }
      break;

    case HOST_DEV_DISCONNECTED :
      phost->device.is_disconnected = 0U;

      /* If a Hub is disconnecting, remove all plugged devices */
      USBH_remove_subdevices(phost);

      DeInitStateMachine(phost);

      /* Re-Initilaize Host for new Enumeration */
      if (phost->pActiveClass != NULL)
      {
        phost->pActiveClass->DeInit(phost);
        phost->pActiveClass = NULL;
      }

      if (phost->pUser != NULL)
      {
        phost->pUser(phost, HOST_USER_DISCONNECTION);
      }
      USBH_UsrLog("USB%s Device disconnected", usb_get_path(phost, NULL, -1));

      if (phost->device.is_ReEnumerated == 1U)
      {
        phost->device.is_ReEnumerated = 0U;

        /* Start the host and re-enable Vbus */
        USBH_Start(phost);
      }
      else
      {
        /* Device Disconnection Completed, start USB Driver */
        USBH_LL_Start(phost);
      }

#if (USBH_USE_OS == 1U)
      phost->os_msg = (uint32_t)USBH_PORT_EVENT;
#if (osCMSIS < 0x20000U)
      (void)osMessagePut(phost->os_event, phost->os_msg, 0U);
#else
      (void)osMessageQueuePut(phost->os_event, &phost->os_msg, 0U, NULL);
#endif
#endif
      break;

    case HOST_ABORT_STATE:
    default :
      break;
  }
  return USBH_OK;
}

static void
trim_spaces(char *str)
{
    char *ptr;
    char *eptr = str + strlen(str);

    for (ptr = eptr - 1; ptr >= str; ptr--) {
        if (*ptr == ' ')
            *ptr = '\0';
        else
            break;
    }
}

/**
  * @brief  USBH_HandleEnum
  *         This function includes the complete enumeration process
  * @param  phost: Host Handle
  * @retval USBH_Status
  *
  * Enumeration:
  *   1. The host port (or hub port) is reset, which puts the attached USB
  *      device into an initialized state where it responds to device 0.
  *   2. A partial device descriptor is acquired, so that the maximum packet
  *      size can be determined.
  *   3. Host pipes are opened, which initializes the STM32 hardware for
  *      sending and receiving from the control endpoints (80=in, 00=out)
  *      with appropriate sizes and device address.
  *   4. The full device descriptor is then read from the device.
  *      This provides, among other things, the VID, PID, and class of the
  *      device.
  *   5. The device address is then assigned, after which pipes are reopened
  *      to initialize hardware with the new addresss.
  *   6. The first part of the configuration descriptor is read, which
  *      provides the total length of the descriptor.
  *   7. The full configuration descriptor is read.
  *   8. Manufacturer, product, and serial strings are then read.
  *      This ends enumeration.
  *
  * When a USB Hub is involved, there are steps before 1 above:
  *   0A. Hub detects the device connection and reports port status change
  *       sending an interrupt transfer to the host on its interrupt IN EP.
  *   0B. Hub class driver gets status with USB_REQUEST_GET_STATUS
  *   0C. Hub class driver sets HUB_FEATURE_SEL_PORT_POWER
  *   0D. Hub class driver sets HUB_FEAT_SEL_PORT_RESET
  *   0E. Hub reports back that port is enabled and device is attached.
  *
  * * A hub is required to route all traffic for address 0 to any device
  *   that has just been reset and is enabled. Other ports are blocked
  *   from receiving address 0 traffic.
  *
  *   5A. The hub sees the device address assignment and routes future
  *       traffic for that address only to that port.
  */
/*
 * Enumeration order, as Linux and Windows do it:
 *   1. GET_DESCRIPTOR(device, wLength 64) at address 0. The device returns
 *      its 18-byte descriptor (ending the transfer with a short packet);
 *      bMaxPacketSize0 is taken from it.
 *   2. SET_ADDRESS.
 *   3. GET_DESCRIPTOR(device, 18) at the new address.
 *   4. Configuration descriptor, strings, ...
 *
 * This replaces the ST order (GET_DESCRIPTOR(8) and GET_DESCRIPTOR(18)
 * at address 0, then SET_ADDRESS), so device firmware sees the same
 * sequence it sees on the common hosts.
 */
static USBH_StatusTypeDef USBH_HandleEnum(USBH_HandleTypeDef *phost)
{
  USBH_StatusTypeDef Status = USBH_BUSY;
  USBH_StatusTypeDef tstatus;

  switch (phost->EnumState)
  {
    case ENUM_IDLE:
      /* Get Device Desc at address 0 (asking for 64) : EP0 MaxPacketSize */

      /*
       * Ensure we are talking with the correct device. Only (re)open the
       * pipes before a request starts; re-initializing the channel while
       * a transfer is in flight clears its pending interrupt status.
       */
      if (phost->RequestState == CMD_SEND)
        USBH_OpenControlPipes(phost);

      tstatus = USBH_Get_DevDesc(phost, 64U);
      if (tstatus == USBH_OK) {
        printf("USB%s da=%u len=%x type=%x bcdUSB=%x class=%x subclass=%x proto=%x maxpacket=%x\n",
               usb_get_path(phost, NULL, -1),
               phost->device.address,
               phost->device.DevDesc.bLength,
               phost->device.DevDesc.bDescriptorType,
               phost->device.DevDesc.bcdUSB,
               phost->device.DevDesc.bDeviceClass,
               phost->device.DevDesc.bDeviceSubClass,
               phost->device.DevDesc.bDeviceProtocol,
               phost->device.DevDesc.bMaxPacketSize);
        phost->Control.pipe_size = phost->device.DevDesc.bMaxPacketSize;

        /* Assign the address next; full descriptor after that */
        phost->EnumState = ENUM_SET_ADDR;

        /* modify control channels configuration for MaxPacket size */
        USBH_OpenControlPipes(phost);
      } else if (tstatus != USBH_BUSY) {
        Status = USBH_FAIL;
        phost->EnumState = ENUM_DONE;
      }
      break;

    case ENUM_GET_FULL_DEV_DESC:
      /* Get FULL Device Desc  */
      tstatus = USBH_Get_DevDesc(phost, USB_DEVICE_DESC_SIZE);
      if (tstatus == USBH_OK)
      {
        USBH_UsrLog("USB%s VID:PID %04x:%04x",
                    usb_get_path(phost, NULL, -1),
                    phost->device.DevDesc.idVendor,
                    phost->device.DevDesc.idProduct);
        /* Address already assigned */
        phost->EnumState = ENUM_GET_CFG_DESC;
//      phost->Control.pipe_size = phost->device.DevDesc.bMaxPacketSize;
      } else if (tstatus != USBH_BUSY) {
        Status = USBH_FAIL;
        phost->EnumState = ENUM_DONE;
      }
      break;

    case ENUM_SET_ADDR:
      /* set address */
      tstatus = USBH_SetAddress(phost, phost->address);
      if (tstatus != USBH_OK)
      {
        /* Previously retried forever on failure */
        if (tstatus != USBH_BUSY)
        {
          USBH_UsrLog("USB%s Set address failed", usb_get_path(phost, NULL, -1));
          Status = USBH_FAIL;
          phost->EnumState = ENUM_DONE;
        }
      }
      else
      {
        USBH_Delay(2U);
        phost->device.address = phost->address;

        /* user callback for device address assigned */
        USBH_UsrLog("USB%s Address (#%d) assigned.",
                    usb_get_path(phost, NULL, -1), phost->device.address);
        /* Read the full device descriptor at the new address next */
        phost->EnumState = ENUM_GET_FULL_DEV_DESC;

        /* modify control channels to update device address */
        USBH_OpenControlPipes(phost);
      }
      break;

    case ENUM_GET_CFG_DESC:
      /* get standard configuration descriptor */
      tstatus = USBH_Get_CfgDesc(phost, USB_CONFIGURATION_DESC_SIZE);
      if (tstatus == USBH_OK)
      {
        phost->EnumState = ENUM_GET_FULL_CFG_DESC;
      }
      else if (tstatus != USBH_BUSY)
      {
        Status = USBH_FAIL;
        phost->EnumState = ENUM_DONE;
      }
      break;

    case ENUM_GET_FULL_CFG_DESC:
      /* get FULL config descriptor (config, interface, endpoints) */
      tstatus = USBH_Get_CfgDesc(phost, phost->device.CfgDesc.wTotalLength);
      if (tstatus == USBH_OK) {
        phost->EnumState = ENUM_GET_MFC_STRING_DESC;
      } else if (tstatus != USBH_BUSY) {
        Status = USBH_FAIL;
        phost->EnumState = ENUM_DONE;
      }
      break;

    case ENUM_GET_MFC_STRING_DESC:
      if (phost->device.DevDesc.iManufacturer != 0U)
      {
        /* Check that Manufacturer String is available */

        tstatus = USBH_Get_StringDesc(phost,
                                      phost->device.DevDesc.iManufacturer,
                                      phost->device.Data, 0xFFU);
        if (tstatus == USBH_OK) {
          /* User callback for Manufacturing string */
          USBH_UsrLog("USB%s Manufacturer: %s",
                      usb_get_path(phost, NULL, -1),
                      (char *)(void *)phost->device.Data);
          phost->EnumState = ENUM_GET_PRODUCT_STRING_DESC;
          strncpy(phost->device.manufacturer_string, (char *)(void *)phost->device.Data,
                  sizeof (phost->device.manufacturer_string));
          phost->device.manufacturer_string[sizeof (phost->device.manufacturer_string) - 1] = '\0';
          trim_spaces(phost->device.manufacturer_string);

#if (USBH_USE_OS == 1U)
          phost->os_msg = (uint32_t)USBH_STATE_CHANGED_EVENT;
#if (osCMSIS < 0x20000U)
          (void)osMessagePut(phost->os_event, phost->os_msg, 0U);
#else
          (void)osMessageQueuePut(phost->os_event, &phost->os_msg, 0U, NULL);
#endif
#endif
        } else if (tstatus != USBH_BUSY) {
            goto fail_manufacturer_string;
        }
      }
      else
      {
fail_manufacturer_string:
        USBH_UsrLog("USB%s Manufacturer: N/A", usb_get_path(phost, NULL, -1));
        phost->EnumState = ENUM_GET_PRODUCT_STRING_DESC;

#if (USBH_USE_OS == 1U)
        phost->os_msg = (uint32_t)USBH_STATE_CHANGED_EVENT;
#if (osCMSIS < 0x20000U)
        (void)osMessagePut(phost->os_event, phost->os_msg, 0U);
#else
        (void)osMessageQueuePut(phost->os_event, &phost->os_msg, 0U, NULL);
#endif
#endif
      }
      break;

    case ENUM_GET_PRODUCT_STRING_DESC:
      if (phost->device.DevDesc.iProduct != 0U)
      {
        /* Check that Product string is available */
        tstatus = USBH_Get_StringDesc(phost,
                                      phost->device.DevDesc.iProduct,
                                      phost->device.Data, 0xFFU);
        if (tstatus == USBH_OK) {
          /* User callback for Product string */
          USBH_UsrLog("USB%s Product: %s", usb_get_path(phost, NULL, -1), (char *)(void *)phost->device.Data);
          phost->EnumState = ENUM_GET_SERIALNUM_STRING_DESC;
          strncpy(phost->device.product_string, (char *)(void *)phost->device.Data,
                  sizeof (phost->device.product_string));
          phost->device.product_string[sizeof (phost->device.product_string) - 1] = '\0';
          trim_spaces(phost->device.product_string);
        } else if (tstatus != USBH_BUSY) {
            goto fail_product_string;
        }
      }
      else
      {
fail_product_string:
        USBH_UsrLog("USB%s Product : N/A", usb_get_path(phost, NULL, -1));
        phost->EnumState = ENUM_GET_SERIALNUM_STRING_DESC;

#if (USBH_USE_OS == 1U)
        phost->os_msg = (uint32_t)USBH_STATE_CHANGED_EVENT;
#if (osCMSIS < 0x20000U)
        (void)osMessagePut(phost->os_event, phost->os_msg, 0U);
#else
        (void)osMessageQueuePut(phost->os_event, &phost->os_msg, 0U, NULL);
#endif
#endif
      }
      break;

    case ENUM_GET_SERIALNUM_STRING_DESC:
      if (phost->device.DevDesc.iSerialNumber != 0U)
      {
        /* Check that Serial number string is available */
        tstatus = USBH_Get_StringDesc(phost,
                                      phost->device.DevDesc.iSerialNumber,
                                      phost->device.Data, 0xFFU);
        if (tstatus == USBH_OK) {
          /* User callback for Serial number string */
          USBH_UsrLog("USB%s Serial: %s", usb_get_path(phost, NULL, -1), (char *)(void *)phost->device.Data);
          Status = USBH_OK;
          phost->EnumState = ENUM_DONE;
        } else if (tstatus != USBH_BUSY) {
            goto fail_serialnum_string;
        }
      }
      else
      {
fail_serialnum_string:
        USBH_UsrLog("USB%s Serial: N/A", usb_get_path(phost, NULL, -1));
        Status = USBH_OK;
        phost->EnumState = ENUM_DONE;

#if (USBH_USE_OS == 1U)
        phost->os_msg = (uint32_t)USBH_STATE_CHANGED_EVENT;
#if (osCMSIS < 0x20000U)
        (void)osMessagePut(phost->os_event, phost->os_msg, 0U);
#else
        (void)osMessageQueuePut(phost->os_event, &phost->os_msg, 0U, NULL);
#endif
#endif
      }
      break;

    case ENUM_DONE:
    default:
      break;
  }
  return Status;
}


/**
  * @brief  USBH_LL_SetTimer
  *         Set the initial Host Timer tick
  * @param  phost: Host Handle
  * @retval None
  */
void  USBH_LL_SetTimer(USBH_HandleTypeDef *phost, uint32_t time)
{
  phost->Timer = time;
}


/**
  * @brief  USBH_LL_IncTimer
  *         Increment Host Timer tick
  * @param  phost: Host Handle
  * @retval None
  */
void  USBH_LL_IncTimer(USBH_HandleTypeDef *phost)
{
  phost->Timer ++;
  USBH_HandleSof(phost);
}


/**
  * @brief  USBH_HandleSof
  *         Call SOF process
  * @param  phost: Host Handle
  * @retval None
  */
static void  USBH_HandleSof(USBH_HandleTypeDef *phost)
{
  if ((phost->gState == HOST_CLASS) && (phost->pActiveClass != NULL))
  {
    phost->pActiveClass->SOFProcess(phost);
  }
}


/**
  * @brief  USBH_PortEnabled
  *         Port Enabled
  * @param  phost: Host Handle
  * @retval None
  */
void USBH_LL_PortEnabled(USBH_HandleTypeDef *phost)
{
  phost->device.PortEnabled = 1U;

#if (USBH_USE_OS == 1U)
  phost->os_msg = (uint32_t)USBH_PORT_EVENT;
#if (osCMSIS < 0x20000U)
  (void)osMessagePut(phost->os_event, phost->os_msg, 0U);
#else
  (void)osMessageQueuePut(phost->os_event, &phost->os_msg, 0U, NULL);
#endif
#endif

  return;
}


/**
  * @brief  USBH_LL_PortDisabled
  *         Port Disabled
  * @param  phost: Host Handle
  * @retval None
  */
void USBH_LL_PortDisabled(USBH_HandleTypeDef *phost)
{
  phost->device.PortEnabled = 0U;

  return;
}


/**
  * @brief  HCD_IsPortEnabled
  *         Is Port Enabled
  * @param  phost: Host Handle
  * @retval None
  */
uint8_t USBH_IsPortEnabled(USBH_HandleTypeDef *phost)
{
  return (phost->device.PortEnabled);
}


/**
  * @brief  USBH_LL_Connect
  *         Handle USB Host connexion event
  * @param  phost: Host Handle
  * @retval USBH_Status
  */
USBH_StatusTypeDef  USBH_LL_Connect(USBH_HandleTypeDef *phost)
{
  phost->device.is_connected = 1U;
  phost->device.is_disconnected = 0U;
  phost->device.is_ReEnumerated = 0U;

#if (USBH_USE_OS == 1U)
  phost->os_msg = (uint32_t)USBH_PORT_EVENT;
#if (osCMSIS < 0x20000U)
  (void)osMessagePut(phost->os_event, phost->os_msg, 0U);
#else
  (void)osMessageQueuePut(phost->os_event, &phost->os_msg, 0U, NULL);
#endif
#endif

  return USBH_OK;
}


/**
  * @brief  USBH_LL_Disconnect
  *         Handle USB Host disconnection event
  * @param  phost: Host Handle
  * @retval USBH_Status
  */
USBH_StatusTypeDef  USBH_LL_Disconnect(USBH_HandleTypeDef *phost)
{
  USBH_UsrLog("\rUSBH_LL_Disconnect [%d]", phost->address);

  /* Always select the root device (mainly if its below a hub) */
  phost = USBH_get_root_device(phost);
  USBH_switch_to_dev(phost);

  /* update device connection states */
  phost->device.is_disconnected = 1U;
  phost->device.is_connected = 0U;
  phost->device.PortEnabled = 0U;
  memset(&phost->device.DevDesc, 0, sizeof (phost->device.DevDesc));
  memset(&phost->device.CfgDesc, 0, sizeof (phost->device.CfgDesc));
  memset(&phost->device.manufacturer_string, 0, sizeof (phost->device.manufacturer_string));
  memset(&phost->device.product_string, 0, sizeof (phost->device.product_string));

  /* Stop Host */
  USBH_LL_Stop(phost);

  /* FRee Control Pipes */
  USBH_FreePipe(phost, phost->Control.pipe_in);
  USBH_FreePipe(phost, phost->Control.pipe_out);
#if (USBH_USE_OS == 1U)
  phost->os_msg = (uint32_t)USBH_PORT_EVENT;
#if (osCMSIS < 0x20000U)
  (void)osMessagePut(phost->os_event, phost->os_msg, 0U);
#else
  (void)osMessageQueuePut(phost->os_event, &phost->os_msg, 0U, NULL);
#endif
#endif

  return USBH_OK;
}


#if (USBH_USE_OS == 1U)
/**
  * @brief  USB Host Thread task
  * @param  pvParameters not used
  * @retval None
  */

#if (osCMSIS < 0x20000U)
static void USBH_Process_OS(void const *argument)
{
  osEvent event;

  for (;;)
  {
    event = osMessageGet(((USBH_HandleTypeDef *)argument)->os_event, osWaitForever);
    if (event.status == osEventMessage)
    {
      USBH_Process((USBH_HandleTypeDef *)argument);
    }
  }
}
#else
static void USBH_Process_OS(void *argument)
{
  osStatus_t status;

  for (;;)
  {
    status = osMessageQueueGet(((USBH_HandleTypeDef *)argument)->os_event,
                               &((USBH_HandleTypeDef *)argument)->os_msg, NULL, osWaitForever);
    if (status == osOK)
    {
      USBH_Process((USBH_HandleTypeDef *)argument);
    }
  }
}
#endif /* (osCMSIS < 0x20000U) */


/**
* @brief  USBH_LL_NotifyURBChange
*         Notify URB state Change
* @param  phost: Host handle
* @retval USBH Status
*/
USBH_StatusTypeDef  USBH_LL_NotifyURBChange(USBH_HandleTypeDef *phost)
{
  phost->os_msg = (uint32_t)USBH_PORT_EVENT;

#if (osCMSIS < 0x20000U)
  (void)osMessagePut(phost->os_event, phost->os_msg, 0U);
#else
  (void)osMessageQueuePut(phost->os_event, &phost->os_msg, 0U, NULL);
#endif

  return USBH_OK;
}
#endif
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
