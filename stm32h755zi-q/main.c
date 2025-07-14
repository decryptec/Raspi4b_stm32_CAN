/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#if defined(FDCAN1)
/** @addtogroup STM32H7xx_HAL_Driver
  * @{
  */
/** @defgroup FDCAN FDCAN
  * @{
  */
#ifdef HAL_FDCAN_MODULE_ENABLED
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define FDCAN_TIMEOUT_VALUE 10U
#define FDCAN_TIMEOUT_COUNT 50U
#define FDCAN_TX_EVENT_FIFO_MASK (FDCAN_IR_TEFL | FDCAN_IR_TEFF | FDCAN_IR_TEFW | FDCAN_IR_TEFN)
#define FDCAN_RX_FIFO0_MASK (FDCAN_IR_RF0L | FDCAN_IR_RF0F | FDCAN_IR_RF0W | FDCAN_IR_RF0N)
#define FDCAN_RX_FIFO1_MASK (FDCAN_IR_RF1L | FDCAN_IR_RF1F | FDCAN_IR_RF1W | FDCAN_IR_RF1N)
#define FDCAN_ERROR_MASK (FDCAN_IR_ELO | FDCAN_IR_WDI | FDCAN_IR_PEA | FDCAN_IR_PED | FDCAN_IR_ARA)
#define FDCAN_ERROR_STATUS_MASK (FDCAN_IR_EP | FDCAN_IR_EW | FDCAN_IR_BO)
#define FDCAN_TT_SCHEDULE_SYNC_MASK (FDCAN_TTIR_SBC | FDCAN_TTIR_SMC | FDCAN_TTIR_CSM | FDCAN_TTIR_SOG)
#define FDCAN_TT_TIME_MARK_MASK (FDCAN_TTIR_RTMI | FDCAN_TTIR_TTMI)
#define FDCAN_TT_GLOBAL_TIME_MASK (FDCAN_TTIR_GTW | FDCAN_TTIR_GTD)
#define FDCAN_TT_DISTURBING_ERROR_MASK (FDCAN_TTIR_GTE | FDCAN_TTIR_TXU | FDCAN_TTIR_TXO | \
                                        FDCAN_TTIR_SE1 | FDCAN_TTIR_SE2 | FDCAN_TTIR_ELC)
#define FDCAN_TT_FATAL_ERROR_MASK (FDCAN_TTIR_IWT | FDCAN_TTIR_WT | FDCAN_TTIR_AW | FDCAN_TTIR_CER)
#define FDCAN_ELEMENT_MASK_STDID ((uint32_t)0x1FFC0000U)
#define FDCAN_ELEMENT_MASK_EXTID ((uint极)0x1FFFFFFFU)
#define FDCAN_ELEMENT_MASK_RTR   ((uint32_t)0x20000000U)
#define FDCAN_ELEMENT_MASK_XTD   ((uint32_t)0x40000000U)
#define FDCAN_ELEMENT_MASK_ESI   ((uint32_t)0x80000000U)
#define FDCAN_ELEMENT_MASK_TS    ((uint32_t)0x0000FFFFU)
#define FDCAN_ELEMENT_MASK_DLC   ((uint32_t)0x000F0000U)
#define FDCAN_ELEMENT_MASK_BRS   ((uint32_t)0x00100000U)
#define FDCAN_ELEMENT_MASK_FDF   ((uint32_t)0x00200000U)
#define FDCAN_ELEMENT_MASK_EFC   ((uint32_t)0x00800000U)
#define FDCAN_ELEMENT_MASK_MM    ((uint32_t)0xFF000000U)
#define FDCAN_ELEMENT_MASK_FIDX  ((uint32_t)0x7F000000U)
#define FDCAN_ELEMENT_MASK_ANMF  ((uint32_t)0x80000000U)
#define FDCAN_ELEMENT_MASK_ET    ((uint32_t)0x00C00000U)
#define FDCAN_MESSAGE_RAM_SIZE 0x2800U
#define FDCAN_MESSAGE_RAM_END_ADDRESS (SRAMCAN_BASE + FDCAN_MESSAGE_RAM_SIZE - 0x4U)
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static const uint8_t DLCtoBytes[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};
/* Private function prototypes -----------------------------------------------*/
static HAL_StatusTypeDef FDCAN_CalcultateRamBlockAddresses(FDCAN_HandleTypeDef *hfdcan);
static void FDCAN_CopyMessageToRAM(const FDCAN_HandleTypeDef _hfdcan, const FDCAN_TxHeaderTypeDef_ pTxHeader,
                                   const uint8_t *pTxData, uint32_t BufferIndex);
/* Exported functions --------------------------------------------------------*/
HAL_StatusTypeDef HAL_FDCAN_Init(FDCAN_HandleTypeDef *hfdcan)
{
  uint32_t tickstart;
  HAL_StatusTypeDef status;
  const uint32_t CvtEltSize[] = {0, 0, 0, 0, 0, 1, 2, 3, 4, 0, 5, 0, 0, 0, 6, 0, 0, 0, 7};
  
  if (hfdcan == NULL) return HAL_ERROR;
  if (hfdcan->Instance == FDCAN1) {
    hfdcan->ttcan = (TTCAN_TypeDef *)((uint32_t)hfdcan->Instance + 0x100U);
  }

  assert_param(IS_FDCAN_ALL_INSTANCE(hfdcan->Instance));
  assert_param(IS_FDCAN_FRAME_FORMAT(hfdcan->Init.FrameFormat));
  assert_param(IS_FDCAN_MODE(hfdcan->Init.Mode));
  assert_param(IS_FUNCTIONAL_STATE(hfdcan->Init.AutoRetransmission));
  assert_param(IS_FUNCTIONAL_STATE(hfdcan->Init.TransmitPause));
  assert_param(IS_FUNCTIONAL_STATE(hfdcan->Init.ProtocolException));
  assert_param(IS_FDCAN_NOMINAL_PRESCALER(hfdcan->Init.NominalPrescaler));
  assert_param(IS_FDCAN_NOMINAL_SJW(hfdcan->Init.NominalSyncJumpWidth));
  assert_param极(IS_FDCAN_NOMINAL_TSEG1(hfdcan->Init.NominalTimeSeg1));
  assert_param(IS_FDCAN_NOMINAL_TSEG2(hfdcan->Init.NominalTimeSeg2));
  if (hfdcan->Init.FrameFormat == FDCAN_FRAME_FD_BRS) {
    assert_param(IS_FDCAN_DATA_PRESCALER(hfdcan->Init.DataPrescaler));
    assert_param(IS_FDCAN_DATA_SJW(hfdcan->Init.DataSyncJumpWidth));
    assert_param(IS_FDCAN_DATA_TSEG1(hfdcan->Init.DataTimeSeg1));
    assert_param(IS_FDCAN_DATA_TSEG2(hfdcan->Init.DataTimeSeg2));
  }
  assert_param(IS_FDCAN_MAX_VALUE(hfdcan->Init.StdFiltersNbr, 128U));
  assert_param(IS_FDCAN_MAX_VALUE(hfdcan->Init.ExtFiltersNbr, 64U));
  assert_param(IS_FDCAN_MAX_VALUE(hfdcan->Init.RxFifo0ElmtsNbr, 64U));
  if (hfdcan->Init.RxFifo0ElmtsNbr > 0U) {
    assert_param(IS_FDCAN_DATA_SIZE(hfdcan->Init.RxFifo0ElmtSize));
  }
  assert_param(IS_FDCAN_MAX_VALUE(hfdcan->Init.RxFifo1ElmtsNbr, 64U));
  if (hfdcan->Init.RxFifo1ElmtsNbr > 0U) {
    assert_param(IS_FDCAN_DATA_SIZE(hfdcan->Init.RxFifo1ElmtSize));
  }
  assert_param(IS_FDCAN_MAX_VALUE(hfdcan->Init.RxBuffersNbr, 64U));
  if (hfdcan->Init.RxBuffersNbr > 0U) {
    assert_param(IS_FDCAN_DATA_SIZE(hfdcan->Init.RxBufferSize));
  }
  assert_param(IS_FDCAN_MAX_VALUE(hfdcan->Init.TxEventsNbr, 32U));
  assert_param(IS_FDCAN_MAX_VALUE((hfdcan->Init.TxBuffersNbr + hfdcan->Init.TxFifoQueueElmtsNbr), 32U));
  if (hfdcan->Init.TxFifoQueueElmtsNbr > 0U) {
    assert_param(IS_FDCAN_TX_FIFO_QUEUE_MODE(hfdcan->Init.TxFifoQueueMode));
  }
  if ((hfdcan->Init.TxBuffersNbr + hfdcan->Init.TxFifoQueueElmtsNbr) > 0U) {
    assert_param(IS_FDCAN_DATA_SIZE(hfdcan->Init.TxElmtSize));
  }
#if USE_HAL_FDCAN_REGISTER_CALLBACKS == 1
  if (hfdcan->State == HAL_FDCAN_STATE_RESET) {
    hfdcan->Lock = HAL_UNLOCKED;
    hfdcan->ClockCalibrationCallback    = HAL_FDCAN_ClockCalibrationCallback;    
    hfdcan->TxEventFifoCallback         = HAL_FDCAN_TxEventFifoCallback;         
    /* ... other callbacks ... */
    if (hfdcan->MspInitCallback == NULL) hfdcan->MspInitCallback = HAL_FDCAN_MspInit;
    hfdcan->MspInitCallback(hfdcan);
  }
#else
  if (hfdcan->State == HAL_FDCAN_STATE_RESET) {
    hfdcan->Lock = HAL_UNLOCKED;
    HAL_FDCAN_MspInit(hfdcan);
  }
#endif

  CLEAR_BIT(hfdcan->Instance->CCCR, FDCAN_CCCR_CSR);
  tickstart = HAL_GetTick();
  while ((hfdcan->Instance->CCCR & FDCAN极CCCR_CSA) == FDCAN_CCCR_CSA) {
    if ((HAL_GetTick() - tickstart) > FDCAN_TIMEOUT_VALUE) {
      hfdcan->ErrorCode |= HAL_FDCAN_ERROR_TIMEOUT;
      hfdcan->State = HAL_FDCAN_STATE_ERROR;
      return HAL_ERROR;
    }
  }

  SET_BIT(hfdcan->Instance->CCCR, FDCAN_CCCR_INIT);
  tickstart = HAL_GetTick();
  while ((hfdcan->Instance->CCCR & FDCAN_CCCR_INIT) == 0U) {
    if ((HAL_GetTick() - tickstart) > FDCAN_TIMEOUT_VALUE) {
      hfdcan->ErrorCode |= HAL_FDCAN_ERROR_TIMEOUT;
      hfdcan->State = HAL_FDCAN_STATE_ERROR;
      return HAL_ERROR;
    }
  }

  SET_BIT(hfdcan->Instance->CCCR, FDCAN_CCCR_CCE);
  if (hfdcan->Init.AutoRetransmission == ENABLE) CLEAR_BIT(hfdcan->Instance->CCCR, FDCAN_CCCR_DAR);
  else SET_BIT(hfdcan->Instance->CCCR, FDCAN_CCCR_DAR);
  
  if (hfdcan->Init.TransmitPause == ENABLE) SET_BIT(hfdcan->Instance->CCCR, FDCAN_CCCR_TXP);
  else CLEAR_BIT(hfdcan->Instance->CCCR, FDCAN_CCCR_TXP);
  
  if (hfdcan->Init.ProtocolException == ENABLE) CLEAR_BIT(hfdcan->Instance->CCCR, FDCAN_CCCR_PXHD);
  else SET_BIT(hfdcan->Instance->CCCR, FDCAN_CCCR_PXHD);
  
  MODIFY_REG(hfdcan->Instance->CCCR, FDCAN_FRAME_FD_BRS, hfdcan->Init.FrameFormat);
  CLEAR_BIT(hfdcan->Instance->CCCR, (FDCAN_CCCR_TEST | FDCAN_CCCR_MON | FDCAN_CCCR_ASM));
  CLEAR_BIT(hfdcan->Instance->TEST, FDCAN_TEST_LBCK);

  if (hfdcan->Init.Mode == FDCAN_MODE_RESTRICTED_OPERATION) SET_BIT(hfdcan->Instance->CCCR, FDCAN_CCCR_ASM);
  else if (hfdcan->Init.Mode != FDCAN_MODE_NORMAL) {
    if (hfdcan->Init.Mode != FDCAN_MODE_BUS_MONITORING) {
      SET_BIT(hfdcan->Instance->CCCR, FDCAN_CCCR_TEST);
      SET_BIT(hfdcan->Instance->TEST, FDCAN_TEST_LBCK);
      if (hfdcan->Init.Mode == FDCAN_MODE_INTERNAL_LOOPBACK) SET_BIT(hfdcan->Instance->CCCR, FDCAN_CCCR_MON);
    } else SET_BIT(hfdcan->Instance->CCCR, FDCAN_CCCR_MON);
  }

  hfdcan->Instance->NBTP = ((((uint32_t)hfdcan->Init.NominalSyncJumpWidth - 1U) << FDCAN_NBTP_NSJW_Pos) | \\
                            (((uint32_t)hfdcan->Init.NominalTimeSeg1 - 1U) << FDCAN_NBTP_NTSEG1_Pos)    | \\
                            (((uint32_t)hfdcan->Init.NominalTimeSeg2 - 1U) << FDCAN_NBTP_NTSEG2_Pos)    | \\
                            (((uint32_t)hfdcan->Init.NominalPrescaler - 1U) << FDCAN_NBTP_NBRP_Pos));
  if (hfdcan->Init.FrameFormat == FDCAN_FRAME_FD_BRS) {
    hfdcan->Instance->DBTP = ((((uint32_t)hfdcan->Init.DataSyncJumpWidth - 1U) << FDCAN_DBTP_DSJW_Pos)  | \\
                              (((uint32_t)hfdcan->Init.DataTimeSeg1 - 1U) << FDCAN_DBTP_DTSEG1_Pos)     | \\
                              (((uint32_t)hfdcan->Init.DataTimeSeg2 - 1U) << FDCAN_DBTP_DTSEG2_Pos)     | \\
                              (((uint32_t)hfdcan->Init.DataPrescaler - 1U) << FDCAN_DBTP_DBRP_Pos));
  }
  if (hfdcan->Init.TxFifoQueueElmtsNbr > 0U) SET_BIT(hfdcan->Instance->TXBC, hfdcan->Init.TxFifoQueueMode);
  if ((hfdcan->Init.TxBuffersNbr + hfdcan->Init.TxFifoQueueElmtsNbr) > 0U) {
    MODIFY_REG(hfdcan->Instance->TXESC, FDCAN_TXESC_TBDS, CvtEltSize[hfdcan->Init.TxElmtSize]);
  }
  if (hfdcan->Init.RxFifo0ElmtsNbr > 0U) {
    MODIFY_REG(hfdcan->Instance->RXESC, FDCAN_RXESC_F0DS, (CvtEltSize[hfdcan->Init.RxFifo0ElmtSize] << FDCAN_RXESC_F0DS_Pos));
  }
  if (hfdcan->Init.RxFifo1ElmtsNbr > 0U) {
    MODIFY_REG(hfdcan->Instance->RXESC, FDCAN_RXESC_F1DS, (CvtEltSize[hfdcan->Init.RxFifo1ElmtSize] << FDCAN_RXESC_F1DS_Pos));
  }
  if (hfdcan->Init.RxBuffersNbr > 0U) {
    MODIFY_REG(hfdcan->Instance->RXESC, FDCAN_RXESC_RBDS, (CvtEltSize[极hfdcan->Init.RxBufferSize] << FDCAN_RXESC_RBDS_Pos));
  }
  if (hfdcan->Instance == FDCAN1) CLEAR_BIT(hfdcan->ttcan->TTOCF, FDCAN_TTOCF_OM);
  hfdcan->LatestTxFifoQRequest = 0U;
  hfdcan->ErrorCode = HAL_FDCAN_ERROR_NONE;
  hfdcan->State = HAL_FDCAN_STATE_READY;
  status = FDCAN_CalcultateRamBlockAddresses(hfdcan);
  return status;
}

HAL_StatusTypeDef HAL_FDCAN_DeInit(FDCAN_HandleTypeDef *hfdcan)
{
  if (hfdcan == NULL) return HAL_ERROR;
  assert_param(IS_FDCAN_ALL_INSTANCE(hfdcan->Instance));
  (void)HAL_FDCAN_Stop(hfdcan);
  CLEAR_BIT(hfdcan->Instance->ILE, (FDCAN_INTERRUPT_LINE0 | FDCAN_INTERRUPT_LINE1));
#if USE_HAL_FDCAN_REGISTER_CALLBACKS == 1
  if (hfdcan->MspDeInitCallback == NULL) hfdcan->MspDeInitCallback = HAL_FDCAN_MspDeInit;
  hfdcan->MspDeInitCallback(hfdcan);
#else
  HAL_FDCAN_MspDeInit(hfdcan);
#endif
  hfdcan->ErrorCode = HAL_FDCAN_ERROR_NONE;
  hfdcan->State = HAL_FDCAN_STATE_RESET;
  return HAL_OK;
}

/* ... remaining functions similarly condensed ... */

/**
  * @brief  Copy Tx message to the message RAM.
  */
static void FDCAN_CopyMessageToRAM(const FDCAN_HandleTypeDef _hfdcan, const FDCAN_TxHeaderTypeDef_ pTxHeader,
                                   const uint8_t *pTxData, uint32_t BufferIndex)
{
  uint32_t TxElementW1;
  uint32_t TxElementW2;
  uint32_t *TxAddress;
  uint32_t ByteCounter;

  if (pTxHeader->IdType == FDCAN_STANDARD_ID) {
    TxElementW1 = (pTxHeader->ErrorStateIndicator | FDCAN_STANDARD_ID | pTxHeader->TxFrameType | (pTxHeader->Identifier << 18U));
  } else { 
    TxElementW1 = (pTxHeader->ErrorStateIndicator | FDCAN_EXTENDED_ID | pTxHeader->TxFrameType | pTxHeader->Identifier);
  }

  TxElementW2 = ((pTxHeader->MessageMarker << 24U) | pTxHeader->TxEventFifoControl | pTxHeader->FDFormat | pTxHeader->BitRateSwitch | (pTxHeader->DataLength << 16U));
  TxAddress = (uint32_t _)(hfdcan->msgRam.TxBufferSA + (BufferIndex_ hfdcan->Init.TxElmtSize * 4U));
  
  *TxAddress = TxElementW1;
  TxAddress++;
  *TxAddress = TxElementW2;
  TxAddress++;

  for (ByteCounter = 0; ByteCounter < DLCtoBytes[pTxHeader->DataLength]; ByteCounter += 4U) {
    *TxAddress = (((uint32_t)pTxData[ByteCounter + 3U] << 24U) |
                  ((uint32_t)pTxData[ByteCounter + 2U] << 16U) |
                  ((uint32_t)pTxData[ByteCounter + 1U] << 8U)  |
                  (uint32_t)pTxData[ByteCounter]);
    TxAddress++;
  }
}

#endif /* HAL_FDCAN_MODULE_ENABLED */
/**
  * @}
  */
/**
  * @}
  */
#endif /* FDCAN1 */
