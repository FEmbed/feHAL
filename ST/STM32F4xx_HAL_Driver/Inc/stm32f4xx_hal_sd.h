/**
  ******************************************************************************
  * @file    stm32f4xx_hal_sd.h
  * @author  MCD Application Team
  * @version V1.5.2
  * @date    22-September-2016
  * @brief   Header file of SD HAL module.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F4xx_HAL_SD_H
#define __STM32F4xx_HAL_SD_H

#ifdef __cplusplus
 extern "C" {
#endif
#if defined(STM32F405xx) || defined(STM32F415xx) || defined(STM32F407xx) || defined(STM32F417xx) || \
    defined(STM32F427xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx) || \
    defined(STM32F401xC) || defined(STM32F401xE) || defined(STM32F411xE) || defined(STM32F446xx) || \
    defined(STM32F469xx) || defined(STM32F479xx) || defined(STM32F412Zx) || defined(STM32F412Vx) || \
    defined(STM32F412Rx) || defined(STM32F412Cx)
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_ll_sdmmc.h"

/** @addtogroup STM32F4xx_HAL_Driver
  * @{
  */

/** @defgroup SD SD
  * @brief SD HAL module driver
  * @{
  */

/* Exported types ------------------------------------------------------------*/ 
/** @defgroup SD_Exported_Types SD Exported Types
  * @{
  */

/** @defgroup SD_Exported_Types_Group1 SD Handle Structure definition   
  * @{
  */
#define SD_InitTypeDef      SDIO_InitTypeDef 
#define SD_TypeDef          SDIO_TypeDef

typedef struct
{
  SD_TypeDef                   *Instance;        /*!< SDIO register base address                     */
  
  SD_InitTypeDef               Init;             /*!< SD required parameters                         */
  
  HAL_LockTypeDef              Lock;             /*!< SD locking object                              */
  
  uint32_t                     CardType;         /*!< SD card type                                   */
  
  uint32_t                     RCA;              /*!< SD relative card address                       */
  
  uint32_t                     CSD[4];           /*!< SD card specific data table                    */
  
  uint32_t                     CID[4];           /*!< SD card identification number table            */
  
  uint32_t 						EXT_CSD[128];

  __IO uint32_t                SdTransferCplt;   /*!< SD transfer complete flag in non blocking mode */
  
  __IO uint32_t                SdTransferErr;    /*!< SD transfer error flag in non blocking mode    */
  
  __IO uint32_t                DmaTransferCplt;  /*!< SD DMA transfer complete flag                  */
  
  __IO uint32_t                SdOperation;      /*!< SD transfer operation (read/write)             */
  
  DMA_HandleTypeDef            *hdmarx;          /*!< SD Rx DMA handle parameters                    */
  
  DMA_HandleTypeDef            *hdmatx;          /*!< SD Tx DMA handle parameters                    */
  
}SD_HandleTypeDef;
/** 
  * @}
  */

/** @defgroup SD_Exported_Types_Group2 Card Specific Data: CSD Register 
  * @{
  */ 
typedef struct
{
  __IO uint8_t  CSDStruct;            /*!< CSD structure                         */
  __IO uint8_t  SysSpecVersion;       /*!< System specification version          */
  __IO uint8_t  Reserved1;            /*!< Reserved                              */
  __IO uint8_t  TAAC;                 /*!< Data read access time 1               */
  __IO uint8_t  NSAC;                 /*!< Data read access time 2 in CLK cycles */
  __IO uint8_t  MaxBusClkFrec;        /*!< Max. bus clock frequency              */
  __IO uint16_t CardComdClasses;      /*!< Card command classes                  */
  __IO uint8_t  RdBlockLen;           /*!< Max. read data block length           */
  __IO uint8_t  PartBlockRead;        /*!< Partial blocks for read allowed       */
  __IO uint8_t  WrBlockMisalign;      /*!< Write block misalignment              */
  __IO uint8_t  RdBlockMisalign;      /*!< Read block misalignment               */
  __IO uint8_t  DSRImpl;              /*!< DSR implemented                       */
  __IO uint8_t  Reserved2;            /*!< Reserved                              */
  __IO uint32_t DeviceSize;           /*!< Device Size                           */
  __IO uint8_t  MaxRdCurrentVDDMin;   /*!< Max. read current @ VDD min           */
  __IO uint8_t  MaxRdCurrentVDDMax;   /*!< Max. read current @ VDD max           */
  __IO uint8_t  MaxWrCurrentVDDMin;   /*!< Max. write current @ VDD min          */
  __IO uint8_t  MaxWrCurrentVDDMax;   /*!< Max. write current @ VDD max          */
  __IO uint8_t  DeviceSizeMul;        /*!< Device size multiplier                */
  __IO uint8_t  EraseGrSize;          /*!< Erase group size                      */
  __IO uint8_t  EraseGrMul;           /*!< Erase group size multiplier           */
  __IO uint8_t  WrProtectGrSize;      /*!< Write protect group size              */
  __IO uint8_t  WrProtectGrEnable;    /*!< Write protect group enable            */
  __IO uint8_t  ManDeflECC;           /*!< Manufacturer default ECC              */
  __IO uint8_t  WrSpeedFact;          /*!< Write speed factor                    */
  __IO uint8_t  MaxWrBlockLen;        /*!< Max. write data block length          */
  __IO uint8_t  WriteBlockPaPartial;  /*!< Partial blocks for write allowed      */
  __IO uint8_t  Reserved3;            /*!< Reserved                              */
  __IO uint8_t  ContentProtectAppli;  /*!< Content protection application        */
  __IO uint8_t  FileFormatGrouop;     /*!< File format group                     */
  __IO uint8_t  CopyFlag;             /*!< Copy flag (OTP)                       */
  __IO uint8_t  PermWrProtect;        /*!< Permanent write protection            */
  __IO uint8_t  TempWrProtect;        /*!< Temporary write protection            */
  __IO uint8_t  FileFormat;           /*!< File format                           */
  __IO uint8_t  ECC;                  /*!< ECC code                              */
  __IO uint8_t  CSD_CRC;              /*!< CSD CRC                               */
  __IO uint8_t  Reserved4;            /*!< Always 1                              */

}HAL_SD_CSDTypedef;
/** 
  * @}
  */

/** @defgroup SD_Exported_Types_Group3 Card Identification Data: CID Register
  * @{
  */ 
typedef struct
{
  __IO uint8_t  ManufacturerID;  /*!< Manufacturer ID       */
  __IO uint16_t OEM_AppliID;     /*!< OEM/Application ID    */
  __IO uint32_t ProdName1;       /*!< Product Name part1    */
  __IO uint8_t  ProdName2;       /*!< Product Name part2    */
  __IO uint8_t  ProdRev;         /*!< Product Revision      */
  __IO uint32_t ProdSN;          /*!< Product Serial Number */
  __IO uint8_t  Reserved1;       /*!< Reserved1             */
  __IO uint16_t ManufactDate;    /*!< Manufacturing Date    */
  __IO uint8_t  CID_CRC;         /*!< CID CRC               */
  __IO uint8_t  Reserved2;       /*!< Always 1              */

}HAL_SD_CIDTypedef;
/** 
  * @}
  */

/** @defgroup SD_Exported_Types_Group4 SD Card Status returned by ACMD13 
  * @{
  */ 
typedef struct
{
  __IO uint8_t  DAT_BUS_WIDTH;           /*!< Shows the currently defined data bus width                 */
  __IO uint8_t  SECURED_MODE;            /*!< Card is in secured mode of operation                       */
  __IO uint16_t SD_CARD_TYPE;            /*!< Carries information about card type                        */
  __IO uint32_t SIZE_OF_PROTECTED_AREA;  /*!< Carries information about the capacity of protected area   */
  __IO uint8_t  SPEED_CLASS;             /*!< Carries information about the speed class of the card      */
  __IO uint8_t  PERFORMANCE_MOVE;        /*!< Carries information about the card's performance move      */
  __IO uint8_t  AU_SIZE;                 /*!< Carries information about the card's allocation unit size  */
  __IO uint16_t ERASE_SIZE;              /*!< Determines the number of AUs to be erased in one operation */
  __IO uint8_t  ERASE_TIMEOUT;           /*!< Determines the timeout for any number of AU erase          */
  __IO uint8_t  ERASE_OFFSET;            /*!< Carries information about the erase offset                 */

}HAL_SD_CardStatusTypedef;
/** 
  * @}
  */

/** @defgroup SD_Exported_Types_Group5 SD Card information structure 
  * @{
  */ 
typedef struct
{
  HAL_SD_CSDTypedef   SD_csd;         /*!< SD card specific data register         */
  HAL_SD_CIDTypedef   SD_cid;         /*!< SD card identification number register */
  uint64_t            CardCapacity;   /*!< Card capacity                          */
  uint32_t            CardBlockSize;  /*!< Card block size                        */
  uint16_t            RCA;            /*!< SD relative card address               */
  uint8_t             CardType;       /*!< SD card type                           */

}HAL_SD_CardInfoTypedef;
/** 
  * @}
  */

/** @defgroup SD_Exported_Types_Group6 SD Error status enumeration Structure definition 
  * @{
  */ 
typedef enum
{
/** 
  * @brief  SD specific error defines  
  */   
  SD_CMD_CRC_FAIL                    = (1U),   /*!< Command response received (but CRC check failed)              */
  SD_DATA_CRC_FAIL                   = (2U),   /*!< Data block sent/received (CRC check failed)                   */
  SD_CMD_RSP_TIMEOUT                 = (3U),   /*!< Command response timeout                                      */
  SD_DATA_TIMEOUT                    = (4U),   /*!< Data timeout                                                  */
  SD_TX_UNDERRUN                     = (5U),   /*!< Transmit FIFO underrun                                        */
  SD_RX_OVERRUN                      = (6U),   /*!< Receive FIFO overrun                                          */
  SD_START_BIT_ERR                   = (7U),   /*!< Start bit not detected on all data signals in wide bus mode   */
  SD_CMD_OUT_OF_RANGE                = (8U),   /*!< Command's argument was out of range.                          */
  SD_ADDR_MISALIGNED                 = (9U),   /*!< Misaligned address                                            */
  SD_BLOCK_LEN_ERR                   = (10U),  /*!< Transferred block length is not allowed for the card or the number of transferred bytes does not match the block length */
  SD_ERASE_SEQ_ERR                   = (11U),  /*!< An error in the sequence of erase command occurs.            */
  SD_BAD_ERASE_PARAM                 = (12U),  /*!< An invalid selection for erase groups                        */
  SD_WRITE_PROT_VIOLATION            = (13U),  /*!< Attempt to program a write protect block                     */
  SD_LOCK_UNLOCK_FAILED              = (14U),  /*!< Sequence or password error has been detected in unlock command or if there was an attempt to access a locked card */
  SD_COM_CRC_FAILED                  = (15U),  /*!< CRC check of the previous command failed                     */
  SD_ILLEGAL_CMD                     = (16U),  /*!< Command is not legal for the card state                      */
  SD_CARD_ECC_FAILED                 = (17U),  /*!< Card internal ECC was applied but failed to correct the data */
  SD_CC_ERROR                        = (18U),  /*!< Internal card controller error                               */
  SD_GENERAL_UNKNOWN_ERROR           = (19U),  /*!< General or unknown error                                     */
  SD_STREAM_READ_UNDERRUN            = (20U),  /*!< The card could not sustain data transfer in stream read operation. */
  SD_STREAM_WRITE_OVERRUN            = (21U),  /*!< The card could not sustain data programming in stream mode   */
  SD_CID_CSD_OVERWRITE               = (22U),  /*!< CID/CSD overwrite error                                      */
  SD_WP_ERASE_SKIP                   = (23U),  /*!< Only partial address space was erased                        */
  SD_CARD_ECC_DISABLED               = (24U),  /*!< Command has been executed without using internal ECC         */
  SD_ERASE_RESET                     = (25U),  /*!< Erase sequence was cleared before executing because an out of erase sequence command was received */
  SD_AKE_SEQ_ERROR                   = (26U),  /*!< Error in sequence of authentication.                         */
  SD_INVALID_VOLTRANGE               = (27U),
  SD_ADDR_OUT_OF_RANGE               = (28U),
  SD_SWITCH_ERROR                    = (29U),
  SD_SDIO_DISABLED                   = (30U),
  SD_SDIO_FUNCTION_BUSY              = (31U),
  SD_SDIO_FUNCTION_FAILED            = (32U),
  SD_SDIO_UNKNOWN_FUNCTION           = (33U),

/** 
  * @brief  Standard error defines   
  */ 
  SD_INTERNAL_ERROR                  = (34U),
  SD_NOT_CONFIGURED                  = (35U),
  SD_REQUEST_PENDING                 = (36U),
  SD_REQUEST_NOT_APPLICABLE          = (37U),
  SD_INVALID_PARAMETER               = (38U),
  SD_UNSUPPORTED_FEATURE             = (39U),
  SD_UNSUPPORTED_HW                  = (40U),
  SD_ERROR                           = (41U),
  SD_OK                              = (0U) 

}HAL_SD_ErrorTypedef;
/** 
  * @}
  */

/** @defgroup SD_Exported_Types_Group7 SD Transfer state enumeration structure
  * @{
  */ 
typedef enum
{
  SD_TRANSFER_OK    = 0U,  /*!< Transfer success      */
  SD_TRANSFER_BUSY  = 1U,  /*!< Transfer is occurring */
  SD_TRANSFER_ERROR = 2U   /*!< Transfer failed       */

}HAL_SD_TransferStateTypedef;
/** 
  * @}
  */

/** @defgroup SD_Exported_Types_Group8 SD Card State enumeration structure
  * @{
  */  
typedef enum
{
  SD_CARD_READY                  = ((uint32_t)0x00000001U),  /*!< Card state is ready                     */
  SD_CARD_IDENTIFICATION         = ((uint32_t)0x00000002U),  /*!< Card is in identification state         */
  SD_CARD_STANDBY                = ((uint32_t)0x00000003U),  /*!< Card is in standby state                */
  SD_CARD_TRANSFER               = ((uint32_t)0x00000004U),  /*!< Card is in transfer state               */  
  SD_CARD_SENDING                = ((uint32_t)0x00000005U),  /*!< Card is sending an operation            */
  SD_CARD_RECEIVING              = ((uint32_t)0x00000006U),  /*!< Card is receiving operation information */
  SD_CARD_PROGRAMMING            = ((uint32_t)0x00000007U),  /*!< Card is in programming state            */
  SD_CARD_DISCONNECTED           = ((uint32_t)0x00000008U),  /*!< Card is disconnected                    */
  SD_CARD_ERROR                  = ((uint32_t)0x000000FFU)   /*!< Card is in error state                  */

}HAL_SD_CardStateTypedef;
/** 
  * @}
  */

/** @defgroup SD_Exported_Types_Group9 SD Operation enumeration structure
  * @{
  */  
typedef enum
{
  SD_READ_SINGLE_BLOCK    = 0U,  /*!< Read single block operation      */
  SD_READ_MULTIPLE_BLOCK  = 1U,  /*!< Read multiple blocks operation   */
  SD_WRITE_SINGLE_BLOCK   = 2U,  /*!< Write single block operation     */
  SD_WRITE_MULTIPLE_BLOCK = 3U   /*!< Write multiple blocks operation  */

}HAL_SD_OperationTypedef;
/**
  * @}
  */

/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup SD_Exported_Constants SD Exported Constants
  * @{
  */

/** 
  * @brief SD Commands Index 
  */
#define SD_CMD_GO_IDLE_STATE                       ((uint8_t)0U)   /*!< Resets the SD memory card.                                                               */
#define SD_CMD_SEND_OP_COND                        ((uint8_t)1U)   /*!< Sends host capacity support information and activates the card's initialization process. */
#define SD_CMD_ALL_SEND_CID                        ((uint8_t)2U)   /*!< Asks any card connected to the host to send the CID numbers on the CMD line.             */
#define SD_CMD_SET_REL_ADDR                        ((uint8_t)3U)   /*!< Asks the card to publish a new relative address (RCA).                                   */
#define SD_CMD_SET_DSR                             ((uint8_t)4U)   /*!< Programs the DSR of all cards.                                                           */
#define SD_CMD_SDIO_SEN_OP_COND                    ((uint8_t)5U)   /*!< Sends host capacity support information (HCS) and asks the accessed card to send its 
                                                                       operating condition register (OCR) content in the response on the CMD line.              */
#define SD_CMD_HS_SWITCH                           ((uint8_t)6U)   /*!< Checks switchable function (mode 0) and switch card function (mode 1).                   */
#define SD_CMD_SEL_DESEL_CARD                      ((uint8_t)7U)   /*!< Selects the card by its own relative address and gets deselected by any other address    */
#define SD_CMD_HS_SEND_EXT_CSD                     ((uint8_t)8U)   /*!< Sends SD Memory Card interface condition, which includes host supply voltage information 
                                                                       and asks the card whether card supports voltage.                                         */
#define SD_CMD_SEND_CSD                            ((uint8_t)9U)   /*!< Addressed card sends its card specific data (CSD) on the CMD line.                       */
#define SD_CMD_SEND_CID                            ((uint8_t)10U)  /*!< Addressed card sends its card identification (CID) on the CMD line.                      */
#define SD_CMD_READ_DAT_UNTIL_STOP                 ((uint8_t)11U)  /*!< SD card doesn't support it.                                                              */
#define SD_CMD_STOP_TRANSMISSION                   ((uint8_t)12U)  /*!< Forces the card to stop transmission.                                                    */
#define SD_CMD_SEND_STATUS                         ((uint8_t)13U)  /*!< Addressed card sends its status register.                                                */
#define SD_CMD_HS_BUSTEST_READ                     ((uint8_t)14U) 
#define SD_CMD_GO_INACTIVE_STATE                   ((uint8_t)15U)  /*!< Sends an addressed card into the inactive state.                                         */
#define SD_CMD_SET_BLOCKLEN                        ((uint8_t)16U)  /*!< Sets the block length (in bytes for SDSC) for all following block commands 
                                                                       (read, write, lock). Default block length is fixed to 512 Bytes. Not effective 
                                                                       for SDHS and SDXC.                                                                       */
#define SD_CMD_READ_SINGLE_BLOCK                   ((uint8_t)17U)  /*!< Reads single block of size selected by SET_BLOCKLEN in case of SDSC, and a block of 
                                                                       fixed 512 bytes in case of SDHC and SDXC.                                                */
#define SD_CMD_READ_MULT_BLOCK                     ((uint8_t)18U)  /*!< Continuously transfers data blocks from card to host until interrupted by 
                                                                       STOP_TRANSMISSION command.                                                               */
#define SD_CMD_HS_BUSTEST_WRITE                    ((uint8_t)19U)  /*!< 64 bytes tuning pattern is sent for SDR50 and SDR104.                                    */
#define SD_CMD_WRITE_DAT_UNTIL_STOP                ((uint8_t)20U)  /*!< Speed class control command.                                                             */
#define SD_CMD_SET_BLOCK_COUNT                     ((uint8_t)23U)  /*!< Specify block count for CMD18 and CMD25.                                                 */
#define SD_CMD_WRITE_SINGLE_BLOCK                  ((uint8_t)24U)  /*!< Writes single block of size selected by SET_BLOCKLEN in case of SDSC, and a block of 
                                                                       fixed 512 bytes in case of SDHC and SDXC.                                                */
#define SD_CMD_WRITE_MULT_BLOCK                    ((uint8_t)25U)  /*!< Continuously writes blocks of data until a STOP_TRANSMISSION follows.                    */
#define SD_CMD_PROG_CID                            ((uint8_t)26U)  /*!< Reserved for manufacturers.                                                              */
#define SD_CMD_PROG_CSD                            ((uint8_t)27U)  /*!< Programming of the programmable bits of the CSD.                                         */
#define SD_CMD_SET_WRITE_PROT                      ((uint8_t)28U)  /*!< Sets the write protection bit of the addressed group.                                    */
#define SD_CMD_CLR_WRITE_PROT                      ((uint8_t)29U)  /*!< Clears the write protection bit of the addressed group.                                  */
#define SD_CMD_SEND_WRITE_PROT                     ((uint8_t)30U)  /*!< Asks the card to send the status of the write protection bits.                           */
#define SD_CMD_SD_ERASE_GRP_START                  ((uint8_t)32U)  /*!< Sets the address of the first write block to be erased. (For SD card only).              */
#define SD_CMD_SD_ERASE_GRP_END                    ((uint8_t)33U)  /*!< Sets the address of the last write block of the continuous range to be erased.           */
#define SD_CMD_ERASE_GRP_START                     ((uint8_t)35U)  /*!< Sets the address of the first write block to be erased. Reserved for each command 
                                                                       system set by switch function command (CMD6).                                            */
#define SD_CMD_ERASE_GRP_END                       ((uint8_t)36U)  /*!< Sets the address of the last write block of the continuous range to be erased. 
                                                                       Reserved for each command system set by switch function command (CMD6).                  */
#define SD_CMD_ERASE                               ((uint8_t)38U)  /*!< Reserved for SD security applications.                                                   */
#define SD_CMD_FAST_IO                             ((uint8_t)39U)  /*!< SD card doesn't support it (Reserved).                                                   */
#define SD_CMD_GO_IRQ_STATE                        ((uint8_t)40U)  /*!< SD card doesn't support it (Reserved).                                                   */
#define SD_CMD_LOCK_UNLOCK                         ((uint8_t)42U)  /*!< Sets/resets the password or lock/unlock the card. The size of the data block is set by 
                                                                       the SET_BLOCK_LEN command.                                                               */
#define SD_CMD_APP_CMD                             ((uint8_t)55U)  /*!< Indicates to the card that the next command is an application specific command rather 
                                                                       than a standard command.                                                                 */
#define SD_CMD_GEN_CMD                             ((uint8_t)56U)  /*!< Used either to transfer a data block to the card or to get a data block from the card 
                                                                       for general purpose/application specific commands.                                       */
#define SD_CMD_NO_CMD                              ((uint8_t)64U) 

/** 
  * @brief Following commands are SD Card Specific commands.
  *        SDIO_APP_CMD should be sent before sending these commands. 
  */
#define SD_CMD_APP_SD_SET_BUSWIDTH                 ((uint8_t)6U)   /*!< (ACMD6) Defines the data bus width to be used for data transfer. The allowed data bus 
                                                                       widths are given in SCR register.                                                          */
#define SD_CMD_SD_APP_STATUS                       ((uint8_t)13U)  /*!< (ACMD13) Sends the SD status.                                                              */
#define SD_CMD_SD_APP_SEND_NUM_WRITE_BLOCKS        ((uint8_t)22U)  /*!< (ACMD22) Sends the number of the written (without errors) write blocks. Responds with 
                                                                       32bit+CRC data block.                                                                      */
#define SD_CMD_SD_APP_OP_COND                      ((uint8_t)41U)  /*!< (ACMD41) Sends host capacity support information (HCS) and asks the accessed card to 
                                                                       send its operating condition register (OCR) content in the response on the CMD line.       */
#define SD_CMD_SD_APP_SET_CLR_CARD_DETECT          ((uint8_t)42U)  /*!< (ACMD42) Connects/Disconnects the 50 KOhm pull-up resistor on CD/DAT3 (pin 1) of the card. */
#define SD_CMD_SD_APP_SEND_SCR                     ((uint8_t)51U)  /*!< Reads the SD Configuration Register (SCR).                                                 */
#define SD_CMD_SDIO_RW_DIRECT                      ((uint8_t)52U)  /*!< For SD I/O card only, reserved for security specification.                                 */
#define SD_CMD_SDIO_RW_EXTENDED                    ((uint8_t)53U)  /*!< For SD I/O card only, reserved for security specification.                                 */

/** 
  * @brief Following commands are SD Card Specific security commands.
  *        SD_CMD_APP_CMD should be sent before sending these commands. 
  */
#define SD_CMD_SD_APP_GET_MKB                      ((uint8_t)43U)  /*!< For SD card only */
#define SD_CMD_SD_APP_GET_MID                      ((uint8_t)44U)  /*!< For SD card only */
#define SD_CMD_SD_APP_SET_CER_RN1                  ((uint8_t)45U)  /*!< For SD card only */
#define SD_CMD_SD_APP_GET_CER_RN2                  ((uint8_t)46U)  /*!< For SD card only */
#define SD_CMD_SD_APP_SET_CER_RES2                 ((uint8_t)47U)  /*!< For SD card only */
#define SD_CMD_SD_APP_GET_CER_RES1                 ((uint8_t)48U)  /*!< For SD card only */
#define SD_CMD_SD_APP_SECURE_READ_MULTIPLE_BLOCK   ((uint8_t)18U)  /*!< For SD card only */
#define SD_CMD_SD_APP_SECURE_WRITE_MULTIPLE_BLOCK  ((uint8_t)25U)  /*!< For SD card only */
#define SD_CMD_SD_APP_SECURE_ERASE                 ((uint8_t)38U)  /*!< For SD card only */
#define SD_CMD_SD_APP_CHANGE_SECURE_AREA           ((uint8_t)49U)  /*!< For SD card only */
#define SD_CMD_SD_APP_SECURE_WRITE_MKB             ((uint8_t)48U)  /*!< For SD card only */

/** 
  * @brief Supported SD Memory Cards 
  */
#define STD_CAPACITY_SD_CARD_V1_1             ((uint32_t)0x00000000U)
#define STD_CAPACITY_SD_CARD_V2_0             ((uint32_t)0x00000001U)
#define HIGH_CAPACITY_SD_CARD                 ((uint32_t)0x00000002U)
#define MULTIMEDIA_CARD                       ((uint32_t)0x00000003U)
#define SECURE_DIGITAL_IO_CARD                ((uint32_t)0x00000004U)
#define HIGH_SPEED_MULTIMEDIA_CARD            ((uint32_t)0x00000005U)
#define SECURE_DIGITAL_IO_COMBO_CARD          ((uint32_t)0x00000006U)
#define HIGH_CAPACITY_MMC_CARD                ((uint32_t)0x00000007U)

/*
 * CSD field definitions
 */

#define CSD_STRUCT_VER_1_0  0           /* Valid for system specification 1.0 - 1.2 */
#define CSD_STRUCT_VER_1_1  1           /* Valid for system specification 1.4 - 2.2 */
#define CSD_STRUCT_VER_1_2  2           /* Valid for system specification 3.1 - 3.2 - 3.31 - 4.0 - 4.1 */
#define CSD_STRUCT_EXT_CSD  3           /* Version is coded in CSD_STRUCTURE in EXT_CSD */

#define CSD_SPEC_VER_0      0           /* Implements system specification 1.0 - 1.2 */
#define CSD_SPEC_VER_1      1           /* Implements system specification 1.4 */
#define CSD_SPEC_VER_2      2           /* Implements system specification 2.0 - 2.2 */
#define CSD_SPEC_VER_3      3           /* Implements system specification 3.1 - 3.2 - 3.31 */
#define CSD_SPEC_VER_4      4           /* Implements system specification 4.0 - 4.1 */

/*
 * EXT_CSD fields
 */

#define EXT_CSD_FLUSH_CACHE		32      /* W */
#define EXT_CSD_CACHE_CTRL		33      /* R/W */
#define EXT_CSD_POWER_OFF_NOTIFICATION	34	/* R/W */
#define EXT_CSD_PACKED_FAILURE_INDEX	35	/* RO */
#define EXT_CSD_PACKED_CMD_STATUS	36	/* RO */
#define EXT_CSD_EXP_EVENTS_STATUS	54	/* RO, 2 bytes */
#define EXT_CSD_EXP_EVENTS_CTRL		56	/* R/W, 2 bytes */
#define EXT_CSD_DATA_SECTOR_SIZE	61	/* R */
#define EXT_CSD_GP_SIZE_MULT		143	/* R/W */
#define EXT_CSD_PARTITION_SETTING_COMPLETED 155	/* R/W */
#define EXT_CSD_PARTITION_ATTRIBUTE	156	/* R/W */
#define EXT_CSD_PARTITION_SUPPORT	160	/* RO */
#define EXT_CSD_HPI_MGMT		161	/* R/W */
#define EXT_CSD_RST_N_FUNCTION		162	/* R/W */
#define EXT_CSD_BKOPS_EN		163	/* R/W */
#define EXT_CSD_BKOPS_START		164	/* W */
#define EXT_CSD_SANITIZE_START		165     /* W */
#define EXT_CSD_WR_REL_PARAM		166	/* RO */
#define EXT_CSD_RPMB_MULT		168	/* RO */
#define EXT_CSD_FW_CONFIG		169	/* R/W */
#define EXT_CSD_BOOT_WP			173	/* R/W */
#define EXT_CSD_ERASE_GROUP_DEF		175	/* R/W */
#define EXT_CSD_PART_CONFIG		179	/* R/W */
#define EXT_CSD_ERASED_MEM_CONT		181	/* RO */
#define EXT_CSD_BUS_WIDTH		183	/* R/W */
#define EXT_CSD_HS_TIMING		185	/* R/W */
#define EXT_CSD_POWER_CLASS		187	/* R/W */
#define EXT_CSD_REV			192	/* RO */
#define EXT_CSD_STRUCTURE		194	/* RO */
#define EXT_CSD_CARD_TYPE		196	/* RO */
#define EXT_CSD_DRIVER_STRENGTH		197	/* RO */
#define EXT_CSD_OUT_OF_INTERRUPT_TIME	198	/* RO */
#define EXT_CSD_PART_SWITCH_TIME        199     /* RO */
#define EXT_CSD_PWR_CL_52_195		200	/* RO */
#define EXT_CSD_PWR_CL_26_195		201	/* RO */
#define EXT_CSD_PWR_CL_52_360		202	/* RO */
#define EXT_CSD_PWR_CL_26_360		203	/* RO */
#define EXT_CSD_SEC_CNT			212	/* RO, 4 bytes */
#define EXT_CSD_S_A_TIMEOUT		217	/* RO */
#define EXT_CSD_REL_WR_SEC_C		222	/* RO */
#define EXT_CSD_HC_WP_GRP_SIZE		221	/* RO */
#define EXT_CSD_ERASE_TIMEOUT_MULT	223	/* RO */
#define EXT_CSD_HC_ERASE_GRP_SIZE	224	/* RO */
#define EXT_CSD_BOOT_MULT		226	/* RO */
#define EXT_CSD_SEC_TRIM_MULT		229	/* RO */
#define EXT_CSD_SEC_ERASE_MULT		230	/* RO */
#define EXT_CSD_SEC_FEATURE_SUPPORT	231	/* RO */
#define EXT_CSD_TRIM_MULT		232	/* RO */
#define EXT_CSD_PWR_CL_200_195		236	/* RO */
#define EXT_CSD_PWR_CL_200_360		237	/* RO */
#define EXT_CSD_PWR_CL_DDR_52_195	238	/* RO */
#define EXT_CSD_PWR_CL_DDR_52_360	239	/* RO */
#define EXT_CSD_BKOPS_STATUS		246	/* RO */
#define EXT_CSD_POWER_OFF_LONG_TIME	247	/* RO */
#define EXT_CSD_GENERIC_CMD6_TIME	248	/* RO */
#define EXT_CSD_CACHE_SIZE		249	/* RO, 4 bytes */
#define EXT_CSD_PWR_CL_DDR_200_360	253	/* RO */
#define EXT_CSD_FIRMWARE_VERSION	254	/* RO, 8 bytes */
#define EXT_CSD_SUPPORTED_MODE		493	/* RO */
#define EXT_CSD_TAG_UNIT_SIZE		498	/* RO */
#define EXT_CSD_DATA_TAG_SUPPORT	499	/* RO */
#define EXT_CSD_MAX_PACKED_WRITES	500	/* RO */
#define EXT_CSD_MAX_PACKED_READS	501	/* RO */
#define EXT_CSD_BKOPS_SUPPORT		502	/* RO */
#define EXT_CSD_HPI_FEATURES		503	/* RO */

/*
 * EXT_CSD field definitions
 */

#define EXT_CSD_WR_REL_PARAM_EN		(1<<2)

#define EXT_CSD_BOOT_WP_B_PWR_WP_DIS	(0x40)
#define EXT_CSD_BOOT_WP_B_PERM_WP_DIS	(0x10)
#define EXT_CSD_BOOT_WP_B_PERM_WP_EN	(0x04)
#define EXT_CSD_BOOT_WP_B_PWR_WP_EN	(0x01)

#define EXT_CSD_PART_CONFIG_ACC_MASK	(0x7)
#define EXT_CSD_PART_CONFIG_ACC_BOOT0	(0x1)
#define EXT_CSD_PART_CONFIG_ACC_RPMB	(0x3)
#define EXT_CSD_PART_CONFIG_ACC_GP0	(0x4)

#define EXT_CSD_PART_SETTING_COMPLETED	(0x1)
#define EXT_CSD_PART_SUPPORT_PART_EN	(0x1)

#define EXT_CSD_CMD_SET_NORMAL		(1<<0)
#define EXT_CSD_CMD_SET_SECURE		(1<<1)
#define EXT_CSD_CMD_SET_CPSECURE	(1<<2)

#define EXT_CSD_CARD_TYPE_HS_26	(1<<0)	/* Card can run at 26MHz */
#define EXT_CSD_CARD_TYPE_HS_52	(1<<1)	/* Card can run at 52MHz */
#define EXT_CSD_CARD_TYPE_HS	(EXT_CSD_CARD_TYPE_HS_26 | \
				 EXT_CSD_CARD_TYPE_HS_52)
#define EXT_CSD_CARD_TYPE_DDR_1_8V  (1<<2)   /* Card can run at 52MHz */
					     /* DDR mode @1.8V or 3V I/O */
#define EXT_CSD_CARD_TYPE_DDR_1_2V  (1<<3)   /* Card can run at 52MHz */
					     /* DDR mode @1.2V I/O */
#define EXT_CSD_CARD_TYPE_DDR_52       (EXT_CSD_CARD_TYPE_DDR_1_8V  \
					| EXT_CSD_CARD_TYPE_DDR_1_2V)
#define EXT_CSD_CARD_TYPE_HS200_1_8V	(1<<4)	/* Card can run at 200MHz */
#define EXT_CSD_CARD_TYPE_HS200_1_2V	(1<<5)	/* Card can run at 200MHz */
						/* SDR mode @1.2V I/O */
#define EXT_CSD_CARD_TYPE_HS200		(EXT_CSD_CARD_TYPE_HS200_1_8V | \
					 EXT_CSD_CARD_TYPE_HS200_1_2V)
#define EXT_CSD_CARD_TYPE_HS400_1_8V	(1<<6)	/* Card can run at 200MHz DDR, 1.8V */
#define EXT_CSD_CARD_TYPE_HS400_1_2V	(1<<7)	/* Card can run at 200MHz DDR, 1.2V */
#define EXT_CSD_CARD_TYPE_HS400		(EXT_CSD_CARD_TYPE_HS400_1_8V | \
					 EXT_CSD_CARD_TYPE_HS400_1_2V)

#define EXT_CSD_BUS_WIDTH_1	0	/* Card is in 1 bit mode */
#define EXT_CSD_BUS_WIDTH_4	1	/* Card is in 4 bit mode */
#define EXT_CSD_BUS_WIDTH_8	2	/* Card is in 8 bit mode */
#define EXT_CSD_DDR_BUS_WIDTH_4	5	/* Card is in 4 bit DDR mode */
#define EXT_CSD_DDR_BUS_WIDTH_8	6	/* Card is in 8 bit DDR mode */

#define EXT_CSD_TIMING_BC	0	/* Backwards compatility */
#define EXT_CSD_TIMING_HS	1	/* High speed */
#define EXT_CSD_TIMING_HS200	2	/* HS200 */
#define EXT_CSD_TIMING_HS400	3	/* HS400 */
#define EXT_CSD_DRV_STR_SHIFT	4	/* Driver Strength shift */

#define EXT_CSD_SEC_ER_EN	BIT(0)
#define EXT_CSD_SEC_BD_BLK_EN	BIT(2)
#define EXT_CSD_SEC_GB_CL_EN	BIT(4)
#define EXT_CSD_SEC_SANITIZE	BIT(6)  /* v4.5 only */

#define EXT_CSD_RST_N_EN_MASK	0x3
#define EXT_CSD_RST_N_ENABLED	1	/* RST_n is enabled on card */

#define EXT_CSD_NO_POWER_NOTIFICATION	0
#define EXT_CSD_POWER_ON		1
#define EXT_CSD_POWER_OFF_SHORT		2
#define EXT_CSD_POWER_OFF_LONG		3

#define EXT_CSD_PWR_CL_8BIT_MASK	0xF0	/* 8 bit PWR CLS */
#define EXT_CSD_PWR_CL_4BIT_MASK	0x0F	/* 8 bit PWR CLS */
#define EXT_CSD_PWR_CL_8BIT_SHIFT	4
#define EXT_CSD_PWR_CL_4BIT_SHIFT	0

#define EXT_CSD_PACKED_EVENT_EN	BIT(3)

/*
 * EXCEPTION_EVENT_STATUS field
 */
#define EXT_CSD_URGENT_BKOPS		BIT(0)
#define EXT_CSD_DYNCAP_NEEDED		BIT(1)
#define EXT_CSD_SYSPOOL_EXHAUSTED	BIT(2)
#define EXT_CSD_PACKED_FAILURE		BIT(3)

#define EXT_CSD_PACKED_GENERIC_ERROR	BIT(0)
#define EXT_CSD_PACKED_INDEXED_ERROR	BIT(1)

/*
 * BKOPS status level
 */
#define EXT_CSD_BKOPS_LEVEL_2		0x2

/*
 * BKOPS modes
 */
#define EXT_CSD_MANUAL_BKOPS_MASK	0x01

/*
 * MMC_SWITCH access modes
 */

#define MMC_SWITCH_MODE_CMD_SET	0x00	/* Change the command set */
#define MMC_SWITCH_MODE_SET_BITS	0x01	/* Set bits which are 1 in value */
#define MMC_SWITCH_MODE_CLEAR_BITS	0x02	/* Clear bits which are 1 in value */
#define MMC_SWITCH_MODE_WRITE_BYTE	0x03	/* Set target to value */
/**
  * @}
  */
  
/* Exported macro ------------------------------------------------------------*/
/** @defgroup SD_Exported_macros SD Exported Macros
  * @brief macros to handle interrupts and specific clock configurations
  * @{
  */
 
/**
  * @brief  Enable the SD device.
  * @retval None
  */ 
#define __HAL_SD_SDIO_ENABLE() __SDIO_ENABLE()

/**
  * @brief  Disable the SD device.
  * @retval None
  */
#define __HAL_SD_SDIO_DISABLE() __SDIO_DISABLE()

/**
  * @brief  Enable the SDIO DMA transfer.
  * @retval None
  */ 
#define __HAL_SD_SDIO_DMA_ENABLE() __SDIO_DMA_ENABLE()

/**
  * @brief  Disable the SDIO DMA transfer.
  * @retval None
  */
#define __HAL_SD_SDIO_DMA_DISABLE()  __SDIO_DMA_DISABLE()
 
/**
  * @brief  Enable the SD device interrupt.
  * @param  __HANDLE__: SD Handle  
  * @param  __INTERRUPT__: specifies the SDIO interrupt sources to be enabled.
  *         This parameter can be one or a combination of the following values:
  *            @arg SDIO_IT_CCRCFAIL: Command response received (CRC check failed) interrupt
  *            @arg SDIO_IT_DCRCFAIL: Data block sent/received (CRC check failed) interrupt
  *            @arg SDIO_IT_CTIMEOUT: Command response timeout interrupt
  *            @arg SDIO_IT_DTIMEOUT: Data timeout interrupt
  *            @arg SDIO_IT_TXUNDERR: Transmit FIFO underrun error interrupt
  *            @arg SDIO_IT_RXOVERR:  Received FIFO overrun error interrupt
  *            @arg SDIO_IT_CMDREND:  Command response received (CRC check passed) interrupt
  *            @arg SDIO_IT_CMDSENT:  Command sent (no response required) interrupt
  *            @arg SDIO_IT_DATAEND:  Data end (data counter, SDIDCOUNT, is zero) interrupt
  *            @arg SDIO_IT_STBITERR: Start bit not detected on all data signals in wide 
  *                                   bus mode interrupt
  *            @arg SDIO_IT_DBCKEND:  Data block sent/received (CRC check passed) interrupt
  *            @arg SDIO_IT_CMDACT:   Command transfer in progress interrupt
  *            @arg SDIO_IT_TXACT:    Data transmit in progress interrupt
  *            @arg SDIO_IT_RXACT:    Data receive in progress interrupt
  *            @arg SDIO_IT_TXFIFOHE: Transmit FIFO Half Empty interrupt
  *            @arg SDIO_IT_RXFIFOHF: Receive FIFO Half Full interrupt
  *            @arg SDIO_IT_TXFIFOF:  Transmit FIFO full interrupt
  *            @arg SDIO_IT_RXFIFOF:  Receive FIFO full interrupt
  *            @arg SDIO_IT_TXFIFOE:  Transmit FIFO empty interrupt
  *            @arg SDIO_IT_RXFIFOE:  Receive FIFO empty interrupt
  *            @arg SDIO_IT_TXDAVL:   Data available in transmit FIFO interrupt
  *            @arg SDIO_IT_RXDAVL:   Data available in receive FIFO interrupt
  *            @arg SDIO_IT_SDIOIT:   SD I/O interrupt received interrupt
  *            @arg SDIO_IT_CEATAEND: CE-ATA command completion signal received for CMD61 interrupt     
  * @retval None
  */
#define __HAL_SD_SDIO_ENABLE_IT(__HANDLE__, __INTERRUPT__) __SDIO_ENABLE_IT((__HANDLE__)->Instance, (__INTERRUPT__))

/**
  * @brief  Disable the SD device interrupt.
  * @param  __HANDLE__: SD Handle   
  * @param  __INTERRUPT__: specifies the SDIO interrupt sources to be disabled.
  *          This parameter can be one or a combination of the following values:
  *            @arg SDIO_IT_CCRCFAIL: Command response received (CRC check failed) interrupt
  *            @arg SDIO_IT_DCRCFAIL: Data block sent/received (CRC check failed) interrupt
  *            @arg SDIO_IT_CTIMEOUT: Command response timeout interrupt
  *            @arg SDIO_IT_DTIMEOUT: Data timeout interrupt
  *            @arg SDIO_IT_TXUNDERR: Transmit FIFO underrun error interrupt
  *            @arg SDIO_IT_RXOVERR:  Received FIFO overrun error interrupt
  *            @arg SDIO_IT_CMDREND:  Command response received (CRC check passed) interrupt
  *            @arg SDIO_IT_CMDSENT:  Command sent (no response required) interrupt
  *            @arg SDIO_IT_DATAEND:  Data end (data counter, SDIDCOUNT, is zero) interrupt
  *            @arg SDIO_IT_STBITERR: Start bit not detected on all data signals in wide 
  *                                   bus mode interrupt
  *            @arg SDIO_IT_DBCKEND:  Data block sent/received (CRC check passed) interrupt
  *            @arg SDIO_IT_CMDACT:   Command transfer in progress interrupt
  *            @arg SDIO_IT_TXACT:    Data transmit in progress interrupt
  *            @arg SDIO_IT_RXACT:    Data receive in progress interrupt
  *            @arg SDIO_IT_TXFIFOHE: Transmit FIFO Half Empty interrupt
  *            @arg SDIO_IT_RXFIFOHF: Receive FIFO Half Full interrupt
  *            @arg SDIO_IT_TXFIFOF:  Transmit FIFO full interrupt
  *            @arg SDIO_IT_RXFIFOF:  Receive FIFO full interrupt
  *            @arg SDIO_IT_TXFIFOE:  Transmit FIFO empty interrupt
  *            @arg SDIO_IT_RXFIFOE:  Receive FIFO empty interrupt
  *            @arg SDIO_IT_TXDAVL:   Data available in transmit FIFO interrupt
  *            @arg SDIO_IT_RXDAVL:   Data available in receive FIFO interrupt
  *            @arg SDIO_IT_SDIOIT:   SD I/O interrupt received interrupt
  *            @arg SDIO_IT_CEATAEND: CE-ATA command completion signal received for CMD61 interrupt     
  * @retval None
  */
#define __HAL_SD_SDIO_DISABLE_IT(__HANDLE__, __INTERRUPT__) __SDIO_DISABLE_IT((__HANDLE__)->Instance, (__INTERRUPT__))

/**
  * @brief  Check whether the specified SD flag is set or not. 
  * @param  __HANDLE__: SD Handle   
  * @param  __FLAG__: specifies the flag to check. 
  *          This parameter can be one of the following values:
  *            @arg SDIO_FLAG_CCRCFAIL: Command response received (CRC check failed)
  *            @arg SDIO_FLAG_DCRCFAIL: Data block sent/received (CRC check failed)
  *            @arg SDIO_FLAG_CTIMEOUT: Command response timeout
  *            @arg SDIO_FLAG_DTIMEOUT: Data timeout
  *            @arg SDIO_FLAG_TXUNDERR: Transmit FIFO underrun error
  *            @arg SDIO_FLAG_RXOVERR:  Received FIFO overrun error
  *            @arg SDIO_FLAG_CMDREND:  Command response received (CRC check passed)
  *            @arg SDIO_FLAG_CMDSENT:  Command sent (no response required)
  *            @arg SDIO_FLAG_DATAEND:  Data end (data counter, SDIDCOUNT, is zero)
  *            @arg SDIO_FLAG_STBITERR: Start bit not detected on all data signals in wide bus mode.
  *            @arg SDIO_FLAG_DBCKEND:  Data block sent/received (CRC check passed)
  *            @arg SDIO_FLAG_CMDACT:   Command transfer in progress
  *            @arg SDIO_FLAG_TXACT:    Data transmit in progress
  *            @arg SDIO_FLAG_RXACT:    Data receive in progress
  *            @arg SDIO_FLAG_TXFIFOHE: Transmit FIFO Half Empty
  *            @arg SDIO_FLAG_RXFIFOHF: Receive FIFO Half Full
  *            @arg SDIO_FLAG_TXFIFOF:  Transmit FIFO full
  *            @arg SDIO_FLAG_RXFIFOF:  Receive FIFO full
  *            @arg SDIO_FLAG_TXFIFOE:  Transmit FIFO empty
  *            @arg SDIO_FLAG_RXFIFOE:  Receive FIFO empty
  *            @arg SDIO_FLAG_TXDAVL:   Data available in transmit FIFO
  *            @arg SDIO_FLAG_RXDAVL:   Data available in receive FIFO
  *            @arg SDIO_FLAG_SDIOIT:   SD I/O interrupt received
  *            @arg SDIO_FLAG_CEATAEND: CE-ATA command completion signal received for CMD61
  * @retval The new state of SD FLAG (SET or RESET).
  */
#define __HAL_SD_SDIO_GET_FLAG(__HANDLE__, __FLAG__) __SDIO_GET_FLAG((__HANDLE__)->Instance, (__FLAG__))

/**
  * @brief  Clear the SD's pending flags.
  * @param  __HANDLE__: SD Handle  
  * @param  __FLAG__: specifies the flag to clear.  
  *          This parameter can be one or a combination of the following values:
  *            @arg SDIO_FLAG_CCRCFAIL: Command response received (CRC check failed)
  *            @arg SDIO_FLAG_DCRCFAIL: Data block sent/received (CRC check failed)
  *            @arg SDIO_FLAG_CTIMEOUT: Command response timeout
  *            @arg SDIO_FLAG_DTIMEOUT: Data timeout
  *            @arg SDIO_FLAG_TXUNDERR: Transmit FIFO underrun error
  *            @arg SDIO_FLAG_RXOVERR:  Received FIFO overrun error
  *            @arg SDIO_FLAG_CMDREND:  Command response received (CRC check passed)
  *            @arg SDIO_FLAG_CMDSENT:  Command sent (no response required)
  *            @arg SDIO_FLAG_DATAEND:  Data end (data counter, SDIDCOUNT, is zero)
  *            @arg SDIO_FLAG_STBITERR: Start bit not detected on all data signals in wide bus mode
  *            @arg SDIO_FLAG_DBCKEND:  Data block sent/received (CRC check passed)
  *            @arg SDIO_FLAG_SDIOIT:   SD I/O interrupt received
  *            @arg SDIO_FLAG_CEATAEND: CE-ATA command completion signal received for CMD61
  * @retval None
  */
#define __HAL_SD_SDIO_CLEAR_FLAG(__HANDLE__, __FLAG__) __SDIO_CLEAR_FLAG((__HANDLE__)->Instance, (__FLAG__))

/**
  * @brief  Check whether the specified SD interrupt has occurred or not.
  * @param  __HANDLE__: SD Handle   
  * @param  __INTERRUPT__: specifies the SDIO interrupt source to check. 
  *          This parameter can be one of the following values:
  *            @arg SDIO_IT_CCRCFAIL: Command response received (CRC check failed) interrupt
  *            @arg SDIO_IT_DCRCFAIL: Data block sent/received (CRC check failed) interrupt
  *            @arg SDIO_IT_CTIMEOUT: Command response timeout interrupt
  *            @arg SDIO_IT_DTIMEOUT: Data timeout interrupt
  *            @arg SDIO_IT_TXUNDERR: Transmit FIFO underrun error interrupt
  *            @arg SDIO_IT_RXOVERR:  Received FIFO overrun error interrupt
  *            @arg SDIO_IT_CMDREND:  Command response received (CRC check passed) interrupt
  *            @arg SDIO_IT_CMDSENT:  Command sent (no response required) interrupt
  *            @arg SDIO_IT_DATAEND:  Data end (data counter, SDIDCOUNT, is zero) interrupt
  *            @arg SDIO_IT_STBITERR: Start bit not detected on all data signals in wide 
  *                                   bus mode interrupt
  *            @arg SDIO_IT_DBCKEND:  Data block sent/received (CRC check passed) interrupt
  *            @arg SDIO_IT_CMDACT:   Command transfer in progress interrupt
  *            @arg SDIO_IT_TXACT:    Data transmit in progress interrupt
  *            @arg SDIO_IT_RXACT:    Data receive in progress interrupt
  *            @arg SDIO_IT_TXFIFOHE: Transmit FIFO Half Empty interrupt
  *            @arg SDIO_IT_RXFIFOHF: Receive FIFO Half Full interrupt
  *            @arg SDIO_IT_TXFIFOF:  Transmit FIFO full interrupt
  *            @arg SDIO_IT_RXFIFOF:  Receive FIFO full interrupt
  *            @arg SDIO_IT_TXFIFOE:  Transmit FIFO empty interrupt
  *            @arg SDIO_IT_RXFIFOE:  Receive FIFO empty interrupt
  *            @arg SDIO_IT_TXDAVL:   Data available in transmit FIFO interrupt
  *            @arg SDIO_IT_RXDAVL:   Data available in receive FIFO interrupt
  *            @arg SDIO_IT_SDIOIT:   SD I/O interrupt received interrupt
  *            @arg SDIO_IT_CEATAEND: CE-ATA command completion signal received for CMD61 interrupt
  * @retval The new state of SD IT (SET or RESET).
  */
#define __HAL_SD_SDIO_GET_IT  (__HANDLE__, __INTERRUPT__) __SDIO_GET_IT  ((__HANDLE__)->Instance, __INTERRUPT__)

/**
  * @brief  Clear the SD's interrupt pending bits.
  * @param  __HANDLE__: SD Handle
  * @param  __INTERRUPT__: specifies the interrupt pending bit to clear. 
  *          This parameter can be one or a combination of the following values:
  *            @arg SDIO_IT_CCRCFAIL: Command response received (CRC check failed) interrupt
  *            @arg SDIO_IT_DCRCFAIL: Data block sent/received (CRC check failed) interrupt
  *            @arg SDIO_IT_CTIMEOUT: Command response timeout interrupt
  *            @arg SDIO_IT_DTIMEOUT: Data timeout interrupt
  *            @arg SDIO_IT_TXUNDERR: Transmit FIFO underrun error interrupt
  *            @arg SDIO_IT_RXOVERR:  Received FIFO overrun error interrupt
  *            @arg SDIO_IT_CMDREND:  Command response received (CRC check passed) interrupt
  *            @arg SDIO_IT_CMDSENT:  Command sent (no response required) interrupt
  *            @arg SDIO_IT_DATAEND:  Data end (data counter, SDIO_DCOUNT, is zero) interrupt
  *            @arg SDIO_IT_STBITERR: Start bit not detected on all data signals in wide 
  *                                   bus mode interrupt
  *            @arg SDIO_IT_SDIOIT:   SD I/O interrupt received interrupt
  *            @arg SDIO_IT_CEATAEND: CE-ATA command completion signal received for CMD61
  * @retval None
  */
#define __HAL_SD_SDIO_CLEAR_IT(__HANDLE__, __INTERRUPT__) __SDIO_CLEAR_IT((__HANDLE__)->Instance, (__INTERRUPT__))
/**
  * @}
  */
  
/* Exported functions --------------------------------------------------------*/
/** @defgroup SD_Exported_Functions SD Exported Functions
  * @{
  */

/** @defgroup SD_Exported_Functions_Group1 Initialization and de-initialization functions
  * @{
  */
HAL_SD_ErrorTypedef HAL_SD_Init(SD_HandleTypeDef *hsd, HAL_SD_CardInfoTypedef *SDCardInfo);
HAL_StatusTypeDef   HAL_SD_DeInit (SD_HandleTypeDef *hsd);
void HAL_SD_MspInit(SD_HandleTypeDef *hsd);
void HAL_SD_MspDeInit(SD_HandleTypeDef *hsd);
/**
  * @}
  */

/** @defgroup SD_Exported_Functions_Group2 I/O operation functions
  * @{
  */
/* Blocking mode: Polling */
HAL_SD_ErrorTypedef HAL_SD_ReadBlocks(SD_HandleTypeDef *hsd, uint32_t *pReadBuffer, uint64_t ReadAddr, uint32_t BlockSize, uint32_t NumberOfBlocks);
HAL_SD_ErrorTypedef HAL_SD_WriteBlocks(SD_HandleTypeDef *hsd, uint32_t *pWriteBuffer, uint64_t WriteAddr, uint32_t BlockSize, uint32_t NumberOfBlocks);
HAL_SD_ErrorTypedef HAL_SD_Erase(SD_HandleTypeDef *hsd, uint64_t startaddr, uint64_t endaddr);

/* Non-Blocking mode: Interrupt */
void HAL_SD_IRQHandler(SD_HandleTypeDef *hsd);

/* Callback in non blocking modes (DMA) */
void HAL_SD_DMA_RxCpltCallback(DMA_HandleTypeDef *hdma);
void HAL_SD_DMA_RxErrorCallback(DMA_HandleTypeDef *hdma);
void HAL_SD_DMA_TxCpltCallback(DMA_HandleTypeDef *hdma);
void HAL_SD_DMA_TxErrorCallback(DMA_HandleTypeDef *hdma);
void HAL_SD_XferCpltCallback(SD_HandleTypeDef *hsd);
void HAL_SD_XferErrorCallback(SD_HandleTypeDef *hsd);

/* Non-Blocking mode: DMA */
HAL_SD_ErrorTypedef HAL_SD_ReadBlocks_DMA(SD_HandleTypeDef *hsd, uint32_t *pReadBuffer, uint64_t ReadAddr, uint32_t BlockSize, uint32_t NumberOfBlocks);
HAL_SD_ErrorTypedef HAL_SD_WriteBlocks_DMA(SD_HandleTypeDef *hsd, uint32_t *pWriteBuffer, uint64_t WriteAddr, uint32_t BlockSize, uint32_t NumberOfBlocks);
HAL_SD_ErrorTypedef HAL_SD_CheckWriteOperation(SD_HandleTypeDef *hsd, uint32_t Timeout);
HAL_SD_ErrorTypedef HAL_SD_CheckReadOperation(SD_HandleTypeDef *hsd, uint32_t Timeout);
/**
  * @}
  */

/** @defgroup SD_Exported_Functions_Group3 Peripheral Control functions
  * @{
  */
HAL_SD_ErrorTypedef HAL_SD_Get_CardInfo(SD_HandleTypeDef *hsd, HAL_SD_CardInfoTypedef *pCardInfo);
HAL_SD_ErrorTypedef HAL_MMC_Get_CardInfo(SD_HandleTypeDef *hsd, HAL_SD_CardInfoTypedef *pCardInfo);
HAL_SD_ErrorTypedef HAL_MMC_Switch(SD_HandleTypeDef *hsd,  uint8_t set,
									uint8_t index,
									uint8_t value);
HAL_SD_ErrorTypedef HAL_SD_WideBusOperation_Config(SD_HandleTypeDef *hsd, uint32_t WideMode);
HAL_SD_ErrorTypedef HAL_SD_StopTransfer(SD_HandleTypeDef *hsd);
HAL_SD_ErrorTypedef HAL_SD_HighSpeed (SD_HandleTypeDef *hsd);
/**
  * @}
  */
  
/* Peripheral State functions  ************************************************/
/** @defgroup SD_Exported_Functions_Group4 Peripheral State functions
  * @{
  */
HAL_SD_ErrorTypedef HAL_SD_SendSDStatus(SD_HandleTypeDef *hsd, uint32_t *pSDstatus);
HAL_SD_ErrorTypedef HAL_SD_GetCardStatus(SD_HandleTypeDef *hsd, HAL_SD_CardStatusTypedef *pCardStatus);
HAL_SD_TransferStateTypedef HAL_SD_GetStatus(SD_HandleTypeDef *hsd);
/**
  * @}
  */
  
/**
  * @}
  */
    
/* Private types -------------------------------------------------------------*/
/** @defgroup SD_Private_Types SD Private Types
  * @{
  */

/**
  * @}
  */ 

/* Private defines -----------------------------------------------------------*/
/** @defgroup SD_Private_Defines SD Private Defines
  * @{
  */

/**
  * @}
  */ 
          
/* Private variables ---------------------------------------------------------*/
/** @defgroup SD_Private_Variables SD Private Variables
  * @{
  */

/**
  * @}
  */ 

/* Private constants ---------------------------------------------------------*/
/** @defgroup SD_Private_Constants SD Private Constants
  * @{
  */

/**
  * @}
  */ 

/* Private macros ------------------------------------------------------------*/
/** @defgroup SD_Private_Macros SD Private Macros
  * @{
  */

/**
  * @}
  */

/* Private functions prototypes ----------------------------------------------*/
/** @defgroup SD_Private_Functions_Prototypes SD Private Functions Prototypes
  * @{
  */

/**
  * @}
  */

/* Private functions ---------------------------------------------------------*/
/** @defgroup SD_Private_Functions SD Private Functions
  * @{
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
#endif /* STM32F405xx || STM32F415xx || STM32F407xx || STM32F417xx || STM32F427xx || STM32F437xx || STM32F429xx || STM32F439xx ||
          STM32F401xC || STM32F401xE || STM32F411xE || STM32F446xx || STM32F469xx || STM32F479xx || STM32F412Zx || STM32F412Vx ||
          STM32F412Rx || STM32F412Cx */
#ifdef __cplusplus
}
#endif

#endif /* __STM32F4xx_HAL_SD_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
