#ifndef IAP_H
#define IAP_H
#include "main.h"
#include "cmsis_os.h"

enum flash_update_state {
    IDLE,
    START_FLASH_UPDATE,
    WAIT_FOR_ARM_RESPONSE,
    SEND_FLASH_UPDATE_COMMAND,
    RECEIVED_RESPONSE_FROM_ARM,
    SEND_FLASHUPDATE_DATA,
    WAIT_DATA_WRITE_COMPLETE_FROM_ARM,
    FLASHUPDATE_COMPLETE,
    WAIT_RESPONSE_FLASHUPDATE_COMPLETE_STATE,
    SEND_FLASH_IMAGE_VERIFY,
    WAIT_FLASH_VERIFY_RESULT,
    SEND_RESTART_COMMAND,
    WAIT_ARM_RESTART_STATUS,
    DONE,

    ARM_RECEIVED_FU_COMMEND_ERASE_ALL_OK,
    ARM_DATA_WRITE_COMPLETE,
    ARM_RESTART_NOW,
};
/* communication with host computer msg class */
enum {
    ANALOG_MSG = 1,
    ALARM_MSG = 2,
    DFU_APPLICATION = 3,
    DFU_FIRMWARE = 4,
    COMMAND_CLASS = 5,
};

enum flash_update_command {
    /* received */
    SendFlashUpdateCommand = 0x01,
    SendFlashUpdateData = 0x03,
    SendFlashUpdateComplete = 0x05,
    SendFlashImageVerify = 0x07,
    SendRestartCommand = 0x0B,
    /* send */
    ReceivedARM_FU_CommandEraseAll_OK = 0x02,
    ReceivedARM_DataWriteComplete = 0x04,
    ReceivedFlashupdateCompleteState = 0x06,
    ReceivedFlashImageVerifyNoProblem = 0x08,
    ReceivedFlashImageVerifyError = 0x09,
    ReceivedRestartNOW = 0x0C,


    /*tms320f280049*/
    TMS320F280049_APPLiCATION_FU = 0x14,
    TMS320F280049_ALL_FLASH_FU = 0x15,

    FlashVerifyError = 0xFA,
    FlashUnlockError = 0xFB,
    CRC_Error = 0xFC,
    FlashEraseError = 0xFD,
    FlashProgramError = 0xFE,
};
#define RECEIVED_USB_MSG_JUST_NOW   1
#define USB_MSG_NOT_YET_RECEIVED    0

#define RAM_BASE_ADDRESS            0x20000000UL
#define FLASH_BASE_ADDRESS          0x08000000UL
#define FLASH_MAX_SIZE              0x00040000UL
#define APPLICATION_BASE_ADDRESS    0x0800D000UL
#define CODE_VALID_MAGIC_NUMBER     0x7472FFFFUL


#define RAM_END_ADDRESS                 (RAM_BASE_ADDRESS + 0xC000)
#define FLASH_END_ADDRESS               (FLASH_BASE_ADDRESS + FLASH_MAX_SIZE)

#define FIRMWARE_IMAGE_TAG_ADDRESS_P  (FLASH_BASE_ADDRESS + 0x400)
#define FIRMWARE_IMAGE_TAG_ADDRESS    (*(__IO uint32_t*)(FIRMWARE_IMAGE_TAG_ADDRESS_P))
#define FIRMWARE_IMAGE_TAG            ((struct boot_headers *)FIRMWARE_IMAGE_TAG_ADDRESS)

#define FIRMWARE_STACK_VALUE            (*(__IO uint32_t*)FLASH_BASE_ADDRESS)
#define FIRMWARE_RESET_ADDRESS          (*(__IO uint32_t*)(FLASH_BASE_ADDRESS + 4))

#define APPLICATION_IMAGE_TAG_ADDRESS_P   (APPLICATION_BASE_ADDRESS + 0x400)
#define APPLICATION_IMAGE_TAG_ADDRESS     (*(__IO uint32_t*)(APPLICATION_IMAGE_TAG_ADDRESS_P))
#define APPLICATION_IMAGE_TAG             ((struct boot_headers *)APPLICATION_IMAGE_TAG_ADDRESS)

#define APPLICATION_STACK_VALUE             (*(__IO uint32_t*)APPLICATION_BASE_ADDRESS)
#define APPLICATION_RESET_ADDRESS           (*(__IO uint32_t*)(APPLICATION_BASE_ADDRESS + 4))

#define USB_PULLUP_DISABLE()    GPIOD->BRR = GPIO_PIN_3;  // disable usb full speed
#define USB_PULLUP_ENABLE()     GPIOD->BSRR = GPIO_PIN_3;  // enable usb full speed


#define FLASH_ADDR_VALID(addr) (addr >= APPLICATION_BASE_ADDRESS) && \
        (addr < (FLASH_BASE_ADDRESS + FLASH_MAX_SIZE))

#pragma pack (1)
struct boot_headers {
    uint32_t image_addr;
    uint32_t image_len;
    uint32_t magic_number;
    uint32_t rsvd1;
    uint32_t image_verify[4];
};
#pragma pack (1)
struct flash_update_info_t {
    struct usb_msg *usb_rx_msg;
    struct usb_msg *usb_tx_msg;
    struct boot_headers *image_info;
    
    uint32_t current_iap_target;
    /*arm*/
    uint32_t fu_status;
    uint32_t fu_addr_base;
    uint32_t fu_current_address;
    uint32_t fu_next_address;
    uint32_t is_received_usb_msg;
    uint32_t host_resend_num;
    uint32_t image_verify_err_num;
    uint32_t current_received_program_addr_from_mcu;

    uint32_t err_id;
    uint32_t err_code;
};
enum ERROR_CODE {
    NO_PROBLEM = 0,
    RECEIVED_FU_ADDRESS_OVERRANGE,
    RECEIVED_FU_ADDRESS_MISMATCH,
    FLASH_PROGRAM_FAILED,
    FLASH_ERASE_FAILED,
    FLASH_UNLOCK_FAILED,
    HOST_WAITING_TIMEOUT_RESEND,
};
#pragma pack(1)
struct usb_msg {
    uint8_t msg_class;
    uint8_t msg_id;
    uint16_t msg_len;
    uint32_t address;
    uint8_t data[52];
};
extern uint32_t flash_update_status;
extern struct usb_msg msg_rx_usb;
extern struct usb_msg msg_tx_usb;
void iap_process(void);
void copy_received_msg(uint8_t *ptr, uint32_t length);
void jump_to_user_application_if(void);
uint8_t tx_msg_packed(uint8_t msg_class, uint8_t msg_id);
#ifdef DEBUG1
#define PAUSE_MCU_RUNNING	__asm("BKPT 127");
#else
#define PAUSE_MCU_RUNNING	;
#endif
#endif
