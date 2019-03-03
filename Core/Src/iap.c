
#include "iap.h"
#include "usbd_cdc_if.h"
#include "printf_reloc.h"
#include "delay_cpu_clk.h"

struct usb_msg msg_rx_usb;
struct usb_msg msg_tx_usb;
uint32_t received_usb_msg = USB_MSG_NOT_YET_RECEIVED;
uint32_t flash_update_status = IDLE;
static int verify_image_info(uint32_t tag_addr);
const uint32_t firmware_magic_number_address __attribute__((at(FIRMWARE_IMAGE_TAG_ADDRESS_P))) = 0x87654321;
FLASH_EraseInitTypeDef earse_application_image = 
{
/*!< TypeErase: Mass erase or page erase. This parameter can be a value of @ref FLASHEx_Type_Erase */
    .TypeErase = FLASH_TYPEERASE_PAGES,   
/*!< PageAdress: Initial FLASH page address to erase when mass erase is disabled
    This parameter must be a number between Min_Data = 0x08000000 and Max_Data = FLASH_BANKx_END (x = 1 or 2 depending on devices)*/
    .PageAddress = APPLICATION_BASE_ADDRESS,
/*!< NbPages: Number of pagess to be erased.
                             This parameter must be a value between Min_Data = 1 and Max_Data = (max number of pages - value of initial page)*/
    .NbPages = 102,
};
uint32_t ErrorSector;

uint8_t tx_msg_packed(uint8_t msg_class, uint8_t msg_id)
{
    msg_tx_usb.msg_class = msg_class;
    msg_tx_usb.msg_id = msg_id;
    msg_tx_usb.msg_len = 52;
    return (CDC_Transmit_FS(&msg_tx_usb.msg_class, 2));
}

void received_msg_process(void)
{
    msg_rx_usb.msg_class = 0;
    msg_rx_usb.msg_id = 0;
    received_usb_msg = USB_MSG_NOT_YET_RECEIVED;
}

int write_to_flash(void) {
    if ((msg_rx_usb.address < APPLICATION_BASE_ADDRESS) && (msg_rx_usb.address > (FLASH_END_ADDRESS - sizeof(struct usb_msg)))) {
        return 1;
    }
    uint32_t address = msg_rx_usb.address;
    uint32_t *data = (uint32_t *)msg_rx_usb.data;
    int length = msg_rx_usb.msg_len / 4;
    //GPIOB->BSRR = GPIO_PIN_0;
    for (int i = 0; i < length; ++i) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, (uint64_t)*data) != HAL_OK) {
            return -1;
        }
#if 0
        while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY));
        SET_BIT(FLASH->CR, FLASH_CR_PG);

        /* Write data in the address */
        *(__IO uint16_t*)address = *data;
        while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY));
        CLEAR_BIT(FLASH->CR, FLASH_CR_PG);
#endif
        address += 4;
        data++;
    }
    //GPIOB->BRR = GPIO_PIN_0;
    return HAL_OK;
}


void iap_process(void)
{
    switch (flash_update_status){
        case IDLE:
			tx_msg_packed(COMMAND_CLASS, 0x21);
            if (received_usb_msg) {
                if ((msg_rx_usb.msg_class == DFU_APPLICATION) && (msg_rx_usb.msg_id == SendFlashUpdateCommand)) {
                    flash_update_status = ARM_RECEIVED_FU_COMMEND_ERASE_ALL_OK;
                    received_msg_process();
                    debug("SEND_FLASH_UPDATE_COMMAND\r\n");
                }

                received_msg_process();
            }
            break;
        case ARM_RECEIVED_FU_COMMEND_ERASE_ALL_OK:
            debug("1\r\n");
            if (HAL_FLASH_Unlock() == HAL_OK) {
                debug("2\r\n");
                if (HAL_FLASHEx_Erase(&earse_application_image, &ErrorSector) == HAL_OK) {
                    debug("3\r\n");
                    if (tx_msg_packed(DFU_APPLICATION, ReceivedARM_FU_CommandEraseAll_OK) == USBD_OK) {
                        flash_update_status = SEND_FLASHUPDATE_DATA;
                        debug("ARM_RECEIVED_FU_COMMEND_ERASE_ALL_OK\r\n");
                    }
                    else {
                        debug("4\r\n");
                        __asm("BKPT 127");
                    }
                }
                else {
                    debug("5\r\n");
                    __asm("BKPT 127");
                }
            }
            else {
                debug("6\r\n");
                __asm("BKPT 127");
            }
            break;
        case SEND_FLASHUPDATE_DATA:
            if (received_usb_msg) {
                if ((msg_rx_usb.msg_class == DFU_APPLICATION) && (msg_rx_usb.msg_id == SendFlashUpdateData)) {
                    if (write_to_flash() == HAL_OK) {
                        flash_update_status = ARM_DATA_WRITE_COMPLETE;
                        debug("SEND_FLASHUPDATE_DATA, address is 0x%x\r\n", msg_rx_usb.address);
                        goto transmit_completed;
                    } 
                    else {
                        debug("ERROR: write to flash, address is %#08x error!\r\n", msg_rx_usb.address);
                        __asm("BKPT 127");
                    }
                    received_msg_process();
                }
                else if ((msg_rx_usb.msg_class == DFU_APPLICATION) && (msg_rx_usb.msg_id == SendFlashUpdateComplete)) {
                    flash_update_status = FLASHUPDATE_COMPLETE;
                }
                else {
                    debug("7\r\n");
                }

                received_msg_process();
            }
            break;
        case ARM_DATA_WRITE_COMPLETE:
            transmit_completed:
                received_msg_process();
                debug("ARM_DATA_WRITE_COMPLETE\r\n");
                if (tx_msg_packed(DFU_APPLICATION, ReceivedARM_DataWriteComplete) == USBD_OK) {
                    flash_update_status = SEND_FLASHUPDATE_DATA;
                }
            break;
        case FLASHUPDATE_COMPLETE:
            if (tx_msg_packed(DFU_APPLICATION, ReceivedFlashupdateCompleteState) == USBD_OK) {
                debug("FLASHUPDATE_COMPLETE\r\n");
                flash_update_status = SEND_FLASH_IMAGE_VERIFY;
            }
            break;
        case SEND_FLASH_IMAGE_VERIFY:
            if (received_usb_msg) {
                if ((msg_rx_usb.msg_class == DFU_APPLICATION) && (msg_rx_usb.msg_id == SendFlashImageVerify)) {
                    flash_update_status = WAIT_FLASH_VERIFY_RESULT;
                    debug("SEND_FLASH_IMAGE_VERIFY\r\n");
                }
                received_msg_process();
            }
            break;

        case WAIT_FLASH_VERIFY_RESULT:
            if (verify_image_info(APPLICATION_IMAGE_TAG_ADDRESS) == 0) {
                if (tx_msg_packed(DFU_APPLICATION, ReceivedFlashImageVerifyNoProblem) == USBD_OK) {
                    debug("Received Flash Image Verify No Problem\r\n");
                    flash_update_status = SEND_RESTART_COMMAND;
                    break;
                }
            }
            else {
                if (tx_msg_packed(DFU_APPLICATION, ReceivedFlashImageVerifyError) == USBD_OK) {
                    debug("Received Flash Image Verify Error\r\n");
                    flash_update_status = IDLE;
                }
            }
            break;
        case SEND_RESTART_COMMAND:
            if (received_usb_msg) {
                if ((msg_rx_usb.msg_class == DFU_APPLICATION) && (msg_rx_usb.msg_id == SendRestartCommand)) {
                    flash_update_status = ARM_RESTART_NOW;
                    debug("SEND_RESTART_COMMAND\r\n");
                }
                received_msg_process();
            }
            break;

        case ARM_RESTART_NOW:
            if (tx_msg_packed(DFU_APPLICATION, ReceivedRestartNOW) == USBD_OK) {
                flash_update_status = DONE;
            }
            debug("ARM_RESTART_NOW\r\n");
            break;
        case DONE:
            /* waiting send the last response to host */
            osDelay(100);
            debug("JUMP_TO_USER_APPLICATION\r\n");
            jump_to_user_application_if();
			debug("what\r\n");
            break; 
    }

}


void copy_received_msg(uint8_t *ptr, uint32_t length)
{
    uint8_t *p = &(msg_rx_usb.msg_class);
    if (length > sizeof(struct usb_msg)) {
        return;
    }
    for (uint32_t i = 0; i < length; ++i) {
        *p++ = *ptr++;
    }
    received_usb_msg = RECEIVED_USB_MSG_JUST_NOW;
    return;
}

static void close_allof_interrupt(void)
{
    NVIC->ICER[0] = 0xFFFFFFFF;
    NVIC->ICER[1] = 0xFFFFFFFF;
    NVIC->ICER[2] = 0xFFFFFFFF;
}

static __asm void the_entry(const uint32_t zero, const uint32_t app_entry, const uint32_t msp_r)
{
	PRESERVE8
    /* set sp = msp */
    mov r3, #0
    msr control, r3
	/* set msp = msp_r */
    msr msp, r2
    dsb
    isb
	/* jump to app */
    bx r1
    nop
}

static int verify_image_tag(const struct boot_headers * const image)
{
    uint32_t image_crc_cal = 0;
    uint32_t i = 0;
    uint32_t image_length = image->image_len / 4;
    const uint32_t *image_ptr = (uint32_t *)(image->image_addr);

    if (image->magic_number != 0xD00DFEED) {
        return -1;
    }
    for ( ; i < image_length; ++i) {
        image_crc_cal += *image_ptr;
        image_ptr++;
    }
    if (image_crc_cal == image->image_verify[0]) {
            return 0;
    }
    return -1;
}

static int verify_image_info(uint32_t tag_addr)
{
    const struct boot_headers * const image_info = (struct boot_headers *)tag_addr;
    /* verify application validity */
    if (tag_addr == 0x87654321) {
        return 0;
    }
    else if (FLASH_ADDR_VALID(tag_addr)) {
        if ((FLASH_ADDR_VALID(image_info->image_addr)) && \
        (FLASH_ADDR_VALID(image_info->image_addr + image_info->image_len))) {
            if (verify_image_tag(image_info) == 0) {
                return 0;
            }
        }
    }

    return -1;
}

void jump_to_user_application_if(void)
{
    /* image tag error, should not jump */
    if (verify_image_info(APPLICATION_IMAGE_TAG_ADDRESS) == -1) {
        return;
    }
    if ((APPLICATION_STACK_VALUE & 0x2FFE0000 ) == RAM_BASE_ADDRESS) {
		
        USB_PULLUP_DISABLE();
		debug("**************************\r\n");
		debug("Ready Jump to application!\r\n");
		debug("**************************\r\n");
		Delay_Clk(4000000);
        HAL_RCC_DeInit();
        __disable_irq();
        close_allof_interrupt();
		
        /* jump to application reset handler */
        the_entry(0, APPLICATION_RESET_ADDRESS, APPLICATION_STACK_VALUE);
    }
	else {
        goto err;
    }
    err:
        __disable_irq();
        GPIOB->BSRR = GPIO_PIN_0;
        while(1);
}
