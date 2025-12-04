#include "User_Com.h"
#include "usbd_cdc.h"
#include "usbd_core.h"

/*!< endpoint address */
#define CDC_IN_EP 0x81
#define CDC_OUT_EP 0x02
#define CDC_INT_EP 0x83

#define USBD_VID 0x66CC
#define USBD_PID 0x2233
#define USBD_MAX_POWER 100
#define USBD_LANGID_STRING 1033

/*!< config descriptor size */
#define USB_CONFIG_SIZE (9 + CDC_ACM_DESCRIPTOR_LEN)

/*!< global descriptor */
static const uint8_t cdc_descriptor[] = {
    USB_DEVICE_DESCRIPTOR_INIT(USB_2_0, 0xEF, 0x02, 0x01, USBD_VID, USBD_PID,
                               0x0100, 0x01),
    USB_CONFIG_DESCRIPTOR_INIT(USB_CONFIG_SIZE, 0x02, 0x01,
                               USB_CONFIG_BUS_POWERED, USBD_MAX_POWER),
    CDC_ACM_DESCRIPTOR_INIT(0x00, CDC_INT_EP, CDC_OUT_EP, CDC_IN_EP, 0x02),
    ///////////////////////////////////////
    /// string0 descriptor
    ///////////////////////////////////////
    USB_LANGID_INIT(USBD_LANGID_STRING),
    ///////////////////////////////////////
    /// string1 descriptor
    ///////////////////////////////////////
    /* Rhine-Lab */
    0x14,       /*!< bLength */
    0x03,       /*!< bDescriptorType */
    0x52, 0x00, /*!< 'R' wcChar0 */
    0x68, 0x00, /*!< 'h' wcChar1 */
    0x69, 0x00, /*!< 'i' wcChar2 */
    0x6e, 0x00, /*!< 'n' wcChar3 */
    0x65, 0x00, /*!< 'e' wcChar4 */
    0x2d, 0x00, /*!< '-' wcChar5 */
    0x4c, 0x00, /*!< 'L' wcChar6 */
    0x61, 0x00, /*!< 'a' wcChar7 */
    0x62, 0x00, /*!< 'b' wcChar8 */
    ///////////////////////////////////////
    /// string2 descriptor
    ///////////////////////////////////////
    /* LX FlightController */
    0x28,       /*!< bLength */
    0x03,       /*!< bDescriptorType */
    0x4c, 0x00, /*!< 'L' wcChar0 */
    0x58, 0x00, /*!< 'X' wcChar1 */
    0x20, 0x00, /*!< ' ' wcChar2 */
    0x46, 0x00, /*!< 'F' wcChar3 */
    0x6c, 0x00, /*!< 'l' wcChar4 */
    0x69, 0x00, /*!< 'i' wcChar5 */
    0x67, 0x00, /*!< 'g' wcChar6 */
    0x68, 0x00, /*!< 'h' wcChar7 */
    0x74, 0x00, /*!< 't' wcChar8 */
    0x43, 0x00, /*!< 'C' wcChar9 */
    0x6f, 0x00, /*!< 'o' wcChar10 */
    0x6e, 0x00, /*!< 'n' wcChar11 */
    0x74, 0x00, /*!< 't' wcChar12 */
    0x72, 0x00, /*!< 'r' wcChar13 */
    0x6f, 0x00, /*!< 'o' wcChar14 */
    0x6c, 0x00, /*!< 'l' wcChar15 */
    0x6c, 0x00, /*!< 'l' wcChar16 */
    0x65, 0x00, /*!< 'e' wcChar17 */
    0x72, 0x00, /*!< 'r' wcChar18 */
    ///////////////////////////////////////
    /// string3 descriptor
    ///////////////////////////////////////
    /* 76 */
    0x06,       /*!< bLength */
    0x03,       /*!< bDescriptorType */
    0x37, 0x00, /*!< '7' wcChar0 */
    0x36, 0x00, /*!< '6' wcChar1 */
#ifdef CONFIG_USB_HS
    ///////////////////////////////////////
    /// device qualifier descriptor
    ///////////////////////////////////////
    0x0a, USB_DESCRIPTOR_TYPE_DEVICE_QUALIFIER, 0x00, 0x02, 0x02, 0x02, 0x01,
    0x40, 0x01, 0x00,
#endif
    0x00};
#define CDC_BUFFER_SIZE 2048

USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t read_buffer[CDC_BUFFER_SIZE];
USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t write_buffer[CDC_BUFFER_SIZE];
volatile uint8_t ep_tx_busy_flag = false;

#define CDC_QUEUE 1       // 1: enable queue, 0: disable queue
#define CDC_MAX_QUEUE 16  // max queue size

#if CDC_QUEUE
uint32_t write_queued_pos[CDC_MAX_QUEUE] = {0};
uint32_t write_queued_len[CDC_MAX_QUEUE] = {0};
uint32_t write_queued_count = 0;
uint32_t write_queued_tail = 0;
void send_queued_data(void) {
  ep_tx_busy_flag = true;
  write_queued_count--;
  if (!usbd_ep_start_write(CDC_IN_EP,
                           write_buffer + write_queued_pos[write_queued_count],
                           write_queued_len[write_queued_count]) == 0) {
    ep_tx_busy_flag = false;
  }
}
#endif

#ifdef CONFIG_USB_HS
#define CDC_MAX_MPS 512
#else
#define CDC_MAX_MPS 64
#endif

void usbd_configure_done_callback(void) {
  usbd_ep_start_read(CDC_OUT_EP, read_buffer, CDC_BUFFER_SIZE);
}

void usbd_cdc_acm_bulk_out(uint8_t ep, uint32_t nbytes) {
  // USB_LOG_RAW("actual out len:%d\r\n", nbytes);

  // UserCom_DataAnl(read_buffer, nbytes);

  // for (uint32_t i = 0; i < nbytes; i++) {
  //   UserCom_GetOneByte(read_buffer[i]);
  // }

  UserCom_GetBuffer(read_buffer, nbytes);

  /* setup next out ep read transfer */
  usbd_ep_start_read(CDC_OUT_EP, read_buffer, CDC_BUFFER_SIZE);
}

void usbd_cdc_acm_bulk_in(uint8_t ep, uint32_t nbytes) {
  // USB_LOG_RAW("actual in len:%d\r\n", nbytes);
  if ((nbytes % CDC_MAX_MPS) == 0 && nbytes) {
    /* send zlp */
    usbd_ep_start_write(CDC_IN_EP, NULL, 0);
  } else {
#if CDC_QUEUE
    if (write_queued_count) {
      send_queued_data();
    } else {
      ep_tx_busy_flag = false;
    }
#else
    ep_tx_busy_flag = false;
#endif
  }
}

/*!< endpoint call back */
struct usbd_endpoint cdc_out_ep = {.ep_addr = CDC_OUT_EP,
                                   .ep_cb = usbd_cdc_acm_bulk_out};

struct usbd_endpoint cdc_in_ep = {.ep_addr = CDC_IN_EP,
                                  .ep_cb = usbd_cdc_acm_bulk_in};

struct usbd_interface intf0;
struct usbd_interface intf1;

void cdc_acm_init(void) {
  usbd_desc_register(cdc_descriptor);
  usbd_add_interface(usbd_cdc_acm_init_intf(&intf0));
  usbd_add_interface(usbd_cdc_acm_init_intf(&intf1));
  usbd_add_endpoint(&cdc_out_ep);
  usbd_add_endpoint(&cdc_in_ep);
  usbd_initialize();
}
volatile uint8_t dtr_enable = 0;

void usbd_cdc_acm_set_dtr(uint8_t intf, uint8_t dtr) {
  if (dtr) {
    dtr_enable = 1;
  } else {
    dtr_enable = 0;
  }
  ep_tx_busy_flag = false;
}

#include "Drv_led.h"

void cdc_acm_data_send(uint8_t *buf, uint32_t len) {
#if CDC_QUEUE  // queue
  if (dtr_enable) {
    if (write_queued_count < CDC_MAX_QUEUE) {
      if (write_queued_tail + len * CONFIG_USB_ALIGN_SIZE >= CDC_BUFFER_SIZE) {
        write_queued_tail = 0;
      }
      memcpy(write_buffer + write_queued_tail, buf, len);
      write_queued_pos[write_queued_count] = write_queued_tail;
      write_queued_len[write_queued_count] = len;
      write_queued_tail += len * CONFIG_USB_ALIGN_SIZE;
      write_queued_count++;
    } else {  // queue full
      user_led.brightness[3] = 20;
    }
    if (!ep_tx_busy_flag) {
      send_queued_data();
    }
  }
#else  // no queue
  u16 timeout = 0;
  if (dtr_enable) {
    while (ep_tx_busy_flag) {
      if (dtr_enable == 0 || timeout++ > 10000) {
        ep_tx_busy_flag = false;
        dtr_enable = 0;
        return;
      }
    }
    ep_tx_busy_flag = true;
    memcpy(write_buffer, buf, len);
    if (!usbd_ep_start_write(CDC_IN_EP, write_buffer, len) == 0) {
      ep_tx_busy_flag = false;
    }
  }
#endif
}
