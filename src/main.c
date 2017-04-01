#include <Arduino.h>

#include <usb.h>
#include <usb_standard.h>
#include <instance/sercom0.h>

#include "audio_descriptors.h"

#include <component/usb.h>
#include <parts.h>
#include <io.h>
#include "samd/usb_samd.h"

#include "hw.h"

#define USB_DEVICE_EPINTFLAG_TRCPT0_Pos 0            /**< \brief (USB_DEVICE_EPINTFLAG) Transfer Complete 0 */
#define USB_DEVICE_EPINTFLAG_TRCPT0 (1 << USB_DEVICE_EPINTFLAG_TRCPT0_Pos)
#define USB_DEVICE_EPINTFLAG_TRCPT1_Pos 1            /**< \brief (USB_DEVICE_EPINTFLAG) Transfer Complete 1 */
#define USB_DEVICE_EPINTFLAG_TRCPT1 (1 << USB_DEVICE_EPINTFLAG_TRCPT1_Pos)

#define SERCOM0_GCLK_ID_SLOW 19

#define USB_EP0_SIZE 64
#define USB_EP1_SIZE AUDIO_STREAM_EPSIZE
#define USB_EP2_SIZE AUDIO_STREAM_EPSIZE

uint32_t led_on;
uint32_t is_loopback;
uint32_t is_local;
uint32_t is_remote;

#define MODE_ID (is_loopback ? 0 : 1)
#define MODE_SHIFT (is_loopback ? 0 : 4)
#define MODE_COMPLETION(active) (active + MODE_SHIFT)

#define OUTPUT_EP 0x01
#define INPUT_EP 0x82

#define OUTPUT_RING_CH 0x01
#define INPUT_RING_CH 0x02

#define RING_SIZE 4
#define RING_MASK 0x01
#define MS_BUFFER_SIZE 256
#define MS_BUFFER_LENGTH 192
#define MS_BUFFER_ZLP false

#define GCLK_SYSTEM 0
#define GCLK_32K    2

#define PIN_PA(pin) (pin)
#define PIN_PB(pin) (pin + 32)

#define FEATHER_D0 PIN_PA(11)
#define FEATHER_D1 PIN_PA(10)
#define FEATHER_D5 PIN_PA(15)
#define FEATHER_D6 PIN_PA(20)
#define FEATHER_D9 PIN_PA(7)
#define FEATHER_D10 PIN_PA(18)
#define FEATHER_D11 PIN_PA(16)
#define FEATHER_D12 PIN_PA(19)
#define FEATHER_D13 PIN_PA(17)

#define FEATHER_A0 PIN_PA(2)
#define FEATHER_A1 PIN_PB(8)
#define FEATHER_A2 PIN_PB(9)
#define FEATHER_A3 PIN_PA(4)
#define FEATHER_A4 PIN_PA(5)
#define FEATHER_A5 PIN_PB(2)
#define FEATHER_AREFA PIN_PA(3)
#define FEATHER_AREFB FEATHER_A3

#define FEATHER_UART_SERCOM 5
#define FEATHER_TX FEATHER_D1
#define FEATHER_RX FEATHER_D0

#define FEATHER_I2C_SERCOM 3
#define FEATHER_I2C_SERCOM_ALT 5
#define FEATHER_SCL PIN_PA(23)
#define FEATHER_SDA PIN_PA(22)

#define FEATHER_SPI_SERCOM 4
#define FEATHER_SCK PIN_PB(11)
#define FEATHER_MOSI PIN_PB(10)
#define FEATHER_MISO PIN_PA(12)
#define FEATHER_SCK_MUX MUX_PB11D_SERCOM4_PAD3
#define FEATHER_MOSI_MUX MUX_PB10D_SERCOM4_PAD2
#define FEATHER_MISO_MUX MUX_PA12D_SERCOM4_PAD0

#define FEATHER_LED FEATHER_D13

#define PIN_USB_DM 24
#define PIN_USB_DM_MUX MUX_PA24G_USB_DM
#define PIN_USB_DP 25
#define PIN_USB_DP_MUX MUX_PA25G_USB_DP
#define LED_PIN FEATHER_D13
// #define LED_PIN FEATHER_D1

#define SPI_GCLK 3
#define SPI_DMA_TX 4
#define SPI_DMA_RX 5
#define SPI_MDIPO 0
#define SPI_MDOPO 1
#define SPI_SDIPO 2
#define SPI_SDOPO 3
#define SPI_CPOL 1
#define SPI_CPHA 0
#define SPI_BAUD 5

#define PIN_SPI_LOOPBACK_JUMPER 40
#define PIN_SPI_LOCAL_JUMPER FEATHER_A0

#define SPI_SERCOM 4
#define PIN_SPI_SCK FEATHER_SCK /* 43 */
#define PIN_SPI_MOSI FEATHER_MOSI /* 42 */
#define PIN_SPI_MISO FEATHER_MISO /* 12 */
#define PIN_SPI_SS FEATHER_A2 /* 41 */
#define PIN_SPI_SCK_MUX MUX_PB11D_SERCOM4_PAD3
#define PIN_SPI_MOSI_MUX MUX_PB10D_SERCOM4_PAD2
#define PIN_SPI_MISO_MUX MUX_PA12D_SERCOM4_PAD0
#define PIN_SPI_SS_MUX MUX_PB09D_SERCOM4_PAD1

#define PIN_SPI_SEN FEATHER_D12

#define SPI_EIC_REMOTE_READY EIC_INTFLAG_EXTINT2
#define SPI_EIC_INTENSET_REMOTE_READY EIC_INTENSET_EXTINT2
#define PIN_SPI_REMOTE_READY FEATHER_D10 /* 18 */
#define PIN_SPI_REMOVE_READY_EIC_MUX MUX_PA16A_EIC_EXTINT2

// #define SPI_SERCOM 1
// #define PIN_SPI_SCK FEATHER_D12 /* 19 */
// #define PIN_SPI_MOSI FEATHER_D10 /* 18 */
// #define PIN_SPI_MISO FEATHER_D11 /* 16 */
// #define PIN_SPI_SS FEATHER_D13 /* 17 */
// #define PIN_SPI_SCK_MUX MUX_PA19C_SERCOM1_PAD3
// #define PIN_SPI_MOSI_MUX MUX_PA18C_SERCOM1_PAD2
// #define PIN_SPI_MISO_MUX MUX_PA16C_SERCOM1_PAD0
// #define PIN_SPI_SS_MUX MUX_PA17C_SERCOM1_PAD1
//
// #define PIN_SPI_SEN FEATHER_D6
//
// #define SPI_EIC_REMOTE_READY EIC_INTFLAG_EXTINT7
// #define SPI_EIC_INTENSET_REMOTE_READY EIC_INTENSET_EXTINT7
// #define PIN_SPI_REMOTE_READY FEATHER_D9 /* 7 */
// #define PIN_SPI_REMOVE_READY_EIC_MUX MUX_PA07A_EIC_EXTINT7

volatile uint8_t USB_DeviceState;
volatile uint8_t USB_Device_ConfigurationNumber;

USB_ENDPOINTS(3);

uint32_t tick = 0;
USB_ALIGN uint8_t output_buffer[MS_BUFFER_SIZE];
USB_ALIGN uint8_t input_buffer[MS_BUFFER_SIZE];
typedef struct {
    // bool ready;
    USB_ALIGN uint8_t buffer[MS_BUFFER_SIZE];
} __attribute__((packed)) ring_item;
typedef struct {
    uint8_t index;
    uint8_t active;
    uint8_t hot;
} __attribute__((packed)) ring_buffer;
USB_ALIGN ring_buffer ring = {
    .index = 0,
    .active = 0,
    .hot = 0,
};
USB_ALIGN ring_item ring_items[RING_SIZE];

// const USB_StringDescriptor msft_os = {
//     .bLength = 18,
//     .bDescriptorType = USB_DTYPE_String,
//     .bString = {'M','S','F','T','1','0','0',0xee},
// };
//
// const USB_MicrosoftCompatibleDescriptor msft_compatible = {
//     .dwLength = sizeof(USB_MicrosoftCompatibleDescriptor) + sizeof(USB_MicrosoftCompatibleDescriptor_Interface),
//     .bcdVersion = 0x0100,
//     .wIndex = 0x0004,
//     .bCount = 1,
//     .reserved = {0, 0, 0, 0, 0, 0, 0},
//     .interfaces = {
//         {
//             .bFirstInterfaceNumber = 0,
//             .reserved1 = 0,
//             .compatibleID = "WINUSB\0\0",
//             .subCompatibleID = {0, 0, 0, 0, 0, 0, 0, 0},
//             .reserved2 = {0, 0, 0, 0, 0, 0},
//         }
//     }
// };
//
// const USB_MicrosoftExtendedPropertiesDescriptor msft_extended = {
//     .dwLength = 146,
//     .bcdVersion = 0x0100,
//     .wIndex = 0x05,
//     .wCount = 0x01,
//     .dwPropLength = 136,
//     .dwType = 7,
//     .wNameLength = 42,
//     .name = u"DeviceInterfaceGUIDs\0",
//     .dwDataLength = 80,
//     .data = u"{3c33bbfd-71f9-4815-8b8f-7cd1ef928b3d}\0\0",
// };

/// Callback on reset
void usb_cb_reset(void) {}

#define MSFT_ID 0xee

/// Callback when a setup packet is received
void usb_cb_control_setup(void) {
	uint8_t recipient = usb_setup.bmRequestType & USB_REQTYPE_RECIPIENT_MASK;
    if (recipient == USB_RECIPIENT_DEVICE) {
        switch(usb_setup.bRequest) {
            case MSFT_ID:
                // led_on = 1;
                break;
            // case MSFT_ID: return handle_msft_compatible(&msft_compatible, &msft_extended);
            // case REQ_PWR: return req_gpio(usb_setup.wIndex, usb_setup.wValue);
            // case REQ_INFO: return req_info(usb_setup.wIndex);
            // case REQ_BOOT: return req_boot();
            // case REQ_RESET: return req_reset();
            // case REQ_OPENWRT_BOOT_STATUS: return req_boot_status();
        }
    }
    if (usb_setup.bmRequestType == AUDIO_REQUEST_TYPE) {
        // led_on = usb_setup.wValue >> 8 == 0x02;
        if (usb_setup.wIndex >> 8 == FDS_OUTPUT_FEATURE_ID) {
            if (usb_setup.wValue >> 8 == AUDIO_FEATURE_VOLUME) {
                switch (usb_setup.bRequest) {
                    case AUDIO_REQ_GET_CUR:
                        ep0_buf_in[0] = 0x00;
                        ep0_buf_in[1] = 0x00;
                        usb_ep0_in(2);
                        return usb_ep0_out();
                    case AUDIO_REQ_GET_MIN:
                        ep0_buf_in[0] = 0xfb;
                        ep0_buf_in[1] = 0xf3;
                        usb_ep0_in(2);
                        return usb_ep0_out();
                    case AUDIO_REQ_GET_MAX:
                        ep0_buf_in[0] = 0x00;
                        ep0_buf_in[1] = 0x00;
                        usb_ep0_in(2);
                        return usb_ep0_out();
                    case AUDIO_REQ_GET_RES:
                        ep0_buf_in[0] = 0x01;
                        ep0_buf_in[1] = 0x00;
                        usb_ep0_in(2);
                        return usb_ep0_out();
                }
            }
        }
                // led_on = 1;
    }
    return usb_ep0_stall();
}

void fds_noop_completion(uint32_t summary) {}

static inline void fds_start_inout() {
    uint8_t index = ring.index;
    ring.index = ring.index + 1 & RING_MASK;
    usb_ep_start_out(OUTPUT_EP, ring_items[ring.index].buffer, MS_BUFFER_LENGTH);
    usb_ep_start_in(INPUT_EP, ring_items[index].buffer, MS_BUFFER_LENGTH, MS_BUFFER_ZLP);
}

void fds_inout_completion(uint32_t summary) {
    // led_on = 1;

    if (summary & (OUTPUT_RING_CH | INPUT_RING_CH)) {
        fds_start_inout();
    }
    else if (summary & OUTPUT_RING_CH) {
        if (ring.hot & INPUT_RING_CH == 0x00) {
            ring.hot |= INPUT_RING_CH;
            fds_start_inout();
        }
        else {
            ring.hot &= ~OUTPUT_RING_CH;
        }
    }
    else if (summary & INPUT_RING_CH) {
        if (ring.hot & OUTPUT_RING_CH == 0x00) {
            ring.hot |= OUTPUT_RING_CH;
            fds_start_inout();
        }
    }
    else {
        if (ring.hot & (OUTPUT_RING_CH | INPUT_RING_CH) == 0x00) {
            ring.hot |= (OUTPUT_RING_CH | INPUT_RING_CH);
            fds_start_inout();
        }
    }
}

void fds_out_completion(uint32_t summary) {
    ring.hot &= ~summary;

    if (ring.hot & OUTPUT_RING_CH == 0x00) {
        ring.hot |= OUTPUT_RING_CH;
        usb_ep_start_out(OUTPUT_EP, ring_items[ring.index].buffer, MS_BUFFER_LENGTH);
    }
}

void fds_in_completion(uint32_t summary) {
    ring.hot &= ~summary;

    if (ring.hot & INPUT_RING_CH == 0x00) {
        ring.hot |= INPUT_RING_CH;
        usb_ep_start_in(INPUT_EP, ring_items[ring.index].buffer, MS_BUFFER_LENGTH, MS_BUFFER_ZLP);
    }
}

static inline void fds_local_start_inout() {
    uint8_t index = ring.index;
    ring.index = ring.index + 1 & RING_MASK;
    usb_ep_start_out(OUTPUT_EP, ring_items[ring.index].buffer, MS_BUFFER_LENGTH);
    usb_ep_start_in(INPUT_EP, ring_items[ring.index + 2].buffer, MS_BUFFER_LENGTH, MS_BUFFER_ZLP);

    if (is_remote) {
        // memcpy(output_buffer, ring_items[index].buffer, MS_BUFFER_LENGTH);
        // memcpy(ring_items[index].buffer, ring_items[index].buffer + 4, 4);

        // pin_pull_down(PIN_SPI_SS);
        // pin_mux(PIN_SPI_SS, PIN_SPI_SS_MUX);
        // pin_low(PIN_SPI_SS);

        sercom_spi_slave_init(SPI_SERCOM, SPI_SDIPO, SPI_SDOPO,
            SPI_CPOL, SPI_CPHA);
        SERCOM4->SPI.INTENSET.bit.SSL = 1;
        // sercom(SPI_SERCOM)->SPI.INTENSET.bit.TXC = 1;

        sercom(SPI_SERCOM)->SPI.DATA.reg = ring_items[index].buffer[MS_BUFFER_LENGTH - 1];

        // dma_sercom_start_rx(SPI_DMA_RX, SPI_SERCOM, ring_items[index + 2].buffer, MS_BUFFER_LENGTH);
        // dma_sercom_start_tx(SPI_DMA_TX, SPI_SERCOM, ring_items[ring.index].buffer, MS_BUFFER_LENGTH);

        pin_high(PIN_SPI_REMOTE_READY);
    }
    else {
        // pin_low(PIN_SPI_SS);
    }
}

void fds_local_inout_completion(uint32_t summary) {
    if (summary & (OUTPUT_RING_CH | INPUT_RING_CH)) {
        fds_local_start_inout();
    }
    else if (summary & OUTPUT_RING_CH) {
        if (ring.hot & INPUT_RING_CH == 0x00) {
            ring.hot |= INPUT_RING_CH;
            fds_local_start_inout();
        }
        else {
            ring.hot &= ~OUTPUT_RING_CH;
        }
    }
    else if (summary & INPUT_RING_CH) {
        if (ring.hot & OUTPUT_RING_CH == 0x00) {
            ring.hot |= OUTPUT_RING_CH;
            fds_local_start_inout();
        }
    }
    else {
        if (ring.hot & (OUTPUT_RING_CH | INPUT_RING_CH) == 0x00) {
            ring.hot |= (OUTPUT_RING_CH | INPUT_RING_CH);
            fds_local_start_inout();
        }
    }
}
void fds_local_out_completion(uint32_t summary) {
    ring.hot &= ~summary;

    if (ring.hot & OUTPUT_RING_CH == 0x00) {
        ring.hot |= OUTPUT_RING_CH;
        uint8_t index = ring.index;
        ring.index = ring.index + 1 & RING_MASK;
        usb_ep_start_out(OUTPUT_EP, ring_items[index].buffer, MS_BUFFER_LENGTH);
        if (is_remote) {
            sercom_spi_slave_init(SPI_SERCOM, SPI_SDIPO, SPI_SDOPO,
                SPI_CPOL, SPI_CPHA);
            SERCOM4->SPI.INTENSET.bit.SSL = 1;

            sercom(SPI_SERCOM)->SPI.DATA.reg = ring_items[ring.index].buffer[MS_BUFFER_LENGTH - 1];

            // dma_sercom_start_rx(SPI_DMA_RX, SPI_SERCOM, NULL, MS_BUFFER_LENGTH);
            // dma_sercom_start_tx(SPI_DMA_TX, SPI_SERCOM, ring_items[ring.index].buffer, MS_BUFFER_LENGTH);

            pin_high(PIN_SPI_REMOTE_READY);
        }
        else {
            // pin_low(PIN_SPI_SS);
        }
    }
}
void fds_local_in_completion(uint32_t summary) {
    ring.hot &= ~summary;

    if (ring.hot & INPUT_RING_CH == 0x00) {
        ring.hot |= INPUT_RING_CH;
        uint8_t index = ring.index;
        ring.index = ring.index + 1 & RING_MASK;
        usb_ep_start_in(INPUT_EP, ring_items[index].buffer, MS_BUFFER_LENGTH, MS_BUFFER_ZLP);
        if (is_remote) {
            sercom_spi_slave_init(SPI_SERCOM, SPI_SDIPO, SPI_SDOPO,
                SPI_CPOL, SPI_CPHA);
            sercom(SPI_SERCOM)->SPI.INTENSET.bit.SSL = 1;
            sercom(SPI_SERCOM)->SPI.INTENSET.bit.RXC = 1;

            sercom(SPI_SERCOM)->SPI.DATA.reg = 0;

            // dma_sercom_start_rx(SPI_DMA_RX, SPI_SERCOM, ring_items[ring.index].buffer, MS_BUFFER_LENGTH);
            // dma_sercom_start_tx(SPI_DMA_TX, SPI_SERCOM, NULL, MS_BUFFER_LENGTH);

            pin_high(PIN_SPI_REMOTE_READY);
        }
        else {
            // pin_low(PIN_SPI_SS);
        }
    }
}

static inline void fds_remote_start_inout() {
    uint8_t index = ring.index;
    ring.index = ring.index + 1 & RING_MASK;
    usb_ep_start_out(OUTPUT_EP, ring_items[ring.index].buffer, MS_BUFFER_LENGTH);
    usb_ep_start_in(INPUT_EP, ring_items[index + 2].buffer, MS_BUFFER_LENGTH, MS_BUFFER_ZLP);
    dma_sercom_start_rx(SPI_DMA_RX, SPI_SERCOM, ring_items[ring.index + 2].buffer, MS_BUFFER_LENGTH);
    dma_sercom_start_tx(SPI_DMA_TX, SPI_SERCOM, ring_items[index].buffer, MS_BUFFER_LENGTH);
}

void fds_remote_inout_completion(uint32_t summary) {
    if (summary & (OUTPUT_RING_CH | INPUT_RING_CH)) {
        fds_remote_start_inout();
    }
    else if (summary & OUTPUT_RING_CH) {
        if (ring.hot & INPUT_RING_CH == 0x00) {
            ring.hot |= INPUT_RING_CH;
            fds_remote_start_inout();
        }
        else {
            ring.hot &= ~OUTPUT_RING_CH;
        }
    }
    else if (summary & INPUT_RING_CH) {
        if (ring.hot & OUTPUT_RING_CH == 0x00) {
            ring.hot |= OUTPUT_RING_CH;
            fds_remote_start_inout();
        }
    }
    else {
        if (ring.hot & (OUTPUT_RING_CH | INPUT_RING_CH) == 0x00) {
            ring.hot |= (OUTPUT_RING_CH | INPUT_RING_CH);
            fds_remote_start_inout();
        }
    }
}
void fds_remote_out_completion(uint32_t summary) {
    ring.hot &= ~summary;

    if (ring.hot & OUTPUT_RING_CH == 0x00) {
        ring.hot |= OUTPUT_RING_CH;
        uint8_t index = ring.index;
        ring.index = ring.index + 1 & RING_MASK;
        usb_ep_start_out(OUTPUT_EP, ring_items[index].buffer, MS_BUFFER_LENGTH);
        dma_sercom_start_tx(SPI_DMA_TX, SPI_SERCOM, ring_items[ring.index].buffer, MS_BUFFER_LENGTH);
    }
}
void fds_remote_in_completion(uint32_t summary) {
    ring.hot &= ~summary;

    if (ring.hot & INPUT_RING_CH == 0x00) {
        ring.hot |= INPUT_RING_CH;
        uint8_t index = ring.index;
        ring.index = ring.index + 1 & RING_MASK;
        usb_ep_start_in(INPUT_EP, ring_items[index].buffer, MS_BUFFER_LENGTH, MS_BUFFER_ZLP);
        dma_sercom_start_rx(SPI_DMA_RX, SPI_SERCOM, ring_items[ring.index].buffer, MS_BUFFER_LENGTH);
    }
}

typedef void (*fds_completion_handle)(uint32_t);

fds_completion_handle fds_usb_completion;
fds_completion_handle fds_completions[8];

/// Callback on a completion interrupt
void usb_cb_completion(uint32_t summary) {
}
void usb_cb_control_in_completion(void) {
}
void usb_cb_control_out_completion(void) {
}

/// Callback for a SET_CONFIGURATION request
bool usb_cb_set_configuration(uint8_t config) {
    return true;
}

void fds_output_init(void) {
    // led_on = 1;

    usb_enable_ep(OUTPUT_EP, USB_EP_TYPE_ISOCHRONOUS, USB_EP1_SIZE);

    ring.active |= OUTPUT_RING_CH;

    fds_usb_completion = fds_completions[MODE_COMPLETION(ring.active)];

    ring.index = 0;

    ring.hot |= OUTPUT_RING_CH;
    usb_ep_start_out(OUTPUT_EP, ring_items[ring.index].buffer, MS_BUFFER_LENGTH);
}

void fds_output_disable(void) {
    led_on = 0;

    usb_disable_ep(OUTPUT_EP);

    ring.hot &= ~OUTPUT_RING_CH;
    ring.active &= ~OUTPUT_RING_CH;

    fds_usb_completion = fds_completions[MODE_COMPLETION(ring.active)];

    // if (!is_loopback) {
    //     dma_abort(SPI_DMA_TX);
    // }
}

void fds_input_init(void) {
    // led_on = 1;

    usb_enable_ep(INPUT_EP, USB_EP_TYPE_ISOCHRONOUS, USB_EP2_SIZE);

    ring.active |= INPUT_RING_CH;

    fds_usb_completion = fds_completions[MODE_COMPLETION(ring.active)];

    // usb_ep_start_in(INPUT_EP, NULL, 0, false);
}

void fds_input_disable(void) {
    led_on = 0;

    usb_disable_ep(INPUT_EP);

    ring.hot &= ~INPUT_RING_CH;
    ring.active &= ~INPUT_RING_CH;

    fds_usb_completion = fds_completions[MODE_COMPLETION(ring.active)];

    // if (!is_loopback) {
    //     dma_abort(SPI_DMA_RX);
    // }

    // dma_sercom_start_rx(SPI_DMA_RX, SPI_SERCOM, ring_items[ring.index + 2].buffer, MS_BUFFER_LENGTH);
    // dma_sercom_start_tx(SPI_DMA_TX, SPI_SERCOM, ring_items[ring.index].buffer, MS_BUFFER_LENGTH);
}

enum FDSInterface {
    FDSInterface_0,
    FDSInterface_Output,
    FDSInterface_Input,
};

enum FDSAltSetting {
    FDSAltSetting_0,
    FDSAltSetting_On,
};

/// Callback for a SET_INTERFACE request
bool usb_cb_set_interface(uint16_t interface, uint16_t altsetting) {
    if (interface == FDSInterface_Output) {
        if (altsetting == FDSAltSetting_On) {
            if (ring.active & OUTPUT_RING_CH) {
                fds_output_disable();
            }
            fds_output_init();
        }
        else {
            fds_output_disable();
        }
    }
    else if (interface == FDSInterface_Input) {
        if (altsetting == FDSAltSetting_On) {
            if (ring.active & INPUT_RING_CH) {
                fds_input_disable();
            }
            fds_input_init();
        }
        else {
            fds_input_disable();
        }
    }
    return true;
}

typedef uint8_t u8;

#define NVM_DFLL_COARSE_POS    58
#define NVM_DFLL_COARSE_SIZE   6
#define NVM_DFLL_FINE_POS      64
#define NVM_DFLL_FINE_SIZE     10

uint32_t dfll_nvm_val() {
  uint32_t coarse = ( *((uint32_t *)(NVMCTRL_OTP4)
      + (NVM_DFLL_COARSE_POS / 32))
    >> (NVM_DFLL_COARSE_POS % 32))
    & ((1 << NVM_DFLL_COARSE_SIZE) - 1);
  if (coarse == 0x3f) {
    coarse = 0x1f;
  }
  uint32_t fine = ( *((uint32_t *)(NVMCTRL_OTP4)
      + (NVM_DFLL_FINE_POS / 32))
    >> (NVM_DFLL_FINE_POS % 32))
    & ((1 << NVM_DFLL_FINE_SIZE) - 1);
  if (fine == 0x3ff) {
    fine = 0x1ff;
  }

  return SYSCTRL_DFLLVAL_COARSE(coarse) | SYSCTRL_DFLLVAL_FINE(fine);
}

void dfll_wait_for_sync() {
  while (!SYSCTRL->PCLKSR.bit.DFLLRDY);
}

void gclk_enable(uint32_t id, uint32_t src, uint32_t div) {
  GCLK->GENDIV.reg = GCLK_GENDIV_ID(id) | GCLK_GENDIV_DIV(div);
  GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(id) | GCLK_GENCTRL_GENEN | GCLK_GENCTRL_SRC(src);
}

void gclk_init() {
  // Various bits in the INTFLAG register can be set to one at startup.
  SYSCTRL->INTFLAG.reg = SYSCTRL_INTFLAG_BOD33RDY | SYSCTRL_INTFLAG_BOD33DET |
      SYSCTRL_INTFLAG_DFLLRDY;

  NVMCTRL->CTRLB.bit.RWS = 2;

  // Initialize GCLK
  PM->APBAMASK.reg |= PM_APBAMASK_GCLK;
  GCLK->CTRL.reg = GCLK_CTRL_SWRST;
  while (GCLK->CTRL.reg & GCLK_CTRL_SWRST);

  // SERCOM slow clock (Shared by all SERCOM)
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN |
      GCLK_CLKCTRL_GEN(0) |
      GCLK_CLKCTRL_ID(SERCOM0_GCLK_ID_SLOW);
}

void clock_init_crystal(u8 clk_system, u8 clk_32k) {
  gclk_init();

  SYSCTRL->XOSC32K.reg
    = SYSCTRL_XOSC32K_ENABLE
    | SYSCTRL_XOSC32K_XTALEN
    | SYSCTRL_XOSC32K_EN32K
    | SYSCTRL_XOSC32K_AAMPEN
    | SYSCTRL_XOSC32K_RUNSTDBY;

  gclk_enable(clk_32k, GCLK_SOURCE_XOSC32K, 1);

  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN |
      GCLK_CLKCTRL_GEN(clk_32k) |
      GCLK_CLKCTRL_ID(SYSCTRL_GCLK_ID_DFLL48);

  SYSCTRL->DFLLCTRL.reg = SYSCTRL_DFLLCTRL_ENABLE;
  dfll_wait_for_sync();
  SYSCTRL->DFLLVAL.reg = dfll_nvm_val();
  dfll_wait_for_sync();
  SYSCTRL->DFLLMUL.reg
    = SYSCTRL_DFLLMUL_MUL(1465) // round(48000000 / 32768)
    | SYSCTRL_DFLLMUL_CSTEP((0x1f / 4))
    | SYSCTRL_DFLLMUL_FSTEP((0xff / 4));
  dfll_wait_for_sync();
  SYSCTRL->DFLLCTRL.reg = SYSCTRL_DFLLCTRL_ENABLE | SYSCTRL_DFLLCTRL_MODE;
  dfll_wait_for_sync();
  SYSCTRL->DFLLCTRL.reg = SYSCTRL_DFLLCTRL_ENABLE | SYSCTRL_DFLLCTRL_MODE | SYSCTRL_DFLLCTRL_ONDEMAND;

  gclk_enable(clk_system, GCLK_SOURCE_DFLL48M, 1);
  while (GCLK->STATUS.bit.SYNCBUSY);
}

int main() {
    clock_init_crystal(GCLK_SYSTEM, GCLK_32K);

    init();

    fds_completions[0] = fds_noop_completion;
    fds_completions[1] = fds_out_completion;
    fds_completions[2] = fds_in_completion;
    fds_completions[3] = fds_inout_completion;
    fds_completions[4] = fds_noop_completion;
    fds_completions[5] = fds_local_out_completion;
    fds_completions[6] = fds_local_in_completion;
    fds_completions[7] = fds_local_inout_completion;

    fds_usb_completion = fds_completions[MODE_COMPLETION(ring.active)];

    // memset(&ring, 0, sizeof(ring));

    // // Set pin 13 to out
    // pin_out(12);

    // Set pin 13 to high
    // led_on = true;
    pin_out(LED_PIN);
    pin_set(LED_PIN, false);
    // pinMode(13, OUTPUT);
    // digitalWrite(13, HIGH);

    NVIC_ClearPendingIRQ(TCC0_IRQn);
    NVIC_SetPriority(TCC0_IRQn, 0xff);
    NVIC_EnableIRQ(TCC0_IRQn);

    // Configure a time based interrupt.
    PM->APBCMASK.reg |= 1 << (PM_APBCMASK_TCC0_Pos + 0);

    // GCLK->CLKCTRL.reg |= GCLK_CLKCTRL_ID(TCC0_GCLK_ID);
    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN |
        GCLK_CLKCTRL_GEN(0) |
        GCLK_CLKCTRL_ID(TCC0_GCLK_ID + 0/2);

    TCC0->PER.bit.PER = 500;
    TCC0->CTRLBSET.reg = TCC_CTRLBSET_CMD_RETRIGGER;

    while (TCC0->SYNCBUSY.reg > 0);

    TCC0->CTRLA.reg = TCC_CTRLA_PRESCALER_DIV1 | TCC_CTRLA_RUNSTDBY | TC_CTRLA_PRESCSYNC_PRESC;
    TCC0->CTRLBSET.reg = TCC_CTRLBSET_DIR; // | TCC_CTRLBSET_ONESHOT;

    while (TCC0->SYNCBUSY.reg > 0);

    TCC0->CTRLA.bit.ENABLE = 1;
    TCC0->INTENSET.reg = TCC_INTENSET_OVF;

    Tcc *t0 = TCC0;
    // t0->CTRLA.bit.PRESCALER = 3;
    // t0->CTRLA.bit.RUNSTDBY = 1;
    // t0->CTRLA.bit.PRESCSYNC = 1;
    // t0->PER.bit.PER = 125;
    // t0->INTENSET.bit.OVF = 1;
    // t0->CTRLA.bit.ENABLE = 1;

    NVIC_SetPriority(USB_IRQn, 0);
    pin_mux(PIN_USB_DM, MUX_PA24G_USB_DM);
    pin_mux(PIN_USB_DP, MUX_PA25G_USB_DP);
    usb_set_speed(USB_SPEED_FULL);
    usb_init();
    usb_attach();

    dma_init();
    NVIC_EnableIRQ(DMAC_IRQn);
    NVIC_SetPriority(DMAC_IRQn, 0xff);

    eic_init();
    NVIC_EnableIRQ(EIC_IRQn);
    NVIC_SetPriority(EIC_IRQn, 1);

    pin_in(PIN_SPI_LOOPBACK_JUMPER);
    is_loopback = pin_read(PIN_SPI_LOOPBACK_JUMPER);
    is_loopback = false;

    pin_in(PIN_SPI_LOCAL_JUMPER);
    is_local = pin_read(PIN_SPI_LOCAL_JUMPER);
    is_remote = !is_local && !is_loopback;

    // if (is_local) {
    //     pin_in(PIN_SPI_REMOTE_READY);
    //     pin_mux_eic(PIN_SPI_REMOTE_READY);
    //     eic_config(PIN_SPI_REMOTE_READY, EIC_CONFIG_SENSE_RISE);
    // }

    __DMB();
    __enable_irq();

    if (!is_loopback) {
        NVIC_EnableIRQ(SERCOM4_IRQn);
        NVIC_SetPriority(SERCOM4_IRQn, 1);

        // set up clock in case we need to use a divider
        sercom_clock_enable(SPI_SERCOM, GCLK_SYSTEM, 1);

        dma_sercom_configure_tx(SPI_DMA_TX, SPI_SERCOM);
        // if (is_local) {
        //     DMAC->CHCTRLB.reg = DMAC_CHCTRLB_LVL_LVL1 | DMAC_CHCTRLB_TRIGACT_BEAT | DMAC_CHCTRLB_TRIGSRC(SPI_SERCOM * 2 + 2);
        // }
        // if (is_remote) {
        //     DMAC->CHCTRLB.reg = DMAC_CHCTRLB_LVL_LVL1 | DMAC_CHCTRLB_TRIGACT_BEAT | DMAC_CHCTRLB_TRIGSRC(SPI_SERCOM * 2 + 1);
        // }
        dma_sercom_configure_rx(SPI_DMA_RX, SPI_SERCOM);
        // if (is_local) {
        //     DMAC->CHCTRLB.reg = DMAC_CHCTRLB_LVL_LVL0 | DMAC_CHCTRLB_TRIGACT_BEAT | DMAC_CHCTRLB_TRIGSRC(SPI_SERCOM * 2 + 1);
        // }
        // if (is_remote) {
        //     DMAC->CHCTRLB.reg = DMAC_CHCTRLB_LVL_LVL0 | DMAC_CHCTRLB_TRIGACT_BEAT | DMAC_CHCTRLB_TRIGSRC(SPI_SERCOM * 2 + 1);
        // }
        // dma_enable_interrupt(SPI_DMA_TX);
        dma_enable_interrupt(SPI_DMA_RX);

        if (is_local) {
            pin_out(PIN_SPI_MOSI);
            pin_out(PIN_SPI_SCK);
            pin_in(PIN_SPI_MISO);
            pin_out(PIN_SPI_SS);
        }
        else {
            pin_in(PIN_SPI_MOSI);
            pin_in(PIN_SPI_SCK);
            pin_out(PIN_SPI_MISO);
            pin_in(PIN_SPI_SS);
        }

        pin_mux(PIN_SPI_MOSI, PIN_SPI_MOSI_MUX);
        pin_mux(PIN_SPI_MISO, PIN_SPI_MISO_MUX);
        pin_mux(PIN_SPI_SCK, PIN_SPI_SCK_MUX);
        pin_mux(PIN_SPI_SS, PIN_SPI_SS_MUX);

        if (is_local) {
            sercom_spi_master_init(SPI_SERCOM, SPI_MDIPO, SPI_MDOPO,
                SPI_CPOL, SPI_CPHA, SPI_BAUD);

            pin_in(PIN_SPI_REMOTE_READY);
            pin_mux_eic(PIN_SPI_REMOTE_READY);
            eic_config(PIN_SPI_REMOTE_READY, EIC_CONFIG_SENSE_RISE);
            EIC->INTENSET.reg = SPI_EIC_INTENSET_REMOTE_READY;

            pin_out(PIN_SPI_SEN);
            pin_high(PIN_SPI_SEN);
        }
        else if (is_remote) {
            sercom_spi_slave_init(SPI_SERCOM, SPI_SDIPO, SPI_SDOPO,
                SPI_CPOL, SPI_CPHA);

            pin_out(PIN_SPI_REMOTE_READY);
            pin_low(PIN_SPI_REMOTE_READY);
        }
    }

    // NVIC_SetPriority(USB_IRQn, 0xff);

    // led_on = !led_on;
    // pin_set(LED_PIN, false);

    // led_on = 1;

    while (1) { __NOP(); }

    // while (1) {
    //     led_on = !led_on;
    //     pin_set(LED_PIN, led_on);
    //
    //     int i = 2400000;
    //     while (i--) {}
    // }

    return 0;
}

uint32_t sof_tick = 0;
uint32_t dma_tick = 0;

void spi_dma_completion(void) {
    dma_tick += 1;
    sof_tick = 0;

    if (is_remote) {
        // led_on = 1;
    //     pin_low(PIN_SPI_REMOTE_READY);
        // pin_out(PIN_SPI_SS);
    }
    if (is_local) {
        // uint8_t index = ring.index + 1 & RING_MASK;
        // ring_items[index + 2].buffer[0] = sercom(SPI_SERCOM)->SPI.DATA.reg;

        // pin_out(PIN_SPI_SEN);
        // pin_high(PIN_SPI_SEN);
        // sercom_reset(SPI_SERCOM);

        uint8_t index = ring.index + 1 & RING_MASK;
        // memcpy(ring_items[index + 2].buffer, ring_items[index + 2].buffer + 4, 4);
        // uint8_t *buffer = ring_items[index + 2].buffer;
        // buffer[0] = buffer[4];
        // buffer[1] = buffer[5];
        // buffer[2] = buffer[6];
        // buffer[3] = buffer[7];
        // buffer[MS_BUFFER_LENGTH - 4] = buffer[MS_BUFFER_LENGTH - 8];
        // buffer[MS_BUFFER_LENGTH - 3] = buffer[MS_BUFFER_LENGTH - 7];
        // buffer[MS_BUFFER_LENGTH - 2] = buffer[MS_BUFFER_LENGTH - 6];
        // buffer[MS_BUFFER_LENGTH - 1] = buffer[MS_BUFFER_LENGTH - 5];

        // int16_t *buffer = ring_items[index + 2].buffer;
        // int16_t tmp = buffer[2];
        // int16_t tmp2 = buffer[4];
        // buffer[0] = tmp2 + (tmp - tmp2);
        // tmp = buffer[3];
        // tmp2 = buffer[5];
        // buffer[1] = tmp2 + (tmp - tmp2);
        // tmp = buffer[MS_BUFFER_LENGTH - 6];
        // tmp2 = buffer[MS_BUFFER_LENGTH - 4];
        // buffer[MS_BUFFER_LENGTH - 2] = tmp2 - (tmp - tmp2);
        // tmp = buffer[MS_BUFFER_LENGTH - 5];
        // tmp2 = buffer[MS_BUFFER_LENGTH - 3];
        // buffer[MS_BUFFER_LENGTH - 1] = tmp2- (tmp - tmp2);
    }
    // led_on += 1;

    // uint8_t index = ring.index + 1 & RING_MASK;
    // memcpy(input_buffer, ring_items[index + 2].buffer, MS_BUFFER_LENGTH);

    // uint8_t index = ring.index + 1 & RING_MASK;
    // memset(ring_items[index + 2].buffer, 0, MS_BUFFER_LENGTH);
}

void spi_start(void) {
    uint32_t size = 0;
    // dma_sercom_start_rx(SPI_DMA_RX, SPI_SERCOM, NULL, size);
    // dma_sercom_start_tx(SPI_DMA_TX, SPI_SERCOM, NULL, size);
}

void spi_remote_ready(void) {
    // led_on = 1;
    // __NOP();
    pin_high(PIN_SPI_SEN);

    if (ring.active & (OUTPUT_RING_CH | INPUT_RING_CH)) {
        sercom_spi_master_init(SPI_SERCOM, SPI_MDIPO, SPI_MDOPO,
            SPI_CPOL, SPI_CPHA, SPI_BAUD);

        // sercom(SPI_SERCOM)->SPI.INTFLAG.bit.RXC = 1;
        // sercom(SPI_SERCOM)->SPI.INTENSET.bit.RXC = 1;

        // sercom(SPI_SERCOM)->SPI.DATA.reg = 0;

        // do {
        //     uint8_t index = ring.index + 1 & RING_MASK;
        //     memset(ring_items[index + 2].buffer, 0, MS_BUFFER_LENGTH);
        // } while(0);

        pin_low(PIN_SPI_SEN);

        uint8_t index = ring.index + 1 & RING_MASK;
        dma_sercom_start_rx(SPI_DMA_RX, SPI_SERCOM, ring_items[index + 2].buffer, MS_BUFFER_LENGTH);
        dma_sercom_start_tx(SPI_DMA_TX, SPI_SERCOM, ring_items[index].buffer, MS_BUFFER_LENGTH);
    }
    else if (ring.active & (OUTPUT_RING_CH)) {
        sercom_spi_master_init(SPI_SERCOM, SPI_MDIPO, SPI_MDOPO,
            SPI_CPOL, SPI_CPHA, SPI_BAUD);

        pin_low(PIN_SPI_SEN);

        dma_sercom_start_rx(SPI_DMA_RX, SPI_SERCOM, NULL, MS_BUFFER_LENGTH);
        dma_sercom_start_tx(SPI_DMA_TX, SPI_SERCOM, ring_items[ring.index].buffer, MS_BUFFER_LENGTH);
    }
    else if (ring.active & (INPUT_RING_CH)) {
        sercom_spi_master_init(SPI_SERCOM, SPI_MDIPO, SPI_MDOPO,
            SPI_CPOL, SPI_CPHA, SPI_BAUD);

        pin_low(PIN_SPI_SEN);

        dma_sercom_start_rx(SPI_DMA_RX, SPI_SERCOM, ring_items[ring.index].buffer, MS_BUFFER_LENGTH);
        dma_sercom_start_tx(SPI_DMA_TX, SPI_SERCOM, NULL, MS_BUFFER_LENGTH);
    }

    // pin_high(PIN_SPI_SEN);
}

void DMAC_Handler(void) {
    // led_on = 1;

    u32 intpend = DMAC->INTPEND.reg;
    if (intpend & DMAC_INTPEND_TCMPL) {
        u32 id = intpend & DMAC_INTPEND_ID_Msk;

        if (id == SPI_DMA_RX) {
            spi_dma_completion();
        }
        // else if (id == SPI_DMA_TX) {
        //     spi_dma_completion();
        // }
    }

    if (intpend & (DMAC_INTPEND_TERR | DMAC_INTPEND_SUSP)) {
        // invalid();
    }

    u32 id = intpend & DMAC_INTPEND_ID_Msk;
    DMAC->CHID.reg = id;
    uint32_t flags = DMAC->CHINTFLAG.reg;
    DMAC->CHINTFLAG.reg = flags;

    DMAC->INTPEND.reg = intpend;
}

uint32_t eic_tick = 0;

void EIC_Handler(void) {
    led_on += 1;

    u32 flags = EIC->INTFLAG.reg;
    EIC->INTFLAG.reg = flags;
    if (flags & SPI_EIC_REMOTE_READY) {
        if (tick > 100) {
            eic_tick = 0;
        }

        tick = 0;

        eic_tick++;

        spi_remote_ready();
    }
}

uint8_t ring_index = 0;
uint32_t tx_index = 0;

void SERCOM4_Handler(void) {
    uint32_t flags = sercom(SPI_SERCOM)->SPI.INTFLAG.reg;
    if (flags & SERCOM_SPI_INTFLAG_TXC) {
        sercom(SPI_SERCOM)->SPI.DATA.reg = ring_items[ring_index].buffer[tx_index];
    }
    if (is_remote) {
        if (flags & SERCOM_SPI_INTFLAG_SSL) {
            uint8_t index = ring.index + 1 & RING_MASK;
            if (ring.active & (OUTPUT_RING_CH | INPUT_RING_CH)) {
                dma_sercom_start_rx(SPI_DMA_RX, SPI_SERCOM, ring_items[index + 2].buffer, MS_BUFFER_LENGTH);
                dma_sercom_start_tx(SPI_DMA_TX, SPI_SERCOM, ring_items[ring.index].buffer, MS_BUFFER_LENGTH);
                ring_index = index;
                tx_index = MS_BUFFER_LENGTH - 1;
            }
            else if (ring.active & OUTPUT_RING_CH) {
                dma_sercom_start_rx(SPI_DMA_RX, SPI_SERCOM, NULL, MS_BUFFER_LENGTH);
                dma_sercom_start_tx(SPI_DMA_TX, SPI_SERCOM, ring_items[ring.index].buffer, MS_BUFFER_LENGTH);
            }
            else if (ring.active & INPUT_RING_CH) {
                dma_sercom_start_rx(SPI_DMA_RX, SPI_SERCOM, ring_items[ring.index].buffer, MS_BUFFER_LENGTH);
                dma_sercom_start_tx(SPI_DMA_TX, SPI_SERCOM, NULL, MS_BUFFER_LENGTH);
            }

            // dma_sercom_start_tx(SPI_DMA_TX, SPI_SERCOM, ring_items[index].buffer, MS_BUFFER_LENGTH);
        }
    }
    if (is_local) {
        if (flags & SERCOM_SPI_INTFLAG_RXC) {
            sercom(SPI_SERCOM)->SPI.INTENCLR.bit.RXC = 1;

            uint8_t index = ring.index + 1 & RING_MASK;
            // ring_items[index + 2].buffer[MS_BUFFER_LENGTH - 1] = sercom(SPI_SERCOM)->SPI.DATA.reg;
            // dma_sercom_start_rx(SPI_DMA_RX, SPI_SERCOM, ring_items[index + 2].buffer, MS_BUFFER_LENGTH - 1);
        }
    }

    SERCOM4->SPI.INTFLAG.reg = flags;
}

void TCC0_Handler(void) {
    tick += 1;

    // led_on = !led_on;
    pin_set(LED_PIN, led_on % 1000 == 1);
    // pin_set(LED_PIN, eic_tick % 1000 == 1);

    // Tcc *t0 = TCC0;
    TCC0->INTFLAG.reg = TCC_INTFLAG_OVF;
}

void usb_cb_sof(void) {
    if (is_remote) {
        pin_low(PIN_SPI_REMOTE_READY);
    }
    fds_usb_completion(ring.hot);

    sof_tick += 1;
    if (!is_loopback && sof_tick > 10) {
        memset(ring_items, 0, sizeof(ring_items));
        sof_tick = 0;
    }
}
