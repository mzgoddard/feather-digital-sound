#include <Arduino.h>

#include <usb.h>
#include <usb_standard.h>
#include <instance/sercom0.h>

#include "audio_descriptors.h"

#include <component/usb.h>
#include <parts.h>
#include <io.h>
#include "samd/usb_samd.h"

#define USB_DEVICE_EPINTFLAG_TRCPT0_Pos 0            /**< \brief (USB_DEVICE_EPINTFLAG) Transfer Complete 0 */
#define USB_DEVICE_EPINTFLAG_TRCPT0 (1 << USB_DEVICE_EPINTFLAG_TRCPT0_Pos)
#define USB_DEVICE_EPINTFLAG_TRCPT1_Pos 1            /**< \brief (USB_DEVICE_EPINTFLAG) Transfer Complete 1 */
#define USB_DEVICE_EPINTFLAG_TRCPT1 (1 << USB_DEVICE_EPINTFLAG_TRCPT1_Pos)

#define SERCOM0_GCLK_ID_SLOW 19

#define USB_EP0_SIZE 64
#define USB_EP1_SIZE AUDIO_STREAM_EPSIZE
#define USB_EP2_SIZE AUDIO_STREAM_EPSIZE

uint32_t led_on;

volatile uint8_t USB_DeviceState;
volatile uint8_t USB_Device_ConfigurationNumber;

USB_ENDPOINTS(3);

#define OUTPUT_EP 0x01
#define INPUT_EP 0x82

#define RING_SIZE 2
#define RING_MASK (RING_SIZE - 1)
#define MS_BUFFER_SIZE 192
#define MS_BUFFER_LENGTH 192
#define MS_BUFFER_ZLP false

uint32_t tick = 0;
uint8_t output_buffer[MS_BUFFER_SIZE];
typedef struct {
    // bool ready;
    uint8_t buffer[MS_BUFFER_SIZE];
} __attribute__((packed)) ring_item;
typedef struct {
    uint8_t output_index;
    bool output_stalled;
    bool output_active;
    bool output_hot;
    uint16_t output_tick;
    uint8_t input_available;
    uint8_t input_index;
    bool input_stalled;
    bool input_active;
    bool input_hot;
    uint16_t input_tick;
    bool item_ready[RING_SIZE];
    // ring_item items[RING_SIZE];
} __attribute__((packed)) ring_buffer;
USB_ALIGN ring_buffer ring = {
    .output_index = 0,
    .output_stalled = false,
    .output_active = false,
    .output_tick = 0,
    .input_available = 0,
    .input_index = 0,
    .input_stalled = false,
    .input_active = false,
    .input_tick = 0,
    .item_ready = {
        false,
        false,
    },
    // .items = {
    //     {
    //         .ready = false,
    //     },
    //     {
    //         .ready = false,
    //     },
    //     {
    //         .ready = false,
    //     },
    //     {
    //         .ready = false,
    //     },
    // },
};
USB_ALIGN ring_item ring_items[RING_SIZE];

// UsbDeviceDescriptor usb_endpoints[] = {};
// const uint8_t usb_num_endpoints = 0;

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
                led_on = 1;
    }
    return usb_ep0_stall();
}

void fds_noop_completion(uint32_t summary) {}

static inline void fds_start_inout() {
    ring.input_index = ring.output_index;
    ring.output_index = (ring.output_index + 1 & RING_MASK);
    usb_ep_start_out(OUTPUT_EP, ring_items[ring.output_index].buffer, MS_BUFFER_LENGTH);
    usb_ep_start_in(INPUT_EP, ring_items[ring.input_index].buffer, MS_BUFFER_LENGTH, MS_BUFFER_ZLP);
}

void fds_inout_completion(uint32_t summary) {
    if (summary & 0x06) {
        fds_start_inout();
    }
    else if (summary & 0x02) {
        // if (ring.hot & 0x02 == 0x00) {
        //     ring.hot |= 0x02;
        //     fds_start_inout();
        // }
        // else {
        //     ring.hot &= ~0x01;
        // }
        if (!ring.input_hot) {
            ring.input_hot = true;
            fds_start_inout();
        }
        else {
            ring.output_hot = false;
        }
    }
    else if (summary & 0x04) {
        // if (ring.hot & 0x01 == 0x00) {
        //     ring.hot |= 0x01;
        //     fds_start_inout();
        // }
        if (!ring.output_hot) {
            ring.output_hot = true;
            fds_start_inout();
        }
        else {
            ring.input_hot = false;
        }
    }
    else {
        // if (ring.hot & 0x03 == 0x00) {
        //     ring.hot |= 0x03;
        //     fds_start_inout();
        // }
        if (!ring.output_hot && !ring.input_hot) {
            ring.output_hot = true;
            ring.input_hot = true;
            fds_start_inout();
        }
    }
    // if (summary & 1 << 1) {
    //     if (summary & 1 << 2) {
    //         fds_start_inout();
    //     }
    //     else {
    //         if (!ring.input_hot) {
    //             ring.input_hot = true;
    //             fds_start_inout();
    //         }
    //         else {
    //             ring.output_hot = false;
    //         }
    //     }
    // }
    // else {
    //     if (summary & 1 << 2) {
    //         if (!ring.output_hot) {
    //             ring.output_hot = true;
    //             fds_start_inout();
    //         }
    //     }
    //     else {
    //         if (!ring.output_hot && !ring.input_hot) {
    //             ring.output_hot = true;
    //             ring.input_hot = true;
    //             fds_start_inout();
    //         }
    //     }
    // }
}

void fds_out_completion(uint32_t summary) {
    if (summary & 1 << 1) {
        ring.output_hot = false;
    }
    if (summary & 1 << 2) {
        ring.input_hot = false;
    }

    if (!ring.output_hot) {
        ring.output_hot = true;
        usb_ep_start_out(OUTPUT_EP, ring_items[ring.output_index].buffer, MS_BUFFER_LENGTH);
    }
}

void fds_in_completion(uint32_t summary) {
    if (summary & 1 << 1) {
        ring.output_hot = false;
    }
    if (summary & 1 << 2) {
        ring.input_hot = false;
    }

    if (!ring.input_hot) {
        ring.input_hot = true;
        usb_ep_start_in(INPUT_EP, ring_items[ring.output_index].buffer, MS_BUFFER_LENGTH, MS_BUFFER_ZLP);
    }
}

typedef void (*fds_completion_handle)(uint32_t);

fds_completion_handle fds_usb_completion;

/// Callback on a completion interrupt
void usb_cb_completion(uint32_t summary) {
    // led_on = 1;
    // uint32_t summary = USB->DEVICE.EPINTSMRY.reg;

    // led_on = summary > 1;

    // fds_usb_completion(summary);

    // if (summary & 1 << 1) {
    //     ring.output_hot = false;
    // }
    // if (summary & 1 << 2) {
    //     ring.input_hot = false;
    // }
    //
    // if (ring.output_active) {
    //     if (ring.input_active) {
    //         if (!ring.output_hot && !ring.input_hot) {
    //             ring.output_hot = true;
    //             ring.input_hot = true;
    //             usb_ep_start_in(INPUT_EP, ring_items[ring.output_index].buffer, MS_BUFFER_LENGTH, MS_BUFFER_ZLP);
    //             ring.output_index = !ring.output_index;
    //             usb_ep_start_out(OUTPUT_EP, ring_items[ring.output_index].buffer, MS_BUFFER_LENGTH);
    //         }
    //     }
    //     else {
    //         if (!ring.output_hot) {
    //             ring.output_hot = true;
    //             usb_ep_start_out(OUTPUT_EP, ring_items[ring.output_index].buffer, MS_BUFFER_LENGTH);
    //         }
    //     }
    // }
    // else if (ring.input_active) {
    //     if (!ring.input_hot) {
    //         ring.input_hot = true;
    //         usb_ep_start_in(INPUT_EP, ring_items[ring.output_index].buffer, MS_BUFFER_LENGTH, MS_BUFFER_ZLP);
    //     }
    // }

    // if (summary & (1 << 2)) {
    //     // led_on = 1;
    //
    //     usb_ep_handled(INPUT_EP);
    //
    //     if (ring.input_active && ring.input_hot) {
    //         ring.input_hot = false;
    //
    //         if (ring.output_active) {
    //             ring_items[ring.input_index].ready = false;
    //             ring.input_index = (ring.input_index + 1) % RING_SIZE;
    //         }
    //         else {
    //             ring_items[ring.input_index].ready = true;
    //             // memset(ring_items[ring.input_index].buffer, 0, MS_BUFFER_LENGTH);
    //             // memset(ring_items[ring.input_index].buffer, 0xfff0, 96);
    //             // memset(ring_items[ring.input_index].buffer + 96, 0x000f, 96);
    //         }
    //
    //         // if (!ring.output_hot && ring.output_stalled && !ring_items[ring.output_index].ready) {
    //         //     ring.output_hot = true;
    //         //     ring.output_stalled = false;
    //         //     usb_ep_start_out(OUTPUT_EP, ring_items[ring.output_index].buffer, MS_BUFFER_LENGTH);
    //         // }
    //
    //         // if (ring_items[ring.input_index].ready) {
    //         //     usb_ep_start_in(INPUT_EP, ring_items[ring.input_index].buffer, MS_BUFFER_LENGTH, MS_BUFFER_ZLP);
    //         // }
    //         // else {
    //         //     // usb_set_stall_ep(INPUT_EP);
    //         //     ring.input_stalled = true;
    //         // }
    //     }
    // }
    // if (summary & (1 << 1)) {
    //     // led_on = usb_endpoints[1].DeviceDescBank[0].PCKSIZE.bit.BYTE_COUNT == MS_BUFFER_LENGTH;
    //
    //     usb_ep_handled(OUTPUT_EP);
    //
    //     if (ring.output_active && ring.output_hot) {
    //         ring.output_hot = false;
    //
    //         if (ring.input_active) {
    //             ring_items[ring.output_index].ready = true;
    //             ring.output_index = (ring.output_index + 1) % RING_SIZE;
    //             if (ring.input_available < 2) {
    //                 ring.input_available += 1;
    //             }
    //         }
    //
    //         // if (ring.input_stalled && ring_items[ring.input_index].ready) {
    //         //     ring.input_stalled = false;
    //         //     usb_ep_start_in(INPUT_EP, ring_items[ring.input_index].buffer, MS_BUFFER_LENGTH, MS_BUFFER_ZLP);
    //         // }
    //
    //         // if (!ring_items[ring.output_index].ready) {
    //         //     ring.output_hot = true;
    //         //     usb_ep_start_out(OUTPUT_EP, ring_items[ring.output_index].buffer, MS_BUFFER_LENGTH);
    //         // }
    //         // else {
    //         //     // usb_set_stall_ep(OUTPUT_EP);
    //         //     ring.output_stalled = true;
    //         // }
    //     }
    //     if (!ring.input_hot && ring.output_active && !ring.output_hot) {
    //         if (!ring_items[ring.output_index].ready) {
    //             ring.output_hot = true;
    //             usb_ep_start_out(OUTPUT_EP, ring_items[ring.output_index].buffer, MS_BUFFER_LENGTH);
    //         }
    //         else {
    //             // usb_set_stall_ep(OUTPUT_EP);
    //             ring.output_stalled = true;
    //         }
    //     }
    // }
    // if (summary & (1 << 2)) {
    //     if (ring.output_active && !ring.output_hot) {
    //         if (!ring_items[ring.output_index].ready) {
    //             ring.output_hot = true;
    //             usb_ep_start_out(OUTPUT_EP, ring_items[ring.output_index].buffer, MS_BUFFER_LENGTH);
    //         }
    //         else {
    //             // usb_set_stall_ep(OUTPUT_EP);
    //             ring.output_stalled = true;
    //         }
    //     }
    // }

    // bool out_complete = USB->DEVICE.DeviceEndpoint[1].EPINTFLAG.reg == USB_DEVICE_EPINTFLAG_TRCPT0;
    // bool in_complete = USB->DEVICE.DeviceEndpoint[2].EPINTFLAG.reg == USB_DEVICE_EPINTFLAG_TRCPT1;

    // if (ring.output_active) {
    //     if (ring.input_active) {
    //         if (summary & 1 << 1) {
    //             if (summary & 1 << 2) {
    //                 usb_ep_start_in(INPUT_EP, ring_items[ring.output_index].buffer, MS_BUFFER_LENGTH, MS_BUFFER_ZLP);
    //                 ring.output_index = !ring.output_index;
    //                 usb_ep_start_out(OUTPUT_EP, ring_items[ring.output_index].buffer, MS_BUFFER_LENGTH);
    //             }
    //             else {
    //                 if (!ring.input_hot) {
    //                     ring.input_hot = true;
    //                     usb_ep_start_in(INPUT_EP, ring_items[ring.output_index].buffer, MS_BUFFER_LENGTH, MS_BUFFER_ZLP);
    //                     ring.output_index = !ring.output_index;
    //                     usb_ep_start_out(OUTPUT_EP, ring_items[ring.output_index].buffer, MS_BUFFER_LENGTH);
    //                 }
    //                 else {
    //                     ring.output_hot = false;
    //                 }
    //             }
    //         }
    //         else {
    //             if (summary & 1 << 2) {
    //                 ring.input_hot = false;
    //             }
    //             if (!ring.output_hot && !ring.input_hot) {
    //                 ring.output_hot = true;
    //                 ring.input_hot = true;
    //                 usb_ep_start_in(INPUT_EP, ring_items[ring.output_index].buffer, MS_BUFFER_LENGTH, MS_BUFFER_ZLP);
    //                 ring.output_index = !ring.output_index;
    //                 usb_ep_start_out(OUTPUT_EP, ring_items[ring.output_index].buffer, MS_BUFFER_LENGTH);
    //             }
    //         }
    //     }
    //     else {
    //         if (summary & 1 << 1) {
    //             ring.output_hot = false;
    //         }
    //         if (!ring.output_hot) {
    //             ring.output_hot = true;
    //             usb_ep_start_out(OUTPUT_EP, ring_items[ring.output_index].buffer, MS_BUFFER_LENGTH);
    //         }
    //     }
    // }
    // else if (ring.input_active) {
    //     if (summary & 1 << 2) {
    //         ring.input_hot = false;
    //     }
    //     if (!ring.output_hot && !ring.input_hot) {
    //         ring.input_hot = true;
    //         usb_ep_start_in(INPUT_EP, ring_items[ring.output_index].buffer, MS_BUFFER_LENGTH, MS_BUFFER_ZLP);
    //     }
    // }

    // if (ring.output_active) {
    //     if (ring.input_active) {
    //         if (ring.output_hot) {
    //             if (summary & 1 << 1) {
    //                 ring.output_hot = false;
    //                 ring.item_ready[ring.output_index] = true;
    //                 ring.output_index = (ring.output_index + 1) & RING_MASK;
    //
    //                 if (ring.input_hot) {
    //                     if (summary & 1 << 2) {
    //                         ring.item_ready[ring.input_index] = false;
    //                         ring.input_index = (ring.input_index + 1) & RING_MASK;
    //
    //                         if (!ring.item_ready[ring.output_index]) {
    //                             ring.output_hot = true;
    //                             usb_ep_start_out(OUTPUT_EP, ring_items[ring.output_index].buffer, MS_BUFFER_LENGTH);
    //                         }
    //                         if (ring.item_ready[ring.input_index]) {
    //                             usb_ep_start_in(INPUT_EP, ring_items[ring.input_index].buffer, MS_BUFFER_LENGTH, MS_BUFFER_ZLP);
    //                         }
    //                         else {
    //                             ring.input_hot = false;
    //                         }
    //                     }
    //                     else {
    //                         if (!ring.item_ready[ring.output_index]) {
    //                             ring.output_hot = true;
    //                             usb_ep_start_out(OUTPUT_EP, ring_items[ring.output_index].buffer, MS_BUFFER_LENGTH);
    //                         }
    //                         if (!ring.input_hot && ring.item_ready[ring.input_index]) {
    //                             ring.input_hot = true;
    //                             usb_ep_start_in(INPUT_EP, ring_items[ring.input_index].buffer, MS_BUFFER_LENGTH, MS_BUFFER_ZLP);
    //                         }
    //                     }
    //                 }
    //                 else {
    //                     if (!ring.item_ready[ring.output_index]) {
    //                         ring.output_hot = true;
    //                         usb_ep_start_out(OUTPUT_EP, ring_items[ring.output_index].buffer, MS_BUFFER_LENGTH);
    //                     }
    //                     if (!ring.input_hot && ring.item_ready[ring.input_index]) {
    //                         ring.input_hot = true;
    //                         usb_ep_start_in(INPUT_EP, ring_items[ring.input_index].buffer, MS_BUFFER_LENGTH, MS_BUFFER_ZLP);
    //                     }
    //                 }
    //             }
    //             else {
    //                 if (summary & 1 << 2 && ring.input_hot) {
    //                     ring.input_hot = false;
    //                     ring.item_ready[ring.input_index] = false;
    //                     ring.input_index = (ring.input_index + 1) & RING_MASK;
    //                 }
    //                 if (!ring.item_ready[ring.output_index]) {
    //                     ring.output_hot = true;
    //                     usb_ep_start_out(OUTPUT_EP, ring_items[ring.output_index].buffer, MS_BUFFER_LENGTH);
    //                 }
    //                 if (!ring.input_hot && ring.item_ready[ring.input_index]) {
    //                     ring.input_hot = true;
    //                     usb_ep_start_in(INPUT_EP, ring_items[ring.input_index].buffer, MS_BUFFER_LENGTH, MS_BUFFER_ZLP);
    //                 }
    //             }
    //         }
    //         else {
    //             if (summary & 1 << 2 && ring.input_hot) {
    //                 ring.input_hot = false;
    //                 ring.item_ready[ring.input_index] = false;
    //                 ring.input_index = (ring.input_index + 1) & RING_MASK;
    //             }
    //             if (!ring.output_hot && !ring.item_ready[ring.output_index]) {
    //                 ring.output_hot = true;
    //                 usb_ep_start_out(OUTPUT_EP, ring_items[ring.output_index].buffer, MS_BUFFER_LENGTH);
    //             }
    //             if (!ring.input_hot && ring.item_ready[ring.input_index]) {
    //                 ring.input_hot = true;
    //                 usb_ep_start_in(INPUT_EP, ring_items[ring.input_index].buffer, MS_BUFFER_LENGTH, MS_BUFFER_ZLP);
    //             }
    //         }
    //     }
    //     else {
    //         if (summary & 1 << 1 && ring.output_hot) {
    //             ring.output_hot = false;
    //         }
    //         if (summary & 1 << 2 && ring.input_hot) {
    //             ring.input_hot = false;
    //             ring.item_ready[ring.input_index] = false;
    //             ring.input_index = (ring.input_index + 1) & RING_MASK;
    //         }
    //         if (!ring.output_hot && !ring.item_ready[ring.output_index]) {
    //             ring.output_hot = true;
    //             usb_ep_start_out(OUTPUT_EP, ring_items[ring.output_index].buffer, MS_BUFFER_LENGTH);
    //         }
    //     }
    // }
    // else {
    //     if (ring.input_active) {
    //         if (summary & 1 << 1 && ring.output_hot) {
    //             ring.output_hot = false;
    //             ring.item_ready[ring.output_index] = true;
    //             ring.output_index = (ring.output_index + 1) & RING_MASK;
    //         }
    //         if (summary & 1 << 2 && ring.input_hot) {
    //             ring.input_hot = false;
    //         }
    //         if (!ring.input_hot && ring.item_ready[ring.input_index]) {
    //             ring.input_hot = true;
    //             usb_ep_start_in(INPUT_EP, ring_items[ring.input_index].buffer, MS_BUFFER_LENGTH, MS_BUFFER_ZLP);
    //         }
    //     }
    //     else {
    //         if (summary & 1 << 1 && ring.output_hot) {
    //             ring.output_hot = false;
    //         }
    //         if (summary & 1 << 2 && ring.input_hot) {
    //             ring.input_hot = false;
    //         }
    //     }
    // }

    // bool out_complete = summary & 1 << 1;
    // bool in_complete = summary & 1 << 2;
    //
    // if (out_complete) {
    //     if (ring.output_active && ring.output_hot) {
    //         ring.output_hot = false;
    //
    //         if (ring.input_active) {
    //             ring.item_ready[ring.output_index] = true;
    //             ring.output_index = (ring.output_index + 1) & RING_MASK;
    //         }
    //     }
    // }
    //
    // if (in_complete) {
    //     // led_on = 1;
    //     // usb_ep_handled(INPUT_EP);
    //
    //     if (ring.input_active && ring.input_hot) {
    //         ring.input_hot = false;
    //
    //         if (ring.output_active) {
    //             ring.item_ready[ring.input_index] = false;
    //             ring.input_index = (ring.input_index + 1) & RING_MASK;
    //         }
    //     }
    // }
    //
    // // if (ring.output_stalled)
    // if (ring.output_active && !ring.output_hot && !ring.item_ready[ring.output_index]) {
    //     ring.output_hot = true;
    //     usb_ep_start_out(OUTPUT_EP, ring_items[ring.output_index].buffer, MS_BUFFER_LENGTH);
    // }
    //
    // // if (ring.input_stalled)
    // if (ring.input_active && !ring.input_hot && ring.item_ready[ring.input_index]) {
    //     ring.input_hot = true;
    //     usb_ep_start_in(INPUT_EP, ring_items[ring.input_index].buffer, MS_BUFFER_LENGTH, MS_BUFFER_ZLP);
    // }
}
void usb_cb_control_in_completion(void) {
}
void usb_cb_control_out_completion(void) {
}

/// Callback for a SET_CONFIGURATION request
bool usb_cb_set_configuration(uint8_t config) {
    // led_on = 1;
    return true;
}

void fds_tick(void) {
    // uint32_t summary = (usb_ep_ready(1) ? (1 << 1) : 0) | (usb_ep_ready(2) ? (1 << 2) : 0);
    //
    // if (summary & (1 << 1)) {
    //     // led_on = usb_endpoints[1].DeviceDescBank[0].PCKSIZE.bit.BYTE_COUNT == MS_BUFFER_LENGTH;
    //     if (ring.output_active && ring.output_hot) {
    //         ring.output_hot = false;
    //
    //         if (ring.input_active) {
    //             ring_items[ring.output_index].ready = true;
    //             ring.output_index = (ring.output_index + 1) % RING_SIZE;
    //         }
    //
    //         // if (ring.input_stalled && ring_items[ring.input_index].ready) {
    //         //     ring.input_stalled = false;
    //         //     usb_ep_start_in(INPUT_EP, ring_items[ring.input_index].buffer, MS_BUFFER_LENGTH, MS_BUFFER_ZLP);
    //         // }
    //     }
    //     if (ring.output_active && !ring.output_hot) {
    //         if (!ring_items[ring.output_index].ready) {
    //             ring.output_hot = true;
    //             usb_ep_start_out(OUTPUT_EP, ring_items[ring.output_index].buffer, MS_BUFFER_LENGTH);
    //         }
    //         else {
    //             // usb_set_stall_ep(OUTPUT_EP);
    //             ring.output_stalled = true;
    //         }
    //     }
    // }
    // if (summary & (1 << 2)) {
    //     // led_on = 1;
    //
    //     if (ring.input_active && ring.input_hot) {
    //         ring.input_hot = false;
    //
    //         if (ring.output_active) {
    //             ring_items[ring.input_index].ready = false;
    //             ring.input_index = (ring.input_index + 1) % RING_SIZE;
    //         }
    //         else {
    //             ring_items[ring.input_index].ready = true;
    //             // memset(ring_items[ring.input_index].buffer, 0, MS_BUFFER_LENGTH);
    //             // memset(ring_items[ring.input_index].buffer, 0xfff0, 96);
    //             // memset(ring_items[ring.input_index].buffer + 96, 0x000f, 96);
    //         }
    //
    //         // if (!ring.output_hot && ring.output_stalled && !ring_items[ring.output_index].ready) {
    //         //     ring.output_hot = true;
    //         //     ring.output_stalled = false;
    //         //     usb_ep_start_out(OUTPUT_EP, ring_items[ring.output_index].buffer, MS_BUFFER_LENGTH);
    //         // }
    //     }
    //     if (ring.input_active && !ring.input_hot) {
    //         if (ring_items[ring.input_index].ready) {
    //             // led_on = 1;
    //             usb_ep_start_in(INPUT_EP, ring_items[ring.input_index].buffer, MS_BUFFER_LENGTH, MS_BUFFER_ZLP);
    //         }
    //         else {
    //             // usb_set_stall_ep(INPUT_EP);
    //             ring.input_stalled = true;
    //         }
    //     }
    // }
}

void fds_output_init(void) {
    usb_enable_ep(OUTPUT_EP, USB_EP_TYPE_ISOCHRONOUS, USB_EP1_SIZE);

    ring.output_active = true;

    if (ring.input_active) {
        fds_usb_completion = fds_inout_completion;
    }
    else {
        fds_usb_completion = fds_out_completion;
    }

    ring.output_index = ring.input_index;

    if (!ring.item_ready[ring.output_index]) {
        ring.output_hot = true;
        usb_ep_start_out(OUTPUT_EP, ring_items[ring.output_index].buffer, MS_BUFFER_LENGTH);
    }
    else {
        // usb_set_stall_ep(OUTPUT_EP);
        ring.output_stalled = true;
    }
}

void fds_output_disable(void) {
    usb_disable_ep(OUTPUT_EP);

    ring.output_hot = false;
    ring.output_active = false;
    ring.output_stalled = false;

    if (ring.input_active) {
        fds_usb_completion = fds_in_completion;
    }
    else {
        fds_usb_completion = fds_noop_completion;
    }
}

void fds_input_init(void) {
    // led_on = 1;

    usb_enable_ep(INPUT_EP, USB_EP_TYPE_ISOCHRONOUS, USB_EP2_SIZE);

    ring.input_active = true;

    if (ring.output_active) {
        fds_usb_completion = fds_inout_completion;
    }
    else {
        fds_usb_completion = fds_in_completion;
    }

    // if (!ring.output_active) {
    //     ring_items[ring.input_index].ready = true;
    //     memset(ring_items[ring.input_index].buffer, 0, MS_BUFFER_LENGTH);
    // }
    //
    // if (ring_items[ring.input_index].ready) {
    //     usb_ep_start_in(INPUT_EP, ring_items[ring.input_index].buffer, MS_BUFFER_LENGTH, MS_BUFFER_ZLP);
    // }
    // else {
    //     // usb_set_stall_ep(INPUT_EP);
    //     ring.input_stalled = true;
    // }
}

void fds_input_disable(void) {
    led_on = 0;

    int i;

    usb_disable_ep(INPUT_EP);

    ring.input_available = 0;

    ring.input_hot = false;
    ring.input_active = false;
    ring.input_stalled = false;

    for (i = 0; i < RING_SIZE; i++) {
        ring.item_ready[i] = false;
    }

    if (ring.output_active) {
        fds_usb_completion = fds_out_completion;
    }
    else {
        fds_usb_completion = fds_noop_completion;
    }
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
            if (ring.output_active) {
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
            if (ring.input_active) {
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

// /// Callbck for a GET_DESCRIPTOR request
// uint16_t usb_cb_get_descriptor(uint8_t type, uint8_t index, const uint8_t** descriptor_ptr) {
//     return 0;
// }

inline static void pin_mux(uint8_t p, uint8_t mux) {
  if (p & 1) {
    PORT->Group[p / 32].PMUX[(p % 32) / 2].bit.PMUXO = mux;
  } else {
    PORT->Group[p / 32].PMUX[(p % 32) / 2].bit.PMUXE = mux;
  }

  PORT->Group[p / 32].PINCFG[p % 32].bit.PMUXEN = 1;
}

inline static void pin_gpio(uint8_t p) {
  PORT->Group[p / 32].PINCFG[p % 32].bit.PMUXEN = 0;
}

inline static void pin_out(uint8_t p) {
  pin_gpio(p);
  PORT->Group[p / 32].DIRSET.reg = (1<<(p % 32));
}

inline static void pin_dir(uint8_t p, bool out) {
  if (out) {
    PORT->Group[p / 32].DIRSET.reg = (1<<(p % 32));
  } else {
    PORT->Group[p / 32].DIRCLR.reg = (1<<(p % 32));
  }
}

inline static void pin_high(uint8_t p) {
  PORT->Group[p / 32].OUTSET.reg = (1<<(p % 32));
}

inline static void pin_low(uint8_t p) {
  PORT->Group[p / 32].OUTCLR.reg = (1<<(p % 32));
}

inline static void pin_toggle(uint8_t p) {
  PORT->Group[p / 32].OUTTGL.reg = (1<<(p % 32));
}

inline static void pin_set(uint8_t p, bool high) {
  if (high) {
    PORT->Group[p / 32].OUTSET.reg = (1<<(p % 32));
  } else {
    PORT->Group[p / 32].OUTCLR.reg = (1<<(p % 32));
  }
}

inline static void pin_in(uint8_t p) {
  pin_gpio(p);
  PORT->Group[p / 32].PINCFG[(p % 32)].bit.INEN = 1;
  PORT->Group[p / 32].DIRCLR.reg = (1<<(p % 32));
}

inline static void pin_pull_up(uint8_t p) {
  pin_in(p);
  PORT->Group[p / 32].PINCFG[(p % 32)].bit.PULLEN = 1;
  pin_high(p);
}

inline static void pin_pull_down(uint8_t p) {
  pin_in(p);
  PORT->Group[p / 32].PINCFG[(p % 32)].bit.PULLEN = 1;
  pin_low(p);
}

inline static void pin_float(uint8_t p) {
  pin_in(p);
  PORT->Group[p / 32].PINCFG[(p % 32)].bit.PULLEN = 0;
}

inline static bool pin_read(uint8_t p) {
  return (PORT->Group[p / 32].IN.reg & (1<<(p % 32))) != 0;
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

void init_systick() {
	if (SysTick_Config(48000000 / 1000)) {	/* Setup SysTick Timer for 1 msec interrupts  */
		while (1) {}								/* Capture error */
	}
	NVIC_SetPriority(SysTick_IRQn, 0x0);
}

#define GCLK_SYSTEM 0
#define GCLK_32K    2

#define PIN_USB_DM 24
#define PIN_USB_DP 25
#define LED_PIN 17

int main() {
    clock_init_crystal(GCLK_SYSTEM, GCLK_32K);
    // init_systick();

    init();

    fds_usb_completion = fds_noop_completion;

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

    __DMB();
    __enable_irq();

    // NVIC_SetPriority(USB_IRQn, 0xff);

    // led_on = !led_on;
    // pin_set(LED_PIN, false);

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


// /* SysTick IRQ handler */
// void SysTick_Handler(void) {
//     if (ring.output_active) {
//         if (ring.input_active) {
//             usb_ep_start_in(INPUT_EP, ring_items[ring.output_index].buffer, MS_BUFFER_LENGTH, MS_BUFFER_ZLP);
//             ring.output_index = !ring.output_index;
//             usb_ep_start_out(OUTPUT_EP, ring_items[ring.output_index].buffer, MS_BUFFER_LENGTH);
//         }
//         else {
//             usb_ep_start_out(OUTPUT_EP, ring_items[ring.output_index].buffer, MS_BUFFER_LENGTH);
//         }
//     }
//     else if (ring.input_active) {
//         usb_ep_start_in(INPUT_EP, ring_items[ring.output_index].buffer, MS_BUFFER_LENGTH, MS_BUFFER_ZLP);
//     }
// }

void TCC0_Handler(void) {
    // led_on = !led_on;
    pin_set(LED_PIN, led_on % 1000 == 1);

    // Tcc *t0 = TCC0;
    TCC0->INTFLAG.reg = TCC_INTFLAG_OVF;

    // led_on = ring.input_index == 0;
    // led_on = 1;

    // if (ring.output_active) {
    //     if (ring.input_active) {
    //         usb_ep_start_in(INPUT_EP, ring_items[ring.output_index].buffer, MS_BUFFER_LENGTH, MS_BUFFER_ZLP);
    //         ring.output_index = !ring.output_index;
    //         usb_ep_start_out(OUTPUT_EP, ring_items[ring.output_index].buffer, MS_BUFFER_LENGTH);
    //     }
    //     else {
    //         usb_ep_start_out(OUTPUT_EP, ring_items[ring.output_index].buffer, MS_BUFFER_LENGTH);
    //     }
    // }
    // else if (ring.input_active) {
    //     usb_ep_start_in(INPUT_EP, ring_items[ring.output_index].buffer, MS_BUFFER_LENGTH, MS_BUFFER_ZLP);
    // }

    // if (ring.output_active) {
    //     ring.output_tick++;
    // }
    // if (ring.input_active) {
    //     ring.input_tick++;
    // }

    // bool out_complete = usb_endpoints[1].DeviceDescBank[0].PCKSIZE.bit.BYTE_COUNT == MS_BUFFER_LENGTH;
    // bool in_complete = // usb_endpoints[2].DeviceDescBank[1].PCKSIZE.bit.BYTE_COUNT == 0;
//     bool out_complete = USB->DEVICE.DeviceEndpoint[1].EPINTFLAG.reg == USB_DEVICE_EPINTFLAG_TRCPT0;
//     bool in_complete = USB->DEVICE.DeviceEndpoint[2].EPINTFLAG.reg == USB_DEVICE_EPINTFLAG_TRCPT1;
    // bool out_complete = 1;
    // bool in_complete = 1;
    //
    // led_on = in_complete;
    //
    // if (out_complete) {
    //     // led_on = 1;
    //     usb_ep_handled(OUTPUT_EP);
    //
    //     if (ring.output_active && ring.output_hot) {
    //         ring.output_hot = false;
    //
    //         if (ring.input_active) {
    //             ring_items[ring.output_index].ready = true;
    //             ring.output_index = (ring.output_index + 1) % RING_SIZE;
    //             if (ring.input_available < 2) {
    //                 ring.input_available += 1;
    //             }
    //         }
    //     }
    // }
    //
    // if (in_complete) {
    //     // led_on = 1;
    //     usb_ep_handled(INPUT_EP);
    //
    //     if (ring.input_active && ring.input_hot) {
    //         ring.input_hot = false;
    //
    //         if (ring.output_active) {
    //             ring_items[ring.input_index].ready = false;
    //             ring.input_index = (ring.input_index + 1) % RING_SIZE;
    //         }
    //     }
    // }
    //
    // if (ring.output_active && !ring.output_hot && !ring_items[ring.output_index].ready) {
    //     ring.output_hot = true;
    //     ring.output_stalled = false;
    //     usb_ep_start_out(OUTPUT_EP, ring_items[ring.output_index].buffer, MS_BUFFER_LENGTH);
    // }
    // else {
    //     // usb_set_stall_ep(OUTPUT_EP);
    //     ring.output_stalled = true;
    // }
    //
    // if (ring.input_active && !ring.input_hot && ring_items[ring.input_index].ready && ring.input_available > 1) {
    //     // led_on = 1;
    //     // ring.input_available -= 1;
    //     ring.input_hot = true;
    //     ring.input_stalled = false;
    //     usb_ep_start_in(INPUT_EP, ring_items[ring.input_index].buffer, MS_BUFFER_LENGTH, MS_BUFFER_ZLP);
    // }
    // else {
    //     // usb_set_stall_ep(INPUT_EP);
    //     ring.input_stalled = true;
    // }
}

void usb_cb_sof(void) {
    tick += 1;

    // bool out_complete = 1;
    // bool in_complete = 1;
    //
    // led_on = in_complete;
    //
    // if (out_complete) {
    //     // led_on = 1;
    //     usb_ep_handled(OUTPUT_EP);
    //
    //     if (ring.output_active && ring.output_hot) {
    //         ring.output_hot = false;
    //
    //         if (ring.input_active) {
    //             ring_items[ring.output_index].ready = true;
    //             ring.output_index = (ring.output_index + 1) % RING_SIZE;
    //             if (ring.input_available < 2) {
    //                 ring.input_available += 1;
    //             }
    //         }
    //     }
    // }
    //
    // if (in_complete) {
    //     // led_on = 1;
    //     usb_ep_handled(INPUT_EP);
    //
    //     if (ring.input_active && ring.input_hot) {
    //         ring.input_hot = false;
    //
    //         if (ring.output_active) {
    //             ring_items[ring.input_index].ready = false;
    //             ring.input_index = (ring.input_index + 1) % RING_SIZE;
    //         }
    //     }
    // }
    //
    // if (ring.output_active && !ring.output_hot && !ring_items[ring.output_index].ready) {
    //     ring.output_hot = true;
    //     ring.output_stalled = false;
    //     usb_ep_start_out(OUTPUT_EP, ring_items[ring.output_index].buffer, MS_BUFFER_LENGTH);
    // }
    // else {
    //     // usb_set_stall_ep(OUTPUT_EP);
    //     ring.output_stalled = true;
    // }
    //
    // if (ring.input_active && !ring.input_hot && ring_items[ring.input_index].ready && ring.input_available > 1) {
    //     // led_on = 1;
    //     // ring.input_available -= 1;
    //     ring.input_hot = true;
    //     ring.input_stalled = false;
    //     usb_ep_start_in(INPUT_EP, ring_items[ring.input_index].buffer, MS_BUFFER_LENGTH, MS_BUFFER_ZLP);
    // }
    // else {
    //     // usb_set_stall_ep(INPUT_EP);
    //     ring.input_stalled = true;
    // }

    // bool out_complete = USB->DEVICE.DeviceEndpoint[1].EPINTFLAG.reg == USB_DEVICE_EPINTFLAG_TRCPT0;
    // bool in_complete = USB->DEVICE.DeviceEndpoint[2].EPINTFLAG.reg == USB_DEVICE_EPINTFLAG_TRCPT1;
    //
    // led_on = in_complete;
    //
    // if (out_complete) {
    //     // led_on = 1;
    //     usb_ep_handled(OUTPUT_EP);
    //
    //     if (ring.output_active && ring.output_hot) {
    //         ring.output_hot = false;
    //
    //         if (ring.input_active) {
    //             ring_items[ring.output_index].ready = true;
    //             ring.output_index = (ring.output_index + 1) % RING_SIZE;
    //             if (ring.input_available < 2) {
    //                 ring.input_available += 1;
    //             }
    //         }
    //     }
    // }
    //
    // if (in_complete) {
    //     // led_on = 1;
    //     usb_ep_handled(INPUT_EP);
    //
    //     if (ring.input_active && ring.input_hot) {
    //         ring.input_hot = false;
    //
    //         if (ring.output_active) {
    //             ring_items[ring.input_index].ready = false;
    //             ring.input_index = (ring.input_index + 1) % RING_SIZE;
    //         }
    //     }
    // }

    // if (ring.output_active && !ring.output_hot && !ring_items[ring.output_index].ready) {
    //     ring.output_hot = true;
    //     ring.output_stalled = false;
    //     ring.output_tick = tick;
    //     usb_ep_start_out(OUTPUT_EP, ring_items[ring.output_index].buffer, MS_BUFFER_LENGTH);
    // }
    // else {
    //     // usb_set_stall_ep(OUTPUT_EP);
    //     ring.output_stalled = true;
    // }
    //
    // // led_on = usb_endpoints[1].DeviceDescBank[0].PCKSIZE.bit.BYTE_COUNT == MS_BUFFER_LENGTH;
    // // led_on = !ring_items[ring.input_index].ready;
    //
    // if (ring.input_active && !ring.input_hot && ring_items[ring.input_index].ready && ring.input_available > 3) {
    //     // ring.input_available -= 1;
    //     ring.input_hot = true;
    //     ring.input_stalled = false;
    //     ring.input_tick = tick;
    //     usb_ep_start_in(INPUT_EP, ring_items[ring.input_index].buffer, MS_BUFFER_LENGTH, MS_BUFFER_ZLP);
    // }
    // else {
    //     // usb_set_stall_ep(INPUT_EP);
    //     ring.input_stalled = true;
    // }

    // led_on += 1;

    // if (tick % 2 == 0)

    fds_usb_completion((ring.output_hot ? (1 << 1) : 0) | (ring.input_hot ? (1 << 2) : 0));
}
