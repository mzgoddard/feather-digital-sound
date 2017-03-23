#include <stdbool.h>

#include <instance/sercom0.h>

#include "audio_descriptors.h"

#define AUDIO_SAMPLE_FREQUENCY_BYTES AUDIO_SAMPLE_FREQUENCY & 0xff, \
                                    (AUDIO_SAMPLE_FREQUENCY >> 8) & 0xff, \
                                    (AUDIO_SAMPLE_FREQUENCY >> 16) & 0xff

extern bool led_on;

__attribute__((__aligned__(4))) const USB_DeviceDescriptor DeviceDescriptor =
{
	.bLength = sizeof(USB_DeviceDescriptor),
	.bDescriptorType = USB_DTYPE_Device,
	.bcdUSB = VERSION_BCD(01.10),
	.bDeviceClass = 0x00,
	.bDeviceSubClass = 0x00,
	.bDeviceProtocol = 0x00,
	.bMaxPacketSize0 = FIXED_CONTROL_ENDPOINT_SIZE,
    .idVendor = 0x03EB,
    .idProduct = 0x2047,
    // .idVendor = 0x0d8c,
    // .idProduct = 0x0102,
	.bcdDevice = VERSION_BCD(00.01),
	.iManufacturer = 0x01,
	.iProduct = 0x02,
	.iSerialNumber = NO_DESCRIPTOR,
	.bNumConfigurations = FIXED_NUM_CONFIGURATIONS,
};

__attribute__((__aligned__(4))) const USB_Configuration ConfigurationDescriptor =
{
	.Config =
		{
        	.bLength = sizeof(USB_ConfigurationDescriptor),
        	.bDescriptorType = USB_DTYPE_Configuration,
        	.wTotalLength = sizeof(USB_Configuration),
            .bNumInterfaces = 3,
        	.bConfigurationValue = 1,
        	.iConfiguration = NO_DESCRIPTOR,
        	.bmAttributes = (USB_CONFIG_ATTR_BUSPOWERED | USB_CONFIG_ATTR_SELFPOWERED),
        	.bMaxPower = USB_CONFIG_POWER_MA(100),
		},

	.Control_Interface =
		{
        	.bLength = sizeof(USB_InterfaceDescriptor),
        	.bDescriptorType = USB_DTYPE_Interface,
        	.bInterfaceNumber = 0,
        	.bAlternateSetting = 0,
        	.bNumEndpoints = 0,
        	.bInterfaceClass = 0x01,
        	.bInterfaceSubClass = 0x01,
        	.bInterfaceProtocol = 0x00,
        	.iInterface = NO_DESCRIPTOR,
		},

	.Control_Header =
		{
        	.bLength = sizeof(USB_AudioControl),
        	.bDescriptorType = USB_DTYPE_CSInterface,
        	.bDescriptorSubtype = AUDIO_DSUBTYPE_CSInterface_Header,
        	.bcdADC = VERSION_BCD(01.00),
        	.wTotalLength = (sizeof(USB_AudioControl) +
                             sizeof(USB_AudioInputTerminal) +
                             sizeof(USB_AudioFeatureUnit) +
                             sizeof(USB_AudioOutputTerminal) +
                             sizeof(USB_AudioInputTerminal) +
                             sizeof(USB_AudioOutputTerminal)),
            .bInCollection = 2,
            .bInterfaceNumbers = {1, 2},
		},

	.Output_InputTerminal =
		{
            .bLength = sizeof(USB_AudioInputTerminal),
            .bDescriptorType = USB_DTYPE_CSInterface,
            .bDescriptorSubtype = AUDIO_DSUBTYPE_CSInterface_InputTerminal,
            .bTerminalID = 0x01,
            .wTerminalType = AUDIO_TERMINAL_STREAMING,
            .bAssocTerminal = 0x00,
            .bNrChannels = 2,
            // .bNrChannels = 1,
            .wChannelConfig = AUDIO_CHANNEL_LEFT_FRONT | AUDIO_CHANNEL_RIGHT_FRONT,
            // .wChannelConfig = 0,
            .iChannelNames = NO_DESCRIPTOR,
            .iTerminal = NO_DESCRIPTOR,
		},

    .Output_Feature =
        {
        	.bLength = sizeof(USB_AudioFeatureUnit),
        	.bDescriptorType = USB_DTYPE_CSInterface,
        	.bDescriptorSubtype = AUDIO_DSUBTYPE_CSInterface_Feature,
        	.bUnitID = 0x02,
        	.bSourceID = 0x01,
        	.bControlSize = 1,
        	.bmaControls = {AUDIO_FEATURE_VOLUME},
        	.iFeature = 0,
        },

	.Output_OutputTerminal =
		{
        	.bLength = sizeof(USB_AudioOutputTerminal),
        	.bDescriptorType = USB_DTYPE_CSInterface,
        	.bDescriptorSubtype = AUDIO_DSUBTYPE_CSInterface_OutputTerminal,
        	.bTerminalID = 0x05,
        	.wTerminalType = AUDIO_TERMINAL_OUT_HEADPHONES,
        	.bAssocTerminal = 0x00,
        	.bSourceID = 0x02,
        	.iTerminal = NO_DESCRIPTOR,
		},

    .Input_InputTerminal =
        {
                .bLength = sizeof(USB_AudioInputTerminal),
                .bDescriptorType = USB_DTYPE_CSInterface,
                .bDescriptorSubtype = AUDIO_DSUBTYPE_CSInterface_InputTerminal,
                .bTerminalID = 0x03,
                .wTerminalType = AUDIO_TERMINAL_IN_PERSONAL_MIC,
                .bAssocTerminal = 0x00,
                .bNrChannels = 2,
                // .bNrChannels = 1,
                .wChannelConfig = AUDIO_CHANNEL_LEFT_FRONT | AUDIO_CHANNEL_RIGHT_FRONT,
                // .wChannelConfig = 0,
                .iChannelNames = NO_DESCRIPTOR,
                .iTerminal = NO_DESCRIPTOR,
        },

    .Input_OutputTerminal =
        {
                .bLength = sizeof(USB_AudioOutputTerminal),
                .bDescriptorType = USB_DTYPE_CSInterface,
                .bDescriptorSubtype = AUDIO_DSUBTYPE_CSInterface_OutputTerminal,
                .bTerminalID = 0x04,
                .wTerminalType = AUDIO_TERMINAL_STREAMING,
                .bAssocTerminal = 0x00,
                .bSourceID = 0x03,
                .iTerminal = NO_DESCRIPTOR,
        },

	.Output_Interface_Alt0 =
		{
        	.bLength = sizeof(USB_InterfaceDescriptor),
        	.bDescriptorType = USB_DTYPE_Interface,
        	.bInterfaceNumber = 1,
        	.bAlternateSetting = 0,
        	.bNumEndpoints = 0,
        	.bInterfaceClass = 0x01,
        	.bInterfaceSubClass = 0x02,
        	.bInterfaceProtocol = 0x00,
        	.iInterface = NO_DESCRIPTOR,
        },

	.Output_Interface_Alt1 =
		{
        	.bLength = sizeof(USB_InterfaceDescriptor),
        	.bDescriptorType = USB_DTYPE_Interface,
        	.bInterfaceNumber = 1,
        	.bAlternateSetting = 1,
        	.bNumEndpoints = 1,
        	.bInterfaceClass = 0x01,
        	.bInterfaceSubClass = 0x02,
        	.bInterfaceProtocol = 0x00,
        	.iInterface = NO_DESCRIPTOR,
		},

	.Output_Stream =
		{
        	.bLength = sizeof(USB_AudioStream),
        	.bDescriptorType = USB_DTYPE_CSInterface,
        	.bDescriptorSubtype = AUDIO_DSUBTYPE_CSInterface_General,
        	.bTerminalLink = 0x01,
        	.bDelay = 1,
        	.wFormatTag = 0x0001,
		},

	.Output_Format =
		{
        	.bLength = sizeof(USB_AudioFormat),
        	.bDescriptorType = USB_DTYPE_CSInterface,
        	.bDescriptorSubtype = AUDIO_DSUBTYPE_CSInterface_FormatType,
        	.bFormatType = 0x01,
            .bNrChannels = 0x02,
            // .bNrChannels = 0x01,
            .bSubFrameSize = 0x02,
            // .bSubFrameSize = 0x01,
            .bBitResolution = 16,
            // .bBitResolution = 8,
        	.bSampleFrequencyType = (sizeof(ConfigurationDescriptor.Output_Format.SampleFrequencies) /
			                             sizeof(USB_AudioSampleFreq)),
        	.SampleFrequencies = {AUDIO_SAMPLE_FREQUENCY_BYTES},
		},

	.Output_Endpoint =
		{
        	.bLength = sizeof(USB_AudioEndpoint),
        	.bDescriptorType = USB_DTYPE_Endpoint,
        	.bEndpointAddress = (ENDPOINT_DESCRIPTOR_DIR_OUT | AUDIO_STREAM_OUTPUT_EPNUM),
        	.bmAttributes = (USB_EP_TYPE_ISOCHRONOUS | ENDPOINT_ATTR_SYNC | ENDPOINT_USAGE_DATA),
        	.wMaxPacketSize = AUDIO_STREAM_EPSIZE,
        	.bInterval = 1,
            .bRefresh = 0,
            .bSynchAddress = 0,
		},

	.Output_StreamEndpoint =
		{
            .bLength = sizeof(USB_AudioStreamEndpoint),
            .bDescriptorType = USB_DTYPE_CSEndpoint,
            .bDescriptorSubtype = AUDIO_DSUBTYPE_CSEndpoint_General,
            .bmAttributes = AUDIO_EP_ACCEPTS_SMALL_PACKETS,
            .bLockDelayUnits = 0x00,
            .wLockDelay = 0x0000,
		},

    .Input_Interface_Alt0 =
        {
                .bLength = sizeof(USB_InterfaceDescriptor),
                .bDescriptorType = USB_DTYPE_Interface,
                .bInterfaceNumber = 2,
                .bAlternateSetting = 0,
                .bNumEndpoints = 0,
                .bInterfaceClass = 0x01,
                .bInterfaceSubClass = 0x02,
                .bInterfaceProtocol = 0x00,
                .iInterface = NO_DESCRIPTOR,
            },

    .Input_Interface_Alt1 =
        {
                .bLength = sizeof(USB_InterfaceDescriptor),
                .bDescriptorType = USB_DTYPE_Interface,
                .bInterfaceNumber = 2,
                .bAlternateSetting = 1,
                .bNumEndpoints = 1,
                .bInterfaceClass = 0x01,
                .bInterfaceSubClass = 0x02,
                .bInterfaceProtocol = 0x00,
                .iInterface = NO_DESCRIPTOR,
        },

    .Input_Stream =
        {
                .bLength = sizeof(USB_AudioStream),
                .bDescriptorType = USB_DTYPE_CSInterface,
                .bDescriptorSubtype = AUDIO_DSUBTYPE_CSInterface_General,
                .bTerminalLink = 0x04,
                .bDelay = 1,
                .wFormatTag = 0x0001,
        },

    .Input_Format =
        {
                .bLength = sizeof(USB_AudioFormat),
                .bDescriptorType = USB_DTYPE_CSInterface,
                .bDescriptorSubtype = AUDIO_DSUBTYPE_CSInterface_FormatType,
                .bFormatType = 0x01,
                .bNrChannels = 0x02,
                // .bNrChannels = 0x01,
                .bSubFrameSize = 0x02,
                // .bSubFrameSize = 0x01,
                .bBitResolution = 16,
                // .bBitResolution = 8,
                .bSampleFrequencyType = (sizeof(ConfigurationDescriptor.Output_Format.SampleFrequencies) /
                                         sizeof(USB_AudioSampleFreq)),
                .SampleFrequencies = {AUDIO_SAMPLE_FREQUENCY_BYTES},
        },

    .Input_Endpoint =
        {
                .bLength = sizeof(USB_AudioEndpoint),
                .bDescriptorType = USB_DTYPE_Endpoint,
                .bEndpointAddress = (ENDPOINT_DESCRIPTOR_DIR_IN | AUDIO_STREAM_INPUT_EPNUM),
                .bmAttributes = (USB_EP_TYPE_ISOCHRONOUS | ENDPOINT_ATTR_SYNC | ENDPOINT_USAGE_DATA),
                .wMaxPacketSize = AUDIO_STREAM_EPSIZE,
                .bInterval = 1,
                .bRefresh = 0,
                .bSynchAddress = 0,
        },

    .Input_StreamEndpoint =
        {
                .bLength = sizeof(USB_AudioStreamEndpoint),
                .bDescriptorType = USB_DTYPE_CSEndpoint,
                .bDescriptorSubtype = AUDIO_DSUBTYPE_CSEndpoint_General,
                .bmAttributes = AUDIO_EP_ACCEPTS_SMALL_PACKETS,
                .bLockDelayUnits = 0x00,
                .wLockDelay = 0x0000,
        }
};

/** Language descriptor structure. This descriptor, located in FLASH memory, is returned when the host requests
 *  the string descriptor with index 0 (the first index). It is actually an array of 16-bit integers, which indicate
 *  via the language ID table available at USB.org what languages the device supports for its string descriptors.
 */
__attribute__((__aligned__(4))) const USB_StringDescriptor LanguageString =
{
	.bLength = USB_STRING_LEN(1),
	.bDescriptorType = USB_DTYPE_String,
	.bString = LANGUAGE_ID_ENG,
};

/** Manufacturer descriptor string. This is a Unicode string containing the manufacturer's details in human readable
 *  form, and is read out upon request by the host when the appropriate string ID is requested, listed in the Device
 *  Descriptor.
 */
__attribute__((__aligned__(4))) const USB_StringDescriptor ManufacturerString =
{
	.bLength = USB_STRING_LEN(15),
	.bDescriptorType = USB_DTYPE_String,
	.bString = u"Michael Goddard",
};

/** Product descriptor string. This is a Unicode string containing the product's details in human readable form,
 *  and is read out upon request by the host when the appropriate string ID is requested, listed in the Device
 *  Descriptor.
 */
__attribute__((__aligned__(4))) const USB_StringDescriptor ProductString =
{
	.bLength = USB_STRING_LEN(15),
	.bDescriptorType = USB_DTYPE_String,
	.bString = u"Sound Loop Card",
};

/** This function is called by the library when in device mode, and must be overridden (see library "USB Descriptors"
 *  documentation) by the application code so that the address and size of a requested descriptor can be given
 *  to the USB library. When the device receives a Get Descriptor request on the control endpoint, this function
 *  is called so that the descriptor details can be passed back and the appropriate descriptor sent back to the
 *  USB host.
 */
uint16_t usb_cb_get_descriptor(uint8_t type, uint8_t index, const uint8_t** descriptor_ptr)
{
	const void* Address = NULL;
	uint16_t    Size    = NO_DESCRIPTOR;

    // led_on = (type & 0x02) > 0;

	switch (type)
	{
		case USB_DTYPE_Device:
			Address = &DeviceDescriptor;
			Size    = sizeof(USB_DeviceDescriptor);
			break;
		case USB_DTYPE_Configuration:
            // led_on = 1;
			Address = &ConfigurationDescriptor;
			Size    = sizeof(USB_Configuration);
			break;
		case USB_DTYPE_String:
			switch (index)
			{
				case 0x00:
					Address = &LanguageString;
					Size    = LanguageString.bLength;
					break;
				case 0x01:
					Address = &ManufacturerString;
					Size    = ManufacturerString.bLength;
					break;
				case 0x02:
					Address = &ProductString;
					Size    = ProductString.bLength;
					break;
			}

			break;
	}

	*descriptor_ptr = Address;
	return Size;
}
