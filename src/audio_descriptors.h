#include <class/audio/audio_standard.h>

#define FDS_OUTPUT_ITERMINAL_ID 1
#define FDS_OUTPUT_FEATURE_ID 2
#define FDS_OUTPUT_OTERMINAL_ID 5
#define FDS_INPUT_ITERMINAL_ID 3
#define FDS_INPUT_OTERMINAL_ID 4

#define FIXED_CONTROL_ENDPOINT_SIZE 64
#define FIXED_NUM_CONFIGURATIONS 1
#define AUDIO_SAMPLE_FREQUENCY 48000
#define AUDIO_STREAM_OUTPUT_EPNUM 1
#define AUDIO_STREAM_INPUT_EPNUM 2
#define AUDIO_STREAM_EPSIZE 256

typedef struct {
    USB_ConfigurationDescriptor Config;
    USB_InterfaceDescriptor Control_Interface;
    USB_AudioControl Control_Header;
    USB_AudioInputTerminal Output_InputTerminal;
    USB_AudioFeatureUnit Output_Feature;
    USB_AudioOutputTerminal Output_OutputTerminal;
    USB_AudioInputTerminal Input_InputTerminal;
    USB_AudioOutputTerminal Input_OutputTerminal;
    USB_InterfaceDescriptor Output_Interface_Alt0;
    USB_InterfaceDescriptor Output_Interface_Alt1;
    USB_AudioStream Output_Stream;
    USB_AudioFormat Output_Format;
    USB_AudioEndpoint Output_Endpoint;
    USB_AudioStreamEndpoint Output_StreamEndpoint;
    USB_InterfaceDescriptor Input_Interface_Alt0;
    USB_InterfaceDescriptor Input_Interface_Alt1;
    USB_AudioStream Input_Stream;
    USB_AudioFormat Input_Format;
    USB_AudioEndpoint Input_Endpoint;
    USB_AudioStreamEndpoint Input_StreamEndpoint;
} __attribute__((packed)) USB_Configuration;
