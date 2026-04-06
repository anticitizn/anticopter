
#ifndef ANTICOPTER_COMMS_CONFIG
#define ANTICOPTER_COMMS_CONFIG

typedef struct
{
    uint32_t mtu;
    uint16_t magic_number;
    uint16_t port;

} comms_config_t;

#endif