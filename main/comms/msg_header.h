
#ifndef ANTICOPTER_MSG_HEADER_H
#define ANTICOPTER_MSG_HEADER_H

// Comms message header, 20 bytes long
typedef struct 
{
    uint16_t magic_number;   // Magic number
    uint8_t  version;        // Protocol version number
    uint16_t msg_type;       // Message type; see msg_type_t  
    uint8_t  flags;          // Message flags, see msg_flags_t
    uint32_t seq;            // Message sequence number (incremented separately for each sender)
    uint32_t frag_seq;       // If message is fragmented, messages that get split retain the same sequence number, 
                             // and the fragmentation sequence number is incremented instead
    uint32_t timestamp;      // Microseconds since communication was established
    uint16_t payload_len;    // Length of the packet payload in bytes
} __attribute__((packed)) msg_header_t;

typedef enum
{
    MSG_FLAG_FRAGMENTED      = 1 << 0, // If set, the contained data is too large to fit within one packet and has been split
                                       // across multiple packets, which means that frag_seq has to be considered.
                                       // This can typically only happen for image and log data
    MSG_FLAG_FRAGMENTED_LAST = 1 << 1, // If set, this is the last packet of a fragmented message
    MSG_FLAG_WRITE           = 1 << 2, // If set, the sender intends to write data to the receiver
                                       // If zero, the receiver can safely ignore any attached payload, 
                                       // but is typically expected to return data as indicated by msg_type
} msg_flags_t;

#endif
