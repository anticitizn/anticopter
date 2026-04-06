
#ifndef ANTICOPTER_MSG_SEND_H
#define ANTICOPTER_MSG_SEND_H

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#include "msg_header.h"
#include "msg_type.h"

extern struct sockaddr_storage source_addr;
extern int sock;

void send_message(const msg_header_t header, const void *payload)
{
    struct iovec iov[2];
    iov[0].iov_base = &header;
    iov[0].iov_len  = sizeof(header);
    iov[1].iov_base = payload;
    iov[1].iov_len  = header.payload_len;

    struct msghdr msg = {0};
    msg.msg_name = &source_addr;
    msg.msg_namelen = sizeof(source_addr);
    msg.msg_iov = iov;
    msg.msg_iovlen = 2;

    sendmsg(sock, &msg, 0);
}

#endif
