#ifndef TCP_SERVER_HPP_
#define TCP_SERVER_HPP_

#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h> 

#define SD_RECEIVE 0x00     
#define SD_SEND 0x01        
#define SD_BOTH 0x02        


typedef struct server_info {
    int server_sockfd;       
    int client_sockfd;      
    int len;                 
    struct sockaddr_in my_addr;     
    struct sockaddr_in remote_addr; 
    int sin_size;            
#define BUFSIZE 1200        
    unsigned char sendbuf[BUFSIZE]; 
    unsigned char recebuf[BUFSIZE]; 
} ser_info;

struct ser_dri {
    struct proc_img *comm_img; 
    ser_info *server;          
};

class Qttcp_ser
{

public:
    Qttcp_ser(){}
    ~Qttcp_ser(){}

    void ser_init(ser_info *server, int port);

    int ser_socket_create(ser_info *server);

    int ser_bind(ser_info *server);

    int ser_listen(ser_info *server);

    int ser_accept(ser_info *server);

    int ser_recv(ser_info *server);

    int ser_recv_nowait(ser_info *server);

    int ser_send(ser_info *server, int len);

    void ser_close(ser_info *server);

    int ser_reconnect(ser_info *server, int port);

    struct proc_img *comm_img; 
    ser_info *server;          
    bool reconnect;          
};


#endif