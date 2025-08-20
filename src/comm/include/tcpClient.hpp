#ifndef TCP_CLIENT_HPP_
#define TCP_CLIENT_HPP_

#include <stdio.h>  
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/types.h>  
#include <sys/socket.h>  
#include <netinet/in.h>  
#include <arpa/inet.h>  

typedef struct client_info {
    int client_sockfd; 
    int len;           
    struct sockaddr_in remote_addr; 
    unsigned char sendbuf[1200];   
    unsigned char recebuf[1200];   
} cli_info;

struct cli_dri {
    cli_info   *info; 
    struct client_ops *cli_ops; 
};

class Qttcp_cli
{
public:
    Qttcp_cli(){}
    ~Qttcp_cli(){}

    void cli_init(cli_info *client, char *ip, int port);

    int cli_socket_create(cli_info *client);

    int cli_connect(cli_info *client);

    int cli_send(cli_info *client);

    int cli_send_len(cli_info *client, int len);

    int cli_recv(cli_info *client);

    int cli_recv_back(cli_info *client);

    void cli_close(cli_info *client);

    int cli_reconnect(cli_info *client, char *ip, int port);

    cli_info *client; 
    bool reconnect;   

};

#endif
