#include "tcpServer.hpp"
#include <string.h>
#include <stdio.h>  
#include <arpa/inet.h>  
#include <stdlib.h>
#include <unistd.h>

void Qttcp_ser::ser_init(ser_info *server, int port) {
    memset(&server->my_addr, 0, sizeof(server->my_addr)); 
    server->my_addr.sin_family = AF_INET;      
    server->my_addr.sin_addr.s_addr = INADDR_ANY; 
    server->my_addr.sin_port = htons(port);    
}

int Qttcp_ser::ser_socket_create(ser_info *server) {
    int ret = 0;
 
    ret = (server->server_sockfd = socket(PF_INET, SOCK_STREAM, 0));
    if (ret < 0) {
        perror("socket create failed!\n"); 
        return -1;
    }

    int on = 1;
    setsockopt(server->server_sockfd, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));

    return ret;
}

int Qttcp_ser::ser_bind(ser_info *server) {
    int ret = 0;

    ret = bind(server->server_sockfd, (struct sockaddr *)&server->my_addr, sizeof(struct sockaddr));
    if (ret < 0) {
        perror("bind failed!\n"); 
        return -1;
    }

    return ret;
}

int Qttcp_ser::ser_listen(ser_info *server) {
    int ret = 0;

    ret = listen(server->server_sockfd, 5);
    if (ret < 0) {
        perror("listen failed!\n"); 
        return -1;
    }
    server->sin_size = sizeof(struct sockaddr_in); 

    return ret;
}

int Qttcp_ser::ser_accept(ser_info *server) {
    int ret = 0;
   
    ret = (server->client_sockfd = accept(server->server_sockfd, (struct sockaddr *)&server->remote_addr, (socklen_t *)&server->sin_size));
    if (ret < 0) {
        perror("accept failed!\n"); 
        return -1;
    }

    printf("accept client %s\n", inet_ntoa(server->remote_addr.sin_addr));

    return ret;
}

int Qttcp_ser::ser_recv(ser_info *server) {
    int ret = 0;

    ret = recv(server->client_sockfd, server->recebuf, BUFSIZE, MSG_NOSIGNAL);
    if (ret < 0) {
        return -1;
    }
    server->len = ret; 

    return server->len;
}

int Qttcp_ser::ser_recv_nowait(ser_info *server) {
    int ret = 0;

    ret = recv(server->client_sockfd, server->recebuf, BUFSIZE, MSG_DONTWAIT);
    if (ret < 0) {
        return ret;
    }
    server->len = ret; 

    return server->len;
}

int Qttcp_ser::ser_send(ser_info *server, int len) {
    int ret = 0;

    ret = send(server->client_sockfd, server->sendbuf, len, MSG_NOSIGNAL);
    if (ret < 0) {
        return -1;
    }
    return ret;
}

void Qttcp_ser::ser_close(ser_info *server) {
    if (server->client_sockfd != 0)
    {
        close(server->client_sockfd); 
    }
    
    if (server->server_sockfd != 0)
    {
        close(server->server_sockfd); 
    }
    
}

int Qttcp_ser::ser_reconnect(ser_info *server, int port) {
    int ret = 0;

    memset(&server->my_addr, 0, sizeof(server->my_addr));
    server->my_addr.sin_family = AF_INET;
    server->my_addr.sin_addr.s_addr = INADDR_ANY;
    server->my_addr.sin_port = htons(port);

    ret = (server->server_sockfd = socket(PF_INET, SOCK_STREAM, 0));
    if (ret < 0) {
        perror("socket create failed!\n");
        return -2;
    }

    int on = 1;
    setsockopt(server->server_sockfd, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));

    ret = bind(server->server_sockfd, (struct sockaddr *)&server->my_addr, sizeof(struct sockaddr));
    if (ret < 0) {
        perror("bind failed!\n");
        return -2;
    }

    ret = listen(server->server_sockfd, 5);
    if (ret < 0) {
        perror("listen failed!\n");
        return -2;
    }
    server->sin_size = sizeof(struct sockaddr_in);

    ret = (server->client_sockfd = accept(server->server_sockfd, (struct sockaddr *)&server->remote_addr, (socklen_t *)&server->sin_size));
    if (ret < 0) {
        perror("accept failed!\n");
        return -1;
    }
    printf("accept client %s\n", inet_ntoa(server->remote_addr.sin_addr));

    return ret;
}