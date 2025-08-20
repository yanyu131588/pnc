#include "tcpClient.hpp"

void Qttcp_cli::cli_init(cli_info *client,char *ip,int port)
{
    memset(&client->remote_addr, 0, sizeof(client->remote_addr)); 
    client->remote_addr.sin_family = AF_INET; 
    client->remote_addr.sin_addr.s_addr = inet_addr(ip); 
    client->remote_addr.sin_port = htons(port); 
}

int Qttcp_cli::cli_socket_create(cli_info *client)
{
    if ((client->client_sockfd = socket(PF_INET, SOCK_STREAM, 0)) < 0) {
        perror("socket create error!\n"); 
        return -3; 
    }
    return 0; 
}

int Qttcp_cli::cli_connect(cli_info *client) {
    int ret = 0;

    ret = connect(client->client_sockfd, (struct sockaddr *)&client->remote_addr, sizeof(struct sockaddr));
    if (ret < 0) {
        printf("connect failed , error code = %d!\n", ret); 
        return -1; 
    }
    return ret; 
}

int Qttcp_cli::cli_send(cli_info *client) {
    int ret = 0;
  
    ret = send(client->client_sockfd, client->sendbuf, strlen((char *)client->sendbuf), 1); 
    if (ret < 0) {
        printf("send failed ret = %d\n", ret); 
        return ret;
    }

    client->len = ret; 
    return 0;
}

int Qttcp_cli::cli_send_len(cli_info *client, int len) {
    int ret = 0;
   
    ret = send(client->client_sockfd, client->sendbuf, len, MSG_NOSIGNAL); 
    if (ret < 0) {
        printf("send failed ret = %d\n", ret); 
        return ret;
    }

    client->len = ret; 
    return ret;
}

int Qttcp_cli::cli_recv(cli_info *client) {
    int ret = 0;
  
    ret = recv(client->client_sockfd, client->recebuf, 1200, MSG_DONTWAIT); 
    if (ret < 0) {
        return ret; 
    }
    client->len = ret; 
    
    return ret;
}

int Qttcp_cli::cli_recv_back(cli_info *client)
{   
    int ret = 0;

    while (1) {
        printf("waiting for ack!\n"); 
        ret = recv(client->client_sockfd, client->recebuf, 1200, 0); 
        printf("receive complete!, ret = %d\n", ret);

        if (ret < 0) {
            perror("while receive failed !\n"); 
            return ret;
        }

        if (client->len == ret) {
            client->recebuf[client->len] = '\0'; 
            printf("received:%s\n", client->recebuf); 
            break;
        } else {
            perror("len not equal to receive !\n"); 
            return ret;
        }
    }

    return 0;
}

void Qttcp_cli::cli_close(cli_info *client)
{
 close(client->client_sockfd); 
    free(client); 
}

int Qttcp_cli::cli_reconnect(cli_info *client,char *ip,int port)
{
    int ret=0;

    cli_init(client, ip, port);

    ret = cli_socket_create(client);
    if (ret < 0) {
        printf("create socket failed!\n");
        return ret;
    }

    ret = cli_connect(client);
    if (ret < 0) {
        printf("[%s,%d]: try again!\n", __FUNCTION__, __LINE__);
        return ret;
    }

   return ret;
}
