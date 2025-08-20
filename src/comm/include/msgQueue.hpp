#ifndef MSG_QUEUE_HPP_
#define MSG_QUEUE_HPP_
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/msg.h>
#include <errno.h>

#define BUF_SIZE 4096  
#define MAX_TEXT 4096


struct main_msg {
    long type;   
    int x;      
    int y;      
    int yaw;   
    

    int v;       
    int theta;   

    float laserscan[380];
    
    float intensity[380];
    unsigned char ultrasound[8];
    int time;  
};

struct _msg {
    main_msg data; 

    struct msqid_ds msq_buf; 
    int msgid; 
};

class Msgq
{
public:
    Msgq()
    {}
    ~Msgq(){}
    int msg_create(struct _msg *msg,int num);

    int msg_recv(struct _msg *msg_from,struct _msg *msg_to);
    int msg_recv(struct _msg *msg);
    int msg_send(struct _msg *msg);
    int msg_del(struct _msg *msg);
    int msg_stat(struct _msg *msg);

    int msg_recv_nowait(struct _msg *msg_from,struct _msg *msg_to);
    int msg_recv_nowait(struct _msg *msg);

};

#endif 
