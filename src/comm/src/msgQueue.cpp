#include "msgQueue.hpp"
#include <iostream>


int Msgq::msg_create(struct _msg *msg,int num)
{
    int ret=0;
    memset((void*)&msg->data,0,sizeof(msg->data));
    msg->msgid=-1;

    msg->msgid=msgget((key_t)num,0666|IPC_CREAT);;
    if(msg->msgid == -1)
    {
        fprintf(stderr,"failed get queueid%d\n",errno);
        return -1;
    }
    //
    struct msqid_ds buf; 
    msgctl(msg->msgid, IPC_STAT, &buf);  
    buf.msg_qbytes = MAX_TEXT;     
    msgctl(msg->msgid, IPC_SET, &buf); 
    return 0;
}

int Msgq::msg_recv(struct _msg *msg_from,struct _msg *msg_to)
{
    int ret = 0;
    ret = msgrcv(msg_from->msgid,(void*)&msg_to->data,BUF_SIZE,0,0);

    return ret;
}

int Msgq::msg_recv(struct _msg *msg)
{
    int ret = 0;
    ret = msgrcv(msg->msgid,(void*)&msg->data,BUF_SIZE,0,0);

    return ret;
}

int Msgq::msg_recv_nowait(struct _msg *msg_from,struct _msg *msg_to)
{
    int ret = 0;
    ret = msgrcv(msg_from->msgid,(void*)&msg_to->data,BUF_SIZE,0,IPC_NOWAIT);

    return ret;
}

int Msgq::msg_recv_nowait(struct _msg *msg)
{
    int ret = 0;
    ret = msgrcv(msg->msgid,(void*)&msg->data,BUF_SIZE,0,IPC_NOWAIT);

    return ret;
}

int Msgq::msg_send(struct _msg *msg)
{
    int ret = 0;
    ret = msgsnd(msg->msgid,(void*)&msg->data,sizeof(msg->data),0);

    return ret;
}

int Msgq::msg_del(struct _msg *msg)
{
    int ret = 0;
    ret = msgctl(msg->msgid,IPC_RMID,0);

    return ret;
}

int Msgq::msg_stat(struct _msg *msg)
{
    int ret = 0;
    msgctl(msg->msgid,IPC_STAT,&msg->msq_buf);

    return ret;
} 
