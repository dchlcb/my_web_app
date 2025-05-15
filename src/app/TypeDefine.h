#ifndef TYPE_DEFINE_H
#define TYPE_DEFINE_H

#define MSGBUF_MAX 10*1024 //消息队列缓冲区大小

typedef struct
{
	int can_sfd;
	int slave_id;
}PTHREAD_DATA;

//动作枚举
typedef enum
{
	en_MOVE = 1,
	en_STOP = 2,
	en_PAUSE = 3
}AcitonType;

//轴枚举
typedef enum
{
	en_X = 1,
	en_Y = 2,
	en_Z = 3,
	en_O = 4
}AxisType;

#pragma pack(push, 1)
// 消息队列结构
typedef struct 
{
    long msgtype;
    char msgdata[MSGBUF_MAX];
}msgType;
#pragma pack(pop)



#endif