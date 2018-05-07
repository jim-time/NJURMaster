#ifndef _MODE_DUTY_H_
#define _MODE_DUTY_H_
#include "stm32f4xx.h"
enum
{
	SYS_STOPSTATE=0,
	SYS_PREPARESTATE=1,
	SYS_NORMALSTATE=2,
	SYS_CALISTATE=3
};
enum
{
	MC_NORMAL=0,			//�����������̨�ȶ��������涯
	MC_MODE1,					//��̨�ܿ��ƣ����̲��ܿ��ƣ�������ת��
	MC_MODE2,					//����ǰ�����ҿɿأ���̨�ܿ�
	MC_MODE3					//����

};
#define SYS_PREPARETIME 5000
extern u8 SysMode,ControlMode;
void WorkStateFSM(u32 sys);
u8 GetWSCurrent(void);
#endif
