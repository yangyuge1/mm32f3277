#ifndef __IMAGE_H
#define __IMAGE_H


#define w_max 180
#define w_min 40

//������������
typedef unsigned char		uint8;													//  8 bits 
typedef unsigned short int	uint16;													// 16 bits 
typedef unsigned long int	uint32;													// 32 bits 
typedef unsigned long long	uint64;													// 64 bits 

typedef char				int8;													//  8 bits 
typedef short int			int16;													// 16 bits 
typedef long  int			int32;													// 32 bits 
typedef long  long			int64;													// 64 bits 

typedef volatile int8		vint8;														//  8 bits 
typedef volatile int16		vint16;													// 16 bits 
typedef volatile int32		vint32;													// 32 bits 
typedef volatile int64		vint64;													// 64 bits 
typedef volatile uint8		vuint8;													//  8 bits 
typedef volatile uint16		vuint16;												// 16 bits 
typedef volatile uint32		vuint32;												// 32 bits 
typedef volatile uint64		vuint64;												// 64 bits 

void erode(void);  //���帯ʴ����
void dilate(void);  //�������ͺ���
uint8 min(uint8 x,uint8 y);  //�Ƚ϶�����Сֵ�ĺ���
uint8 max(uint8 x,uint8 y);  //�Ƚ϶������ֵ�ĺ���
int Find(int x, int parent[]);  //�����Ҹ��ڵ㣬����һ���Ҷ��
void Union(int x, int y, int parent[]);  
void two_pass(void);
void findshape();
void findLabel(void);
void findotherled(void);
void showLabel(void);
void findcenter_2(void);
void findcenter_4(void);
int findwide(void);
void fuction(void);

#endif
