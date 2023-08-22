#include "image.h"
#include "SEEKFREE_MT9V03X.h"

extern uint8 i,j,l,b,c,lable,t,y,x,maxlable,x_average,y_average,area;
extern uint8 mt9v03x_image2[MT9V03X_H][MT9V03X_W];
extern uint8 mt9v03x_image3[MT9V03X_H][MT9V03X_W];
extern uint8 mt9v03x_image33[MT9V03X_H][MT9V03X_W];
extern uint8 mt9v03x_image4[MT9V03X_H][MT9V03X_W];
extern uint8 zero[MT9V03X_H][MT9V03X_W];  //zero中放着是连通域标签一样的值
extern int parent[1000];  															//用于使标签一样的连通域相等
extern float density,scale;
extern uint8 Lable[100]; 
extern int bounder;
//生成透视变换图片
uint8 mt9v03x_image_222[MT9V03X_H][w_max]={0};
extern int num;
extern float num_max;
// **************************** 代码区域 ****************************
void erode(void)  //定义腐蚀函数
{
//如果一个白点四周九宫格范围内都是白点，则改点保留，否则黑化
	for(i=1;i<MT9V03X_H-1;i++) 
			{
				for(j=1;j<MT9V03X_W-1;j++)
				{
					if(
						mt9v03x_image2[i][j]==0||
						mt9v03x_image2[i+1][j]==0||
						mt9v03x_image2[i-1][j]==0||
						mt9v03x_image2[i][j+1]==0||
						mt9v03x_image2[i+1][j+1]==0||
						mt9v03x_image2[i+1][j-1]==0||
						mt9v03x_image2[i-1][j]==0||
						mt9v03x_image2[i-1][j+1]==0||
						mt9v03x_image2[i-1][j-1]==0
					)
						{
							mt9v03x_image3[i][j]=0;
						}
					else
					{
						mt9v03x_image3[i][j]=255;
					}
				}
			}
//边框填充
			for(i=0;i<MT9V03X_W;i++)
				{
					mt9v03x_image3[0][i]=mt9v03x_image2[0][i];
					mt9v03x_image3[MT9V03X_H-1][i]=mt9v03x_image2[MT9V03X_H-1][i];
				}
		  for(i=0;i<MT9V03X_H;i++)
				{
					mt9v03x_image3[i][0]=mt9v03x_image2[i][0];
					mt9v03x_image3[i][MT9V03X_W-1]=mt9v03x_image2[i][MT9V03X_W-1];
				}
}


void dilate(void)  //定义膨胀函数
{
	for(i=1;i<MT9V03X_H-1;i++) 
			{
				for(j=1;j<MT9V03X_W-1;j++)
				{
					if(
						mt9v03x_image2[i][j]==255||
						mt9v03x_image2[i+1][j]==255||
						mt9v03x_image2[i-1][j]==255||
						mt9v03x_image2[i][j+1]==255||
						mt9v03x_image2[i+1][j+1]==255||
						mt9v03x_image2[i+1][j-1]==255||
						mt9v03x_image2[i-1][j]==255||
						mt9v03x_image2[i-1][j+1]==255||
						mt9v03x_image2[i-1][j-1]==255
					)
						{
							mt9v03x_image33[i][j]=255;
						}
					else
					{
						mt9v03x_image33[i][j]=0;
					}
				}
			}
	for(i=0;i<MT9V03X_W;i++)
	{
		mt9v03x_image33[0][i]=mt9v03x_image3[0][i];
		mt9v03x_image33[MT9V03X_H-1][i]=mt9v03x_image3[MT9V03X_H-1][i];
	}
	for(i=0;i<MT9V03X_H;i++)
	{
		mt9v03x_image33[i][0]=mt9v03x_image3[i][0];
		mt9v03x_image33[i][MT9V03X_W-1]=mt9v03x_image3[i][MT9V03X_W-1];
	}
}

uint8 min(uint8 x,uint8 y)  //比较二者最小值的函数
{
	if(x>=y)
		return y;
	else
		return x;
}

uint8 max(uint8 x,uint8 y)  //比较二者最大值的函数
{
	if(x>=y)
		return x;
	else
		return y;
}

int Find(int x, int parent[])  //用于找根节点，可以一次找多个
{
//****用于修改标签值的函数，可以将一个连通域中的标签值修改为一个*****//
	 int i = x;                       
	 while (0 != parent[i])			   
					 i = parent[i];
	 return i;
}
void Union(int x, int y, int parent[])  
{

//****用于合并相同的连通域，主要采用的是比较左边和上面两个领域的标签值来进行合并，四领域合并方法****//
	 int i = x;
	 int j = y;
	 while (0 != parent[i])
				 i = parent[i];
	 while (0 != parent[j])
				 j = parent[j];
	 if (i != j)       
				 parent[i] = j;  
}




void two_pass(void)
{
	if(bounder<=0)y=0;
	else y=bounder;

	for(; y<MT9V03X_H; y++)
		{
			for(x=0; x<MT9V03X_W; x++)
					{
						if(mt9v03x_image2[y][x]!=0)
							{
								int left = (x-1<0)?0:zero[y][x-1];
								int up = (y-1<0)?0:zero[y-1][x];
										if(left!=0||up!=0)
										{
											if(left!=0&&up!=0)
											{
												zero[y][x] = min(left,up);
												if(left<=up)
												    Union(up,left,parent);
												else if(up<left)
												    Union(left,up,parent);
											}
								  else
									    zero[y][x]=max(left,up);
								}
								else
								{
								    zero[y][x]=++lable;
								}
							}
			//printf("%d  AAA BBB\r\n",lable);
					}
		}
		
	if(bounder<=0)y=0;
	else y=bounder;
		
  for(; y<MT9V03X_H;y++)
		{
			for(x=0; x<MT9V03X_W;x++)
			{
				if(zero[y][x]!=0)
					zero[y][x] = Find(zero[y][x],parent);
			}
		}
}

//****找到最大的连通域*****//
void findshape(void)
{
  uint8 count=0,counts=0,xlow=100,xhigh=0,ylow=100,yhigh=0,X=0,Y=0,M=0;
	
  for(i=1;i<=lable;i++)
	{
		count=0;
		counts=0;
		xlow=100;
		xhigh=0;
		ylow=100;
		yhigh=0;
		X=0;
		Y=0;
			
	if(bounder<=0)y=0;
	else y=bounder;
		
	  for(;y<MT9V03X_H;y++)
		{
		  for(x=0;x<MT9V03X_W;x++)
			{
			  count = zero[y][x];
				if(count == i)
				{counts++;
				if(x<xlow)
          xlow=x;
        if(x>xhigh)
          xhigh=x;
        if(y<ylow)
          ylow=y;
        if(y>yhigh)
          yhigh=y;	}				
			}
		}
	  X=xhigh-xlow+1;
		Y=yhigh-ylow+1;
		M=X*Y;
		density=M*1.0f/counts;
		scale=Y*1.0f/X;
//超级差就用
//			if(0<=density&&density<=1.45)
//			{
//				if(0.2<=scale&&scale<=0.9)
//					Lable[i]=i;
//			}
//			else if(scale==1&&density==0)
//			{
//				Lable[i]=i;
//			}
//			else
//				Lable[i]=0;
//			if(xhigh==xlow||ylow==yhigh)
				Lable[i]=i;
}
}
//****找到最大的连通域*****//
void findLabel(void)
{
	uint8 count=0,maxc=0,counts=0;
  for(i=1;i<=lable;i++)
	{
		if(Lable[i]!=0)
		{
	
	if(bounder<=0)y=0;
	else y=bounder;
	
		  for(;y<MT9V03X_H; y++)
		  {
			  for(x=0; x<MT9V03X_W; x++)
			  {
				  count = zero[y][x];
				  if(count==i)
					  counts++;
			  }
		  }
		  //printf(" %d, %d  \r\n",i,counts);
		  if(maxc < counts)
		  {	maxc = counts;
		 	  maxlable=i;
		  }
		  counts=0;
	}
	else
		maxlable=0;
		Lable[i]=0;     //清零
	}
}
//****只显示最大连通域//*****
void showLabel(void)
{
	if(maxlable!=0)
	{
		
	if(bounder<=0)y=0;
	else y=bounder;
		
	for(;y<MT9V03X_H;y++)
	{
		for(x=0;x<MT9V03X_W;x++)
		{
			if(zero[y][x]==maxlable)
				mt9v03x_image4[y][x] = 255;
			else
				mt9v03x_image4[y][x] = 0;
		}
	}
}
}


//*****用于找到连通域的中心坐标//*****
void findcenter_4(void)
{
	int x_sum=0,y_sum=0;
	
	if(bounder<=0)y=0;
	else y=bounder;
	
	for(;y<MT9V03X_H;y++)
	{
		for(x=0;x<MT9V03X_W;x++)
		{
			if(mt9v03x_image2[y][x]==255)
			{
				x_sum+=x;
				y_sum+=y;
				num++;
			}
		}
}	
	  area=num;
		x_average=x_sum/num;
		y_average=y_sum/num;
}


void findcenter_2(void)
{
	float x_sum=0,y_sum=0;
	int min=MT9V03X_W;	
	if(bounder<=0)y=0;
	else y=bounder;
	
	for(;y<MT9V03X_H;y++)
	{
		for(x=0;x<MT9V03X_W;x++)
		{
			if(mt9v03x_image2[y][x]==255)
			{
				if(x<=min)
					min=x;
				
				y_sum+=y;
				num++;
			}
		}
}	
	  area=num;
		if(min==MT9V03X_W)
			min=0;

		x_average=min;
		y_average=y_sum/num;
//	}
//	else
//	{x_average=0;
//		y_average=0;
//	}
//		printf("%d,%d\r\n",x_average,y_average);
}
//void findcenter(void)
//{
//	int num=0,x_sum=0,y_sum=0;
//	
//	if(bounder<=0)
//	y=0;
//	else 
//	y=bounder;
//	int sum_1=0;
////加权平均
//	for(;y<MT9V03X_H;y++)
//	{
//		for(x=0;x<MT9V03X_W;x++)
//		{
//				sum_1+=mt9v03x_image2[y][x];
//				x_sum+=x*mt9v03x_image2[y][x];
//				y_sum+=y*mt9v03x_image2[y][x];
//		}
//	}	
//	  area=num;
//		x_average=x_sum/sum_1;
//		y_average=y_sum/sum_1;
////		printf("%d,%d\r\n",x_average,y_average);
//}
int findwide(void)
{
	int left=MT9V03X_W,right=0;	
	//有界搜索
	if(bounder<=y_average-5)
	{
		if(y_average-5>0)y=y_average-10;
		else y=0;
	}
	else 
	{
		if(bounder>0)y=bounder;
		else y=0;
	}

	for(;y<y_average+5;y++)
	{
		for(x=1;x<MT9V03X_W;x++)
		{
				if(mt9v03x_image2[y][x]==255)
				{
					if(x<left)
					left=x;
					if(x>right)
					right=x;
				}		
		}
	}	
	return (right-left);
}

void fuction(void)
{
	int y,x;
	for(y=0;y<MT9V03X_H;y++)
	{
		for(x=0;x<MT9V03X_W;x++)
		{
			int wid=w_max-(w_max-w_min)*y/MT9V03X_H;
				mt9v03x_image_222[y][(w_max/2-(MT9V03X_W/2-x)*wid/MT9V03X_W)]=mt9v03x_image2[y][x];	
		}
	}	
}