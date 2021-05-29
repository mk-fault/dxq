/*******************************************************************************
* 文档: 3D_Rotateapplitation.c
* 作者: 执念执战
* 		QQ:572314251
*		微信：zhinianzhizhan
*		随梦，随心，随愿，恒执念，为梦执战，执战苍天！
*
* 描述:
*	1.三维图形变换操作
*       2.3D-transform.c包含“变换算法”和“投影算法”
*       
*       3.3D_Rotateapplitation对“变换矩阵算法”和“投影算法”的应用
*       4.rotation_font3D.c实现三维空间中旋转图片（优化算法）
*       5.本3D算法学习自《基于STC单片机“12864液晶显示旋转立方体和图片（优化算法）”实例》
*	6.		作者是Wu LianWei
*	7.本文档为3D_Rotateapplitation对“变换矩阵算法”和“投影算法”的应用
*	8：注意：视点到平面的距离，当屏幕上的立方图转着转着乱了，
		满图画色时试着改改视点到物体的距离FOCAL_DISTANCE、比例变换矩阵中数值的大小及XO,YO 的大小
		要确保图形旋转过程中不会超出边界，不然就会乱码
	9.下面注释中的240 400等数字是之前在240*400的屏幕上移植时写的，自行转换为128   64 等，不影响
	
******* 进行了后期整合，将多个文档整合为四个。//时间：2017/2/5
*******
	10.转载使用时请注明出处
*******************************************************************************/



#include <math.h>
#include <Display_3D.h>


/**********************************************************/
//**函数：Retate_cube
//**功能：旋转立方体
//**说明：  1：旋转立方体进行3D显示，基于3D图形变换算法和投影算法
//				  
//					 边学边写
//**作者：  执念执战
///**时间：2015-11-30 
/***********************************************************/


#define  SIZE  8

_3Dzuobiao const Cube[8]=		//const 相当于51 中的code，表示存放在rom中且为只读
{																	
	{0,0,0},
	{16,0,0},//将正方体的边长改长一些，适应800x480的大屏，不然忒小    2018-12-1 执念执战 wcc
	{0,16,0},
	{16,16,0},
	
	{0,0,16},
	{16,0,16},	//只这是正方体特征点坐标,相当于正方形的边长，可以增加一个函数来修改长短，以及9变成长方形或其他的
	{0,16,16},	//后面有些参数没有单独剥离出来，所以此处修改后，有些地方可能也要相应修改。请实际操作进行修改。
	{16,16,16}	//在arduino的游戏机中看到过有画三维旋转飞机的，原理上应该一样，将飞机的各个点存起来，区别就是画线的顺序。
				//所以以后可以剥离出来按照点顺序来连接画线的函数，就可以画任意的图像了。等以后画掌机了再修改吧 2017-2-5
	
 //   {0,0,0},
 //   {8,0,0},
 //   {0,8,0},
 //   {8,8,0},
    
//  {0,0,8},
//	{8,0,8},	//只这是正方体特征点坐标,相当于正方形的边长，可以增加一个函数来修改长短，以及9变成长方形或其他的
//	{0,8,8},	//后面有些参数没有单独剥离出来，所以此处修改后，有些地方可能也要相应修改。请实际操作进行修改。
//	{8,8,8}	//在arduino的游戏机中看到过有画三维旋转飞机的，原理上应该一样，将飞机的各个点存起来，区别就是画线的顺序。
				//所以以后可以剥离出来按照点顺序来连接画线的函数，就可以画任意的图像了。等以后画掌机了再修改吧 2017-2-5
	
//	{-1,-1,-1},
//{1,-1,-1},
//{1,-1,1},
//{-1,-1,1}, 
//{-1, 1,-1},
//{1, 1,-1},
//{1, 1,1},
//{-1, 1,1}
};

_3Dzuobiao const Triangle[3]=
{
	{4,2,0},
	{2,6,0},
	{6,6,0}
};

///**********************************************************/
///**函数: Rate_cube
///**功能：旋转立方体，透视投影
///**说明：x,y,z,分别为对应轴上的旋转角度
//				color为颜色
//				XO,YO 为原点

//  只能按照预设的方向旋转
//					 边学边写
//**作者：  执念执战
///**时间：2015-11-30
//修改时间： 2017/2/5   23:00
//					添加原点设置的形参
///***********************************************************/

void RateCube(float x,float y,float z,GUI_COLOR color,uint16_t XO,uint16_t YO)
{
	
	
//int16_t  XO;
//int16_t  YO;
	float gMAT[4][4];
	uint8_t i;
	_3Dzuobiao temp;
	_2Dzuobiao Cube_dis[8];
	
	structure_3D(gMAT);          	//构造为单位矩阵
	Translate3D(gMAT,-8,-8,-8); 	//平移变换矩阵,当为边长的一半的倒数时，图像绕着自己旋转，即原点在中心
	Scale_3D(gMAT,2,2,2);		//比例变换矩阵
	Rotate_3D(gMAT,x,y,z);		//旋转变换矩阵
	Translate3D(gMAT,0,0,0); 	//平移变换矩阵	   x:调节距离中心点的位置，相当于下面Point0.z
					//		y：上下调动位置 ，具体根据实际修改
//	XO=128;		//128  正中
//	YO=64+64;	//0 时最低边显示在y=0上
	
	for(i=0;i<8;i++)
{
		temp=vector_matrix_MULTIPLY(Cube[i],gMAT);	//矢量与矩阵相乘
		Cube_dis[i]=	PerProject(temp,XO,YO);		//正射投影
		Cube_dis[i].x+=OLED_X_MAX;
		Cube_dis[i].y+=OLED_Y_MAX;
		Cube_dis[i].x%=OLED_X_MAX;			//用来解决超出屏幕后乱码的问题。去掉后顺时针转到超出左边界后会找不到坐标无限划线，
		Cube_dis[i].y%=OLED_Y_MAX;			//加上屏幕的宽度就解决了，按照说明书，宽度显存为240，高度为432（虽然像素为400个），
					//	还要注意图像不要大到超过两个屏
}

	 GUI_SetColor(color);
	 GUI_DrawLine(Cube_dis[0].x,Cube_dis[0].y,Cube_dis[1].x,Cube_dis[1].y);
	 GUI_DrawLine(Cube_dis[0].x,Cube_dis[0].y,Cube_dis[2].x,Cube_dis[2].y);
	 GUI_DrawLine(Cube_dis[3].x,Cube_dis[3].y,Cube_dis[1].x,Cube_dis[1].y);
	 GUI_DrawLine(Cube_dis[3].x,Cube_dis[3].y,Cube_dis[2].x,Cube_dis[2].y);
	
	
	 GUI_DrawLine(Cube_dis[0+4].x,Cube_dis[0+4].y,Cube_dis[1+4].x,Cube_dis[1+4].y);
	 GUI_DrawLine(Cube_dis[0+4].x,Cube_dis[0+4].y,Cube_dis[2+4].x,Cube_dis[2+4].y);
	 GUI_DrawLine(Cube_dis[3+4].x,Cube_dis[3+4].y,Cube_dis[1+4].x,Cube_dis[1+4].y);
	 GUI_DrawLine(Cube_dis[3+4].x,Cube_dis[3+4].y,Cube_dis[2+4].x,Cube_dis[2+4].y);
	
	
	 GUI_DrawLine(Cube_dis[0].x,Cube_dis[0].y,Cube_dis[0+4].x,Cube_dis[0+4].y);
	 GUI_DrawLine(Cube_dis[1].x,Cube_dis[1].y,Cube_dis[1+4].x,Cube_dis[1+4].y);
	 GUI_DrawLine(Cube_dis[2].x,Cube_dis[2].y,Cube_dis[2+4].x,Cube_dis[2+4].y);
	 GUI_DrawLine(Cube_dis[3].x,Cube_dis[3].y,Cube_dis[3+4].x,Cube_dis[3+4].y);
		



}


///**********************************************************/
///**函数: RotatePic32x32
///**功能：在三维空间中旋转一个32x32的字符或图形
///**说明：1：根据设定的方式变换每个图片的每个点的坐标，并显示
//			2: 旋转角度  0~360度
//			3：az取正值为顺时针旋转
//					 边学边写		
//**作者：  执念执战
///**时间：2015-11-30
//修改时间： 2017/2/5   23:00
//					添加原点设置的形参
///***********************************************************/


void RotatePic32X32(unsigned char *dp,float ax,float ay,float az,GUI_COLOR color,uint16_t XO,uint16_t YO,uint16_t ZO)
{
//	int16_t  XO;
//	int16_t  YO;
	uint8_t i,j,k;
	uint16_t temp;
	float gMAT[4][4];
	_3Dzuobiao Point0;
	_3Dzuobiao Point1;
	_2Dzuobiao PointDis;
	
	structure_3D(gMAT);						//构建单位矩阵
	Translate3D(gMAT,-16,-16,-ZO); 	//平移变换矩阵
	Scale_3D(gMAT,1,1,1);		 //比例变换矩阵
	Rotate_3D(gMAT,ax,ay,az);	//旋转变换矩阵
	Translate3D(gMAT,16,16,ZO); 	//平移变换矩阵	   x:调节距离中心点的位置，相当于下面Point0.z
					//		  y：上下调动位置 ，具体根据实际修改
//	XO=128;
//	YO=64;
	
	for(j=0;j<32;j++)
	{
		for(i=0;i<4;i++)
		{
			for(k=0;k<8;k++)
			{
				temp=0x01<<k;//_crol_(0x01,k);
				if(*(dp+j)&temp)
				{
					Point0.x=(i * 8) + (7 - k);
					Point0.y=j;
					Point0.z=0;		//此参数能够改变字符距离旋转轴中心的距离
					
					Point1=vector_matrix_MULTIPLY(Point0,gMAT);//矢量与矩阵相乘
					PointDis=PerProject(Point1,XO,YO);	   //映射投影
					PointDis.x+=OLED_X_MAX;
					PointDis.y+=OLED_Y_MAX; //用来解决超出屏幕后乱码的问题。去掉后顺时针转到超出左边界后会找不到坐标无限划线，
					PointDis.x%=OLED_X_MAX;	//加上屏幕的宽度就解决了，按照说明书，宽度显存为240，高度为432（虽然像素为400个），
					PointDis.y%=OLED_Y_MAX;				//	还要注意图像不要大到超过两个屏
					GUI_DrawPoint(PointDis.x,PointDis.y,color);
				}
			}
		}
		dp+=4;
	}
	
	
	
}