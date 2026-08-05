#include "mid_lcd.h"
#include "mid_lcdfont.h"

/*
 * Function Content: lcd fill with the specified color and coordinates
 * Function Parameter: lcd interface with x,y,xend,yend coordinates and color
 * Return Value: null
 */
void lcd_fill(lcd_interface *lcd_t,uint16_t xsta,uint16_t ysta,
				uint16_t xend,uint16_t yend,uint16_t color)
{
	uint16_t i = 0,j = 0;
	lcd_t->lcd_address_set(lcd_t,xsta,ysta,xend-1,yend-1);
	for(i=ysta;i<yend;i++)
	{
		for(j=xsta;j<xend;j++)
		{
			lcd_t->lcd_write_data16(lcd_t,color);
		}
	}
}

/*
 * Function Content: lcd draw dots at the specified position
 * Function Parameter: lcd interface with x,y coordinates and color
 * Return Value: null
 */
void lcd_drawPoint(lcd_interface *lcd_t,uint16_t x, uint16_t y, uint16_t color)
{
	lcd_t->lcd_address_set(lcd_t,x,y,x,y);
	lcd_t->lcd_write_data16(lcd_t,color);
}

/*
 * Function Content: lcd draw line at the specified position
 * Function Parameter: lcd interface with x1,y1,x2,y2 coordinates and color
 * Return Value: null
 */
void lcd_drawLine(lcd_interface *lcd_t,uint16_t x1,uint16_t y1,
					uint16_t x2,uint16_t y2,uint16_t color)
{
	uint16_t t = 0;
	int xerr=0,yerr=0,delta_x=0,delta_y=0,distance=0;
	int incx=0,incy=0,uRow=0,uCol=0;
	delta_x=x2-x1;	//Calculate coordinate increments
	delta_y=y2-y1;
	uRow=x1;
	uCol=y1;	//line start coordinate
	if(delta_x>0){
		incx = 1;	//Set the single-step direction
	}
	else if(delta_x==0){
		incx = 0;	// perpendicular line
	}
	else {
		incx=-1;
		delta_x=-delta_x;
	}
	if(delta_y>0){
		incy=1;
	}
	else if (delta_y==0){
		incy=0;	//horizontal line
	}
	else {
		incy=-1;
		delta_y=-delta_y;
	}
	if(delta_x>delta_y){
		distance=delta_x;
	}
	else {
		distance=delta_y;	//Select the basic increment coordinate axes
	}
	for(t=0;t<distance+1;t++)
	{
		lcd_drawPoint(lcd_t,uRow,uCol,color);
		xerr+=delta_x;
		yerr+=delta_y;
		if(xerr>distance)
		{
			xerr-=distance;
			uRow+=incx;
		}
		if(yerr>distance)
		{
			yerr-=distance;
			uCol+=incy;
		}
	}
}

/*
 * Function Content: lcd draw rectangle at the specified position
 * Function Parameter: lcd interface with x1,y1,x2,y2 coordinates and color
 * Return Value: null
 */
void lcd_drawRectangle(lcd_interface *lcd_t,uint16_t x1, uint16_t y1,
						uint16_t x2, uint16_t y2,uint16_t color)
{
	lcd_drawLine(lcd_t,x1,y1,x2,y1,color);
	lcd_drawLine(lcd_t,x1,y1,x1,y2,color);
	lcd_drawLine(lcd_t,x1,y2,x2,y2,color);
	lcd_drawLine(lcd_t,x2,y1,x2,y2,color);
}

/*
 * Function Content: lcd draw circle at the specified position
 * Function Parameter: lcd interface with x0,y0 coordinates and r,color
 * Return Value: null
 */
void lcd_drawCircle(lcd_interface *lcd_t, uint16_t x0,uint16_t y0,uint8_t r,uint16_t color)
{
	uint16_t a=0,b=0;
	a=0;b=r;	  
	while(a<=b)
	{
		lcd_drawPoint(lcd_t,x0-b,y0-a,color);             //3
		lcd_drawPoint(lcd_t,x0+b,y0-a,color);             //0
		lcd_drawPoint(lcd_t,x0-a,y0+b,color);             //1
		lcd_drawPoint(lcd_t,x0-a,y0-b,color);             //2
		lcd_drawPoint(lcd_t,x0+b,y0+a,color);             //4
		lcd_drawPoint(lcd_t,x0+a,y0-b,color);             //5
		lcd_drawPoint(lcd_t,x0+a,y0+b,color);             //6
		lcd_drawPoint(lcd_t,x0-b,y0+a,color);             //7
		a++;
		if((a*a+b*b)>(r*r))//Determine whether the point to be drawn is too far away
		{
			b--;
		}
	}
}

/*
 * Function Content: lcd show char at the specified position and size
 * Function Parameter: lcd interface with x,y coordinates and num,fc,bc,sizey,mode
 * 						mode = 0 Non-overlapping mode,= 1 overlapping mode
 * Return Value: null
 */
void lcd_showChar(lcd_interface *lcd_t, uint16_t x,uint16_t y,
				uint8_t num,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode)
{
	uint8_t temp=0,sizex=0,t=0,m=0;
	uint16_t i=0,TypefaceNum=0;
	uint16_t x0 = x;
	sizex=sizey/2;
	TypefaceNum=(sizex/8+((sizex%8)?1:0))*sizey;
	num = num - ' ';
	lcd_t->lcd_address_set(lcd_t,x,y,x+sizex-1,y+sizey-1);
	for(i=0;i<TypefaceNum;i++)
	{ 
		if(sizey==12){
			temp = ascii_1206[num][i];
		}
		else if(sizey==16){
			temp = ascii_1608[num][i];
		}
		else if(sizey==24){
			temp = ascii_2412[num][i];
		}
		else if(sizey==32){
			temp = ascii_3216[num][i];
		}
		else{
			return;
		}
		for(t=0;t<8;t++)
		{
			if(!mode)
			{
				if(temp & (0x01<<t)){
					lcd_t->lcd_write_data16(lcd_t,fc);
				}
				else{
					lcd_t->lcd_write_data16(lcd_t,bc);
				}
				m++;
				if(m%sizex==0){
					m=0;
					break;
				}
			}
			else
			{
				if(temp & (0x01<<t)){
					lcd_drawPoint(lcd_t,x,y,fc);
				}
				x++;
				if((x-x0)==sizex)
				{
					x=x0;
					y++;
					break;
				}
			}
		}
	}   	 	  
}

/*
 * Function Content: lcd show string at the specified position and size
 * Function Parameter: lcd interface with x,y coordinates and *p,fc,bc,sizey,mode
 * 						mode = 0 Non-overlapping mode,= 1 overlapping mode
 * Return Value: null
 */
void lcd_showString(lcd_interface *lcd_t,uint16_t x,uint16_t y,
			const uint8_t *p,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode)
{         
	while(*p!='\0')
	{       
		lcd_showChar(lcd_t,x,y,*p,fc,bc,sizey,mode);
		x+=sizey/2;
		p++;
	}  
}

/*
 * Function Content: lcd show chinese at the specified position and size
 * Function Parameter: lcd interface with x,y coordinates and *s,fc,bc,sizey,mode
 * 						mode = 0 Non-overlapping mode,= 1 overlapping mode
 * Return Value: null
 */
void lcd_ShowChinese(lcd_interface *lcd_t, uint16_t x,uint16_t y,
			uint8_t *s,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode)
{
	while(*s!=0)
	{
		if(sizey==12) lcd_showChinese12x12(lcd_t,x,y,s,fc,bc,sizey,mode);
		else if(sizey==16) lcd_showChinese16x16(lcd_t,x,y,s,fc,bc,sizey,mode);
		else if(sizey==24) lcd_showChinese24x24(lcd_t,x,y,s,fc,bc,sizey,mode);
		else if(sizey==32) lcd_showChinese32x32(lcd_t,x,y,s,fc,bc,sizey,mode);
		else return;
		s+=2;
		x+=sizey;
	}
}

/*
 * Function Content: lcd show chinese12x12 at the specified position and size
 * Function Parameter: lcd interface with x,y coordinates and *s,fc,bc,sizey,mode
 * 						mode = 0 Non-overlapping mode,= 1 overlapping mode
 * Return Value: null
 */
void lcd_showChinese12x12(lcd_interface *lcd_t,uint16_t x,uint16_t y,
		uint8_t *s,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode)
{
	uint8_t i=0,j=0,m=0;
	uint16_t k=0;
	uint16_t HZnum=0;
	uint16_t TypefaceNum=0;
	uint16_t x0=x;
	TypefaceNum=(sizey/8+((sizey%8)?1:0))*sizey;
	HZnum=sizeof(tfont12)/sizeof(typFNT_GB12);
	for(k=0;k<HZnum;k++) 
	{
		if((tfont12[k].Index[0]==*(s))&&(tfont12[k].Index[1]==*(s+1)))
		{ 	
			lcd_t->lcd_address_set(lcd_t,x,y,x+sizey-1,y+sizey-1);
			for(i=0;i<TypefaceNum;i++)
			{
				for(j=0;j<8;j++)
				{	
					if(!mode)
					{
						if(tfont12[k].Msk[i]&(0x01<<j)){
							lcd_t->lcd_write_data16(lcd_t,fc);
						}
						else {
							lcd_t->lcd_write_data16(lcd_t,bc);
						}
						m++;
						if(m%sizey==0)
						{
							m=0;
							break;
						}
					}
					else
					{
						if(tfont12[k].Msk[i]&(0x01<<j)){
							lcd_drawPoint(lcd_t,x,y,fc);
						}
						x++;
						if((x-x0)==sizey)
						{
							x=x0;
							y++;
							break;
						}
					}
				}
			}
		}				  	
		continue;
	}
} 

/*
 * Function Content: lcd show chinese16x16 at the specified position and size
 * Function Parameter: lcd interface with x,y coordinates and *s,fc,bc,sizey,mode
 * 						mode = 0 Non-overlapping mode,= 1 overlapping mode
 * Return Value: null
 */
void lcd_showChinese16x16(lcd_interface *lcd_t,uint16_t x,uint16_t y,
		uint8_t *s,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode)
{
	uint8_t i=0,j=0,m=0;
	uint16_t k=0;
	uint16_t HZnum=0;
	uint16_t TypefaceNum=0;
	uint16_t x0=x;
	TypefaceNum=(sizey/8+((sizey%8)?1:0))*sizey;
	HZnum=sizeof(tfont16)/sizeof(typFNT_GB16);
	for(k=0;k<HZnum;k++) 
	{
		if ((tfont16[k].Index[0]==*(s))&&(tfont16[k].Index[1]==*(s+1)))
		{ 	
			lcd_t->lcd_address_set(lcd_t,x,y,x+sizey-1,y+sizey-1);
			for(i=0;i<TypefaceNum;i++)
			{
				for(j=0;j<8;j++)
				{	
					if(!mode)
					{
						if(tfont16[k].Msk[i]&(0x01<<j)){
							lcd_t->lcd_write_data16(lcd_t,fc);
						}
						else {         
							lcd_t->lcd_write_data16(lcd_t,bc);
						}
						m++;
						if(m%sizey==0)
						{
							m=0;
							break;
						}
					}
					else
					{
						if(tfont16[k].Msk[i]&(0x01<<j))
						{
							lcd_drawPoint(lcd_t,x,y,fc);
						}
						x++;
						if((x-x0)==sizey)
						{
							x=x0;
							y++;
							break;
						}
					}
				}
			}
		}				  	
		continue;
	}
} 

/*
 * Function Content: lcd show chinese24x24 at the specified position and size
 * Function Parameter: lcd interface with x,y coordinates and *s,fc,bc,sizey,mode
 * 						mode = 0 Non-overlapping mode,= 1 overlapping mode
 * Return Value: null
 */
void lcd_showChinese24x24(lcd_interface *lcd_t,uint16_t x,uint16_t y,
		uint8_t *s,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode)
{
	uint8_t i=0,j=0,m=0;
	uint16_t k=0;
	uint16_t HZnum=0;
	uint16_t TypefaceNum=0;
	uint16_t x0=x;
	TypefaceNum=(sizey/8+((sizey%8)?1:0))*sizey;
	HZnum=sizeof(tfont24)/sizeof(typFNT_GB24);
	for(k=0;k<HZnum;k++) 
	{
		if ((tfont24[k].Index[0]==*(s))&&(tfont24[k].Index[1]==*(s+1)))
		{ 	
			lcd_t->lcd_address_set(lcd_t,x,y,x+sizey-1,y+sizey-1);
			for(i=0;i<TypefaceNum;i++)
			{
				for(j=0;j<8;j++)
				{	
					if(!mode)
					{
						if(tfont24[k].Msk[i]&(0x01<<j)){
							lcd_t->lcd_write_data16(lcd_t,fc);
						}
						else{
							lcd_t->lcd_write_data16(lcd_t,bc);
						}
						m++;
						if(m%sizey==0)
						{
							m=0;
							break;
						}
					}
					else
					{
						if(tfont24[k].Msk[i]&(0x01<<j)){
							lcd_drawPoint(lcd_t,x,y,fc);
						}
						x++;
						if((x-x0)==sizey)
						{
							x=x0;
							y++;
							break;
						}
					}
				}
			}
		}				  	
		continue;
	}
} 

/*
 * Function Content: lcd show chinese32x32 at the specified position and size
 * Function Parameter: lcd interface with x,y coordinates and *s,fc,bc,sizey,mode
 * 						mode = 0 Non-overlapping mode,= 1 overlapping mode
 * Return Value: null
 */
void lcd_showChinese32x32(lcd_interface *lcd_t,uint16_t x,uint16_t y,
		uint8_t *s,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode)
{
	uint8_t i=0,j=0,m=0;
	uint16_t k=0;
	uint16_t HZnum=0;
	uint16_t TypefaceNum=0;
	uint16_t x0=x;
	TypefaceNum=(sizey/8+((sizey%8)?1:0))*sizey;
	HZnum=sizeof(tfont32)/sizeof(typFNT_GB32);
	for(k=0;k<HZnum;k++) 
	{
		if ((tfont32[k].Index[0]==*(s))&&(tfont32[k].Index[1]==*(s+1)))
		{ 	
			lcd_t->lcd_address_set(lcd_t,x,y,x+sizey-1,y+sizey-1);
			for(i=0;i<TypefaceNum;i++)
			{
				for(j=0;j<8;j++)
				{	
					if(!mode)
					{
						if(tfont32[k].Msk[i]&(0x01<<j)){
							lcd_t->lcd_write_data16(lcd_t,fc);
						}
						else {
							lcd_t->lcd_write_data16(lcd_t,bc);
						}
						m++;
						if(m%sizey==0)
						{
							m=0;
							break;
						}
					}
					else
					{
						if(tfont32[k].Msk[i]&(0x01<<j)){
							lcd_drawPoint(lcd_t,x,y,fc);
						}
						x++;
						if((x-x0)==sizey)
						{
							x=x0;
							y++;
							break;
						}
					}
				}
			}
		}				  	
		continue;
	}
}

