#include "Key.h"

uint8_t Key_row[1]={0xff};   //保存按键行扫描情况的状态数组

char KEY_ROW_SCAN(void)
{
    //读出行扫描状态
    //Key_row[0] = Key_row[0] | (HAL_GPIO_ReadPin(GPIOE,KEY_row1_Pin)<<2);
    //Key_row[0] = Key_row[0] | (HAL_GPIO_ReadPin(GPIOE,KEY_row2_Pin)<<1);
    //Key_row[0] = Key_row[0] | (HAL_GPIO_ReadPin(GPIOE,KEY_row3_Pin));
    
    if(Key_row[0] != 0x0f)         //行扫描有变化，判断该列有按键按下
    {
      HAL_Delay(10);                    //消抖
      if(Key_row[0] != 0x0f)
        {   
                //printf("Key_Row_DATA = 0x%x\r\n",Key_row[0]);
                switch(Key_row[0])
                {
                    case 0x0b:         //1011 判断为该列第1行的按键按下
                        return 1;
                    case 0x0d:         //1101 判断为该列第2行的按键按下
                        return 2;
                    case 0x0e:         //1110 判断为该列第3行的按键按下
                        return 3;
                    default :
                        return 0;
                }
        }
        else return 0;
    }
    else return 0;
}

char KEY_SCAN(void)
{    
    char Key_Num=0;       //1-16对应的按键数
    char key_row_num=0;        //行扫描结果记录
    
    KEY_CLO0_OUT_LOW;        
    if( (key_row_num=KEY_ROW_SCAN()) != 0 )
    { 
        while(KEY_ROW_SCAN() != 0);  //消抖
        Key_Num = 0 + key_row_num;
    }
    KEY_CLO0_OUT_HIGH;
    
    KEY_CLO1_OUT_LOW;        
    if( (key_row_num=KEY_ROW_SCAN()) != 0 )
    { 
        while(KEY_ROW_SCAN() != 0);
        Key_Num = 3 + key_row_num;
    }
    KEY_CLO1_OUT_HIGH;
    
    KEY_CLO2_OUT_LOW;    
    if( (key_row_num=KEY_ROW_SCAN()) != 0 )
    { 
        while(KEY_ROW_SCAN() != 0);
        Key_Num = 6 + key_row_num;
    }
    KEY_CLO2_OUT_HIGH;
    
    KEY_CLO3_OUT_LOW;    
    if( (key_row_num=KEY_ROW_SCAN()) != 0 )
    {
        while(KEY_ROW_SCAN() != 0);
        Key_Num = 12 + key_row_num;
    }
    KEY_CLO3_OUT_HIGH;
    
    return Key_Num;
}