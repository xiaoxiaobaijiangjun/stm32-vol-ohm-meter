#include "hw_key.h"

/*
 * Function Content: key instance registration
 * Function Parameter: key gpio¡¢pin and press_level
 * Return Value: KEY_HandleDef
 */
key_handle_t Key_Init(GPIO_TypeDef *key_gpio,uint16_t key_pin,uint8_t key_press_level)
{
	key_handle_t key_handle;
	key_handle.key_gpio = key_gpio;
	key_handle.key_pin = key_pin;
	key_handle.key_press_level = key_press_level;
	key_handle.key_state = KEY_NoPress;
	key_handle.keyCnt = 0;
	key_handle.keyCount = 0;
	key_handle.keyFcnt = 0;
	key_handle.keyLongFlag = 0;
	return key_handle;
}

/*
 * Function Content: scanf one key -- 10ms make one call
 * Function Parameter: KEY_HandleDef *key_handle
 * Return Value: No
 */
void Key_Scanf(key_handle_t *key_handle)
{
	key_handle->key_state = KEY_NoPress;
	//if key pressing
	if(HAL_GPIO_ReadPin(key_handle->key_gpio,key_handle->key_pin) == key_handle->key_press_level)
	{
		key_handle->keyCnt++;				//record the time when the button is pressed
		if(key_handle->keyCnt >= 120)
		{
			key_handle->keyCnt = 120;		//prevent data from exceeding the boundaries
		}
	}
	else
	{
		//if the key is lifted up
		if((key_handle->keyCnt >= 100) && (key_handle->keyLongFlag == 0)){
			//key press time > 1s and before no keyLongFlag,set the state as KeyLongPress
			key_handle->key_state = KeyLongPress;
		}
		else if(key_handle->keyCnt >= 2)
		{
			//key press time>20ms,proof is a single press
			key_handle->keyCount++;     			//key num increased
			key_handle->keyFcnt = DOUBLE_TIME;   	//set double click time
			key_handle->keyLongFlag = 1;    		//set keyLongFlag as 1
		}
		key_handle->keyCnt = 0;
		if(key_handle->keyFcnt)
		{
			key_handle->keyFcnt--;  //reduce double click time
			if(key_handle->keyFcnt <= 0)    
			{
					//double click time is end
					if(key_handle->keyCount == 1){
							key_handle->key_state = KeyPress;	//click
					}
					else if(key_handle->keyCount == 2){
							key_handle->key_state = KeyDoublePress;	//double click
					}
					key_handle->keyFcnt = 0;
					key_handle->keyCount = 0;
					key_handle->keyLongFlag = 0;
			}
		}
	}
}
