
////////////////////////////////////////////////////////////////////////////////////////////

// method to create a more noticable notification to the user
// instead of a 0.5 second pulse, there will be three 0.4 second pulses separated by 0.15 seconds
void alert_user();

void alert_user(){
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
	  HAL_Delay(400);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
	  HAL_Delay(150);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
	  HAL_Delay(400);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
	  HAL_Delay(150);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
	  HAL_Delay(400);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
}
