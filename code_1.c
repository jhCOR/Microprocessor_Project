#include<stm32f10x.h>
#define Is_opening 1
#define Is_closing 2
#define Open_state 3
#define Closed_state 4

volatile unsigned int state, led_index, count, blink_count;
unsigned int col,blink_state;

volatile unsigned int is_operate,is_lock,is_repair;
static int row = 0;

u8 led9[9]={0x00,0x80,0xC0,0xE0,0xF0,0xF8,0xFC,0xFE, 0xFF};

int main(void){
	RCC->APB2ENR |= (1<<2)  /* GPIOA */
              | (1<<3)  /* GPIOB */
              | (1<<4)  /* GPIOC */
              | (1<<0)  /* AFIO   */
              | (1<<11);/* TIM1   */
	RCC->APB1ENR|=0x00000001;
	
	GPIOC->CRL = 0x33333333; // set output, port C
	GPIOB->CRH = 0x33333333; // set output, port B

  GPIOA->CRL = 0x88000000; // set PA6, 7 input
  GPIOA->CRH = 0x00000008; // set PA8 input
  GPIOA->ODR |= (1 << 6) | (1 << 7) | (1 << 8); // pull up

	EXTI->FTSR |= (1 << 8) | (1 << 7) | (1 << 6); // Falling trigger for PA8, PA7, PA6
	EXTI->IMR |= (1 << 8) | (1 << 7) | (1 << 6);  // Unmask EXTI lines 8, 7, 6
	AFIO->EXTICR[1] = 0x00000000;  // EXTI4~7
	AFIO->EXTICR[2] = 0x00000000;  // EXTI8~11
	NVIC->ISER[0] |= (1 << 23);
	

	TIM1->CR1 = 0x00;
	TIM1->CR2 = 0x00;
	TIM1->PSC = 0x07FF;
  TIM1->ARR = 0x0020;
	TIM1->DIER = 0x0001;
	NVIC->ISER[0] |= (1 << 25); // TIM1_UP Interrupt Enable
	
	TIM1->CR1|=0x0001; // timer for dot matrix ON
	
	TIM2->CR1=0x04;
	TIM2->CR2=0x00;
	TIM2->PSC = 35999; // Prescaler
	TIM2->ARR = 99 ; // Auto-reload register
	TIM2->DIER=0x0001;
	
	NVIC->ISER[0] |= (1<<28);
	
	row=1;
	col=0;
	
	state = Closed_state;
  led_index = 0; 
  is_lock = 0;   
  is_repair = 0; 
  blink_state = 0;
	count = 0;
	blink_count = 0;
	is_operate = 0;
	
	while(1){}
}

// for dot matrix
void TIM1_UP_IRQHandler(void){
	if((TIM1->SR&0x0001)!=0){
		// turn off alternatively row 0, 7
		
		if(row==1){ 
			GPIOC->ODR=~row;
			row=row<<7;
			
			col=led9[led_index];
			GPIOB->ODR=col<<8;
		}else{
			GPIOC->ODR=~row;
			row=1;
			col = 0x00; 
			if(is_lock==1){
				if(blink_count>=200){
					if(blink_state==0){
						col=0x80;
						blink_state = 1;
					}else{
						col=0x00;
						blink_state = 0;
					}
					blink_count=0;
				}else{
					blink_count++;
				}
			}else if(is_repair==1){
				if(blink_count>=200){

					if(blink_state==0){
						col=0x40;
						blink_state = 1;
					}else{
						col=0x00;
						blink_state = 0;
					}
					blink_count=0;
				}else{
					blink_count++;
				}
				
			}else if(is_lock==0 && is_repair==0){
				col=0xC0;
			}
			GPIOB->ODR=col<<8;
			
		}
		
		TIM1->SR&=~(1<<0); //clearUIF
	}
}

// for checking time
void TIM2_IRQHandler(void){
	if((TIM2->SR&0x0001)!=0){
		
		if(state==Is_opening){
			if(led_index < 8) led_index++;
			if(led_index>=8){
				state=Open_state;
			}
		}else if(state==Open_state){
			if(is_repair==1){
				TIM2->CR1 &= ~0x0001;
			}else{
				if(count==10){
					state=Is_closing;
					count = 0;
				}
			}
		}else if(state==Is_closing){
			if(count==2){
				led_index--;
				count = 0;
			}
			if(led_index==0){
					state = Closed_state;
					TIM2->CR1 &= ~0x0001; 
			}
		}
		count++;
		TIM2->SR&=~(1<<0); //clearUIF
	}
}

void EXTI9_5_IRQHandler(void) {

    if (EXTI->PR & (1 << 8)) { // S1 button is pushed
			if(is_lock == 0 && is_repair == 0){ 
				is_operate = 1;
				state = Is_opening;
				count = 0;
				
				TIM2->CR1 |= 0x0001; 
			}
			EXTI->PR |= (1 << 8); 
    }
    else if (EXTI->PR & (1 << 7)) { // S2 button is pushed
			if(is_repair == 0){ 
				is_lock = !is_lock;
				
				if(is_lock == 1){
					is_operate = 0;
					is_repair = 0;
					count = 0;
					
					state = Is_closing;
					
					TIM2->CR1 |= 0x0001;
				}
			}
			EXTI->PR |= (1 << 7); 
    }
		
    else if (EXTI->PR & (1 << 6)) { // S3 button is pushed
		is_repair = !is_repair;
			
		if(is_repair == 1){ 
			count = 0;
			is_lock = 0;
			is_operate = 0;
			state = Is_opening; // state = Is_closing, is_lock==1 -> need to close before lock
			
			TIM2->CR1 |= 0x0001; 
		} else { 
			count = 0;
			state = Is_closing;
			
			TIM2->CR1 |= 0x0001; 
		}
		EXTI->PR |= (1 << 6); 
    }
}
