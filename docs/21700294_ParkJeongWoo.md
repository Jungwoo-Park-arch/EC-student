---
description: EC_HAL API
---

# Embedded Controller HAL

Written by:   박정우

Course:  임베디드컨트롤러



Program: 		C/C++

IDE/Compiler: Keil uVision 5

OS: 					WIn10

MCU:  				STM32F411RE, Nucleo-64





## GPIO Digital In/Out 

### Header File

 `#include "ecGPIO.h"`


```c++
#include "stm32f4xx.h"

#ifndef __ECGPIO_H
#define __ECGPIO_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

// pin
#define      PA5                5
#define      PA6                6
#define      PA7                7
#define      PB6                6
#define      PC7                7
#define      PA9                9
#define      PA8                8
#define      PB10               10
	 
#define      BUTTON_PIN        13

// Setting
#define      HIGH               1 
#define      LOW                0

// MODE Setting
#define      INPUT        			0
#define      OUTPUT       			1 
#define      ALTERNATE    			2
#define      ANALOG       			3 

// Output type Setting
#define      PUSH_PULL 					0
#define      OPEN_DRAIN 				1

// Output speed Setting
#define      LOW_SPEED          0
#define      MEDIUM_SPEED       1 
#define      FAST_SPEED         2 
#define      HIGH_SPEED         3 

// Output PUPD Setting
#define      NO_PUPD    	      0
#define      PULL_UP            1 
#define      PULL_DOWN          2 
#define      RESERVED_2         3 


void GPIO_init(GPIO_TypeDef *Port, int pin, uint32_t mode);

void GPIO_write(GPIO_TypeDef *Port, int pin, uint32_t output);

uint32_t  GPIO_read(GPIO_TypeDef *Port, uint32_t pin);

void GPIO_mode(GPIO_TypeDef* Port, int pin, uint32_t mode);

void GPIO_ospeed(GPIO_TypeDef* Port, int pin, uint32_t speed);

void GPIO_otype(GPIO_TypeDef* Port, int pin, uint32_t type);

void GPIO_pudr(GPIO_TypeDef* Port, int pin, uint32_t pudr);

void sevensegment_init(void);

void sevensegment_decode(int number);

void GPIO_setting(GPIO_TypeDef* Port, int pin, uint32_t speed, uint32_t type, uint32_t pudr);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
```




### GPIO_init\(\)

Initializes GPIO pins with default setting and Enables GPIO Clock. Mode: In/Out/AF/Analog

```c++
void GPIO_init(GPIO_TypeDef *Port, int pin, int mode);
```

**Parameters**

* **Port:**  Port Number,  GPIOA~GPIOH

* **pin**:  pin number (int) 0~15

* **mode**:   INPUT (0), OUTPUT (1),  AF(02), ANALOG (03)

  

**Example code**

```c++
GPIO_init(GPIOA, 5, OUTPUT);
GPIO_init(GPIOC, 13, INPUT); //GPIO_init(GPIOC, 13, 0);
```



### GPIO_mode\(\)

Configures  GPIO pin modes: In/Out/AF/Analog

```c++
void GPIO_mode(GPIO_TypeDef *Port, int pin, int mode);
```

**Parameters**

* **Port:**  Port Number,  GPIOA~GPIOH

* **pin**:  pin number (int) 0~15

* **mode**:   INPUT (0), OUTPUT (1),  AF(02), ANALOG (03)

  

**Example code**

```c++
GPIO_mode(GPIOA, 5, OUTPUT); //set pin5 output mode
```



### GPIO_pupdr\(\)

Configure the I/O pull-up or pull-down: No PullupPulldown/ Pull-Up/Pull-Down/Analog/Reserved

```c++
void GPIO_pupdr(GPIO_TypeDef* Port, int pin, int pupd);
```

**Parameters**

* **Port:**  Port Number,  GPIOA~GPIOH
* **pin**:  pin number (int) 0~15
* **pupd**:   No PullupPulldown (0),  Pull-Up(1), Pull-Down(2) , Reserved(3)



**Example code**

```c++
GPIO_pupdr(GPIOA, 5, 0);  // 0: No PUPD pin5
```



### GPIO_ospeed\(\)

Configure the I/O output speed: Low speed/Medium speed/Fast speed/High speed

```c++
void GPIO_ospeed(GPIO_TypeDef* Port, int pin, int speed);
```

**Parameters**

* **Port:**  Port Number,  GPIOA~GPIOH
* **pin**:  pin number (int) 0~15
* **speed**:  Low speed(0), Medium speed(1), Fast speed(2), High speed(3)



**Example code**

```c++
GPIO_ospeed(GPIOA, 5, 3);  // 3: Fast speed pin5
```



### GPIO_otype\(\)

Configure the output type of the I/O port:  Output push-pull/Output open-drain

```c++
void GPIO_otype(GPIO_TypeDef* Port, int pin, int type);
```

**Parameters**

* **Port:**  Port Number,  GPIOA~GPIOH
* **pin**:  pin number (int) 0~15
* **type**:   Output push-pull(0)/Output open-drain(1)



**Example code**

```c++
GPIO_otype(GPIOA, 5, 0);  // 0: push-pull
```



### GPIO_read\(\)

Receive the input signal 

```c++
int GPIO_read(GPIO_TypeDef* Port, int pin);
```

**Parameters**

* **Port:**  Port Number,  GPIOA~GPIOH
* **pin**:  pin number (int) 0~15



**Example code**

```c++
GPIO_read(GPIOA, 13);  // read signal of GPIOA pin13
```



### GPIO_write(\)

Configures output of on/off: LOW/HIGH

```c++
void GPIO_write(GPIO_TypeDef* Port, int pin, int output);
```

**Parameters**

* **Port:**  Port Number,  GPIOA~GPIOH
* **pin**:  pin number (int) 0~15
* **output**:   LOW(0), HIGH(1)



**Example code**

```c++
GPIO_output(GPIOA, 5, 0);  // 0: LOW 
```



### sevensegment_init(\)

Initializes 7-segment  8pins

```c++
void sevensegment_init();
```

This function includes others functions 

ex) GPIO_init, GPIO_pudr, GPIO_otype, GPIO_ospeed

so, it define register's state

**Example code**

```c++
void sevensegment_init(); // setting registers
```



### sevensegment_decode(\)

According to Input signal, change the 7-segment display

```c++
void sevensegment_decode(int number);
```

**Parameters**

* number: the number shown on the display (0~9)

**Example code**

```c++
void sevensegment_init(5); // Appear to number 5 on the 7-segment
```



------

## Interrupt EXTI 

### Header File

 `#include "ecEXTI.h"`


```c++
#include "stm32f4xx.h"

#ifndef __ECEXTI_H
#define __ECEXTI_H

#define FALL 0
#define RISE 1
#define BOTH 2

#define PA_pin 0x0
#define PB_pin 0x1
#define PC_pin 0x2
#define PD_pin 0x3
#define PE_pin 0x4

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */
	 
void EXTI_init (GPIO_TypeDef *Port, uint32_t pin, int edge , int prior);
void EXTI_enable(uint32_t pin);
void EXTI_disnable(uint32_t pin);
uint32_t is_pending_EXTI(uint32_t pin);
void clear_pending_EXTI(uint32_t pin);
void EXTI15_10_IRQHandler(void);
void LED_toggle(GPIO_TypeDef *Port, uint32_t pin);
	 
#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
```



### EXTI_init\(\)

Initializes EXTI pins with default setting. 

```c++
void EXTI_init (GPIO_TypeDef *Port, uint32_t pin, int edge , int prior);
```

**Parameters**

* **Port:**  Port Number,  GPIOA~GPIOH
* **pin**:  pin number (int) 0~15
* **edge**: FALL(0), RISE (1), BOTH(2)
* **prior**: prior number. The smaller the number, the more priority

**Example code**

```c++
EXTI_init(GPIOC, 13, FALL, 0); //When falling edge, Pin 13 of GPIOC port does interrupt   
```



### EXTI_enable\(\)

Enable the EXTI that fits the pin number.

```c++
void EXTI_enable(uint32_t pin);
```

**Parameters**

* **Pin:**  pin number (int) 0~15

**Example code**

```c++
EXTI_enable(13); //EXTI13 is enable
```



### EXTI_disable\(\)

Disable the EXTI that fits the pin number.

```c++
void EXTI_disable(uint32_t pin);
```

**Parameters**

* **Pin:**  pin number (int) 0~15

**Example code**

```c++
EXTI_disable(13); //EXTI13 is disable
```



### is_pending_EXTI\(\)

Check the pending is ON

```c++
uint32_t is_pending_EXTI(uint32_t pin);
```

**Parameters**

* **Pin:**  pin number (int) 0~15

**Example code**

```c++
is_pending_EXTI(13); // If Pending of pin 13 is ON, retrun 1. 
```



### clear_pending_EXTI\(\)

Clear the pending's value

```c++
void clear_pending_EXTI(uint32_t pin);
```

**Parameters**

* **Pin:**  pin number (int) 0~15

**Example code**

```c++
clear_pending_EXTI(13); //clear pending of pin 13
```



### EXTIx_IRQHandler()

If EXTIx interrupt is ON,  Implement this function

```c++
void EXTIx_IRQHandler(void); x = 0,1,2 ....
```

***caution**: The function name varies depending on the pin number. and use in main.c (Refence: Spec Sheet)

**Example code**

```c++
void EXTIx_IRQHandler(void); // called by interrupt
```



### LED_toggle()

A function that toggles the LED.

```c++
void LED_toggle(GPIO_TypeDef *Port, uint32_t pin) 
```

**Parameters**

* **Port**: Port Number,  GPIOA~GPIOH
* **Pin:**  pin number (int) 0~15

**Example code**

```c++
LED_toggle(GPIOC, 13); pin 13 LED does toggle 
```



## Interrupt SysTick

### Header File

 `#include "ecSystick.h"`


```c++
#include "stm32f4xx.h"

#ifndef __ECSYSTICK_H
#define __ECSYSTICK_H

#define MCU_CLK_PLL 84000000
#define MCU_CLK_HSI 16000000


#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */
	 

void SysRick_Initialize(uint32_t msec);
void delay_ms(uint32_t msec);
void SysTick_reset(void);
uint32_t SysTick_val(void);
void SysTick_enable(void);
void SysTick_disable (void);
void SysTick_Handler(void);
 
#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
```
![image](https://user-images.githubusercontent.com/84221531/139367509-558fa483-d7ab-4edb-a8aa-9fb9b82de6bc.png)


### SysRick_Initialize()

Initialize Systick Clock.

```c++
void SysRick_Initialize(uint32_t msec);
```

**Parameters**

* **msec**: reload value + 1

  Calculate Method: Reload = ( Systick inverval(Goal)/ Clock Period ) -1

**Example code**

```c++
SysRick_Initialize(84000); // Systick clock is 1ms , assume PLL Clock 84MHz
```



### delay_ms()

Delay by the input time.

```c++
void delay_ms(uint32_t msec);
```

**Parameters**

* **msec**: Time of ms Unit

**Example code**

```c++
delay_ms(1000); // Give 1s delay
```



### SysTick_val()

Return the current Systick reload

```c++
uint32_t SysTick_val(void);
```

**Example code**

```c++
SysTick_val(void); // return current reload value
```



### SysTick_enable()

Enable Systick 

```c++
void SysTick_enable(void);
```

**Example code**

```c++
SysTick_enable(); // enable Systick interrupt.
```



### SysTick_disable()

Disable Systick 

```c++
void SysTick_disable(void);
```

**Example code**

```c++
SysTick_disable(); // disable Systick interrupt.
```



### SysTick_Handler()

If Systick interrupt is ON,  Implement this function

```c++
void SysTick_Handler(void);
```

***caution**: use in main.c 

**Example code**

```c++
SysTick_Handler(); // Called by interrupt
```



### SysTick_reset()

reset the systick val

```c++
void SysTick_reset(void);
```

**Example code**

```c++
SysTick_reset(); //reset the val value
```



## EC_API

### Header File

 `#include "EC_GPIO_API_student.h"`


```c++
#include "stm32f411xe.h"
#include "ecRCC.h"
#include "ecGPIO.h"
#include "ecEXTI.h"
#include "ecSystick.h"

#ifndef __EC_GPIO_API_H
#define __EC_GPIO_API_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */
	
class EC_Ticker
{
public:
		EC_Ticker(int reload);
		
	   void delay_ms (uint32_t  delay_t);
		
		void reset(void);
		
		uint32_t read_ms (void);
	

private:
		
};
 
#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
```



### EC_Ticker::EC_Ticker()

using the class function, It is easy for users to understand.

```c++
EC_Ticker::EC_Ticker(int reload);
```

**Parameters**

* **reload**: Systick period(Unit: ms) 

**Example code**

```c++
EC_Ticker tick(1); // "Tick" is a name that the user selectively determines.
				   // The function defined in 'class' is used as 'tick.xx'.
```



### EC_Ticker::delay_ms()

Delay by the input time.

```c++
void EC_Ticker::delay_ms(uint32_t delay)
```

**Parameters**

* **delay**:  Time of ms Unit(msec)

**Example code**

```c++
tick.delay_ms(1000) // delay 1000ms
```



### EC_Ticker::reset()

Reset the systick val

```c++
void EC_Ticker::reset(void)
```

**Example code**

```c++
tick.reset(); // Reset the val value
```



### EC_Ticker::read_ms()

Read the current time (Unit: ms)

```c++
uint32_t EC_Ticker::read_ms(void)
```

**Example code**

```c++
tick.read_ms(); // Read the current time
```



## Timer

### Header File

 `#include "ecTIM.h"`


```c++
#include "stm32f4xx.h"
#include "ecSystick.h"

#ifndef __ECTIM_H
#define __ECTIM_H

#define UP_COUNT 0
#define DOWN_COUNT 1

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

/* Timer Configuration */
     
typedef struct {
	GPIO_TypeDef *port;
	int pin;
	TIM_TypeDef *timer;
	TIM_TypeDef *timx;
	int ch;
}TIM_t;	 
	 
void TIM_init(TIM_TypeDef *timerx, uint32_t msec);
void TIM_period_us(TIM_TypeDef* timx, uint32_t usec);
void TIM_period_ms(TIM_TypeDef* timx, uint32_t msec);
void TIM_INT_init(TIM_TypeDef* timerx, uint32_t msec);
void TIM_INT_enable(TIM_TypeDef* timx);
void TIM_INT_disable(TIM_TypeDef* timx);
uint32_t is_UIF(TIM_TypeDef *TIMx);
void clear_UIF(TIM_TypeDef *TIMx);

/* Input Capture*/

// Edge Type
#define RISE 0
#define FALL 1
#define BOTH 2

//Input Capture

typedef struct{
	GPIO_TypeDef *port;
	int pin;   
	TIM_TypeDef *timer;
	int ch;  		 //int Timer Channel
	int ICnum; 		 //int IC number
} IC_t;



void ICAP_init(IC_t *ICx, GPIO_TypeDef *port, int pin);
void ICAP_setup(IC_t *ICx, int IC_number, int edge_type);
void ICAP_counter_us(IC_t *ICx, int usec);

void ICAP_pinmap(IC_t *timer_pin);

uint32_t is_pending_TIM(TIM_TypeDef *TIMx);
void clear_pending_TIM(TIM_TypeDef *TIMx);

uint32_t is_CCIF(TIM_TypeDef *TIMx, uint32_t ccNum);
void clear_CCIF(TIM_TypeDef *TIMx, uint32_t ccNum);


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
```



### TIM_init()

Turn on Timer counter : default upcounter

```c++
void TIM_init(TIM_TypeDef *timerx, uint32_t msec);
```

**Parameters**

* **TIM_TypeDef *timerx**:  TIM1~TIM11
* **msec**: Count period // Unit msec 

**Example code**

```c++
TIM_init(TIM2, 1) //1ms Timer ON.
```



### TIM_period_us()

Select TImer period: Unit(us)

```c++
void TIM_period_us(TIM_TypeDef* timx, uint32_t usec);
```

**Parameters**

* **TIM_TypeDef *timerx**:  TIM1~TIM11
* **usec**: Timer period.

**Example code**

```c++
TIM_period_us(TIM2, 10); //Setting timer 2  (10usec)  
```



### TIM_period_ms()

Select TImer period: Unit(ms)

```c++
void TIM_period_ms(TIM_TypeDef* timx, uint32_t msec);
```

**Parameters**

* **TIM_TypeDef *timerx**:  TIM1~TIM11
* **msec**: Timer period.

**Example code**

```c++
TIM_period_ms(TIM2, 10); //Setting timer 2  (10msec) 
```



### TIM_INT_init()

Interrupt by using Timer 

```c++
void TIM_INT_init(TIM_TypeDef* timerx, uint32_t msec);
```

**Parameters**

* **TIM_TypeDef *timerx**:  TIM1~TIM11
* **msec**: Timer period. Specify IRQ according to the timer.

**Example code**

```c++
TIM_INT_init(TIM2,10) // Setting interrupt every 10ms
```



### TIM_INT_enable()

TImer interrupt ON

```c++
void TIM_INT_enable(TIM_TypeDef* timx);
```

**Parameters**

* **TIM_TypeDef *timerx**:  TIM1~TIM11

**Example code**

```c++
TIM_INT_enable(TIM2) // Timer 2 interrput ON
```



### TIM_INT_disable()

TImer interrupt OFF

```c++
void TIM_INT_disable(TIM_TypeDef* timx);
```

**Parameters**

* **TIM_TypeDef *timerx**:  TIM1~TIM11

**Example code**

```c++
TIM_INT_disable(TIM2) // Timer 2 interrput OFF
```



### is_UIF()

Check UIFlag : ON , OFF

```c++
uint32_t is_UIF(TIM_TypeDef *TIMx);
```

**Parameters**

* **TIM_TypeDef *timerx**:  TIM1~TIM11

**Example code**

```c++
is_UIF(TIM2); //Drive (TIMx->SR & TIM_SR_UIF)!= 0 // delay 1000ms
```



### clear_UIF()

Clear UIFlage

```c++
void clear_UIF(TIM_TypeDef *TIMx);
```

**Parameters**

* **TIM_TypeDef *timerx**:  TIM1~TIM11

**Example code**

```c++
clear_UIF(TIM2) // clear Timer 2 UI Flag 
```



### ICAP_init()

Initialize Input capture mode, and select input port and pin

```c++
void ICAP_init(IC_t *ICx, GPIO_TypeDef *port, int pin);
```

**Parameters**

* **IC_t *ICx**:  go to structure ex) &echo ,&trig
* **GPIO_TypeDef *port**: GPIOA~GPIOH
* **pin**: PIN 0~15

**Example code**

```c++
ICAP_init(&echo,GPIOB,10); //Input Capture for echo, Input capture init PB10   	
```



### ICAP_setup()

Select input capture channel and edge detech

```c++
void ICAP_setup(IC_t *ICx, int IC_number, int edge_type);
```

**Parameters**

* **IC_t *ICx**:  go to structure ex) &echo ,&trig
* **IC_number**: IC channel number ex) Timer2 channel 3, 4
* **edge_type**: RISE(0), FALL(1), BOTH(2) // edge detech

**Example code**

```c++
ICAP_setup(&echo, 3, RISE); //when rising edge channel 3 does input capture  
```



### ICAP_counter_us()

Select input capture timer's  counter period

```c++
void ICAP_counter_us(IC_t *ICx, int usec);
```

**Parameters**

* **IC_t *ICx**:  go to structure ex) &echo ,&trig
* **usec**: timer period (Unit: usec)

**Example code**

```c++
ICAP_counter_us(&echo, 10); //counter period is 10usec
```



### is_pending_TIM()

Check UI Flag : ON , OFF

```c++
uint32_t is_pending_TIM(TIM_TypeDef *TIMx);
```

**Parameters**

* **TIM_TypeDef *timerx**:  TIM1~TIM11

**Example code**

```c++
is_pending_TIM(TIM2) // check Timer 2 UI Flag 
```



### clear_pending_TIM()

Clear UI Flage

```c++
void clear_pending_TIM(TIM_TypeDef *TIMx);
```

**Parameters**

* **TIM_TypeDef *timerx**:  TIM1~TIM11

**Example code**

```c++
clear_pending_TIM(TIM2) // clear Timer 2 UI Flag 
```



### is_CCIF()

Check capture compare interrupt Flage ON, OFF

```c++
uint32_t is_CCIF(TIM_TypeDef *TIMx, uint32_t ccNum);
```

**Parameters**

* **TIM_TypeDef *timerx**:  TIM1~TIM11
* **ccNum**: timer's channel number 1~4

**Example code**

```c++
is_CCIF(TIM2, 3); //if it becomes capture compare, TIMx_SR bit gets 1
```



### clear_CCIF()

Clear Capture Compare interrupt Flage

```c++
void clear_CCIF(TIM_TypeDef *TIMx, uint32_t ccNum);
```

**Parameters**

* **TIM_TypeDef *timerx**:  TIM1~TIM11
* **ccNum**: timer's channel ex) 1~4

**Example code**

```c++
clear_CCIF(TIM2, 3); //Clear capture compare flag of channel 3 of TIM2 
```





## PWM

### Header File

 `#include "ecPWM.h"`


```c++
/**
******************************************************************************
* @author  SSSLAB
* @Mod		 2021-8-12 by YKKIM  	
* @brief   Embedded Controller:  EC_HAL_for_student_exercise 
* 
******************************************************************************
*/

#include "stm32f411xe.h"
#include "ecGPIO.h"
#include "ecTIM.h"  			// change to ecTIM.h

#ifndef __EC_PWM_H
#define __EC_PWM_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

define LED_PIN 5

typedef struct{
   GPIO_TypeDef *port;
   int pin;
   TIM_TypeDef *timer;
   int ch;
} PWM_t;


/* PWM Configuration */
void PWM_init(PWM_t *pwm, GPIO_TypeDef *port, int pin);  
void PWM_period_ms(PWM_t *pwm,  uint32_t msec);		
void PWM_period_us(PWM_t *PWM_pin, uint32_t usec);  

void PWM_pulsewidth_ms(PWM_t *pwm, float pulse_width_ms);
void PWM_duty(PWM_t *pwm, float duty);
void PWM_pinmap(PWM_t *PWM_pin);



#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
```



### PWM_init()

Select OUTPUT(PWM) Port, Pin 

```c++
void PWM_init(PWM_t *pwm, GPIO_TypeDef *port, int pin);  
```

**Parameters**

* **PWM_t *pwm**:  determined by port and pin number. //@PWM
* **GPIO_TypeDef *port**: GPIOA~GPIOH
* **pin**: 0~15

**Example code**

```c++
PWM_init(&PWM,GPIOA,1);  // Init PA1, according to Port and pin number, determined PWM CH
```



### PWM_period_ms()

Select PWM period: Unit(ms)

```c++
void PWM_period_ms(PWM_t *pwm,  uint32_t msec);	
```

**Parameters**

* **PWM_t *pwm**:  determined by port and pin number. //@PWM
* **msec**: PWM period (ms)

**Example code**

```c++
PWM_period_ms(&PWM,20); // PWM Period 20ms   
```



### PWM_period_us()

Select PWM period: Unit(us)

```c++
void PWM_period_us(PWM_t *PWM_pin, uint32_t usec);  
```

**Parameters**

* **PWM_t *pwm**:  determined by port and pin number. //@PWM
* **usec**: PWM period (us)

**Example code**

```c++
PWM_period_us(&PWM,20); // PWM Period 20us   
```



### PWM_pulsewidth_ms()

Make the pulsewidth Unit(ms)

```c++
void PWM_pulsewidth_ms(PWM_t *pwm, float pulse_width_ms);
```

**Parameters**

* **PWM_t *pwm**:  determined by port and pin number. //@PWM
* **pulse_width_ms**: pulse width Unit(ms)  // "Pulse width" within the declared period of "PWM".

**Example code**

```c++
PWM_pulsewidth_ms(&pwm, 0.5) //if PWM period 20ms, pulse width is 0.5ms
```



### PWM_duty()

Select the duty ratio

```c++
void PWM_duty(PWM_t *pwm, float duty);
```

**Parameters**

* **PWM_t *pwm**:  determined by port and pin number. //@PWM
* **duty**: "duty ratio" that I want to use. 

**Example code**

```c++
PWM_duty(0.5/20) //  pulsewidth 0.5ms / total 20ms = duty // 2.5% 
```



### PWM_pinmap()

Select PWM channel

```c++
void PWM_pinmap(PWM_t *PWM_pin);
```

**Parameters**

* **PWM_t *PWM_pin**:  PWM struct

**Example code**

```c++
PWM_pinmap(pwm) //matching each channel
```

![image](https://user-images.githubusercontent.com/84221531/138924327-a954a858-dbab-4b97-ab68-6dff2936022b.png)

![image](https://user-images.githubusercontent.com/84221531/139368612-297ebd6d-6cf9-4eb6-9282-9df1f9ef59c4.png)

## Stepper Motor

### Header File

 `#include "ecStepper.h"`


```c++

#include "stm32f411xe.h"
#include "ecGPIO.h"
#include "ecSystick.h"
			
#ifndef __EC_STEPPER_H
#define __EC_STEPPER_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

//State mode
#define HALF 0
#define FULL 1	 
	 
/* Stepper Motor */
//stepper motor function

typedef struct{
   GPIO_TypeDef *port1;
   int pin1;
	 GPIO_TypeDef *port2;
   int pin2;
	 GPIO_TypeDef *port3;
   int pin3;
	 GPIO_TypeDef *port4;
   int pin4;
	 int _step_num;
} Stepper_t;

	 
void Stepper_init(GPIO_TypeDef* port1, int pin1, GPIO_TypeDef* port2, int pin2, GPIO_TypeDef* port3, int pin3, GPIO_TypeDef* port4, int pin4);
     
void Stepper_setSpeed (long whatSpeed, int mode);
void Stepper_step(int steps, int direction, int mode); 
void Stepper_run (int direction, int mode); 
void Stepper_stop (void);
void Stepper_pinOut (uint32_t state, int mode);
     
#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif

```



### Stepper_init()

Select stepper motor output pin and initialization

```c++
void Stepper_init(GPIO_TypeDef* port1, int pin1, GPIO_TypeDef* port2, int pin2, GPIO_TypeDef* port3, int pin3, GPIO_TypeDef* port4, int pin4);
```

**Parameters**

* **GPIO_TypeDef* port1~4**:  Select GPIO port GPIOA~GPIOH
* **pin1~4**: Select the pin number 0~15 

**Example code**

```c++
Stepper_init(GPIOB,10,GPIOB,4,GPIOB,5,GPIOB,3); // PB10, PB4, PB5, PB3 initialization 
```



### Stepper_setSpeed()

Select Rotation speed of the motor

```c++
void Stepper_setSpeed (long whatSpeed, int mode);
```

**Parameters**

* **whatSpeed**: Motor Speed (Unit: RPM)  
* **mode**: FULL(0) , HALF(1) 

**Example code**

```c++
Stepper_setSpeed (3,FULL); //3RPM, FULL mode
```



### Stepper_step()

The number of next step, 5steps => S0-> S1-> S2-> S3-> S0

```c++
void Stepper_step(int steps, int direction, int mode); 
```

**Parameters**

* **steps**: total steps numbers
* **direction**: Right cycle(0), Left cycle(1)
* **mode**: FULL(0) , HALF(1) 

**Example code**

```c++
Stepper_step(1000, Right, FULL); // FULL MODE, Right cycle, 1000steps 
```



### Stepper_run ()

NOT use in this LAB

```c++
void Stepper_run (int direction, int mode); 
```

**Parameters**

* 

**Example code**

```c++

```



### Stepper_stop()

Stop the motor.

```c++
void Stepper_stop(void);
```

**Example code**

```c++
Stepper_stop(); // stop the motor
```



### Stepper_pinOut()

It receives voltage depending on the condition.

```c++
void Stepper_pinOut(uint32_t state, int mode);
```

**Parameters**

* **state**: S0, S1, S2 ...
* **mode**:  FULL(0) , HALF(1) 

**Example code**

```c++
Stepper_pinOut(1, FULL)// FULL MODE, Output the state S1 
```



## ADC(Analog to Digital Converter)

### Header File

 `#include "ecADC.h"`


```c++
#ifndef __MY_ADC_H
#define __MY_ADC_H
#include "stm32f411xe.h"

// ADC trigmode
#define SW 0
#define TRGO 1

// ADC contmode
#define CONT 0
#define SINGLE 1

// Edge Type
#define RISE 1
#define FALL 2
#define BOTH 3

#define _DEFAULT 0

// ADC setting
void ADC_init(GPIO_TypeDef *port, int pin, int trigmode);			// trigmode : SW , TRGO
void ADC_continue(int contmode); 													// contmode : CONT, SINGLE / Operate both ADC,JADC
void ADC_TRGO(TIM_TypeDef* TIMx, uint32_t msec, int edge);
void ADC_sequence(int length, int *seq); 
void TIM_TRGO_init(TIM_TypeDef* timx, uint32_t msec);

void ADC_start(void);

uint32_t is_ADC_EOC(void);
uint32_t is_ADC_OVR(void);
void clear_ADC_OVR(void);

uint32_t ADC_read(void);
uint32_t ADC_pinmap(GPIO_TypeDef *port, int pin);
#endif

```



### ADC_init()	

Initialize GPIO port and pin, and select trig mode 

```c++
void ADC_init(GPIO_TypeDef *port, int pin, int trigmode);	
```

**Parameters**

* **GPIO_TypeDef *port**: GPIOA~GPIOH
* **pin**: 0~15
* **trigmode**: SW(0) // internal , TRGO(1) //external   

**Example code**

```c++
ADC_init(GPIOB,1,TRGO); // PB1 set by ADC mode and it will start by TRGO trig
```



### ADC_TRGO()

Setting TRGO mode 

```c++
void ADC_TRGO(TIM_TypeDef* TIMx, uint32_t msec, int edge);
```

**Parameters**

* **TIM_TypeDef* TIMx**: TIMER number: ex) TIM2, TIM3
* **msec**:  timer period
* **edge**: RISE(0), FALL(1), BOTH(2) //edge detech

**Example code**

```c++
ADC_TRGO(TIM2,1,RISE);	// HW trig mode settting, TIM2(1msec) 
```



### ADC_continue()

Choose repetition: Continuous, Single

```c++
void ADC_continue(int contmode); 	
```

**Parameters**

* **contmode**: CONT(0), SINGLE(1)  

**Example code**

```c++
ADC_continue(SINGLE); //Single conversion mode ON, it is used TRGO
```



### ADC_sequence()

Set the Channel sequence

```c++
void ADC_sequence(int length, int *seq); 
```

**Parameters**

* **length**: The number of channels ex) if use ch1, ch2 ..... length = 2
* ***seq**:  what i have to do first ex) seq[2] = {8 , 9} <-- channel 8 is fisrt 

**Example code**

```c++
ADC_sequence(2,seq); //Total channel's number is 2. The priority is to have a low index number.
```



### TIM_TRGO_init()

Set the TRGO timer 

```c++
void TIM_TRGO_init(TIM_TypeDef* timx, uint32_t msec);
```

**Parameters**

* **TIM_TypeDef* timx**: Timer number ex) TIM2, TIM3
* **msec**: timer clock

**Example code**

```c++
TIM_TRGO_init(TIM2,1) // TRGO timer clock ON. TIM2 1msec
```



### ADC_start()

ADC is started 

```c++
void ADC_start(void);
```

**Example code**

```c++
ADC_start(); //ADC start 
```



### is_ADC_EOC()

If conversion has been finished, this bit is setted as 1. Then, it is cleared by software. 

```c++
uint32_t is_ADC_EOC(void);
```

**Example code**

```c++
is_ADC_EOC(); //Check if Conversion is finished
```



### is_ADC_OVR()

this bit is set by hardware when data are lost.

```c++
uint32_t is_ADC_OVR(void);
```

**Example code**

```c++
is_ADC_OVR(); // check the overrap.
```



### clear_ADC_OVR()

Clear overrap bit

```c++
void clear_ADC_OVR(void);
```

**Example code**

```c++
clear_ADC_OVR(); // clear the overrap.
```



### ADC_read()

Read current data.

```c++
uint32_t ADC_read(void);
```

**Example code**

```c++
ADC_read(); //current analog data.
```



### ADC_pinmap()

Match the Channel each port and pin

```c++
uint32_t ADC_pinmap(GPIO_TypeDef *port, int pin);
```

**Parameters**

* **GPIO_TypeDef *port**: GPIOA~GPIOH
* **pin**: pin number 0~15 

**Example code**

```c++
ADC_pinmap(GPIOB, 0); // it match channel of port and pin
```

## UART

 `#include "ecUART.h"`


```c++
#ifndef __EC_UART_H
#define __EC_UART_H

#include "stm32f411xe.h"
#include <stdio.h>

/************************************************/
//  USART1  TX : PA9 , PB6    RX : PA10 , PB3
// 	USART2  TX : PA2  				RX : PA3
// 	USART6	TX : PA11, PC6  	RX : PA12, PC7
//*******************************************/

void UART2_init();
void USART_write(USART_TypeDef * USARTx, uint8_t *buffer, uint32_t nBytes);
void USART_delay(uint32_t us);
void USART_init(USART_TypeDef* USARTx, uint32_t baud);
void USART_begin(USART_TypeDef* USARTx, GPIO_TypeDef* GPIO_TX, int pinTX,GPIO_TypeDef* GPIO_RX, int pinRX, int baud);
uint8_t USART_getc(USART_TypeDef *USARTx);
uint32_t is_USART_RXNE(USART_TypeDef * USARTx);


#endif
```



### UART2_init()

Initialize only UART2. when we communicate with Tera-Term, this function is necessary.

```c++
void UART2_init();
```

**Example code**

```c++
UART2_init(); // Start UART2
```



### USART_write()

this function for writing in tera-term.

```c++
void USART_write(USART_TypeDef * USARTx, uint8_t *buffer, uint32_t nBytes);
```

**Parameters**

* **USART_TypeDef * USARTx**: USART1, USART2 ...
* **buffer**: saved data
* **nBytes**: the number of Character

**Example code**

```c++
USART_write(USART1, &muc2Data, 1); //write down the muc2Data's value in tera-term
```



### USART_delay()

We need waiting time so Use this function. This function works similar to delay_ms()

```c++
void USART_delay(uint32_t us);
```

**Parameters**

* **us**: delay time (Unit: usec)

**Example code**

```c++
USART_delay(300); delay 300 usec
```



### USART_init()

in this funcition, Specify the pin according to Uart's number.

```c++
void USART_init(USART_TypeDef* USARTx, uint32_t baud);
```

**Parameters**

* **USART_TypeDef* USARTx**: USART1, USART2 ...
* **baud**: communication rate //9600 .. 

**Example code**

```c++
  USART_init(USART2, 38400); //ON USART2 and baud-rate 38400
```



### USART_begin()

Turn on the USART communication. this function usually use in the USART_init()

```c++
void USART_begin(USART_TypeDef* USARTx, GPIO_TypeDef* GPIO_TX, int pinTX,int GPIO_TypeDef* GPIO_RX, int pinRX, int baud);
```

**Parameters**

* **USART_TypeDef* USARTx**: USART1, USART2 ...

* **GPIO_TypeDef* GPIO_TX**: GPIOA~GPIOH 

* **pinTX**: 0~15

* **pinTX,GPIO_TypeDef* GPIO_RX**: GPIOA~GPIOH

* **pinRX**: 0~15

  *Check the UASRAT Pin map
  
  ![image](https://user-images.githubusercontent.com/84221531/143729060-f5b9680a-3e6d-4790-8499-b547cd364926.png)

**Example code**

```c++
  USART_begin(USART1, GPIOA,9,GPIOA,10, 9600); //USART1 baud: 9600 
```



### USART_getc()

Get the Character value by user

```c++
uint8_t USART_getc(USART_TypeDef *USARTx);
```

**Parameters**

* **USART_TypeDef* USARTx**: USART1, USART2 ...

**Example code**

```c++
USART_getc(USART1); //get Usart1's value
```



### is_USART_RXNE()

Make sure you are ready to read the value.

```c++
uint32_t is_USART_RXNE(USART_TypeDef * USARTx);
```

**Parameters**

* **USART_TypeDef* USARTx**: USART1, USART2 ...
![image](https://user-images.githubusercontent.com/84221531/143729012-2bba2191-e665-4c2b-9f68-d8144016719c.png)


**Example code**

```
is_USART_RXNE(USART1); 
```

![image](https://user-images.githubusercontent.com/84221531/143729012-2bba2191-e665-4c2b-9f68-d8144016719c.png)



Class or Header name

### Function Name

```text

```

**Parameters**

* p1
* p2

**Example code**

```text

```
