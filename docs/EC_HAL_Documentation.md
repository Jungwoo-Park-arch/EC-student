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
#define      PA5            5
#define      PA6            6
#define      PA7            7
#define      PB6            6

#define      BUTTON_PIN     13

// Setting
#define      HIGH           1 
#define      LOW            0

// MODE Setting
#define      INPUT        	0
#define      OUTPUT       	1 
#define      ALTERNATE    	2
#define      ANALOG       	3 

// Output type Setting
#define      PUSH_PULL 		0
#define      OPEN_DRAIN		1

// Output speed Setting
#define      LOW_SPEED      0
#define      MEDIUM_SPEED   1 
#define      FAST_SPEED     2 
#define      HIGH_SPEED     3 

// Output PUPD Setting
#define      PULL_UP       1 
#define      PULL_DOWN      2 
#define      RESERVED_2     3 



void GPIO_init(GPIO_TypeDef *Port, int pin, int mode);
void GPIO_write(GPIO_TypeDef *Port, int pin, int Output);
int  GPIO_read(GPIO_TypeDef *Port, int pin);
void GPIO_mode(GPIO_TypeDef* Port, int pin, int mode);
void GPIO_ospeed(GPIO_TypeDef* Port, int pin, int speed);
void GPIO_otype(GPIO_TypeDef* Port, int pin, int type);
void GPIO_pupdr(GPIO_TypeDef* Port, int pin, int pudr);

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



------



## Class or Header name

### Function Name

```text

```

**Parameters**

* p1
* p2

**Example code**

```text

```
