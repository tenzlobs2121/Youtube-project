/**
**************************************************************************
* @file     at32f415_board.h
* @version  v2.0.7
* @date     2022-08-16
* @brief    header file for at-start board. set of firmware functions to
*           manage leds and push-button. initialize delay function.
**************************************************************************
*                       Copyright notice & Disclaimer
*
* The software Board Support Package (BSP) that is made available to
* download from Artery official website is the copyrighted work of Artery.
* Artery authorizes customers to use, copy, and distribute the BSP
* software and its related documentation for the purpose of design and
* development in conjunction with Artery microcontrollers. Use of the
* software is governed by this copyright notice and the following disclaimer.
*
* THIS SOFTWARE IS PROVIDED ON "AS IS" BASIS WITHOUT WARRANTIES,
* GUARANTEES OR REPRESENTATIONS OF ANY KIND. ARTERY EXPRESSLY DISCLAIMS,
* TO THE FULLEST EXTENT PERMITTED BY LAW, ALL EXPRESS, IMPLIED OR
* STATUTORY OR OTHER WARRANTIES, GUARANTEES OR REPRESENTATIONS,
* INCLUDING BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE, OR NON-INFRINGEMENT.
*
**************************************************************************
*/

#ifndef __AT32F415_BOARD_H
#define __AT32F415_BOARD_H

#ifdef __cplusplus
extern "C" {
#endif
  
#include "stdio.h"
#include "at32f415.h"
#include "i2c_application.h"
#include "bq76952.h"
#include "FreeRTOS.h"
#include "task.h"
  
  
  /** @addtogroup AT32F415_board
  * @{
  */
  
  /** @addtogroup BOARD
  * @{
  */
  
  /** @defgroup BOARD_pins_definition
  * @{
  */
  
#define NOAH_F415_V1
  
  
#if !defined (NOAH_F415_V1)
#error "please select first the board at-start device used in your application (in at32f415_Noah_board.h file)"
#endif
  
  /******************** define led ********************/
  typedef enum
  {
    LED1                                   = 0,
    LED2                                   = 1,
    LED3                                   = 2
  } led_type;
  
#define LED_NUM                          3
  
#if defined (NOAH_F415_V1)
#define LED1_PIN                         GPIO_PINS_4
#define LED1_GPIO                        GPIOA
#define LED1_GPIO_CRM_CLK                CRM_GPIOA_PERIPH_CLOCK
  
#define LED2_PIN                         GPIO_PINS_5
#define LED2_GPIO                        GPIOA
#define LED2_GPIO_CRM_CLK                CRM_GPIOA_PERIPH_CLOCK
  
#define LED3_PIN                         GPIO_PINS_6
#define LED3_GPIO                        GPIOA
#define LED3_GPIO_CRM_CLK                CRM_GPIOA_PERIPH_CLOCK
#endif
  
  /******************** define std output ********************/
  typedef enum
  {
    EN_12V                                   = 0,
    EN_3V3                                   = 1,
    HV_BUCK_CO                               = 2,
    CONT_DRV                                 = 3,
    NTC_PWR_EN                               = 4,
    TELE_PWR_EN                              = 5,
    CAN_PHY_MODE                             = 6,
    EEPROM_WP                                = 7
  } std_out_type;
  
#define OUT_NUM                          8
  
  
#define EN_12V_PIN                         GPIO_PINS_13
#define EN_12V_GPIO                        GPIOC
#define EN_12V_GPIO_CRM_CLK                CRM_GPIOC_PERIPH_CLOCK
  
#define EN_3V3_PIN                         GPIO_PINS_1
#define EN_3V3_GPIO                        GPIOA
#define EN_3V3_GPIO_CRM_CLK                CRM_GPIOA_PERIPH_CLOCK
  
#define HV_BUCK_CO_PIN                         GPIO_PINS_0
#define HV_BUCK_CO_GPIO                        GPIOB
#define HV_BUCK_CO_GPIO_CRM_CLK                CRM_GPIOB_PERIPH_CLOCK
  
#define CONT_DRV_PIN                       GPIO_PINS_11
#define CONT_DRV_GPIO                        GPIOB
#define CONT_DRV_GPIO_CRM_CLK                CRM_GPIOB_PERIPH_CLOCK
  
#define NTC_PWR_EN_PIN                         GPIO_PINS_12
#define NTC_PWR_EN_GPIO                        GPIOB
#define NTC_PWR_EN_GPIO_CRM_CLK                CRM_GPIOB_PERIPH_CLOCK
  
#define TELE_PWR_EN_PIN                         GPIO_PINS_15
#define TELE_PWR_EN_GPIO                        GPIOB
#define TELE_PWR_EN_GPIO_CRM_CLK                CRM_GPIOB_PERIPH_CLOCK 
  
#define CAN_PHY_MODE_PIN                         GPIO_PINS_11
#define CAN_PHY_MODE_GPIO                        GPIOA
#define CAN_PHY_MODE_GPIO_CRM_CLK                CRM_GPIOA_PERIPH_CLOCK
  
#define EEPROM_WP_PIN                         GPIO_PINS_7
#define EEPROM_WP_GPIO                        GPIOB
#define EEPROM_WP_GPIO_CRM_CLK                CRM_GPIOB_PERIPH_CLOCK  
  
  /******************** define std output ********************/
  typedef enum
  {
    AFE_BOTHOFF                              = 0,
    AFE_RST_SHUT                             = 1,
    AFE_nWAKE                                = 2
  } afe_out_type;
  
#define AFEO_NUM                               3
  
  
#define AFE_BOTHOFF_PIN                         GPIO_PINS_7
#define AFE_BOTHOFF_GPIO                        GPIOA
#define AFE_BOTHOFF_GPIO_CRM_CLK                CRM_GPIOA_PERIPH_CLOCK
  
#define AFE_RST_SHUT_PIN                         GPIO_PINS_1
#define AFE_RST_SHUT_GPIO                        GPIOB
#define AFE_RST_SHUT_GPIO_CRM_CLK                CRM_GPIOB_PERIPH_CLOCK
  
#define AFE_nWAKE_PIN                         GPIO_PINS_8
#define AFE_nWAKE_GPIO                        GPIOA
#define AFE_nWAKE_GPIO_CRM_CLK                CRM_GPIOA_PERIPH_CLOCK
  
  /**************** define debug print uart ******************/
  
#define PRINT_UART                       USART1
#define PRINT_UART_CRM_CLK               CRM_USART1_PERIPH_CLOCK
#define PRINT_UART_TX_PIN                GPIO_PINS_9
#define PRINT_UART_TX_GPIO               GPIOA
#define PRINT_UART_TX_GPIO_CRM_CLK       CRM_GPIOA_PERIPH_CLOCK
  
  /**************** define Telematics  uart2 ******************/
  
#define TELE_UART                       USART2
#define TELE_UART_CRM_CLK               CRM_USART2_PERIPH_CLOCK
#define TELE_UART_TX_PIN                GPIO_PINS_2
#define TELE_UART_TX_GPIO               GPIOA
#define TELE_UART_TX_GPIO_CRM_CLK       CRM_GPIOA_PERIPH_CLOCK  
  
  
  /******************* define I2C *******************/
  
#define I2C_TIMEOUT                      0x00FFFFFF
  
#define I2Cx_SPEED                       400000
#define I2Cx_ADDRESS                     0x87
  
#define I2Cx_PORT                        I2C1
#define I2Cx_CLK                         CRM_I2C1_PERIPH_CLOCK
  
#define I2Cx_SCL_PIN                     GPIO_PINS_6
#define I2Cx_SCL_GPIO_PORT               GPIOF
#define I2Cx_SCL_GPIO_CLK                CRM_GPIOF_PERIPH_CLOCK
  
#define I2Cx_SDA_PIN                     GPIO_PINS_7
#define I2Cx_SDA_GPIO_PORT               GPIOF
#define I2Cx_SDA_GPIO_CLK                CRM_GPIOF_PERIPH_CLOCK
  
#define I2Cx_EVT_IRQn                    I2C1_EVT_IRQn
#define I2Cx_ERR_IRQn                    I2C1_ERR_IRQn
  
#define BUF_SIZE                         8
#define MASTER_BOARD  
  
  
  
  
  /******************* define button *******************/
  typedef enum
  {
    USER_BUTTON                            = 0,
    NO_BUTTON                              = 1
  } button_type;
  
#define USER_BUTTON_PIN                  GPIO_PINS_0
#define USER_BUTTON_PORT                 GPIOA
#define USER_BUTTON_CRM_CLK              CRM_GPIOA_PERIPH_CLOCK
  
  /******************* AFE BQ76952 definations *******************/
  
  
  
#define DEV_ADDR  0x8<<1  // BQ769x2 address is 0x10 including R/W bit or 0x8 as 7-bit address
#define CRC_Mode 0 // 0 for disabled, 1 for enabled
#define MAX_BUFFER_SIZE 10
#define R 0 // Read; Used in DirectCommands and Subcommands functions
#define W 1 // Write; Used in DirectCommands and Subcommands functions
#define W2 2 // Write data with two bytes; Used in Subcommands function
  //user addition
#define DEV_ADDR_WRITE         0x10   // BQ769x2 address is 0x10 including R/W bit or 0x8 as 7-bit address
#define DEV_ADDR_READ          0x10
  
  
  
  //uint8_t RX_data [2] = {0x00, 0x00}; // used in several functions to store data read from BQ769x2
  //uint8_t RX_32Byte [32] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  //0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  ////used in Subcommands read function
  
  
  
  /**
  * @}
  */
  
  /** @defgroup BOARD_exported_functions
  * @{
  */
  
  /******************** functions ********************/
  void Noah_board_init(void);
  
  /* led operation function */
  void at32_led_init(led_type led);
  void at32_led_on(led_type led);
  void at32_led_off(led_type led);
  void at32_led_toggle(led_type led);
  
  /* std out operation function */
  void at32_opin_init(std_out_type opin);
  void at32_opin_on(std_out_type opin);
  void at32_opin_off(std_out_type opin);
  
  
  /* AFE out operation function */
  void at32_afeo_init(afe_out_type afeo);
  void at32_afeo_on(afe_out_type afeo);
  void at32_afeo_off(afe_out_type afeo);
  
  
  /* button operation function */
  void at32_button_init(void);
  button_type at32_button_press(void);
  uint8_t at32_button_state(void);
  
  /* delay function */
  void delay_init(void);
  void delay_us(uint32_t nus);
  void delay_ms(uint16_t nms);
  void delay_sec(uint16_t sec);
  
  /* printf uart init function */
  void uart_print_init(uint32_t baudrate);
  
  /* telematics uart2 init function */
  void uart_tele_init(uint32_t baudrate2);
  
  /* I2C bus init function */
  
  void error_handler(uint32_t error_code);
  void i2c_lowlevel_init(i2c_handle_type* hi2c);
  
  /* AFE BQ76952  functions */
  unsigned char Checksum(unsigned char *ptr, unsigned char len);
  void CommandSubcommands(uint16_t command);
  void DirectCommands(uint8_t command, uint16_t data, uint8_t type);
  int I2C_ReadReg(uint8_t reg_addr, uint8_t *reg_data, uint8_t count);
  void I2C_WriteReg(uint8_t reg_addr, uint8_t *reg_data, uint8_t count);
  void BQ769x2_SetRegister(uint16_t reg_addr, uint32_t reg_data, uint8_t datalen);
  void BQ769x2_Init();
  unsigned char CRC8(unsigned char *ptr, unsigned char len);
  void CopyArray(uint8_t *source, uint8_t *dest, uint8_t count);
  uint16_t BQ769x2_ReadAlarmStatus();
  float BQ769x2_ReadTemperature(uint8_t command);
  uint16_t BQ769x2_ReadAlarmStatus();
  uint16_t BQ769x2_ReadVoltage(uint8_t command);
  void BQ769x2_ReadAllVoltages();
  uint16_t BQ769x2_ReadCurrent();
  
  
  /* CAN stuff   */
  void can_configuration(void);
  void can_gpio_config(void);
  void can_transmit_data(void);
  void CAN1_RX0_IRQHandler(void);
  void CAN1_SE_IRQHandler(void);
  
  /* PWM on GPIOB pin 6 and GPIOA pin 6 (LED 3)*/
  void digital_PWM(int PWM_PIN,volatile uint32_t frequency,uint32_t duty_cycle);
    
  /* Interrupt configuration GPIOA pin 15*/
  void exint_line0_config_GPIOA_15(void);
  
  /* Interrupt configuration GPIOB pin 3*/
  void exint_line1_config_GPIOB_3(void);
  
  /*Interrupt handler for GPIOA pin 15*/
 // void EXINT0_IRQHandler_For_GPIOA_15(void);
  
    /*Interrupt handler for GPIOB pin 3*/
  //void EXINT0_IRQHandler_For_GPIOB_3(void);
  
  
  /* FreeRTOS Fuction Declaration.*/
  void led2_task_function(void *pvParameters);
  void CAN_Communication_task_function(void *pvParameters);
  void BQ769x2_ReadAlarmStatus_task_function(void *pvParameters);
  void PWM_task_funtion(void *pvParameters);
    
    
    /**
    * @}
    */
    
    /**
    * @}
    */
    
    /**
    * @}
    */
    
#ifdef __cplusplus
}
#endif

#endif

