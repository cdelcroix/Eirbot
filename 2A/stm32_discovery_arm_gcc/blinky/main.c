

//#define USE_STDPERIPH_DRIVER
#include "stm32f4xx.h"
/*#include <hal/gpio_interface.hpp>
#include <hal/pin_interface.hpp>
#include <hal/macros_define.hpp>*/
#include "init.h"
#include "platform.h"
/*
  struct GPIO:HAL::GPIO_DriverInterface<u8> {//u8 -> 1 octect car 8 pins par port
    using Parent = GPIO_DriverInterface<u8>;

    //! \brief The GPIO Pin Mode
    struct Mode:Parent::Mode {
      AVERSIVE_AVAILABLE_ELEMENT(UNDEFINED);
      AVERSIVE_UNAVAILABLE_ELEMENT(INPUT);
      AVERSIVE_AVAILABLE_ELEMENT(OUTPUT);
    };
    
    //! \brief The GPIO Pin Output Mode
    struct OutputMode:Parent::OutputMode {
      AVERSIVE_AVAILABLE_ELEMENT(UNDEFINED);
      AVERSIVE_UNAVAILABLE_ELEMENT(PUSH_PULL);
      AVERSIVE_UNAVAILABLE_ELEMENT(OPEN_DRAIN);
    };

    //! \brief The GPIO Pin Pull Policy
    struct Pull:Parent::Pull {
      AVERSIVE_AVAILABLE_ELEMENT(UNDEFINED);
      AVERSIVE_UNAVAILABLE_ELEMENT(UP);
      AVERSIVE_UNAVAILABLE_ELEMENT(DOWN);
    };
  };

  struct PIN:HAL::PIN_DriverInterface<u8> {
    
    //! \name Settings
    //! @{
    template<typename GPIOType, typename PinType, typename Settings>
    static void init(GPIOType gpio, PinType pin, const Settings& settings){
    static constexpr GPIO_TypeDef* _GPIO[]={GPIOA,GPIOB,GPIOC,GPIOD};
long mask=0b11<<(2*pin);
if(settings.mode==GPIO::Mode::OUTPUT){
long cfg=0b01<<(2*pin);
_GPIO[gpio]->MODER&=~mask;//set 2 bits at 0
_GPIO[gpio]->MODER|=cfg;


}
}
    //! @}

    //! \name Value
    //! @{
    template<typename GPIOType, typename PinType>
    static bool getValue(GPIOType gpio, PinType pin);

    template<typename GPIOType, typename PinType, typename ValueType>
    static void setValue(GPIOType gpio, PinType  pin, ValueType value);

    template<typename GPIOType, typename PinType>
    static void toggle(GPIOType gpio, PinType pin);
    //! @}
  };

//Quick hack, approximately 1ms delay
void ms_delay(int ms)
{
   while (ms-- > 0) {
      volatile int x=5971;
      while (x-- > 0)
         __asm("nop");
   }
}

*/

//Flash orange LED at about 1hz
int main(void)
{



/*    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;  // enable the clock to GPIOD
    __asm("dsb");                         // stall instruction pipeline, until instruction completes, as
                                          //    per Errata 2.1.13, "Delay after an RCC peripheral clock enabling"

GPIO::Settings s;
s.mode=GPIO::Mode::OUTPUT;

PIN::init(3,12,s);
PIN::init(3,13,s);*/

init_lidar();
    //GPIOD->MODER = (1 << 24);             // set pin 12 (ie LD4 green) to be general purpose output
    /*GPIOD->MODER = GPIOD->MODER & (1 << 26); 
            // set pin 13 (ie LD3 orange) to be general purpose output
    GPIOD->MODER = GPIOD->MODER &(1 << 28);             // set pin 14 (ie LD5 red) to be general purpose output
    GPIOD->MODER = GPIOD->MODER &(1 << 30);             // set pin 15 (ie LD6 blue) to be general purpose output*/

    for (;;) {
// USART_SendData(USART1, 'b');
 //while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);

      /* ms_delay(500);//avant 500
       GPIOD->ODR ^= (1 << 12);  
ms_delay(500);//avant 500
       GPIOD->ODR ^= (1 << 13); 
ms_delay(500);//avant 500
*/
         // Toggle the pin 
/*
       ms_delay(500);//avant 500
       GPIOD->ODR ^= (1 << 13);           // Toggle the pin 
       ms_delay(500);//avant 500
       GPIOD->ODR ^= (1 << 14);           // Toggle the pin 
       ms_delay(500);//avant 500
       GPIOD->ODR ^= (1 << 15);           // Toggle the pin 
       ms_delay(500);//avant 500
*/
    }
}
