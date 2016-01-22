#include <hal/stm32cubef4/gpio.hpp>
//#include <hal/stm32cubef4/uart.hpp>
#define MSG "hello world!\r\n"

//Baudrate 115200
//Parity NONE
//StopBit ONE_BIT
//WordSize 8

using namespace HAL::STM32CubeF4;

constexpr auto& LED1 = G10;

int main(int, char**) {

int i;
///////////////////////////////////////////:
	GPIO::Settings settings_led;
  settings_led.mode = GPIO::Mode::OUTPUT;
  settings_led.output_mode = GPIO::OutputMode::PUSH_PULL;
  settings_led.pull = GPIO::Pull::DOWN;

  GPIO::init(LED1, settings_led);
  /////////////////////////////////////////////  
  /*UART::Settings settings;
  settings.baudrate=115200;
  settings.parity=UART::Parity::NONE;
  settings.stop_bit=UART::StopBit::ONE_BIT;
  settings.word_size=8;
  UART::init(SERIAL_USART1,A9,A10,settings);//UART vers raspberry*/
  while(1)
  {
  	for(i=0;i<1000000;i++);
  	GPIO::toggle(LED1);
  	//UART::putChar(SERIAL_USART1,'A');
  	//UART::write(SERIAL_USART1, (u8*)MSG, sizeof(MSG));
  }
}
