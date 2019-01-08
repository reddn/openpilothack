// void setup_timer4() {
//   // setup
//   TIM4->PSC = 48-1;          // tick on 1 us
//   TIM4->CR1 = TIM_CR1_CEN;   // enable
//   TIM4->ARR = 30-1;          // 33.3 kbps
//
//   // in case it's disabled
//   NVIC_EnableIRQ(TIM4_IRQn);
//
//   // run the interrupt
//   TIM4->DIER = TIM_DIER_UIE; // update interrupt
//   TIM4->SR = 0;
// }

uint8_t volatile hondaSerialBuffer[8] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
int8_t volatile hondaSerialBufferi = -1;
int8_t volatile hondaSerialBufferLastSenti = -1;
bool volatile hondaSerial_lkas_active = false;
uint8_t hondaSerial_lkas_off_array[][4] =  {{0x20,0x80,0xc0,0xa0},{0x00,0x80,0xc0,0xc0}};
uint8_t volatile hondaSerialCounterbit = 0x00;
uint8_t volatile hondaSerial_error_count = 0x00;
uint8_t hondaSerialFrame[2][4];
bool volatile hondaSerial_func_is_running = false;

void setup_hondaSerialSteering() {
  // setup
  uart_ring *ur = &lin1_ring;
  ur->uart->CR1 &= ~USART_CR1_PS;
  ur->uart->CR1 |= USART_CR1_PCE | USART_CR1_M;
  uart_set_baud(ur->uart, 300);
  TIM7->PSC = 2048;          // tick on 1 us 2949-1
  TIM7->CR1 = TIM_CR1_CEN;   // enable
  TIM7->ARR = 88;          // 89 -1  87.73805Hz    looking for 87.72

  // in case it's disabled
  NVIC_EnableIRQ(TIM7_IRQn);

  TIM7->DIER = TIM_DIER_UIE; // update interrupt
  TIM7->SR = 0;
}

void stop_hondaSerialStering(){
  // TIM7->CR1 = TIM_CR1_CN; // disable timer_init   //FIX ME. how to disable??
}

void hondaSerial_sendLKASOffArray(){
  for (uint8_t i = 0; i < 4; i++) while (!putc(&lin1_ring, hondaSerial_lkas_off_array[hondaSerialCounterbit][i]));
  hondaSerialCounterbit = hondaSerialCounterbit ? 0x00 : 0x01;
}

uint8_t chksm(){
	uint16_t local = hondaSerialFrame[hondaSerialCounterbit][0] + hondaSerialFrame[hondaSerialCounterbit][1] +
                    hondaSerialFrame[hondaSerialCounterbit][2];
	local = local % 512;
	local = 512 - local;
	return (uint8_t)(local % 256);
}

void hondaSerial_sendLKASOnArray(){
  hondaSerialFrame[hondaSerialCounterbit][0] = (hondaSerialBuffer[hondaSerialBufferi] & 0xF) | (hondaSerialCounterbit << 0x5);
  hondaSerialFrame[hondaSerialCounterbit][1] = ((hondaSerialBuffer[hondaSerialBufferi] & 0xF) & 0x1F) | 0xA0;
  hondaSerialFrame[hondaSerialCounterbit][2] = 0x80;
  hondaSerialFrame[hondaSerialCounterbit][3] = chksm();
  for (uint8_t i = 0; i < 4; i++) while (!putc(&lin1_ring, hondaSerialFrame[hondaSerialCounterbit][i]));
  hondaSerialBufferLastSenti = hondaSerialBufferi;
  hondaSerialCounterbit = hondaSerialCounterbit ? 0x00 : 0x01;
}

void hondaSerial_setLKASValuesOff(){
  hondaSerial_lkas_active = false;
  hondaSerialBufferi = 0xFF;
  hondaSerialBufferLastSenti = 0xFF;
}

void TIM7_IRQHandler(void){ // propbably need to use switches, 3 ifs is ugly

  if(hondaSerial_lkas_active){  //func_is_running is true if function is being ran. this should not happen.
    if(hondaSerial_func_is_running){
      hondaSerial_setLKASValuesOff();  //lock out it running, set everything to off and return
      return;
    }
    hondaSerial_func_is_running = true;
    //create the 4 byte message and send it. if nothign in the buffer, errorCOunt +1, and resent last,
    // if errorcount > 0 and nothign in buffer, turn lkas off -- if error count already  > 0, turn off lkas
    if(hondaSerialBufferi == -1 || hondaSerial_error_count > 0) {
      hondaSerial_setLKASValuesOff();
      hondaSerial_sendLKASOffArray();
      return;
    }
    //hondaSerialBufferi has nothing and its time to send again, if something was last sent, send it again, and errorcount ++,
    //Fix below
    if(hondaSerialBufferLastSenti == hondaSerialBufferi ){ // error count is not > 0, sending last sent again w updated counterbit and checksum
      hondaSerial_error_count++;

    }else { // nothing was sent before but lkas is active , honestly this should never happen, but you never know...lkas off  and send
      hondaSerial_setLKASValuesOff();
      hondaSerial_sendLKASOffArray();
      return;
    }
    hondaSerial_sendLKASOnArray();
    hondaSerial_func_is_running = false;
  } else{  // if lkas not active
    //send lkas off fixed signals. array is in order of counterbit as index
    hondaSerial_sendLKASOffArray();
  }

}

//just to see below..
// uint8_t hondaSerialBuffer[6] = {0x00,0x00,0x00,0x00,0x00,0x00};
// int8_t hondaSerialBufferi = -1;
// bool hondaSerial_lkas_active = false;
