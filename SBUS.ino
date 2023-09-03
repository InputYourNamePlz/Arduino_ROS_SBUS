// SBUS to PWM
// inverted SBUS signal into RX1(Serial1)
// all 18 channels out on D2-D13, A0-A5

// this is the longest that six PWM pulses should take to output (in half-microseconds)
#define MAX_GROUP_TIME 24138
// if SBUS frames are received faster than this

uint8_t frame[24]; // SBUS received bytes. 1st byte of 25-byte frame is always 0x0F, and is not stored
uint16_t dTimes[6]; // times (in half-microsecond units) for the edges of the first 6 PWM channels, to be output on Port D (pins D2-D7)
uint16_t bTimes[6]; // similarly for the next 6 PWM channels, output on Port B (pins D8-D13) 
uint16_t cTimes[6]; // and for the last 6 PWM channels, output on Port C (pins A0-A5) 
uint16_t maxTime = 18000; // greatest of { dTimes[5], bTimes[5], cTimes[5] } after this time the pulse output loop can be terminated; 


uint16_t pwmValues[18]; // 배열 형태로 PWM 값을 저장

void setup() {
  Serial.begin(100000, SERIAL_8E2); // SBUS runs at 100kBaud 8E2: 25-byte frames, without gaps, take 3000us
  PORTD = 0b00000000; //  1st 6 PWM channel outputs on D2-D7
  DDRD  = 0b11111100;
  PORTB = 0b00000000; // next 6 PWM channel outputs on D8-D13 
  DDRB  = 0b00111111;
  PORTC = 0b00000000; // last 6 PWM channel outputs on A0-A5
  DDRC  = 0b00111111;

  // using 16-bit Timer1 to time simple intervals in half-microseconds for outputting the servo pulses
  TCCR1A = 0x00;
  TCCR1B = 0x02;
}

void loop() {
  sbus();
}


void sbus() {
  while(Serial.available()) {
    // try to receive a 25-byte frame that begins with 0x0F and ends with 0x00
    // scan until an 0x0F byte is received, or the receive buffer is empty
    int b = Serial.read();
    if (b != 0x0F) {
      continue;
    }
    frame[23] = 0xFF; // a good received frame overwrites this with 0x00
    // now attempt to receive 24 bytes
    int i = 0;
    while (i < 24) {
      i += Serial.readBytes(frame + i, min(Serial.available(), 24 - i));
    }
    if (i == 24 && frame[23] == 0x000) {
      // 각 채널의 PWM 값을 배열에 저장
      for (uint8_t ch = 0; ch < 18; ch++) {
        pwmValues[ch] = channel(ch) + 1976; // 수정 필요한 부분
      }
      // outputting pulses takes 12.07ms with an extra 0.1ms calculation time.
      // If SBUS frames arrive faster then that the next one will already have started now.
      // empty the serial receive buffer so that the output pulses are synced with the SBUS frames.
      // for example, if the SBUS frames arrive at 11ms intervals, then every other received frame will be discarded, and the PWM frequency will be 22ms
      while (Serial.available()) {
         Serial.read();
      }
      break;
    }
  }
}


uint16_t channel(uint8_t ch) { // extract 11-bit channel[ch] value from frame. ch 0-15
  uint8_t k = ch > 7 ? 11 : 0; // offset into frame array: 0 for channels 0-7, 11 for channels 8-15
  switch (ch % 8) { // pattern repeats (except for k-offset) after 8 channels
    case 0:
      return (uint16_t)frame[0+k] | ((((uint16_t)frame[1+k]) & 0x07) << 8);
    case 1:
      return ((uint16_t)(frame[1+k] & 0xF8) >> 3) | ((((uint16_t)frame[2+k]) & 0x3F) << 5);
    case 2:
      return ((uint16_t)(frame[2+k] & 0xC0) >> 6) | ((((uint16_t)frame[3+k])) << 2) | ((((uint16_t)frame[4+k]) & 0x01) << 10);
    case 3:
      return ((uint16_t)(frame[4+k] & 0xFE) >> 1) | ((((uint16_t)frame[5+k]) & 0x0F) << 7);
    case 4:
      return ((uint16_t)(frame[5+k] & 0xF0) >> 4) | ((((uint16_t)frame[6+k]) & 0x7F) << 4);
    case 5:
      return ((uint16_t)(frame[6+k] & 0x80) >> 7) | ((((uint16_t)frame[7+k])) << 1) | ((((uint16_t)frame[8+k]) & 0x03) << 9);
    case 6:
      return ((uint16_t)(frame[8+k] & 0xFC) >> 2) | ((((uint16_t)frame[9+k]) & 0x1F) << 6);
    case 7:
      return ((uint16_t)(frame[9+k] & 0xE0) >> 5) | (((uint16_t)frame[10+k]) << 3);
  }
  return 0xFFFF; // execution never reaches here, but this supresses a compliler warning
}
