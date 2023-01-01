//Timer2 init acording datasheet
void rtcInit(void)
{
    //Disable timer2 interrupts
    TIMSK2  = 0;

    //Enable asynchronous mode
    ASSR  = (1<<AS2);

    //set initial counter value
    TCNT2=0;

    //set prescaller 64
    TCCR2B |= (1<<CS22);

    //wait for registers update
    while (ASSR & ((1<<TCN2UB)|(1<<TCR2BUB)));

    //clear interrupt flags
    TIFR2  = (1<<TOV2);

    //enable TOV2 interrupt
    TIMSK2  = (1<<TOIE2);
}

void portInit(void)
{
  DDRD |= (1<<DDD7); // set pin D7 arduino OUTPUT (TIMER2)

  PORTD &= ~(1<<PORTD7); // pin D7 arduino LOW (TIMER2 LED OFF)  

  // Pin RAIN Sensor
  DDRD &= ~(1<<DDD2);    // pin PD2 as INPUT
  PORTD |= (1<<PORTD2);  // pin PD2 as PULL-UP

  // Pin WIND Sensor
  DDRD &= ~(1<<DDD3);    // pin PD3 as INPUT
  PORTD |= (1<<PORTD3);  // pin PD3 as PULL-UP
  
  pinMode(BATTERYV,INPUT);
  pinMode(WINDDIRV,INPUT);

}

// RAIN ISR PD2
ISR(INT0_vect) 
{
  // precautionary while we do other stuff
  EIMSK &= ~(1<<INT0);

  data_pld.rain += 1.0;

  EIMSK |= (1<<INT0);
}

// WIND ISR PD3
ISR(INT1_vect)
{   
  // precautionary while we do other stuff
  EIMSK &= ~(1<<INT1);

  windInst[idxWindInst] += 1;

  EIMSK |= (1<<INT1);
}

//Overflow ISR
ISR(TIMER2_OVF_vect)
{
  if(++idxWindInst >= 3)
  {
    idxWindInst = 0;
    ledTimer2Acceso = true;

    uint16_t subTot = 0;
    for(int i=0;i < 3; i++)
    {
      subTot += windInst[i];
      windInst[i] = 0; // azzera il buffer dei 3 secondi
    }
    windData[idxSeconds] = subTot / 3;
    if(++idxSeconds >= MAX_BUFFER)
    {
      sendData = true;
      idxSeconds = 0;    
    }
  }

}


void readWindDir(void)
{
  float readWindDir = (float) (3.3 / 1024.0) * analogRead(WINDDIRV);
  if (readWindDir >= 3.03)
        wind_pld.windDir = 270,0;
  else  if (readWindDir >= 2.84)
          wind_pld.windDir = 315,0;        
  else  if (readWindDir >= 2.65)
          wind_pld.windDir = 292,5;        
  else  if (readWindDir >= 2.51)
          wind_pld.windDir = 0;        
  else  if (readWindDir >= 2.24)
          wind_pld.windDir = 337,5;        
  else  if (readWindDir >= 2.01)
          wind_pld.windDir = 225,0;        
  else  if (readWindDir >= 1.91)
          wind_pld.windDir = 247,5;        
  else  if (readWindDir >= 1.47)
          wind_pld.windDir = 45,0;        
  else  if (readWindDir >= 1.29)
          wind_pld.windDir = 22,5;        
  else  if (readWindDir >= 0.91)
          wind_pld.windDir = 180,0;        
  else  if (readWindDir >= 0.77)
          wind_pld.windDir = 202,5;        
  else  if (readWindDir >= 0.58)
          wind_pld.windDir = 135,0;        
  else  if (readWindDir >= 0.39)
          wind_pld.windDir = 157,5;        
  else  if (readWindDir >= 0.28)
          wind_pld.windDir = 90,0;        
  else  if (readWindDir >= 0.25)
          wind_pld.windDir = 67,5;        
  else  if (readWindDir >= 0.19)
          wind_pld.windDir = 112,5;              
}

