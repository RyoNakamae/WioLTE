#include <WioLTEforArduino.h>

//#define BUZZER_PIN      (WIOLTE_D38)
#define BUZZER_PIN      (WIOLTE_D20)
#define BUZZER_ON_TIME  (100)
#define BUZZER_OFF_TIME (3000)

WioLTE Wio;


void setup()
{
  SerialUSB.println("setup Start");
  Wio.Init();
  Wio.PowerSupplyGrove(true);
  
  pinMode(BUZZER_PIN, OUTPUT);
  delay(500);
  SerialUSB.println("setup OK");
}

void loop()
{
  SerialUSB.println("loop start");
  digitalWrite(BUZZER_PIN, HIGH);
  delay(BUZZER_ON_TIME);

  digitalWrite(BUZZER_PIN, LOW);
  delay(BUZZER_OFF_TIME);
  SerialUSB.println("loop end");
}

