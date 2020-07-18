  // YuLetter

  #include <WioLTEforArduino.h>
  #include <ADXL345.h>          // https://github.com/Seeed-Studio/Accelerometer_ADXL345
  #include <TinyGPS++.h>
  #include <stdio.h>

  #define SENSOR_TemperatureAndHumidity_PIN    (WIOLTE_D38)
  #define BUTTON_PIN  (WIOLTE_D20)

  #define INTERVAL_LOOP (1000)
  #define INTERVAL_Accelerometer  (18000)

  #define INTERVAL_SEND   (1000)
  #define RECEIVE_TIMEOUT (10000)

  //500:12分ぐらい
  #define LARGE_LOOP_LIMIT  (500)  //ここの数字が大きいとHelthCheckまでの間隔が長くなる

  #define G_AVE_LIMIT  (0.03) //ここの値でどれくらいの揺れ？を検知するかをチューニングする
  #define G_AVE_LOOP_LIMIT  (20)  //何回分の加速度を集計して平均値を算出するか？


  //状態に対する色設定
  #define COLOR_SETUP 0, 10, 0
  #define COLOR_NONE 0, 0, 0
  #define COLOR_GPS 10, 0, 10
  #define COLOR_TempAndHumi 10, 10, 10
  #define COLOR_BUTTON 10, 10, 0
  #define COLOR_Accelerometer 10, 0, 0
  #define COLOR_Send 0, 0, 10
  #define COLOR_ERROR 255, 0, 0

  WioLTE Wio;
  TinyGPSPlus gps;
  ADXL345 Accel;

  int large_loop_count;
  double  before_g_ave;
  double  latitude;
  double  longitude;
  volatile int mode = 0;

  void setup()
  {
    delay(500);

    SerialUSB.println("### I/O Initialize.");

    GpsBegin(&Serial);
    delay(500);

    Wio.Init();
    Wio.LedSetRGB(COLOR_SETUP);

    SerialUSB.println("### Power supply ON.");
    Wio.PowerSupplyGrove(true);
    delay(500);

    Accel.powerOn();
    delay(500);

    pinMode(BUTTON_PIN, INPUT);
    delay(500);

    attachInterrupt(BUTTON_PIN, change_state, RISING);

    TemperatureAndHumidityBegin(SENSOR_TemperatureAndHumidity_PIN);
    delay(500);

    Wio.PowerSupplyLTE(true);
    delay(500);

    SerialUSB.println("### Turn on or reset.");
    if (!Wio.TurnOnOrReset()) {
      SerialUSB.println("### ERROR! ###");
      return;
    }

    SerialUSB.println("### Connecting to \"soracom.io\".");
    if (!Wio.Activate("soracom.io", "sora", "sora")) 
    {
      SerialUSB.println("### SORACOM Activate ERROR ###");
      return;
    }

    large_loop_count = 0;
    before_g_ave = 0;
    SerialUSB.println("### Setup completed.");

    SerialUSB.print("G_AVE_LOOP_LIMIT : ");
    SerialUSB.println(G_AVE_LOOP_LIMIT);
    SerialUSB.print("G_AVE_LIMIT : ");
    SerialUSB.println(G_AVE_LIMIT);

    Wio.LedSetRGB(COLOR_NONE);
  }

  void loop()
  {
    if(GpsRead())
    {
      SerialUSB.print("LAT= "); 
      SerialUSB.print(latitude, 6);
      SerialUSB.print(" / LON= ");
      SerialUSB.println(longitude, 6);
    }else
    {
      SerialUSB.println("GPS Data INVALID");
      latitude = 0;
      longitude = 0;
    }
    delay(500);

    unsigned long start = micros();

    CheckShake();
    delay(500);

    large_loop_count ++;
  // 温度湿度についても一定時間ごとにチェックして送信
    if(large_loop_count >= LARGE_LOOP_LIMIT)
    {
      float temp;
      float humi;
      get_TemperatureAndHumidity(&temp, &humi);

      Wio.LedSetRGB(COLOR_Send);
      SerialUSB.println("### Send !! ");
      SendData(temp , humi);
      delay(500);
      mode = 0;

      //何もしないと平常値に戻るタイミングでもう一回送信してしまうので、ここで値を初期化しておく。
      before_g_ave = 0;
      large_loop_count = 0;
    }

    Wio.LedSetRGB(COLOR_NONE);
    delay(INTERVAL_LOOP);
  }

  ////////////////////////////////////////////////////////////////////////////////////////
  //
  //データ送信
  void SendData(float temp , float humi)
  {
    char data[200];
    sprintf(data,"{\"lat\":%.6lf,\"lon\":%.6lf,\"temp\":%.1f,\"humi\":%.1f,\"g_ave\":%.3lf,\"mode\":%d}", latitude, longitude, temp, humi, before_g_ave, mode);

    SerialUSB.print("- Open ");
    int connectId;
    connectId = Wio.SocketOpen("harvest.soracom.io", 8514, WIOLTE_UDP);
    if (connectId < 0) 
    {
      SerialUSB.println("### SocketOpen ERROR! ###");
      SerialUSB.println(connectId);
      goto err;
    }

    SerialUSB.println(" -- Send ");
    SerialUSB.println(data);
    if (!Wio.SocketSend(connectId, data)) 
    {
      SerialUSB.println("### SocketSend ERROR! ###");
      goto err_close;
    }

    SerialUSB.print(" --- Receive ");
    int length;
    length = Wio.SocketReceive(connectId, data, sizeof (data), RECEIVE_TIMEOUT);
    if (length < 0) 
    {
      SerialUSB.println("### SocketReceive ERROR! ###");
      goto err_close;
    }
    if (length == 0)
    {
      SerialUSB.println("### RECEIVE TIMEOUT! ###");
      goto err_close;
    }

  err_close:
    SerialUSB.println(" ---- Close ");
    if (!Wio.SocketClose(connectId))
    {
      SerialUSB.println("### SocketClose ERROR! ###");
      goto err;
    }

  err:
    delay(INTERVAL_SEND);
  }

  ////////////////////////////////////////////////////////////////////////////////////////
  //
  //揺れチェック
  void CheckShake()
  {
    SerialUSB.println("### Accelerometer : CheckShake");
    Wio.LedSetRGB(COLOR_Accelerometer);

    double g_sum = 0;
    for(int cnt=0;cnt<G_AVE_LOOP_LIMIT;cnt++)
    {
      g_sum += Accelerometer();
    }
    double currnet_g_ave = g_sum / G_AVE_LOOP_LIMIT;
    SerialUSB.print("before_g_ave : ");
    SerialUSB.print(before_g_ave);
    SerialUSB.print(" / currnet_g_ave : ");
    SerialUSB.println(currnet_g_ave);

    if(before_g_ave!=0 && abs(before_g_ave-currnet_g_ave) >= G_AVE_LIMIT)
    {
      //揺れてる
      Wio.LedSetRGB(COLOR_ERROR);
      SerialUSB.println(">>> Shake!! <<<");
      SetShakeState();
      mode = 2;
    }

    //差し替える
    before_g_ave = currnet_g_ave;
    Wio.LedSetRGB(COLOR_NONE);

  }

  ////////////////////////////////////////////////////////////////////////////////////////
  //
  void SetShakeState()
  {
    large_loop_count = LARGE_LOOP_LIMIT;
  }
  ////////////////////////////////////////////////////////////////////////////////////////
  //
  //ボタンによる割込み処理
  volatile bool StateChanged = false;
  volatile bool State = false;

  void change_state()
  {
    SerialUSB.println(">>> Push Button <<<");
    Wio.LedSetRGB(COLOR_BUTTON);
    State = !State;
    StateChanged = true;

    mode = 1;
    SetShakeState();
  }


  ////////////////////////////////////////////////////////////////////////////////////////
  //

  // 加速度センサー
  //void Accelerometer()
  double Accelerometer()
  {
    double xyz[3];
    double ax,ay,az;
    Accel.getAcceleration(xyz);
    ax = xyz[0];
    ay = xyz[1];
    az = xyz[2];

    double res = sqrtf(ax * ax + ay * ay + az * az); 

    return res;
  }

  ////////////////////////////////////////////////////////////////////////////////////////
  //
  // 温湿度
  void get_TemperatureAndHumidity(float* temp, float* humi)
  {
    SerialUSB.println("### Temperature And Humidity");
    Wio.LedSetRGB(COLOR_TempAndHumi);

    if (!TemperatureAndHumidityRead(temp, humi)) {
      SerialUSB.println("ERROR!");
      goto err;
    }

    SerialUSB.print("Current humidity = ");
    SerialUSB.print(*humi);
    SerialUSB.print("%  ");
    SerialUSB.print("temperature = ");
    SerialUSB.print(*temp);
    SerialUSB.println("C");

  err:
    delay(1000);
  }

  int TemperatureAndHumidityPin;

  void TemperatureAndHumidityBegin(int pin)
  {
    TemperatureAndHumidityPin = pin;
    DHT11Init(TemperatureAndHumidityPin);
  }
  bool TemperatureAndHumidityRead(float* temperature, float* humidity)
  {
    byte data[5];

    DHT11Start(TemperatureAndHumidityPin);
    for (int i = 0; i < 5; i++) data[i] = DHT11ReadByte(TemperatureAndHumidityPin);
    DHT11Finish(TemperatureAndHumidityPin);

    if(!DHT11Check(data, sizeof (data))) return false;
    if (data[1] >= 10) return false;
    if (data[3] >= 10) return false;

    *humidity = (float)data[0] + (float)data[1] / 10.0f;
    *temperature = (float)data[2] + (float)data[3] / 10.0f;

    return true;
  }

  void DHT11Init(int pin)
  {
    digitalWrite(pin, HIGH);
    pinMode(pin, OUTPUT);
  }

  void DHT11Start(int pin)
  {
    // Host the start of signal
    digitalWrite(pin, LOW);
    delay(18);

    // Pulled up to wait for
    pinMode(pin, INPUT);
    while (!digitalRead(pin)) ;

    // Response signal
    while (digitalRead(pin)) ;

    // Pulled ready to output
    while (!digitalRead(pin)) ;
  }

  byte DHT11ReadByte(int pin)
  {
    byte data = 0;

    for (int i = 0; i < 8; i++) {
      while (digitalRead(pin)) ;

      while (!digitalRead(pin)) ;
      unsigned long start = micros();

      while (digitalRead(pin)) ;
      unsigned long finish = micros();

      if ((unsigned long)(finish - start) > 50) data |= 1 << (7 - i);
    }

    return data;
  }

  void DHT11Finish(int pin)
  {
    // Releases the bus
    while (!digitalRead(pin)) ;
    digitalWrite(pin, HIGH);
    pinMode(pin, OUTPUT);
  }

  bool DHT11Check(const byte* data, int dataSize)
  {
    if (dataSize != 5) return false;

    byte sum = 0;
    for (int i = 0; i < dataSize - 1; i++) {
      sum += data[i];
    }

    return data[dataSize - 1] == sum;
  }

  ////////////////////////////////////////////////////////////////////////////////////////
  //
  // GPS
  #define GPS_OVERFLOW_STRING "OVERFLOW"

  HardwareSerial* GpsSerial;
  char GpsData[200];
  char GpsDataLength;

  void GpsBegin(HardwareSerial* serial)
  {
    GpsSerial = serial;
    GpsSerial->begin(9600);
    GpsDataLength = 0;
  }

  bool GpsRead()
  {
    SerialUSB.println("### GpsRead Start");
    Wio.LedSetRGB(COLOR_GPS);

    while (GpsSerial->available()) 
    {
      if(gps.encode(GpsSerial->read()))
      {
        if (gps.location.isValid())
        {
          latitude = gps.location.lat();
          longitude = gps.location.lng();
          return true;
        }
      }
    }
    return false;
  }

  void displayGPSInfo()
  {
    SerialUSB.println("-- -- -- ");
    if (gps.location.isValid()) 
    {
      SerialUSB.print("LAT= "); SerialUSB.println(gps.location.lat(), 6);
      SerialUSB.print("LON= "); SerialUSB.println(gps.location.lng(), 6);
    }else
    {
      SerialUSB.println(F("INVALID"));
    }
  }
  ////////////////////////////////////////////////////////////////////////////////////////