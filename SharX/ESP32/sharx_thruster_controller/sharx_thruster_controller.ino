unsigned long lastCommandTime = 0;

const unsigned long WATCHDOG_TIMEOUT_MS = 500;

float leftThruster = 0.0;
float rightThruster = 0.0;

bool watchdogStopped = false;


void setup()
{
  Serial.begin(115200);
  delay(1000);

  lastCommandTime = millis();

  Serial.println("ESP32 ready");
}


void loop()
{
  
  // Serial command processing
  if (Serial.available())
  {
    String message = Serial.readStringUntil('\n');
    message.trim();

    if (message == "ping")
    {
      Serial.println("alive");
    }

    else if (message.startsWith("THRUST,"))
    {
      int firstComma = message.indexOf(',');
      int secondComma = message.indexOf(',', firstComma + 1);

      if (firstComma < 0 || secondComma < 0)
      {
        Serial.println("ERR,bad_format");
      }
      else
      {
        String leftString = message.substring(
          firstComma + 1,
          secondComma
        );

        String rightString = message.substring(
          secondComma + 1
        );

        leftThruster = leftString.toFloat();
        rightThruster = rightString.toFloat();

        // Valid command received -> reset watchdog
        lastCommandTime = millis();
        watchdogStopped = false;

        Serial.print("THRUST_OK,");
        Serial.print(leftThruster, 3);
        Serial.print(",");
        Serial.println(rightThruster, 3);
      }
    }

    else
    {
      Serial.print("unknown: ");
      Serial.println(message);
    }
  }

  // Watchdog
  unsigned long now = millis();

  if (
    now - lastCommandTime > WATCHDOG_TIMEOUT_MS
    && !watchdogStopped
  )
  {
    leftThruster = 0.0;
    rightThruster = 0.0;

    watchdogStopped = true;

    Serial.println("WATCHDOG_STOP,0.000,0.000");
  }
}
