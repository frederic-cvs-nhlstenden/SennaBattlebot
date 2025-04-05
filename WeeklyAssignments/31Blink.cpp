const int _buttonPin = 8; // Button pin definition
const int _ledPin = 9;    // LED pin definition

int _buttonState;
int _lastButtonState = LOW;
int _blinkSpeed = 500;
bool _ledState = LOW;
unsigned long previousMillis = 0;

void setup()
{
  pinMode(_ledPin, OUTPUT);
  pinMode(_buttonPin, INPUT);
}

void loop()
{
  _buttonState = digitalRead(_buttonPin);

  // Check for a state change from HIGH to LOW, to toggle blink speed
  if (_buttonState == LOW && _lastButtonState == HIGH)
  {
    if (_blinkSpeed == 500)
    {
      _blinkSpeed = 50;
    }
    else
    {
      _blinkSpeed = 500;
    }
    delay(200);
  }

  _lastButtonState = _buttonState;

  // Blink the LED using non-blocking timing based on millis()
  if (millis() - previousMillis >= _blinkSpeed)
  {
    previousMillis = millis();
    _ledState = !_ledState;           // Toggle LED state
    digitalWrite(_ledPin, _ledState); // Apply new state to LED
  }
}