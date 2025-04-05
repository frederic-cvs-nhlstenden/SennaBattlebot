int _ledRed = 9;
int _ledYellow = 4;
int _ledGreen = 2;

int _redDuration = 3000;
int _yellowDuration = 1000;
int _greenDuration = 4000;

const int _buttonPin = 8;
int _buttonState = 0;

void setup()
{
  pinMode(_ledRed, OUTPUT);
  pinMode(_ledYellow, OUTPUT);
  pinMode(_ledGreen, OUTPUT);

  pinMode(_buttonPin, INPUT);

  digitalWrite(_ledYellow, HIGH);
  digitalWrite(_ledGreen, HIGH);
  digitalWrite(_ledRed, HIGH);
}

void loop()
{

  _buttonState = digitalRead(_buttonPin);

  // If the pushbutton is pressed, the buttonState is set to HIGH, cycling through the traffic light sequence:
  if (_buttonState == LOW)
  {

    digitalWrite(_ledYellow, HIGH);
    digitalWrite(_ledGreen, HIGH);
    digitalWrite(_ledRed, LOW);
    delay(_redDuration);
    digitalWrite(_ledRed, HIGH);
    digitalWrite(_ledGreen, LOW);
    delay(_greenDuration);
    digitalWrite(_ledGreen, HIGH);
    digitalWrite(_ledYellow, LOW);
    delay(_yellowDuration);
    digitalWrite(_ledYellow, HIGH);
  }
  else
  {
    // Default state is red light on
    digitalWrite(_ledRed, LOW);
  }
}