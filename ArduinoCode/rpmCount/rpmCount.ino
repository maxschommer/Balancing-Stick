int Htime;              //integer for storing high time
int Ltime;                //integer for storing low time
float Ttime;            // integer for storing total time of a cycle
float frequency;        //storing frequency

int senPin = 8;
int pinVal = 0;

long int lastTime = 0;
long int sigTime = 0;
void setup()
{
    pinMode(senPin,INPUT);
    Serial.begin(9600);
    lastTime = micros();
}
void loop()
{
    int pHigh = digitalRead(8);
    if (pHigh != pinVal)
    {
      pinVal = pHigh;
      if (pHigh)
      {
        Serial.print("Signal Time: ");
        sigTime = micros()-lastTime;
        Serial.println(1000000/sigTime);
        lastTime = micros();
      }
      
    }
    

}
