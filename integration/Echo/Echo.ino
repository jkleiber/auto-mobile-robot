

const byte numChars = 128;
char receivedChars[numChars]; // an array to store the received data
boolean newData = false;


//void recvWithEndMarker() {
void serialEvent() {
 static byte ndx = 0;
 char endMarker = '\n';
 char rc;
 
 while (Serial.available() > 0){// && newData == false) {
   rc = (char)Serial.read();
  
   if (rc != endMarker) {
     receivedChars[ndx] = rc;
     ndx++;
     if (ndx >= numChars) {
      ndx = numChars - 1;
     }
   }
   else {
     receivedChars[ndx] = '\0'; // terminate the string
     ndx = 0;
     newData = true;
   }
 }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {

  if (newData == true)
  {
      Serial.print(receivedChars);
      Serial.print("\n");
      newData = false;
  }

  delayMicroseconds(100);
}
