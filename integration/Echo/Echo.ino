

const byte numChars = 128;
char receivedChars[numChars]; // an array to store the received data
boolean newData = false;


void recvWithEndMarker() {
 static byte ndx = 0;
 char endMarker = '\n';
 char rc;
 
 while (Serial.available() > 0 && newData == false) {
   rc = Serial.read();
  
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

  recvWithEndMarker();

  if (newData == true)
  {
    Serial.println("yes");
//    Serial.println(receivedChars);
    newData = false;
  }

  delay(5);
}
