void setup() {
  pinMode(13, OUTPUT);
  Serial.begin(9600);
}


void loop() {
  if (Serial.available()){
      for (i = 0); i < 10; i++) {
      reader[i] = Serial.read();
      delay(2);
    }
  }
    if(reader[0]=='2'&&reader[1]=='O'&&reader[2]=='N'){
    digitalWrite(13,HIGH);
    delay(500);
    Serial.print("AON");
  }
  if(reader[0]=='2'&&reader[1]=='O'&&reader[2]=='F'&&reader[3]=='F'){
    digitalWrite(13, LOW);
    delay(500);
    Serial.print("AOFF");
  }

  for(i=0; i<10;i++)
    reader[i]=0;
}
/*
   Welcome to MultiWii.

   If you see this message, chances are you are using the Arduino IDE. That is ok.
   To get the MultiWii program configured for your copter, you must switch to the tab named 'config.h'.
   Maybe that tab is not visible in the list at the top, then you must use the drop down list at the right
   to access that tab. In that tab you must enable your baord or sensors and optionally various features.
   For more info go to http://www.multiwii.com/wiki/index.php?title=Main_Page

   Have fun, and do not forget MultiWii is made possible and brought to you under the GPL License.

*/

