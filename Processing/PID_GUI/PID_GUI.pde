import controlP5.*;
import processing.serial.*;

Serial arduino;

ControlP5 cp5;



 
String P, I, D, Speed;
 
void setup() {
  size(700, 400);
  cp5 = new ControlP5(this);
  cp5.addTextfield("P").setPosition(20, 100).setSize(200, 40).setAutoClear(false);
  cp5.addTextfield("I").setPosition(240, 100).setSize(200, 40).setAutoClear(false);
  cp5.addTextfield("D").setPosition(460, 100).setSize(200, 40).setAutoClear(false);
  cp5.addTextfield("Speed").setPosition(300, 40).setSize(80, 40).setAutoClear(false);

  cp5.addBang("Submit").setPosition(300, 170).setSize(80, 40); 
  arduino = new Serial(this, Serial.list()[1], 9600);
}

 
void draw () {
  background(0);
  
  if (arduino.available()>0){
   String serialVal = arduino.readStringUntil('.');
   if (serialVal!=null)  println(serialVal);
  }
  
  
}
 
void Submit() {
  P = cp5.get(Textfield.class,"P").getText();
  I = cp5.get(Textfield.class,"I").getText();
  D = cp5.get(Textfield.class,"D").getText();
  Speed = cp5.get(Textfield.class,"Speed").getText();

  if (P.length() == 2) P = "0" + P;
  if (P.length() == 1) P = "00" + P;
  if (I.length() == 2) I = "0" + I;
  if (I.length() == 1) I = "00" + I;
  if (D.length() == 2) D = "0" + D;
  if (D.length() == 1) D = "00" + D;
  if (Speed.length() == 2) Speed = "0" + Speed;
  if (Speed.length() == 1) Speed = "00" + Speed;

  print(" P = " + P);
  print(" I = " + I);
  print(" D = " + D);
  print(" Speed = " + Speed);

  
  println();
  
  arduino.write(P);
  arduino.write(" ");
  arduino.write(I);
  arduino.write(" ");
  arduino.write(D);
  arduino.write(".");

  
}