#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Keypad.h>
#include <EEPROM.h>
#include <Servo.h>

// --- PINI ---
const int pinServo = 10;
const int pinRelay = 11;
const int trigPin = 46; 
const int echoPin = 47;

LiquidCrystal_I2C lcd(0x27, 16, 2); 
Servo usaServo;

// --- KEYPAD ---
const byte ROWS = 4;
const byte COLS = 4;
char keys[ROWS][COLS] = {
  {'1','2','3','A'}, {'4','5','6','B'}, {'7','8','9','C'}, {'*','0','#','D'}
};
byte rowPins[ROWS] = {30, 31, 32, 33}; 
byte colPins[COLS] = {34, 35, 36, 37}; 
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

String inputPin = "";
String masterPin = ""; 
bool modSchimbare = false;

void setup() {
  Serial.begin(9600); // Pornim Serialul pentru monitorizare
  lcd.init();
  lcd.backlight();
  
  usaServo.attach(pinServo, 500, 2500); 
  usaServo.write(0); 
  
  pinMode(pinRelay, OUTPUT);
  digitalWrite(pinRelay, HIGH); // Bec stins

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  citestePinDinEEPROM();
  resetInterfata();
  Serial.println("Sistem pornit. Asteptare cod...");
}

void loop() {
  char tasta = keypad.getKey();
  if (tasta) {
    if (tasta == '#') verificaCod();
    else if (tasta == '*') { modSchimbare = false; resetInterfata(); }
    else if (tasta == 'A') { 
      inputPin = ""; modSchimbare = true; 
      lcd.clear(); lcd.print("Cod Actual:"); 
    }
    else if (inputPin.length() < 4) {
      inputPin += tasta;
      lcd.setCursor(inputPin.length() - 1, 1);
      lcd.print('*');
    }
  }
}

long citesteDistanta() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long durata = pulseIn(echoPin, HIGH, 30000); 
  if (durata == 0) return 999; 
  return durata * 0.034 / 2;
}

void gestioneazaAcces() {
  // 1. Deschidere
  digitalWrite(pinRelay, LOW); // Aprinde Becul
  usaServo.write(185);         // Deschide usa
  
  lcd.clear();
  lcd.print("BINE ATI VENIT!");
  Serial.println("Acces Permis. Usa deschisa, bec aprins.");
  
  delay(5000); // Timp sa intre omul

  // 2. Monitorizare interior via Serial
  unsigned long momentUltimaDetectie = millis();
  const long timpAsteptareDupaPlecare = 5000; 

  while (true) {
    long distanta = citesteDistanta();
    
    Serial.print("Senzor Interior - Distanta: ");
    Serial.print(distanta);
    Serial.print(" cm | Status: ");

    if (distanta < 10) { 
      Serial.println("PREZENTA DETECTATA");
      momentUltimaDetectie = millis(); 
    } else {
      Serial.println("ZONA LIBERA");
    }

    if (millis() - momentUltimaDetectie > timpAsteptareDupaPlecare) {
      Serial.println("Timp expirat. Se declanseaza inchiderea.");
      break; 
    }
    
    delay(500); // Verificam de 2 ori pe secunda
  }

  // 3. Inchidere simultana
  lcd.clear();
  lcd.print("Inchidere...");
  Serial.println("Se inchide usa si becul simultan.");
  
  usaServo.write(0);            // Comanda inchidere usa
  digitalWrite(pinRelay, HIGH); // Stingere imediata bec
  
  delay(2000); // Timp pentru mecanismul servo
  resetInterfata();
}

// --- FUNCTII AUXILIARE ---
void verificaCod() {
  lcd.clear();
  if (modSchimbare) {
    if (inputPin == masterPin) {
      lcd.print("Cod Nou:");
      inputPin = "";
      modSchimbare = false;
      while (inputPin.length() < 4) {
        char t = keypad.waitForKey();
        if (t >= '0' && t <= '9') {
          inputPin += t;
          lcd.setCursor(inputPin.length() - 1, 1);
          lcd.print('*');
        }
      }
      masterPin = inputPin;
      salveazaPinInEEPROM(masterPin);
      lcd.clear(); lcd.print("Pin Salvat!"); delay(2000); resetInterfata();
    } else { lcd.print("Eroare Auth!"); delay(2000); resetInterfata(); }
  } else {
    if (inputPin == masterPin) gestioneazaAcces();
    else { lcd.print("Cod Incorect!"); delay(2000); resetInterfata(); }
  }
}

void salveazaPinInEEPROM(String pin) { for (int i = 0; i < 4; i++) EEPROM.write(i, pin[i]); }

void citestePinDinEEPROM() {
  masterPin = "";
  for (int i = 0; i < 4; i++) {
    char c = EEPROM.read(i);
    if (c == 0xFF || c == 0) { masterPin = "1234"; break; }
    masterPin += c;
  }
}

void resetInterfata() {
  lcd.clear(); 
  lcd.print("Introduceti Cod:");
  lcd.setCursor(0, 1); 
  inputPin = ""; 
  modSchimbare = false;
}
