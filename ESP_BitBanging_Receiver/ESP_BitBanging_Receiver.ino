#include <ESP8266WiFi.h>
#define RX_PIN D5             
#define TX_PIN D6             

#define BAUD_RATE 1000     // Přenosová rychlost v bps
#define TOLERANCE 0.1           // Tolerance pro časování (10 %)
#define BIT_DELAY (1000000 / BAUD_RATE) // Zpoždění pro jeden bit (v mikrosekundách)
#define START_STOP_FACTOR 16    // Násobek pro start/stop bit

void setup() {
  WiFi.mode(WIFI_OFF);
WiFi.forceSleepBegin();
delay(1); // Krátká pauza pro uložení stavu
  pinMode(RX_PIN, INPUT);
  Serial.begin(9600);
}

// receiver - ESP8266
void loop() {
  String receivedData = receiveManchesterByte();
  if (receivedData != "") {    
    String validated = validateCRC16(receivedData);
    if (validated != "") {
      Serial.println("CRC OK!");
      Serial.println(validated);
      delay(100);                   // Pauza mezi odesláním jednotlivých zpráv
      sendManchesterByte(receivedData);
      onSuccessfull(receivedData);
    } else {
      Serial.println("INVALID CRC!");
      Serial.println(receivedData);
    }
  }
}

String receiveManchesterByte() {
  char receivedData[128];
  int index = 0;
  //char debugOutput[2048] = "";
   //int index = 0;

  // Čekání na start bit - čeká na HIGH
  while (digitalRead(RX_PIN) == LOW) {
    yield(); // Umožní ostatním procesům běh
  }

  // Měření délky start bitu
  unsigned long startTime = micros(); // začne počítata čas
  while (digitalRead(RX_PIN) == HIGH) { // čeká na LOW
    yield();
  }
  unsigned long measuredDuration = micros() - startTime; // spočítá čas

  // Validace start bitu na základě délky
  unsigned long expectedStartDuration = BIT_DELAY * START_STOP_FACTOR;
  if (measuredDuration < expectedStartDuration * (1.0 - TOLERANCE) ||
    measuredDuration > expectedStartDuration * (1.0 + TOLERANCE)) { // když je čas delší, nebo kratší hoď chybu
  //Serial.print(".");
  return "";
  }

  // Synchronizace na polovinu délky bitu
  delayMicroseconds((4 * BIT_DELAY) + (BIT_DELAY / 4));
  
  // Čtení dat
  int stopBitCounter = 0;
  while (true) {
    
    uint8_t data = 0;
    
    for (int i = 7; i >= 0; i--) {
       bool firstHalf = preciseDigitalRead(RX_PIN,BIT_DELAY / 2); // 650 ns
       bool secondHalf = preciseDigitalRead(RX_PIN,BIT_DELAY / 2);
      if (stopBitCounter == START_STOP_FACTOR) {
        return String(receivedData);
      }
      // Rekonstrukce logického bitu
      if (!firstHalf && secondHalf) {
        data |= (1 << i); // Logická 1 - 62 ns
      } else if (firstHalf && !secondHalf) {
        // Logická 0
      } else if (!firstHalf && !secondHalf) {
        stopBitCounter++;
        i++;
      } else {
        Serial.println("Data bit timing error!");
        Serial.println(String(receivedData));
        //debugOutput[index++] = '\0';
        //Serial.println(debugOutput);
        return "";
      }
    }
    receivedData[index++] = (char)data; // 25 ns
  }
return "";
  
}

void sendManchesterByte(const String& data) {
  String toSend = appendCRC16(data);
  digitalWrite(TX_PIN, HIGH);
  delayMicroseconds(BIT_DELAY * START_STOP_FACTOR);
  digitalWrite(TX_PIN, LOW);
  delayMicroseconds(4 * BIT_DELAY);
  
  //char debugOutput[(toSend.length()*4*8)+1] = "";
  int index = 0;
  // Iterace přes každý znak v řetězci
  for (size_t k = 0; k < toSend.length(); k++) {
    uint8_t character = static_cast<uint8_t>(toSend[k]); // 537 ns
    for (int i = 7; i >= 0; i--) {
      bool bit = (character >> i) & 0x01; // 12 ns
      if (bit) {
        preciseDigitalWrite(TX_PIN, LOW,BIT_DELAY / 2); 
        preciseDigitalWrite(TX_PIN, HIGH,BIT_DELAY / 2); 
      } else {
        preciseDigitalWrite(TX_PIN, HIGH,BIT_DELAY / 2); 
        preciseDigitalWrite(TX_PIN, LOW,BIT_DELAY / 2); 
      }
    }
  }
  //debugOutput[index++] = '\0';
  digitalWrite(TX_PIN, LOW);
  delayMicroseconds(BIT_DELAY * START_STOP_FACTOR);
  digitalWrite(TX_PIN, HIGH);
  delayMicroseconds(BIT_DELAY);
  digitalWrite(TX_PIN, LOW);
  
}

String validateCRC16(const String& input) {
    // Extrahujeme CRC16 z posledních 4 znaků (2 bajty) řetězce
    int length = input.length();
    if (length < 4) {
        return "";  // Pokud je řetězec příliš krátký na CRC, vrátíme prázdný řetězec
    }

    // Poslední 4 znaky (2 bajty) budou CRC16
    String crcHex = input.substring(length - 4);
    uint16_t expectedCRC = strtol(crcHex.c_str(), NULL, 16);  // Převede hex na číslo

    // Získáme původní text (bez CRC)
    String originalText = input.substring(0, length - 4);

    // Vypočítáme CRC16 pro původní text
    uint16_t calculatedCRC = calculateCRC16(reinterpret_cast<const uint8_t*>(originalText.c_str()), originalText.length());

    // Pokud CRC odpovídá, vrátíme původní text bez CRC
    if (calculatedCRC == expectedCRC) {
        return originalText;
    } else {
        return "";  // Pokud CRC neodpovídá, vrátíme prázdný řetězec
    }
}

uint16_t calculateCRC16(const uint8_t *data, size_t length) {
    uint16_t crc = 0xFFFF;  // Počáteční hodnota CRC
    
    for (size_t i = 0; i < length; i++) {
        crc ^= (data[i] << 8);  // Zahrnutí aktuálního bytu (shift na levou část CRC)
        
        for (uint8_t bit = 8; bit > 0; bit--) {
            // Pokud je bit nejvíce vlevo v CRC 1, provedeme XOR s polynomem
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x11021;  // XOR s CRC-16-CCITT-FALSE polynomem
            } else {
                crc = crc << 1;  // Jen posun
            }
        }
    }
    
    return crc;  // Vrátí CRC-16
}
bool preciseDigitalRead(uint8_t pin, uint32_t durationUs) {
    // Převod na cykly procesoru (80 MHz = 12.5 ns na cyklus)
    uint32_t startCycles = ESP.getCycleCount();
    uint32_t targetCycles = startCycles + (durationUs * ESP.getCpuFreqMHz()); // Přesný výpočet cyklů

    // Načtení počátečního stavu pinu
    bool initialState = digitalRead(pin);

    // Čekání pomocí cyklů procesoru bez yield() - čisté aktivní čekání
    while (ESP.getCycleCount() < targetCycles) {
        // Můžete přidat nějaký jednoduchý kód pro minimalizaci spotřeby
        // __asm__ volatile("nop"); // No operation - pro optimalizaci a šetření cyklů
    }

    // Vrátí počáteční stav pinu
    return initialState;
}

void preciseDigitalWrite(uint8_t pin, bool state, uint32_t durationUs) {
    // Převod na cykly procesoru (80 MHz = 12.5 ns na cyklus)
    uint32_t startCycles = ESP.getCycleCount();
    uint32_t targetCycles = startCycles + (durationUs * ESP.getCpuFreqMHz()); // Přesný výpočet cyklů

    // Nastavení GPIO podle zadaného stavu
    digitalWrite(pin, state);

    // Čekání pomocí cyklů procesoru bez yield() - čisté aktivní čekání
    while (ESP.getCycleCount() < targetCycles) {
        // Můžete přidat nějaký jednoduchý kód pro minimalizaci spotřeby
        //__asm__ volatile("nop"); // No operation - pro optimalizaci a šetření cyklů
    }
}

void onSuccessfull(const String& data) {
  
}
}
