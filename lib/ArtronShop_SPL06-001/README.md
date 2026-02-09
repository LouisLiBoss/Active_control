# ArtronShop_SPL06-001

Arduino library of SPL06-001 Digital pressure sensor

## Examples

```C++
#include <Arduino.h>
#include <Wire.h>
#include <ArtronShop_SPL06-001.h>

ArtronShop_SPL06_001 spl06_001(0x77, &Wire); // ADDR: 0 => 0x77, ADDR: 1 => 0x76

void setup() {
  Serial.begin(115200);

  Wire.begin();
  while (!spl06_001.begin()) {
    Serial.println("SPL06-001 not found !");
    delay(1000);
  }
}

void loop() {
  Serial.print("Pressure: ");
  Serial.print(spl06_001.pressure() * 0.01, 1); // 1 Pa = 0.01 hPa so Pa * 0.01 = hPa
  Serial.print(" hPa\tTemperature: ");
  Serial.print(spl06_001.temperature(), 1);
  Serial.print(" *C");
  Serial.println();
  delay(1000);
}
```

