/*#include <Arduino.h>
#include <Wire.h>
#include <MS561101BA03.hpp>

MS561101BA03 barometer1(0x76);
MS561101BA03 barometer2(0x77);

const int NUM_SAMPLES = 100;

double pressureSamples1[NUM_SAMPLES];
double pressureSamples2[NUM_SAMPLES];

void setup() {
  Serial.begin(115200);
  Wire.begin();

  Serial.println("Initializing MS5611 sensors...");
  if (!barometer1.init()) {
    Serial.println("Failed to initialize barometer 1 (0x76)");
  }
  if (!barometer2.init()) {
    Serial.println("Failed to initialize barometer 2 (0x77)");
  }
  Serial.println("Initialization complete.");
}

void loop() {
  double sum1 = 0.0, sumSq1 = 0.0;
  double sum2 = 0.0, sumSq2 = 0.0;

  for (int i = 0; i < NUM_SAMPLES; i++) {
    // ---- Barometer 1 ----
    auto dataOpt1 = barometer1.getData();
    if (dataOpt1.has_value()) {
      const SensorData &data1 = dataOpt1.value();
      auto pressureOpt1 = data1.getData("pressure");

      if (pressureOpt1.has_value()) {
        double pressure1 = static_cast<double>(std::get<float>(pressureOpt1.value()));
        pressureSamples1[i] = pressure1;
        sum1 += pressure1;
        sumSq1 += pressure1 * pressure1;
      } else {
        Serial.println("[Baro1] Pressure key missing!");
      }
    } else {
      Serial.println("[Baro1] No data!");
    }

    // ---- Barometer 2 ----
    auto dataOpt2 = barometer2.getData();
    if (dataOpt2.has_value()) {
      const SensorData &data2 = dataOpt2.value();
      auto pressureOpt2 = data2.getData("pressure");

      if (pressureOpt2.has_value()) {
        double pressure2 = static_cast<double>(std::get<float>(pressureOpt2.value()));
        pressureSamples2[i] = pressure2;
        sum2 += pressure2;
        sumSq2 += pressure2 * pressure2;
      } else {
        Serial.println("[Baro2] Pressure key missing!");
      }
    } else {
      Serial.println("[Baro2] No data!");
    }

    delay(10);
  }

  double mean1 = sum1 / NUM_SAMPLES;
  double var1 = (sumSq1 / NUM_SAMPLES) - (mean1 * mean1);

  double mean2 = sum2 / NUM_SAMPLES;
  double var2 = (sumSq2 / NUM_SAMPLES) - (mean2 * mean2);

  // ---- Print results ----
  Serial.println("\n---- Barometer Statistics ----");
  Serial.printf("Barometer 1 (0x76): Mean = %.2f hPa, Variance = %.6f\n", mean1, var1);
  Serial.printf("Barometer 2 (0x77): Mean = %.2f hPa, Variance = %.6f\n", mean2, var2);
  Serial.printf("Pressure difference: %.2f hPa\n", mean1 - mean2);
  Serial.println("---------------------------------\n");

  delay(2000);
}
*/