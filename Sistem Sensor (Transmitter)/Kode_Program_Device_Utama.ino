//LORA SENDER ( uji coba user id dengan kode gps first then lora, fast stabilizer )
#include "MAX30105.h"
#include "heartRate.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include <SPI.h>
#include <LoRa.h>

#define REPORTING_PERIOD_MS 500    // Lebih cepat dari 1000ms ke 500ms
#define ONE_WIRE_BUS 4
#define FINGER_ON 7000             // Ambang deteksi jari
#define MINIMUM_SPO2 60.0
#define ss 5
#define rst 14
#define dio0 26

// Buffer yang lebih kecil untuk response cepat
#define MAX_BPM_BUFFER 5           // Kurangi dari 10 ke 5 untuk response cepat
#define MAX_SPO2_BUFFER 5          // Kurangi dari 10 ke 5 untuk response cepat
#define VALID_READING_THRESHOLD 2  // Kurangi dari 3 ke 2 untuk response cepat

MAX30105 particleSensor;
TinyGPSPlus gps;
HardwareSerial gpsSerial(2); // UART2 pada ESP32 untuk GPS NEO6M

// Heart rate calculation variables - dengan buffer yang lebih kecil untuk response cepat
const byte RATE_SIZE = 6;  // Kurangi dari 8 ke 6
byte rates[RATE_SIZE];
byte rateSpot = 0;
long lastBeat = 0;
float beatsPerMinute;
int beatAvg;
int lastBeatAvg = 0;

// Filter variabel untuk BPM - buffer lebih kecil
float bpmBuffer[MAX_BPM_BUFFER];
int bpmBufferIndex = 0;
int validBpmReadings = 0;
float filteredBPM = 0;
bool bpmIsValid = false;

// Blood oxygen calculation variables
double avered = 0;
double aveir = 0;
double sumirrms = 0;
double sumredrms = 0;
double SpO2 = 0;
double ESpO2 = 90.0;
double FSpO2 = 0.8;        // Kurangi dari 0.7 ke 0.8 untuk response lebih cepat
double frate = 0.90;       // Kurangi dari 0.95 ke 0.90 untuk response lebih cepat
int i = 0;
int Num = 25;              // Kurangi dari 50 ke 25 untuk response lebih cepat

// Filter variabel untuk SpO2 - buffer lebih kecil
float spo2Buffer[MAX_SPO2_BUFFER];
int spo2BufferIndex = 0;
int validSpo2Readings = 0;
float filteredSpO2 = 0;
bool spo2IsValid = false;
float lastValidSpO2 = 95.0;

// DS18B20 temperature sensor setup
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensor(&oneWire);
float suhuDS18B20;

// Fungsi kalibrasi suhu
float calibrateTemperature(float rawTemp) {
    const float CALIBRATION_OFFSET = 0.75;
    const float CALIBRATION_FACTOR = 1.0;
    
    float calibratedTemp = (rawTemp * CALIBRATION_FACTOR) + CALIBRATION_OFFSET;
    calibratedTemp = round(calibratedTemp * 10.0) / 10.0;
    
    return calibratedTemp;
}

// Fungsi kalibrasi BPM
float calibrateBPM(float rawBPM) {
    const float CALIBRATION_FACTOR = 1.171;
    const float CALIBRATION_OFFSET = 0.0;
    
    float calibratedBPM = (rawBPM * CALIBRATION_FACTOR) + CALIBRATION_OFFSET;
    calibratedBPM = round(calibratedBPM);
    
    if (calibratedBPM < 40) calibratedBPM = 40;
    if (calibratedBPM > 220) calibratedBPM = 220;
    
    return calibratedBPM;
}

// Voltage and battery capacity variables
float Volt1;
float Volt;
float capacity;

const float V_min = 3.0;
const float V_max = 4.2;

// Timing variables
unsigned long lastVitalReport = 0;
unsigned long lastTempRequest = 0;
unsigned long lastTempRead = 0;
unsigned long lastGPSReport = 0;
unsigned long lastBatteryCheck = 0;
bool tempRequestSent = false;

// Variabel untuk cek stabilitas - DIPERCEPAT
unsigned long fingerPresentTime = 0;
bool fingerStable = false;
const unsigned long STABILITY_DELAY = 1000; // Kurangi dari 2000ms ke 1000ms (1 detik)

//VARIABLE USER ID
const char* userID = "User3";

void setup() {
    Serial.begin(115200);
    Serial.println("System Start - Fast Response Mode");

    // Initialize MAX30102
    if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
        Serial.println("MAX30102 not found");
        while (1);
    }

    // Konfigurasi MAX30102 yang dioptimalkan untuk response cepat
    byte ledBrightness = 0x3F;          // Medium brightness (antara 0x1F dan 0x7F)
    byte sampleAverage = 4;             // Kurangi dari 8 ke 4 untuk response cepat
    byte ledMode = 2;                   // Mode 2 = Red + IR
    int sampleRate = 600;               // Naikkan dari 400 ke 600 Hz untuk response cepat
    int pulseWidth = 311;               // Medium pulse width untuk balance speed vs signal
    int adcRange = 8192;                // Medium ADC range untuk balance noise vs speed
    
    particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
    particleSensor.enableDIETEMPRDY();
    
    particleSensor.setPulseAmplitudeRed(0x0F);     // Medium amplitude
    particleSensor.setPulseAmplitudeGreen(0);
    particleSensor.setPulseAmplitudeIR(0x0F);      // Medium amplitude
    
    // Inisialisasi buffer dengan ukuran yang lebih kecil
    for (int i = 0; i < MAX_BPM_BUFFER; i++) bpmBuffer[i] = 0;
    for (int i = 0; i < MAX_SPO2_BUFFER; i++) spo2Buffer[i] = 0;

    // Initialize DS18B20
    sensor.begin();
    sensor.setResolution(12);

    // Initialize GPS NEO6M
    gpsSerial.begin(9600, SERIAL_8N1, 16, 17);

    // Initialize LoRa
    LoRa.setPins(ss, rst, dio0);
    while (!LoRa.begin(923.0E6)) {
        Serial.println("Connecting LoRa...");
        delay(500);
    }
    LoRa.setSyncWord(0xF3);
    LoRa.setTxPower(15);
    LoRa.setCodingRate4(8);
    LoRa.setSignalBandwidth(250E3);
    LoRa.setSpreadingFactor(9);
    Serial.println("LoRa Initialized Successfully");
}

void sendLoRaData(float bpm, double spo2, float suhu, double lat, double lng, float batteryCapacity) {
    LoRa.beginPacket();
    LoRa.print("ID: ");
    LoRa.print(userID);
    LoRa.print(" BPM: ");
    LoRa.print(bpm, 1);
    LoRa.print(", SpO2: ");
    LoRa.print(spo2, 1);
    LoRa.print(", Suhu: ");
    LoRa.print(suhu, 1);
    LoRa.print("C, Lat: ");
    LoRa.print(lat, 6);
    LoRa.print(", Lng: ");
    LoRa.print(lng, 6);
    LoRa.print(", Battery: ");
    LoRa.print(batteryCapacity, 1);
    LoRa.print("%");
    LoRa.endPacket();
}

// Fungsi untuk filter BPM dengan response cepat
float filterBPM(float newBPM) {
    float calibratedBPM = calibrateBPM(newBPM);
    
    if (calibratedBPM < 40 || calibratedBPM > 220) {
        return filteredBPM;
    }
    
    // Weighted average: nilai baru memiliki bobot lebih besar untuk response cepat
    float weight = 0.6;  // Bobot tinggi untuk nilai baru (60%)
    if (validBpmReadings == 0) {
        filteredBPM = calibratedBPM;
    } else {
        filteredBPM = (filteredBPM * (1.0 - weight)) + (calibratedBPM * weight);
    }
    
    // Tetap gunakan buffer untuk smoothing ringan
    bpmBuffer[bpmBufferIndex] = calibratedBPM;
    bpmBufferIndex = (bpmBufferIndex + 1) % MAX_BPM_BUFFER;
    
    if (validBpmReadings < MAX_BPM_BUFFER) {
        validBpmReadings++;
    }
    
    return filteredBPM;
}

// Fungsi untuk filter SpO2 dengan response cepat
float filterSpO2(float newSpO2) {
    float calibrationFactor = 1.12;
    float calibratedSpO2 = newSpO2 * calibrationFactor;
    
    if (calibratedSpO2 > 100.0) {
        calibratedSpO2 = 100.0;
    }
    
    if (calibratedSpO2 < MINIMUM_SPO2 || calibratedSpO2 > 100) {
        return filteredSpO2;
    }
    
    // Weighted average: nilai baru memiliki bobot lebih besar untuk response cepat
    float weight = 0.5;  // Bobot medium untuk nilai baru (50%)
    if (validSpo2Readings == 0) {
        filteredSpO2 = calibratedSpO2;
    } else {
        filteredSpO2 = (filteredSpO2 * (1.0 - weight)) + (calibratedSpO2 * weight);
    }
    
    // Tetap gunakan buffer untuk smoothing ringan
    spo2Buffer[spo2BufferIndex] = calibratedSpO2;
    spo2BufferIndex = (spo2BufferIndex + 1) % MAX_SPO2_BUFFER;
    
    if (validSpo2Readings < MAX_SPO2_BUFFER) {
        validSpo2Readings++;
    }
    
    lastValidSpO2 = filteredSpO2;
    return filteredSpO2;
}

void checkVitalSigns() {
    long irValue = particleSensor.getIR();

    if (irValue > FINGER_ON) {
        // Cek stabilitas dengan waktu yang lebih singkat (1 detik)
        if (!fingerStable) {
            if (fingerPresentTime == 0) {
                fingerPresentTime = millis();
                Serial.println("Finger detected. Stabilizing... (1 sec)");
            } else if (millis() - fingerPresentTime > STABILITY_DELAY) {
                fingerStable = true;
                Serial.println("Finger stable. Starting fast measurements...");
            }
        }
        
        if (fingerStable) {
            // Cek heart beat
            if (checkForBeat(irValue)) {
                long delta = millis() - lastBeat;
                lastBeat = millis();
                
                beatsPerMinute = 60 / (delta / 1000.0);
                
                if (beatsPerMinute < 220 && beatsPerMinute > 40) {
                    rates[rateSpot++] = (byte)beatsPerMinute;
                    rateSpot %= RATE_SIZE;
                    
                    // Hitung rata-rata BPM
                    beatAvg = 0;
                    byte validValues = 0;
                    for (byte x = 0; x < RATE_SIZE; x++) {
                        if (rates[x] > 0) {
                            beatAvg += rates[x];
                            validValues++;
                        }
                    }
                    
                    if (validValues > 0) {
                        beatAvg /= validValues;
                        beatAvg = filterBPM(beatAvg);
                        bpmIsValid = (validBpmReadings >= VALID_READING_THRESHOLD);
                        lastBeatAvg = beatAvg;
                    }
                }
            }

            // Proses data untuk SpO2
            if (particleSensor.available()) {
                i++;
                uint32_t ir = particleSensor.getFIFOIR();
                uint32_t red = particleSensor.getFIFORed();

                double fir = (double)ir;
                double fred = (double)red;

                aveir = aveir * frate + fir * (1.0 - frate);
                avered = avered * frate + fred * (1.0 - frate);
                
                sumirrms += (fir - aveir) * (fir - aveir);
                sumredrms += (fred - avered) * (fred - avered);

                if ((i % Num) == 0) {
                    if (avered > 0 && aveir > 0 && sumredrms > 0 && sumirrms > 0) {
                        double R = (sqrt(sumirrms) / aveir) / (sqrt(sumredrms) / avered);
                        SpO2 = 104.0 - 18.0 * R;
                        
                        if (SpO2 >= MINIMUM_SPO2 && SpO2 <= 100) {
                            ESpO2 = FSpO2 * ESpO2 + (1.0 - FSpO2) * SpO2;
                            ESpO2 = filterSpO2(ESpO2);
                            spo2IsValid = (validSpo2Readings >= VALID_READING_THRESHOLD);
                        }
                    }
                    
                    sumredrms = 0.0;
                    sumirrms = 0.0;
                    i = 0;
                }
                particleSensor.nextSample();
            }

            // Laporan data tanda vital dengan interval yang lebih cepat
            if (millis() - lastVitalReport >= REPORTING_PERIOD_MS) {
                Serial.print("BPM: ");
                if (bpmIsValid) {
                    Serial.print(beatAvg);
                } else {
                    Serial.print("--");
                }
                
                Serial.print(", SpO2: ");
                if (spo2IsValid) {
                    Serial.print(ESpO2, 1);
                    Serial.print("%");
                } else {
                    Serial.print("--");
                }
                
                Serial.print(" (IR: ");
                Serial.print(irValue);
                Serial.println(")");
                
                lastVitalReport = millis();
            }
        } else {
            // Tampilkan progress stabilisasi
            if (millis() - lastVitalReport >= 200) {  // Update setiap 200ms selama stabilisasi
                unsigned long elapsed = millis() - fingerPresentTime;
                Serial.print("Stabilizing... ");
                Serial.print(elapsed);
                Serial.print("/");
                Serial.print(STABILITY_DELAY);
                Serial.println("ms");
                lastVitalReport = millis();
            }
        }
    } else {
        // Reset saat tidak ada jari
        if (millis() - lastVitalReport >= REPORTING_PERIOD_MS) {
            resetVitalSigns();
            Serial.println("Place your finger on the sensor");
            lastVitalReport = millis();
        }
    }
}

void resetVitalSigns() {
    // Reset semua variabel ke nilai awal
    for (byte rx = 0; rx < RATE_SIZE; rx++) rates[rx] = 0;
    beatAvg = 0;
    rateSpot = 0;
    lastBeat = 0;
    
    // Reset variabel SpO2
    avered = 0;
    aveir = 0;
    sumirrms = 0;
    sumredrms = 0;
    SpO2 = 0;
    ESpO2 = 90.0;
    
    // Reset variabel stabilitas
    fingerStable = false;
    fingerPresentTime = 0;
    
    // Reset filter buffers
    for (int i = 0; i < MAX_BPM_BUFFER; i++) bpmBuffer[i] = 0;
    for (int i = 0; i < MAX_SPO2_BUFFER; i++) spo2Buffer[i] = 0;
    bpmBufferIndex = 0;
    spo2BufferIndex = 0;
    validBpmReadings = 0;
    validSpo2Readings = 0;
    filteredBPM = 0;
    filteredSpO2 = 0;
    bpmIsValid = false;
    spo2IsValid = false;
}

void handleTemperature() {
    unsigned long currentMillis = millis();

    if (!tempRequestSent && (currentMillis - lastTempRequest >= 1000)) {
        sensor.requestTemperatures();
        tempRequestSent = true;
        lastTempRequest = currentMillis;
        lastTempRead = currentMillis;
    }

    if (tempRequestSent && (currentMillis - lastTempRead >= 600)) {  // Kurangi dari 800ms ke 600ms
        float rawTemp = sensor.getTempCByIndex(0);
        if (rawTemp != -127.00) {
            suhuDS18B20 = calibrateTemperature(rawTemp);
        }
        Serial.print("Suhu: ");
        Serial.println(suhuDS18B20, 1);
        tempRequestSent = false;
    }
}

void handleGPS() {
    unsigned long currentMillis = millis();

    while (gpsSerial.available() > 0) {
        if (gps.encode(gpsSerial.read())) {
            if (gps.location.isValid() && (currentMillis - lastGPSReport >= 800)) {  // Kurangi dari 1000ms ke 800ms
                Serial.print("Lat: ");
                Serial.print(gps.location.lat(), 6);
                Serial.print(", Lng: ");
                Serial.println(gps.location.lng(), 6);
                
                // Kirim data LoRa dengan threshold yang lebih rendah
                float bpmToSend = (validBpmReadings >= 1) ? beatAvg : 0;  // Kurangi threshold
                float spo2ToSend = (validSpo2Readings >= 1) ? ESpO2 : 0;  // Kurangi threshold
                
                sendLoRaData(bpmToSend, spo2ToSend, suhuDS18B20, 
                             gps.location.lat(), gps.location.lng(), capacity);
                lastGPSReport = currentMillis;
            }
        }
    }

    if (currentMillis > 5000 && gps.charsProcessed() < 10) {
        Serial.println(F("No GPS data received: check wiring."));
        delay(5000);
    }
}

void readBatteryCapacity() {
    unsigned long currentMillis = millis();
    if (currentMillis - lastBatteryCheck >= 3000) {  // Kurangi dari 5000ms ke 3000ms
        Volt1 = analogRead(35);
        float rawVolt = Volt1 * 0.00088 * 5;
        float calibrationFactor = 0.6868;
        Volt = rawVolt * calibrationFactor;
        
        if (Volt >= V_max) {
            capacity = 100.0;
        } else if (Volt <= V_min) {
            capacity = 0.0;
        } else {
            capacity = ((Volt - V_min) / (V_max - V_min)) * 100.0;
        }
        
        Serial.print("Tegangan (Raw): ");
        Serial.print(rawVolt, 3);
        Serial.println(" V");
        
        Serial.print("Tegangan (Calibrated): ");
        Serial.print(Volt, 2);
        Serial.println(" V");
        
        Serial.print("Kapasitas Baterai: ");
        Serial.print(capacity);
        Serial.println(" %");

        lastBatteryCheck = currentMillis;
    }
}

void loop() {
    checkVitalSigns();
    handleTemperature();
    handleGPS();
    readBatteryCapacity();
}