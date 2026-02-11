// fichier main testant des fonctions écartées de la boucle prinicpale pour optimisation
#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <SerialFlash.h>
#include "ArtronShop_SPL06-001.h"
#include <ICM42688.h>

// --- Configuration Matérielle ---
const int SERVO_PIN = 5; // Pin connecté au servo PTK 7466W
Servo finServo; // Utilisation de la bibliothèque "Servo"
const int FLASH_CS_PIN = 10; // Pin pour la mémoire Flash W25Q128
uint32_t flashAddress = 0; // Pour savoir où on en est dans la mémoire

ArtronShop_SPL06_001 spl06(0x76, &Wire); // adresse nécessaire pour l'I2C, à vérifier selon le câblage (0x76 ou 0x77)
ICM42688 imu(Wire, 0x68);

// --- Structure et Paramètres du Gain Scheduling ---
struct PIDGains { 
    float kp;
    float ki;
    float kd;
};

// Tableau de gains pour différentes plages de vitesse (exemple)
const PIDGains lowSpeed    = {2.0f, 0.8f, 0.15f}; 
const PIDGains midSpeed     = {1.5f, 0.5f, 0.10f};  
const PIDGains highSpeed    = {1.0f, 0.3f, 0.05f}; 
const PIDGains extremeSpeed = {0.7f, 0.2f, 0.03f}; 

PIDGains currentGains; // Voir ligne 52

// --- Variables de contrôle et de la vitesse verticale (dérivée de l'altitude) ---
float error, lastError, integral, derivative, output; // Variables pour le PID (voir ligne 74 à 81) )
float setpoint = 0.0f; // consigne de vitesse angulaire (rad/s)
float pressure, altitude, lastAltitude, verticalVelocity; // variables pour vitesse verticale (voir ligne 96 à 104)

// Constantes pour optimisation barométrique (pour calculer l'altitude, on va avoir besoin de diviser par la pression atmosphérique, ce qui est long, donc un fais l'inverse pour pouvoir multiplier plus tard)
const float P0 = 1013.25f;
const float INV_P0 = 1.0f / P0;

// --- Timing (300 Hz) ---
unsigned long lastTime;
const unsigned long DT_MICROS = 3333;
const float FIXED_DT = 3333.0f / 1000000.0f; // dt constant pour alléger loop()

// --- Contraintes Servo (output min/max) ---
const float OUT_MIN = -1.0f; // pid limité en 0.26f à vérifier (-1;1?)
const float OUT_MAX = 1.0f;

// --- Variables pour le Filtre Complémentaire, fonction readIMU, angle, coeff et vitesse angulaire ---
float angle_roll = 0.0f;
const float ALPHA = 0.96f;
float rollRate = 0.0f; // pour le PID

// --- Fonctions déportées pour facilitation de la lecture de la boucle principale ---
void updateNavigation(float dt); // Mise à jour de l'altitude et de la vitesse verticale
void selectGains(); // Sélection des gains PID selon la vitesse verticale
void applyOutput(float out); // Application de la sortie PID au servo avec saturation
float getPressure(); // Lecture de la pression atmosphérique par le baromètre SPL06-001
float readIMU();      // Lecture de la vitesse angulaire par le gyroscope ICM42688P

void setup() {
    Serial.begin(115200);
    finServo.attach(SERVO_PIN);
    currentGains = lowSpeed; // Initialisation par défaut (lowSpeed ? et c'est à la fin de la combustion que la vitesse est maximale, n'est ce pas ?)
    lastTime = micros();
    // baromètre SPL06-001
    Wire.begin();       // Indispensable pour l'I2C
    spl06.begin();      // Réveille le baromètre

    // IMU ICM42688P
    if (imu.begin() < 0) {
        Serial.println("Erreur : ICM42688 introuvable !");
    } else { // configuration du mode Expert pour le filtre nattif de l'ICM42688
        Serial.println("ICM42688 connecté. Configuration du mode Expert...");
        imu.setGyroFS(ICM42688::dps2000); 
        imu.setGyroODR(ICM42688::odr1k);
        Serial.println("Gyro configuré : FS=2000, ODR=1kHz");
    }

    // Mémoire Flash W25Q128
    if (!SerialFlash.begin(FLASH_CS_PIN)) {
        Serial.println("Erreur : Mémoire Flash introuvable !");
    }
}

void loop() { // boucle principale optimisée pour 300 Hz, avec contrôle strict du timing et fonctions déportées pour la lisibilité
    unsigned long currentTime = micros();
    
    // Contrôle strict du timing (300 Hz)
    if (currentTime - lastTime >= DT_MICROS) { 
        lastTime = currentTime;

        // 1. Mise à jour Navigation (Altitude/Vitesse)
        updateNavigation(FIXED_DT); // FIXED_DT = dt plus tard dans "updateNavigation"

        // 2. Choix des gains selon la vitesse (Gain Scheduling)
        selectGains();

        // 3. Acquisition Angle pour data logging, mais aussi en même temps mise à jour de la vitesse angulaire pour le PID
        float currentAngle = readIMU(); 

        // 5. Calcul de l'erreur pour le PID
        error = setpoint - rollRate;

        // 6. Calcul PID avec Anti-Windup (Clamping)
        if (!(output >= OUT_MAX && error > 0) && !(output <= OUT_MIN && error < 0)) { // clamping
            integral += error * FIXED_DT; // Calcul de l'intégrale
        }

        derivative = (error - lastError) / FIXED_DT; // Calcul de la dérivée
        lastError = error;

        output = (currentGains.kp * error) + (currentGains.ki * integral) + (currentGains.kd * derivative);

        // 7. Saturation et Envoi au Servo
        applyOutput(output); // output = out plus tard dans "applyOutput"

        // 8. Data Logging (Mémoire Flash Interne)
        struct LogPacket {
            uint32_t time;
            float alt;
            float vel;
            float ang;
        };

        LogPacket packet = { micros(), altitude, verticalVelocity, currentAngle };

        // Si l'adresse dépasse 16 Mo, on arrête d'écrire pour éviter un éventuel plantage.
        if (flashAddress + sizeof(packet) <= 16777216) {
        SerialFlash.write(flashAddress, &packet, sizeof(packet));
        flashAddress += sizeof(packet); // Incrément de l'adresse pour le prochain enregistrement, pour éviter d'écraser les données précédentes
        }
    }
}


// Pression, altitude et vitesse verticale, puis filtre passe-bas de l'altitude
void updateNavigation(float dt) {
    pressure = getPressure();
    float rawAltitude = 44330.0f * (1.0f - powf(pressure * INV_P0, 0.1903f));
    
    // 1. Calcul de la vitesse brute (réactivité maximale)
    float rawVelocity = (rawAltitude - lastAltitude) / dt;

    // 2. Filtrage de la vitesse (optionnel, pour éviter les coups de calculs brusques)
    verticalVelocity = (0.7f * verticalVelocity) + (0.3f * rawVelocity);

    // 3. Mise à jour pour le prochain cycle
    altitude = rawAltitude; // On garde l'altitude pour le log
    lastAltitude = rawAltitude;
}

// Sélection des gains PID en fonction de la vitesse verticale (exemple)
void selectGains() {
    if (verticalVelocity < 50.0f)      currentGains = lowSpeed;
    else if (verticalVelocity < 150.0f) currentGains = midSpeed;
    else if (verticalVelocity < 300.0f) currentGains = highSpeed;
    else                               currentGains = extremeSpeed;
}

void applyOutput(float out) {
    // Saturation (Clamping bis, celui qui maintiens la sortie dans les limites)
    if (out > OUT_MAX) out = OUT_MAX;
    else if (out < OUT_MIN) out = OUT_MIN;
    
    // Conversion directe -1/1 vers 1000/2000 us (plus rapide que map :"int pwmValue = map(output * 100, -100, 100, 1000, 2000);" askip)
    int pwm = 1500 + (int)(out * 500.0f);
    finServo.writeMicroseconds(pwm);
}

// Fonction déportée pour la lecture de la pression
float getPressure() {
    // Lecture de la pression via la bibliothèque ArtronShop
    float p = spl06.pressure();
    
    // Sécurité : si le capteur renvoie une valeur incohérente (0 ou négatif)
    // on renvoie P0 (pression standard) pour ne pas faire planter le PID.
    if (p <= 0) return P0; 
    
    return p;
}

// Fonction déportée pour la lecture de la vitesse angulaire (étonnamment le gyroscope donne naturellement une vitesse angulaire, donc une consigne du même type est plus pratique, pas d'intégration)
float readIMU() {
    // 1. Lecture des données
    imu.getAGT(); 

    // 2. Vitesse de roulis (rad/s)
    float rollRate = imu.getGyroBiasX();

    // 3. Angle de roulis via l'accéléromètre
    float accelRoll = atan2(imu.getAccelBiasY_mss(), imu.getAccelBiasZ_mss());

    // 4. FILTRE COMPLÉMENTAIRE (mais qui sert que pour le datalogging de l'angle, ce qui est crucial après le vol pour savoir ce qui s'est passé, alors que durant on ne se servira que de la vitesse angulaire)
    angle_roll = ALPHA * (angle_roll + rollRate * FIXED_DT) + (1.0f - ALPHA) * accelRoll;

    return angle_roll; 
}