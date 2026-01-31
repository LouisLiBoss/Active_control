// fichier main testant des fonctions écartées de la boucle prinicpale pour optimisation
#include <Arduino.h>
#include <Servo.h>

// --- Configuration Matérielle ---
const int SERVO_PIN = 5; // Pin connecté au servo PTK 7466W
Servo finServo; // Utilisation de la bibliothèque "Servo"

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
float setpoint = 0.0f; // consigne d'angle
float pressure, altitude, lastAltitude, verticalVelocity; // variables pour vitesse verticale (voir ligne 96 à 104)

// Constantes pour optimisation barométrique (pour calculer l'altitude, on va avoir besoin de diviser par la pression atmosphérique, ce qui est long, donc un fais l'inverse pour pouvoir multiplier plus tard)
const float P0 = 1013.25f;
const float INV_P0 = 1.0f / P0;

// --- Timing (300 Hz) ---
unsigned long lastTime;
const unsigned long DT_MICROS = 3333;
const float FIXED_DT = 3333.0f / 1000000.0f; // dt constant pour alléger loop()

#include <Arduino.h>
#include <Wire.h>
#include "ArtronShop_SPL06-001.h"
#include <ICM42688.h>

ArtronShop_SPL06_001 spl06(0x76, &Wire); 
ICM42688 imu(Wire, 0x68);

// --- Contraintes Servo (output min/max) ---
const float OUT_MIN = -1.0f;
const float OUT_MAX = 1.0f;

// --- Fonctions déportées pour facilitation de la lecture de la boucle principale ---
void updateNavigation(float dt); // Mise à jour de l'altitude et de la vitesse verticale
void selectGains(); // Sélection des gains PID selon la vitesse verticale
void applyOutput(float out); // Application de la sortie PID au servo avec saturation
float getPressure(); // Lecture de la pression atmosphérique par le baromètre SPL06-001
float readIMU();      // Lecture de l'angle par le gyroscope ICM42688P

void setup() {
    Serial.begin(115200);
    finServo.attach(SERVO_PIN);
    currentGains = lowSpeed; // Initialisation par défaut (lowSpeed ? et c'est à la fin de la combustion que la vitesse est maximale, n'est ce pas ?)
    lastTime = micros();
    // baromètre SPL06-001
    Wire.begin();       // Indispensable pour l'I2C
    spl06.begin();      // Réveille le baromètre
}

void loop() {
    unsigned long currentTime = micros();
    
    // Contrôle strict du timing (300 Hz)
    if (currentTime - lastTime >= DT_MICROS) { 
        lastTime = currentTime;

        // 1. Mise à jour Navigation (Altitude/Vitesse)
        updateNavigation(FIXED_DT); // FIXED_DT = dt plus tard dans "updateNavigation"

        // 2. Acquisition Angle et calcul Erreur
        float currentAngle = readIMU(); 
        error = setpoint - currentAngle;

        // 3. Choix des gains selon la vitesse (Gain Scheduling)
        selectGains();

        // 4. Calcul PID avec Anti-Windup (Clamping)
        if (!(output >= OUT_MAX && error > 0) && !(output <= OUT_MIN && error < 0)) { // clamping
            integral += error * FIXED_DT; // Calcul de l'intégrale
        }

        derivative = (error - lastError) / FIXED_DT; // Calcul de la dérivée
        lastError = error;

        output = (currentGains.kp * error) + (currentGains.ki * integral) + (currentGains.kd * derivative);

        // 5. Saturation et Envoi au Servo
        applyOutput(output); // output = out plus tard dans "applyOutput"

        // 6. Data Logging
        Serial.print(altitude); Serial.print(",");
        Serial.print(verticalVelocity); Serial.print(",");
        Serial.print(currentAngle); Serial.print(",");
        Serial.println(output);
    }
}


// Pression, altitude et vitesse verticale, puis filtre passe-bas de l'altitude
void updateNavigation(float dt) {
    pressure = getPressure();
    // Formule barométrique optimisée avec powf (calcul de l'altitude grâce à la pression)
    float rawAltitude = 44330.0f * (1.0f - powf(pressure * INV_P0, 0.1903f)); // rawAltitude : altitude brute avant filtrage, fonction mathématique powf pour les exposants, celui-ci étant dans ce cas 0.1903
    
    // Filtre passe-bas (0.8/0.2) pour lisser la vitesse
    altitude = (0.8f * lastAltitude) + (0.2f * rawAltitude); 
    verticalVelocity = (altitude - lastAltitude) / dt;
    lastAltitude = altitude;
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

// Fonction déportée pour la lecture de l'angle
float readIMU() { // Reste à implémenter la lecture de l'angle (important : le ICM42688P est sensible aux vibrations, donc bon filtrage et éventuellement soft-mounts mécaniques (petits bouts de caoutchouc))
   
    return 0.0f;
}