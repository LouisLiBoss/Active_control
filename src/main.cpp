#include <Arduino.h>
#include <Servo.h>

// --- Configuration Matérielle ---
const int SERVO_PIN = 5; // Pin PWM correspondant à ta ligne bleue
Servo finServo;

// --- Paramètres du PID ---
float Kp = 1.5, Ki = 0.5, Kd = 0.1; 
float error, lastError, integral, derivative;
float setpoint = 0.0; // Angle cible (verticale)
float output;

// --- Timing (300 Hz) ---
unsigned long lastTime;
const unsigned long DT_MICROS = 3333; // 1s / 300Hz = 3333 microseconds

// --- Contraintes Servo PTK 7466W ---
const float OUT_MIN = -1.0; // Saturation basse
const float OUT_MAX = 1.0;  // Saturation haute

void setup() {
    Serial.begin(115200);
    finServo.attach(SERVO_PIN);
    lastTime = micros();
}

void loop() {
    unsigned long currentTime = micros();
    
    // Contrôle strict du timing (300 Hz)
    if (currentTime - lastTime >= DT_MICROS) {
        float dt = (currentTime - lastTime) / 1000000.0;
        lastTime = currentTime;

        // 1. Acquisition (Simulée ici, à remplacer par ton capteur ICM42688P)
        float currentAngle = 0.0; // Lecture de ton filtre complémentaire/Kalman

        // 2. Calcul de l'erreur
        error = setpoint - currentAngle;

        // 3. Calcul de l'intégrale avec CLAMPING (Anti-Windup)
        // On n'ajoute à l'intégrale que si la sortie n'est pas déjà saturée
        if (!(output >= OUT_MAX && error > 0) && !(output <= OUT_MIN && error < 0)) {
            integral += error * dt;
        }

        // 4. Calcul de la dérivée
        derivative = (error - lastError) / dt;
        lastError = error;

        // 5. Calcul de la correction PID
        output = (Kp * error) + (Ki * integral) + (Kd * derivative);

        // 6. Saturation finale (Clamping)
        if (output > OUT_MAX) output = OUT_MAX;
        if (output < OUT_MIN) output = OUT_MIN;

        // 7. Envoi au Servo PTK (Conversion -1/1 vers 1000us/2000us)
        int pwmValue = map(output * 100, -100, 100, 1000, 2000);
        finServo.writeMicroseconds(pwmValue);

        // 8. Télémétrie pour le Data Logger UART
        Serial.print(currentAngle); Serial.print(",");
        Serial.println(output);
    }
}