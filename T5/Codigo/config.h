#pragma once
#define RADAR_LD2410C 1

constexpr int RADAR_RX = 4, RADAR_TX = 5;
constexpr int LIDAR_RX = 20, LIDAR_TX = 21;
constexpr int IMU_SDA = 6, IMU_SCL = 7;
constexpr int MOTOR_IZQ = 0, MOTOR_CENTRO = 1, MOTOR_DER = 3;
constexpr bool MOTORES_HABILITADOS = false;
constexpr bool MODO_PC = true;

constexpr float SIGNO_GIRO = -1.0f;
constexpr float SIGNO_RADAR_X = 1.0f; // verificar izquierda/derecha fisicamente
constexpr float CENTRO_GRADOS = 12.0f;
constexpr float MAX_INCLINACION = 25.0f; // respecto a la postura calibrada
constexpr float DISTANCIA_MIN_CM = 20.0f, DISTANCIA_ALERTA_CM = 150.0f;
constexpr float DISTANCIA_CERCA_CM = 100.0f, DISTANCIA_URGENTE_CM = 50.0f;
constexpr uint32_t CADUCIDAD_IMU_MS = 120, CADUCIDAD_SENSOR_MS = 350;
constexpr uint32_t INACTIVIDAD_MS = 30000;
constexpr int MUESTRAS_CALIBRACION = 300; 
constexpr float FILTRO_GIRO_TAU_S = 0.04f; // suavizado de giro, respuesta rapida
