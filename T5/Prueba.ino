#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Preferences.h>
#include <ld2410.h>
#include "esp_sleep.h"

#define MPU_SDA 6
#define MPU_SCL 7

#define LD_RX 4
#define LD_TX 5
#define LD_OUT 3

#define TF_RX 0
#define TF_TX 1

const int MAX_OBJETIVOS = 8;

const float ANGULO_MINIMO = -90.0;
const float ANGULO_MAXIMO = 90.0;

const float DISTANCIA_TF_MINIMA = 3.0;
const float DISTANCIA_TF_MAXIMA = 200.0;

const float TOLERANCIA_ANGULO = 14.0;
const float TOLERANCIA_DISTANCIA = 45.0;
const float TOLERANCIA_LD_TF = 55.0;

const unsigned long TIEMPO_AHORRO_MS = 20000;
const unsigned long TIEMPO_SUSPENDIDO_MS = 90000;
const unsigned long TIEMPO_SUENO_PROFUNDO_MS = 300000;

const unsigned long INTERVALO_GUARDADO_MS = 15000;
const unsigned long TIEMPO_REDUCCION_CONFIANZA_MS = 12000;
const unsigned long TIEMPO_OBJETIVO_HISTORICO_MS = 30000;

Adafruit_MPU6050 sensorMPU;
ld2410 sensorRadar;
Preferences preferencias;

struct Objetivo {
  bool activo;
  bool confirmado;
  bool historico;
  bool enMovimiento;

  uint8_t id;
  uint8_t confianza;
  uint8_t detecciones;

  float angulo;
  float distancia;

  unsigned long ultimaDeteccion;
  unsigned long ultimaReduccion;
};

struct ObjetivoGuardado {
  uint8_t valido;
  uint8_t enMovimiento;
  uint8_t confianza;
  uint8_t reservado;

  float angulo;
  float distancia;
};

enum Modo {
  NORMAL,
  AHORRO,
  SUSPENDIDO
};

Objetivo objetivos[MAX_OBJETIVOS];
Modo modoActual = NORMAL;

bool mpuDisponible = false;
bool radarDisponible = false;
bool calibracionLista = false;
bool memoriaPendiente = false;
bool presenciaAnterior = false;

float sesgoGiroscopio = 0;
float anguloGrados = 0;
float velocidadGiroGrados = 0;
float distanciaTF = -1;

unsigned long ultimaLecturaImuUs = 0;
unsigned long ultimaLecturaTfMs = 0;
unsigned long ultimoGuardadoMs = 0;
unsigned long ultimaActividadMs = 0;

const char* nombreModo() {
  if (modoActual == NORMAL) {
    return "NORMAL";
  }
  if (modoActual == AHORRO) {
    return "AHORRO";
  }
  return "SUSPENDIDO";
}

unsigned long periodoLecturaTF() {
  if (modoActual == NORMAL) {
    return 70;
  }
  if (modoActual == AHORRO) {
    return 250;
  }
  return 800;
}

unsigned long periodoEnvioDatos() {
  if (modoActual == NORMAL) {
    return 120;
  }
  if (modoActual == AHORRO) {
    return 400;
  }
  return 1000;
}

void guardarConfiguracion() {
  preferencias.begin("baston", false);
  preferencias.putFloat("sesgo", sesgoGiroscopio);
  preferencias.putBool("calibrado", true);
  preferencias.end();

  calibracionLista = true;
}

void guardarObjetivos(bool forzar = false) {
  unsigned long ahora = millis();

  if (!forzar && (!memoriaPendiente || ahora - ultimoGuardadoMs < INTERVALO_GUARDADO_MS)) {
    return;
  }

  ObjetivoGuardado datos[MAX_OBJETIVOS] = {};

  for (int i = 0; i < MAX_OBJETIVOS; i++) {
    Objetivo &objetivo = objetivos[i];

    if (objetivo.activo && objetivo.confirmado && !objetivo.historico) {
      datos[i] = {1,(uint8_t)objetivo.enMovimiento,objetivo.confianza,0,objetivo.angulo,objetivo.distancia };
    }
  }

  preferencias.begin("baston", false);
  preferencias.putBytes("objetivos", datos, sizeof(datos));
  preferencias.end();

  ultimoGuardadoMs = ahora;
  memoriaPendiente = false;

  Serial.println("MEM,GUARDADO");
}

void cargarMemoria() {
  preferencias.begin("baston", true);
  sesgoGiroscopio = preferencias.getFloat("sesgo", 0);
  calibracionLista = preferencias.getBool("calibrado", false);

  ObjetivoGuardado datos[MAX_OBJETIVOS] = {};

  if (preferencias.getBytesLength("objetivos") == sizeof(datos)) {
    preferencias.getBytes("objetivos", datos, sizeof(datos));
  }
  preferencias.end();

  unsigned long ahora = millis();

  for (int i = 0; i < MAX_OBJETIVOS; i++) {
    objetivos[i] = {};
    objetivos[i].id = i + 1;

    if (!datos[i].valido) {
      continue;
    }

    Objetivo &objetivo = objetivos[i];
    objetivo.activo = true;
    objetivo.historico = true;
    objetivo.enMovimiento = datos[i].enMovimiento;
    objetivo.confianza = min((int)datos[i].confianza, 40);
    objetivo.angulo = datos[i].angulo;
    objetivo.distancia = datos[i].distancia;
    objetivo.ultimaDeteccion = ahora;
    objetivo.ultimaReduccion = ahora;
  }

  if (calibracionLista) {
    Serial.printf("CAL,CARGADA,%.4f\n", sesgoGiroscopio);
  }
}

bool iniciarMPU() {
  Wire.begin(MPU_SDA, MPU_SCL);

  if (!sensorMPU.begin(0x68, &Wire)) {
    return false;
  }

  sensorMPU.setGyroRange(MPU6050_RANGE_250_DEG);
  sensorMPU.setAccelerometerRange(MPU6050_RANGE_2_G);
  sensorMPU.setFilterBandwidth(MPU6050_BAND_21_HZ);

  return true;
}

float leerGiroZ() {
  sensors_event_t aceleracion;
  sensors_event_t giro;
  sensors_event_t temperatura;

  sensorMPU.getEvent(&aceleracion, &giro, &temperatura);
  return giro.gyro.z * RAD_TO_DEG;
}

void calibrarGiroscopio() {
  if (!mpuDisponible) {
    return;
  }

  Serial.println("CAL,MANTEN_QUIETO_3_SEGUNDOS");

  float suma = 0;
  const int CANTIDAD_MUESTRAS = 600;

  for (int i = 0; i < CANTIDAD_MUESTRAS; i++) {
    sensorRadar.read();
    suma += leerGiroZ();
    delay(5);
  }

  sesgoGiroscopio = suma / CANTIDAD_MUESTRAS;
  anguloGrados = 0;

  ultimaLecturaImuUs = micros();
  ultimaActividadMs = millis();

  guardarConfiguracion();

  Serial.printf("CAL,OK,%.4f\n", sesgoGiroscopio);
}

void actualizarIMU() {
  if (!mpuDisponible) {
    return;
  }

  unsigned long ahora = micros();

  if (ultimaLecturaImuUs == 0) {
    ultimaLecturaImuUs = ahora;
    return;
  }

  float tiempoTranscurrido = (ahora - ultimaLecturaImuUs) / 1000000.0f;

  if (tiempoTranscurrido < 0.008f) {
    return;
  }

  if (tiempoTranscurrido > 0.08f) {
    tiempoTranscurrido = 0.02f;
  }

  ultimaLecturaImuUs = ahora;
  velocidadGiroGrados = leerGiroZ() - sesgoGiroscopio;

  if (fabs(velocidadGiroGrados) < 0.35f) {
    velocidadGiroGrados = 0;
  }

  anguloGrados = constrain(
    anguloGrados + velocidadGiroGrados * tiempoTranscurrido,
    -110.0f,
    110.0f
  );

  if (fabs(velocidadGiroGrados) > 1.5f) {
    ultimaActividadMs = millis();
  }
}

float leerTF() {
  const uint8_t comando[] = { 0x55, 0xAA, 0x81, 0x00, 0xFA };

  while (Serial0.available()) {
    Serial0.read();
  }

  Serial0.write(comando, sizeof(comando));
  Serial0.flush();

  uint8_t respuesta[8] = {};
  int cantidadBytes = 0;
  unsigned long inicioEspera = millis();

  while (millis() - inicioEspera < 45 && cantidadBytes < 8) {
    sensorRadar.read();

    while (Serial0.available() && cantidadBytes < 8) {
      uint8_t datoRecibido = Serial0.read();

      if (cantidadBytes == 0 && datoRecibido != 0x55) {
        continue;
      }

      if (cantidadBytes == 1 && datoRecibido != 0xAA) {
        cantidadBytes = 0;
        continue;
      }

      respuesta[cantidadBytes++] = datoRecibido;
    }
  }

  if (cantidadBytes != 8 ||
      respuesta[2] != 0x81 ||
      respuesta[3] != 0x03 ||
      respuesta[6] != 0 ||
      respuesta[7] != 0xFA) {
    return -1;
  }

  float centimetros = (((uint16_t)respuesta[4] << 8) | respuesta[5]) / 10.0f;

  if (centimetros >= DISTANCIA_TF_MINIMA && centimetros <= DISTANCIA_TF_MAXIMA) {
    return centimetros;
  }

  return -1;
}

bool coincideConRadar(float distanciaTfActual, bool &enMovimiento, uint8_t &energia) {
  float mejorDiferencia = 9999;
  enMovimiento = false;
  energia = 0;

  if (sensorRadar.movingTargetDetected()) {
    float distanciaRadar = sensorRadar.movingTargetDistance();
    float diferencia = fabs(distanciaTfActual - distanciaRadar);

    if (distanciaRadar > 0 && diferencia < mejorDiferencia) {
      mejorDiferencia = diferencia;
      enMovimiento = true;
      energia = sensorRadar.movingTargetEnergy();
    }
  }

  if (sensorRadar.stationaryTargetDetected()) {
    float distanciaRadar = sensorRadar.stationaryTargetDistance();
    float diferencia = fabs(distanciaTfActual - distanciaRadar);

    if (distanciaRadar > 0 && diferencia < mejorDiferencia) {
      mejorDiferencia = diferencia;
      enMovimiento = false;
      energia = sensorRadar.stationaryTargetEnergy();
    }
  }

  return mejorDiferencia <= TOLERANCIA_LD_TF;
}

int buscarObjetivo(float angulo, float distancia) {
  int mejorIndice = -1;
  float mejorPuntuacion = 9999;

  for (int i = 0; i < MAX_OBJETIVOS; i++) {
    Objetivo &objetivo = objetivos[i];

    if (!objetivo.activo) {
      continue;
    }

    float diferenciaAngulo = fabs(objetivo.angulo - angulo);
    float diferenciaDistancia = fabs(objetivo.distancia - distancia);

    if (diferenciaAngulo > TOLERANCIA_ANGULO || diferenciaDistancia > TOLERANCIA_DISTANCIA) {
      continue;
    }

    float puntuacion = (diferenciaAngulo / TOLERANCIA_ANGULO) + (diferenciaDistancia / TOLERANCIA_DISTANCIA);

    if (puntuacion < mejorPuntuacion) {
      mejorPuntuacion = puntuacion;
      mejorIndice = i;
    }
  }

  return mejorIndice;
}

int buscarEspacioDisponible() {
  for (int i = 0; i < MAX_OBJETIVOS; i++) {
    if (!objetivos[i].activo) {
      return i;
    }
  }

  int indiceMenorConfianza = 0;

  for (int i = 1; i < MAX_OBJETIVOS; i++) {
    if (objetivos[i].confianza < objetivos[indiceMenorConfianza].confianza) {
      indiceMenorConfianza = i;
    }
  }

  if (objetivos[indiceMenorConfianza].confianza < 75) {
    return indiceMenorConfianza;
  }

  return -1;
}

void actualizarObjetivo(
  int indice,
  float angulo,
  float distancia,
  bool coincidenciaSensores,
  bool enMovimiento,
  uint8_t energia
) {
  Objetivo &objetivo = objetivos[indice];

  bool eraHistorico = objetivo.historico;
  float anguloAnterior = objetivo.angulo;
  float distanciaAnterior = objetivo.distancia;

  if (objetivo.detecciones == 0 || eraHistorico) {
    objetivo.angulo = angulo;
    objetivo.distancia = distancia;
  } else {
    objetivo.angulo += (angulo - objetivo.angulo) * 0.35f;
    objetivo.distancia += (distancia - objetivo.distancia) * 0.35f;
  }

  objetivo.activo = true;
  objetivo.historico = false;
  objetivo.enMovimiento = enMovimiento;
  objetivo.ultimaDeteccion = millis();
  objetivo.ultimaReduccion = millis();

  if (objetivo.detecciones < 255) {
    objetivo.detecciones++;
  }

  int aumentoConfianza = coincidenciaSensores ? (12 + energia / 20) : 6;
  objetivo.confianza = min(100, (int)objetivo.confianza + aumentoConfianza);

  if (objetivo.detecciones >= 3 && objetivo.confianza >= 60) {
    objetivo.confirmado = true;
  }

  bool cambioImportante = fabs(objetivo.angulo - anguloAnterior) > 2.5f ||
                          fabs(objetivo.distancia - distanciaAnterior) > 8 ||
                          eraHistorico;

  if (cambioImportante) {
    ultimaActividadMs = millis();

    if (objetivo.confirmado) {
      memoriaPendiente = true;
    }
  }
}

void crearObjetivo(float angulo, float distancia, bool enMovimiento, uint8_t energia) {
  int indice = buscarEspacioDisponible();

  if (indice < 0) {
    return;
  }

  objetivos[indice] = {};

  Objetivo &objetivo = objetivos[indice];
  objetivo.id = indice + 1;
  objetivo.activo = true;
  objetivo.enMovimiento = enMovimiento;
  objetivo.angulo = angulo;
  objetivo.distancia = distancia;
  objetivo.detecciones = 1;
  objetivo.confianza = constrain(30 + energia / 5, 30, 50);
  objetivo.ultimaDeteccion = millis();
  objetivo.ultimaReduccion = millis();

  ultimaActividadMs = millis();
}

void procesarDeteccion(float angulo, float distanciaTfActual) {
  if (!radarDisponible || !sensorRadar.isConnected() || !sensorRadar.presenceDetected()) {
    return;
  }

  if (angulo < ANGULO_MINIMO ||
      angulo > ANGULO_MAXIMO ||
      distanciaTfActual < DISTANCIA_TF_MINIMA ||
      distanciaTfActual > DISTANCIA_TF_MAXIMA) {
    return;
  }

  bool enMovimiento = false;
  uint8_t energia = 0;

  bool coincidenciaSensores = coincideConRadar(distanciaTfActual, enMovimiento, energia);
  int indice = buscarObjetivo(angulo, distanciaTfActual);

  if (indice >= 0) {
    if (coincidenciaSensores || objetivos[indice].confirmado || objetivos[indice].historico) {
      actualizarObjetivo(
        indice,
        angulo,
        distanciaTfActual,
        coincidenciaSensores,
        enMovimiento,
        energia
      );
    }
  } else if (coincidenciaSensores) {
    crearObjetivo(angulo, distanciaTfActual, enMovimiento, energia);
  }
}

void reducirConfianzaObjetivos() {
  unsigned long ahora = millis();

  for (int i = 0; i < MAX_OBJETIVOS; i++) {
    Objetivo &objetivo = objetivos[i];

    if (!objetivo.activo) {
      continue;
    }

    unsigned long tiempoSinDetectar = ahora - objetivo.ultimaDeteccion;

    if (objetivo.historico) {
      if (tiempoSinDetectar > TIEMPO_OBJETIVO_HISTORICO_MS) {
        objetivo.activo = false;
        Serial.printf("TARGET_OFF,%d\n", objetivo.id);
      }
      continue;
    }

    if (tiempoSinDetectar > TIEMPO_REDUCCION_CONFIANZA_MS && ahora - objetivo.ultimaReduccion >= 1000) {
      objetivo.ultimaReduccion = ahora;

      if (objetivo.confianza > 3) {
        objetivo.confianza -= 3;
      } else {
        objetivo.confianza = 0;
      }

      if (objetivo.confianza == 0) {
        objetivo.activo = false;
        objetivo.confirmado = false;
        memoriaPendiente = true;
        Serial.printf("TARGET_OFF,%d\n", objetivo.id);
      }
    }
  }
}

void cambiarModo(Modo nuevoModo) {
  if (nuevoModo == modoActual) {
    return;
  }

  modoActual = nuevoModo;
  Serial.printf("MODE,%s\n", nombreModo());

  if (modoActual != NORMAL) {
    guardarObjetivos(true);
  }
}

void entrarSuenoProfundo() {
  if (digitalRead(LD_OUT) == HIGH) {
    Serial.println("POWER,NO_DUERME_PRESENCIA");
    return;
  }

  guardarObjetivos(true);
  guardarConfiguracion();

  Serial.println("POWER,DEEP_SLEEP");
  Serial.flush();

  delay(100);

  esp_deep_sleep_enable_gpio_wakeup(1ULL << LD_OUT, ESP_GPIO_WAKEUP_GPIO_HIGH);
  esp_deep_sleep_start();
}

void actualizarModoEnergia() {
  unsigned long ahora = millis();
  bool hayPresencia = false;

  if (radarDisponible && sensorRadar.isConnected() && sensorRadar.presenceDetected()) {
    hayPresencia = true;
  }

  if (hayPresencia != presenciaAnterior) {
    presenciaAnterior = hayPresencia;
    ultimaActividadMs = ahora;
    cambiarModo(NORMAL);
  }

  unsigned long tiempoSinActividad = ahora - ultimaActividadMs;

  if (tiempoSinActividad < TIEMPO_AHORRO_MS) {
    cambiarModo(NORMAL);
  } else if (tiempoSinActividad < TIEMPO_SUSPENDIDO_MS) {
    cambiarModo(AHORRO);
  } else {
    cambiarModo(SUSPENDIDO);
  }

  if (!hayPresencia && tiempoSinActividad >= TIEMPO_SUENO_PROFUNDO_MS && digitalRead(LD_OUT) == LOW) {
    entrarSuenoProfundo();
  }
}

void enviarDatos() {
  bool hayPresencia = false;
  int distanciaMovimiento = 0;
  int distanciaEstatica = 0;

  if (radarDisponible && sensorRadar.isConnected()) {
    if (sensorRadar.presenceDetected()) {
      hayPresencia = true;
    }
    if (sensorRadar.movingTargetDetected()) {
      distanciaMovimiento = sensorRadar.movingTargetDistance();
    }
    if (sensorRadar.stationaryTargetDetected()) {
      distanciaEstatica = sensorRadar.stationaryTargetDistance();
    }
  }

  int distanciaGeneral = 0;

  if (distanciaMovimiento > 0 && distanciaEstatica > 0) {
    distanciaGeneral = min(distanciaMovimiento, distanciaEstatica);
  } else {
    distanciaGeneral = max(distanciaMovimiento, distanciaEstatica);
  }

  Serial.printf(
    "RADAR,%.2f,%d,%d,%d,%d\n",
    anguloGrados,
    hayPresencia,
    distanciaGeneral,
    distanciaMovimiento,
    distanciaEstatica
  );

  Serial.printf("TOF,%.1f\nMODE,%s\n", distanciaTF, nombreModo());

  for (int i = 0; i < MAX_OBJETIVOS; i++) {
    Objetivo &objetivo = objetivos[i];

    if (!objetivo.activo) {
      continue;
    }

    Serial.printf(
      "TARGET,%d,%.2f,%.1f,%d,%d,%d,%d\n",
      objetivo.id,
      objetivo.angulo,
      objetivo.distancia,
      objetivo.confianza,
      objetivo.enMovimiento,
      objetivo.confirmado,
      objetivo.historico
    );
  }
}

void setup() {
  pinMode(LD_OUT, INPUT);

  Serial.begin(115200);
  delay(800);

  Serial0.begin(115200, SERIAL_8N1, TF_RX, TF_TX);
  Serial1.begin(256000, SERIAL_8N1, LD_RX, LD_TX);

  radarDisponible = sensorRadar.begin(Serial1);
  mpuDisponible = iniciarMPU();

  cargarMemoria();

  Serial.printf("BOOT,LD2410C,%s\n", radarDisponible ? "OK" : "ERROR");
  Serial.printf("BOOT,MPU6050,%s\n", mpuDisponible ? "OK" : "ERROR");

  if (mpuDisponible && !calibracionLista) {
    calibrarGiroscopio();
  }

  anguloGrados = 0;
  ultimaLecturaImuUs = micros();
  ultimaActividadMs = millis();

  presenciaAnterior = radarDisponible && sensorRadar.isConnected() && sensorRadar.presenceDetected();

  Serial.println("CAL,CENTRO_INICIAL_0");
}

void loop() {
  sensorRadar.read();
  actualizarIMU();

  unsigned long ahora = millis();

  if (ahora - ultimaLecturaTfMs >= periodoLecturaTF()) {
    ultimaLecturaTfMs = ahora;
    distanciaTF = leerTF();

    if (distanciaTF > 0) {
      procesarDeteccion(anguloGrados, distanciaTF);
    }
  }

  reducirConfianzaObjetivos();
  actualizarModoEnergia();
  guardarObjetivos();

  static unsigned long ultimoEnvioMs = 0;

  if (ahora - ultimoEnvioMs >= periodoEnvioDatos()) {
    ultimoEnvioMs = ahora;
    enviarDatos();
  }

  delay(1);
}
