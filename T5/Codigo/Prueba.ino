#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include <HWCDC.h>
#include "Config.h"
#include "Logica.h"

HardwareSerial lidar(0), radar(1);
Preferences memoria;
struct Calibracion {
    uint32_t version;
    float bias[3], rollRef, pitchRef, ruido, gravedadRef;
};

Calibracion cal = {};
bool calibrada = false, centroFijado = false, calibrando = false, imuConfigurada = false;
bool imuVista = false, radarVisto = false, lidarVisto = false, lidarValido = false;
uint8_t imuDir = 0x68;
float ax = 0, ay = 0, az = 1, gx = 0, gy = 0, gz = 0;
float roll = 0, pitch = 0, rumbo = 0, velocidadGiro = 0, actividad = 0, normaAnterior = 1;
float giroFiltrado = 0;
uint32_t tIMU = 0, tRadar = 0, tLidar = 0, tActividad = 0, tCal = 0;
uint32_t tMuestra = 0, tEnvio = 0, tReintento = 0, tPasoUS = 0, secuencia = 0;
bool reposo = false, golpe = false;
String mensaje = "INICIANDO";
int progreso = 0;
float suma[5] = {}, sumaCuad[3] = {}, primerA[3] = {}, primerG[3] = {};
float lidarCm = 0;
int lidarAmp = 0;
struct Objetivo {
    float x = 0,
        y = 0,
        vel = 0,
        dist = 0,
        ang = 0;
    bool valido = false;
} objetivos[3];

int cercano = -1, alertaZona = 0, fuente = 0;
float alertaDist = 0, alertaAng = 0;
bool alerta = false;
uint32_t confirmadaMS = 0, motorMS = 0;
Confirmacion confLidar, confRadar;
uint8_t presencia2410 = 0, energiaMov2410 = 0, energiaQuieto2410 = 0;
uint16_t distanciaMov2410 = 0, distanciaQuieto2410 = 0, distanciaDetectada2410 = 0;
float distancia2410 = 0;

bool vigente(bool visto, uint32_t t, uint32_t limite) {
    return visto && uint32_t(millis() - t) <= limite;
}

bool imuOK() {
    return vigente(imuVista, tIMU, CADUCIDAD_IMU_MS);
}

bool radarOK() {
    return vigente(radarVisto, tRadar, CADUCIDAD_SENSOR_MS);
}

bool lidarOK() {
    return vigente(lidarVisto, tLidar, CADUCIDAD_SENSOR_MS);
}

bool orientacionOK() {
    return imuOK() && calibrada && centroFijado && !calibrando && !golpe &&
        fabsf(wrap180(roll - cal.rollRef)) <= MAX_INCLINACION &&
        fabsf(pitch - cal.pitchRef) <= MAX_INCLINACION;
}

bool escribirRegistro(uint8_t r, uint8_t v) {
    Wire.beginTransmission(imuDir);
    Wire.write(r);
    Wire.write(v);
    return Wire.endTransmission() == 0;
}

bool leerRegistros(uint8_t r, uint8_t *b, uint8_t n) {
    Wire.beginTransmission(imuDir);
    Wire.write(r);
    if (Wire.endTransmission(false) != 0)
        return false;
    if (Wire.requestFrom(imuDir, n, true) != n) {
        while (Wire.available()) Wire.read();
        return false;
    }
    for (uint8_t i = 0; i < n; i++) b[i] = Wire.read();
    return true;
}

bool iniciarIMU() {
    uint8_t id = 0;
    for (uint8_t dir : {uint8_t(0x68), uint8_t(0x69)}) {
        imuDir = dir;
        if (!leerRegistros(0x75, &id, 1) || (id & 0x7e) != 0x68)
            continue;
        if (escribirRegistro(0x6b, 1) && escribirRegistro(0x1a, 3) &&
            escribirRegistro(0x19, 9) && escribirRegistro(0x1b, 0) && escribirRegistro(0x1c, 0))
            return true;
    }
    return false;
}

int16_t beSigned(const uint8_t *b) {
    return int16_t((uint16_t(b[0])<<8)|b[1]);
}

void reiniciarVentana() {
    progreso = 0;
    for (float &v : suma) v = 0;
    for (float &v : sumaCuad) v = 0;
}

void comenzarCalibracion() {
    if (!imuOK()) {
        mensaje = "ERROR_IMU";
        return;
    }
    calibrando = true;
    centroFijado = false;
    tCal = millis();
    reiniciarVentana();
    confLidar.limpiar();
    confRadar.limpiar();
    mensaje = "CAL_QUIETO";
}

void pasoCalibracion(float r, float p) {
    float norm = sqrtf(ax * ax + ay * ay + az * az);
    float g[3] = {gx, gy, gz}, a[3] = {ax, ay, az};
    if (!isfinite(norm) || norm < 0.8f || norm > 1.2f) {
        reiniciarVentana();
        mensaje = "CAL_GRAVEDAD_FUERA_RANGO";
        return;
    }
    if (fabsf(cosf(p * DEG_TO_RAD)) <= 0.15f) {
        reiniciarVentana();
        mensaje = "CAL_EJE_X_CASI_VERTICAL";
        return;
    }
    for (int i = 0; i < 3; i++) {
        if (fabsf(g[i]) > 25) {
            reiniciarVentana();
            mensaje = "CAL_GIRO_EXCESIVO";
            return;
        }
        if (progreso && fabsf(a[i] - primerA[i]) > 0.04f) {
            reiniciarVentana();
            mensaje = "CAL_CAMBIO_DE_POSTURA";
            return;
        }
        if (progreso && fabsf(g[i] - primerG[i]) > 1.2f) {
            reiniciarVentana();
            mensaje = "CAL_GIRO_INESTABLE";
            return;
        }
    }
    if (!progreso)
        for (int i = 0; i < 3; i++) {
            primerA[i] = a[i];
            primerG[i] = g[i];
        }
    for (int i = 0; i < 3; i++) {
        suma[i] += g[i];
        sumaCuad[i] += g[i] * g[i];
    }
    suma[3] += sinf(r * DEG_TO_RAD);
    suma[4] += cosf(r * DEG_TO_RAD);
    static float sumaPitch = 0, sumaNorm = 0;
    if (!progreso) {
        sumaPitch = 0;
        sumaNorm = 0;
    }
    sumaPitch += p;
    sumaNorm += norm;
    ++progreso;
    mensaje = "CAL_QUIETO";
    if (progreso < MUESTRAS_CALIBRACION)
        return;
    float ruido2 = 0;
    float biasNuevo[3];
    for (int i = 0; i < 3; i++) {
        float media = suma[i] / progreso;
        float var = fmaxf(0, sumaCuad[i] / progreso - media * media);
        if (var > 0.16f) {
            reiniciarVentana();
            mensaje = "CAL_RUIDO_EXCESIVO";
            return;
        }
        biasNuevo[i] = media;
        ruido2 += var;
    }
    for (int i = 0; i < 3; i++)
        cal.bias[i] = biasNuevo[i];
    cal.version = 2;
    cal.rollRef = atan2f(suma[3], suma[4]) * RAD_TO_DEG;
    cal.pitchRef = sumaPitch / progreso;
    cal.ruido = sqrtf(ruido2);
    cal.gravedadRef = sumaNorm / progreso;
    Calibracion anterior = {};
    bool igual = memoria.getBytesLength("cal") == sizeof(cal) &&
        memoria.getBytes("cal", &anterior, sizeof(anterior)) == sizeof(anterior) &&
        memcmp(&anterior, &cal, sizeof(cal)) == 0;
    size_t escrito = igual ? sizeof(cal) : memoria.putBytes("cal", &cal, sizeof(cal));
    calibrada = true;
    calibrando = false;
    centroFijado = true;
    rumbo = 0;
    giroFiltrado = 0;
    tActividad = millis();
    mensaje = escrito == sizeof(cal) ? "CAL_GUARDADA" : "CAL_RAM_ERROR_GUARDADO";
}

void muestrearIMU() {
    uint8_t b[14];
    if (!imuConfigurada || !leerRegistros(0x3b, b, 14)) {
        imuConfigurada = false;
        imuVista = false;
        centroFijado = false;
        calibrando = false;
        confLidar.limpiar();
        confRadar.limpiar();
        return;
    }
    bool primera = !imuVista;
    ax = beSigned(b) / 16384.0f;
    ay = beSigned(b + 2) / 16384.0f;
    az = beSigned(b + 4) / 16384.0f;
    gx = beSigned(b + 8) / 131.0f;
    gy = beSigned(b + 10) / 131.0f;
    gz = beSigned(b + 12) / 131.0f;
    uint32_t ahoraUS = micros();
    float dt = primera ? 0.01f : uint32_t(ahoraUS - tPasoUS) * 1e-6f;
    tPasoUS = ahoraUS;
    tIMU = millis();
    imuVista = true;
    if (dt > 0.12f) {
        dt = 0.01f;
        centroFijado = false;
    }
    float r = atan2f(ay, az) * RAD_TO_DEG;
    float p = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;
    float norm = sqrtf(ax * ax + ay * ay + az * az);
    float dx = gx - cal.bias[0], dy = gy - cal.bias[1], dz = gz - cal.bias[2];
    float gravedad = calibrada ? cal.gravedadRef : 1;
    golpe = fabsf(norm - gravedad) > 0.25f || fabsf(gx) > 240 || fabsf(gy) > 240 || fabsf(gz) > 240;
    if (primera || calibrando) {
        roll = r;
        pitch = p;
    }
    else {
        float rr = roll * DEG_TO_RAD, pp = pitch * DEG_TO_RAD;
        float cosP = cosf(pp);
        if (fabsf(cosP) > 0.15f) {
            roll = wrap180(roll + (dx + sinf(rr) * tanf(pp) * dy + cosf(rr) * tanf(pp) * dz) * dt);
            pitch += (cosf(rr) * dy - sinf(rr) * dz) * dt;
        }
        if (fabsf(norm - gravedad) < 0.12f) {
            float k = 1 - expf(-dt / 0.5f);
            roll = wrap180(roll + k * wrap180(r - roll));
            pitch += k * (p - pitch);
        }
    }
    float rr = roll * DEG_TO_RAD, pp = pitch * DEG_TO_RAD;
    float cosP = cosf(pp);
    if (fabsf(cosP) <= 0.15f && !calibrando) centroFijado = false;
    velocidadGiro = fabsf(cosP) > 0.15f ? SIGNO_GIRO * (dy * sinf(rr) + dz * cosf(rr)) / cosP : 0;
    float umbral = fmaxf(0.15f, fminf(0.8f, 3 * cal.ruido));
    float entrada = fabsf(velocidadGiro) > umbral ? velocidadGiro : 0;
    if (primera || calibrando || golpe) giroFiltrado = 0;
    else giroFiltrado += (1 - expf(-dt / FILTRO_GIRO_TAU_S)) * (entrada - giroFiltrado);
    if (calibrada && centroFijado && !calibrando && !golpe && fabsf(cosP) > 0.15f)
        rumbo = wrap180(rumbo + giroFiltrado * dt);
    actividad = sqrtf(dx * dx + dy * dy + dz * dz);
    if (actividad > 1.2f || fabsf(norm - gravedad) > 0.06f || fabsf(norm - normaAnterior) > 0.025f) tActividad = millis();
    normaAnterior = norm;
    if (calibrando) pasoCalibracion(r, p);
    if (!centroFijado && !calibrando && fabsf(cosP) > 0.15f) comenzarCalibracion();
}

uint8_t rb[96] = {}, lb[9] = {};
uint8_t rn = 0, ln = 0;
uint32_t tByteRadar = 0, tByteLidar = 0;
void evaluarRadar() {
    cercano = -1;
    for (int i = 0; i < 3; i++) {
        const uint8_t *p = rb + 4 + 8 * i;
        Objetivo &o = objetivos[i];
        o.x = SIGNO_RADAR_X * ldSigned(p) / 10.0f;
        o.y = ldSigned(p + 2) / 10.0f;
        o.vel = ldSigned(p + 4);
        o.dist = hypotf(o.x, o.y);
        o.ang = atan2f(o.x, o.y) * RAD_TO_DEG;
        o.valido = le16(p + 6) > 0 && o.y > 0 && o.dist >= 20 && o.dist <= 600 && fabsf(o.ang) <= 65;
        if (o.valido && (cercano < 0 || o.dist < objetivos[cercano].dist)) cercano = i;
    }
    radarVisto = true;
    tRadar = millis();
    bool v = cercano >= 0 && orientacionOK();
    float d = v ? objetivos[cercano].dist : 0;
    float a = v ? wrap180(rumbo + objetivos[cercano].ang) : 0;
    confRadar.agregar(v && d <= DISTANCIA_ALERTA_CM, d, a, tRadar, CENTRO_GRADOS);
}

void leerRadar() {
#if RADAR_LD2410C
    if (rn && uint32_t(millis() - tByteRadar) > 100) {
        rn = 0;
        confRadar.limpiar();
    }
    while (radar.available()) {
        uint8_t b = radar.read();
        tByteRadar = millis();
        if (rn == sizeof(rb)) {
            memmove(rb, rb + 1, sizeof(rb) - 1);
            rn = sizeof(rb) - 1;
        }
        rb[rn++] = b;
        // Conservar solo una cabecera valida y esperar longitud + payload + cola.
        while (rn >= 4 && !(rb[0] == 0xf4 && rb[1] == 0xf3 && rb[2] == 0xf2 && rb[3] == 0xf1)) {
            memmove(rb, rb + 1, --rn);
        }
        if (rn < 6) continue;
        uint16_t longitud = le16(rb + 4);
        if (longitud < 13 || longitud > sizeof(rb) - 10) {
            confRadar.limpiar();
            memmove(rb, rb + 1, --rn);
            continue;
        }
        if (rn < longitud + 10) continue;
        const uint8_t *p = rb + 6;
        const uint8_t *f = p + longitud;
        bool trama = (p[0] == 1 || p[0] == 2) && p[1] == 0xaa &&
            p[longitud - 2] == 0x55 && p[longitud - 1] == 0 &&
            f[0] == 0xf8 && f[1] == 0xf7 && f[2] == 0xf6 && f[3] == 0xf5;
        if (!trama) {
            confRadar.limpiar();
            memmove(rb, rb + 1, --rn);
            continue;
        }
        radarVisto = true;
        tRadar = millis();
        presencia2410 = p[2];
        distanciaMov2410 = le16(p + 3);
        energiaMov2410 = p[5];
        distanciaQuieto2410 = le16(p + 6);
        energiaQuieto2410 = p[8];
        distanciaDetectada2410 = le16(p + 9);
        bool mov = presencia2410 <= 3 && (presencia2410 & 1) && distanciaMov2410 >= 20 && distanciaMov2410 <= 600;
        bool quieto = presencia2410 <= 3 && (presencia2410 & 2) && distanciaQuieto2410 >= 20 && distanciaQuieto2410 <= 600;
        distancia2410 = mov ? distanciaMov2410 : 0;
        if (quieto && (!mov || distanciaQuieto2410 < distancia2410)) distancia2410 = distanciaQuieto2410;
        confRadar.agregar((mov || quieto) && distancia2410 <= DISTANCIA_ALERTA_CM && orientacionOK(),
            distancia2410, rumbo, tRadar, CENTRO_GRADOS);
        rn = 0;
    }
#else
    if (rn && uint32_t(millis() - tByteRadar) > 100) rn = 0;
    while (radar.available()) {
        uint8_t b = radar.read();
        tByteRadar = millis();
        if (rn == 30) {
            memmove(rb, rb + 1, 29);
            rn = 29;
        }
        rb[rn++] = b;
        if (rn == 30 && rb[0] == 0xaa && rb[1] == 0xff && rb[2] == 3 && rb[3] == 0 && rb[28] == 0x55 && rb[29] == 0xcc) {
            evaluarRadar();
            rn = 0;
        }
    }
#endif
}

void leerLidar() {
    if (ln && uint32_t(millis() - tByteLidar) > 100) ln = 0;
    while (lidar.available()) {
        uint8_t b = lidar.read();
        tByteLidar = millis();
        if (ln == 9) {
            memmove(lb, lb + 1, 8);
            ln = 8;
        }
        lb[ln++] = b;
        if (ln == 9 && lb[0] == 0x59 && lb[1] == 0x59) {
            uint8_t chk = 0;
            for (int i = 0; i < 8; i++) chk += lb[i];
            if (chk != lb[8]) {
                confLidar.limpiar();
                continue;
            }
            lidarCm = le16(lb + 2);
            lidarAmp = le16(lb + 4);
            lidarVisto = true;
            tLidar = millis();
            lidarValido = lidarCm >= DISTANCIA_MIN_CM && lidarCm <= 800 && lidarAmp >= 100 && lidarAmp != 65535;
            confLidar.agregar(lidarValido && lidarCm <= DISTANCIA_ALERTA_CM && orientacionOK(),
                lidarCm, rumbo, tLidar, CENTRO_GRADOS);
            ln = 0;
        }
    }
}

void actualizarAlerta() {
    bool estaba = alerta;
    int zonaAnterior = alertaZona;
    alerta = false;
    fuente = 0;
    alertaDist = 0;
    bool habilitada = orientacionOK() && radarOK() && lidarOK() && !reposo;
    if (!habilitada) {
        confLidar.limpiar();
        confRadar.limpiar();
    }
    if (confLidar.lista && uint32_t(millis() - confLidar.instante) > CADUCIDAD_SENSOR_MS) confLidar.limpiar();
    if (confRadar.lista && uint32_t(millis() - confRadar.instante) > CADUCIDAD_SENSOR_MS) confRadar.limpiar();
    Confirmacion *elegida = nullptr;
    // Cada sensor confirma por separado: TF-Luna detecta objetos estaticos aunque
    // el radar no detecte personas. No se inventa una correspondencia entre ellos.
    if (habilitada && confLidar.lista) {
        elegida = &confLidar;
        fuente = 1;
    }
    if (habilitada && confRadar.lista && (!elegida || confRadar.distancia < elegida->distancia)) {
        elegida = &confRadar;
        fuente = 2;
    }
    if (elegida) {
        alerta = true;
        alertaDist = elegida->distancia;
        alertaAng = elegida->angulo;
        alertaZona = elegida->zona;
#if RADAR_LD2410C
        if (fuente == 2) alertaZona = 9;
#endif
        if (!estaba || zonaAnterior != alertaZona) {
            confirmadaMS = millis();
            motorMS = 0;
        }
    }
    int pins[3] = {MOTOR_IZQ, MOTOR_CENTRO, MOTOR_DER};
    uint32_t periodo = alertaDist <= DISTANCIA_CERCA_CM ? 180 : 400;
    bool pulso = alerta && (alertaDist <= DISTANCIA_URGENTE_CM || uint32_t(millis() - confirmadaMS)%periodo < 100);
    for (int i = 0; i < 3; i++) {
        bool encender = MOTORES_HABILITADOS && pulso && i == alertaZona + 1;
        digitalWrite(pins[i], encender ? HIGH : LOW);
        if (encender && !motorMS) motorMS = millis();
    }
    if (!alerta || !MOTORES_HABILITADOS) motorMS = 0;
}

const char *estado() {
    if (calibrando) return "CALIBRANDO";
    if (!imuOK() || !radarOK() || !lidarOK()) return "FALLO_SENSOR";
    if (!calibrada) return "SIN_CALIBRACION";
    if (!centroFijado) return "ESPERANDO_REFERENCIA";
    if (!orientacionOK()) return "ORIENTACION_INVALIDA";
    if (reposo) return MODO_PC ? "REPOSO_PC" : "AHORRO_CPU";
    return alerta ? "ALERTA" : "MONITOREO";
}

void enviar() {
#if RADAR_LD2410C
    const char *protocolo = "COMB2410";
    float distanciaRadar = radarOK() ? distancia2410 : 0;
    int indiceRadar = -1;
#else
    const char *protocolo = "COMB1";
    float distanciaRadar = radarOK() && cercano >= 0 ? objetivos[cercano].dist : 0;
    int indiceRadar = cercano;
#endif
    puertoPC.printf("%s,%lu,%lu,%s,%d,%d,%d,%d,%d,%d,%d,%.3f,%.3f,%.3f,%d,%d,%d,%.1f,%.1f,%d,%.1f,%d,%.1f,%d,%d,%lu,%lu",
        protocolo,
        (unsigned long)++secuencia, (unsigned long)millis(), estado(), imuOK(), radarOK(), lidarOK(),
        calibrada, centroFijado, calibrando, progreso, rumbo, roll - cal.rollRef, pitch - cal.pitchRef,
        orientacionOK(), actividad > 1.2f || fabsf(normaAnterior - (calibrada ? cal.gravedadRef : 1)) > 0.06f, reposo, velocidadGiro,
        lidarOK() ? lidarCm : 0, lidarOK() ? lidarAmp : 0,
        distanciaRadar, indiceRadar,
        alertaDist, alerta ? alertaZona : 9, fuente, (unsigned long)confirmadaMS, (unsigned long)motorMS);
#if RADAR_LD2410C
    puertoPC.printf(",%d,%d,%d,%d,%d,%d,%.1f,%d,%.4f,%.3f,%.3f,%.3f", radarOK() ? presencia2410 : 0,
        radarOK() ? distanciaMov2410 : 0, radarOK() ? energiaMov2410 : 0,
        radarOK() ? distanciaQuieto2410 : 0, radarOK() ? energiaQuieto2410 : 0,
        radarOK() ? distanciaDetectada2410 : 0, distanciaRadar,
        radarOK() && presencia2410 >= 1 && presencia2410 <= 3,
        sqrtf(ax * ax + ay * ay + az * az), gx, gy, gz);
#else
    for (auto &o : objetivos) puertoPC.printf(",%d,%.1f,%.1f,%.1f", radarOK() && o.valido, o.x, o.y, o.vel);
#endif
    puertoPC.printf(",%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%d,%s\n", cal.bias[0], cal.bias[1], cal.bias[2],
        cal.rollRef, cal.pitchRef, cal.ruido, MOTORES_HABILITADOS, mensaje.c_str());
}

void setup() {
    puertoPC.begin(115200);
    for (int p : {MOTOR_IZQ, MOTOR_CENTRO, MOTOR_DER}) {
        pinMode(p, OUTPUT);
        digitalWrite(p, LOW);
    }
    memoria.begin("comb", false);
    if (memoria.getBytesLength("cal") == sizeof(cal)) {
        memoria.getBytes("cal", &cal, sizeof(cal));
        calibrada = cal.version == 2 && isfinite(cal.rollRef) && isfinite(cal.pitchRef) &&
            isfinite(cal.ruido) && cal.ruido >= 0 && cal.ruido < 2 &&
            isfinite(cal.gravedadRef) && cal.gravedadRef >= 0.8f && cal.gravedadRef <= 1.2f;
        for (float b : cal.bias) if (!isfinite(b) || fabsf(b) > 25) calibrada = false;
    }
    if (!calibrada) cal = {};
    mensaje = calibrada ? "CAL_CARGADA_VERIFICANDO_QUIETO" : "CAL_AUTOMATICA_QUIETO";
    radar.setRxBufferSize(512);
    lidar.setRxBufferSize(256);
    radar.begin(256000, SERIAL_8N1, RADAR_RX, RADAR_TX);
    lidar.begin(115200, SERIAL_8N1, LIDAR_RX, LIDAR_TX);
    Wire.begin(IMU_SDA, IMU_SCL);
    Wire.setClock(400000);
    Wire.setTimeOut(20);
    imuConfigurada = iniciarIMU();
    tActividad = millis();
}

void loop() {
    while (puertoPC.available()) puertoPC.read();
    uint32_t ahora = millis();
    if (!imuConfigurada && uint32_t(ahora - tReintento) >= 1000) {
        tReintento = ahora;
        imuConfigurada = iniciarIMU();
        if (imuConfigurada) mensaje = "IMU_RECUPERADA_CAL_AUTOMATICA";
    }
    if (calibrando && uint32_t(ahora - tCal) > 15000) {
        reiniciarVentana();
        tCal = ahora;
        mensaje = "ESPERANDO_BASTON_QUIETO";
    }
    if (uint32_t(ahora - tMuestra) >= 10) {
        tMuestra = ahora;
        muestrearIMU();
    }
    leerRadar();
    leerLidar();
    bool nuevoReposo = imuOK() && calibrada && !calibrando && uint32_t(ahora - tActividad) >= INACTIVIDAD_MS;
    if (nuevoReposo != reposo) {
        reposo = nuevoReposo;
        if (!MODO_PC) setCpuFrequencyMhz(reposo ? 80 : 160);
    }
    actualizarAlerta();
    uint32_t periodo = !MODO_PC && reposo ? 500 : 50;
    if (uint32_t(ahora - tEnvio) >= periodo) {
        tEnvio = ahora;
        enviar();
    }
    delay(1);
}
