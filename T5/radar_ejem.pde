import processing.serial.*;

Serial puerto;

int estadoObjetivo = 0;
float distanciaMovimiento = 0;
int energiaMovimiento = 0;
float distanciaQuieto = 0;
int energiaQuieto = 0;

float aceleracionX = 0;
float aceleracionY = 0;
float aceleracionZ = 0;
float giroZ = 0;

float distanciaObjetivo = 0;
float distanciaSuave = 0;
float anguloObjetivo = 0;
int ultimoDatoRecibido = 0;

float escala = 1.0;
float suavizado = 0.15;

float anguloBarrido = 0;
float anguloMinimo = -50;
float anguloMaximo = 50;

float zonaMuertaGiro = 1.5;
float sensibilidadGiro = 1.0;
int tiempoAnteriorBarrido = 0;

void setup() {
  size(900, 650);
  frameRate(60);

  println("PUERTOS DISPONIBLES:");
  printArray(Serial.list());

  puerto = new Serial(this, "COM9", 115200);
  puerto.clear();
  puerto.bufferUntil('\n');

  tiempoAnteriorBarrido = millis();
  println("ESP32 conectado correctamente.");
}

void draw() {
  background(20);

  if (estadoObjetivo != 0 && distanciaObjetivo > 0) {
    distanciaSuave = lerp(distanciaSuave, distanciaObjetivo, suavizado);
  }

  actualizarBarrido();

  dibujarRadar();
  dibujarBarrido();
  dibujarObjetivo();
  dibujarSensor();
  mostrarInformacion();
}

void actualizarBarrido() {
  int tiempoActual = millis();
  float dt = (tiempoActual - tiempoAnteriorBarrido) / 1000.0;
  tiempoAnteriorBarrido = tiempoActual;

  dt = constrain(dt, 0, 0.1);

  if (giroZ > zonaMuertaGiro) {
    anguloBarrido += giroZ * dt * sensibilidadGiro;
  } else if (giroZ < -zonaMuertaGiro) {
    anguloBarrido += giroZ * dt * sensibilidadGiro;
  }

  anguloBarrido = constrain(anguloBarrido, anguloMinimo, anguloMaximo);
}

void dibujarRadar() {
  pushMatrix();
  translate(width / 2, height - 45);

  stroke(85);
  strokeWeight(2);
  noFill();

  for (int distancia = 100; distancia <= 500; distancia += 100) {
    float radio = distancia * escala;
    arc(0, 0, radio * 2, radio * 2, PI, TWO_PI);

    fill(170);
    textSize(14);
    text((distancia / 100) + " m", 5, -radio);
    noFill();
  }

  stroke(100);
  line(0, 0, 0, -(500 * escala));

  float longitud = 500 * escala;
  float anguloIzquierda = radians(anguloMinimo);
  float anguloDerecha = radians(anguloMaximo);

  line(0, 0, sin(anguloIzquierda) * longitud, -cos(anguloIzquierda) * longitud);
  line(0, 0, sin(anguloDerecha) * longitud, -cos(anguloDerecha) * longitud);

  popMatrix();

  fill(255);
  textSize(22);
  text("LD2410C - RADAR", 20, 35);

  fill(150);
  textSize(13);
  text("Barrido controlado con MPU6050", 20, 58);
}

void dibujarBarrido() {
  pushMatrix();
  translate(width / 2, height - 45);

  float longitud = 500 * escala;

  for (int i = 5; i >= 1; i--) {
    float anguloEstela = anguloBarrido - (i * 1.5);
    float radianes = radians(anguloEstela);
    float x = sin(radianes) * longitud;
    float y = -cos(radianes) * longitud;
    int transparencia = 20 + ((5 - i) * 20);

    stroke(0, 255, 100, transparencia);
    strokeWeight(2);
    line(0, 0, x, y);
  }

  float radianes = radians(anguloBarrido);
  float x = sin(radianes) * longitud;
  float y = -cos(radianes) * longitud;

  stroke(0, 255, 100);
  strokeWeight(4);
  line(0, 0, x, y);

  popMatrix();
}

void dibujarSensor() {
  pushMatrix();
  translate(width / 2, height - 45);

  noStroke();
  fill(0, 255, 100);
  ellipse(0, 0, 32, 32);

  fill(20);
  ellipse(0, 0, 12, 12);

  fill(0, 210, 80);
  rectMode(CENTER);
  rect(0, 13, 50, 12, 5);
  rectMode(CORNER);

  popMatrix();
}

void dibujarObjetivo() {
  if (estadoObjetivo == 0 || distanciaSuave <= 0) {
    return;
  }

  pushMatrix();
  translate(width / 2, height - 45);

  float distancia = constrain(distanciaSuave, 0, 500);
  float radianes = radians(anguloObjetivo);
  float posicionX = sin(radianes) * distancia * escala;
  float posicionY = -cos(radianes) * distancia * escala;

  noStroke();

  if (estadoObjetivo == 1) {
    fill(255, 80, 80);
  } else if (estadoObjetivo == 2) {
    fill(80, 180, 255);
  } else {
    fill(255, 200, 70);
  }

  ellipse(posicionX, posicionY, 34, 34);

  fill(255);
  textSize(16);
  text(nf(distancia / 100.0, 1, 2) + " m", posicionX + 25, posicionY + 5);

  popMatrix();
}

void mostrarInformacion() {
  fill(255);
  textSize(17);

  String textoEstado = "SIN OBJETIVO";
  if (estadoObjetivo == 1) {
    textoEstado = "MOVIMIENTO";
  } else if (estadoObjetivo == 2) {
    textoEstado = "QUIETO";
  } else if (estadoObjetivo == 3) {
    textoEstado = "MOVIMIENTO + QUIETO";
  }

  text("Estado: " + textoEstado, 20, 100);

  if (estadoObjetivo != 0) {
    text("Distancia: " + nf(distanciaSuave / 100.0, 1, 2) + " m", 20, 130);
  }

  text("Movimiento: " + int(distanciaMovimiento) + " cm", 20, 165);
  text("Energia movimiento: " + energiaMovimiento, 20, 190);
  text("Quieto: " + int(distanciaQuieto) + " cm", 20, 220);
  text("Energia quieto: " + energiaQuieto, 20, 245);

  text("IMU X: " + nf(aceleracionX, 1, 2), 680, 100);
  text("IMU Y: " + nf(aceleracionY, 1, 2), 680, 130);
  text("IMU Z: " + nf(aceleracionZ, 1, 2), 680, 160);
  text("Giro Z: " + nf(giroZ, 1, 2) + " deg/s", 680, 190);

  fill(0, 255, 100);
  text("Barrido: " + nf(anguloBarrido, 1, 1) + " grados", 680, 230);

  String direccion = "CENTRO";
  if (giroZ > zonaMuertaGiro) {
    direccion = "DERECHA";
  } else if (giroZ < -zonaMuertaGiro) {
    direccion = "IZQUIERDA";
  }

  text("Direccion: " + direccion, 680, 260);

  if (millis() - ultimoDatoRecibido > 1000) {
    fill(255, 100, 100);
    text("SIN DATOS DEL ESP32", 650, 310);
  }
}

void serialEvent(Serial puerto) {
  String linea = puerto.readStringUntil('\n');

  if (linea == null) {
    return;
  }

  linea = trim(linea);

  if (!linea.startsWith("RADAR2410,")) {
    return;
  }

  String[] datos = split(linea, ',');

  if (datos.length < 11) {
    return;
  }

  ultimoDatoRecibido = millis();

  estadoObjetivo = int(datos[1]);
  distanciaMovimiento = float(datos[2]);
  energiaMovimiento = int(datos[3]);
  distanciaQuieto = float(datos[4]);
  energiaQuieto = int(datos[5]);

  aceleracionX = float(datos[7]);
  aceleracionY = float(datos[8]);
  aceleracionZ = float(datos[9]);
  giroZ = float(datos[10]);

  if (estadoObjetivo == 0) {
    distanciaObjetivo = 0;
    return;
  }

  if (estadoObjetivo == 1) {
    if (distanciaMovimiento > 0) {
      distanciaObjetivo = distanciaMovimiento;
      anguloObjetivo = anguloBarrido;
    }
  } else if (estadoObjetivo == 2) {
    if (distanciaQuieto > 0) {
      distanciaObjetivo = distanciaQuieto;
      anguloObjetivo = anguloBarrido;
    }
  } else if (estadoObjetivo == 3) {
    if (distanciaMovimiento > 0) {
      distanciaObjetivo = distanciaMovimiento;
      anguloObjetivo = anguloBarrido;
    } else if (distanciaQuieto > 0) {
      distanciaObjetivo = distanciaQuieto;
      anguloObjetivo = anguloBarrido;
    }
  }
}
