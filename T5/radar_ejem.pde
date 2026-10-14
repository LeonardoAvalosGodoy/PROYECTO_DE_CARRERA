import processing.serial.*;
import java.util.Locale;

// Abrir esta carpeta como sketch de Processing 4. No usa librerias externas.
Serial puerto;
String[] puertos = {};
String puertoActual = "", error = "", ultimoMensaje = "", estado = "SIN CONEXION";
volatile String pendiente = null;
String[] datos = null;
int ultimoPaquete = -10000, seleccionado = 0, lineasMalas = 0;
long secuenciaAnterior = -1;
PrintWriter registro;
boolean grabando = false, vistaSensor = false;
boolean registroAutomatico = true;
int ultimaBusqueda = -2000;
String archivoRegistro = "";
final float CX = 400, CY = 650, RADIO = 350, ESCALA = RADIO/600.0;
float[] estelaAngulos = new float[32];
int[] estelaTiempos = new int[32];
int estelaSiguiente=0, estelaCantidad=0;
final int DURACION_ESTELA_MS=650;

void settings() { size(1180, 780); }
void setup() {
  surface.setTitle("COMB - Banco de prueba USB / LD2410C + TF-Luna + MPU6050");
  textFont(createFont("SansSerif", 15));
  frameRate(40); refrescarPuertos();
}
void refrescarPuertos() {
  puertos = Serial.list(); seleccionado = constrain(seleccionado, 0, max(0, puertos.length-1));
}
void conectar(int i) {
  desconectar();
  if (i < 0 || i >= puertos.length) return;
  try {
    puerto = new Serial(this, puertos[i], 115200);
    puertoActual = puertos[i]; puerto.clear(); puerto.bufferUntil('\n'); error = "";
    ultimoPaquete=millis(); // margen de arranque antes de intentar reconectar
  } catch (Exception e) { error = "No se pudo abrir: " + e.getMessage(); puerto = null; }
}
void desconectar() {
  if(puerto != null) { puerto.stop(); puerto = null; }
  if(grabando) cerrarRegistro();
  pendiente = null; datos = null; ultimoPaquete = -10000; secuenciaAnterior = -1;
  estelaCantidad=0;
}
void serialEvent(Serial p) {
  String s = p.readStringUntil('\n');
  if(s == null) return;
  s = trim(s);
  if(!s.startsWith("COMB1,") && !s.startsWith("COMB2410,")) return;
  // Event serial y draw corren en hilos distintos: draw aplica una instantanea.
  pendiente = s;
}
void recibir() {
  String s = pendiente; pendiente = null;
  if(s == null) return;
  String[] d = split(s, ',');
  if(d.length != 47) { lineasMalas++; return; }
  if(!d[0].equals("COMB1") && !d[0].equals("COMB2410")) { lineasMalas++; return; }
  try {
    for(int i=1; i<46; i++) {
      if(i==3) continue;
      double v = Double.parseDouble(d[i]);
      if(Double.isNaN(v) || Double.isInfinite(v)) throw new Exception();
    }
    long seq = Long.parseLong(d[1]);
    long tiempo = Long.parseLong(d[2]);
    if(seq <= secuenciaAnterior) { ultimoMensaje = "ESP32 reiniciado / secuencia reiniciada"; estelaCantidad=0; }
    secuenciaAnterior = seq;
    datos = d; estado = d[3]; ultimoPaquete = millis();
    actualizarEstela();
    if(grabando) { registro.println(millis()+","+s); registro.flush(); }
  } catch(Exception e) { lineasMalas++; }
}
float valor(int i) { return datos == null ? 0 : Float.parseFloat(datos[i]); }
boolean si(int i) { return valor(i) == 1; }
boolean radar2410() { return datos!=null && datos[0].equals("COMB2410"); }
String nombreRadar() { return radar2410()?"LD2410C":"LD2450"; }
boolean conectado() { return datos != null && millis()-ultimoPaquete < 1000; }
float normalizarAngulo(float a) { return ((a+180)%360+360)%360-180; }
float anguloPantalla(float relativo) { return normalizarAngulo(relativo + (vistaSensor ? 0 : valor(11))); }
boolean enFrente(float a) { return a>=-90 && a<=90; }
boolean direccionDisponible() { return conectado() && si(4) && si(7) && si(8) && !si(9); }
String textoDireccion() {
  if(!conectado()) return "ESPERANDO AL ESP32";
  if(!si(4)) return "SIN LECTURA DEL MPU6050";
  if(si(9) || !si(7) || !si(8)) return "DIRECCION PENDIENTE DE CALIBRAR";
  float a=normalizarAngulo(valor(11));
  return "APUNTA " + (zona(a).equals("CENTRO")?"AL FRENTE":"A LA "+zona(a)) + "  " + nf(a,1,1) + " grados";
}
void dibujarDireccion() {
  noStroke(); fill(22,35,46); rect(24,160,736,92,10);
  fill(151,172,184); textSize(12); text("DIRECCION ESTIMADA DEL BASTON / REFERENCIA INICIAL",40,180);
  fill(direccionDisponible()?color(147,255,174):color(255,190,104));
  textSize(23); text(textoDireccion(),40,211);
  textSize(12); fill(184,200,206);
  if(conectado() && (si(9) || !si(7) || !si(8))) {
    text("Apoyalo inmovil hacia delante. Progreso: "+int(valor(10))+" / 300 muestras. No pulses teclas.",40,235);
  } else if(direccionDisponible()) {
    text(vistaSensor?"Vista relativa: el sensor queda al centro; la direccion real se indica arriba.":"La flecha verde sigue el MPU6050. No representa el lado de una persona detectada.",40,235);
  } else text("La direccion se muestra al recibir datos y completar la calibracion automatica.",40,235);
}
void actualizarEstela() {
  if(!si(4) || !si(7) || !si(8) || si(9)) { estelaCantidad=0; return; }
  float a=vistaSensor?0:normalizarAngulo(valor(11));
  if(!enFrente(a)) { estelaCantidad=0; return; }
  estelaAngulos[estelaSiguiente]=a; estelaTiempos[estelaSiguiente]=millis();
  estelaSiguiente=(estelaSiguiente+1)%32; estelaCantidad=min(32,estelaCantidad+1);
}
void dibujarBarrido(float a) {
  noStroke();
  for(int i=estelaCantidad-1;i>=1;i--) {
    int p=(estelaSiguiente-1-i+64)%32, q=(p+1)%32;
    int edad=millis()-estelaTiempos[p];
    if(edad>DURACION_ESTELA_MS || abs(estelaAngulos[q]-estelaAngulos[p])>30) continue;
    fill(65,255,112,70*(1-edad/(float)DURACION_ESTELA_MS));
    triangle(CX,CY,px(600,estelaAngulos[p]),py(600,estelaAngulos[p]),px(600,estelaAngulos[q]),py(600,estelaAngulos[q]));
  }
  stroke(55,255,112,35); strokeWeight(12); line(CX,CY,px(600,a),py(600,a));
  stroke(65,255,112,100); strokeWeight(5); line(CX,CY,px(600,a),py(600,a));
  stroke(150,255,165); strokeWeight(2); line(CX,CY,px(600,a),py(600,a));
  float tx=px(600,a), ty=py(600,a), bx=px(570,a), by=py(570,a);
  noStroke(); fill(156,255,181);
  triangle(tx,ty,bx+cos(radians(a))*7,by+sin(radians(a))*7,bx-cos(radians(a))*7,by-sin(radians(a))*7);
}
float px(float cm, float grados) { return CX + sin(radians(grados))*cm*ESCALA; }
float py(float cm, float grados) { return CY - cos(radians(grados))*cm*ESCALA; }
String zona(float a) { return a < -12 ? "IZQUIERDA" : (a > 12 ? "DERECHA" : "CENTRO"); }
String nivelDistancia(float cm) {
  if(cm<20 || cm>150) return "FUERA DE RANGO";
  if(cm<=50) return "MUY CERCA";
  if(cm<=100) return "CERCANO";
  return "AVISO TEMPRANO";
}
int colorDistancia(float cm) {
  return cm<=50?color(255,102,106):(cm<=100?color(255,164,78):color(255,220,108));
}
void dibujarNivelAviso() {
  textSize(14);
  if(conectado() && int(valor(24))!=0) {
    fill(colorDistancia(valor(22)));
    text(nivelDistancia(valor(22))+" | "+nf(valor(22)/100,1,2)+" m | aviso desde 1.5 m",28,145);
  } else {
    fill(149,173,184); text("Aviso desde 1.5 m | cercano a 1 m | muy cerca a 0.5 m",28,145);
  }
}
void draw() {
  recibir(); gestionarConexion(); background(13, 22, 30);
  if(conectado() && registroAutomatico && !grabando) iniciarRegistro();
  fill(235); textSize(25); text("COMB | Prueba de sensores", 24, 34);
  fill(145, 169, 184); textSize(13);
  text("Vista frontal 180 grados | " + (vistaSensor ? "relativa al sensor" : "respecto al frente inicial") + " | Clic cambia vista", 24, 58);
  dibujarCuadricula();
  dibujarDireccion();
  if(conectado()) dibujarLecturas();
  else { fill(255, 170, 90); textSize(18); text("ESPERANDO DATOS REALES", 250, 235); }
  dibujarNivelAviso();
  dibujarPanel();
  fill(180); textSize(13);
  text("Inicio automatico: mantener el baston quieto y hacia delante unos 3 segundos. CSV automatico.", 24, 752);
  text("El MPU6050 muestra el barrido del baston. No mide tu trayectoria ni tu posicion.", 24, 772);
}
void dibujarCuadricula() {
  noFill(); strokeWeight(1);
  for(int m=1;m<=6;m++) {
    stroke(34,85,55); arc(CX,CY,m*100*ESCALA*2,m*100*ESCALA*2,PI,TWO_PI);
    fill(127,178,141); textSize(12); text(m+" m",CX+7,CY-m*100*ESCALA+15); noFill();
  }
  stroke(34,85,55);
  for(int a=-60;a<=60;a+=30) line(CX,CY,px(600,a),py(600,a));
  stroke(74,137,90); line(CX-RADIO,CY,CX+RADIO,CY);
  fill(156,205,166); text("FRENTE / 0 grados",CX-59,CY-RADIO-18);
  text("IZQUIERDA / -90",CX-RADIO,CY+27); text("DERECHA / +90",CX+RADIO-96,CY+27);
  for(int a=-60;a<=60;a+=30) {
    if(a==0) continue;
    text((a>0?"+":"")+a,px(625,a)-10,py(625,a));
  }
  fill(100,220,200); noStroke(); circle(CX,CY,14);
  fill(170); text("Baston",CX-21,CY+27);
  textSize(12); text("La estela sigue el movimiento real del baston; no barre sola.",24,716);
}
void dibujarLecturas() {
  if(si(4) && si(8) && si(7) && !si(9)) {
    float a=vistaSensor ? 0 : normalizarAngulo(valor(11));
    if(enFrente(a)) dibujarBarrido(a);
    else { fill(255,178,94); text("Baston fuera de la vista frontal: "+nf(a,1,1)+" grados",28,154); }
    if(!radar2410() && enFrente(a)) {
      // Cono ilustrativo de LD2450, no barrido automatico.
      stroke(62,223,165,50);
      float izq=constrain(a-60,-90,90), der=constrain(a+60,-90,90);
      line(CX,CY,px(600,izq),py(600,izq)); line(CX,CY,px(600,der),py(600,der));
    }
    fill(130,220,186); textSize(12); text("Apuntamiento del modulo",28,100);
    if(enFrente(a) && si(6) && valor(18)>=20 && valor(18)<=600 && valor(19)>=100 && valor(19)!=65535) {
      float d=valor(18); noStroke(); fill(255,192,80); circle(px(d,a),py(d,a),12);
      String etiqueta="TF-Luna "+nf(d/100,1,2)+" m";
      text(etiqueta,constrain(px(d,a)+10,24,760-textWidth(etiqueta)),py(d,a)-8);
    }
  }
  if(radar2410() && si(5) && si(34) && valor(20)>0) {
    // Arco de distancia: el LD2410C no entrega direccion ni posiciones XY.
    noFill(); stroke(255,125,140,150); strokeWeight(2);
    arc(CX,CY,valor(20)*ESCALA*2,valor(20)*ESCALA*2,PI,TWO_PI);
    fill(255,170,175); textSize(13);
    text("LD2410C: presencia a "+nf(valor(20)/100,1,2)+" m; angulo desconocido",28,123);
  }
  if(!radar2410() && si(5)) for(int i=0;i<3;i++) {
    int base=27+4*i;
    if(!si(base)) continue;
    float x=valor(base+1), y=valor(base+2), d=sqrt(x*x+y*y);
    // En vista de referencia se requiere centro fijado, nunca usar yaw invalido.
    if(!vistaSensor && (!si(4) || !si(8) || si(9))) continue;
    float a=anguloPantalla(degrees(atan2(x,y)));
    if(!enFrente(a) || d>600) continue;
    noStroke(); fill(i==int(valor(21)) ? color(255,110,116) : color(131,168,255));
    circle(px(d,a),py(d,a),i==int(valor(21)) ? 16 : 11);
    fill(230); textSize(12);
    String etiqueta="T"+(i+1)+" "+nf(d/100,1,2)+" m";
    text(etiqueta,constrain(px(d,a)+12,24,760-textWidth(etiqueta)),min(CY-8,py(d,a)+14));
  }
}
void fila(String nombre, String v, int y, boolean bien) {
  fill(135,157,172); textSize(14); text(nombre,806,y);
  fill(bien ? color(103,225,164) : color(255,159,108)); text(v,962,y);
}
void dibujarPanel() {
  noStroke(); fill(22,35,46); rect(788,12,380,716,14);
  fill(232); textSize(18); text("Conexion USB",806,42);
  if(puerto == null && !conectado()) {
    fill(175); textSize(14); text("Haz clic en tu puerto para conectar:",806,70);
    for(int i=0;i<min(puertos.length,8);i++) {
      fill(40,62,80); rect(806,85+i*31,332,26,5);
      fill(225); text(puertos[i],818,104+i*31);
    }
    if(puertos.length==0) { fill(255,166,100); text("Sin puertos. Busqueda automatica...",806,105); }
    fill(175); text("Cierra antes el monitor serial de Arduino.",806,360);
    fill(255,158,110); text(error,806,395,340,140);
    return;
  }
  fill(180); textSize(14); text(puertoActual+" / 115200",806,69);
  fill(conectado() ? color(103,225,164) : color(255,159,108));
  text(conectado() ? estado : "SIN DATOS / USB DESCONECTADO",806,97);
  if(!conectado()) { fill(175); text("Esperando datos / reconexion automatica.",806,127); return; }
  fila("MPU6050",si(4)?"OK":"SIN DATOS",132,si(4));
  fila(nombreRadar(),si(5)?"OK":"SIN DATOS",155,si(5));
  fila("TF-Luna",si(6)?"OK":"SIN DATOS",178,si(6));
  fila("Calibracion",si(7)?"DISPONIBLE":"PENDIENTE",209,si(7));
  fila("Centro",si(8)?"AUTOMATICO":"ESPERANDO",232,si(8));
  if(si(9)) {
    fill(255,193,78); text("Calibrando: "+int(valor(10))+"/300. No mover.",806,255);
  } else {
    fila("Giro baston",nf(valor(11),1,1)+" grados",255,si(4) && si(8));
  }
  fila("Zona barrido",si(8)?zona(valor(11)):"SIN REFERENCIA",278,si(8));
  fila("Inclinacion R/P",nf(valor(12),1,1)+" / "+nf(valor(13),1,1),301,si(14));
  fila("Actividad IMU",si(15)?"MOVIMIENTO":"BAJA",324,si(4));
  fill(160); textSize(12); text("Baja actividad no significa que estes detenido.",806,346);
  text("Avanzar recto conserva el angulo aproximadamente.",806,364);
  fill(220); textSize(17); text("Deteccion y alerta",806,397);
  boolean a=int(valor(24))!=0;
  fila("Distancia aviso",a?nf(valor(22)/100,1,2)+" m":"NINGUNA",426,a);
  fila("Fuente",int(valor(24))==1?"TF-Luna":(int(valor(24))==2?nombreRadar():"-"),449,a);
  fila("Motores",si(45)?"HABILITADOS":"SOLO PANTALLA",472,si(45));
  String[] zs={"IZQ","CENTRO","DER"};
  for(int i=0;i<3;i++) {
    fill(a && int(valor(23))==i-1 ? color(255,112,105) : color(47,64,79));
    rect(806+i*112,492,104,37,5); fill(235); textSize(13); text(zs[i],820+i*112,516);
  }
  if(a && radar2410() && int(valor(24))==2) {
    fill(255,169,173); textSize(12); text("Presencia confirmada; direccion desconocida",806,544);
  }
  fill(170); textSize(12);
  if(radar2410()) {
    text("Dist mov/quieto: "+nf(valor(28),1,0)+" / "+nf(valor(30),1,0)+" cm",806,565);
    text("Energia mov/quieto: "+int(valor(29))+" / "+int(valor(31)),806,585);
    text("Mensaje: "+datos[46],806,604,343,20);
    if(si(9) && valor(35)>0) {
      text("G: "+nf(valor(35),1,3)+"  gyro XYZ: "+nf(valor(36),1,1)+" / "+nf(valor(37),1,1)+" / "+nf(valor(38),1,1),806,627);
    }
  } else {
    text("Bias gyro: "+nf(valor(39),1,3)+", "+nf(valor(40),1,3)+", "+nf(valor(41),1,3),806,551);
    text("Mensaje: "+datos[46],806,574,343,46);
  }
  if(!radar2410() || !si(9) || valor(35)<=0) text("Paquetes invalidos: "+lineasMalas,806,625);
  fill(grabando ? color(255,191,79) : color(160));
  text(grabando?"CSV: REGISTRO AUTOMATICO":"Esperando datos para registrar CSV",806,649);
  fill(150); text(ultimoMensaje,806,674,340,44);
}
void mousePressed() {
  if(mouseY>=40 && mouseY<=65 && mouseX<780) { vistaSensor=!vistaSensor; estelaCantidad=0; return; }
  if(puerto != null || mouseX<806 || mouseX>1138) return;
  int i=(mouseY-85)/31;
  if(mouseY>=85 && i>=0 && i<min(puertos.length,8)) conectar(i);
}
void gestionarConexion() {
  if(millis()-ultimaBusqueda<2000) return;
  ultimaBusqueda=millis(); refrescarPuertos();
  if(puerto!=null && millis()-ultimoPaquete>4000) {
    desconectar();
  }
  if(puerto==null) {
    // No probar puertos de otros equipos: reutilizar el elegido o el unico puerto.
    for(int i=0;i<puertos.length;i++) if(puertos[i].equals(puertoActual)) { conectar(i); return; }
    if(puertos.length==1) conectar(0);
  }
}
void iniciarRegistro() {
    try {
      archivoRegistro="registros/prueba_"+year()+nf(month(),2)+nf(day(),2)+"_"+nf(hour(),2)+nf(minute(),2)+nf(second(),2)+".csv";
      registro=createWriter(archivoRegistro);
      String columnasRadar=radar2410() ? "presencia_estado,dist_mov_cm,energia_mov,dist_quieto_cm,energia_quieto,dist_detectada_cm,dist_elegida_cm,presencia,norma_g,gx_raw_dps,gy_raw_dps,gz_raw_dps" : "t1_valido,t1_x_cm,t1_y_cm,t1_vel_cms,t2_valido,t2_x_cm,t2_y_cm,t2_vel_cms,t3_valido,t3_x_cm,t3_y_cm,t3_vel_cms";
      registro.println("pc_ms,protocolo,seq,esp_ms,estado,imu_ok,radar_ok,lidar_ok,calibrada,centro,calibrando,muestras,yaw_deg,roll_rel_deg,pitch_rel_deg,orientacion_ok,actividad,reposo,giro_dps,lidar_cm,amp,radar_dist_cm,indice_radar,alerta_cm,zona,fuente,confirmacion_ms,motor_ms,"+columnasRadar+",bias_x,bias_y,bias_z,roll_ref,pitch_ref,ruido,motores_habilitados,mensaje");
      grabando=true; ultimoMensaje="CSV: "+archivoRegistro;
    } catch(Exception e) { registroAutomatico=false; error="Error CSV: "+e.getMessage(); ultimoMensaje=error; }
}
void cerrarRegistro() { if(registro!=null) { registro.flush(); registro.close(); } grabando=false; }
void exit() { cerrarRegistro(); if(puerto!=null) puerto.stop(); super.exit(); }
