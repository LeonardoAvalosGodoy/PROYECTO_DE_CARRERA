#pragma once
#include <stdint.h>
#include <math.h>
inline uint16_t le16(const uint8_t *p) { return p[0] | (uint16_t(p[1]) << 8); }

inline int ldSigned(const uint8_t *p) {
  uint16_t v = le16(p);
  return (v & 0x8000) ? int(v & 0x7fff) : -int(v & 0x7fff);
}
inline float wrap180(float a) {
  while (a > 180) a -= 360;
  while (a <= -180) a += 360;
  return a;
}
inline int zonaDe(float angulo, float limite) {
  return angulo < -limite ? -1 : (angulo > limite ? 1 : 0);
}
struct Confirmacion {
  int n = 0, zona = 0;
  float valores[3] = {}, distancia = 0, angulo = 0;
  bool lista = false;
  uint32_t instante = 0;
  void limpiar() { n = 0; lista = false; distancia = 0; }
  // Solo llamar por una NUEVA lectura fisica, nunca por un tick o envio USB.
  void agregar(bool valida, float d, float a, uint32_t ahora, float limite) {
    int z = zonaDe(a, limite);
    if (!valida) { limpiar(); return; }
    if (n && (z != zona || fabsf(d - valores[(n-1)%3]) > 40)) limpiar();
    zona = z;
    valores[n%3] = d;
    if (n < 30000) ++n; else n = 3;
    angulo = a; instante = ahora;
    lista = n >= 3;
    if (lista) {
      float x=valores[0], y=valores[1], z3=valores[2];
      distancia = fmaxf(fminf(x,y), fminf(fmaxf(x,y),z3));
    }
  }
};
