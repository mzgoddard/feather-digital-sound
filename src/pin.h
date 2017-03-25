#pragma once

#include <parts.h>
#include <io.h>

inline static void pin_mux(uint8_t p, uint8_t mux) {
  if (p & 1) {
    PORT->Group[p / 32].PMUX[(p % 32) / 2].bit.PMUXO = mux;
  } else {
    PORT->Group[p / 32].PMUX[(p % 32) / 2].bit.PMUXE = mux;
  }

  PORT->Group[p / 32].PINCFG[p % 32].bit.PMUXEN = 1;
}

inline static void pin_gpio(uint8_t p) {
  PORT->Group[p / 32].PINCFG[p % 32].bit.PMUXEN = 0;
}

inline static void pin_out(uint8_t p) {
  pin_gpio(p);
  PORT->Group[p / 32].DIRSET.reg = (1<<(p % 32));
}

inline static void pin_dir(uint8_t p, bool out) {
  if (out) {
    PORT->Group[p / 32].DIRSET.reg = (1<<(p % 32));
  } else {
    PORT->Group[p / 32].DIRCLR.reg = (1<<(p % 32));
  }
}

inline static void pin_high(uint8_t p) {
  PORT->Group[p / 32].OUTSET.reg = (1<<(p % 32));
}

inline static void pin_low(uint8_t p) {
  PORT->Group[p / 32].OUTCLR.reg = (1<<(p % 32));
}

inline static void pin_toggle(uint8_t p) {
  PORT->Group[p / 32].OUTTGL.reg = (1<<(p % 32));
}

inline static void pin_set(uint8_t p, bool high) {
  if (high) {
    PORT->Group[p / 32].OUTSET.reg = (1<<(p % 32));
  } else {
    PORT->Group[p / 32].OUTCLR.reg = (1<<(p % 32));
  }
}

inline static void pin_in(uint8_t p) {
  pin_gpio(p);
  PORT->Group[p / 32].PINCFG[(p % 32)].bit.INEN = 1;
  PORT->Group[p / 32].DIRCLR.reg = (1<<(p % 32));
}

inline static void pin_pull_up(uint8_t p) {
  pin_in(p);
  PORT->Group[p / 32].PINCFG[(p % 32)].bit.PULLEN = 1;
  pin_high(p);
}

inline static void pin_pull_down(uint8_t p) {
  pin_in(p);
  PORT->Group[p / 32].PINCFG[(p % 32)].bit.PULLEN = 1;
  pin_low(p);
}

inline static void pin_float(uint8_t p) {
  pin_in(p);
  PORT->Group[p / 32].PINCFG[(p % 32)].bit.PULLEN = 0;
}

inline static bool pin_read(uint8_t p) {
  return (PORT->Group[p / 32].IN.reg & (1<<(p % 32))) != 0;
}

inline static void pin_mux_eic(Pin p) {
    if ((p % 32) & 1) {
      PORT->Group[p / 32].PMUX[(p % 32)/2].bit.PMUXO = 0;
    } else {
      PORT->Group[p / 32].PMUX[(p % 32)/2].bit.PMUXE = 0;
    }

    PORT->Group[p / 32].PINCFG[(p % 32)].bit.PMUXEN = 1;
}

inline static u8 pin_extint(uint8_t p) {
  return p % 16;
}
