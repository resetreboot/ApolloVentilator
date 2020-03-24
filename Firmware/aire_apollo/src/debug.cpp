#ifndef DEBUGFUNCC
#define DEBUGFUNCC

#ifdef DEBUG
  #define TRACE(cadena) Serial.println("DEBUG: "  cadena)
#else
  #define TRACE(cadena) {}
#endif


#endif
