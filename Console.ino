// ===================== CONSOLE SERIE =====================
// ENTER (\r\n ou \n) quand debug tourne => console + stop debug
// Q/q => resume debug
static char lineBuf[24];
static uint8_t lineLen = 0;

#define DEBUG_LEVEL 1



static void printConsoleHelp() {
  Serial.println();
  Serial.println(F("=== Help ==="));
  Serial.println(F("M <val> : Input Mode (1=CPPM, 2=SBUS, 3=IBUS, 4=SUMD, 5=JETI, 6=SRXL, 7=SRXL2, 8=CRSF)"));
  Serial.println(F("C <val> : Channel (1..16)"));
  Serial.println(F("S       : Print configuration"));
  Serial.println(F("Q/q     : Quit Console + Restart debug"));
  Serial.println();
}

static void enterConsoleMode() {
  consoleMode = true;
  dbgRun = false;
  lineLen = 0;
  printConsoleHelp();
}

static void exitConsoleMode() {
  consoleMode = false;
  dbgRun = true;
  lineLen = 0;
  Serial.println(F("\n[Debug resumed]\n"));
}

// ===================== EEPROM load/save =====================
void cfgLoad() {
  EEPROM.get(0, Nv);
}

void cfgSave() {
  EEPROM.put(0, Nv);
}

static void handleConsoleLine(char *s) {
  while (*s == ' ' || *s == '\t') s++;
  if (*s == 0) { Serial.print(F("> ")); return; }

  if ((s[0] == 'Q' || s[0] == 'q') && s[1] == 0) { exitConsoleMode(); return; }

  if ((s[0] == 'S' || s[0] == 's') && s[1] == 0) {
    PRINTF("\r\nInput Mode: %s\r\n", INPUT_MODE_STR[Nv.InputType]);
    PRINTF("Channel   : %d\r\n", Nv.Channel);
    return;
  }

  if (s[0] == 'M' || s[0] == 'm') {
    while (*s && *s != ' ' && *s != '\t') s++;
    while (*s == ' ' || *s == '\t') s++;
    uint8_t h = atoi(s);
    if (h >= 1 && h <= 16) {
      Nv.InputType = h;
      cfgSave();
      PRINTF("\r\nOK Input Mode = %s saved\r\n", INPUT_MODE_STR[Nv.InputType]);
    } else {
      Serial.println(F("\r\nERR Channel (1..8)\r\n"));
    }
    return;
  }

  if (s[0] == 'C' || s[0] == 'c') {
    while (*s && *s != ' ' && *s != '\t') s++;
    while (*s == ' ' || *s == '\t') s++;
    uint8_t h = atoi(s);
    if (h >= 1 && h <= 16) {
      Nv.Channel = h;
      cfgSave();
      Serial.print(F("\r\nOK Channel=saved\r\n"));
    } else {
      Serial.println(F("\r\nERR Channel (1..16)\r\n"));
    }
    return;
  }

  // float sp = atof(s);
  // if (sp >= -20.0f && sp <= 120.0f) {
  //   //TEMP_SET_C = sp;
  //   cfgSave();
  //   //Serial.print(F("OK Set=")); Serial.print(TEMP_SET_C, 1); Serial.println(F("C (saved)"));
  // } else {
  //   Serial.println(F("ERR setpoint (-20..120)"));
  // }
  // Serial.print(F("> "));
}

static void serialService() {
#if DEBUG_LEVEL == 0
  return;
#else
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (!consoleMode) {
      // entrée console sur ENTER
      if (c == '\n') { enterConsoleMode(); continue; }
      // ignore tout le reste
      continue;
    }

    if (c == '\r') continue;
    if (c == '\n') {
      lineBuf[lineLen] = 0;
      handleConsoleLine(lineBuf);
      lineLen = 0;
      continue;
    }

    if (c == 8 || c == 127) { if (lineLen > 0) lineLen--; continue; }
    if (lineLen < sizeof(lineBuf) - 1) lineBuf[lineLen++] = c;
  }
#endif
}
