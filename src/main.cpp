/*
* ===============================================================
* Maquina Medieval de seleccion por color.
* Trabajo Práctico Integrador nº2
* Licenciatura en Sistemas de Automatización y Robótica - UNLZ
* ===============================================================
*/

/*
* 2019. Software liberado bajo licencia WTFPL
* No se aceptan reclamos ni devoluciones
*/

#include <MD_TCS230.h>                                  // https://github.com/MajicDesigns/MD_TCS230
#include <FreqCount.h>                                  // https://github.com/PaulStoffregen/FreqCount
#include <Stepper.h>                                    // https://www.arduino.cc/en/Reference/Stepper

/*
 *  Sensor de Color
 *  http://www.w-r-e.de/robotik/data/opt/tcs230.pdf
 */

// S0                                                   // PIN S0 del Sensor de color en HIGH por hardware
// S1                                                   // PIN S1 del Sensor de color en HIGH por hardware
// OUT                                                  // PIN OUT del sensor al pin 5 de arduino según MD_TCS230.h linea 27
// OE                                                   // PIN Output Enable del Sensor en LOW por hardware
#define  S2              12                             // Salida del Sensor de color S2 al pin 12 del micro
#define  S3              13                             // Salida del Sensor de color S3 al pin 13 del micro

/*
 *  Motor Paso a Paso
 *  http://www.dynetics.eu/media/2678/stepper-motors.pdf
 */

// Driver L298n -> OUT1 azul - OUT2 rojo - OUT3 verde - OUT4 negro
#define  MOTOR0           8                             // Pin IN1 del driver paso a pasos
#define  MOTOR1           9                             // Pin IN2 del driver paso a pasos
#define  MOTOR2           10                            // Pin IN3 del driver paso a pasos
#define  MOTOR3           11                            // Pin IN4 del driver paso a pasos
#define  PASOS            200                           // Cantidad de pasos de motor
#define  VELOCIDAD        300                           // Velocidad del Motor rpm

// Motor DC
#define  MOTORADCA        6                             // Pines
#define  MOTORADCB        7                             // Pines

// Fines de Carrera
#define  FIN_CARRERA_D    A0                            // Fin de Carrera Derecho
#define  FIN_CARRERA_I    A1                            // Fin de Carrera Izquierdo
#define  FIN_CARRERA_M    A2                            // Fin de Carrera Rojo
#define  FIN_CARRERA_AB   A3                            // Fin de Carrera Arriba cable amarillo telefono
#define  FIN_CARRERA_AR   A4                            // Fin de Carrera Arriba cable verde telefono

// Para el switch case del setup (creo)
#define BLANCO  0
#define NEGRO   1
//int ByN =     0;

// Inicializa la libreria del sensor de COLOR en CS MT_TCS230.h
MD_TCS230	CS(S2, S3);

// Inicializa la libreria del motor paso a paso stepper.h
Stepper motorPaP(PASOS, MOTOR0, MOTOR1, MOTOR2, MOTOR3);

// define la variables de la librería MT_TCS230
colorData	rgb;
sensorData	sd;
// Valores del programa de calibración
sensorData sdWhite = { 128490, 125410, 144230 };
//sensorData sdWhite = { 125680, 121120, 137210 };
sensorData sdBlack = { 11580, 12090, 16100 };
// sensorData sdBlack = { 10820, 11200, 14900 };

// Estructura de que contie la una tabla de colores para hacer las comparaciones
// Los valores se obtienen del programa de calibracion
typedef struct
{
  char    name[9];  // color name 8+nul
  colorData rgb;    // RGB value
} colorTable;
// Valores del programa de calibración
colorTable ct[] =
{
  { "NADA", {0, 0, 0} },
  //{"NADA", {50, 25, 21} },
  {"color 1", {8, 44, 107} },
  // {"color 1", {10, 46, 109} },
  {"color 2", {97, 9, 12} },
  // {"color 2", {104, 13, 17} },
  {"color 3", {253, 251, 248} },
  // {"color 3", {255, 255, 255} },

};


// Variables de estado de la máquina
int  paso           = 0;
int  contador       = 0;
int  colorLeido[2]  = {22,44};
int  colorDestino   = 0;
int  posicion       = 0;
int  distanciaFdC   = 0;
bool calibracion    = false;
bool datoLeido      = false;
bool fdcI           = false;
bool fdcD           = false;
bool fdcM           = false;


/*
*  ______ _    _ _   _  _____ _____ ____  _   _ ______  _____
* |  ____| |  | | \ | |/ ____|_   _/ __ \| \ | |  ____|/ ____|
* | |__  | |  | |  \| | |      | || |  | |  \| | |__  | (___
* |  __| | |  | | . ` | |      | || |  | | . ` |  __|  \___ \
* | |    | |__| | |\  | |____ _| || |__| | |\  | |____ ____) |
* |_|     \____/|_| \_|\_____|_____\____/|_| \_|______|_____/
*
*/

// ===========================================================
// Compara color leido con la tabla fija Devuelve la posición del array ct del color encontrado.
uint8_t colorMatch(colorData *rgb)
{
  // Root mean square distance between the color and colors in the table.
  // FOr a limited range of colors this method works ok using RGB
  // We don't work out the square root or the mean as it has no effect on the
  // comparison for minimum. Square of the distance is used to ensure that
  // negative distances do not subtract from the total.
  int32_t		d;
  uint32_t	v, minV = 999999L;
  uint8_t		minI;

  for (uint8_t i=0; i<ARRAY_SIZE(ct); i++)
  {
    v = 0;
    for (uint8_t j=0; j<RGB_SIZE; j++)
    {
      d = ct[i].rgb.value[j] - rgb->value[j];
      v += (d * d);
    }
    if (v < minV)	// new best
    {
      minV = v;
      minI = i;
    }
    if (v == 0)		// perfect match, no need to search more
    break;
  }
  // Serial.print(F("colorMatch() - "));
  // Serial.println(ct[minI].name);
  // Serial.println(F(""));

  return(minI);
}

// ===========================================================
// Lee sensor y guarda en &sd y &rgb los valores leidos
void readSensor()
{
  //Serial.println(F("readSensor()"));

  static  bool  waiting;

  if (!waiting)
  {
    CS.read();
    //Serial.println(F("CS.read()"));
    waiting   = true;
    datoLeido = false;
  }
  else
  {
    if (CS.available())
    {
      CS.getRaw(&sd);
      CS.getRGB(&rgb);
      /*Serial.print(F("readSensor() - "));
      Serial.print("RAW [");
      Serial.print(sd.value[0]);
      Serial.print(",");
      Serial.print(sd.value[1]);
      Serial.print(",");
      Serial.print(sd.value[2]);
      Serial.print("] ");

      Serial.print("RGB [");
      Serial.print(rgb.value[TCS230_RGB_R]);
      Serial.print(",");
      Serial.print(rgb.value[TCS230_RGB_G]);
      Serial.print(",");
      Serial.print(rgb.value[TCS230_RGB_B]);
      Serial.println("]"); */
      datoLeido = true;
      waiting   = false;
    }
  }
}

// ===========================================================
// Desliga las bobinas del motor para preservar la circuitería berreta
void motorPaP_OFF ()
{
  digitalWrite(MOTOR0, LOW);
  digitalWrite(MOTOR1, LOW);
  digitalWrite(MOTOR2, LOW);
  digitalWrite(MOTOR3, LOW);
}

// ===========================================================
// Motor busca Fine de Carrera DERECHO
void motorPaP_Derecha ()
{
  // BUSCA FIN DE CARRERA DERECHO
  if ( fdcD )
  {
    // No hace nada porque ya está acá!
  }
  else
  {
    while( !digitalRead(FIN_CARRERA_D) )
    {
      motorPaP.step(1);
    }
    motorPaP.step(200);
    while( digitalRead(FIN_CARRERA_D) )
    {
      motorPaP.step(-1);
    }
    motorPaP.step(-400);
    // Serial.println(F("[FIN DE CARRERA DERECHO OK]"));
    // Serial.println(F(""));
    motorPaP_OFF();
    delay(1000);
  }

  motorPaP_OFF();

  fdcD = true;
  fdcI = false;
  fdcM = false;
}

// ===========================================================
// Motor busca Fin de Carrera IZQUIERDO
void motorPaP_Izquirda ()
{
  if ( fdcI )
  {
    // No hace nada porque ya está acá!
  }
  else
  {
    while( !digitalRead(FIN_CARRERA_I) )
    {
      motorPaP.step(-1);
    }
    motorPaP.step(-200);
    while( digitalRead(FIN_CARRERA_I) )
    {
      motorPaP.step(1);
    }
    motorPaP.step(400);
    // Serial.println(F("[FIN DE CARRERA IZQUIERDA OK]"));
    // Serial.println(F(""));
    motorPaP_OFF();
    delay(1000);
  }

  fdcD = false;
  fdcI = true;
  fdcM = false;
}

// ===========================================================
// Motor busca Fin de Carrera MEDIO
void motorPaP_Medio ()
{
  int contador = 0;

  if (fdcM)
  {
    // No hace nada porque ya está acá!
  }
  else
  {
    if ( fdcI )
    {
      motorPaP.step(distanciaFdC);
      // Serial.println(F("[Buscando FdC MEDIO] ->"));
      // while( !digitalRead(FIN_CARRERA_M) )
      // {
      //   motorPaP.step(1);
      // }
      // motorPaP.step(200);
      // while( digitalRead(FIN_CARRERA_M) )
      // {
      //   motorPaP.step(1);
      //   contador++;
      // }
      // motorPaP.step(-(contador / 2));
    }
    if ( fdcD )
    {
      motorPaP.step(-distanciaFdC);
      // Serial.println(F("[Buscando FdC MEDIO] <-"));
      // while( !digitalRead(FIN_CARRERA_M) )
      // {
      //   motorPaP.step(-1);
      // }
      // motorPaP.step(-200);
      // while( digitalRead(FIN_CARRERA_M) )
      // {
      //   motorPaP.step(-1);
      //   contador++;
      // }
      // motorPaP.step(contador / 2);
    }
  }

  fdcD = false;
  fdcI = false;
  fdcM = true;
  motorPaP_OFF();
  delay(1000);
}

// ===========================================================
// Motor busca Fines de Carrera Para el SETUP
void motorPaP_Setup  ()
{
  Serial.println(F("[CHEQUEANDO FINES DE CARRERA]"));
  int pasosFCFC = 0;
  Serial.print(F("[Busca FdC derecho... ]"));
  // BUSCA FIN DE CARRERA DERECHO
  while( !digitalRead(FIN_CARRERA_D) )
  {
    motorPaP.step(1);
    if (digitalRead(FIN_CARRERA_I))     // Encuentra fin de carrera izquierdo MAL
    {
      Serial.println(F("[BUSCANDO FIN DE CARRERA DERECHO ENCONTRÉ EL IZQUIERDO]"));
      Serial.println(F("[HALT & CATCH FIRE]"));
      motorPaP_OFF();
      while (true)
      {}
    }
  }
  Serial.println(F("[FdC DERECHO OK]"));
  Serial.println(F(""));
  while (digitalRead(FIN_CARRERA_D))
  {
    motorPaP.step(-1);
  }
  motorPaP.step(-30);
  motorPaP_OFF();
  delay(1000);

  // BUSCA FIN DE CARRERA IZQUIERDO
  Serial.print(F("[Busca FdC izquierdo... ]"));
  while( !digitalRead(FIN_CARRERA_I) )
  {
    pasosFCFC++;
    motorPaP.step(-1);
    if (digitalRead(FIN_CARRERA_D))    // Encuentra fin de carrera derecho MAL
    {
      Serial.println(F("[BUSCANDO FIN DE CARRERA IZQUIERDO ENCONTRÉ EL DERECHO]"));
      Serial.println(F("[HALT & CATCH FIRE]"));
      motorPaP_OFF();
      while (true)
      {}
    }
  }

  Serial.println(F("[FdC IZQUIERDA OK]"));
  Serial.println(F(""));
  while ( digitalRead(FIN_CARRERA_I) )
  {
    motorPaP.step(1);
  }
  motorPaP.step(30);
  motorPaP_OFF();
  fdcD = false;
  fdcI = true;
  distanciaFdC = pasosFCFC / 2;
  delay(1000);
}

// ===========================================================
// Motor Colores, hace algo diferente segun el color
void motorPaP_MueveAColor (int color)
{
  switch (color)
  {
    case 0:
    Serial.println(F("SIN LECTURA"));
    motorPaP_OFF();
    posicion = 0;
    break;

    case 1:
    Serial.println(F("Posición 1 - IZ"));
    motorPaP_Izquirda();
    delay(1000);
    posicion = 1;
    break;

    case 2:
    Serial.println(F("Posición 2 - M"));
    motorPaP_Medio();
    delay(1000);
    posicion = 2;
    break;

    case 3:
    Serial.println(F("Posición 3 - D"));
    motorPaP_Derecha();
    delay(1000);
    posicion = 3;
    break;
  }
}

// ===========================================================
// Motor DC Sube y Baja
void motorDC_DescartaPieza()
{
  Serial.print(F("SUBE... "));
  // Motor DC sube
  digitalWrite(MOTORADCA, LOW);
  digitalWrite(MOTORADCB, HIGH);
  while ( ! digitalRead (FIN_CARRERA_AR) )
  {
  }
  Serial.print(F("BAJA... "));
  // Motor DC baja
  digitalWrite(MOTORADCA, HIGH);
  digitalWrite(MOTORADCB, LOW);
  while ( ! digitalRead (FIN_CARRERA_AB) )
  {
  }
  // Apaga el motor DC
  digitalWrite(MOTORADCA, LOW);
  digitalWrite(MOTORADCB, LOW);
  Serial.println(F("OK! "));
}

// ===========================================================
// Setup
void setup()
{
  // setea los pines de los fines de carrera como entrada
  pinMode(FIN_CARRERA_D, INPUT);
  pinMode(FIN_CARRERA_I, INPUT);
  pinMode(FIN_CARRERA_M, INPUT);
  pinMode(FIN_CARRERA_AR, INPUT); // analog
  pinMode(FIN_CARRERA_AB, INPUT); // analog
  pinMode(MOTORADCA, OUTPUT);
  pinMode(MOTORADCB, OUTPUT);

  Serial.begin(115200);                                                         // inicia comunicacion serie a 115200baudios
  Serial.println(F("[Maquina Medieval de colores]"));
  Serial.println(F("[ver. 0.0.0.0.1]"));
  Serial.println(F(""));

  motorPaP.setSpeed(VELOCIDAD);

  CS.begin();

  if (digitalRead(FIN_CARRERA_I) && digitalRead(FIN_CARRERA_D))                 // Tocaron el boton de calibración. Por lo que está en uno
  {
    calibracion = true;
  }
  else                                                                          // Usa los datos de calibracion harcodeados.
  {
    CS.setDarkCal(&sdBlack);
    CS.setWhiteCal(&sdWhite);
    motorPaP_Setup();
    motorDC_DescartaPieza();
  }
}

// ===========================================================
// Loop
void loop()
{

  readSensor();
  if (calibracion)
  {
    readSensor();
    switch(paso)
    {
      case 0:
      if ( contador == 0 )
      {
        Serial.println(F("[Modo Calibración]"));
        Serial.println(F("\n1. Presentar muestra color BLANCO y pulsar FdC medio"));
        contador++;
      }
      while( !digitalRead(FIN_CARRERA_M) )
      {
        // readSensor();
      }
      if ( datoLeido )
      {
        Serial.println(F("Linea para copiar -> "));
        Serial.print(F("sensorData sdWhite = { "));
        Serial.print(sd.value[0]);
        Serial.print(F(", "));
        Serial.print(sd.value[1]);
        Serial.print(F(", "));
        Serial.print(sd.value[2]);
        Serial.println(F(" };"));
        CS.setWhiteCal(&sd);
        delay(2000);
        paso++;
        contador = 0;
      }
      break;

      case 1:
      if ( contador == 0 )
      {
        Serial.println(F("\n2. Presentar muestra color NEGRO y pulsar FdC medio..."));
        contador++;
      }

      while( !digitalRead(FIN_CARRERA_M) )
      {
        // readSensor();
      }
      if ( datoLeido )
      {
        Serial.println(F("Linea para copiar -> "));
        Serial.print(F("sensorData sdBlack = { "));
        Serial.print(sd.value[0]);
        Serial.print(F(", "));
        Serial.print(sd.value[1]);
        Serial.print(F(", "));
        Serial.print(sd.value[2]);
        Serial.println(F(" };"));
        CS.setDarkCal(&sd);
        delay(2000);
        paso++;
        contador = 0;
      }
      break;

      case 2:
      if ( contador == 0 )
      {
        Serial.println(F("\n3. Liberar el sensor y pulsar FdC medio para valor sin muestra..."));
        contador++;
      }

      while( !digitalRead(FIN_CARRERA_M) )
      {
        // readSensor();
      }

      if ( datoLeido )
      {
        Serial.println(F("Linea para copiar -> "));
        Serial.print(F("{\"NADA\", {"));
        Serial.print(rgb.value[TCS230_RGB_R]);
        Serial.print(F(", "));
        Serial.print(rgb.value[TCS230_RGB_G]);
        Serial.print(F(", "));
        Serial.print(rgb.value[TCS230_RGB_B]);
        Serial.println("} },");
        delay(2000);
        paso++;
        contador = 0;
      }
      break;

      case 3:
      if ( contador == 0 )
      {
        Serial.println(F("\n4. Colocar el primer color y pulsar FdC medio..."));
        contador++;
      }

      while( !digitalRead(FIN_CARRERA_M) )
      {
        // readSensor();
      }

      if ( datoLeido )
      {
        Serial.println(F("Linea para copiar -> "));
        Serial.print(F("{\"color 1\", {"));
        Serial.print(rgb.value[TCS230_RGB_R]);
        Serial.print(F(", "));
        Serial.print(rgb.value[TCS230_RGB_G]);
        Serial.print(F(", "));
        Serial.print(rgb.value[TCS230_RGB_B]);
        Serial.println("} },");
        delay(2000);
        paso++;
        contador = 0;
      }
      break;

      case 4:
      if ( contador == 0 )
      {
        Serial.println(F("\n5. Colocar el segundo color y pulsar FdC medio..."));
        contador++;
      }

      while( !digitalRead(FIN_CARRERA_M) )
      {
        // readSensor();
      }

      if ( datoLeido )
      {
        Serial.println(F("Linea para copiar -> "));
        Serial.print(F("{\"color 2\", {"));
        Serial.print(rgb.value[TCS230_RGB_R]);
        Serial.print(F(", "));
        Serial.print(rgb.value[TCS230_RGB_G]);
        Serial.print(F(", "));
        Serial.print(rgb.value[TCS230_RGB_B]);
        Serial.println("} },");
        delay(2000);
        paso++;
        contador = 0;
      }
      break;

      case 5:
      if ( contador == 0 )
      {
        Serial.println(F("\n6. Colocar el tercer color y pulsar FdC medio"));
        contador++;
      }

      while( !digitalRead(FIN_CARRERA_M) )
      {
        // readSensor();
      }

      if ( datoLeido )
      {
        Serial.println(F("Linea para copiar -> "));
        Serial.print(F("{\"color 3\", {"));
        Serial.print(rgb.value[TCS230_RGB_R]);
        Serial.print(F(", "));
        Serial.print(rgb.value[TCS230_RGB_G]);
        Serial.print(F(", "));
        Serial.print(rgb.value[TCS230_RGB_B]);
        Serial.println("} },");
        delay(2000);
        paso++;
        contador = 0;
      }
      break;

      case 6:
      Serial.println(F("\nFIN CALIBRACION. Copiar valores obtenidos y volver a compilar y subir el programa"));
      while ( true )
      {
      }
      break;
    }
  }

  else
  {
    readSensor();
    switch (paso)
    {
      case 0:
      if ( datoLeido && contador <= 1) // Para evitar fallos hace dos lecturas y las compara.
      {
        colorLeido[contador] = colorMatch(&rgb);
        Serial.println(F("\n{ Paso 0 }"));
        Serial.print(F("Color leido = "));
        Serial.println(ct[colorLeido[contador]].name);
        // Serial.println(colorLeido[contador]);
        Serial.print(F("Contador = "));
        Serial.println(contador);
        contador++;
      }
      if ( contador > 1 )
      {
        if ( colorLeido[0] == colorLeido[1] )
        {
          if ( colorLeido[0] == 0 ) // Si el color detectado es el de "sin pieza" pasa al paso 3.
          {
            Serial.println(F("Esperando pieza... "));
            colorLeido[0]  = 44;
            colorLeido[1]  = 14;
            paso = 3;
            contador = 0;
            break;
          }
          Serial.println(F("Color confirmado. \nFin Paso 0"));
          colorDestino = colorLeido[0];
          colorLeido[0]  = 44;
          colorLeido[1]  = 14;
          paso++;
          contador = 0;
        }

        else
        {
          Serial.println(F("Error en el sensado. Se reintenta."));
          contador = 0;
        }
      }
      break;

      case 1:
      Serial.println(F("\n{ Paso 1 }"));
      // Serial.print(F("Color detectado: "));
      // Serial.println(ct[colorDestino].name);
      Serial.println(F("Moviendo a la posicicón del color... "));
      motorPaP_MueveAColor(colorDestino);
      Serial.println(F("Fin Paso 1 "));
      paso++;
      break;

      case 2:
      Serial.println(F("\n{ Paso 2 }"));
      Serial.println(F("Descarte de pieza:"));
      motorDC_DescartaPieza();
      paso = 0;
      contador = 0;
      break;

      case 3:
      Serial.println(F("Esperado pieza, no hago nada."));
      paso = 0;
      contador = 0;
      break;

      default:
      paso = 0;
      contador = 0;
      break;
    }
  }
}
