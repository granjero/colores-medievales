/*
* ===============================================================
* Máquina Medieval de seleccion por color.
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
 *  Sensor de Color - http://www.w-r-e.de/robotik/data/opt/tcs230.pdf
 */

// S0                                                   // PIN S0 del Sensor de color en HIGH por hardware
// S1                                                   // PIN S1 del Sensor de color en HIGH por hardware
// OUT                                                  // PIN OUT del sensor al pin 5 de arduino según MD_TCS230.h linea 27
// OE                                                   // PIN Output Enable del Sensor en LOW por hardware
#define  S2              12                             // Salida del Sensor de color S2 al pin 12 del micro
#define  S3              13                             // Salida del Sensor de color S3 al pin 13 del micro


/*
 *  Motor Paso a Paso - http://www.dynetics.eu/media/2678/stepper-motors.pdf
 */

// Driver L298n -> OUT1 azul - OUT2 rojo - OUT3 verde - OUT4 negro
#define  MOTOR0           8                             // Pin IN1 del driver paso a pasos
#define  MOTOR1           9                             // Pin IN2 del driver paso a pasos
#define  MOTOR2           10                            // Pin IN3 del driver paso a pasos
#define  MOTOR3           11                            // Pin IN4 del driver paso a pasos
#define  PASOS            200                           // Cantidad de pasos de motor
#define  VELOCIDAD        300                           // Velocidad del Motor rpm

// Motor DC
#define  MOTORADCA        A4 //6                             // Pines
#define  MOTORADCB        A5 //7                             // Pines

// Fines de Carrera
#define  FIN_CARRERA_D    A0                            // Fin de Carrera Derecho
#define  FIN_CARRERA_I    A1                            // Fin de Carrera Izquierdo
//#define  FIN_CARRERA_M    A2                            // Fin de Carrera Rojo
#define  FIN_CARRERA_AB   A2                           // Fin de Carrera Arriba cable amarillo telefono
#define  FIN_CARRERA_AR   A3                            // Fin de Carrera Arriba cable verde telefono

/*
 *  Ultrasonic Sensor HC-SR04 - https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf
 */
#define TRIGGER           6
#define ECHOPIN           7

// Para el switch case del setup (creo)
//#define BLANCO  0
//#define NEGRO   1
//int ByN =     0;

// Inicializa la libreria del sensor de COLOR en CS MT_TCS230.h
MD_TCS230	CS(S2, S3);

// Inicializa la libreria del motor paso a paso stepper.h
Stepper motorPaP(PASOS, MOTOR0, MOTOR1, MOTOR2, MOTOR3);

// define la variables de la librería MT_TCS230
colorData	rgb;
sensorData	sd;
// Valores del programa de calibración
sensorData sdWhite = { 166580, 164720, 191170 };
sensorData sdBlack = { 11500, 11920, 15780 };

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
  {"color 1", {210, 139, 56} },
  {"color 2", {126, 17, 19} },
  {"color 3", {255, 254, 254} },
};


// Variables de estado de la máquina
int  paso           = 0;
int  contador       = 0;
int  colorLeido[4]  = {22,44,33,55};
int  colorDestino   = 0;
int  posicion       = 0;
int  distanciaAlMedio   = 0;
bool calibracion    = false;
bool datoLeido      = false;
bool fdcI           = false;
bool fdcD           = false;
bool fdcM           = false;
bool paso1          = true;

long duration, cm;

/*
* F U N C I O N E S
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
  static  bool  waiting;

  if (!waiting)
  {
    CS.read();
    waiting   = true;
    datoLeido = false;
  }
  else
  {
    if (CS.available())
    {
      CS.getRaw(&sd);
      CS.getRGB(&rgb);
      //Serial.print(F("("));
      //Serial.print(rgb.value[TCS230_RGB_R]);
      //Serial.print(F(","));
      //Serial.print(rgb.value[TCS230_RGB_G]);
      //Serial.print(F(","));
      //Serial.print(rgb.value[TCS230_RGB_B]);
      //Serial.println(F(")"));

      //Serial.print(F("("));
      //Serial.print(rgb.value[0]);
      //Serial.print(F(","));
      //Serial.print(rgb.value[1]);
      //Serial.print(F(","));
      //Serial.print(rgb.value[2]);
      //Serial.println(F(")"));

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
    while( ! digitalRead (FIN_CARRERA_D) )
    {
      motorPaP.step(1);
    }
    while( digitalRead (FIN_CARRERA_D) )
    {
      motorPaP.step(-1);
    }
    motorPaP_OFF();
  }

  motorPaP_OFF();

  fdcD = true;
  fdcI = false;
  fdcM = false;

  delay(1000);
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
    while( digitalRead(FIN_CARRERA_I) )
    {
      motorPaP.step(1);
    }
    motorPaP_OFF();
  }

  motorPaP_OFF();

  fdcD = false;
  fdcI = true;
  fdcM = false;

  delay(1000);
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
      motorPaP.step(distanciaAlMedio);
    }
    if ( fdcD )
    {
      motorPaP.step(-distanciaAlMedio);
    }
  }

  motorPaP_OFF();

  fdcD = false;
  fdcI = false;
  fdcM = true;

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
  while( !digitalRead(FIN_CARRERA_D) )  // Mientras el Fin de Carrera DERECHO esté sin activar
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
  while (digitalRead(FIN_CARRERA_D))
  {
    motorPaP.step(-1);
  }
  motorPaP_OFF();
  Serial.println(F("[FdC DERECHO OK]"));
  //  motorPaP.step(-30);
  delay(500);

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
  while ( digitalRead(FIN_CARRERA_I) )
  {
    motorPaP.step(1);
  }
  //motorPaP.step(30);
  motorPaP_OFF();
  Serial.println(F("[FdC IZQUIERDA OK]"));
  Serial.println(F(""));
  fdcD = false;
  fdcI = true;
  distanciaAlMedio = pasosFCFC / 2;
  delay(1000);
}

// ===========================================================
// Motor Colores, hace algo diferente segun el color
void motorPaP_MueveAColor (int color)
{
  switch (color)
  {
    case 0:
    Serial.println(F("Posición 1 - IZ"));
    motorPaP_Izquirda();
    delay(1000);
    posicion = 1;
    break;

    case 1:
    Serial.println(F("Posición 2 - M"));
    motorPaP_Medio();
    delay(1000);
    posicion = 2;
    break;

    case 2:
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
  delay(150);
  digitalWrite(MOTORADCA, LOW);
  digitalWrite(MOTORADCB, LOW);
  Serial.print(F("BAJA... "));
  // Motor DC baja
  digitalWrite(MOTORADCA, HIGH);
  digitalWrite(MOTORADCB, LOW);
  while ( ! digitalRead (FIN_CARRERA_AB) )
  {
  }
  delay(150);
  // Apaga el motor DC
  digitalWrite(MOTORADCA, LOW);
  digitalWrite(MOTORADCB, LOW);
  delay(250);
  Serial.println(F("OK! "));
}

// ===========================================================
// Motor DC Sube y Baja
void motorDC_BAJA()
{
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
  delay(250);
  Serial.println(F(" OK! "));
}

// ===========================================================
// Lee Sensor Distancia
int sensorDistancia()
{
  long duration, cm;

  digitalWrite(TRIGGER, LOW);
  delayMicroseconds(5);
  digitalWrite(TRIGGER, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER, LOW);
  pinMode(ECHOPIN, INPUT);
  duration = pulseIn(ECHOPIN, HIGH);
  cm = (duration/2) / 29.1;     // Divide by 29.1 or multiply by 0.0343
  return cm;
}


// ===========================================================
// Setup
void setup()
{
  // setea los pines de los fines de carrera como entrada
  pinMode(FIN_CARRERA_D, INPUT);
  pinMode(FIN_CARRERA_I, INPUT);
  //pinMode(FIN_CARRERA_M, INPUT);
  pinMode(FIN_CARRERA_AR, INPUT); // analog
  pinMode(FIN_CARRERA_AB, INPUT); // analog
  pinMode(MOTORADCA, OUTPUT);
  pinMode(MOTORADCB, OUTPUT);
  pinMode(TRIGGER, OUTPUT); // Sets the trigPin as an Output
  pinMode(ECHOPIN, INPUT); // Sets the echoPin as an Input

  Serial.begin(115200);                                                         // inicia comunicacion serie a 115200baudios
  Serial.println(F("[Maquina Medieval de colores]"));
  Serial.println(F("[ver. 0.0.0.1]"));
  Serial.println(F(""));

  motorPaP.setSpeed(VELOCIDAD);

  CS.begin();

  // Calibracion de Colores
  if (digitalRead(FIN_CARRERA_I) && digitalRead(FIN_CARRERA_D))                 // Tocaron el boton de calibración. Por lo que está en uno
  {
    calibracion = true;
  }

  // Entra en modo prueba de Desdcartador de pieza
  else if (digitalRead(FIN_CARRERA_I) && !digitalRead(FIN_CARRERA_D))                 // Tocaron el boton de calibración. Por lo que está en uno
  {
    calibracion = true;
    paso = 7;                                                                   // el paso 7 de calibracion en el loop
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

  // readSensor();
  if (calibracion)
  {
    readSensor();
    switch(paso)
    {
      case 0:
      if ( contador == 0 )
      {
        Serial.println(F("[Modo Calibración]"));
        delay(3000);
        Serial.println(F("\n1. Presentar muestra color BLANCO y pulsar FdC IZ"));
        contador++;
      }
      while( !digitalRead(FIN_CARRERA_I) )
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
        Serial.println(F("\n2. Presentar muestra color NEGRO y pulsar FdC IZ..."));
        contador++;
      }

      while( !digitalRead(FIN_CARRERA_I) )
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
      // if ( contador == 0 )
      // {
      //   Serial.println(F("\n3. Liberar el sensor y pulsar FdC IZ para valor sin muestra..."));
      //   contador++;
      // }
      //
      // while( !digitalRead(FIN_CARRERA_I) )
      // {
      //   // readSensor();
      // }
      //
      // if ( datoLeido )
      // {
      //   Serial.println(F("Linea para copiar -> "));
      //   Serial.print(F("{\"NADA\", {"));
      //   Serial.print(rgb.value[TCS230_RGB_R]);
      //   Serial.print(F(", "));
      //   Serial.print(rgb.value[TCS230_RGB_G]);
      //   Serial.print(F(", "));
      //   Serial.print(rgb.value[TCS230_RGB_B]);
      //   Serial.println("} },");
      //   delay(2000);
      paso++;
      //   contador = 0;
      // }

      break;

      case 3:
      if ( contador == 0 )
      {
        Serial.println(F("\n4. Colocar el primer color y pulsar FdC IZ..."));
        contador++;
      }

      while( !digitalRead(FIN_CARRERA_I) )
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
        Serial.println(F("\n5. Colocar el segundo color y pulsar FdC IZ..."));
        contador++;
      }

      while( !digitalRead(FIN_CARRERA_I) )
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
        Serial.println(F("\n6. Colocar el tercer color y pulsar FdC IZ"));
        contador++;
      }

      while( !digitalRead(FIN_CARRERA_I) )
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

      case 7:
      Serial.println(F("\nTest Descartador de Pieza..."));
      motorDC_DescartaPieza();
      delay(5000);
      break;
    }
  }

  else
  {
    switch (paso)
    {
      case 0:
      if(paso1)
      {
        paso1 = false;
        Serial.println(F("\n{ Esperando Pieza }"));
        //Serial.println(F(" Esperando Pieza "));
        //Serial.print(cm);
        //Serial.println(F(" cm "));
      }

      delay(1000);

      if (sensorDistancia() < 10)
      {
        paso = 1;
        paso1 = true;
      }
      else
      paso = 0;
      contador = 0;
      break;

      case 1:
      readSensor();
      if ( datoLeido && contador <= 4) // Para evitar fallos hace dos lecturas y las compara.
      {
        if (sensorDistancia() > 10)
        {
          paso = 4;
        }
        colorLeido[contador] = colorMatch(&rgb);
        Serial.print(F("\n{ Paso 1 } - Chequeo de color # "));
        // Serial.println(colorLeido[contador]);
        Serial.print(contador);
        Serial.print(F(" - Color leido = "));
        Serial.print(F("("));
        Serial.print(rgb.value[0]);
        Serial.print(F(","));
        Serial.print(rgb.value[1]);
        Serial.print(F(","));
        Serial.print(rgb.value[2]);
        Serial.print(F(") "));
        Serial.print(ct[colorLeido[contador]].name);
        Serial.print(F("  "));
        contador++;
        delay(250);
      }
      if ( contador > 3 )
      {
        if ( (colorLeido[0] == colorLeido[1]) && (colorLeido[2] == colorLeido[3]) && (colorLeido[0] == colorLeido[3]) )
        {
          // if ( colorLeido[0] == 0 ) // Si el color detectado es el de "sin pieza" pasa al paso 3.
          // {
          //   Serial.println(F("Esperando pieza... "));
          //   colorLeido[0]  = 44;
          //   colorLeido[1]  = 14;
          //   paso = 3;
          //   contador = 0;
          //   break;
          // }
          Serial.print(F("\n{ Paso 1 } - Color Confirmado  "));
          Serial.print(colorLeido[0] + 1);
          Serial.print(colorLeido[1] + 1);
          Serial.print(colorLeido[2] + 1);
          Serial.print(colorLeido[3] + 1);

          colorDestino = colorLeido[0];
          colorLeido[0]  = 44;
          colorLeido[1]  = 14;
          colorLeido[2]  = 33;
          colorLeido[3]  = 77;
          Serial.println(F("\nFin Paso 1 "));
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

      case 2:
      Serial.print(F("\n{ Paso 2 }  "));
      // Serial.print(F("Color detectado: "));
      // Serial.println(ct[colorDestino].name);
      Serial.print(F("Moviendo a la posicicón del color...  "));
      motorPaP_MueveAColor(colorDestino);
      Serial.println(F("Fin Paso 2 "));
      paso++;
      break;

      case 3:
      Serial.print(F("\n{ Paso 3 } "));
      Serial.print(F("Descarte de pieza:"));
      motorDC_DescartaPieza();
      Serial.println(F("Fin Paso 3 "));
      paso = 0;
      contador = 0;
      paso1 = true;
      break;

      case 4:
      Serial.println(F("\nProblemas con la Pieza. Se reintenta."));
      paso = 0;
      contador = 0;
      paso1 = true;
      break;

      default:
      paso = 0;
      contador = 0;
      break;
      }
  }
}
