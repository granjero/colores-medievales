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

/*                                            _
 *                                           | |
 *  ___  ___ _ __  ___  ___  _ __    ___ ___ | | ___  _ __
 * / __|/ _ \ '_ \/ __|/ _ \| '__|  / __/ _ \| |/ _ \| '__|
 * \__ \  __/ | | \__ \ (_) | |    | (_| (_) | | (_) | |
 * |___/\___|_| |_|___/\___/|_|     \___\___/|_|\___/|_|
 *
 *  http://www.w-r-e.de/robotik/data/opt/tcs230.pdf
*/


// S0                                                   // PIN S0 del Sensor de color en HIGH por hardware
// S1                                                   // PIN S1 del Sensor de color en HIGH por hardware
#define  S2              12                             // Salida del Sensor de color S2 al pin 12 del micro
#define  S3              13                             // Salida del Sensor de color S3 al pin 13 del micro

// Motor Paso a Paso
#define  MOTOR0           8                             // Pines
#define  MOTOR1           9
#define  MOTOR2           10
#define  MOTOR3           11
#define  PASOS            200                           // Pasos de motor
#define  VELOCIDAD        300                           // Velocidad del Motor rpm

// Motor DC
#define  MOTORADCA        A0                             // Pines
#define  MOTORADCB        A1                             // Pines

// Fines de Carrera
#define  FIN_CARRERA_D    2                             // Fin de Carrera Derecho
#define  FIN_CARRERA_I    3                             // Fin de Carrera Izquierdo
#define  FIN_CARRERA_M    4                             // Fin de Carrera Rojo
#define  FIN_CARRERA_AB   A4                            // Fin de Carrera Arriba
#define  FIN_CARRERA_AR   A5                            // Fin de Carrera Arriba

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
sensorData sdWhite = { 125680, 121120, 137210 };
sensorData sdBlack = { 10820, 11200, 14900 };

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
{"NADA", {50, 25, 21} },                // NEGRO
{"color 1", {10, 46, 109} },            // AZUL
{"color 2", {104, 13, 17} },            // ROJO
{"color 3", {255, 255, 255} },          // BLANCO
};


// Variables de estado de la máquina
int  paso           = 0;
int  contador       = 0;
int  colorLeido[2]  = {22,44};
int  colorDestino   = 0;
int  posicion       = 0;
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
      // Serial.println(F("[Buscando FdC MEDIO] ->"));
      while( !digitalRead(FIN_CARRERA_M) )
      {
        motorPaP.step(1);
      }
      motorPaP.step(200);
      while( digitalRead(FIN_CARRERA_M) )
      {
        motorPaP.step(1);
        contador++;
      }
      motorPaP.step(-(contador / 2));
    }
    if ( fdcD )
    {
      Serial.println(F("[Buscando FdC MEDIO] <-"));
      while( !digitalRead(FIN_CARRERA_M) )
      {
        motorPaP.step(-1);
      }
      motorPaP.step(-200);
      while( digitalRead(FIN_CARRERA_M) )
      {
        motorPaP.step(-1);
        contador++;
      }
      motorPaP.step(contador / 2);
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
  Serial.println(F("[Busca FdC derecho... ]"));
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
  Serial.println(F("[Busca FdC izquierdo... ]"));
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
  while ( ! digitalRead (FIN_CARRERA_AR) )
  {
    digitalWrite(MOTORADCA, LOW);
    digitalWrite(MOTORADCB, HIGH);
  }

  Serial.println(F("BAJA... "));
  while ( ! digitalRead (FIN_CARRERA_AB) )
  {
    digitalWrite(MOTORADCA, HIGH);
    digitalWrite(MOTORADCB, LOW);
  }
  // Apaga el motor DC
  Serial.println(F("OK! "));
  digitalWrite(MOTORADCA, LOW);
  digitalWrite(MOTORADCB, LOW);
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

//  Serial.println(F("LOOP"));
  if (calibracion)
  {
    //readSensor();

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
          readSensor();
        }
        if ( datoLeido )
        {
          Serial.print(F("\nLinea para copiar -> "));
          Serial.print(F("sensorData sdWhite = { "));
          Serial.print(sd.value[0]);
          Serial.print(F(", "));
          Serial.print(sd.value[1]);
          Serial.print(F(", "));
          Serial.print(sd.value[2]);
          Serial.println(F(" };"));
          paso++;
          contador = 0;
        }
        break;

      case 1:
        if ( contador == 0 )
        {
          Serial.println(F("\nPresentar muestra color NEGRO y pulsar FdC medio"));
          contador++;
        }

        while( !digitalRead(FIN_CARRERA_M) )
        {
          readSensor();
        }
        if ( datoLeido )
        {
          Serial.print(F("\nLinea para copiar -> "));
          Serial.print(F("sensorData sdBlack = { "));
          Serial.print(sd.value[0]);
          Serial.print(F(", "));
          Serial.print(sd.value[1]);
          Serial.print(F(", "));
          Serial.print(sd.value[2]);
          Serial.println(F(" };"));
          paso++;
          contador = 0;
        }
        break;

      case 2:
        if ( contador == 0 )
        {
          Serial.println(F("\nLiberar el sensor y pulsar FdC medio"));
          contador++;
        }

        while( !digitalRead(FIN_CARRERA_M) )
        {
          readSensor();
        }

        if ( datoLeido )
        {
          Serial.print(F("\nLinea para copiar -> "));
          Serial.print(F("{ \"NADA\", {"));
          Serial.print(rgb.value[TCS230_RGB_R]);
          Serial.print(F(", "));
          Serial.print(rgb.value[TCS230_RGB_G]);
          Serial.print(F(", "));
          Serial.print(rgb.value[TCS230_RGB_B]);
          Serial.println("} }");
          paso++;
          contador = 0;
        }
        break;

      case 3:
        if ( contador == 0 )
        {
          Serial.println(F("\nColocar el primer color y pulsar FdC medio"));
          contador++;
        }

        while( !digitalRead(FIN_CARRERA_M) )
        {
          readSensor();
        }

        if ( datoLeido )
        {
          Serial.print(F("\nLinea para copiar -> "));
          Serial.print(F("{ \"color 1\", {"));
          Serial.print(rgb.value[TCS230_RGB_R]);
          Serial.print(F(", "));
          Serial.print(rgb.value[TCS230_RGB_G]);
          Serial.print(F(", "));
          Serial.print(rgb.value[TCS230_RGB_B]);
          Serial.println("} }");
          paso++;
          contador = 0;
        }
        break;

      case 4:
        if ( contador == 0 )
        {
          Serial.println(F("\nColocar el segundo color y pulsar FdC medio"));
          contador++;
        }

        while( !digitalRead(FIN_CARRERA_M) )
        {
          readSensor();
        }

        if ( datoLeido )
        {
          Serial.print(F("\nLinea para copiar -> "));
          Serial.print(F("{ \"color 2\", {"));
          Serial.print(rgb.value[TCS230_RGB_R]);
          Serial.print(F(", "));
          Serial.print(rgb.value[TCS230_RGB_G]);
          Serial.print(F(", "));
          Serial.print(rgb.value[TCS230_RGB_B]);
          Serial.println("} }");
          paso++;
          contador = 0;
        }
        break;

      case 5:
        if ( contador == 0 )
        {
          Serial.println(F("\nColocar el tercer color y pulsar FdC medio"));
          contador++;
        }

        while( !digitalRead(FIN_CARRERA_M) )
        {
          readSensor();
        }

        if ( datoLeido )
        {
          Serial.print(F("\nLinea para copiar -> "));
          Serial.print(F("{ \"color 3\", {"));
          Serial.print(rgb.value[TCS230_RGB_R]);
          Serial.print(F(", "));
          Serial.print(rgb.value[TCS230_RGB_G]);
          Serial.print(F(", "));
          Serial.print(rgb.value[TCS230_RGB_B]);
          Serial.println("} }");
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
    switch (paso)
    {
      case 0:
        if ( datoLeido && contador <= 1) // Para evitar fallos hace dos lecturas y las compara.
        {
          colorLeido[contador] = colorMatch(&rgb);
          Serial.println(F("\n{ Paso 0 }"));
          Serial.print(F("Color leido = "));
          Serial.println(ct[colorDestino].name);
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
              Serial.println(F("Esperando pieza..."));
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
        Serial.println(F("\n{ Paso 3 }"));
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
