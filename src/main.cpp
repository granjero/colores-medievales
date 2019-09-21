/*
Maquina Medieval de seleccion por color
*/

#include <MD_TCS230.h>
#include <FreqCount.h>
#include <Stepper.h>

// PINES
// Sensor de color

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
int ByN =     0;

// Inicializa la libreria del sensor de COLOR en CS MT_TCS230.h
MD_TCS230	CS(S2, S3);

// Inicializa la libreria del motor paso a paso stepper.h
Stepper motorPaP(PASOS, MOTOR0, MOTOR1, MOTOR2, MOTOR3);

// define la variables de la librería MT_TCS230
colorData	rgb;
sensorData	sd;
// Valores del programa de calibración
sensorData sdBlack = { 10820, 11200, 14900 };
sensorData sdWhite = { 125680, 121120, 137210 };

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
{"NADA", {50, 25, 21} },            // 0 NEGRO
{"AZUL", {10, 46, 109} },           // 3 AZUL
{"ROJO", {104, 13, 17} },           // 2 ROJO
{"BLANCO", {255, 255, 255} },       // 1 BLANCO
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
  ______ _    _ _   _  _____ _____ ____  _   _ ______  _____
 |  ____| |  | | \ | |/ ____|_   _/ __ \| \ | |  ____|/ ____|
 | |__  | |  | |  \| | |      | || |  | |  \| | |__  | (___
 |  __| | |  | | . ` | |      | || |  | | . ` |  __|  \___ \
 | |    | |__| | |\  | |____ _| || |__| | |\  | |____ ____) |
 |_|     \____/|_| \_|\_____|_____\____/|_| \_|______|_____/

*/

// ===========================================================
// Compara color leido con la tabla fija
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
  Serial.print(F("Color RGB detectado: "));
  Serial.println(ct[minI].name);
  Serial.println(F(""));

	return(minI);
}

// ===========================================================
// Lee sensor
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
      Serial.print(F("readSensor() -> Valores Leidos "));
      Serial.print("RAW [");
      Serial.print(sd.value[0]);
      Serial.print(",");
      Serial.print(sd.value[1]);
      Serial.print(",");
      Serial.print(sd.value[2]);
      Serial.print("] ");

      CS.getRGB(&rgb);
      Serial.print("RGB [");
      Serial.print(rgb.value[TCS230_RGB_R]);
      Serial.print(",");
      Serial.print(rgb.value[TCS230_RGB_G]);
      Serial.print(",");
      Serial.print(rgb.value[TCS230_RGB_B]);
      Serial.println("]");
      datoLeido = true;
      waiting   = false;
    }
  }
}

// ===========================================================
// Desliga las bobinas del motor para preservar la circuitería berreta
void motorOFF ()
{
  digitalWrite(MOTOR0, LOW);
  digitalWrite(MOTOR1, LOW);
  digitalWrite(MOTOR2, LOW);
  digitalWrite(MOTOR3, LOW);
}

// ===========================================================
// Motor busca Fine de Carrera DERECHO
void motorFCD ()
{
  // BUSCA FIN DE CARRERA DERECHO
  if ( fdcD )
  {
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
    Serial.println(F("[FIN DE CARRERA DERECHO OK]"));
    Serial.println(F(""));
    motorOFF();
    delay(1000);
  }

  motorOFF();
  fdcD = true;
  fdcI = false;
  fdcM = false;
}

// ===========================================================
// Motor busca Fin de Carrera IZQUIERDO
void motorFCI ()
{
  if ( fdcI )
  {
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
    Serial.println(F("[FIN DE CARRERA IZQUIERDA OK]"));
    Serial.println(F(""));
    motorOFF();
    delay(1000);
  }

  fdcD = false;
  fdcI = true;
  fdcM = false;
}

// ===========================================================
// Motor busca Fin de Carrera MEDIO
void motorFCM ()
{
  int contador = 0;

  if (fdcM)
  {

  }
  else
  {
    if ( fdcI )
    {
      Serial.println(F("[Buscando FdC MEDIO] ->"));
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
  motorOFF();
  delay(1000);
}

// ===========================================================
// Motor busca Fines de Carrera Para el SETUP
void motorFC ()
{
  Serial.println(F("[CHEQUEA FINES DE CARRERA]"));
  int pasosFCFC = 0;
  Serial.println(F("[Buscando FdC derecho...]"));
  // BUSCA FIN DE CARRERA DERECHO
  while( !digitalRead(FIN_CARRERA_D) )
  {
    motorPaP.step(1);
    if (digitalRead(FIN_CARRERA_I))     // Encuentra fin de carrera izquierdo MAL
    {
      Serial.println(F("[BUSCANDO FIN DE CARRERA DERECHO ENCONTRÉ EL IZQUIERDO]"));
      Serial.println(F("[HALT & CATCH FIRE]"));
      motorOFF();
      while (true)
      {}
    }
  }
  Serial.println(F("[FdC DERECHO OK]"));
  Serial.println(F(""));
  while (digitalRead(FIN_CARRERA_D))
  {
    motorPaP.step(-1);
    //delay(10);
  }
  motorPaP.step(-30);
  motorOFF();
  delay(1000);

  // BUSCA FIN DE CARRERA IZQUIERDO
  Serial.println(F("[Buscando FdC izquierdo...]"));
  while( !digitalRead(FIN_CARRERA_I) )
  {
    pasosFCFC++;
    motorPaP.step(-1);
    if (digitalRead(FIN_CARRERA_D))    // Encuentra fin de carrera derecho MAL
    {
      Serial.println(F("[BUSCANDO FIN DE CARRERA IZQUIERDO ENCONTRÉ EL DERECHO]"));
      Serial.println(F("[HALT & CATCH FIRE]"));
      motorOFF();
      while (true)
      {}
    }
  }

  Serial.println(F("[FdC IZQUIERDA OK]"));
  Serial.println(F(""));
  while ( digitalRead(FIN_CARRERA_I) )
  {
    motorPaP.step(1);
    //delay(10
  }
  motorPaP.step(30);
  motorOFF();
  fdcD = false;
  fdcI = true;
  delay(1000);
}

// ===========================================================
// Motor Colores, hace algo diferente segun el color
void motorColor(int color)
{
  switch (color)
  {
    case 0:
      Serial.println(F("SIN LECTURA"));
      motorOFF();
      posicion = 0;
      break;

    case 1:
      Serial.println(F("Ir a Posición 1 - IZ"));
      motorFCI();
      delay(1000);
      posicion = 1;
      break;

    case 2:
      Serial.println(F("Ir a Posición 2 - M"));
      motorFCM();
      delay(1000);
      posicion = 2;
      break;

    case 3:
      Serial.println(F("Ir a Posición 3 - D"));
      motorFCD();
      delay(1000);
      posicion = 3;
      break;
  }
}

// ===========================================================
// Motor DC Sube y Baja
void motorDC()
{
  Serial.println(F("SUBE"));
  while ( ! digitalRead (FIN_CARRERA_AR) )
  {
    digitalWrite(MOTORADCA, LOW);
    digitalWrite(MOTORADCB, HIGH);
  }

  Serial.println(F("BAJA"));
  while ( ! digitalRead (FIN_CARRERA_AB) )
  {
    digitalWrite(MOTORADCA, HIGH);
    digitalWrite(MOTORADCB, LOW);
  }
  // Apaga el motor DC
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
  Serial.println(F("[ver. 0.0.0.0.0.1]"));
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
    motorFC();
    motorDC();
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
    readSensor();
    if (datoLeido)
    {
      switch(ByN)
      {
      case BLANCO:
        Serial.println(F("[Modo Calibración:]"));
        Serial.println(F("Leyendo Muestra color BLANCO"));
        CS.getRaw(&sd);
        CS.setWhiteCal(&sdWhite);
        Serial.println(F("[setWhiteCal]"));
        ByN++;
        break;

      case NEGRO:
        Serial.println(F("[Modo Calibración:]"));
        Serial.println(F("Coloque muesta color NEGRO"));
        while(!digitalRead(FIN_CARRERA_M))
        {

        }
        Serial.println(F("Leyendo Muestra color NEGRO"));
        CS.getRaw(&sd);
        CS.setDarkCal(&sdBlack);
        Serial.println(F("[setDarkCal]"));
        ByN++;
        break;

      default:
        calibracion = false;
        break;
      }
    }
  }

  else
  {
    switch (paso)
    {
      case 0:
        if ( datoLeido && contador <= 1)
        {
          Serial.println(F("dato leido y contador <= 1"));
          colorLeido[contador] = colorMatch(&rgb);
          contador++;
          //delay(1000);
        }
        if ( contador > 1 )
        {
          if ( colorLeido[0] == colorLeido[1] )
          {
            Serial.println(F("dos lecturas iguales"));
            colorDestino = colorLeido[0];
            colorLeido[0]  = 44;
            colorLeido[1]  = 14;
            paso++;
            contador = 0;
          }
          else
          {
            Serial.println(F("dos lecturas distintas"));
            //delay(1000);
            contador = 0;
          }
        }
        break;

      case 1:
        Serial.println(F("paso 1"));
        Serial.print(F("color detectado: "));
        Serial.println(ct[colorDestino].name);
        Serial.println(F("mover a color "));
        motorColor(colorDestino);
        paso++;
        //delay(1000);
        break;

      case 2:
        Serial.println(F("paso 2"));
        Serial.println(F("descarta pieza"));
        motorDC();
        paso = 0;
        contador = 0;
        break;

      case 3:
        paso++;
        break;

      default:
        paso = 0;
        break;
    }



    // motorDC();
    // delay(5000);
    //
    // readSensor();
    // if(datoLeido)
    // {
    //   uint8_t	i = colorMatch(&rgb);
    //   //Serial.print(F("\nColor: "));
    //   //Serial.println(ct[i].name);
    //   //Serial.println(F("..."));
    //   motorColor(i);
    // }

  }
}
