/*
Maquina Medieval de seleccion por color
*/

#include <MD_TCS230.h>
#include <FreqCount.h>
#include <Stepper.h>

// PINES
// Sensor de color
//#define  S0                                           // PIN S0 del Sensor de color en HIGH por hardware
//#define  S1                                           // PIN S1 del Sensor de color en HIGH por hardware
#define  S2              12                             // Salida del Sensor de color S2 al pin 12 del micro
#define  S3              13                             // Salida del Sensor de color S3 al pin 13 del micro
// Motor Paso a Paso
#define  MOTOR0           8
#define  MOTOR1           9
#define  MOTOR2           10
#define  MOTOR3           11
// Fines de Carrera
#define  FIN_CARRERA_D    2                             // Fin de Carrera Derecho
#define  FIN_CARRERA_I    3                             // Fin de Carrera Izquierdo
#define  FIN_CARRERA_M    4                             // Fin de Carrera Rojo

#define VELOCIDAD 300
#define PASOS 200

#define BLANCO  0
#define NEGRO   1

int ByN =     0;
//const int pasosStepper = 200;

MD_TCS230	CS(S2, S3);

Stepper motorPaP(PASOS, MOTOR0, MOTOR1, MOTOR2, MOTOR3);

colorData	rgb;

// Variables de estado del firm
bool calibracion  = false;
bool datoLeido    = false;
bool fdcI         = false;
bool fdcD         = false;

sensorData	sd;
sensorData sdBlack = { 10250, 10490, 13910 };
sensorData sdWhite = { 128850, 124230, 141520 };

// Color Table for matching
typedef struct
{
  char    name[9];  // color name 8+nul
  colorData rgb;    // RGB value
} colorTable;
colorTable ct[] =
{
{"NADA", {50, 25, 21} },            // 0 NEGRO
{"BLANCO", {248, 246, 245} },       // 1 BLANCO
{"ROJO", {106, 16, 19} },           // 2 ROJO
{"AZUL", {11, 45, 105} },           // 3 AZUL
};


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
  Serial.println(minI);
	return(minI);
}

// ===========================================================
// Lee sensor
void readSensor()
{
  // Serial.println(F("Funcion readSensor()"));
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
      Serial.print("RAW [");
      Serial.print(sd.value[0]);
      Serial.print(",");
      Serial.print(sd.value[1]);
      Serial.print(",");
      Serial.print(sd.value[2]);
      Serial.println("]");

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
// Motor busca Fines de Carrera
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
    fdcD = true;
    fdcI = false;
    while( digitalRead(FIN_CARRERA_D) )
    {
      motorPaP.step(-1);
    }
    motorPaP.step(-400);
    Serial.println(F("[FIN DE CARRERA DERECHO OK motorFCD()]"));
    Serial.println(F(""));
    motorOFF();
    delay(1000);
  }

  motorOFF();
  fdcD = true;
  fdcI = false;
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
    fdcD = false;
    fdcI = true;
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
}

// ===========================================================
// Motor busca Fines de Carrera Para el SETUP
void motorFC ()
{
  Serial.println(F("[CHEQUEA FINES DE CARRERA]"));
  int pasosFCFC = 0;
  Serial.println(F("[Buscando FdC derecho...]"));
  Serial.println(F(""));

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
  Serial.println(F("[FIN DE CARRERA DERECHO OK]"));
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
  Serial.println(F(""));

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

  Serial.println(F("FIN DE CARRERA IZQUIERDA OK"));
  Serial.println(F(""));
  while ( digitalRead(FIN_CARRERA_I) )
  {
    motorPaP.step(1);
    //delay(10
  }
  motorPaP.step(30);
  motorOFF();
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
      break;

    case 1:
      Serial.println(F("Primer Color"));
      motorFCD();
      delay(1000);
      break;

    case 2:
      Serial.println(F("Segundo Color"));
      motorFCI();
      delay(1000);
      break;

    case 3:
      Serial.println(F("Tercer Color"));
      delay(1000);
      break;
  }
}



// ===========================================================
// Setup
void setup()
{
  pinMode(FIN_CARRERA_D, INPUT);
  pinMode(FIN_CARRERA_I, INPUT);
  pinMode(FIN_CARRERA_M, INPUT);

  Serial.begin(115200);                               // inicia comunicacion serie a 115200baudios
  Serial.println(F("[Maquina Medieval de colores]"));
  Serial.println(F("[ver. 0.0.0.0.0.1]"));
  Serial.println(F(""));

  motorPaP.setSpeed(VELOCIDAD);
  CS.begin();

  if (digitalRead(FIN_CARRERA_I) && digitalRead(FIN_CARRERA_D))                 // Tocaron el boton de calibración. Por lo que está en uno
  {
    calibracion = true;
  }
  else                                              // Usa los datos de cali harcodeados.
  {
    CS.setDarkCal(&sdBlack);
    CS.setWhiteCal(&sdWhite);
    motorFC();
  }
}


// ===========================================================
// Loop
void loop()
{
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
    readSensor();
    if(datoLeido)
    {
      uint8_t	i = colorMatch(&rgb);
      Serial.print(F("\nColor: "));
      Serial.println(ct[i].name);
      motorColor(i);
    }
  }

}
