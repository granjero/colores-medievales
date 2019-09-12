/*
Maquina Medieval de seleccion por color
*/

#include <MD_TCS230.h>
#include <FreqCount.h>
#include <Stepper.h>

// PINES
// Sensor de color
//#define  S0                                           // PIN S0 del Sensosr de color en HIGH por hardware
//#define  S1                                           // PIN S1 del Sensosr de color en HIGH por hardware
#define  S2              12                             // Salida del Sensor de color S2 al pin 12 del micro
#define  S3              13                             // Salida del Sensor de color S3 al pin 13 del micro
// Motor Paso a Paso
#define  MOTOR0           8
#define  MOTOR1           9
#define  MOTOR2           10
#define  MOTOR3           11
// Pin de Calibracion
#define  PIN_CALIBRACION  2                             // BOTÓN PULL DOWN al pin 2
// Fines de Carrera
#define  FIN_CARRERA_D    7                             // Fin de Carrera Derecho
#define  FIN_CARRERA_I    2                             // Fin de Carrera IzquierdO
#define  FIN_CARRERA_R    3                             // Fin de Carrera Rojo
#define  FIN_CARRERA_G    6                             // Fin de Carrera Verde
//#define  FIN_CARRERA_B                                // Fin de Carrera Azul

#define BLANCO  0
#define NEGRO   1

int ByN =     0;
const int pasosStepper = 200;

MD_TCS230	CS(S2, S3);

Stepper motorPaP(pasosStepper, 8, 9, 10, 11);

colorData	rgb;

// Variables de estado del firm
bool calibracion  = false;
bool datoLeido    = false;

sensorData	sd;
sensorData sdBlack = { 10010, 10330, 14120 };
sensorData sdWhite = { 119730, 113350, 128990 };

// Color Table for matching
typedef struct
{
  char    name[9];  // color name 8+nul
  colorData rgb;    // RGB value
} colorTable;
colorTable ct[] =
{
{"NEGRO", {0, 0, 0} },            // 0 NEGRO
{"BLANCO", {247, 249, 251} },     // 1 BLANCO
{"ROJO", {104, 11, 15} },         // 2 ROJO
{"VERDE", {17, 68, 35} },         // 3 VERDE
{"AZUL", {7, 44, 112} },          // 4 AZUL
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
void motorFC ()
{
  Serial.println(F("CHEQUEO FINES DE CARRERA"));
  int pasosFCFC = 0;
  // BUSCO IZQUIERDA
  for (int i = 0; i < 15000 ; i++)
  {
    motorPaP.step(1);
    if (digitalRead(FIN_CARRERA_D))
    {
      Serial.println(F("BUSCANDO FIN DE CARRERA IZ ENCONTRÉ EL DE"));
      Serial.println(F("HALT & CATCH FIRE"));
      while (true)
      {}
    }
    if (digitalRead(FIN_CARRERA_I))
    {
      Serial.println(F("FIN DE CARRERA IZ OK"));
      delay(1000);                                                // TODO Debounce
      break;
    }
  }
  // BUSCO DERECHA
  for (int i = 0; i < 15000 ; i++)
  {
    pasosFCFC++;
    motorPaP.step(-1);
    if (digitalRead(FIN_CARRERA_I))
    {
      Serial.println(F("BUSCANDO FIN DE CARRERA DE ENCONTRÉ EL IZ"));
      Serial.println(F("HALT & CATCH FIRE"));
      while (true)
      {}
    }

    if (digitalRead(FIN_CARRERA_D))
    {
      Serial.println(F("FIN DE CARRERA DE OK"));
      delay(1000);                                                // TODO Debounce
      break;
    }
  }
  motorPaP.step(pasosFCFC / 2);
  delay(1000);
}

// ===========================================================
// Motor Colores, hace algo diferente segun el color
void motorColor(int color)
{
  int avanza = 1;
  switch (color)
  {
    case 0:
      break;

    case 1:
      Serial.println(F("BLANCO"));
      motorPaP.step(20);
      motorPaP.step(-20);
      delay(1000);
      break;

    case 2:
      Serial.println(F("ROJO"));
      while( !digitalRead(FIN_CARRERA_R))
      {
        motorPaP.step(avanza);
        if( digitalRead(FIN_CARRERA_D) || digitalRead(FIN_CARRERA_I) )
        {
          avanza = avanza * -1;
        }
      }
      delay(1000);
      break;

    case 3:
      Serial.println(F("VERDE"));
      while( !digitalRead(FIN_CARRERA_G) )
      {
        motorPaP.step(avanza);
        if( digitalRead(FIN_CARRERA_D) || digitalRead(FIN_CARRERA_I) )
        {
          avanza = avanza * -1;
        }
      }
      delay(1000);
      break;

    case 4:
      Serial.println(F("AZUL"));
      motorPaP.step(20);
      motorPaP.step(-20);
      break;
  }
}


// ===========================================================
// Setup
void setup()
{
  Serial.begin(115200);                               // inicia comunicacion serie a 115200baudios
  Serial.println(F("[Maquina Medieval de colores]"));

  motorPaP.setSpeed(60);
  motorFC();
  CS.begin();

  pinMode(PIN_CALIBRACION, INPUT);
  pinMode(FIN_CARRERA_D, INPUT);
  pinMode(FIN_CARRERA_I, INPUT);
  pinMode(FIN_CARRERA_R, INPUT);
  pinMode(FIN_CARRERA_G, INPUT);
  //pinMode(FIN_CARRERA_B, INPUT);

  if (digitalRead(PIN_CALIBRACION))                 // Tocaron el boton de calibración. Por lo que está en uno
  {
    calibracion = true;
  }
  else                                              // Usa los datos de cali harcodeados.
  {
    CS.setDarkCal(&sdBlack);
    CS.setWhiteCal(&sdWhite);
  }
}

// ===========================================================
// Loop
void loop()
{
  if (calibracion)
  {
    //Serial.println(F("[LOOP Calibracion]"));
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
        while(!digitalRead(PIN_CALIBRACION))
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
