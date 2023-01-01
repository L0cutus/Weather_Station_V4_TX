#define BATTERYV A2
#define WINDDIRV A3

// ********************************************
// Istantaneous Wind calculated every 3 seconds
// Medium wind calculated every 10 minutes
// Gust is instantaneous wind > 18km/h
// ********************************************
// MAX array buffer for instantaneous wind data ( ((seconds * min) / 3 seconds) -> ((60 * 10) / 3))
#define MAX_BUFFER 200 // 10 minutes, every value is the medium of 3 seconds

unsigned char windData[MAX_BUFFER];  // 200 bytes


// Serial.print buffer
char srlBuf[80];

// payload #00
struct pld_wind {
  float medianWind;     // 4 bytes media degli ultimi 10 minuti
  float gustWind;       // 4 bytes raffica più veloce degli ultimi 10 minuti
  float windDir;        // 4 bytes wind direction
  float instWind;       // 4 bytes instantaneus wind
};                       // 16 bytes

// payload #01
struct pld_data {
  float batteryV;         // 4 bytes
  float temperature;      // 4 bytes
  float humidity;         // 4 bytes
  float pressure;         // 4 bytes
  float rain;             // 4 bytes
};                        // Total: 20 bytes

const uint16_t this_node = 01;   // Address of our node in Octal format
const uint16_t other_node = 00;  // Address of the other node in Octal format
