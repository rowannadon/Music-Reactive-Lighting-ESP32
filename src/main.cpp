#include "FastLED.h"

#include <Arduino.h>
#include <driver/adc.h>
#include <driver/i2s.h>
#include "freertos/queue.h"
#include <arduinoFFT.h>

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

#define LED_PIN     32
#define NUM_LEDS    551
#define BRIGHTNESS  255
#define LED_TYPE    WS2812B
#define COLOR_ORDER GRB
CRGB leds[NUM_LEDS];

void i2sReaderTask(void *param);
void convertBuf(int16_t *buf, int bytesRead);
void processI2SData(uint8_t *i2sData, size_t bytesRead);
void addSample(int16_t sample);
void i2sWriterTask(void *param);
void changePalette(int index);
void itol(int strip, int index, CRGB color);
void simulate(int startPos, int startStrip, int startDirection, int step, CRGB color, int depth);

#define SAMPLING_FREQUENCY 44000
#define MAX_SAMPLES 1024

//i2s number
#define EXAMPLE_I2S_NUM           (I2S_NUM_0)
//i2s sample rate
#define EXAMPLE_I2S_SAMPLE_BITS   (I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB)
//enable display buffer for debug
#define EXAMPLE_I2S_BUF_DEBUG     (0)
//I2S read buffer length
#define EXAMPLE_I2S_READ_LEN      (MAX_SAMPLES)
//I2S built-in ADC unit
#define I2S_ADC_UNIT              ADC_UNIT_1
//I2S built-in ADC channel
#define I2S_ADC_CHANNEL           ADC1_CHANNEL_0

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

DEFINE_GRADIENT_PALETTE( cool_gp ) {
    0,   0,255,255,
  255, 255,  0,255};

DEFINE_GRADIENT_PALETTE( Sunset_Real_gp ) {
    0, 120,  0,  0,
   22, 179, 22,  0,
   51, 255,104,  0,
   85, 167, 22, 18,
  135, 100,  0, 103,
  198,  16,  0, 130,
  255,   0,  0, 160};

DEFINE_GRADIENT_PALETTE( bhw1_28_gp ) {
    0,  75,  1,221,
   30, 252, 73,255,
   48, 169,  0,242,
  119,   0,149,242,
  170,  43,  0,242,
  206, 252, 73,255,
  232,  78, 12,214,
  255,   0,149,242};

DEFINE_GRADIENT_PALETTE( bhw1_01_gp ) {
    0, 227,101,  3,
  117, 194, 18, 19,
  255,  92,  8,192};

DEFINE_GRADIENT_PALETTE( bhw1_w00t_gp ) {
    0,   3, 13, 43,
  104,  78,141,240,
  188, 255,  0,  0,
  255,  28,  1,  1};

DEFINE_GRADIENT_PALETTE( bhw1_05_gp ) {
    0,   1,221, 53,
  255,  73,  3,178};

DEFINE_GRADIENT_PALETTE( bhw1_04_gp ) {
    0, 229,227,  1,
   15, 227,101,  3,
  142,  40,  1, 80,
  198,  17,  1, 79,
  255,   0,  0, 45};

DEFINE_GRADIENT_PALETTE( bhw1_07_gp ) {
    0, 232, 65,  1,
  255, 229,227,  1};

DEFINE_GRADIENT_PALETTE( bhw1_13_gp ) {
    0, 255,255, 45,
  255, 157, 57,197};

DEFINE_GRADIENT_PALETTE( bhw1_15_gp ) {
    0,   1,  8, 87,
   71,  23,195,130,
  122, 186,248,233,
  168,  23,195,130,
  255,   1,  8, 87};

DEFINE_GRADIENT_PALETTE( bhw1_17_gp ) {
    0,  95,  1, 62,
   58, 206, 27,151,
  105, 224, 88,188,
  124, 244,189,230,
  188, 206, 27,151,
  255,  95,  1, 62};

DEFINE_GRADIENT_PALETTE( bhw1_19_gp ) {
    0,  82,  1,  3,
   79,  20,  1,  1,
  181, 139,  1,  0,
  252,  20,  1,  1,
  255,  20,  1,  1};

DEFINE_GRADIENT_PALETTE( bhw1_20_gp ) {
    0,  67, 65,212,
   94, 121,193,219,
  150,  28,136,228,
  234,   6,  1, 22,
  255,   6,  1, 22};

DEFINE_GRADIENT_PALETTE( bhw1_21_gp ) {
    0,  42,  0, 45,
  132,  67, 65,212,
  185,  54, 13,111,
  244,  42,  0, 45,
  255,  42,  0, 45};

DEFINE_GRADIENT_PALETTE( bhw1_greenie_gp ) {
    0,   1, 33,  1,
  122, 121,255,125,
  255,   1, 33,  1};

DEFINE_GRADIENT_PALETTE( bhw1_justducky_gp ) {
    0,  47, 28,  2,
   76, 229, 73,  1,
  163, 255,255,  0,
  255, 229, 73,  1};

DEFINE_GRADIENT_PALETTE( bhw1_purplered_gp ) {
    0, 255,  0,  0,
  255, 107,  1,205};

DEFINE_GRADIENT_PALETTE( bhw1_sunconure_gp ) {
    0,  20,223, 13,
  160, 232, 65,  1,
  252, 232,  5,  1,
  255, 232,  5,  1};

DEFINE_GRADIENT_PALETTE( autumnrose_gp ) {
    0,  71,  3,  1,
   45, 128,  5,  2,
   84, 186, 11,  3,
  127, 215, 27,  8,
  153, 224, 69, 13,
  188, 229, 84,  6,
  226, 242,135, 17,
  255, 247,161, 79};

DEFINE_GRADIENT_PALETTE( fairygarden_gp ) {
    0,  55, 19,103,
   51,  95, 32,133,
  101, 167, 44,162,
  153, 125,182,237,
  204,  84,127,207,
  255,  19, 40,114};

DEFINE_GRADIENT_PALETTE( flame_gp ) {
    0, 252, 42,  1,
   43, 217,  6,  1,
   89, 213, 66,  1,
  127,   3, 74,  1,
  165, 213, 66,  1,
  211, 217,  6,  1,
  255, 252, 42,  1};

DEFINE_GRADIENT_PALETTE( girlcat_gp ) {
    0, 173,229, 51,
   73, 139,199, 45,
  140,  46,176, 37,
  204,   7, 88,  5,
  255,   0, 24,  0};

DEFINE_GRADIENT_PALETTE( healingangel_gp ) {
    0,  94,156,174,
   45,  66,105,166,
   84, 117,117,192,
  127, 173,124,156,
  170, 208,108,106,
  211, 197,119, 73,
  255, 210,221,123};

DEFINE_GRADIENT_PALETTE( otis_gp ) {
    0,  26,  1, 89,
  127,  17,193,  0,
  216,   0, 34, 98,
  255,   0, 34, 98};

DEFINE_GRADIENT_PALETTE( shyviolet_gp ) {
    0,  10,  9, 32,
   38,  41, 36, 79,
   76, 103,107,188,
  127, 142,154,194,
  178,  58, 92,176,
  216,  19, 51,158,
  255,   5, 24,103};

DEFINE_GRADIENT_PALETTE( spellbound_gp ) {
    0, 232,235, 40,
   12, 157,248, 46,
   25, 100,246, 51,
   45,  53,250, 33,
   63,  18,237, 53,
   81,  11,211,162,
   94,  18,147,214,
  101,  43,124,237,
  112,  49, 75,247,
  127,  49, 75,247,
  140,  92,107,247,
  150, 120,127,250,
  163, 130,138,252,
  173, 144,131,252,
  186, 148,112,252,
  196, 144, 37,176,
  211, 113, 18, 87,
  221, 163, 33, 53,
  234, 255,101, 78,
  247, 229,235, 46,
  255, 229,235, 46};

DEFINE_GRADIENT_PALETTE( titannightfall_gp ) {
    0, 173, 72,109,
   28, 112, 33, 68,
   51,  72, 19, 67,
  127,   2,  1, 33,
  204,  72, 19, 67,
  229, 112, 33, 68,
  255, 173, 72,109};

int16_t *m_audioBuffer1;
int16_t *m_audioBuffer2;
// current position in the audio buffer
int32_t m_audioBufferPos = 0;
// current audio buffer
int16_t *m_currentAudioBuffer;
// buffer containing samples that have been captured already
int16_t *m_capturedAudioBuffer;
// I2S reader task
TaskHandle_t m_readerTaskHandle;
// writer task
TaskHandle_t m_writerTaskHandle;
// i2s reader queue
QueueHandle_t m_i2sQueue;

int currentMode = 0;
int currentPalette = 0;
int red = 255;
int green = 0;
int blue = 0;
int brightness = 32;
int position = 0;


CRGBPalette16 palette = Sunset_Real_gp;
CRGBPalette16 targetPalette = spellbound_gp;
int paletteIndex = 0;
CRGB currentColor = CRGB(0, 0, 0);
int currentBrightness = 0;

bool useAutoPalette = true;

int OD = 0;
int pastOD = 0;
int timer = 0;
bool beatFlag = false;
int simulationCounter = 30;
bool simulationDirection = true;

double vReal[MAX_SAMPLES];
double vRealPrev[MAX_SAMPLES];
double vImag[MAX_SAMPLES];
arduinoFFT FFT = arduinoFFT(vReal, vImag, MAX_SAMPLES, SAMPLING_FREQUENCY);

//Low pass chebyshev filter order=1 alpha1=0.05 
class  FilterChLp1
{
	public:
		FilterChLp1()
		{
			v[0]=v[1]=0.0;
		}
	private:
		double v[2];
	public:
		double step(double x) //class II 
		{
			v[0] = v[1];
			v[1] = (1.370092461952019391e-1 * x)
				 + (0.72598150760959612171 * v[0]);
			return 
				 (v[0] + v[1]);
		}
};

//Low pass bessel filter order=1 alpha1=0.1 
class  FilterBeLp1
{
	public:
		FilterBeLp1()
		{
			v[0]=v[1]=0.0;
		}
	private:
		float v[2];
	public:
		float step(float x) //class II 
		{
			v[0] = v[1];
			v[1] = (2.452372752527856026e-1 * x)
				 + (0.50952544949442879485 * v[0]);
			return 
				 (v[0] + v[1]);
		}
};

FilterBeLp1 filterLP;

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
		std::string value = pCharacteristic->getValue();

		if (value.length() > 0) {
			std::string cm = "";
			std::string cp = "";
			std::string r = "";
			std::string g = "";
			std::string b = "";
			std::string br = "";
			std::string p = "";
			int pos = 0;
			for (int i = 0; i < value.length(); i++) {
				if (value[i] == ',') {
					pos++;
				} else {
					switch (pos) {
						case 1:
							cm += value[i];
							break;
						case 2:
							cp += value[i];
							break;
						case 3:
							r += value[i];
							break;
						case 4:
							g += value[i];
							break;
						case 5:
							b += value[i];
							break;
						case 6:
							br += value[i];
							break;
						case 7:
							p += value[i];
							break;
					}
				}
				
			}
			currentMode = atoi(cm.c_str());
			currentPalette = atoi(cp.c_str());
			if (currentPalette == 0) {
				useAutoPalette = true;
			} else {
				useAutoPalette = false;
				changePalette(currentPalette-1);
			}
			red = atoi(r.c_str());
			green = atoi(g.c_str());
			blue = atoi(b.c_str());
			brightness = atoi(br.c_str());
			//Serial.println(value.c_str());
		}
    }
};

void setup() {
	BLEDevice::init("BLE Lights");
	BLEServer *pServer = BLEDevice::createServer();
	BLEService *pService = pServer->createService(SERVICE_UUID);
	BLECharacteristic *pCharacteristic = pService->createCharacteristic(
											CHARACTERISTIC_UUID,
											BLECharacteristic::PROPERTY_READ |
											BLECharacteristic::PROPERTY_WRITE
										);

	pCharacteristic->setCallbacks(new MyCallbacks());
	pCharacteristic->setValue("Hello World");
	pService->start();

	BLEAdvertising *pAdvertising = pServer->getAdvertising();
	pAdvertising->start();

	FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection( TypicalLEDStrip );
    FastLED.setBrightness(  BRIGHTNESS );
    FastLED.clear();

  	Serial.begin(115200);

	i2s_config_t i2s_config;
	i2s_config.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN);
			i2s_config.sample_rate = SAMPLING_FREQUENCY;            
			i2s_config.dma_buf_len = MAX_SAMPLES;                   
			i2s_config.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
			i2s_config.channel_format = I2S_CHANNEL_FMT_ONLY_LEFT;  
			i2s_config.use_apll = false,
			i2s_config.communication_format = I2S_COMM_FORMAT_I2S_MSB;
			i2s_config.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
			i2s_config.dma_buf_count = 2;

	//install and start i2s driver
	adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_11db);
	adc1_config_width(ADC_WIDTH_12Bit);
	ESP_ERROR_CHECK( i2s_driver_install(EXAMPLE_I2S_NUM, &i2s_config,  4, &m_i2sQueue) );
	ESP_ERROR_CHECK( i2s_set_adc_mode(I2S_ADC_UNIT, I2S_ADC_CHANNEL) );		
	delay(100);
	i2s_adc_enable(EXAMPLE_I2S_NUM);

	m_audioBuffer1 = (int16_t *)malloc(sizeof(int) * MAX_SAMPLES);
    m_audioBuffer2 = (int16_t *)malloc(sizeof(int) * MAX_SAMPLES);

    m_currentAudioBuffer = m_audioBuffer1;
    m_capturedAudioBuffer = m_audioBuffer2;

	static uint8_t ucParameterToPass;
    xTaskCreatePinnedToCore(i2sReaderTask, "I2S Reader Task", 4096, &ucParameterToPass, 1, &m_readerTaskHandle, 0);
  	xTaskCreatePinnedToCore(i2sWriterTask, "I2S Writer Task", 4096, &ucParameterToPass, 1, &m_writerTaskHandle, 1);
}

void loop() {
	EVERY_N_SECONDS(5) {
		if (useAutoPalette == true) {
			changePalette(random8(27));
		}
	}
	EVERY_N_MILLISECONDS(5) {
		FastLED.setBrightness(constrain(currentBrightness, 0, 255));
		if (brightness < currentBrightness) {
			currentBrightness -= 2;
		} else if (brightness > currentBrightness) {
			currentBrightness++;
		}
		CRGB newColor = CRGB(red, green, blue);

        switch (currentMode) {
			case 0:
				nblend(currentColor, newColor, 5);
				fill_solid(leds, NUM_LEDS, currentColor);
				break;
			case 1:
				nblendPaletteTowardPalette(targetPalette, palette, 30);
				fill_palette(leds, NUM_LEDS, paletteIndex, 1, targetPalette, 255, LINEARBLEND);
				break;
			case 2:
				simulate(36, 11, 1, simulationCounter, ColorFromPalette(targetPalette, paletteIndex, 255, LINEARBLEND), 0);
				if (beatFlag) {
					beatFlag = false;
					simulationDirection = !simulationDirection;
					changePalette(random8(27));
					Serial.println("big beat");
				}
				fadeToBlackBy(leds, NUM_LEDS, 30);
				break;
			case 3:
				for (int i = 0; i < 13; i++) {
					itol(i, paletteIndex % 100, CRGB(255, 0, 0));
				}
				fadeToBlackBy(leds, NUM_LEDS, 30);
				break;
			case 4:
				break;
		}        
        FastLED.show();
    }

	EVERY_N_MILLISECONDS(20) {
		paletteIndex++;
		if (simulationDirection) {
			simulationCounter++;
		} else {
			simulationCounter--;
		}
		if ((simulationCounter > 250) || (simulationCounter < -250)) {
			simulationCounter = 30;
		}
	}
}

void simulate(int startPos, int startStrip, int startDirection, int step, CRGB color, int depth) {
	if (depth > 5) {
		if (random8(10) ==5) {
			return;
		}
	}
	if (depth > 15) {
		return;
	}
	int d = depth+1;
	int pos;
    switch (startDirection) {
        case 0:
            pos = startPos-step;
            break;
        case 1:
            pos = startPos+step;
    }
	switch(startStrip) {
		case 0:
			if (pos > 15) {
				simulate(0, 1, 1, step-16, color, d);
				simulate(0, 10, 1, step-16, color, d);
			} else if (pos < 0) {
				simulate(72, 9, 0, -pos-1, color, d);
			} else {
				itol(0, pos, color);
			}
			break;
		case 1:
			if (pos > 30) {
				simulate(0, 2, 1, pos-31, color, d);
				simulate(72, 11, 0, pos-31, color, d);
			} else if (pos < 0) {
				simulate(15, 0, 0, pos-31, color, d);
				simulate(0, 10, 1, pos-31, color, d);
			} else {
				itol(1, pos, color);
			}
			break;
		case 2:
			if (pos > 30) {
				simulate(0, 3, 1, pos-31, color, d);
				simulate(0, 12, 1, pos-31, color, d);
			} else if (pos < 0) {
				simulate(30, 1, 0, -pos-1, color, d);
				simulate(72, 11, 0, -pos-1, color, d);
			} else {
				itol(2, pos, color);
			}
			break;
		case 3:
			if (pos > 14) {
				simulate(0, 4, 1, pos-15, color, d);
			} else if (pos < 0) {
				simulate(30, 2, 0, -pos-1, color, d);
				simulate(0, 12, 1, -pos-1, color, d);
			} else {
				itol(3, pos, color);
			}
			break;
		case 4:
			if (pos > 72) {
				simulate(0, 5, 1, pos-73, color, d);
			} else if (pos < 0) {
				simulate(14, 3, 0, -pos-1, color, d);
			} else {
				itol(4, pos, color);
			}
			break;
		case 5:
			if (pos > 14) {
				simulate(0, 6, 1, pos-15, color, d);
				simulate(72, 12, 0, pos-15, color, d);
			} else if (pos < 0) {
				simulate(72, 4, 0, -pos-1, color, d);
			} else {
				itol(5, pos, color);
			}
			break;
		case 6:
			if (pos > 30) {
				simulate(0, 7, 1, pos-31, color, d);
				simulate(0, 11, 1, pos-31, color, d);
			} else if (pos < 0) {
				simulate(72, 12, 1, -pos-1, color, d);
				simulate(14, 5, 1, -pos-1, color, d);
			} else {
				itol(6, pos, color);
			}
			break;
		case 7:
			if (pos > 30) {
				simulate(0, 8, 1, pos-31, color, d);
				simulate(72, 10, 0, pos-31, color, d);
			} else if (pos < 0) {
				simulate(30, 6, 1, -pos-1, color, d);
				simulate(0, 11, 1, -pos-1, color, d);
			} else {
				itol(7, pos, color);
			}
			break;
		case 8:
			if (pos > 15) {
				simulate(0, 9, 1, pos-16, color, d);
			} else if (pos < 0) {
				simulate(72, 10, 0, -pos-1, color, d);
				simulate(30, 7, 0, -pos-1, color, d);
			} else {
				itol(8, pos, color);
			}
			break;
		case 9:
			if (pos > 72) {
				simulate(0, 0, 1, pos-73, color, d);
			} else if (pos < 0) {
				simulate(15, 8, 0, -pos-1, color, d);
			} else {
				itol(9, pos, color);
			}
			break;
		case 10:
			if (pos > 72) {
				simulate(30, 7, 0, pos-73, color, d);
				simulate(0, 8, 1, pos-73, color, d);
			} else if (pos < 0) {
				simulate(15, 0, 0, -pos-1, color, d);
				simulate(0, 1, 1, -pos-1, color, d);
			} else {
				itol(10, pos, color);
			}
			break;
		case 11:
			if (pos > 72) {
				simulate(30, 1, 0, pos-73, color, d);
				simulate(0, 2, 1, pos-73, color, d);
			} else if (pos < 0) {
				simulate(30, 6, 0, -pos-1, color, d);
				simulate(0, 7, 1, -pos-1, color, d);
			} else {
				itol(11, pos, color);
			}
			break;
		case 12:
			if (pos > 72) {
				simulate(14, 5, 0, pos-73, color, d);
				simulate(0, 6, 1, pos-73, color, d);
			} else if (pos < 0) {
				simulate(30, 2, 0, -pos-1, color, d);
				simulate(0, 3, 1, -pos-1, color, d);
			} else {
				itol(12, pos, color);
			}
			break;
	}
}

void itol(int strip, int index, CRGB color) {
	switch (strip) {
		case 0:
			if (index >= 0 && index <= 15)
				leds[index] = color;
			break;
		case 1:
			if (index >= 0 && index <= 30)
				leds[index+16] = color;
			break;
		case 2:
			if (index >= 0 && index <= 30)
				leds[index+47] = color;
			break;
		case 3:
			if (index >= 0 && index <= 14)
				leds[index+78] = color;
			break;
		case 4:
			if (index >= 0 && index <= 72)
				leds[index+93] = color;
			break;
		case 5:
			if (index >= 0 && index <= 14)
				leds[index+166] = color;
			break;
		case 6:
			if (index >= 0 && index <= 30)
				leds[index+181] = color;
			break;
		case 7:
			if (index >= 0 && index <= 30)
				leds[index+212] = color;
			break;
		case 8:
			if (index >= 0 && index <= 15)
				leds[index+243] = color;
			break;
		case 9:
			if (index >= 0 && index <= 72)
				leds[index+259] = color;
			break;
		case 10:
			if (index >= 0 && index <= 72)
				leds[index+332] = color;
			break;
		case 11:
			if (index >= 0 && index <= 72)
				leds[index+405] = color;
			break;
		case 12:
			if (index >= 0 && index <= 72)
				leds[index+478] = color;
			break;
	}
}

void changePalette(int index) {
    switch(index) {
        case 0:
            palette = RainbowColors_p;
            break;
        case 1:
            palette = cool_gp;
            break;
        case 2:
            palette = Sunset_Real_gp;
            break;
        case 3:
            palette = bhw1_28_gp;
            break;
        case 4:
            palette = bhw1_01_gp;
            break;
        case 5:
            palette = bhw1_w00t_gp;
            break;
        case 6:
            palette = bhw1_05_gp;
            break;
        case 7:
            palette = bhw1_04_gp;
            break;
        case 8:
            palette = bhw1_07_gp;
            break; 
        case 9:
            palette = bhw1_13_gp;
            break; 
        case 10:
            palette = bhw1_15_gp;
            break; 
        case 11:
            palette = bhw1_17_gp;
            break; 
        case 12:
            palette = bhw1_19_gp;
            break; 
        case 13:
            palette = bhw1_20_gp;
            break; 
        case 14:
            palette = bhw1_21_gp;
            break; 
        case 15:
            palette = bhw1_greenie_gp;
            break; 
        case 16:
            palette = bhw1_justducky_gp;
            break;
        case 17:
            palette = bhw1_purplered_gp;
            break; 
        case 18:
            palette = bhw1_sunconure_gp;
            break; 
        case 19:
            palette = autumnrose_gp;
            break; 
        case 20:
            palette = fairygarden_gp;
            break; 
        case 21:
            palette = flame_gp;
            break; 
        case 22:
            palette = girlcat_gp;
            break; 
        case 23:
            palette = healingangel_gp;
            break; 
        case 24:
            palette = otis_gp;
            break; 
        case 25:
            palette = shyviolet_gp;
            break; 
        case 26:
            palette = spellbound_gp;
            break; 
        case 27:
            palette = titannightfall_gp;
            break; 
    }
}

void i2sReaderTask(void *param) {
	while (true) {
		// wait for some data to arrive on the queue
		i2s_event_t evt;
		if (xQueueReceive(m_i2sQueue, &evt, portMAX_DELAY) == pdPASS) {
			if (evt.type == I2S_EVENT_RX_DONE) {
				size_t bytesRead = 0;
				do {
					// read data from the I2S peripheral
					uint8_t i2sData[MAX_SAMPLES];
					// read from i2s
					i2s_read(EXAMPLE_I2S_NUM, i2sData, MAX_SAMPLES, &bytesRead, 10);
					// process the raw data
					processI2SData(i2sData, bytesRead);
				} while (bytesRead > 0);
			}
		}
	}
}

void processI2SData(uint8_t *i2sData, size_t bytesRead) {
    int32_t *samples = (int32_t *)i2sData;
    for (int i = 0; i < bytesRead / 4; i++)
    {
        // you may need to vary the >> 11 to fit your volume - ideally we'd have some kind of AGC here
        addSample(samples[i] >> 16);
    }
}

void addSample(int16_t sample)	{
    // add the sample to the current audio buffer
    m_currentAudioBuffer[m_audioBufferPos] = sample;
    m_audioBufferPos++;
    // have we filled the buffer with data?
    if (m_audioBufferPos == MAX_SAMPLES)
    {
        // swap to the other buffer
        std::swap(m_currentAudioBuffer, m_capturedAudioBuffer);
        // reset the buffer position
        m_audioBufferPos = 0;
        // tell the writer task to save the data
        xTaskNotify(m_writerTaskHandle, 1, eIncrement);
    }
}

void i2sWriterTask(void *param) {
  const TickType_t xMaxBlockTime = pdMS_TO_TICKS(100);
  while (true)
  {
    // wait for some samples to save
    uint32_t ulNotificationValue = ulTaskNotifyTake(pdTRUE, xMaxBlockTime);
    if (ulNotificationValue > 0)
    {
      	convertBuf(m_capturedAudioBuffer, MAX_SAMPLES);
		
		FFT.DCRemoval();
		FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
		FFT.Compute(FFT_FORWARD);
		FFT.ComplexToMagnitude();
		int total = 0;
		for (int i = 2; i < MAX_SAMPLES/2; i++) {
			total += max(vReal[i]-vRealPrev[i], 0.0);
		}
		pastOD = OD;
		OD = total;
		if (OD > 20000) {
			beatFlag = true;
		}
    }
  }
}

void convertBuf(int16_t *buf, int size)
{
    for (int i = 0; i < size; i++) {
		vRealPrev[i] = vReal[i];
		vReal[i] = filterLP.step(buf[i]);
		vImag[i] = 0;
		//Serial.println(vReal[i]);
    }
}