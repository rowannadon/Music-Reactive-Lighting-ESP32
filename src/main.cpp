#include "FastLED.h"

#include <Arduino.h>
#include <driver/adc.h>
#include <driver/i2s.h>
#include "freertos/queue.h"
#include <arduinoFFT.h>

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

#include "palettes.h"

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

    xTaskCreatePinnedToCore(i2sReaderTask, "I2S Reader Task", 4096, NULL, 1, &m_readerTaskHandle, 0);
  	xTaskCreatePinnedToCore(i2sWriterTask, "I2S Writer Task", 4096, NULL, 1, &m_writerTaskHandle, 1);
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