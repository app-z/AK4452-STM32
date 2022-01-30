#include <stdio.h>
#include "audioplay.h"
#include "fatfs.h"
#include "greq_glo.h"

//------------------------------------------------------
#define I2S_STANDARD                  I2S_STANDARD_PHILIPS

/* Audio status definition */     
#define AUDIO_OK                        0
#define AUDIO_ERROR                     1
#define AUDIO_TIMEOUT                   2

/* Position in the audio play buffer */
__IO BUFFER_StateTypeDef buffer_offset = BUFFER_OFFSET_NONE;

/* Codec output DEVICE */
#define OUTPUT_DEVICE_SPEAKER         1
#define OUTPUT_DEVICE_HEADPHONE       2
#define OUTPUT_DEVICE_BOTH            3
#define OUTPUT_DEVICE_AUTO            4

/* MUTE commands */
#define AUDIO_MUTE_ON                 1
#define AUDIO_MUTE_OFF                0

/* Defines for the Audio playing process */
#define PAUSE_STATUS     ((uint32_t)0x00) /* Audio Player in Pause Status */
#define RESUME_STATUS    ((uint32_t)0x01) /* Audio Player in Resume Status */
#define IDLE_STATUS      ((uint32_t)0x02) /* Audio Player in Idle Status */


#define AUDIO_RESET_GPIO_CLK_ENABLE()         __GPIOD_CLK_ENABLE()
#define AUDIO_RESET_PIN                       GPIO_PIN_4
#define AUDIO_RESET_GPIO                      GPIOD
#define VOLUME_CONVERT(Volume)    (((Volume) > 100)? 100:((uint8_t)(((Volume) * 255) / 100)))

#define CODEC_STANDARD                0x04

/* Variables used in normal mode to manage audio file during DMA transfer */
uint32_t AudioTotalSize           = 0xFFFF; /* This variable holds the total size of the audio file */
int32_t AudioRemSize             = 0xFFFF; /* This variable holds the remaining data in audio file */
uint16_t *CurrentPos ;             /* This variable holds the current position of audio pointer */
__IO uint32_t PauseResumeStatus = IDLE_STATUS;   


#define DMA_MAX_SZE                     0xFFFF

#define DMA_MAX(_X_)                (((_X_) <= DMA_MAX_SZE)? (_X_):DMA_MAX_SZE)
#define AUDIODATA_SIZE                  2   /* 16-bits audio data size */
extern FIL WavFile;

const uint32_t I2SFreq[8] = {8000, 11025, 16000, 22050, 32000, 44100, 48000, 96000};
const uint32_t I2SPLLN[8] = {256, 429, 213, 429, 426, 271, 258, 344};
const uint32_t I2SPLLR[8] = {5, 4, 4, 4, 4, 6, 3, 1};
volatile uint8_t OutputDev = 0;
static uint32_t WaveDataLength = 0;

extern I2C_HandleTypeDef hi2c1;
extern I2S_HandleTypeDef hi2s2;

uint8_t Audio_Buffer[AUDIO_BUFFER_SIZE];

extern WAVE_FormatTypeDef *waveformat;
AUDIO_StateMachine     Audio;
uint32_t samplerate;

greq_dynamic_param_t *pEqualizerParams;

ApplicationTypeDef Appli_state = APPLICATION_IDLE;
char str2[20];
char str3[20];
uint32_t offsetpos;
uint32_t cnt;
uint32_t dur; //���������� ��� ����������� ������� ��������������� �����
//------------------------------------------------------
void Error(void)
{
//  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
}
//------------------------------------------------------
uint8_t CODEC_IO_Write(uint8_t Addr, uint8_t Reg, uint8_t Value)
{
	HAL_StatusTypeDef status = HAL_OK;
	uint8_t result=0;
	status = HAL_I2C_Mem_Write(&hi2c1, Addr,(uint16_t)Reg,
						I2C_MEMADD_SIZE_8BIT,&Value,1,0x1000);
	if(status!=HAL_OK)
	{
		Error();
		return 1;
	}
	return result;
}

//---------------------------------------------------------------------

void write_register(uint8_t dev_addr, uint8_t register_pointer, uint8_t register_value)
{
    uint8_t data[2];

    data[0] = register_pointer;
    data[1] = register_value;    // MSB byte of 16bit data

    HAL_I2C_Master_Transmit(&hi2c1, dev_addr, data, 2, 100);  // data is the start pointer of our array
}

void read_register(uint8_t dev_addr, uint8_t register_pointer, uint8_t* receive_buffer)
{
    // first set the register pointer to the register wanted to be read
    HAL_I2C_Master_Transmit(&hi2c1, dev_addr, &register_pointer, 1, 100);  // note the & operator which gives us the address of the register_pointer variable

    // receive the 2 x 8bit data into the receive buffer
    HAL_I2C_Master_Receive(&hi2c1, dev_addr, receive_buffer, 1, 100);
}

void set_volume(uint8_t volume){
	  write_register(0x20, 0x3, volume);
	  HAL_Delay(3);
	  write_register(0x20, 0x4, volume);
}


//------------------------------------------------------
//void AudioPlay_Init(uint32_t AudioFreq)
//{
//	samplerate = AudioFreq;
//	__IO uint8_t volume=70;
//	if(AudioOut_Init(OUTPUT_DEVICE_AUTO, volume, samplerate)!=0)
//	{
//		Error();
//	}
//}
//------------------------------------------------------
void AudioPlay_Stop(void)
{
	HAL_Delay(1);
	HAL_I2S_DMAStop(&hi2s2);
	HAL_Delay(1);
}

extern void setLevel(int16_t level);

extern void process(int16_t *iobuffer, int32_t nSamples);


void shift_level(int offset, int len, int level);

//------------------------------------------------------
void AudioPlay_Start(uint32_t AudioFreq)
{
	offsetpos=44;
	cnt=0;
	UINT bytesread=0;

	uint8_t level = 0xd0;
	uint32_t tickstart = HAL_GetTick();
	uint32_t delay_volume_control = 50;

	AudioTotalSize=waveformat->FileSize;
	/*Get Data from USB Flash Disk*/
	WaveDataLength=waveformat->FileSize;
	f_lseek(&WavFile, offsetpos);
	f_read(&WavFile,&Audio_Buffer[0],AUDIO_BUFFER_SIZE,&bytesread);
	AudioRemSize=WaveDataLength-bytesread;
	CurrentPos=0;
	{
		HAL_I2S_Transmit_DMA(&hi2s2,(uint16_t*)&Audio_Buffer[0],
									DMA_MAX(AUDIO_BUFFER_SIZE/AUDIODATA_SIZE));
	}

	  set_volume(level);
//	setLevel(-12);

//	char level = 0;
	/*Check if the device is connected*/
	while(AudioRemSize!=0)
	{
		if(buffer_offset==BUFFER_OFFSET_HALF)
		{
			f_read(&WavFile,&Audio_Buffer[0], AUDIO_BUFFER_SIZE/2,(void*)&bytesread);

//			process((int16_t*)&Audio_Buffer[0], AUDIO_BUFFER_SIZE/4/2);

//			if (level > 0) shift_level(0, AUDIO_BUFFER_SIZE/2, level);

			buffer_offset=BUFFER_OFFSET_NONE;
			AudioRemSize-=bytesread;
		}
		if(buffer_offset==BUFFER_OFFSET_FULL)
		{
			f_read(&WavFile,&Audio_Buffer[AUDIO_BUFFER_SIZE/2],
							AUDIO_BUFFER_SIZE/2,(void*)&bytesread);

//			process((int16_t*)&Audio_Buffer[AUDIO_BUFFER_SIZE/2], AUDIO_BUFFER_SIZE/4/2);

//			int offs = AUDIO_BUFFER_SIZE/2;
//			if (level > 0) shift_level(offs, AUDIO_BUFFER_SIZE, level);

			buffer_offset=BUFFER_OFFSET_NONE;
			AudioRemSize-=bytesread;
		}
		if(AudioRemSize<100)
		{
			AudioRemSize=0;
			dur=0;
			sprintf((char*)str3, "%02d:%02d", (int)(dur/60),(int)dur%60);
			AudioPlay_Stop();
		}

		uint32_t time_delay = HAL_GetTick() - tickstart;
		if (time_delay > delay_volume_control) {
			tickstart = HAL_GetTick();

			if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0) == GPIO_PIN_RESET) {
				if (level < 0xff) set_volume(level++);
			}
			if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2) == GPIO_PIN_RESET) {
				if (level > 0) set_volume(level--);
			}
		}

	}
//	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
	AudioPlay_Stop();
}

inline void shift_level(int offset, int len, int level) {
	for(int i = offset; i< len; i+=2) {
		int16_t data = Audio_Buffer[i] | Audio_Buffer[i+1] << 8;
		data = data >> level;
		Audio_Buffer[i] = data;
		Audio_Buffer[i+1] = data >> 8;
	}
}


//------------------------------------------------------
void AudioPlay_HalfTransfer_Callback(void)
{
	buffer_offset=BUFFER_OFFSET_HALF;
}
//------------------------------------------------------
void AudioPlay_Transfer_Callback(void)
{
	buffer_offset=BUFFER_OFFSET_FULL;
	HAL_I2S_Transmit_DMA(&hi2s2,(uint16_t*)&Audio_Buffer[0],
									AUDIO_BUFFER_SIZE/2);
}
