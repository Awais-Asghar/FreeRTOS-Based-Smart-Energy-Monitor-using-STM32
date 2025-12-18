#include "mbed.h"
#include <cmath>

// ---------- SERIAL ----------
BufferedSerial pc(USBTX, USBRX, 115200);
BufferedSerial esp32(PC_6, PC_7, 115200);

// ---------- ADC ----------
AnalogIn voltage_adc(PA_0);
AnalogIn current_adc(PA_1);

// ---------- PARAMETERS ----------
#define NUM_SAMPLES 400
#define ADC_REF 3.3f

#define VOLTAGE_GAIN 100.0f   // calibrate later
#define CURRENT_GAIN 20.0f    // calibrate later

float v_buf[NUM_SAMPLES];
float i_buf[NUM_SAMPLES];

// ---------- FUNCTIONS ----------
float calculate_rms(float *buf)
{
    float mean = 0.0f;
    for (int i = 0; i < NUM_SAMPLES; i++)
        mean += buf[i];
    mean /= NUM_SAMPLES;

    float sum = 0.0f;
    for (int i = 0; i < NUM_SAMPLES; i++) {
        float val = buf[i] - mean;
        sum += val * val;
    }

    return sqrt(sum / NUM_SAMPLES);
}

// ---------- MAIN ----------
int main()
{
    pc.write("STM32 Energy Monitor Started\r\n", 30);

    while (true) {
        // Sample ADC
        for (int i = 0; i < NUM_SAMPLES; i++) {
            v_buf[i] = voltage_adc.read();
            i_buf[i] = current_adc.read();
            wait_us(250); // ~4kHz sampling
        }

        // RMS (normalized)
        float vrms_n = calculate_rms(v_buf);
        float irms_n = calculate_rms(i_buf);

        // Convert to real-world units
        float vrms = vrms_n * ADC_REF * VOLTAGE_GAIN;
        float irms = irms_n * ADC_REF * CURRENT_GAIN;

        // Format UART packet
        char msg[64];
        int len = snprintf(msg, sizeof(msg),
                           "<VRMS=%.2f,IRMS=%.2f>\n",
                           vrms, irms);

        // Send to ESP32
        esp32.write(msg, len);

        // Debug to PC
        pc.write(msg, len);

        ThisThread::sleep_for(1s);
    }
}
