#include "NTControl.h"

#include <iostream>
#include <fstream>
#include <string>

#include "string.h"
#include "Math.h"
#include "windows.h"
#include "Com.h"

#include <stdio.h>
#include <conio.h>
#include "wit_c_sdk.h"

#include <thread>
#include <atomic>
#include <chrono>

#define CHANNEL_ID 3 - 1
#define OUTPUT_RATE RRATE_200HZ

static char s_cDataUpdate = 0;
int iComPort = 8;
int iBaud = 9600;
int iAddress = 0x50;

float InitAngle = 0;
std::atomic<bool> running(false);
std::ofstream outputCsv;
std::chrono::_V2::system_clock::time_point start = std::chrono::high_resolution_clock::now();
std::chrono::_V2::system_clock::time_point end;
std::chrono::milliseconds duration;

void ComRxCallBack(char *p_data, UINT32 uiSize)
{
    for (UINT32 i = 0; i < uiSize; i++)
    {
        WitSerialDataIn(p_data[i]);
    }
}
static void AutoScanSensor(void);
static void SensorUartSend(uint8_t *p_data, uint32_t uiSize);
static void CopeSensorData(uint32_t uiReg, uint32_t uiRegNum);
static void DelayMs(uint16_t ms);

void readData(std::atomic<bool> &running, std::ofstream &outputCsv);
void runMotor(NT_INDEX *ntHandle, signed int steps, unsigned int amplitude, unsigned int frequency);

int main()
{
    // 启动粘滑电机
    NT_INDEX ntHandle;
    NT_STATUS result;
    const char *systemLocator = "usb:id:2250716012";
    const char *options = "sync";

    result = NT_OpenSystem(&ntHandle, systemLocator, options);
    if (result != NT_OK)
    {
        printf("error");
        return 1;
    }

    // 启动陀螺仪
    OpenCOMDevice(iComPort, iBaud);
    WitInit(WIT_PROTOCOL_NORMAL, 0X50);
    WitSerialWriteRegister(SensorUartSend);
    WitRegisterCallBack(CopeSensorData);
    AutoScanSensor();
    WitDelayMsRegister(DelayMs);
    WitSetUartBaud(WIT_BAUD_230400);
    int32_t result1 = WitSetOutputRate(OUTPUT_RATE);

    // std::cout << result1 << std::endl;

    signed int steps = 5000;       //-30000-30000
    unsigned int amplitude = 4000;  // 100-4095
    unsigned int frequency = 10000; // 1-18000
    int k = -1;
    std::string a = "009";

    // 打开CSV文件
    std::string filePath = "D:/Code/TrainData/output/";
    std::string filename = filePath + "data_" + std::to_string(steps) + "_" + std::to_string(amplitude) + "_" + std::to_string(frequency) + "_" + a + ".csv";
    outputCsv.open(filename);
    if (!outputCsv.is_open())
    {
        std::cerr << "Error: Could not open file " << filename << std::endl;
        return 1;
    }

    outputCsv << "k,time,angle\n";

    InitAngle = (float)sReg[Roll] / 32768.0f * 180.0f;

    // std::thread dataThread(readData, std::ref(running), std::ref(outputCsv));
    std::thread motorThread(runMotor, &ntHandle, steps, amplitude, frequency);

    motorThread.join();
    // running = false;
    // dataThread.join();

    Sleep(500);
    DWORD runtime = (double)(1000 * steps) / (double)frequency + 500; // 50+
    NT_StepMove_S(ntHandle, CHANNEL_ID, k * steps, amplitude, frequency);
    Sleep(runtime);

    outputCsv.close();
    WitDeInit();
    CloseCOMDevice();
    NT_CloseSystem(ntHandle);
    return 0;
}

// void readData(std::atomic<bool> &running, std::ofstream &outputCsv)
// {
//     float lastAngle = (float)sReg[Roll] / 32768.0f * 180.0f;
//     int i = 1;
//     auto start = std::chrono::high_resolution_clock::now();
//     auto end = start;
//     auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
//     while (running)
//     {
//         end = std::chrono::high_resolution_clock::now();
//         duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
//         float angle = (float)sReg[Roll] / 32768.0f * 180.0f;
//         float deltaAngle = angle - lastAngle;
//         outputCsv << i << "," << duration.count() << "," << deltaAngle << std::endl;
//         i++;
//         // Sleep(1000/outputRate);
//         std::this_thread::sleep_for(std::chrono::milliseconds(5));
//     }
// }

void runMotor(NT_INDEX *ntHandle, signed int steps, unsigned int amplitude, unsigned int frequency)
{
    // Sleep(500);//50
    start = std::chrono::high_resolution_clock::now();
    running = true;
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    DWORD runtime = (double)(1000 * steps) / (double)frequency + 500; // 50+
    NT_StepMove_S(*ntHandle, CHANNEL_ID, steps, amplitude, frequency);
    // Sleep(runtime);
    std::this_thread::sleep_for(std::chrono::milliseconds(runtime));
    running = false;
}

static void DelayMs(uint16_t ms)
{
    Sleep(ms);
}

static void SensorUartSend(uint8_t *p_data, uint32_t uiSize)
{
    SendUARTMessageLength((const char *)p_data, uiSize);
}
static void CopeSensorData(uint32_t uiReg, uint32_t uiRegNum)
{
    s_cDataUpdate = 1;
    // std::cout << uiReg << 't' << sReg[uiReg] << '\t' << uiRegNum << std::endl;

    if (running == true && uiReg == Roll)
    {
        // std::cout << (float)sReg[Roll] / 32768.0f * 180.0f << std::endl;
        static int i = 1;
        end = std::chrono::high_resolution_clock::now();
        duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        float angle = (float)sReg[Roll] / 32768.0f * 180.0f;
        float deltaAngle = angle - InitAngle;
        outputCsv << i << "," << duration.count() << "," << deltaAngle << std::endl;
        i++;

    }
}

/**
 * 自动检测传感器的波特率
 */
static void AutoScanSensor(void)
{
    const uint32_t c_uiBaud[7] = {4800, 9600, 19200, 38400, 57600, 115200, 230400};
    int i, iRetry;

    for (i = 0; i < 7; i++)
    {
        CloseCOMDevice();
        OpenCOMDevice(iComPort, c_uiBaud[i]);
        // SetBaundrate(c_uiBaud[i]);
        iRetry = 2;
        do
        {
            s_cDataUpdate = 0;
            WitReadReg(AX, 3);
            Sleep(100);
            if (s_cDataUpdate != 0)
            {
                printf("%d baud find sensor\r\n\r\n", c_uiBaud[i]);
                return;
            }
            iRetry--;
        } while (iRetry);
    }
    printf("can not find sensor\r\n");
    printf("please check your connection\r\n");
}

// float ReadAngle()
// {
//     uint8_t head = 0x55;
//     uint8_t type = 0x53;
// }