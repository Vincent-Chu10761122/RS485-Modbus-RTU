#include <iostream>
#include <windows.h>
#include <cstring>
using namespace std;

#define POLY 0x1021
#define INIT 0xFFFF

//計算CRC16檢查值的函數
unsigned short crc16 (unsigned char *data, int len){
    unsigned short crc = INIT;
    for (int i=0; i<len; i++){
        crc ^= (unsigned short) data[i] << 8;
        for (int j=0; j<8; j++){
            if (crc & 0x8000){
                crc = (crc << 1) ^ POLY;
            }
            else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

int main()
{   
    unsigned char data[] = {0x12, 0x34, 0x56, 0x78, 0x9A};
    int len = sizeof(data) / sizeof(data[0]);

    //計算CRC檢查值
    unsigned short crc = crc16(data, len);

    //將CRC16檢查值轉換為字串並輸出
    char crc_str[5];
    sprintf(crc_str, "%04X", crc);
    cout << "CRC16: " << crc_str << endl;

    //定義容器
    HANDLE hSerial;
    DCB dcbSerialParams = {0};
    COMMTIMEOUTS timeouts = {0};
    uint8_t szBuff[256];
    DWORD dwBytesRead = 0;

    //開啟串口
    hSerial = CreateFile("COM6",GENERIC_READ|GENERIC_WRITE,0,NULL,OPEN_EXISTING,FILE_ATTRIBUTE_NORMAL,NULL);

    if (hSerial == INVALID_HANDLE_VALUE){
        std::cout << "Failed to open serial port." << std::endl;
        return 1;
    }

    //設定串口參數
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
    if (!GetCommState(hSerial, &dcbSerialParams)){
        std::cout << "Failed to get serial parameters." << std::endl;
        CloseHandle(hSerial);
        return 1;
    }

    dcbSerialParams.BaudRate = CBR_9600;
    dcbSerialParams.ByteSize = 8;
    dcbSerialParams.StopBits = ONESTOPBIT;
    dcbSerialParams.Parity = NOPARITY;

    if (!SetCommState(hSerial,&dcbSerialParams)){
        std::cout << "Failed to set serial parameters." << std::endl;
        CloseHandle(hSerial);
        return 1;
    }

    //設定串口讀取超時時間
    timeouts.ReadIntervalTimeout = 50;
    timeouts.ReadTotalTimeoutConstant = 50;
    timeouts.ReadTotalTimeoutMultiplier = 10;

    if (!SetCommTimeouts(hSerial,&timeouts)){
        std::cout << "Failed to set serial timeouts." << std::endl;
        return 1;
    }
    
    DWORD dwBytesWritten = 0;
    uint8_t szData[7] = {0xFF, 0x55, 0xA0, 0xDB, 0x00, 0x00, 0x00};
    if (!WriteFile(hSerial,szData,sizeof(szData),&dwBytesWritten,NULL)){
        std::cout << "Failed to write to serial port." << std::endl;
        CloseHandle(hSerial);
        return 1;
    }

    while (true){
        dwBytesRead = 0;
        if (!ReadFile(hSerial, szBuff, sizeof(szBuff), &dwBytesRead, NULL)){
            std::cout << "Failed to read from serial port." << std::endl;
            break;
        }
        else {
            std::cout << "Received data : " << szBuff << std::endl;
        }
    }
    /*
    //寫入資料到串口
    DWORD dwBytesWritten = 0;
    uint8_t szData[7] = {0xFF, 0x55, 0xA0, 0xDB, 0x00, 0x7B, 0x0A};
    if (!WriteFile(hSerial,szData,sizeof(szData),&dwBytesWritten,NULL)){
        std::cout << "Failed to write to serial port." << std::endl;
        CloseHandle(hSerial);
        return 1;
    }
    
    //讀取串口資料
    if (!ReadFile(hSerial,szBuff,sizeof(szBuff),&dwBytesRead,NULL)){
        std::cout << "Failed to read from serial port." << std::endl;
        CloseHandle(hSerial);
        return 1;
    }
    */
    /*
    else {
        //印出讀入資料
        std::cout << "Received data : " << szBuff << std::endl;
    }
    */
    /*
    else{
        std::string data(szBuff, dwBytesRead);
        std::cout << "Received data : " << data.substr(0, 10) << std::endl;
    }
    /*
    else{
        std::cout << "Received data : " << std::string(szBuff, dwBytesRead) << std::endl;
    }
    */

    //關閉串口
    CloseHandle(hSerial);

    return 0;
}