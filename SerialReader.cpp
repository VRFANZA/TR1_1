#include "SerialReader.h"
#include <iostream>

SerialReader::SerialReader(const std::string& portName, int baud) {
    std::string fullPort = "\\\\.\\" + portName; // 例: "COM4" → "\\.\COM4"
    hSerial = CreateFileA(fullPort.c_str(), GENERIC_READ, 0, NULL,
        OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);

    if (hSerial == INVALID_HANDLE_VALUE) {
        std::cerr << "ポートを開けませんでした: " << portName << std::endl;
        return;
    }

    DCB dcb = { 0 };
    dcb.DCBlength = sizeof(dcb);
    GetCommState(hSerial, &dcb);
    dcb.BaudRate = baud;
    dcb.ByteSize = 8;
    dcb.Parity = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    SetCommState(hSerial, &dcb);

    COMMTIMEOUTS timeouts = { 0 };
    timeouts.ReadIntervalTimeout = 50;
    timeouts.ReadTotalTimeoutConstant = 50;
    timeouts.ReadTotalTimeoutMultiplier = 10;
    SetCommTimeouts(hSerial, &timeouts);
}

SerialReader::~SerialReader() {
    if (hSerial != INVALID_HANDLE_VALUE) {
        CloseHandle(hSerial);
    }
}

bool SerialReader::readLine(std::string& outLine) {
    outLine.clear();
    DWORD bytesRead;
    char ch;

    while (true) {
        if (!ReadFile(hSerial, &ch, 1, &bytesRead, NULL) || bytesRead == 0)
            return false;

        if (ch == '\n') break;         // 改行で1行の終わり
        if (ch != '\r') outLine += ch; // CRは無視
    }
    return true;
}