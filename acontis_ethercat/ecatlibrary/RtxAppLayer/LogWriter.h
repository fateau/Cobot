#pragma once

/*---------------------------------------------------------------------------
 * LogWriter.h - Linux-compatible log writer
 *---------------------------------------------------------------------------*/

#include <cstdio>
#include <cstdarg>
#include <cstring>
#include <pthread.h>

struct LogMessage
{
    char text[512];
};

class LogWriter
{
public:
    LogWriter(const char* filePath) : fp(nullptr), isOpen(false), fileSize(0) {
        strncpy(this->filePath, filePath, sizeof(this->filePath) - 1);
        this->filePath[sizeof(this->filePath) - 1] = '\0';
        pthread_mutex_init(&writeMutex, nullptr);
    }
    ~LogWriter() {
        closeFile();
        pthread_mutex_destroy(&writeMutex);
    }

    void openFile() {
        if (strlen(filePath) == 0) return;
        fp = fopen(filePath, "w");
        if (fp) isOpen = true;
    }
    void closeFile() {
        if (fp) { fclose(fp); fp = nullptr; }
        isOpen = false;
    }
    void setFilePath(const char* path) {
        strncpy(filePath, path, sizeof(filePath) - 1);
        filePath[sizeof(filePath) - 1] = '\0';
    }

    void write(const char* format, va_list args) {
        pthread_mutex_lock(&writeMutex);
        vsnprintf(message.text, sizeof(message.text), format, args);
        if (isOpen && fp) {
            fprintf(fp, "%s", message.text);
        }
        // Also buffer for console
        consoleBuffer[consoleCount % 256] = message;
        consoleCount++;
        pthread_mutex_unlock(&writeMutex);
    }
    void writeln(const char* format, va_list args) {
        pthread_mutex_lock(&writeMutex);
        vsnprintf(message.text, sizeof(message.text) - 2, format, args);
        strcat(message.text, "\n");
        if (isOpen && fp) {
            fprintf(fp, "%s", message.text);
        }
        consoleBuffer[consoleCount % 256] = message;
        consoleCount++;
        pthread_mutex_unlock(&writeMutex);
    }

    void flushToFile() {
        if (fp) fflush(fp);
    }
    void flushToConsole() {
        pthread_mutex_lock(&writeMutex);
        while (consolePrinted < consoleCount) {
            printf("%s", consoleBuffer[consolePrinted % 256].text);
            consolePrinted++;
        }
        fflush(stdout);
        pthread_mutex_unlock(&writeMutex);
    }

    unsigned int fileSize;

private:
    FILE*   fp;
    bool    isOpen;
    char    filePath[256];

    LogMessage message;
    LogMessage consoleBuffer[256];
    int consoleCount = 0;
    int consolePrinted = 0;
    pthread_mutex_t writeMutex;
};
