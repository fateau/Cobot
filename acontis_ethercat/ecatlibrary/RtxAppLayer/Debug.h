#pragma once

/*---------------------------------------------------------------------------
 * Debug.h - Linux-compatible debug/logging
 *---------------------------------------------------------------------------*/

#include "LogWriter.h"
#include <time.h>

#define LOG_FILE_NUM 5

class Debug
{
public:
    static void openFile();
    static void openFile(int id, char* filePath);
    static void openFileWithTimeAsFileName(int id, char* filePath);

    static void closeFile();

    static void write(int fileId, const char* format, ...);
    static void writeln(int fileId, const char* format, ...);
    static void flushToFile();
    static void flushToConsole();

private:
    static LogWriter writers[LOG_FILE_NUM];
};
