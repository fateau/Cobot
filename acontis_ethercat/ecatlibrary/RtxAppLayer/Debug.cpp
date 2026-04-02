/*---------------------------------------------------------------------------
 * Debug.cpp - Linux-compatible debug/logging implementation
 *---------------------------------------------------------------------------*/
#include "Debug.h"
#include "Shm.h"
#include <cstdio>
#include <cstdarg>
#include <cstring>
#include <ctime>

extern SHMData* shm;

static const char* emptyPath = "";

LogWriter Debug::writers[LOG_FILE_NUM] = {
    LogWriter(""),  // console
    LogWriter(""),
    LogWriter(""),
    LogWriter(""),
    LogWriter("")
};

void Debug::openFile()
{
    for (int i = 1; i < LOG_FILE_NUM; i++)
        writers[i].openFile();
}
void Debug::openFile(int id, char* filePath)
{
    char fullPath[512];
    if (id == 1) { openFileWithTimeAsFileName(id, filePath); return; }
    if (id == 2) snprintf(fullPath, sizeof(fullPath), "%s/MotorParameter.csv", filePath);
    else if (id == 3) snprintf(fullPath, sizeof(fullPath), "%s/Collision_gfmc.csv", filePath);
    else if (id == 4) snprintf(fullPath, sizeof(fullPath), "%s/StatusWord_Warning.csv", filePath);
    else return;

    writers[id].closeFile();
    writers[id].setFilePath(fullPath);
    writers[id].openFile();
}
void Debug::closeFile()
{
    for (int i = 1; i < LOG_FILE_NUM; i++)
        writers[i].closeFile();
}
void Debug::flushToFile()
{
    for (int i = 1; i < LOG_FILE_NUM; i++)
        writers[i].flushToFile();
}
void Debug::flushToConsole()
{
    writers[0].flushToConsole();
}

void Debug::write(int fileId, const char* format, ...)
{
    if (fileId >= LOG_FILE_NUM) return;
    va_list args;
    va_start(args, format);
    writers[fileId].write(format, args);
    va_end(args);
}
void Debug::writeln(int fileId, const char* format, ...)
{
    if (fileId >= LOG_FILE_NUM) return;
    va_list args;
    va_start(args, format);
    writers[fileId].writeln(format, args);
    va_end(args);
}
void Debug::openFileWithTimeAsFileName(int id, char* filePath)
{
    time_t now = time(nullptr);
    struct tm* t = localtime(&now);
    char fullName[512];
    snprintf(fullName, sizeof(fullName), "%s/RTX_%04d%02d%02d_%02d%02d%02d.csv",
        filePath,
        t->tm_year + 1900, t->tm_mon + 1, t->tm_mday,
        t->tm_hour, t->tm_min, t->tm_sec);

    writers[id].setFilePath(fullName);
    writers[id].openFile();
}
