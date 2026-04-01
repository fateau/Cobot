#pragma once
#include <windows.h>
#include <stdarg.h>
#include "MyQueue.h"

struct LogMessage
{
	char text[512];
};

class LogWriter
{
public:
	LogWriter(char* filePath);
	~LogWriter();

	void openFile();
	void closeFile();
	void setFilePath(char* filePath);

	void write(const char* format, va_list args);
	void writeln(const char* format, va_list args);
	void writelnWithTime(const char* format, va_list args);
	void writelnError(const char* format, va_list args); //SE1007 軌跡規劃

	void flushToFile();
	void flushToConsole();

	unsigned int fileSize;

private:
	HANDLE hFile;
	bool isHandleOpen;//to avoid close handle twice. (would cause exception in debug mode)
	char filePath[256];

	LogMessage message;
	LogMessage tempM;
	DWORD dwWriten;
	MyQueue<LogMessage> logQ;
	_SYSTEMTIME	nowTime;
};

