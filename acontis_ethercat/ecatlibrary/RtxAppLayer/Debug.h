#pragma once

#include "LogWriter.h"

#define LOG_FILE_NUM 5 //PMC 11411 //1127
class Debug
{
public:

	static void openFile();
	static void openFile(int id, char* filePath);
	static void openFileWithTimeAsFileName(int id, char* filePath); 

	static void closeFile();

	static void write(int fileId, char* format, ...);
	static void writeln(int fileId, char* format, ...);
	static void flushToFile();
	static void flushToConsole();     //©³¼h°T®§¦C¦L

	static void writelnError(int fileld, char* format, ...);  

private:
	static LogWriter writers[LOG_FILE_NUM];
	static _SYSTEMTIME nowTime; //SE100701
};



