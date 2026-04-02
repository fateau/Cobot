#include "Debug.h"
#include "Shm.h"

extern SHMData* shm;
_SYSTEMTIME	Debug::nowTime; 

char *fullPath = "";

LogWriter Debug::writers[LOG_FILE_NUM] = {

	LogWriter("write to console."),
	LogWriter(fullPath),
	LogWriter(fullPath),
	LogWriter(fullPath), //PMC 11411 //1127
	LogWriter(fullPath)
	// ... you can add file as you need.
	// also change the #define LOG_FILE_NUM in .h

	//Note: abs path is ok; relative path is not successed yet.

};

void Debug::openFile()
{
	for (int i = 1; i < LOG_FILE_NUM; i++)
		writers[i].openFile();

}
void Debug::openFile(int id, char* filePath)
{
	char *mergePath = "";
	if (id == 1) openFileWithTimeAsFileName(id, filePath);
	if (id == 2) mergePath = "\MotorParameter.csv"; //PMC Modified 11411 //1114
	if (id == 3) mergePath = "\Collision_gfmc.csv"; //PMC Modified 11411 //1114
	if (id == 4) mergePath = "\StatusWord_Warning.csv"; //PMC Modified 11411 //1127

	char* tempPath = _strdup(filePath); //拷貝字串位置

	fullPath = strcat(tempPath, mergePath); //拼接字串

	writers[id].closeFile();
	writers[id].setFilePath(fullPath);
	writers[id].openFile();

}
void Debug::closeFile()
{
	for (int i = 1; i < LOG_FILE_NUM; i++)
		writers[i].closeFile();
}
void Debug::flushToFile() //其他資訊列印
{
	for (int i = 1; i < LOG_FILE_NUM; i++)
		writers[i].flushToFile();
	return;
}
void Debug::flushToConsole() //底層顯示列印
{
	writers[0].flushToConsole();
	return;
}

void Debug::write(int fileId, char* format, ...)
{

	if (fileId >= LOG_FILE_NUM) return;

	va_list args;
	va_start(args, format);

	writers[fileId].write (format, args);

	va_end(args);
}
void Debug::writeln(int fileId, char* format, ...) // 第一次約花 0.06 ms, 第二次以後 0.005ms
{
	if (fileId >= LOG_FILE_NUM) return;
	va_list args;
	va_start(args, format);
	writers[fileId].writeln(format, args);
	va_end(args);
}
void Debug::openFileWithTimeAsFileName(int id, char* filePath) //檔案名稱加入時間軸
{
	GetLocalTime(&nowTime);
	char fullName[512];
	sprintf(fullName, "%s\RTX_%d%02d%02d_%02d%02d%02d.csv",
		filePath,
		nowTime.wYear, nowTime.wMonth, nowTime.wDay,
		nowTime.wHour, nowTime.wMinute, nowTime.wSecond);

	writers[id].setFilePath(fullName);
	writers[id].openFile();
}