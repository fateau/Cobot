#include "LogWriter.h"

#if defined KING_STAR_IO
#include "Rtapi.h" // for printf
#endif

#define LOGQ_SIZE 60000 //存取60秒資料

LogWriter::LogWriter(char* filePath) :logQ(LOGQ_SIZE)
{
	memcpy(this->filePath, filePath, strlen(filePath));
	isHandleOpen = false;
}
LogWriter::~LogWriter()
{
	closeFile();
}

void LogWriter::setFilePath(char* filePath)
{
	memcpy(this->filePath, filePath, strlen(filePath));
}

void LogWriter::openFile()
{
	//Note: 若該檔案正被點開來看，仍可正常運行 (???)

	HANDLE hnd = CreateFile(filePath, GENERIC_WRITE, 0, NULL, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);
	if (hnd == INVALID_HANDLE_VALUE) {
		printf("LogWriter::error! Fail to create file %s\n", filePath);//TODO: throw exception?
		return;
	}
	hFile = hnd;
	fileSize = 0;
	isHandleOpen = true;
	printf("LogWriter::successfully openfile: %s\n", filePath);
}

void LogWriter::closeFile()
{
	if (isHandleOpen == false) return;
	if (hFile == NULL) return;

	CloseHandle(hFile);
	isHandleOpen = false;
}

//花費時間(ms)/呼叫次數:  0.008/1; 0.02/10;  0.15/100  改成static後，可能需要重測
void LogWriter::write(const char* format, va_list args)
{
	vsprintf(message.text, format, args);
	logQ.enqueue(message);
}
void LogWriter::writeln(const char* format, va_list args)
{
	// 中間:message
	vsprintf(message.text, format, args);

	// 後綴: 換行
	int len = strlen(message.text);
	sprintf(message.text + len, "\r\n");

	// 寫入Queue
	logQ.enqueue(message);
}
void LogWriter::writelnWithTime(const char* format, va_list args)
{
	// 前綴:加上時間 [hh:mm:ss:ms]
	GetLocalTime(&nowTime);
	sprintf(message.text, "%02d/%02d/ %d:%d:%d:%d\t",
		nowTime.wMonth, nowTime.wDay,
		nowTime.wHour, nowTime.wMinute, nowTime.wSecond, nowTime.wMilliseconds);

	// 中間:message
	int len;
	len = strlen(message.text);
	vsprintf(message.text + len, format, args);

	// 後綴: 換行
	len = strlen(message.text);
	sprintf(message.text + len, "\r\n");

	// 寫入Queue
	logQ.enqueue(message);
}

//軌跡規劃，檢查目前數量是否超過限制數量
void LogWriter::writelnError(const char* format, va_list args)
{
	//Q滿了刪除第一筆資料
	if (logQ.isFull())  logQ.dequeue();

	// 中間:message
	vsprintf(message.text, format, args);

	// 後綴: 換行
	int len = strlen(message.text);
	sprintf(message.text + len, "\r\n");

	// 寫入Queue
	logQ.enqueue(message);
}


void LogWriter::flushToFile()
{
	int count = logQ.getCount();
	for (int i = 0; i < count; i++) {
		logQ.dequeue(&tempM);
		WriteFile(hFile, tempM.text, strlen(tempM.text), &dwWriten, NULL);
		fileSize += strlen(tempM.text);
	}
}
void LogWriter::flushToConsole()
{
	for (int i = 0; i < logQ.getCount(); i++) {
		logQ.dequeue(&tempM);
		printf("%s", tempM.text);
	}
}

