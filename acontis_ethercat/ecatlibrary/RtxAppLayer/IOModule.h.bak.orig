#pragma once
#include "Def.h"


class IOModule
{
public:
	IOModule(int mId, U16_T masterID, U16_T slaveAddr, int outNum, int inNum);
	~IOModule(void){};

	#if defined KING_STAR_IO
	IO_ECAT*	kingIo;
	#endif

	int		outByteNum;
	int		inpByteNum;

	void	setOutput(int ind, bool onOff);
	bool	getInput(int ind);

	void	getEcatInput();
	void	getEcatOutput();
	void	setEcatOutput();
	
private:
	int		mId;			// 此模組的編號
	U16_T	masterID;	
	U16_T	slaveAddr;		

	BYTE*	inpBytes;		// Ecat偵測到的值。指到shm->ioModules[mId].inpBytes
	BYTE*	outBytes;		// Ecat偵測到的值。指到shm->ioModules[mId].outBytes
	BYTE*	outBytesCmd;	// 要輸出給Ecat的指令。指到shm->ioModules[mId].outBytesCmd 
};

