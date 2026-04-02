#include "IOModule.h"
#include <cstdio>
#include <cstring>
#include "Shm.h"

extern SHMData* shm;

IOModule::IOModule(int mId, U16_T masterID, U16_T slaveAddr, int outByteNum, int inpByteNum)
{
	this->mId		= mId;
	this->masterID	= masterID;
	this->slaveAddr = slaveAddr;
	this->outByteNum = outByteNum;
	this->inpByteNum = inpByteNum;

	inpBytes	= shm->ioModules[mId].inpBytes;
	outBytes	= shm->ioModules[mId].outBytes;
	outBytesCmd = shm->ioModules[mId].outBytesCmd;
}

void IOModule::setOutput(int ind, bool onOff)
{
	if(ind < 0 || ind >= outByteNum*8) return;
	
	if(onOff == true) 
		outBytesCmd[ind/8] |= 1<< (ind%8);
	else 
		outBytesCmd[ind/8] &= ~(1<< (ind%8));
}

bool IOModule::getInput(int ind)
{
	if(ind < 0 || ind >= inpByteNum*8) return false;
	bool ret = (inpBytes[ind/8] >> (ind%8)) & 1;
	return ret;
}

void IOModule::setEcatOutput()
{
	if (!kingIo || !kingIo->pOut) return;
	// Copy command bytes to the process image output region
	int byteLen = outByteNum;
	if (byteLen > 0) {
		memcpy(kingIo->pOut + kingIo->dwOutByteOff, outBytesCmd, byteLen);
	}
}

void IOModule::getEcatInput()
{
	if (!kingIo || !kingIo->pInp) return;
	// Copy process image input region to local buffer
	int byteLen = inpByteNum;
	if (byteLen > 0) {
		memcpy(inpBytes, kingIo->pInp + kingIo->dwInpByteOff, byteLen);
	}
}

void IOModule::getEcatOutput()
{
	if (!kingIo || !kingIo->pOut) return;
	// Read back actual output values from process image
	int byteLen = outByteNum;
	if (byteLen > 0) {
		memcpy(outBytes, kingIo->pOut + kingIo->dwOutByteOff, byteLen);
	}
}
