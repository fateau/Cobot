#include "IOModule.h"
#include <stdio.h> // for printing
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
	if(ind < 0 || ind >= inpByteNum*8) false;
	bool ret = (inpBytes[ind/8] >> (ind%8)) & 1;
	return ret;
}

void IOModule::setEcatOutput()
{
#ifdef KING_STAR_IO
	WriteBits( 
		kingIo->pOut, 
		outBytesCmd,
		kingIo->dwOutOff,
		kingIo->dwOutLength		//bit#.
		);
#endif
}
void IOModule::getEcatInput()
{
	#ifdef KING_STAR_IO
	ReadBits( 
		kingIo->pInp, 
		inpBytes,
		kingIo->dwInpOff,
		kingIo->dwInpLength		//bit#.
	);
	#endif
}
void IOModule::getEcatOutput()
{
	#ifdef KING_STAR_IO
	ReadBits( 
		kingIo->pOut, 
		outBytes,
		kingIo->dwOutOff,
		kingIo->dwOutLength		//bit#.
	);
	#elif defined WIN32_SIMULATE
	for(int i = 0; i< MAX_OUT_BYTE_PER_IO; i++) {
		outBytes[i] = outBytesCmd[i];
	}
	#endif
}