/*
 * MFUnsignedInt8FromNrrdFile.h
 *
 *  Created on: Dec 8, 2008
 *      Author: barrygr
 */

#ifndef MFUNSIGNEDINT8FROMNRRDFILE_H_
#define MFUNSIGNEDINT8FROMNRRDFILE_H_

#include "MFieldFromNrrdFile.h"
#include <teem/nrrd.h>

class MFUnsignedInt8FromNrrdFile : public MFieldFromNrrdFile<unsigned char>
{
public:
	MFUnsignedInt8FromNrrdFile(){nrrdType=nrrdTypeUChar;typeString="unsigned char";}
};
#endif /* MFUNSIGNEDINT8FROMNRRDFILE_H_ */
