/*
 * MFDoubleFromNrrdFile.h
 *
 *  Created on: Dec 8, 2008
 *      Author: barrygr
 */

#ifndef MFDOUBLEFROMNRRDFILE_H_
#define MFDOUBLEFROMNRRDFILE_H_

#include "MFieldFromNrrdFile.h"
#include <teem/nrrd.h>

class MFDoubleFromNrrdFile : public MFieldFromNrrdFile<H3D::H3DDouble>
{
public:
	MFDoubleFromNrrdFile(){nrrdType=nrrdTypeDouble;typeString="double";}
	MFDoubleFromNrrdFile(MFDoubleFromNrrdFile* object)
		: MFieldFromNrrdFile<H3D::H3DDouble>(object)
	{
		nrrdType=nrrdTypeDouble;
		typeString="double";
	}
};
#endif /* MFDOUBLEFROMNRRDFILE_H_ */
