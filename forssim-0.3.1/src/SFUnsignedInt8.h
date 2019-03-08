/*
 * SFUnsignedInt8.h
 *
 *  Created on: Jan 8, 2008
 *      Author: barrygr
 */

#ifndef __SFUNSIGNEDINT8_H__
#define __SFUNSIGNEDINT8_H__

#include <H3D/SField.h>
#include <stdexcept>
#include <climits>
#include <string>

using namespace std;

/// The SFUnsignedInt8 field contains one unsigned 8-bit integer.
class SFUnsignedInt8: public H3D::SField< unsigned char >
{
public:
	SFUnsignedInt8() {}
	SFUnsignedInt8( const unsigned char &_value ): H3D::SField< unsigned char >( _value ){}

	/**
	 *  Receives value in the form of a string from X3D File
	 */
	inline virtual void setValueFromString( const string &s )
	{
		string tempStr = s;

		if(tempStr.empty())
			throw SFUnsignedInt8Exception("Failed to initialize SFUnsignedInt8 object via a string since the initialization string is empty");

		// remove leading whitespaces
		tempStr.erase(tempStr.begin(), tempStr.begin() + tempStr.find_first_not_of(' '));

		// remove trailing whitespaces
		tempStr.erase(tempStr.begin() + tempStr.find_last_not_of(' ') + 1, tempStr.end());

		// insure string only contains numbers
		string::iterator i;
		for(i = tempStr.begin();i < tempStr.end();i++)
			if(!(*i>='0' && *i<='9'))
				throw SFUnsignedInt8Exception("Failed to initialize SFUnsignedInt8 object via a string since the initialization string contains non numeric content");

		// convert string to int
		int tempInt = atoi(tempStr.c_str());

		// convert int to unsigned char and set value
		if(tempInt>=0 && tempInt <= UCHAR_MAX)
			this->setValue((unsigned char)tempInt);
		else
		{
			stringstream failStringStream;
			failStringStream << "Failed to initialize SFUnsignedInt8 object via a string since the string contains a number less than 0 or greater than " << UCHAR_MAX;
			throw SFUnsignedInt8Exception(failStringStream.str());
		}
	}

private:
	/**
	 * Exception class for SFUnsignedInt8
	 */
	class SFUnsignedInt8Exception: public runtime_error
	{
	public:
		/**
		 * Constructor
		 */
		SFUnsignedInt8Exception(const string& what) :
			runtime_error("Exception within SFUnsignedInt8: " + what)
			{
			}
	};
};

#endif



