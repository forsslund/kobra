/*
 * MFUnsignedInt8.h
 *
 *  Created on: Dec 8, 2008
 *      Author: barrygr
 */
#ifndef __MFUNSIGNEDINT8_H__
#define __MFUNSIGNEDINT8_H__

#include <H3D/MField.h>
#include <stdexcept>
#include <climits>
#include <string>

using namespace std;

class MFUnsignedInt8 : public H3D::MField< unsigned char >
{
public:
	MFUnsignedInt8(){}
	MFUnsignedInt8( size_type sz ): H3D::MField< unsigned char >( sz ){}

	/**
	 * Receives values in the form of a string from X3D File
     *
     * it is needed in our nodes that uses integers in the x3d file to describe segments etc
	 */

	inline virtual void setValueFromString( const string &s )
	{
		string tempStr = s;

		if(tempStr.empty())
			throw MFUnsignedInt8Exception("Failed to initialize MFUnsignedInt8 object via a string since the initialization string is empty");

		// insure string only contains numbers and spaces
		string::iterator i;
		for(i = tempStr.begin();i < tempStr.end();i++)
            if(!((*i>='0' && *i<='9') || *i == ' '))
				throw MFUnsignedInt8Exception("Failed to initialize MFUnsignedInt8 object via a string since the initialization string contains non numeric content");

		// split the string by spaces
		vector<string> stringList;
		unsigned int charCount = 0;
		string::iterator firstChar = tempStr.begin();
		for(i = tempStr.begin();i < tempStr.end();i++)
			if(*i==' ')
			{
				if(charCount)
				{
					string tempString(firstChar, i);
					stringList.push_back(tempString);
				}

				firstChar = i+1;
				charCount=0;
			}
			else
				charCount++;
		if(charCount)
		{
			string tempString(firstChar, tempStr.end());
			stringList.push_back(tempString);
		}

		// create a buffer to store the unsigned chars in
		unsigned char *valueArray = new unsigned char[stringList.size()];

		vector<string>::iterator j;
		int k=0;
		for(j = stringList.begin();j < stringList.end();j++,k++)
		{
			// convert string value to int
			int tempInt = atoi(j->c_str());

			// convert int to unsigned char
			if(tempInt>=0 && tempInt <= UCHAR_MAX)
				valueArray[k] = (unsigned char) tempInt;
			else
			{
				stringstream failStringStream;
				failStringStream << "Failed to initialize MFUnsignedInt8 object via a string since the string contains a number less than 0 or greater than " << UCHAR_MAX;
		                delete[] valueArray;
				throw MFUnsignedInt8Exception(failStringStream.str());
			}
		}

        typedef unsigned int uint;
        this->setValueFromVoidPtr(valueArray,uint(stringList.size()),
                         uint(stringList.size()*sizeof(unsigned char)));

		delete[] valueArray;
	}



private:

	/**
	 * Exception class for MFUnsignedInt8
	 */
	class MFUnsignedInt8Exception: public runtime_error
	{
	public:
		/**
		 * Constructor
		 */
		MFUnsignedInt8Exception(const string& what) :
			runtime_error("Exception within MFUnsignedInt8: " + what)
			{
			}
	};
};

#endif

