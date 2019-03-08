/**
 * \file MFieldFromNrrdFile.h
 * \author barrygr
 * \date Dec 8, 2008
 */

#ifndef MFIELDFROMNRRDFILE_H_
#define MFIELDFROMNRRDFILE_H_
#define TEEM_STATIC
#include <teem/air.h>
#include <H3D/MField.h>
#include <stdexcept>
#include <teem/nrrd.h>
#include <vector>
#include <H3DUtil/Image.h>
#include "MFUnsignedInt8.h"
#include <H3D/MFFloat.h>


// The following was required to compile in Windows, don't know what its used for yet
extern const char *nrrdBiffKey;

/**
 *\brief A MField class that can populate itself with data from a nrrd file and which can also save
 * itself to nrrd file
 */
template <class Type>
class MFieldFromNrrdFile :
public H3D::MField<Type>
{
public:
    double spaceOrigin[NRRD_SPACE_DIM_MAX];
    typedef unsigned int uint;

	/**
	 * Constructor
	 */
	MFieldFromNrrdFile()
	: isInitializedFromNrrdFile(false)
	{
	}

	MFieldFromNrrdFile(MFieldFromNrrdFile* object)
	{
    std::cout << "--- contruct from obj" << endl;
    this->noOfDimensions = object->noOfDimensions;
		this->dimensionLengths = object->dimensionLengths;
		this->spacings = object->spacings;
		this->isInitializedFromNrrdFile = object->isInitializedFromNrrdFile;

        unsigned int noOfElements = uint(object->getSize());
		Type* data = new Type[noOfElements];

		for(unsigned int i=0;i<noOfElements;i++)
			data[i] = object->getValueByIndex (i);

        this->setValueFromVoidPtr(data,noOfElements,noOfElements*sizeof(Type),0);

		delete[] data;
	}

	/**
	 * 	Initializes the field's data with data from a nrrd file
	 *	X3D usage:
	 *		<Node fieldName="nrrdFileName.nrrd">
	 * \param The path to the nrrd file
	 */

	inline virtual void setValueFromString( const string& nrrdFilePath )
	{
		NrrdIoState *nrrdIoState;
		char *err;
		Nrrd *nrrdObject = nrrdNew();
		this->nrrdFilePath = nrrdFilePath;

		// read nrrd header
		nrrdIoState = nrrdIoStateNew();
		nrrdIoStateSet(nrrdIoState, nrrdIoStateSkipData, AIR_TRUE);
		if (nrrdLoad(nrrdObject, nrrdFilePath.c_str(), nrrdIoState))
		{
      cout << "File not found: " << nrrdFilePath << ". (In case of timeStamps its ok)\n";
			err = biffGetDone(NRRD);
			nrrdIoState = nrrdIoStateNix(nrrdIoState);
      return; // Not more than that...
      //string errorString(err);
      //throw runtime_error ("Exception within MFieldFromNrrdFile: Trouble reading " + nrrdFilePath + " header: " + errorString);
    }

    cout << "Populating from nrrd file "<<nrrdFilePath<< " into a buffer of type " << typeString << endl ;
    isInitializedFromNrrdFile = true;

    // reset IO state
		nrrdIoState = nrrdIoStateNix(nrrdIoState);

		// check that the file read has the same type as the template type
		if(nrrdObject->type!=nrrdType)
		{
			throw MFieldFromNrrdFileException ("Expected the file " + nrrdFilePath + " to be formatted as " + typeString);
		}
		// the amount of memory required to store the data
        uint bytesRequiredForData = uint(nrrdElementNumber(nrrdObject)*nrrdElementSize(nrrdObject));

		// create our data memory block
		Type* data = new Type[nrrdElementNumber(nrrdObject)];

    noOfDimensions = nrrdObject->dim;

        //cout << "Dimension: "<<nrrdObject->dim<< endl;
        //cout << "Bytes Required for Data: " << bytesRequiredForData << endl;

        nrrdSpaceOriginGet(nrrdObject,spaceOrigin);
        cout << "Origin: " << spaceOrigin[0] << ", " <<
                              spaceOrigin[1] << ", " <<
                              spaceOrigin[2] << "\n";


		for(unsigned int i=0;i<noOfDimensions;i++)
		{
            //cout << "Dimension Lengths: " << nrrdObject->axis[i].size << endl;
            dimensionLengths.push_back(uint(nrrdObject->axis[i].size));

			// check if spaced
			if(airIsNaN(nrrdObject->axis[i].spacing))
			{
        //std::cout << "Expected this axis to have an associated spacing!" << endl;

        // If we have direction vector (sp,0,0), (0,sp,0), (0,0,sp) that will do
        //using nrrd_vector = double[NRRD_SPACE_DIM_MAX];
        //nrrd_vector space_dir[3];
        double d[NRRD_DIM_MAX][NRRD_SPACE_DIM_MAX];
        nrrdAxisInfoGet_nva(nrrdObject,nrrdAxisInfoSpaceDirection, d);
        spacings.push_back(d[i][i]);

      } else {
          cout << "Spacing: " << nrrdObject->axis[i].spacing << endl;
          spacings.push_back(nrrdObject->axis[i].spacing);
      }
		}

		// nrrd object's reference to where data is to be stored
		nrrdObject->data = data;

		if (nrrdLoad(nrrdObject, nrrdFilePath.c_str(), NULL))
		{
			err = biffGetDone(NRRD);
			string errorString(err);
			delete(err);
			throw runtime_error ("Exception within MFieldFromNrrdFile: Trouble reading " + nrrdFilePath + " file: " + errorString);
		}

		// set the memory within the internal vector
        this->setValueFromVoidPtr(data,uint(nrrdElementNumber(nrrdObject)),
                                       uint(bytesRequiredForData),0);

		delete[] data;
		nrrdNix(nrrdObject);
		cout << endl;
	}

	/**
	 * Save the data stored within this field to the nrrd file from which it was initialized from
	 */
	inline virtual void saveDataToNrrdFile()
	{
		saveDataToNrrdFile(this->nrrdFilePath);
	}

	/**
	 * Save the data stored within this field to a nrrd file
	 * \param nrrdFilePath The path to the nrrd file
	 */
	inline virtual void saveDataToNrrdFile(const string& nrrdFilePath)
	{
		if(isInitializedFromNrrdFile)
			saveDataToNrrdFile( nrrdFilePath, noOfDimensions, dimensionLengths, spacings);
		else
			throw MFieldFromNrrdFileException ("Can't save this object to nrrd file since it has not been initialized from such a file");
	}

	/**
	 * 	Save the data stored within this field to a nrrd file having not initialized this object
	 * from a nrrd file earlier. Under such circumstances we must specify dimension and spacing information
	 *	\param nrrdFilePath The path to the nrrd file
	 *	\param noOfDimensions The number of dimensions
	 *	\param dimensionLengths The lengths of each dimention
	 *	\param spacings The spacing of each dimension
	 */
	inline virtual void saveDataToNrrdFile( const string& nrrdFilePath, unsigned int& noOfDimensions, vector<unsigned int>& dimensionLengths, vector<double>& spacings)
	{
		char *err;
		Nrrd *nrrdObject = nrrdNew();

		cout << "Saving data of type "<< typeString << " to nrrd file "<< nrrdFilePath << endl ;

		// since we now have all the information which is read from a nrrd file and are prepared to write it
		// to file, we can in principle state that this object has been initialized from a nrrd file
		isInitializedFromNrrdFile = true;

		nrrdObject->type = nrrdType;
		nrrdObject->dim = noOfDimensions;

		for(unsigned int i=0;i<noOfDimensions;i++)
		{
			nrrdObject->axis[i].size = dimensionLengths[i];
			nrrdObject->axis[i].spacing = spacings[i];
		}

        unsigned int noOfElements = uint(this->getValue().size());

		// create our data memory block
		Type* data = new Type[noOfElements];

		// get the data stored by this
        this->getValueAsVoidPtr(data,noOfElements,noOfElements*sizeof(Type),0);

		// nrrd api's reference to where data is to be stored
		nrrdObject->data = data;

		if (nrrdSave(nrrdFilePath.c_str(), nrrdObject, NULL))
		{
			err = biffGetDone(NRRD);
			string errorString(err);
			delete(err);
			throw runtime_error ("Exception within MFieldFromNrrdFile: Trouble writing " + nrrdFilePath + " file: " + errorString);
		}

		delete[] data;
		nrrdNix(nrrdObject);
	}

	/**
	 *  If we have no nrrd file then we can initialize our data to correspond in size to that of
	 *  image's size. All values stored within the field are initialized with the value valueToInitializeWith
	 *  \param The image object whose dimensions we are to use
	 *  \param The value to use when initializing the values in this objects data
	 */
	inline void setValueFromImageData(H3DUtil::Image *image, Type valueToInitializeWith)
	{
		this->noOfDimensions = 3;						// presume images are 3-D
		dimensionLengths.clear();
		dimensionLengths.push_back(image->width());
		dimensionLengths.push_back(image->height());
		dimensionLengths.push_back(image->depth());

		unsigned int noOfElements = image->width()*image->height()*image->depth();
		Type* data = new Type[noOfElements];

		for(unsigned int i=0;i<noOfElements;i++)
		{
			data[i] = valueToInitializeWith;
		}

        this->setValueFromVoidPtr(data,noOfElements,noOfElements*sizeof(Type),0);

		delete[] data;
	}

	/**
	 * Takes a field as parameter and populates its data with a masked version of this
	 * objects data. The result is that every value in this that is 0 remains 0 in the
	 * passed in parameter and that every value that is not 0 becomes 1.
	 * \param This field's data will be populated according to the data in the calling object
	 */
	virtual inline void GetMaskedVersion(H3D::MFFloat& field)
	{
        unsigned int noOfElements = uint(this->getValue().size());

		float* fieldData = new float[noOfElements];

		typename vector<Type>::const_iterator it;
		it = MFieldFromNrrdFile::getValue().begin();
		int i = 0;
		for(it = MFieldFromNrrdFile::getValue().begin();it<MFieldFromNrrdFile::getValue().end();it++,i++)
		{
			if(*it!=0){
				fieldData[i] = 1.0;
			}
			else
			{
				fieldData[i] = 0.0;
			}
		}
		// set the data stored by field with the masked version
		field.setValueFromVoidPtr(fieldData,noOfElements,noOfElements*sizeof(float),0);

		delete[] fieldData;
	}

	/**
	 * Get the number of dimensions stored within the nrrd file
	 * /return The number of dimensions
	 */
	inline unsigned const int& GetNoOfDimensions()
	{
		return noOfDimensions;
	}

	/**
	 * Set the number of dimensions to be stored within the nrrd file
	 * /param noOfDimensions The number of dimensions
	 */
	inline void SetNoOfDimensions(unsigned const int& noOfDimensions)
	{
		this->noOfDimensions = noOfDimensions;
	}

	/**
	 * Get the lengths of each dimension stored in the nrrd file
	 * /return A vector of dimension lengths
	 */
	inline vector<unsigned int>& GetDimensionLengths()
	{
		return dimensionLengths;
	}

	/**
	 * Set the lengths of each dimension stored in the nrrd file
	 * /param A vector of dimension lengths
	 */
	inline void SetDimensionLengths(vector<unsigned int>& dimensionLengths)
	{
		vector<unsigned int>::iterator i;

		this->dimensionLengths.clear();

		for(i=dimensionLengths.begin();i<dimensionLengths.end();i++)
			this->dimensionLengths.push_back(i);
	}

	/**
	 * Get the spacing of each dimension stored in the nrrd file
	 * /return A vector of each spacing
	 */
	inline vector<double>& GetSpacings()
	{
		return spacings;
	}

	/**
	 * Set the spacing of each dimension stored in the nrrd file
	 * /param A vector of each spacing
	 */
	inline void SetSpacings(vector<double>& spacings)
	{
		vector<double>::iterator i;

		this->spacings.clear();

		for(i=spacings.begin();i<spacings.end();i++)
			this->spacings.push_back(i);
	}

	/**
	 * Is this file initialized from a nrrd file
	 * / return If this file has been initialized form a nrrd file
	 */
	inline bool& IsInitializedFromNrrdFile()
	{
		return isInitializedFromNrrdFile;
	}

protected:
	int nrrdType;							// nrrd api uses an enum to differentiate each data type
	string typeString;

private:
	unsigned int noOfDimensions;
	vector<unsigned int> dimensionLengths;
	vector<double> spacings;				// the spacing between voxels for each axis
	string nrrdFilePath;
	bool isInitializedFromNrrdFile;

	/**
	 * Exception class for MFieldFromNrrdFile
	 */
	class MFieldFromNrrdFileException: public runtime_error
	{
	public:

		/**
		 * Constructor
		 */
		MFieldFromNrrdFileException(const string& what) :
			runtime_error("Exception within MFieldFromNrrdFile: " + what)
			{
			}
	};

};
#endif /* MFIELDFROMNRRDFILE_H_ */
