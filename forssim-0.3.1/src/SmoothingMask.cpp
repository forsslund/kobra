//////////////////////////////////////////////////////////////////////////////
//
/// Author: Sudanthi Wijewickrema
/// 		Health and Biomedical Informatics Research Unit
/// 		University of Melbourne
///
//////////////////////////////////////////////////////////////////////////////

#include "SmoothingMask.h"

#include <functional>
#ifndef M_PI
const double M_PI = 3.141592653589793;
#endif

using namespace FS;

// Add this node to the H3DNodeDatabase system.
H3D::H3DNodeDatabase SmoothingMask::database( "SmoothingMask",
                                         &(newInstance<SmoothingMask>),
                                         typeid( SmoothingMask ),
                                         &H3D::X3DChildNode::database );

// Define the fields that can be accessed
namespace SmoothingMaskInternals {
  FIELDDB_ELEMENT( SmoothingMask, maskRadius, INPUT_OUTPUT );
  FIELDDB_ELEMENT( SmoothingMask, smoothFunc, INPUT_OUTPUT );
  FIELDDB_ELEMENT( SmoothingMask, smoothFuncParam, INPUT_OUTPUT );
}

// Constructor
SmoothingMask::SmoothingMask( Inst< SFInt32 > _maskRadius,
								Inst< SFString > _smoothFunc,
								Inst< MFFloat > _smoothFuncParam):
  maskRadius(_maskRadius),
  smoothFunc(_smoothFunc),
  smoothFuncParam(_smoothFuncParam),
  maskSize(0),
  smoothingMask(NULL),
  maskSum(0.0)
  {

  type_name = "SmoothingMask";
  database.initFields( this ); 

  maskRadius->setValue(2);
  smoothFunc->setValue("Gauss");
  vector<float> funcParams(1, 1.0);
  smoothFuncParam->setValue(funcParams);
}

// Destructor
SmoothingMask::~SmoothingMask(){
	delete [] smoothingMask;
}

// Initialze mask
void SmoothingMask::initialize(){
	createMask();
	X3DChildNode::initialize();
}

// Create mask if the given parameters are valid
void SmoothingMask::createMask(){

	// Get field values
	string smoothingFunc = smoothFunc->getValue();
	int maskRad = maskRadius->getValue();
	vector<float> param = smoothFuncParam->getValue();

	// Not a valid mask if maskRad is leass than 1
	if (maskRad <= 0) return;

	// Get mask info
	maskSize = 2*maskRad + 1;
    int maskVoxels = maskSize*maskSize*maskSize;
	float maskMid = (float)maskRad;
	unsigned int maskIndex = 0;

	// Convert to lower case
	locale loc("");
	const ctype<char>& ct = use_facet<ctype<char> >(loc);
	transform(smoothingFunc.begin(), smoothingFunc.end(), smoothingFunc.begin(), std::bind1st(std::mem_fun(&ctype<char>::tolower), &ct));

	// Create the gaussian kernel. values are negligible after 3*sigma. So for a nicer
	// smoothing, maskRad >= 3*sigma - 1. The larger the maskRad, the slower the smoothing.
	// Threrfore, for best results, make maskRad ~= 3*sigma - 1
	if (smoothingFunc.compare("gauss") == 0) {

		// Needs a parameter
		if (param.size() <= 0) return;

		// Only create the mask if a valid function is given
		smoothingMask = new float[maskVoxels]();
		maskSum = 0;

		float sigma = param[0];
		float gaussVal = 0;
	
		// Calculate the gaussian values for each cell
		for (unsigned int z = 0; z < maskSize; z++) {
			for (unsigned int y = 0; y < maskSize; y++) {
				for (unsigned int x = 0; x < maskSize; x++) {
                    gaussVal = float(exp(-(pow(x-maskMid, 2) + pow(y-maskMid, 2) + pow(z-maskMid, 2)) / (2 * pow(sigma, 2))) / (pow(sqrt(2 * M_PI) * sigma, 3)));
					maskIndex = z*maskSize*maskSize + y*maskSize + x;
					smoothingMask[maskIndex] = gaussVal;
					maskSum = maskSum + gaussVal;
				}
			}
		}
	} else if (smoothingFunc.compare("mean") == 0) {

		// Only create the mask if a valid function is given
		smoothingMask = new float[maskVoxels]();
        maskSum = float(maskVoxels);

		// In a mean smoothing mask, all elements have the same value
		for (int i = 0; i < maskVoxels; i++) {
			smoothingMask[i] = 1.0;
		}
	} else {
		cout << "Error! No smoothing function provided" << endl;
	}
}

// Return the smoothing mask
float * SmoothingMask::getMask(){
	return smoothingMask;
}

// Return the sum of all elements of the smoothing mask
float SmoothingMask::getMaskSum(){
	return maskSum;
}

// Return the size of the smoothing mask. Its volume is maskSize^3
unsigned int SmoothingMask::getMaskSize(){
	return maskSize;
}
