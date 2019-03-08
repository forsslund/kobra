//////////////////////////////////////////////////////////////////////////////
//
/// Author: Sudanthi Wijewickrema
/// 		Health and Biomedical Informatics Research Unit
/// 		University of Melbourne
///
//////////////////////////////////////////////////////////////////////////////

#ifndef	SMOOTHINGMASK_H__
#define SMOOTHINGMASK_H__

#include <H3D/Node.h>
#include <H3D/SFInt32.h>
#include <H3D/SFString.h>
#include <H3D/MFFloat.h>
#include <H3D/X3DChildNode.h>

using namespace H3D;
using namespace std;

namespace FS {

  class SmoothingMask : public X3DChildNode {
	  public:

		  /// Constructor.
		  SmoothingMask(Inst< SFInt32 > _maskRadius			= 0,
						  Inst< SFString > _smoothFunc 		= 0,
						  Inst< MFFloat > _smoothFuncParam 	= 0);

		  /// Destructor
		  ~SmoothingMask();

		  /// Create the mask and initialize variables
		  virtual void initialize();

		  /// Return the smoothing mask as an array
		  float * getMask();

		  /// Return the sum of the values in the smoothing mask
		  float getMaskSum();

		  /// Return the size of the mask. Here we only use 3D masks that are equal in length in all 3
		  /// dimensions
		  unsigned int getMaskSize();

		  /// Radius of the mask that is to be created
		  auto_ptr< SFInt32 > maskRadius;

		  /// Function that should be used for smoothing. Currently, only 'Gauss' and 'Mean' are allowed.
		  /// Any other function can be added by modifying the initialize() function.
		  auto_ptr< SFString > smoothFunc;

		  /// Parameters required by the smoothing function. For example, the Gaussian function
		  /// requires one parameter (sigma)
		  auto_ptr< MFFloat > smoothFuncParam;

		  // The H3DNodeDatabase for this node.
		  static H3DNodeDatabase database;

		  protected:
		  	/// Size of the smoothing mask (maskSize = 2*maskRad + 1)
			unsigned int maskSize;

			/// Array that represents the smoothing mask
			float *smoothingMask;

			/// Sum of all the elements in the smoothing mask
			float maskSum;

			/// Create the smoothing mask and update variables
			void createMask();
  }; 
}

#endif
