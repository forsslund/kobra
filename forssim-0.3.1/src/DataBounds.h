
#ifndef _DATABOUNDS_H
#define	_DATABOUNDS_H

namespace FS {

  // data structure to save the bounds of data
  struct DataBounds {
	float xMin, yMin, zMin, xMax, yMax, zMax;
		
	DataBounds():
	  xMin(numeric_limits<float>::max()), yMin(numeric_limits<float>::max()), zMin(numeric_limits<float>::max()),
	  xMax(numeric_limits<float>::min()), yMax(numeric_limits<float>::min()), zMax(numeric_limits<float>::min()) {}
  };

}

#endif	/*_DATABOUNDS_H*/