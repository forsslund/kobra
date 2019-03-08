//////////////////////////////////////////////////////////////////////////////
//    Copyright 2008 Forsslund Systems AB
//
//    ADrillForce is free software; you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation; either version 2 of the License, or
//    (at your option) any later version.
//
//    ADrillForce is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with ADrillForce; if not, write to the Free Software
//    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//
//    If you wish to use the code in a proprietary project, please contact
//    jonas@forsslundsystems.se for more information.
//
//////////////////////////////////////////////////////////////////////////////

#ifndef _VOLUMES_H
#define	_VOLUMES_H

#include <H3DUtil/Vec3f.h>
#include <H3DUtil/Vec3d.h>
#include "Vec3i.h"
#include "VolumeModel.h"
#include <HAPI/CollisionObjects.h>
#include <HAPI/IntersectionInfo.h>

namespace FS {
/**
 * \brief Helpclass to deal with volumes
 *
 */
class Volumes {
public:

	/**
	 *    VOLUME INTERSECTION WITH SPHERE
	 *    gives volume of intersection and the weighted direction
	 *    based on given iso value. This algorithm is based on
	 *    Agus PhD paper, with an extension with optional iso_variance
	 *
	 */
    static int volumeIntersectionSphere(float &intersectionVolume,
            H3DUtil::ArithmeticTypes::Vec3f &direction,
            VolumeModel *volume_data,
            H3DUtil::ArithmeticTypes::Vec3f center,
            float radius,
            float iso,
            float iso_variance=0);

    static bool pointIntersecting(VolumeModel *volume_model,
            H3DUtil::ArithmeticTypes::Vec3f center);

    static double heightOfSphereSegment(double volume, double radius);

};
}

#endif	/* _VOLUMES_H */

