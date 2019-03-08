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

#ifndef _VEC3I_H
#define	_VEC3I_H

namespace FS {
    /** \brief A 3-dimensional integer vector

    */
    class Vec3i {
        public:
            Vec3i(int _x, int _y, int _z):x(_x),y(_y),z(_z){
            }

            Vec3i():x(0),y(0),z(0){
            }

            friend bool operator<(const Vec3i& vec3i1, const Vec3i& vec3i2)
            {
            	if(vec3i1.z<vec3i2.z)
            		return true;
            	else if(vec3i1.z>vec3i2.z)
            		return false;
            	else
            	{
            		if(vec3i1.y<vec3i2.y)
            			return true;
            		else if(vec3i1.y>vec3i2.y)
            			return false;
            		else
            		{
            			if(vec3i1.x<vec3i2.x)
            				return true;
            			 else
            			    return false;
            		}
            	}
            }

            int x;
            int y;
            int z;
    };

}








#endif	/* _VEC3I_H */

