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

#include "Volumes.h"

using namespace FS;

int Volumes::volumeIntersectionSphere(float &intersectionVolume,
        H3DUtil::ArithmeticTypes::Vec3f &direction,
        VolumeModel *volume_model,
        H3DUtil::ArithmeticTypes::Vec3f center,
        float radius,
        float iso,
        float){

    if(!volume_model){
        intersectionVolume=0;
        direction=H3D::Vec3f(0,0,0);
        return 0;
    }

    float voxelSide = volume_model->getSpacing(); //Assume cubic voxels! TODO: maybe not assume?

    double eta = pow((3.0f/4.0f*voxelSide*voxelSide*voxelSide)/H3DUtil::Constants::pi, (double)(1.0f/3.0f));//0.186105147269820f; // 4/3*PI*eta^3=voxelsize^3 in mm
    double R=radius;

    Vec3i base_voxel = volume_model->getIndexLocalWall(H3D::Vec3f(center.x-(float)R, center.y-(float)R, center.z-(float)R));
    Vec3i top_voxel = volume_model->getIndexLocalWall(H3D::Vec3f(center.x+(float)R, center.y+(float)R, center.z+(float)R));
    top_voxel.x++; // Because getIndexLocalWall() returns floored values
    top_voxel.y++;
    top_voxel.z++;
    top_voxel=volume_model->wall(top_voxel);


	//cout << "top-bottom: " << top_voxel.x-base_voxel.x << "\n";

    /*
     * Step through voxels
     */
    H3D::Vec3f c; // We are approximating the voxels with spheres of the same volume,
    // centered at the voxel center c, with the origin at the center of B
    volume_model->setRelativeCoordinate(center);

    double m0 = 0;

    H3D::Vec3f m1 = H3D::Vec3f(0, 0, 0);

    double sphere_voxel_volume = (4.0/3.0)*H3DUtil::Constants::pi*eta*eta*eta;

    double my = iso;

    Vec3i v;
    for(v.z=base_voxel.z;v.z<=top_voxel.z;v.z++){
        for(v.y=base_voxel.y;v.y<=top_voxel.y;v.y++){
            for(v.x=base_voxel.x;v.x<=top_voxel.x;v.x++){
                float x = volume_model->getMaterialRemaining(v);

                double vo;
                if(x>my)
                    vo = 1;
				else
					vo = 0;

                c = volume_model->getRelativeCoordinate(v);
                double d = c.x * c.x + c.y*c.y + c.z*c.z;

                /*
                 *  If voxel sphere is outside bit sphere, don't cosider
                 */
                if( d > (eta+R)*(eta+R) ){
                    //nop
                }

                /*
                 * If voxel sphere completely inside, add whole
                 */
                else if( d < (R-eta)*(R-eta) ){
                    m0 += vo*sphere_voxel_volume;
                    m1 += vo*c*sphere_voxel_volume;
                }


                /*
                 * If voxel sphere is partly inside
                 */
                else {
                    double d = c.length();
                    double delta_v = H3DUtil::Constants::pi/12*(d*d*d - 6*(R*R+eta*eta)*d + 8*(R*R*R+eta*eta*eta) -3*(eta*eta-R*R)*(eta*eta-R*R)*1/d);
                    double r = (1/2)*d + (R*R-eta*eta)/(2*d);

                    m0 += vo * delta_v * sphere_voxel_volume;
                    m1 += vo * delta_v * sphere_voxel_volume * r/d * c;
                }
            }
        }
    }


    intersectionVolume = (float)m0;

    H3D::Vec3f n_hat;
    H3D::H3DFloat length = m1.length();
    if(length != 0 )
        n_hat = -m1/length;
    else
        n_hat = H3D::Vec3f(0,0,0);
    direction=n_hat;

    return 1;
}

bool Volumes::pointIntersecting(VolumeModel *volume_model, ArithmeticTypes::Vec3f center)
{
    if(!volume_model){
        std::cout << "null pointer exception in pointIntersecting()" << std::endl;
        return false;
    }

    Vec3i voxel = volume_model->getIndexLocal(center);
    if(voxel.x < 0)
        return false;
    if(volume_model->getMaterialRemaining(voxel) > 0){
        return true;
    }
    return false;
}

double Volumes::heightOfSphereSegment(double volume, double radius) {
    //binary search
    double h0, h1, h;

    h = 0;
    h0=0;
    h1=radius*2;

    while(h1-h0>0.000000001){
        h=(h0+h1)/2;
        if(H3DUtil::Constants::pi*h*h*(radius-h/3) > volume)
            h1=h;
        else
            h0=h;
    }

    return h;
}


