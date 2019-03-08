//////////////////////////////////////////////////////////////////////////////
//    Copyright 2012 Jonas Forsslund, Sonny Chan
//
//    CDrillForce is free software; you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation; either version 2 of the License, or
//    (at your option) any later version.
//
//    CDrillForce is distributed in the hope that it will be useful,
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

#ifndef _CHapticDrillForce_H
#define	_CHapticDrillForce_H

#include <H3D/X3DChildNode.h>
#include <HAPI/HAPIForceEffect.h>
#include <stdlib.h>
#include <math.h>
#include "Vec3i.h"
#include "VolumeModel.h"
#include "Volumes.h"
#include <H3D/AnyDevice.h>
#include <H3DUtil/H3DMath.h>
#include <stdio.h>
#include <H3D/Group.h>
#include <H3D/Transform.h>
#include "IntervalCollisionDetector.h"
#include <cml/mathlib/typedef.h>
#include "Eigen/Core"


namespace FS
{

    class CHapticContext {
    public:
        float springConstant;

        //H3D::Transform *proxyTransform;
        H3DUtil::ArithmeticTypes::Matrix3d proxyRotation;
        H3DUtil::ArithmeticTypes::Vec3d proxyPosition;

        CHapticContext(){
            std::cout << " creating haptic context" << std::endl;
            debug_buffercount = 0;
            debug_currentScenegraph = 0;
            generated_force = H3DUtil::ArithmeticTypes::Vec3f(0,0,0);
            proxyRotation.setToIdentity();
            proxyPosition = H3DUtil::ArithmeticTypes::Vec3d(10,0,0);
        }

        double debug_elapsedTimes[1000];
        unsigned int debug_scenegraphCount[1000];
        int debug_buffercount;
        unsigned int debug_currentScenegraph;
        H3D::ArithmeticTypes::Vec3f generated_force;

        std::vector<cml::vector3d>  m_pointShell;



    };

/**
 * \brief This is a HAPIForceEffect that lets you feel the object when you drill.
 */
class CHapticDrillForce: public HAPI::HAPIForceEffect
{
public:
    CHapticDrillForce( const HAPI::Matrix4 & _transform,
			VolumeModel *_volume_model,
            double _bitRadius,
            FS::CHapticContext *_context);

    virtual ~CHapticDrillForce();

	/**
	 * The force of the EffectOutput will be the force of the force field.
	 */
	HAPI::HAPIForceEffect::EffectOutput virtual calculateForces( const EffectInput &input );


    class MyCollisionOracle : public CollisionOracle {
    public:
        MyCollisionOracle(FS::VolumeModel *volume_model,
                          HAPI::Rotation r,
                          HAPI::Vec3 posLocalSpace):
            volume_model(volume_model),
            r(r),
            posLocalSpace(posLocalSpace){}

        virtual bool inside(const cml::vector3d &p) const {
            // point p is in  pointshellspace
            // that means I need to know where is my tool in object space,
            // to be able to query volume_model for a point intersection.

            HAPI::Vec3 pp = HAPI::Vec3(p[0],p[1],p[2]);
            pp = r * pp + posLocalSpace;

            return Volumes::pointIntersecting(volume_model,H3DUtil::ArithmeticTypes::Vec3f(pp));
        }
    protected:
        FS::VolumeModel *volume_model;
        HAPI::Rotation r;
        HAPI::Vec3 posLocalSpace;
    };


private:

	// Parameters
	HAPI::Matrix4 transform;
	VolumeModel *volume_model;
    double bitRadius;
    CHapticContext *context;


    HAPI::Vec3 *generated_force;

	int xmax,ymax,zmax;

    void constrainedAcceleration(const cml::vector3d &a, const cml::vector3d &alpha,
                                 cml::vector3d &ac, cml::vector3d &alphac);

    void computeMassMatrix(std::vector<cml::vector3d> *m_pointShell);

    bool resolvePenetrations(HAPI::Vec3 devicePos);


    std::vector<CollisionContact>   m_contacts;
    std::vector<cml::vector3d>      m_contactPoints;
    std::vector<cml::vector3d>      m_contactNormals;


    typedef Eigen::Matrix<double, 6, 6> matrix66;
    typedef Eigen::Matrix<double, 3, 3> matrix33;
    typedef Eigen::Matrix<double, 3, 1> vector3;

    // 6x6 mass matrix and its decomposition (eigen representation)
    // note the inertia part is in world units, and not physical metres
    matrix66                        m_massMatrix;
    matrix66                        m_massMatrixQ;
    matrix66                        m_massMatrixQinv;
};

}

#endif	/* _CHapticDrillForce_H */

