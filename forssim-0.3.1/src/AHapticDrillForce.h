//////////////////////////////////////////////////////////////////////////////
//    Copyright 2008 Forsslund Systems AB
//
//    AHapticDrillForce is free software; you can redistribute it and/or modify
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

#ifndef _AHapticDrillForce_H
#define	_AHapticDrillForce_H

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

namespace FS
{

    class HapticContext {
    public:
        HAPI::Vec3 storedProxyPos;
        bool proxyActive;
        HAPI::Vec3 storedProxyForce;
        bool proxyEnabled;
        float springConstant;

        HAPI::Vec3 proxyGlobal;

        HapticContext(){ proxyActive = false; proxyEnabled=false;}
    };

/**
 * \brief This is a HAPIForceEffect that lets you feel the object when you drill.
 */
class AHapticDrillForce: public HAPI::HAPIForceEffect
{
public:
	/**
	 *  Constructor
	 *  \param transform
	 *  \param force
	 *  \param interpolate
	 *  \param volumeModel
	 *  \param voxels
	 *  \param parameter_P
	 *  \param parameter_I
	 *  \param parameter_D
	 *  \param hapticFactor
	 *  \param voxelSide
	 *  \param drillFactor
	 *  \param bitRadius
	 *  \param iso
	 *  \param isoSigma
	 *  \param isoConstant
	 */
	AHapticDrillForce( const HAPI::Matrix4 & _transform,
			HAPI::Vec3 *_generated_force,
			bool _interpolate,
			VolumeModel *_volume_model,
			float* _voxels,
			double _parameter_P,
			double _parameter_D,
			double _drillFactor,
                        double _bitRadius,
                        FS::HapticContext *_context);

	virtual ~AHapticDrillForce();

	/**
	 * The force of the EffectOutput will be the force of the force field.
	 */
	HAPI::HAPIForceEffect::EffectOutput virtual calculateForces( const EffectInput &input );

private:
	HAPI::Vec3 evaluate_forces(HAPI::Vec3 probe_position);

	HAPI::Matrix4 transform;
	HAPI::Vec3 *generated_force;
	VolumeModel *volume_model;

    // Parameters
    double parameter_P;
    double parameter_D;
    double drillFactor;
    double bitRadius;

        HapticContext *context;


    HAPI::Vec3 F_t_minus_1;
    HAPI::Vec3 F_t_minus_2;
    HAPI::Vec3 Fe_t_minus_1;

	float *voxels;

	int xmax,ymax,zmax;

};

}

#endif	/* _AHapticDrillForce_H */

