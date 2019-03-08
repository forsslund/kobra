//////////////////////////////////////////////////////////////////////////////
//    Copyright 2008, Forsslund Systems AB
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

#include "AHapticDrillForce.h"
//#include "AHapticPlaybackNode.h"

using namespace FS;

AHapticDrillForce::AHapticDrillForce( const HAPI::Matrix4 & _transform,
	        HAPI::Vec3 *_generated_force,
        bool,
		VolumeModel *_volume_model,
        float*,
		double _parameter_P,
		double _parameter_D,
		double _drillFactor,
                double _bitRadius,
                HapticContext *_context):
			HAPIForceEffect( ),
			transform( _transform),
			generated_force( _generated_force ),
			volume_model( _volume_model),
			parameter_P( _parameter_P),
			parameter_D( _parameter_D),
			drillFactor( _drillFactor),
                        bitRadius( _bitRadius),
                        context( _context){
}

AHapticDrillForce::~AHapticDrillForce()
{
	//Need to delete generated_force which is allocated in ADrillForce::traverseSG
	if( generated_force )
		delete generated_force;
}


HAPI::Vec3 AHapticDrillForce::evaluate_forces(HAPI::Vec3 probe_position)
{
	float intersectVolume=0.001f;
	float R = (float) bitRadius;
	H3DUtil::ArithmeticTypes::Vec3f n_tak;
	Volumes::volumeIntersectionSphere(intersectVolume,n_tak,volume_model,(H3DUtil::ArithmeticTypes::Vec3f)probe_position,R,0.0,0.0);

	HAPI::Vec3 Fe = intersectVolume*(1.0/(R*R*R)) * n_tak;


        if(context->proxyEnabled){
            double sv = 4.0/3.0*3.1415*R*R*R;
            double k = context->springConstant;

            if(intersectVolume > 0.45*sv){
                if(!context->proxyActive)
                    std::cout << "Proxy activated. " << std::endl;
                context->proxyActive = true;
            }

            if(context->proxyActive){
                Fe = k * (context->storedProxyPos - probe_position);
                if( (context->storedProxyPos - probe_position).length() < 0.002 && (intersectVolume < 0.4*sv) ){
                    context->proxyActive = false;

                    std::cout << "Proxy de-activated. " << std::endl;
                }

            } else {
                    HAPI::Vec3 x = Fe/k;
                    context->storedProxyPos = (H3DUtil::ArithmeticTypes::Vec3f)probe_position + x;//n_tak*0.45*R*2.0;//+ x;//*1.1;
            }
        } else {

            // For visuals, routing from stored proxy
            //HAPI::Vec3 x = Fe/2000.0;
            context->storedProxyPos = (H3DUtil::ArithmeticTypes::Vec3f)probe_position; //+ x;//*1.1;
        }

	return Fe;
}


AHapticDrillForce::EffectOutput AHapticDrillForce::calculateForces( const EffectInput &input )
{
	HAPI::Vec3 temp = input.hd->getPosition();
	HAPI::Vec3 F_t = evaluate_forces(transform*(temp)); //probe position in m, model local space

        // Transform back to world space
	HAPI::Vec4 f_output_4 = transform.transformInverse() * HAPI::Vec4(F_t.x, F_t.y, F_t.z, 0);
	HAPI::Vec3 f_output = HAPI::Vec3(f_output_4.x, f_output_4.y, f_output_4.z);

        HAPI::Vec3 p = context->storedProxyPos;

        // This is a good idea, that we move proxy in right direction to
        // allow it to stay on surface and not partly penetrate. However,
        // that needs more thought to work correctly.
        if(!context->proxyActive){
            //p += 0// F_t/context->springConstant;
        }
        else {
            // F_t.normalize();
            //p += F_t * (float) bitRadius*0.85;
        }
        HAPI::Vec4 p_output_4 = transform.transformInverse() * HAPI::Vec4(p.x, p.y, p.z, 0);
        context->proxyGlobal = HAPI::Vec3(p_output_4.x, p_output_4.y, p_output_4.z);


	// scale force
	f_output *= parameter_P;

	// Damping
        f_output += parameter_D*input.hd->getVelocity();

        // Set field
        *generated_force = f_output;

	return EffectOutput(       f_output     );
}


