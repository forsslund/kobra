
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
#include "ADrillForce.h"

using namespace FS;

// Add this node to the H3DNodeDatabase system.
H3D::H3DNodeDatabase ADrillForce::database( "ADrillForce",
		&(newInstance<ADrillForce>),
		typeid( ADrillForce ),
		&H3DForceEffect::database );

namespace ADrillForceInternals
{
H3D::FIELDDB_ELEMENT( ADrillForce, force, INPUT_OUTPUT );
H3D::FIELDDB_ELEMENT( ADrillForce, PID,  INPUT_OUTPUT );
H3D::FIELDDB_ELEMENT( ADrillForce, drillFactor,  INPUT_OUTPUT );
H3D::FIELDDB_ELEMENT( ADrillForce, bitRadius,  INPUT_OUTPUT );
H3D::FIELDDB_ELEMENT( ADrillForce, volumeModel, INPUT_OUTPUT );
H3D::FIELDDB_ELEMENT( ADrillForce, enableProxy, INPUT_OUTPUT );
H3D::FIELDDB_ELEMENT( ADrillForce, springConstant, INPUT_OUTPUT );
H3D::FIELDDB_ELEMENT( ADrillForce, proxyPosition, INPUT_OUTPUT );
}


ADrillForce::ADrillForce( H3D::Inst< H3D::SFVec3f > _force,
		H3D::Inst< H3D::SFVec3f > _PID,
		H3D::Inst< H3D::SFFloat > _drillFactor,
		H3D::Inst< H3D::SFFloat > _bitRadius,
		H3D::Inst< H3D::SFNode  > _volumeModel,
                H3D::Inst< H3D::SFBool  > _enableProxy,
                H3D::Inst< H3D::SFFloat  > _springConstant,
                H3D::Inst< H3D::SFVec3f  > _proxyPosition,
                H3D::Inst< H3D::SFNode>  _metadata ) :
			H3D::H3DForceEffect( _metadata ),
			force( _force ),
			PID( _PID ),
			drillFactor( _drillFactor ),
			bitRadius( _bitRadius ),
                        volumeModel( _volumeModel ),
                        enableProxy( _enableProxy ),
                        proxyPosition( _proxyPosition ),
                        springConstant( _springConstant ) {

	type_name = "ADrillForce";
	database.initFields( this );

	/* Default values */
	force->setValue( H3D::Vec3f( 0, 0, 0 ) );
	PID->setValue(H3D::Vec3f(0.8f, 0, 0));
	drillFactor->setValue(0.0001f);
    bitRadius->setValue(0.0003f);
        enableProxy->setValue(false);
        springConstant->setValue(1000.0);
}

void ADrillForce::initialize()
{
	cout<<"Initializing ADrillForce Node"<<endl;
	H3DForceEffect::initialize();

    context = new HapticContext();
}

void ADrillForce::traverseSG( H3D::TraverseInfo &ti )
{
	volume_model = static_cast< VolumeModel * >( volumeModel->getValue() );

        HAPI::Vec3 *generated_force = new HAPI::Vec3();
        context->proxyEnabled = enableProxy->getValue();
        context->springConstant = springConstant->getValue();

       // proxyPosition->setValue((H3DUtil::ArithmeticTypes::Vec3f)context->storedProxyPos);

	if(ti.hapticsEnabled())
	{
		ti.addForceEffectToAll( new AHapticDrillForce(
				ti.getAccInverseMatrix(),
				generated_force,
                                true,
				volume_model,
				voxels,
				PID->getValue().x,
				PID->getValue().z,
				drillFactor->getValue(),
                                bitRadius->getValue(),
                                context) );
	}

        // Update field
        force->setValue(H3D::Vec3f(*generated_force));
        H3D::Vec3f offset;
        offset.x = ti.getAccForwardMatrix().getElement(0,3);
        offset.y = ti.getAccForwardMatrix().getElement(1,3);
        offset.z = ti.getAccForwardMatrix().getElement(2,3);
        proxyPosition->setValue(H3D::Vec3f(context->proxyGlobal)+offset);
}
