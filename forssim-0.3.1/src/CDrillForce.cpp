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

#include "CDrillForce.h"
#include <H3D/Shape.h>
#include <H3D/Sphere.h>
#include <H3D/Group.h>
#include <H3D/X3D.h>
#include <H3D/Transform.h>
#include <H3D/MFNode.h>

using namespace FS;

// Add this node to the H3DNodeDatabase system.
H3D::H3DNodeDatabase CDrillForce::database( "CDrillForce",
        &(newInstance<CDrillForce>),
        typeid( CDrillForce ),
		&H3DForceEffect::database );

namespace CDrillForceInternals
{
    H3D::FIELDDB_ELEMENT( CDrillForce, force, INPUT_OUTPUT );
    H3D::FIELDDB_ELEMENT( CDrillForce, bitRadius,  INPUT_OUTPUT );
    H3D::FIELDDB_ELEMENT( CDrillForce, volumeModel, INPUT_OUTPUT );
    H3D::FIELDDB_ELEMENT( CDrillForce, springConstant, INPUT_OUTPUT );
    H3D::FIELDDB_ELEMENT( CDrillForce, proxyTransform, INPUT_OUTPUT );
    H3D::FIELDDB_ELEMENT( CDrillForce, cutPosition, INPUT_OUTPUT );
    H3D::FIELDDB_ELEMENT( CDrillForce, offsetFactor, INPUT_OUTPUT );
}


CDrillForce::CDrillForce( H3D::Inst< H3D::SFVec3f > _force,
        H3D::Inst< H3D::SFFloat > _bitRadius,
        H3D::Inst< H3D::SFNode  > _volumeModel,
        H3D::Inst< H3D::SFFloat > _springConstant,
        H3D::Inst< H3D::SFFloat > _offsetFactor,
        H3D::Inst< H3D::SFNode  > _proxyTransform,
        H3D::Inst< H3D::SFVec3f  > _cutPosition,
        H3D::Inst< H3D::SFNode  >  _metadata ) :
        H3D::H3DForceEffect( _metadata ),
			force( _force ),
			bitRadius( _bitRadius ),
            volumeModel( _volumeModel ),
            proxyTransform( _proxyTransform ),
            springConstant( _springConstant ),
            offsetFactor( _offsetFactor ),
            cutPosition(_cutPosition){

    type_name = "CDrillForce";
	database.initFields( this );

	/* Default values */
	force->setValue( H3D::Vec3f( 0, 0, 0 ) );
    bitRadius->setValue(0.0015f);
    springConstant->setValue(700.0);
    cutPosition->setValue(H3D::Vec3f( 0, 0, 0 ));
    offsetFactor->setValue(0.75);

    lastBitRadius = 0.0;
    lastSpringConstant = 0;
}

void CDrillForce::initialize()
{
    cout<<"Initializing CDrillForce Node"<<endl;
	H3DForceEffect::initialize();

    // Make some spheres and put them in debuggroup. The position of these
    // speheres will be used for collision detection. The visibility of
    // these spheres is only for debug purposes hence the name debuggroup.
    // TODO: They should be converted into a private list of points instead
    context = new CHapticContext();
    //context->proxyTransform = static_cast< Transform * >(proxyTransform->getValue());

    //CHapticDrillForce::computeMassMatrix( &(context->m_pointShell) );

    updateHapticContext(); // the code for creating the point shell has been moved to this function

    // Add these spheres to our pointShell in the context struct.
    // =========================================================================
    // Create some pointshellzz
    // Access the debugsphere childs
    // =========================================================================
    //context->m_pointShell.clear();
    /*
    NodeVector theChildren = context->proxyTransform->children->getValue();

    for( MFNode::const_iterator i = theChildren.begin();
         i != theChildren.end(); i++ ) {

        // We have a group arround a transform since X3D::Create... makes a group
        Group *g = dynamic_cast< Group * >( (*i) );
        if(!g)
            continue;

        Transform *t = dynamic_cast< Transform * >( g->children->getValue()[0] );
        if(t){
            HAPI::Vec3 p = t->translation->getValue();

            // add to our list
            cml::vector3d pp(p.x,p.y,p.z);
            context->m_pointShell.push_back(pp);
        }
    }
    */
    // =========================================================================

    std::cout << "Initalized CDRillForce fine. " << std::endl;

}


void CDrillForce::updateHapticContext()
{
	//update spring constant if necessary
	if (springConstant->getValue() != lastSpringConstant)
	{
		context->springConstant = springConstant->getValue();
		lastSpringConstant = springConstant->getValue();
		cout << "HapticContext spring constant updated to value: " << springConstant->getValue() << endl;
	}

	//update point shell if necessary
	if (bitRadius->getValue() != lastBitRadius)
	{
		//Empty point shell
		context->m_pointShell.clear();

	    double radius = bitRadius->getValue();

	    int generated = 0;
	    while(generated < 100){
            double x = rand()/double(RAND_MAX) * 2*radius - radius;
            double y = rand()/double(RAND_MAX) * 2*radius - radius;
            double z = rand()/double(RAND_MAX) * 2*radius - radius;

	        if(x*x+y*y+z*z > radius*radius)
	            continue;

	        // project on surface
	        HAPI::Vec3 p = HAPI::Vec3(x,y,z);
	        if(p.lengthSqr() == 0)
	            continue;
	        p.normalize();
	        p = p * radius;

	        cml::vector3d pp(p.x,p.y,p.z);
            context->m_pointShell.push_back(pp);

	        /*
	        std::stringstream s;
	        s << "<Transform translation='" << p.x << " " << p.y << " " << p.z << "'>";
	      //s << "<Shape><Sphere radius='0.0001'/><Appearance><Material diffuseColor='0 1 0'/></Appearance></Shape>";
	        s << "</Transform>";
	        */
	        //context->proxyTransform->children->push_back(H3D::X3D::createX3DFromString(s.str()));

	        generated++;
	    }
		lastBitRadius=bitRadius->getValue();
		cout << "HapticContext point shell updated with radius: " << bitRadius->getValue() << endl;
	}
}


void CDrillForce::traverseSG( H3D::TraverseInfo &ti )
{
	updateHapticContext();

	volume_model = static_cast< VolumeModel * >( volumeModel->getValue() );

    context->debug_currentScenegraph++;

	if(ti.hapticsEnabled())
	{
        ti.addForceEffectToAll( new CHapticDrillForce(
				ti.getAccInverseMatrix(),
				volume_model,
                bitRadius->getValue(),
                context) );
	}

    // Update field
    force->setValue(H3D::Vec3f(context->generated_force));

    H3D::Transform *t = static_cast< Transform * >(proxyTransform->getValue());
    t->translation->setValue(
                H3DUtil::ArithmeticTypes::Vec3f(context->proxyPosition));
    t->rotation->setValue(
                H3DUtil::ArithmeticTypes::Rotation(context->proxyRotation));

    H3D::Vec3f forceNormal = H3D::Vec3f(context->generated_force);
    forceNormal.normalizeSafe();

    //H3D::Vec3f cpos = context->proxyTransform->translation->getValue()
    //     -1* H3D::Vec3f(*generated_force) / springConstant->getValue();

    H3D::Vec3d cpos = context->proxyPosition //context->proxyTransform->translation->getValue()
            -1*offsetFactor->getValue() * forceNormal * bitRadius->getValue();

    //std::cout << 2* H3D::Vec3f(*generated_force) / springConstant->getValue() << std::endl;
    cutPosition->setValue(H3D::Vec3f(cpos));
}
