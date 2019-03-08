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

#ifndef _ADrillForce_H
#define	_ADrillForce_H
#include <H3D/H3DForceEffect.h>
#include <H3D/SFVec3f.h>
#include "AHapticDrillForce.h"
#include <H3D/Node.h>


namespace FS
{

/**
 * \brief A node that enables drill force
 *
 * Adds a AHapticDrillForce effect to the TraverseInfo.
 */
class ADrillForce: public H3D::H3DForceEffect
{

public:

	/// The SFTextureNode field is dependent on the displayList field
	/// of the containing X3DTextureNode node.
	typedef H3D::DependentSFNode< H3D::X3DTexture3DNode,
	H3D::FieldRef< H3D::H3DDisplayListObject,
	H3D::H3DDisplayListObject::DisplayList,
	&H3D::H3DDisplayListObject::displayList >,
	true >// VolumeData::SFTexture3DNode
	SFTexture3DNode;

	/**
	 *  Constructor
	 *  \param force The force it generates
	 *  \param PID
	 *  \param drillFactor
	 *  \param bitRadius
	 *  \param volumeModel
	 *  \param metadata
	 */
	ADrillForce( H3D::Inst< H3D::SFVec3f > _force = 0,
			H3D::Inst< H3D::SFVec3f > _PID = 0,
			H3D::Inst< H3D::SFFloat > _drillFactor = 0,
			H3D::Inst< H3D::SFFloat > _bitRadius = 0,
			H3D::Inst< H3D::SFNode  > _volumeModel = 0,
                        H3D::Inst< H3D::SFBool  > _enableProxy = 0,
                        H3D::Inst< H3D::SFFloat > _springConstant = 0,
                        H3D::Inst< H3D::SFVec3f > _proxyPosition = 0,
                        H3D::Inst< H3D::SFNode  > _metadata = 0);

	/**
	 *  ADrillForce Node initialization
	 */
	virtual void initialize();

	/**
	 * Traverse the Scene Graph
	 *
	 * Adds a AHapticDrillForce effect to the TraverseInfo.
	 */
	virtual void traverseSG( H3D::TraverseInfo &ti );

	auto_ptr< H3D::SFVec3f > force;
	auto_ptr< H3D::SFVec3f > PID;
        auto_ptr< H3D::SFFloat > drillFactor;
        auto_ptr< H3D::SFFloat > bitRadius;
    auto_ptr< H3D::SFNode > volumeModel;
        auto_ptr< H3D::SFBool > enableProxy;
        auto_ptr< H3D::SFVec3f > proxyPosition;
        auto_ptr< H3D::SFFloat > springConstant;

	float *voxels;
	VolumeModel *volume_model;

	/// The H3DNodeDatabase for this node.
	static H3D::H3DNodeDatabase database;

        HapticContext *context;

};
}




#endif	/* _ADrillForce_H */

