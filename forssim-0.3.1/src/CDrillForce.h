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

#ifndef _CDrillForce_H
#define	_CDrillForce_H
#include <H3D/H3DForceEffect.h>
#include <H3D/SFVec3f.h>
#include "CHapticDrillForce.h"
#include <H3D/Node.h>


namespace FS
{

/**
 * \brief A node that enables drill force
 *
 * Adds a CHapticDrillForce effect to the TraverseInfo.
 */
class CDrillForce: public H3D::H3DForceEffect
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
	 *  \param bitRadius
	 *  \param volumeModel
	 *  \param metadata
	 */
    CDrillForce( H3D::Inst< H3D::SFVec3f > _force = 0,
			H3D::Inst< H3D::SFFloat > _bitRadius = 0,
			H3D::Inst< H3D::SFNode  > _volumeModel = 0,
            H3D::Inst< H3D::SFFloat > _springConstant = 0,
            H3D::Inst< H3D::SFFloat > _offsetFactor = 0,
            H3D::Inst< H3D::SFNode > _proxyTransform = 0,
            H3D::Inst< H3D::SFVec3f > _cutPosition = 0,
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
	auto_ptr< H3D::SFFloat > bitRadius;
	auto_ptr< H3D::SFNode > volumeModel;
    auto_ptr< H3D::SFNode > proxyTransform;
    auto_ptr< H3D::SFFloat > springConstant;
    auto_ptr< H3D::SFFloat > offsetFactor;
    auto_ptr< H3D::SFVec3f > cutPosition;

	VolumeModel *volume_model;

	/// The H3DNodeDatabase for this node.
	static H3D::H3DNodeDatabase database;

    CHapticContext *context;

private:

    /**
     * Updates the context to reflect the latest burr radius and spring constant
     */
    void updateHapticContext();
    // These variables keep track of the previous value of bit radius and spring constant so
    // we can update the haptic context only when they change
	float lastBitRadius;
    float lastSpringConstant;
};
}




#endif	/* _CDrillForce_H */

