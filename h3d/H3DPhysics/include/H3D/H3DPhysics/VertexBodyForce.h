//////////////////////////////////////////////////////////////////////////////
//    Copyright 2004-2019, SenseGraphics AB
//
//    This file is part of H3D API.
//
//    H3D API is free software; you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation; either version 2 of the License, or
//    (at your option) any later version.
//
//    H3D API is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with H3D API; if not, write to the Free Software
//    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//
//    A commercial license is also available. Please contact us at 
//    www.sensegraphics.com for more information.
//
//
/// \file VertexBodyForce.h
/// \brief Header file for VertexBodyForce, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __VERTEXBODYFORCE__
#define __VERTEXBODYFORCE__

#include <H3D/MFVec3f.h>
#include <H3D/MFInt32.h>
#include <H3D/X3DNode.h>
#include <H3D/H3DPhysics/SoftBodyParameters.h>
#include <H3D/H3DPhysics/H3DVertexBodyModifierNode.h>

namespace H3D{  

  /// \ingroup SoftBody
  /// A VertexBodyForce node can be used to apply user defined forces to
  /// specific vertices of a soft body.
  ///
  /// The index and forces fields should be of equal length
  /// and for each pair of values, i, forces[i] will be applied to vertex index[i].
  ///
  /// <b>Examples:</b>
  ///   - <a href="../../examples/softbody/VertexForce.x3d">VertexForce.x3d</a>
  ///     ( <a href="examples/VertexForce.x3d.html">Source</a> )
  ///
  /// \par Internal routes:
  /// \dotfile VertexBodyForce.dot
  class H3DPHYS_API VertexBodyForce : public H3DVertexBodyModifierNode {
  public:

    /// Constructor.
    VertexBodyForce(
      Inst< SFNode > _metadata = 0,
      Inst< ValueUpdater > _valueUpdater = 0,
      Inst< SFH3DBodyNode > _body1 = 0,
      Inst< MFEngineOptions > _engineOptions = 0,
      Inst< MFInt32 > _index = 0,
      Inst< MFVec3f > _forces = 0 );


    /// Forces to apply to vertices. One value per vertex.
    ///
    /// The index and forces fields should be of equal length
    /// and for each pair of values, i, forces[i] will be applied to vertex index[i].
    ///
    /// <b>Access type:</b> inputOutput \n
    /// <b>Default value:</b> [] \n
    /// 
    /// \dotfile VertexBodyForce_forces.dot
    auto_ptr < MFVec3f > forces;

    /// A field used to collect events that require the user defined
    /// per vertex forces to be re-applied in the simulation thread
    ///
    /// \dotfile VertexBodyForce_forcesChanged.dot
    auto_ptr < Field > forcesChanged;

    /// Traverse the scene graph
    virtual void traverseSG ( TraverseInfo& ti );

    /// Calculate forces for deformation
    virtual void calculateForces ( PhysicsEngineParameters::H3DSoftBodyNodeParameters& softBodyParams,
      HAPI::HAPIHapticsDevice& hd );

    /// Accumulate all user defined vertex forces and add to the specified H3DSoftBodyNodeParameters
    void addVertexForces ( PhysicsEngineParameters::H3DSoftBodyNodeParameters& params );

    /// The H3DNodeDatabase for this node.
    static H3DNodeDatabase database;

  protected:

    /// Local parameter of index for thread safety.
    std::vector<H3DInt32> localIndex;

    /// Local parameter of forces for thread safety.
    std::vector<Vec3f> localForces;

    /// Local flag used to indicate that forces should be updated
    bool localUpdateForces;

  };
}
#endif
