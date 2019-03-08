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
/// \file SoftBodyProximityAttachment.cpp
/// \brief Source file for SoftBodyProximityAttachment, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////

#include <H3D/H3DPhysics/SoftBodyProximityAttachment.h>
#include <H3D/H3DPhysics/SoftBodyPhysicsEngineThread.h>

using namespace H3D;  

H3DNodeDatabase SoftBodyProximityAttachment::database( "SoftBodyProximityAttachment", 
                                              &(newInstance< SoftBodyProximityAttachment >), 
                                              typeid( SoftBodyProximityAttachment ),
                                              &SoftBodyAttachment::database);

namespace SoftBodyAttachmentInternals {
  FIELDDB_ELEMENT( SoftBodyProximityAttachment, surfaceProximity, INPUT_OUTPUT )
  FIELDDB_ELEMENT( SoftBodyProximityAttachment, surfaceSoftBodyProximity, INPUT_OUTPUT )
  FIELDDB_ELEMENT( SoftBodyProximityAttachment, softBodyProximity, INPUT_OUTPUT )
  FIELDDB_ELEMENT( SoftBodyProximityAttachment, surfaceGeometry1, INPUT_OUTPUT )
  FIELDDB_ELEMENT( SoftBodyProximityAttachment, surfaceGeometry2, INPUT_OUTPUT )
}

SoftBodyProximityAttachment::SoftBodyProximityAttachment (
                                      Inst< SFNode > _metadata,
                                      Inst< ValueUpdater > _valueUpdater,
                                      Inst< SFH3DSoftBodyNode > _body1,
                                      Inst< MFString     > _forceOutput,
                                      Inst< MFEngineOptions > _engineOptions,
                                      Inst< SFH3DSoftBodyNode > _body2,
                                      Inst< TrackedMFInt32 > _index,
                                      Inst< TrackedMFInt32 > _index2,
                                      Inst< SFFloat > _surfaceProximity,
                                      Inst< SFFloat > _surfaceSoftBodyProximity,
                                      Inst< SFFloat > _softBodyProximity,
                                      Inst< SFX3DComposedGeometry > _surfaceGeometry1,
                                      Inst< SFX3DComposedGeometry > _surfaceGeometry2
                                     )
  : SoftBodyAttachment ( _metadata, _valueUpdater, _body1, _forceOutput, _engineOptions, _body2, _index, _index2 ),
    surfaceProximity ( _surfaceProximity ),
    surfaceSoftBodyProximity ( _surfaceSoftBodyProximity ),
    softBodyProximity ( _softBodyProximity ),
    surfaceGeometry1 ( _surfaceGeometry1 ),
    surfaceGeometry2 ( _surfaceGeometry2 ) {
  // init fields
  type_name = "SoftBodyProximityAttachment";
  database.initFields( this );

  surfaceProximity->setValue ( 0.001f );
  surfaceSoftBodyProximity->setValue ( 0.001f );
  softBodyProximity->setValue ( 0.001f );
}

bool SoftBodyProximityAttachment::initializeConstraint( PhysicsEngineThread& thread ) {
  generateConnectedIndices();
  return SoftBodyAttachment::initializeConstraint ( thread );
}

void SoftBodyProximityAttachment::generateConnectedIndices () {
  // for all surface points in body1, measure distance to closest surface point on body2
  // * if it is below a threshold, add it to a list of close surface vertices
  //
  // For all points in body1
  //  if point is close to a 'close surface vertex'
  //   link the point with the nearest point in body2

  // Lists of indices to attach from body 1 and 2
  vector<H3DInt32> indices1;
  vector<H3DInt32> indices2;

  H3DSoftBodyNode* sb1= static_cast<H3DSoftBodyNode*>(body1->getValue());
  H3DSoftBodyNode* sb2= static_cast<H3DSoftBodyNode*>(body2->getValue());

  if ( sb1 && sb2 ) {
    X3DComposedGeometryNode* g1= dynamic_cast<X3DComposedGeometryNode*>(sb1->geometry->getValue());
    NodeVector surfaceGeometries1;
    if ( !surfaceGeometry1->empty() ) {
      surfaceGeometries1= surfaceGeometry1->getValue();
    } else {
      surfaceGeometries1= sb1->surfaceGeometry->getValue();
    }
    if ( surfaceGeometries1.empty() && g1 ) {
      surfaceGeometries1.push_back ( g1 );
    }

    X3DComposedGeometryNode* g2= dynamic_cast<X3DComposedGeometryNode*>(sb2->geometry->getValue());
    NodeVector surfaceGeometries2;
    if ( !surfaceGeometry2->empty() ) {
      surfaceGeometries2= surfaceGeometry2->getValue();
    } else {
      surfaceGeometries2= sb2->surfaceGeometry->getValue();
    }
    if ( surfaceGeometries2.empty() && g2 ) {
      surfaceGeometries2.push_back ( g2 );
    }

    if ( !surfaceGeometries1.empty() && !surfaceGeometries2.empty() && g1 && g2) {

      // Get a list of points that are close to both the surfaces of body 1 and 2
      H3DFloat surface_proximity= surfaceProximity->getValue();
      vector<Vec3f> closeSurfacePoints;
      for ( NodeVector::const_iterator i= surfaceGeometries1.begin(); i != surfaceGeometries1.end(); ++i ) {
        X3DComposedGeometryNode* g= dynamic_cast<X3DComposedGeometryNode*>(*i);
        if ( g ) {
          X3DCoordinateNode* coord= g->coord->getValue();
          if ( coord ) {
            for ( X3DCoordinateNode::Iterator j= coord->pointBegin(); j != coord->pointEnd(); ++j ) {
              Vec3f p= *j;

              for ( NodeVector::const_iterator k= surfaceGeometries2.begin(); k != surfaceGeometries2.end(); ++k ) {
                X3DComposedGeometryNode* gtmp= dynamic_cast<X3DComposedGeometryNode*>(*k);
                if( gtmp ) {
                  X3DCoordinateNode* coord1= gtmp->coord->getValue();
                  if ( coord1 ) {
                    for ( X3DCoordinateNode::Iterator l= coord->pointBegin(); l != coord->pointEnd(); ++l ) {
                      Vec3f p1= *l;
                      if ( (p1-p).length() < surface_proximity ) {
                        closeSurfacePoints.push_back ( (p+p1)/2 );
                      }
                    }
                  }
                }
              }

            }
          }
        }
      }

      H3DFloat surface_soft_body_proximity= surfaceSoftBodyProximity->getValue();
      H3DFloat soft_body_proximity= softBodyProximity->getValue();

      X3DCoordinateNode* coord1= g1->coord->getValue();
      if ( coord1 ) {
        H3DInt32 index1= 0;
        for ( X3DCoordinateNode::Iterator i= coord1->pointBegin(); i != coord1->pointEnd(); ++i, ++index1 ) {
          Vec3f p= *i;

          bool close= false;
          for ( vector<Vec3f>::iterator j= closeSurfacePoints.begin(); j != closeSurfacePoints.end(); ++j ) {
            if ( (p-*j).length() < surface_soft_body_proximity ) {
              close= true;
              break;
            }
          }

          if ( close ) {
            X3DCoordinateNode* coord2= g2->coord->getValue();
            if ( coord2 ) {
              H3DInt32 _index2= 0;
              for ( X3DCoordinateNode::Iterator j= coord2->pointBegin(); j != coord2->pointEnd(); ++j, ++_index2 ) {
                Vec3f p1= *j;
                if ( (p-p1).length() < soft_body_proximity ) {
                  indices1.push_back ( index1 );
                  indices2.push_back ( _index2 );
                }
              }
            }
          }
        }
      }
    }
  }

  index->setValue ( indices1 );
  index2->setValue ( indices2 );
}
