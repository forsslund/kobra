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
/// \file CollidableShape.cpp
/// \brief Source file for CollidableShape, X3D scene-graph node
///
//
//////////////////////////////////////////////////////////////////////////////
#include <H3D/H3DPhysics/CollidableShape.h>
#include <H3D/H3DPhysics/CollisionSpace.h>
#include <H3D/H3DPhysics/PhysX3CollidableOptions.h>
#include <H3D/H3DPhysics/RigidBodyCollection.h>

using namespace H3D;

H3DNodeDatabase CollidableShape::database( "CollidableShape", 
                                          &(newInstance< CollidableShape >), 
                                          typeid( CollidableShape ),
                                          &X3DNBodyCollidableNode::database);

namespace CollidableShapeInternals {
  FIELDDB_ELEMENT( CollidableShape, shape, INPUT_OUTPUT )
  FIELDDB_ELEMENT( CollidableShape, selfCollide, INPUT_OUTPUT )
  FIELDDB_ELEMENT( CollidableShape, clipPlanes, INPUT_OUTPUT )
  FIELDDB_ELEMENT( CollidableShape, updateShapeBounds, INPUT_OUTPUT )
}

CollidableShape::CollidableShape(Inst< SFBool > _enabled,
                                 Inst< SFNode  > _metadata, 
                                 Inst< SFRotation > _rotation,
                                 Inst< SFVec3f > _translation,
                                 Inst< SFVec3f > _scale,
                                 Inst< SFBound > _bound,
                                 Inst< SFVec3f > _bboxCenter,
                                 Inst< SFVec3f  > _bboxSize,
                                 Inst< ValueUpdater > _valueUpdater,
                                 Inst< MFEngineOptions > _engineOptions,
                                 Inst< SFShapeNode > _shape,
                                 Inst< SFBool      > _selfCollide,
                                 Inst< MFClipPlaneNode > _clipPlanes,
                                 Inst< SFBool > _updateShapeBounds):
X3DNBodyCollidableNode( _enabled, _metadata, _rotation, 
                       _translation, _scale, _bound, _bboxCenter, 
                       _bboxSize, _valueUpdater, _engineOptions ),
                       shape( _shape ),
                       selfCollide( _selfCollide ),
                       clipPlanes( _clipPlanes ),
                       transRotMatrix( new TransformMatrix ),
                       updateShapeBounds( _updateShapeBounds )
{

  type_name = "CollidableShape";
  database.initFields( this );

  // Set names needed by doxygen to not have "unknown" fields in dot files.
  transRotMatrix->setName( "transRotMatrix" );
  // Set owner of fields.
  bound->setOwner( this );
  transRotMatrix->setOwner( this );

  // Set up routes.
  translation->route( transRotMatrix, id );
  rotation->route( transRotMatrix, id );
  scale->route( transRotMatrix, id );
  transRotMatrix->route( bound, id );

  selfCollide->setValue( false );
  
  selfCollide->route( valueUpdater );
  clipPlanes->route( valueUpdater );
  shape->route( valueUpdater );

  debug_collidable_inline.reset( new Inline );

  debug_collidable_inline->displayList->route( displayList );
  shape->route( displayList );

  updateShapeBounds->setValue( true );
  updateShapeBounds->route( valueUpdater );

}

CollidableShape::~CollidableShape() {
  if ( collidableInitialized() )
    deleteCollidable();
  // Set owner to 0 in order to use the check in the onRemove function.
  // Could probably reset the bound and transRotMatrix fields instead but
  // I have no idea if that would screw something else up.
  bound->setOwner( 0 );
  transRotMatrix->setOwner( 0 );
}

void CollidableShape::render() {
  X3DShapeNode *s = dynamic_cast< X3DShapeNode * >(shape->getValue());
  if( s ) {
    PhysX3CollidableOptions *options = NULL;
    for( MFEngineOptions::const_iterator i = engineOptions->begin(); i != engineOptions->end() && options == NULL; i++ ) {
      options = dynamic_cast< PhysX3CollidableOptions * >(*i );
    }

    glMatrixMode( GL_MODELVIEW );
    glPushMatrix();
    const Vec3f &sc = scale->getValue();
    const Rotation &orn = rotation->getValue();
    const Vec3f &pos = translation->getValue();  

    glTranslatef( pos.x, pos.y, pos.z );  
    glRotatef( (GLfloat)(orn.angle*180/H3DUtil::Constants::pi), orn.axis.x, orn.axis.y, orn.axis.z );
    glScalef( sc.x, sc.y, sc.z );

    if( options && !options->saveConvexDecomposition->getValue().empty() ) {
       const string &url = options->saveConvexDecomposition->getValue();
       debug_collidable_inline->url->clear();
       debug_collidable_inline->url->push_back( url );
       debug_collidable_inline->displayList->callList();
    } else {
      s->displayList->callList();
    }
    glMatrixMode( GL_MODELVIEW );
    glPopMatrix();
  }
}


// Note: initializeCollidable() is called by the parent node, never from the
// class itself. CollidableShapes are initialized in the physics
// engine only when they are attached to a CollisionCollection node.
// CollidableShape nodes in the scene graph that are not a part of
// any CollisionCollection will not exist in the physics engine.
int CollidableShape::initializeCollidable( H3D::PhysicsEngineThread *pt, X3DNode *parent ) {
  if( collidableInitialized() ) return 1;

  // If shape field value is NULL, do not create a collidable in
  // the physics engine at all, since it is meaningless.
  if ( shape->getValue() == NULL )
    return 0;

  X3DShapeNode *sh = dynamic_cast< H3D::X3DShapeNode* >( shape->getValue() );

  // If the shape contains no geometry then do not initialize
  if ( !sh || !sh->geometry->getValue() )
    return 0;

  // Purge all pending updates to parameters since we will initialise all
  // parameters when the body is created. Otherwise the parameters will be
  // set again next frame
  valueUpdater->upToDate();

  engine_thread= pt;
  PhysicsEngineParameters::ShapeParameters* p= getCollidableParameters ( true );

  // Get the space id of parent space 
  if ( CollisionSpace *cs = dynamic_cast< CollisionSpace * >( parent ) ) {
    p->setParentSpaceId( cs->getSpaceId() );
  } else if ( parent == NULL ) {
    p->setParentSpaceId( CollidableParameters::NO_SPACE );
  } else {
    p->setParentSpaceId( CollidableParameters::WORLD_SPACE );
  }

  collidable_id = engine_thread->addCollidable( *p );
  collidable_id_map[collidable_id] = this;

  return 0;
}


int CollidableShape::deleteCollidable() {
  if ( collidableInitialized() ) {
    std::map< H3DCollidableId, X3DNBodyCollidableNode * >::iterator j = collidable_id_map.find( collidable_id );
    if( j != collidable_id_map.end() ) collidable_id_map.erase( j );
    engine_thread->removeCollidable( collidable_id );
    engine_thread = NULL;
    return 0;
  }
  return -1;
}


void CollidableShape::initialize() {
  const Vec3f &size = bboxSize->getValue();
  BoxBound *bb = 0;
  if( size.x == -1 || size.y == -1 || size.z == -1 ) {
    // Create a transforedboxbound since the bound will be calculated
    // from the contained shape.
    bb = new TransformedBoxBound();
  } else {
    // Use the hint to calculate the bound. The bound might be recalculated
    // if the contained Shapes bound is a box bound.
    bb = new BoxBound();
    bb->center->setValue( bboxCenter->getValue() );
    bb->size->setValue( bboxSize->getValue() );
  }
  this->bound->setValue( bb );
  X3DChildNode::initialize();
}

PhysicsEngineParameters::ShapeParameters* CollidableShape::getCollidableParameters( bool all_params ) {
  ShapeParameters* p= static_cast<ShapeParameters*> ( X3DNBodyCollidableNode::getCollidableParameters ( all_params ) );

  if( all_params || valueUpdater->hasCausedEvent( selfCollide ) ) {
    p->setSelfCollide( selfCollide->getValue() );
  }

  if( all_params || valueUpdater->hasCausedEvent( clipPlanes ) ) {
    NodeVector clip_planes = clipPlanes->getValue();
    vector< Vec4d > plane_equations;
    for( unsigned int i = 0; i < clip_planes.size(); ++i ) {
      if( clip_planes[i] ) {
        ClipPlane * cp = static_cast< ClipPlane * >(clip_planes[i]);
        if( cp->enabled->getValue() )
          plane_equations.push_back( cp->plane->getValue() );
      }
    }
    p->setClipPlanes( plane_equations );
  }

  if( all_params || valueUpdater->hasCausedEvent( shape ) ) {
    
    X3DShapeNode *sh = dynamic_cast< H3D::X3DShapeNode* >( shape->getValue() );
    
    bool updateTheShape = true;
    if(!all_params) {
    // we shouldn't update the shape if no event has been generated by one of the geometry's routes and we need to specifically exclude a break_list_field in case one exists and has generated an event, otherwise various rendering-related events would cause us to rebuild the shape unnecessarily.
      X3DGeometryNode* geom_displaylist = sh->geometry->getValue();
      if(geom_displaylist && (geom_displaylist->displayList->nrPendingEvents() > 0)) {
        size_t minimum_pending_events_for_update = 1;
        Field::FieldVector r_in = geom_displaylist->displayList->getRoutesIn();
        for( size_t i=0; i<r_in.size(); i++ ){
          if( r_in[i]->getName() == "H3DDisplayListObject::break_list_field" ){
            if(geom_displaylist->displayList->hasCausedEvent(r_in[i])) {
              ++minimum_pending_events_for_update;
            }
          }
        }
          // We calculate how many pending events there should be based on if the break list field or the appearance fields have caused events. It's only if there are more pending events than number of events that we want to ignore (break list and appearance) that the shape should be updated.
        if(geom_displaylist->displayList->nrPendingEvents() < minimum_pending_events_for_update) {
        updateTheShape = false;
        }
      } else {
        updateTheShape = false; // Don't update if no pending events
      }
    }


    if( all_params || updateTheShape ) {
      p->setOriginalShape( sh->geometry->getValue() );
    }

    if( all_params || valueUpdater->hasCausedEvent( updateShapeBounds ) ) {
      if( updateShapeBounds->getValue() ) {
        p->getOriginalShape()->boundTree->upToDate();
      }
      // update the value
      p->setUpdateShapeBounds( updateShapeBounds->getValue() );
    }
  }
  
  return p;
}

void CollidableShape::SFBound::update() {
  if( routes_in.size() > 1 ) {
    // Decide which kind of bound to use depending on the bound of the
    // contained shape.
    Bound *_bound = static_cast< SFBound * >( routes_in[1] )->getValue();
    BoxBound *box_bound = dynamic_cast< BoxBound * >( _bound );
    if( box_bound ) {
      value = new TransformedBoxBound;
    } else if( dynamic_cast< InfiniteBound * >( _bound ) ) {
      value = new InfiniteBound;
    } else if( dynamic_cast< EmptyBound * >( _bound ) ) {
      value = new EmptyBound;
    } else {
      stringstream s;
      s << "Unsupported Bound type " << typeid( *_bound ).name();
      throw Exception::H3DAPIException( s.str(), H3D_FULL_LOCATION );
    }
  } else {
    // simply create a new TransformedBoxBound.
    if( owner ) {
      CollidableShape *cs = static_cast< CollidableShape * >(owner);
      TransformedBoxBound *tb = new TransformedBoxBound();
      tb->center->setValue( cs->bboxCenter->getValue() );
      tb->size->setValue( cs->bboxSize->getValue() );
      value = tb;
    } else
      value = new TransformedBoxBound;
  }
}

void CollidableShape::TransformMatrix::update() {
  if( routes_in.size() > 2 ) {
    // Calculate a new matrix, it is simply the rotation matrix with
    // added translation.
    value= Matrix4f (
      static_cast< SFVec3f * >(routes_in[0])->getValue(),
      static_cast< SFRotation * >(routes_in[1])->getValue(),
      static_cast< SFVec3f * >(routes_in[2])->getValue() );
  }
}

