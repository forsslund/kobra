//////////////////////////////////////////////////////////////////////////////
//    Copyright 2004-2019, SenseGraphics AB
//
//    This file is part of MedX3D.
//
//    MedX3D is free software; you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation; either version 2 of the License, or
//    (at your option) any later version.
//
//    MedX3D is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with MedX3D; if not, write to the Free Software
//    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//
//    A commercial license is also available. Please contact us at 
//    www.sensegraphics.com for more information.
//
//
/// \file OctTree.cpp
/// \brief CPP file for OctTree, X3D scene-graph node
///
//
//
//////////////////////////////////////////////////////////////////////////////

#include <H3D/MedX3D/OctTree.h>
#include <H3D/X3DViewpointNode.h>

using namespace H3D;

// Add this node to the H3DNodeDatabase system.
H3DNodeDatabase OctTree::database("OctTree", 
                                  &(newInstance<OctTree>), 
                                  typeid( OctTree ),
                                  &X3DChildNode::database );

namespace OctTreeInternals {
  FIELDDB_ELEMENT( OctTree, highRes, INPUT_OUTPUT )
  FIELDDB_ELEMENT( OctTree, lowRes, INPUT_OUTPUT )
  FIELDDB_ELEMENT( OctTree, lowResActive, OUTPUT_ONLY )
  FIELDDB_ELEMENT( OctTree, center, INPUT_OUTPUT )
  FIELDDB_ELEMENT( OctTree, range, INPUT_OUTPUT )
}

OctTree::OctTree(   Inst< MFChild      > _highRes,
                 Inst< SFLowResNode > _lowRes,
                 Inst< SFNode       > _metadata,
                 Inst< SFBool       > _lowResActive,
                 Inst< SFVec3f      > _center,  
                 Inst< SFFloat      > _range,
                 Inst< SFBound      > _bound,
                 Inst< SFVec3f      > _bboxCenter,
                 Inst< SFVec3f      > _bboxSize,
                 Inst< DisplayList > _displayList ) :
  X3DChildNode(_metadata),
  X3DBoundedObject(_bound, _bboxCenter, _bboxSize), 
  H3DDisplayListObject( _displayList ),
  use_union_bound( false ),
  highRes( _highRes), lowRes(_lowRes), 
  lowResActive(_lowResActive), center(_center), range(_range)
{

  type_name = "OctTree";
  database.initFields( this );
  center->setValue(Vec3f(0,0,0));
  range->setValue(20);
 
  // ownership
  displayList->setOwner( this );

  //Routing
  lowResActive->route( displayList );
  lowRes->route( displayList );
  highRes->route( displayList );
  center->route( displayList );
  range->route( displayList );
}

void OctTree::traverseSG( TraverseInfo &ti ) {
  X3DViewpointNode *vp = X3DViewpointNode::getActive();
  if( vp ) {
    // goes from the viewports local coordinates to the global
    // then from global to OctTree local coordinates 
    Matrix4f vp_to_OctTree = 
      ti.getAccInverseMatrix() * 
      vp->accForwardMatrix->getValue();

    //gets the position of the vp in OctTree local coordinates.
    Vec3f vp_pos_OctTree = vp_to_OctTree *
      vp->totalPosition->getValue();

    //calculates the distance (d) from the vp position to centre
    H3DFloat distance = (vp_pos_OctTree - center->getValue()).length();
    H3DFloat range_value = range->getValue();
    //cerr << "distance = " << distance << endl;
    const NodeVector &highRes_vector=(highRes->getValue());
    size_t no_of_highRes = highRes_vector.size();
    X3DNode *lowRes_node=(lowRes->getValue());
    if(distance< range_value){
      if(lowResActive->getValue()) lowResActive->setValue(false,id);
      if(highRes_vector.size() !=0){
        if(no_of_highRes<8){
          for(size_t i=0; i<no_of_highRes; ++i){
            highRes_vector[i]->traverseSG(ti);   
          }
        }
        else{
          for(int i=0; i<8; ++i){
            highRes_vector[i]->traverseSG(ti);
          }
        }
      }
    }
    else{
      if(lowRes_node){
        lowRes_node->traverseSG(ti);
        if(!lowResActive->getValue()) lowResActive->setValue(true,id);
      }
    }
  }
}

void OctTree::render(){ 
  const NodeVector &highRes_vector=(highRes->getValue());
  int no_of_highRes = highRes->size();
  X3DNode *lowRes_node=(lowRes->getValue());

  if(lowResActive->getValue()){
    H3DDisplayListObject *c = dynamic_cast< H3DDisplayListObject* >
      (lowRes->getValue() );
    if ( c ) {
      c->displayList->callList();
    }
    else{
      lowRes_node->render();
    }
  }
  else{
    if(no_of_highRes<8){
      for(int i=0; i<no_of_highRes; ++i){
        H3DDisplayListObject *c = dynamic_cast< H3DDisplayListObject* >
          (highRes_vector[i] );
        if ( c ) {
          c->displayList->callList();
        }
        else{
          highRes_vector[i]->render();
        }
      }
    }
    else{
      for(int i=0; i<8; ++i){
        H3DDisplayListObject *c = dynamic_cast< H3DDisplayListObject* >
          ( highRes_vector[i] );
        if ( c ) {
          c->displayList->callList();
        }
        else{
          highRes_vector[i]->render();
        }
      }
    }
  }
}

void OctTree::SFBound::update() {
  value = Bound::SFBoundUnion( routes_in.begin(),
                               routes_in.end() );
}

void OctTree::closestPoint( const Vec3f &p,
                            NodeIntersectResult &result ) {
  if( lowResActive->getValue() ) {
    lowRes->getValue()->closestPoint( p, result );
  }
  else{
    const NodeVector &highRes_nodes = highRes->getValue();
    for( unsigned int i = 0; i < highRes_nodes.size(); ++i ) {
      highRes_nodes[i]->closestPoint( p, result );
    }
  }
}

bool OctTree::movingSphereIntersect( H3DFloat radius,
                                    const Vec3f &from, 
                                    const Vec3f &to,
                                    NodeIntersectResult &result ) {
 if(lowResActive->getValue()){
   if( lowRes->getValue()->movingSphereIntersect( radius, from, to, result ) ){
     return true;
   }
 } else {
    const NodeVector &highRes_nodes = highRes->getValue();
    bool intersection = false;
    for( unsigned int i = 0; i < highRes_nodes.size(); ++i ) {
      if( highRes_nodes[i]->movingSphereIntersect( radius, from, to, result )){
        intersection = true;
      }
    }
    return intersection;
  }
 return false;
}


bool OctTree::lineIntersect(
                  const Vec3f &from, 
                  const Vec3f &to,
                  LineIntersectResult &result ) {
  bool intersect = false;
  Bound * the_bound = bound->getValue();

  if( !the_bound ||
    the_bound->lineSegmentIntersect( from, to ) ) {
      if(lowResActive->getValue()){
        if( lowRes->getValue()->lineIntersect( from, to, result ) ) {
            intersect = true;
        }
      }
      else{
        const NodeVector &highRes_nodes = highRes->getValue();
        for( unsigned int i = 0; i < highRes_nodes.size(); ++i ) {
          if( highRes_nodes[i]->lineIntersect( from, to, result ) ) {
              intersect = true;
          }
        }
      }
  
  }
  return intersect;
}



// Adding highRes 
void OctTree::MFChild::onAdd( Node *n ) {
  MFChildBase::onAdd( n );
  
  X3DChildNode *c = static_cast< X3DChildNode* >( n );
  OctTree *o = static_cast< OctTree* >( owner );
  if ( c ) {  
    if( o->use_union_bound ) {
      H3DBoundedObject *bo = 
        dynamic_cast< H3DBoundedObject * >( n );
      if( bo ) {
        MatrixTransform *t = dynamic_cast< MatrixTransform *>( n );
        if( t ) {
          t->transformedBound->route( o->bound );
        }
        else {
          bo->bound->route( o->bound );
        }
      }
    }
  }
}

void OctTree::MFChild::onRemove( Node *n ) {
  X3DChildNode *c = static_cast< X3DChildNode* >( n );
  OctTree *o = static_cast< OctTree* >( owner );
  if ( c ) {
    if( o->use_union_bound ) {
      H3DBoundedObject *bo = 
        dynamic_cast< H3DBoundedObject * >( n );
      if( bo ) {
        MatrixTransform *t = dynamic_cast< MatrixTransform *>( n );
        if( t ) {
          t->transformedBound->route( o->bound );
        } 
        else {
          bo->bound->unroute( o->bound );
        }
      }
    }

    MFChildBase::onRemove( n );
  }
}
void OctTree::SFLowResNode::onAdd( Node *n ) {
  SFNodeBase::onAdd( n );
  if( n ) {
    X3DGroupingNode *group_node = dynamic_cast<X3DGroupingNode*>(n);
    X3DShapeNode *shape_node = dynamic_cast<X3DShapeNode*>(n);
    X3DVolumeNode *volume_node = dynamic_cast<X3DVolumeNode*>(n);
    OctTree *o = static_cast<OctTree*>(owner);
    if( !(group_node || shape_node || volume_node) ) {
      Node *pi = getPrototypeNode( n );
      if( !(dynamic_cast<X3DGroupingNode *>(pi) ||
             dynamic_cast<X3DShapeNode *>(pi) ||
             dynamic_cast<X3DVolumeNode *>(pi)) ) {
        stringstream s;
        s << "Expecting X3DGroupingNode, X3DShapeNode or X3DVolumeNode";
        throw InvalidNodeType( n->getTypeName(),
                               s.str(),
                               H3D_FULL_LOCATION );
      }
    } else {
      if( o->use_union_bound ) {
        H3DBoundedObject *bo =
          dynamic_cast<H3DBoundedObject *>(n);
        if( bo ) {
          MatrixTransform *t = dynamic_cast<MatrixTransform *>(n);
          if( t ) {
            t->transformedBound->route( o->bound );
          } else {
            bo->bound->route( o->bound );
          }
        }
      }
    }
  }
}

void OctTree::SFLowResNode::onRemove( Node *n ) {
  X3DGroupingNode *group_node = dynamic_cast< X3DGroupingNode* >( n );
  X3DShapeNode *shape_node = dynamic_cast< X3DShapeNode* >( n );
  X3DVolumeNode *volume_node = dynamic_cast< X3DVolumeNode* >( n );
  OctTree *o = static_cast< OctTree* >( owner );
  if ( group_node || shape_node || volume_node) {
    if( o->use_union_bound ) {
      H3DBoundedObject *bo = 
        dynamic_cast< H3DBoundedObject * >( n );
      if( bo ) {
        MatrixTransform *t = dynamic_cast< MatrixTransform *>( n );
        if( t ) {
          t->transformedBound->route( o->bound );
        } else {
          bo->bound->unroute( o->bound );
        }
      }
    }

    //MFChildBase::onRemove( n );
  }
}
