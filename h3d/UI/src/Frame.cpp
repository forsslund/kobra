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
/// \file Frame.cpp
/// \brief CPP file for Frame, X3D scene-graph node
///
//
//
//////////////////////////////////////////////////////////////////////////////

// UI includes
#include <H3D/UI/Frame.h>
#include <H3D/UI/GridLayoutManager.h>

using namespace H3D;

H3DNodeDatabase Frame::database( 
        "Frame", 
        &newInstance< Frame >,
        typeid( Frame ),
        &H3DWidgetNode::database 
        );

namespace FrameInternals {
  FIELDDB_ELEMENT( Frame, children, INPUT_OUTPUT )
  FIELDDB_ELEMENT( Frame, layoutManager, INPUT_OUTPUT )
}

Frame::Frame( Inst< SFNode              > _metadata   ,
              Inst< SFBound             > _bound      ,
              Inst< DisplayList         > _displayList,
              Inst< SFString            > _tag        ,
              Inst< SFBool              > _enabled    ,
              Inst< SFVec3f             > _desiredSize,
              Inst< SFVec3f             > _actualSize,
              Inst< SFLayoutInfoNode    > _layoutInfo,
              Inst< MFWidgetNode        > _children,
              Inst< SFLayoutManagerNode > _layoutManager ) :
  H3DWidgetNode( _metadata, _bound, _displayList, _tag,
                 _enabled, _desiredSize, _actualSize, _layoutInfo ),
  children( _children ),
  layoutManager( _layoutManager ),
  repackField( new RepackField ) {

  type_name = "Frame";
  database.initFields( this );
  repackField->setOwner( this );
  repackField->setName( "repackField" );

  layoutManager->setValue( new GridLayoutManager );

  children->route( displayList );
  layoutManager->route( displayList );

  // route the fields we want to cause a repack of the frame
  layoutManager->route( repackField );
  children->route( repackField );
  desiredSize->route( repackField );
  actualSize->route( repackField );

  children->route( transform.get()->children );
  children->route( repack );
}

void Frame::render() {
  // make sure the frame has been packed properly.
  repackField->upToDate();
  H3DWidgetNode::render();
  transform->displayList->callList();
}

void Frame::traverseSG( TraverseInfo &ti ) {
  // make sure the frame has been packed properly.
  repackField->upToDate();

  H3DWidgetNode::traverseSG( ti );
  transform->traverseSG( ti );
}

void Frame::setParent( H3DWidgetNode * _parent ) {
  H3DWidgetNode::setParent( _parent );

  // Since our parent has changed, notify our children so they know the full hierarchy
  for( NodeVector::const_iterator i = children->begin(); i != children->end(); ++i ) {
    if( H3DWidgetNode* w = dynamic_cast <H3DWidgetNode*> (*i) ) {
      w->setParent( this );
    }
  }
}

