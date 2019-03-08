//////////////////////////////////////////////////////////////////////////////
//    Copyright 2018-2019, SenseGraphics AB
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
/// \file WxReferenceInfoResultDialog.cpp
/// \brief CPP file for WxReferenceInfoResultDialog.
///
//
//
//////////////////////////////////////////////////////////////////////////////

// ---------------------------------------------------------------------------
//  Includes
// ---------------------------------------------------------------------------

#include "WxReferenceInfoResultDialog.h"
#ifdef HAVE_PROFILER

#include <H3DUtil/Console.h>
#include <wx/clipbrd.h>
#include <H3D/Scene.h>
#include <H3D/Profiling.h>
#include <H3D/PythonScript.h>

using namespace std;


WxReferenceInfoResultDialog::WxReferenceInfoResultDialog ( wxWindow *parent,
                 wxWindowID id,
                 const wxString &title,
                 const wxPoint& pos,
                 const wxSize& size,
                 long style
                 ): wxDialog(parent, id, title, pos, size, style)
{
  AUTOREF_DEBUG_NAME( node, "WxReferenceInfoResultDialog::node")

  wxBoxSizer *topsizer = new wxBoxSizer( wxVERTICAL );
  // create text ctrl with minimal size 400x200
  logText = (wxTextCtrl *) NULL;
 /* wxRichTextCtrl* logText = new wxRichTextCtrl ( this, -1, wxT(""),
                             wxDefaultPosition, wxSize(400, 200),
                             wxTE_MULTILINE | wxTE_READONLY|wxTE_RICH2);*/
  logText = new wxTextCtrl( this, -1, wxT(""),
                             wxDefaultPosition, wxSize(400, 200),
                             wxTE_MULTILINE | wxTE_READONLY|wxTE_RICH );

  logText->SetFont( wxFont(10, wxFONTFAMILY_TELETYPE, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_NORMAL) );
  //logText->SetDoubleBuffered(true);
  logText->SetFocus();
  clip_board = new wxClipboard();
  //logText->ShowNativeCaret(false);
  //logText->HideNativeCaret();
  topsizer->Add(logText, 
                1,            // make vertically stretchable
                wxEXPAND |    // make horizontally stretchable
                wxALL,        //   and make border all around
                10 );         // set border width to 10 */

  // Refresh button
  wxButton *refreshButton = new wxButton( this, wxID_REFRESH, wxT("&Refresh") );

  // boxsizer for the buttons
  wxBoxSizer *button_sizer = new wxBoxSizer( wxHORIZONTAL );
  button_sizer->Add(refreshButton,
                    0,          // make horizontally unstretchable
                    wxALL,      // make border all around (implicit top alignment)
                    10 );       // set border width to 10


  topsizer->Add(button_sizer,
                0,                // make vertically unstretchable
                wxALIGN_LEFT ); // no border and centre horizontally
  
  SetSizer( topsizer );      // use the sizer for layout
  
  topsizer->SetSizeHints( this );   // set size hints to honour minimum size
}

WxReferenceInfoResultDialog::~WxReferenceInfoResultDialog() {
  delete clip_board;
}

/*******************Event Table*********************/
BEGIN_EVENT_TABLE(WxReferenceInfoResultDialog, wxDialog)
  EVT_BUTTON (wxID_REFRESH, WxReferenceInfoResultDialog::OnRefresh)
  EVT_IDLE (WxReferenceInfoResultDialog::OnIdle)
  EVT_CLOSE (WxReferenceInfoResultDialog::OnWindowClose)
END_EVENT_TABLE()

/*******************Member Functions*********************/


void WxReferenceInfoResultDialog::OnWindowClose(wxCloseEvent &event) {
  Destroy();
}

void WxReferenceInfoResultDialog::OnCopyToClipboard(wxCommandEvent &event){
  bool copied = false;
  if(!logText->IsEmpty()){
    if(clip_board->Open()){
      clip_board->Clear();
      wxTextDataObject* tmp=new wxTextDataObject();
      tmp->SetText(logText->GetValue());
      clip_board->SetData(tmp);
      clip_board->Flush();
      copied = true;
      clip_board->Close();
    }
  }
}

void WxReferenceInfoResultDialog::setDisplayedNode(H3D::Node *n, bool refresh) {
  node.reset(n);
  if (refresh) {
    updateTextOutput();
  }
}

void WxReferenceInfoResultDialog::OnRefresh(wxCommandEvent &event) {
  updateTextOutput();
}

#include <H3D/PythonTypes.h>

void WxReferenceInfoResultDialog::updateTextOutput() {
  // the number of references of selected node in standalone containers such as AutoRef, AutoRefVector and DEFNodes,
  // excluding such containers that are used for storing SFNode or MFNode values
  unsigned int container_references_standalone = 0;

  // the number of references of selected node for SFNode or MFNode instances.
  unsigned int container_references_fields = 0;

  // the number of references of selected node in the scene
  unsigned int scene_references = 0;

  if( node.get() ) {
    std::stringstream result;
    unsigned int total_ref_count = node->getRefCount();
 
    // References from Scene

    H3D::PythonScript *ps = new H3D::PythonScript;
    ps->url->push_back ("python:pass"); // Minimal inline script to initialize interpreter
    ps->setName ("Injected_" + ps->getName ());
    ps->moduleName->setValue( ps->getName() );
    // Reference node to initialize script
    H3DUtil::AutoRef< H3D::PythonScript >ps_ref(ps);
    PyGILState_STATE state = PyGILState_Ensure();
    PyObject *py_node = H3D::PyNode_FromNode( node.get() );
    PyObject *py_module = ps->getPythonModule(); // borrowed
    if (py_module) {
      PyObject *module_dict = PyModule_GetDict( static_cast< PyObject * >( py_module ) ); // borrowed
      PyObject *py_node_name = PyString_FromString( "node_to_use" ); // new
      PyDict_SetItem( module_dict, 
                      py_node_name,
                      py_node );

      result << ps->execute("import H3DUtils");     
      result << ps->execute("refs = H3DUtils.getNodeRefsFromScene( node_to_use )");

      PyObject *py_refs_name = PyString_FromString( "refs" ); // new
      PyObject *refs = PyDict_GetItem( module_dict, py_refs_name ); // borrowed
      if (refs && PyList_Check(refs)) {
        scene_references = PyList_Size( refs );
        result << ps->execute("i = 0");
     
        for (unsigned int i = 0; i < scene_references; ++i) {
          result << "(" << i << ") Scene ref:" << endl;
          result << ps->execute("H3DUtils.printNodePath ( refs[i], \"      \", sys.stdout )");   
          result << ps->execute("i += 1");          
        }
      }
      Py_DECREF( py_node_name );
      Py_DECREF( py_refs_name );

    }
    Py_DECREF( py_node);
    PyGILState_Release(state);

    // AutoRefVector references
    H3DUtil::AutoRefVectorBase::auto_ref_vectors_lock.lock();
    for( std::set< H3DUtil::AutoRefVectorBase * >::const_iterator i =
             H3DUtil::AutoRefVectorBase::auto_ref_vectors.begin();
         i != H3DUtil::AutoRefVectorBase::auto_ref_vectors.end();
         ++i) {
      vector< H3DUtil::RefCountedClass * > nodes;
      (*i)->getContent( nodes );
      for (vector< H3DUtil::RefCountedClass * >::const_iterator ni = nodes.begin();
        ni != nodes.end();
        ++ni) {
        if (*ni == node.get()) {
          if( (*i)->name == "MFNode::value" ) {
            container_references_fields++;
          } else {
            result << "(" << scene_references + container_references_standalone++ <<") AutoRefVector: " << (*i)->name << endl;
          }
        }
      }
    }
    H3DUtil::AutoRefVectorBase::auto_ref_vectors_lock.unlock();

    // AutoRef references
    H3DUtil::AutoRefBase::auto_refs_lock.lock();
    for( std::set< H3DUtil::AutoRefBase * >::const_iterator i =
      H3DUtil::AutoRefBase::auto_refs.begin();
      i != H3DUtil::AutoRefBase::auto_refs.end();
      ++i) {
      if( (*i)->getContent() == node.get() ) {  
        if( (*i)->name == "SFNode::value" ) {
           container_references_fields++;
        } else {
           result <<  "(" << scene_references + container_references_standalone++ <<") AutoRef:  " << (*i)->name << endl;
        }
      }
    }
    H3DUtil::AutoRefBase::auto_refs_lock.unlock();

    // DEFNodes references
    H3D::X3D::DEFNodesBase::DEF_nodes_lock.lock();
    for (std::set< H3D::X3D::DEFNodesBase * >::const_iterator i =
             H3D::X3D::DEFNodesBase::DEF_nodes.begin();
         i != H3D::X3D::DEFNodesBase::DEF_nodes.end();
         ++i) {
      H3D::X3D::DEFNodes *dn = dynamic_cast<H3D::X3D::DEFNodes *>(*i);
      if (dn) {
        for (H3D::X3D::DEFNodes::const_iterator ni = dn->begin();
          ni != dn->end();
          ++ni) {
          if ((*ni).second == node.get()) {
            result << "(" << scene_references + container_references_standalone++ << ") DEFNodes: " << (*i)->name << endl;
          }
        }
      }
    }
    H3D::X3D::DEFNodesBase::DEF_nodes_lock.unlock();
    unsigned int refs_missing = total_ref_count - scene_references - container_references_standalone;
    result << endl << endl;
    result << "SUMMARY: " << endl;
    result << "Total ref count    : " << total_ref_count << endl;
    result << "Refs in scene      : " << scene_references << endl;
    result << "Refs in containers : " << container_references_standalone << " (+" << container_references_fields << " for fields)" << endl;
    result << "Refs missing       : " << refs_missing;

    if( refs_missing > 0 ) {
        result << endl << endl << "Possible location of missing references below. This lists all places where the node is reachable through any python module. ";
        result << "Ignore references to \"node_to_use\" as that is internal only to the injected PythonScript that is used to get this printout. ";
        result << "If no reference is printed below then the most probable cause is that they are held by python variables with circular references";
        result << " which are not referenced from any python module." << endl << endl;
        result << ps->execute("module_node_refs = H3DUtils.getNodeRefsFromPythonModules ( node_to_use )");   
        result << ps->execute("\
for node_ref in module_node_refs:\n\
  print(\"------------------------------------------------------\")\n\
  H3DUtils.printPythonPath( node_ref[0][1], outstream = sys.stdout )\n\
  H3DUtils.printNodePath( node_ref, outstream = sys.stdout )\n");
 
    }


    logText->SetValue(wxString(result.str().c_str(), wxConvUTF8));
  }
}

void WxReferenceInfoResultDialog::OnIdle(wxIdleEvent &event) {
  if( IsShown() ) {
    H3D::TimeStamp now;
    if( now - last_update > 1 ) {
      if (!node.get()) {
        SetTitle(wxT(""));
      } else {
        string tree_string = node->getTypeName();
        if (node->hasName()) {
          tree_string = tree_string + " (" + node->getName() + ")";
        }

        stringstream s;
        s << node->getTypeName();

        if (node->hasName()) {
          s << " (" << node->getName() << ")";
        }

        s << " (refs: " << node->getRefCount() << ")";

        SetTitle(wxString(s.str().c_str(), wxConvUTF8));
      }
      last_update = now;
    }
  }
}
#endif
