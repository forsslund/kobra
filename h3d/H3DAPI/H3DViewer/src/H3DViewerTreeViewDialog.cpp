//////////////////////////////////////////////////////////////////////////////
//    Copyright 2009-2019, SenseGraphics AB
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
/// \file H3DViewerTreeViewDialog.cpp
/// \brief CPP file for H3DViewerTreeViewDialog.
///
//
//
//////////////////////////////////////////////////////////////////////////////
#include "H3DViewerTreeViewDialog.h"
#include "H3DViewerPopupMenus.h"
#include <fstream>
#include <wx/wx.h>
#include <H3D/MetadataString.h>
#include <H3D/Scene.h>
#include <H3D/Viewpoint.h>
#include <H3D/IndexedTriangleSet.h>
#include <H3D/Coordinate.h>
#include <H3D/MatrixTransform.h>
#include <H3D/X3DShapeNode.h>
#include <H3D/IndexedFaceSet.h>
#include <H3D/X3DGeometryNode.h>
#include <H3D/X3D.h>

#include <H3DUtil/LoadImageFunctions.h>
#include <wx/confbase.h>

H3DViewerTreeViewDialog::H3DViewerTreeViewDialog(wxWindow* parent)
  :
  TreeViewDialog(parent),
  shown_last_loop(false),
  force_update_labels(false) {
  TreeViewTree->AddRoot(wxT("World"));
  // add the bindable nodes in the tree view
  bindable_tree_id = TreeViewTree->AppendItem(TreeViewTree->GetRootItem(),
                                              wxT("Active bindable nodes"));
  TreeViewTree->Expand(TreeViewTree->GetRootItem());

  AUTOREF_DEBUG_NAME( selected_node, "H3DViewerTreeViewDialog::selected_node")
 
  #ifdef USE_PROPGRID
  field_values_panel = new H3DViewerFieldValuesPanelPropGrid(SplitterWindow, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL);
  #else
  field_values_panel = new H3DViewerFieldValuesPanel(SplitterWindow, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL);
  #endif

  // See if there's already a setting for "focus_search_result_checkbox"
  wxConfigBase* cfg = wxConfigBase::Get();
  bool focus_search_result = false;
  bool result = cfg->Read(wxT("focus_search_result_checkbox"), &focus_search_result);

  // If there isn't, set value to false (default).
  if(!result) {
    cfg->Write(wxT("focus_search_result_checkbox"), focus_search_result);
  }

  focus_search_result_checkbox->SetValue(focus_search_result);

  menu_container = new H3DViewerPopupMenus(this, this);
  displayFieldsFromNode(NULL);

  if(SplitterWindow->IsSplit()) {
    SplitterWindow->Unsplit();
  }

  SplitterWindow->SplitVertically(TreeViewPanel, field_values_panel, 340);
  this->Layout();

  this->Connect(wxEVT_CHAR_HOOK, wxKeyEventHandler(H3DViewerTreeViewDialog::onCharHook));
  #ifdef HAVE_PROFILER
  ProfileCheckbox->SetValue(H3D::Profiling::profile_group_nodes);
  #else
  ProfileCheckbox->Hide();
  stop_treeview_update_checkbox->Hide();
  #endif
}

H3DViewerTreeViewDialog::~H3DViewerTreeViewDialog() {
  this->Disconnect(wxEVT_CHAR_HOOK, wxKeyEventHandler(H3DViewerTreeViewDialog::onCharHook));
  // If the menu bar is set to NULL in the constructor it will not be
  // cleaned up. Done here explicitly.
  //if( GetMenuBar() == NULL )
  //  delete m_menubar1;

  // Save focus_search_result_checkbox value.
  wxConfigBase* cfg = wxConfigBase::Get();
  cfg->Write(wxT("focus_search_result_checkbox"), focus_search_result_checkbox->IsChecked());
}

void H3DViewerTreeViewDialog::showImage(X3DTextureNode& _image) {
  H3DViewImage* view_image = new H3DViewImage(this, _image);
  view_image->Show();
}

void H3DViewerTreeViewDialog::OnNodeSelected(wxTreeEvent& event) {

  TreeIdMap::iterator ni = node_map.find(event.GetItem().m_pItem);
  if(ni == node_map.end()) {
    selected_node.reset(NULL);
  } else {
    selected_node.reset((*ni).second.get());
  }
}

bool findStringCase(const wxString &s1, const wxString &s2) {
  return s1.find(s2) != string::npos;
}

bool findStringNoCase(const wxString &s1, const wxString &s2) {
  return findStringCase(s1.Lower(), s2);
}

bool H3DViewerTreeViewDialog::onSearchTextCtrlHelp(const wxTreeItemId &item, const wxString &to_find, wxTreeItemId &found_item, bool(*compare_func)(const wxString &s1, const wxString &s2), const wxTreeItemId &check_parent, bool reverse_direction /* = false*/) {
  if(!check_parent.IsOk() || item != check_parent) {
    if(compare_func(TreeViewTree->GetItemText(item), to_find)) {
      found_item = item;
      return true;
    }

    if(reverse_direction) {
      wxTreeItemId last_child = TreeViewTree->GetLastChild(item);
      if(last_child.IsOk()) {
        if(onSearchTextCtrlHelp(last_child, to_find, found_item, compare_func, wxTreeItemId(), reverse_direction))
          return true;
      }
    } else {
      wxTreeItemIdValue cookie;
      wxTreeItemId first_child = TreeViewTree->GetFirstChild(item, cookie);
      if(first_child.IsOk()) {
        if(onSearchTextCtrlHelp(first_child, to_find, found_item, compare_func, wxTreeItemId(), reverse_direction))
          return true;
      }
    }
  }

  wxTreeItemId id;
  if(reverse_direction) {
    id = TreeViewTree->GetPrevSibling(item);
    while(id.IsOk()) {
      if(compare_func(TreeViewTree->GetItemText(id), to_find)) {
        found_item = id;
        return true;
      }

      wxTreeItemId last_child = TreeViewTree->GetLastChild(id);
      if(last_child.IsOk()) {
        if(onSearchTextCtrlHelp(last_child, to_find, found_item, compare_func, wxTreeItemId(), reverse_direction))
          return true;
      }
      id = TreeViewTree->GetPrevSibling(id);
    }
  } else {
    id = TreeViewTree->GetNextSibling(item);
    while(id.IsOk()) {
      if(compare_func(TreeViewTree->GetItemText(id), to_find)) {
        found_item = id;
        return true;
      }
      wxTreeItemIdValue cookie;
      wxTreeItemId first_child = TreeViewTree->GetFirstChild(id, cookie);
      if(first_child.IsOk()) {
        if(onSearchTextCtrlHelp(first_child, to_find, found_item, compare_func))
          return true;
      }
      id = TreeViewTree->GetNextSibling(id);
    }
  }

  if(check_parent) {
    id = TreeViewTree->GetItemParent(item);
    if(id.IsOk() && onSearchTextCtrlHelp(id, to_find, found_item, compare_func, id, reverse_direction))
      return true;
  }


  return false;
}

void H3DViewerTreeViewDialog::onSearchTextCtrl(wxCommandEvent& event) {
  wxString string_to_find = event.GetString();
  if(string_to_find.IsEmpty()) {
    return;
  }

  bool reverse_direction = false;
  if(event.GetClientData()) {
    reverse_direction = *static_cast<bool*>(event.GetClientData());
  }

  wxTreeItemId found_id;
  wxTreeItemId id_to_search_from;
  wxTreeItemId selected_id = TreeViewTree->GetSelection();
  wxTreeItemId check_parent;

  bool(*compare_func)(const wxString &s1, const wxString &s2) = findStringCase;

  if(!case_sensitive_checkbox->IsChecked()) {
    compare_func = findStringNoCase;
    string_to_find = string_to_find.Lower();
  }

  if(selected_id.IsOk() && compare_func(TreeViewTree->GetItemText(selected_id), string_to_find)) {
    if(reverse_direction) {
      id_to_search_from = TreeViewTree->GetLastChild(selected_id);
    } else {
      wxTreeItemIdValue cookie;
      id_to_search_from = TreeViewTree->GetFirstChild(selected_id, cookie);
    }

    if(!id_to_search_from.IsOk()) id_to_search_from = selected_id;
    check_parent = selected_id;
  } else {
    id_to_search_from = TreeViewTree->GetRootItem();
  }

  if(id_to_search_from.IsOk()) {
    if(onSearchTextCtrlHelp(id_to_search_from, string_to_find, found_id, compare_func, check_parent, reverse_direction)) {
      if(!TreeViewTree->IsSelected(found_id)) {
        // SetFocus so that the selected item becomes marked and you can move
        // around tree with arrow keys without first having to click the element.
        if(focus_search_result_checkbox->IsChecked()) {
          TreeViewTree->SetFocus();
        }

        TreeViewTree->SelectItem(found_id);
      }
    } else if(check_parent && onSearchTextCtrlHelp(TreeViewTree->GetRootItem(), string_to_find, found_id, compare_func, wxTreeItemId(), reverse_direction)) {
      if(!TreeViewTree->IsSelected(found_id)) {
        // SetFocus so that the selected item becomes marked and you can move
        // around tree with arrow keys without first having to click the element.
        if(focus_search_result_checkbox->IsChecked()) {
          TreeViewTree->SetFocus();
        }

        TreeViewTree->SelectItem(found_id);
      }
    }
  }

  TimeStamp end_time;
}

int H3DViewerTreeViewDialog::getNrTriangles(X3DGeometryNode *geom) {
  IndexedFaceSet *ifs = dynamic_cast<IndexedFaceSet *>(geom);
  if(ifs) {
    // IndexedFaceSet nrTriangles function is an upper bound so
    // try to calculate a more exact number here.
    const vector< int > &index = ifs->coordIndex->getValue();

    unsigned int i = 0;
    unsigned int nr_triangles = 0;

    while(i < index.size()) {
      unsigned int nr_face_vertices = 0;
      while(i < index.size() && index[i++] != -1) {
        ++nr_face_vertices;
      }

      if(nr_face_vertices >= 3) {
        nr_triangles += nr_face_vertices - 2;
      }
    }
    return nr_triangles;
  }
  return geom->nrTriangles();
}

void H3DViewerTreeViewDialog::showEntireSceneAsTree(H3DViewerTreeViewDialog::ExpandMode expand_new) {
  if (stop_treeview_update_checkbox->IsChecked()) {
    return;
  }
  // show the scene in the tree view.
  list< pair< H3D::Node *, string > > l;
  Scene *s = *Scene::scenes.begin();
  l.push_back(make_pair(s, s->defaultXMLContainerField()));

  wxTreeItemId tree_root = TreeViewTree->GetRootItem();
  std::stringstream ss;
  ss << "World (nodes alive: " << H3D::Node::nrNodesAlive() << " nodes created: " << Node::nrNodesCreated() << ")";
  TreeViewTree->SetItemText(tree_root, wxString(ss.str().c_str(), wxConvUTF8));

  updateNodeTree(tree_root, l, expand_new);

  l.clear();
  const X3DBindableNode::StackMapType &stacks = X3DBindableNode::getStackMap();
  for(X3DBindableNode::StackMapType::const_iterator i = stacks.begin();
  i != stacks.end(); ++i) {
    X3DBindableNode *b = X3DBindableNode::getActive((*i).first);
    if(b) l.push_back(make_pair(b, b->defaultXMLContainerField()));
  }
  updateNodeTree(bindable_tree_id, l, H3DViewerTreeViewDialog::EXPAND_NONE);
}

string H3DViewerTreeViewDialog::getNodeLabel(H3D::Node *n, const string &container_field) {
  // the name in the tree is NodeType(DEFed name)
  string tree_string = n->getTypeName();
  if(n->hasName()) {
    tree_string = tree_string + " (" + n->getName() + ")";
  }

  X3DGeometryNode *geom = dynamic_cast<X3DGeometryNode *>(n);
  if(geom) {

    int nr_triangles = getNrTriangles(geom);
    if(nr_triangles == -1) {
      tree_string = tree_string + " (Approx nr triangles: Unknown )";
    } else {
      char nr_triangles_str[255];
      sprintf(nr_triangles_str, " (Approx nr triangles: %d )", getNrTriangles(geom));
      tree_string = tree_string + nr_triangles_str;
    }
  }

  #ifdef HAVE_PROFILER
  if(H3D::Profiling::profile_group_nodes) {
    X3DGroupingNode *group = dynamic_cast<X3DGroupingNode *>(n);
    if(group) {
      char profile_string[255];

      sprintf(profile_string, " (r: %.1f ms t: %.1f ms)", 1000 * group->time_in_last_render, 1000 * group->time_in_last_traverseSG);
      tree_string = tree_string + profile_string;
    }
  }
  #endif

#ifdef H3D_REFERENCE_COUNT_DEBUG
  char refs_string[255];
  sprintf(refs_string, " (refs: %d)", n->getRefCount() );
  tree_string = tree_string + refs_string;
#endif

  if(container_field != n->defaultXMLContainerField()) {
    tree_string = tree_string + " (cf: " + container_field + ")";
  }
  return tree_string;
}


void H3DViewerTreeViewDialog::addNodeToTree(wxTreeItemId tree_id,
                                            H3D::Node *n,
                                            string container_field,
                                            H3DViewerTreeViewDialog::ExpandMode expand) {

  if(!n) return;

  if(n->getProtoInstanceParent()) {
    n = n->getProtoInstanceParent();
  }

  // Check if node has a metadata object named "TreeView_expandMode". If it does the value of it overrides
  // any other settings for expand mode for this node.
  X3DNode *x3d_node = dynamic_cast<X3DNode *>(n);
  if(x3d_node) {
    MetadataString *expand_mode_meta = dynamic_cast<MetadataString *>(x3d_node->getMetadataByName("TreeView_expandMode"));
    if(expand_mode_meta) {
      const vector<string> &values = expand_mode_meta->value->getValue();
      if(!values.empty()) {
        const string &mode = values[0];
        if(mode == "EXPAND_NONE") expand = H3DViewerTreeViewDialog::EXPAND_NONE;
        else if(mode == "EXPAND_ALL") expand = H3DViewerTreeViewDialog::EXPAND_ALL;
        else if(mode == "EXPAND_GROUP") expand = H3DViewerTreeViewDialog::EXPAND_GROUP;
      }
    }
  }

  string tree_string = getNodeLabel(n, container_field);
  // add an item for this node in the tree
  wxTreeItemId new_id = TreeViewTree->AppendItem(tree_id, wxString(tree_string.c_str(), wxConvUTF8));

  // add an entry for the tree_id-node pair
  node_map[new_id.m_pItem].reset(n);
  AUTOREF_DEBUG_NAME( node_map[new_id.m_pItem], "H3DViewerTreeViewDialog::node_map")

  bool expand_new_id = (expand == H3DViewerTreeViewDialog::EXPAND_ALL ||
                        (expand == H3DViewerTreeViewDialog::EXPAND_GROUP && (dynamic_cast<X3DGroupingNode *>(n) || dynamic_cast<Scene *>(n))));
  // recursively add all the child nodes of the node to the tree
  H3DNodeDatabase *db = H3DNodeDatabase::lookupNodeInstance(n);
  for(H3DNodeDatabase::FieldDBConstIterator i = db->fieldDBBegin();
  db->fieldDBEnd() != i; ++i) {
    Field *f = i.getField(n); //n->getField( *i );

    if(SFNode *sfnode = dynamic_cast<SFNode *>(f)) {
      if(sfnode->getAccessType() != Field::INPUT_ONLY) {
        addNodeToTree(new_id, sfnode->getValue(), sfnode->getName(), expand_new_id?expand:H3DViewerTreeViewDialog::EXPAND_NONE);
      }
    } else if(MFNode *mfnode = dynamic_cast<MFNode *>(f)) {
      if(mfnode->getAccessType() != Field::INPUT_ONLY) {
        for(MFNode::const_iterator i = mfnode->begin(); i != mfnode->end(); ++i) {
          addNodeToTree(new_id, *i, mfnode->getName(), expand_new_id?expand:H3DViewerTreeViewDialog::EXPAND_NONE);
        }
      }
    }
  }

  // make the tree be open down to leaves or not.
  if(expand_new_id) {
    TreeViewTree->Expand(new_id);
  }
}

const wxColour H3D_wxGRAY = wxColour(255, 140 , 140 );

// color to use in tree view for a node where the node and all of its children have
// been destructed.
#define H3D_COLOUR_REMOVED_BRANCH *wxLIGHT_GREY

// color to use in tree view for a node where the node is destructed but at least
// one of its children is still alive.
#define H3D_COLOUR_REMOVED_NODE_CHILDREN_ALIVE H3D_wxGRAY

bool H3DViewerTreeViewDialog::updateNodeTreeWhenUpdateStopped( wxTreeItemId tree_id ) {
  
  // true if this tree id is a tree node that represents an H3D.Node  
  bool is_h3dnode_container = node_map.find((tree_id).m_pItem) != node_map.end();
 
  // the h3d Node held by the tree_id 
  Node *h3d_node = NULL;

  // will be true if the node held by tree_id was set to NULL during this call
  bool tree_node_deleted = false;
  

  // remove reference to node if the reference count is less than the number of references 
  // held in tree view. Otherwise do normal update to the label. 
  if (is_h3dnode_container) {
    h3d_node = node_map[tree_id.m_pItem].get();
    if( h3d_node ) {
      unsigned int ref_count = h3d_node->getRefCount();
      if ( ref_count <= treeview_node_count_map[h3d_node]) {
        node_map[tree_id.m_pItem].reset(NULL);
        tree_node_deleted = true;
      } else {
        if (H3D::Profiling::profile_group_nodes || force_update_labels) {
          // we do not know container field any more so use default to make it not show up
          string node_label = getNodeLabel(h3d_node, h3d_node->defaultXMLContainerField());
          wxString wx_node_label(wxString(node_label.c_str(), wxConvUTF8) );

          //   check that text has changed in order to avoid flickering if not
          if (TreeViewTree->GetItemText(tree_id) != wx_node_label) {
            TreeViewTree->SetItemText(tree_id, wx_node_label);
          }
        }
      }
    }
  }
  

  // recurse through all children that are not already fully removed.
  wxTreeItemIdValue cookie;
  wxTreeItemId child_id = TreeViewTree->GetFirstChild(tree_id, cookie);
  bool all_children_deleted = true;

  while(child_id.IsOk()) {
    if(  TreeViewTree->GetItemTextColour( child_id ) != H3D_COLOUR_REMOVED_BRANCH ) {
      bool child_deleted =  updateNodeTreeWhenUpdateStopped( child_id );
      all_children_deleted = all_children_deleted && child_deleted;
    }

    child_id = TreeViewTree->GetNextSibling( child_id );
  }

 
  // if this is a tree id that is just a label, such as "Bindable nodes", ignore it.
  if( !is_h3dnode_container ) {
    return false;
  }

  // set color of the tree id 
  if( tree_node_deleted ) {
    if( all_children_deleted ) {
      TreeViewTree->SetItemTextColour(tree_id, H3D_COLOUR_REMOVED_BRANCH);
    } else {
      TreeViewTree->SetItemTextColour(tree_id, H3D_COLOUR_REMOVED_NODE_CHILDREN_ALIVE );
    }
  } else {
    if( all_children_deleted && !h3d_node ) {
      TreeViewTree->SetItemTextColour(tree_id, H3D_COLOUR_REMOVED_BRANCH);
    } 
  }
  
  return all_children_deleted && (!h3d_node || tree_node_deleted );
}

void H3DViewerTreeViewDialog::updateNodeTree(wxTreeItemId tree_id,
                                             list< pair< H3D::Node *, string > > nodes,
                                             H3DViewerTreeViewDialog::ExpandMode expand_new,
                                             bool check_if_expanded) {

  // find all children of tree_id
  list< wxTreeItemId > children_ids;
  wxTreeItemIdValue cookie;

  wxTreeItemId id = TreeViewTree->GetFirstChild(tree_id, cookie);
  if(check_if_expanded)
    if(id.IsOk() && TreeViewTree->HasChildren(tree_id) && !TreeViewTree->IsExpanded(tree_id)) {
      return;
    }
  while(id.IsOk()) {
    children_ids.push_back(id);
    id = TreeViewTree->GetNextSibling(id);
  }

  // update each of the tree ids that already exists. Either by updating
  // them if they still refer to a node or deleting them otherwise.
  for(list< wxTreeItemId >::iterator i = children_ids.begin();
  i != children_ids.end(); ++i) {
    // find the node corresponding to the id in the current tree view.

    if(node_map.find((*i).m_pItem) == node_map.end()) {
      continue;
    }

    Node *id_node = node_map[(*i).m_pItem].get();

    // check if this node still exists in the new node structure
    list< pair< H3D::Node *, string > >::iterator ni;
    for(ni = nodes.begin(); ni != nodes.end(); ++ni) {
      Node *node = (*ni).first;
      if(node->getProtoInstanceParent()) {
        node = node->getProtoInstanceParent();
      }
      if(node == id_node) {
#if defined( HAVE_PROFILER ) || defined( H3D_REFERENCE_COUNT_DEBUG )
        if(H3D::Profiling::profile_group_nodes || force_update_labels) {
          string node_label = getNodeLabel(node, (*ni).second);
          wxString wx_node_label( node_label.c_str());

          // check that text has changed in order to avoid flickering if not
          if( TreeViewTree->GetItemText(*i) != wx_node_label ) {
            TreeViewTree->SetItemText(*i, wx_node_label);
          }
        }
#endif
        break;
      }
    }

    if(ni != nodes.end()) {
      // the node the tree id refers to still exists on this level
      // so recurse down.

      // find all child nodes of the node
      list< pair< H3D::Node *, string > > child_nodes;
      H3DNodeDatabase *db = H3DNodeDatabase::lookupNodeInstance(id_node);
      for(H3DNodeDatabase::FieldDBConstIterator j = db->fieldDBBegin();
      db->fieldDBEnd() != j; ++j) {
        Field *f = j.getField(id_node); //Field *f = id_node->getField( *j );

        if(SFNode *sfnode = dynamic_cast<SFNode *>(f)) {
          if(sfnode->getAccessType() != Field::INPUT_ONLY) {
            Node *n = sfnode->getValue();
            if(n) child_nodes.push_back(make_pair(sfnode->getValue(), sfnode->getName()));
          }
        } else if(MFNode *mfnode = dynamic_cast<MFNode *>(f)) {
          if(mfnode->getAccessType() != Field::INPUT_ONLY) {
            for(MFNode::const_iterator mf = mfnode->begin();
            mf != mfnode->end(); ++mf) {
              if(*mf) child_nodes.push_back(make_pair(*mf, mfnode->getName()));
            }
          }
        }
      }
      // update
      updateNodeTree(*i, child_nodes, expand_new);
      nodes.erase(ni);
    } else {
      // the node does not exist, so remove the tree id and all its children.
      deleteTree(*i);
    }
  }

  // add all new nodes to the tree.
  for(list< pair< H3D::Node *, string > >::iterator i = nodes.begin();
  i != nodes.end(); ++i) {
    addNodeToTree(tree_id, (*i).first, (*i).second, expand_new);
  }

  // make the tree be open down to leaves by default.
  //TreeViewTree->Expand( tree_id );
}


void H3DViewerTreeViewDialog::deleteTree(const wxTreeItemId &id) {
  list< wxTreeItemId > children_ids;
  wxTreeItemIdValue cookie;
  wxTreeItemId child_id = TreeViewTree->GetFirstChild(id, cookie);
  while(child_id.IsOk()) {
    children_ids.push_back(child_id);
    child_id = TreeViewTree->GetNextSibling(child_id);
  }

  for(list< wxTreeItemId >::iterator i = children_ids.begin();
  i != children_ids.end(); ++i) {
    deleteTree(*i);
  }

  node_map.erase(id.m_pItem);
  TreeViewTree->Delete(id);
}

void H3DViewerTreeViewDialog::displayFieldsFromNode(Node *n) {
  field_values_panel->displayFieldsFromNode(n);
}

void H3DViewerTreeViewDialog::clearTreeView() {
  list< pair< Node *, string> > l;
  updateNodeTree(TreeViewTree->GetRootItem(), l);
  updateNodeTree(bindable_tree_id, l, H3DViewerTreeViewDialog::EXPAND_NONE, false);
  displayFieldsFromNode(NULL);
}

void H3DViewerTreeViewDialog::updateNodeCountMap( wxTreeItemId tree_id ) {

  wxTreeItemIdValue cookie;
  wxTreeItemId id = TreeViewTree->GetFirstChild(tree_id, cookie);


  while(id.IsOk()) {
    TreeIdMap::iterator ni = node_map.find(id.m_pItem); 
    if( ni != node_map.end() ) {
      treeview_node_count_map[(*ni).second.get()]++;
    }
    updateNodeCountMap(id);
    id = TreeViewTree->GetNextSibling(id);
  }
}

void H3DViewerTreeViewDialog::OnStopUpdatingCheckbox(wxCommandEvent& event) {
  bool checked = event.IsChecked();
  if (checked) {
    updateNodeCountMap( TreeViewTree->GetRootItem() );
  } else {
    treeview_node_count_map.clear();
  }
}

void H3DViewerTreeViewDialog::OnProfileCheckbox(wxCommandEvent& event) {
  bool checked = event.IsChecked();
  H3D::Profiling::profile_group_nodes = checked;
  force_update_labels = true;
}

void H3DViewerTreeViewDialog::OnIdle(wxIdleEvent& event) {
#ifdef H3D_REFERENCE_COUNT_DEBUG
  force_update_labels = true;
#endif
  try {
    if(IsShown()) {
      
      if( !this->stop_treeview_update_checkbox->IsChecked() ) {

        if(selected_node.get() != field_values_panel->getDisplayedNode()) {
          displayFieldsFromNode(selected_node.get());
        }

        field_values_panel->OnIdle(event);
      }
      TimeStamp now;
      if(now - last_tree_update > 1) {
        if(shown_last_loop) {
          if( this->stop_treeview_update_checkbox->IsChecked() ) {
            wxTreeItemId tree_root = TreeViewTree->GetRootItem();
            std::stringstream ss;
             ss << "World (nodes alive: " << H3D::Node::nrNodesAlive() << " nodes created: " << Node::nrNodesCreated() << ")";
            TreeViewTree->SetItemText(tree_root, wxString(ss.str().c_str(), wxConvUTF8));
            updateNodeTreeWhenUpdateStopped( tree_root );
          } else { 
            showEntireSceneAsTree(H3DViewerTreeViewDialog::EXPAND_NONE);
          }
        } else {
          showEntireSceneAsTree(H3DViewerTreeViewDialog::EXPAND_GROUP);
        }
        #ifdef HAVE_PROFILER
        force_update_labels = false;
        #endif
        last_tree_update = now;
      }
    } else if(shown_last_loop) {
      // make sure we do not hold any references to any nodes by clearing
      // it.
      clearTreeView();
    }

    shown_last_loop = IsShown();
  } catch(...) {
    // ignore any errors
  }
}

void H3DViewerTreeViewDialog::onCharHook(wxKeyEvent& event) {
  if(event.ControlDown()) {
    if(event.GetKeyCode() == 70 /*F*/) {
      // CTRL+F: Search/find
      search_text_ctrl->SetFocus();
      search_text_ctrl->SelectAll();
    }
  }

  // F3 to cycle through search results.
  if(event.GetKeyCode() == WXK_F3) {
    wxCommandEvent e;
    e.SetString(search_text_ctrl->GetValue());

    // Hold down shift to reverse cycle direction.
    if(event.ShiftDown()) {
      bool reverse_direction = true;
      e.SetClientData(&reverse_direction);
      onSearchTextCtrl(e);
    } else {
      bool reverse_direction = false;
      e.SetClientData(&reverse_direction);
      onSearchTextCtrl(e);
    }
  }

  event.Skip();
}

void H3DViewerTreeViewDialog::highlightSearchBox() {
  search_text_ctrl->SetFocus();
  search_text_ctrl->Clear();
}

void H3DViewerTreeViewDialog::expandTree(const wxTreeItemId &id) {
  if(id.IsOk()) {
    wxTreeItemIdValue cookie;
    wxTreeItemId child_id = TreeViewTree->GetFirstChild(id, cookie);
    while(child_id.IsOk()) {
      expandTree(child_id);
      child_id = TreeViewTree->GetNextSibling(child_id);
    }
    TreeViewTree->Expand(id);
  }
}

void H3DViewerTreeViewDialog::collapseTree(const wxTreeItemId &id) {
  if(id.IsOk()) {
    wxTreeItemIdValue cookie;
    wxTreeItemId child_id = TreeViewTree->GetFirstChild(id, cookie);
    while(child_id.IsOk()) {
      collapseTree(child_id);
      child_id = TreeViewTree->GetNextSibling(child_id);
    }
    TreeViewTree->Collapse(id);
  }
}


void H3DViewerTreeViewDialog::OnTreeRightClick(wxTreeEvent& event) {
  TreeViewTree->SelectItem(event.GetItem());
  TreeIdMap::iterator ni = node_map.find(event.GetItem().m_pItem);
  X3DGeometryNode *geom = NULL;
  X3DTextureNode * tex = NULL;

  if(ni != node_map.end()) {
    geom = dynamic_cast<X3DGeometryNode *>((*ni).second.get());
    tex = dynamic_cast<X3DTextureNode *>((*ni).second.get());
  }

  // You are not allowed to use a menu that is attached to a menubar as a popup menu in newer versions of wxwidgets.
  // So we detach the menu we are about to use from our menu_container. But we need the event handler in the menu_container
  // since otherwise we would have to regenerate our H3DViewerTreeViewdialog class and put all the event handling there.
  // So as a compromise we make sure to set the same event handler the menu previously had after we've detached it.
  if(geom) {
    if (menu_container->RightClickMenuGeometry->IsAttached()) {
      menu_container->RightClickMenuGeometry->Detach();
      menu_container->RightClickMenuGeometry->SetEventHandler(menu_container->GetEventHandler());
    }
    PopupMenu(menu_container->RightClickMenuGeometry);
  }
  else if(tex) {
    if (menu_container->RightClickMenuTexture->IsAttached()) {
      menu_container->RightClickMenuTexture->Detach();
      menu_container->RightClickMenuTexture->SetEventHandler(menu_container->GetEventHandler());
    }
    PopupMenu(menu_container->RightClickMenuTexture);
  }
  else {
    if (menu_container->RightClickMenu->IsAttached()) {
      menu_container->RightClickMenu->Detach();
      menu_container->RightClickMenu->SetEventHandler(menu_container->GetEventHandler());
    }
    PopupMenu(menu_container->RightClickMenu);
  }
}


void H3DViewerTreeViewDialog::OnClose(wxCloseEvent& event) {
  Hide();
}

void H3DViewerTreeViewDialog::btnCloseClick(wxCommandEvent& event) {
  Hide();
}

void H3DViewerTreeViewDialog::collectAllTriangles(Node *n,
                                                  const Matrix4f &transform,
                                                  vector< Vec3f > &triangles) {

  if(!n) return;


  if(X3DShapeNode *shape = dynamic_cast<X3DShapeNode *>(n)) {
    X3DGeometryNode *geom = shape->geometry->getValue();
    if(geom) {
      vector< HAPI::Collision::Triangle > tris;
      geom->boundTree->getValue()->getAllTriangles(tris);
      for(unsigned int i = 0; i < tris.size(); ++i) {
        triangles.push_back(transform * (Vec3f)tris[i].a);
        triangles.push_back(transform * (Vec3f)tris[i].b);
        triangles.push_back(transform * (Vec3f)tris[i].c);
      }
    }
  }

  MatrixTransform *t = dynamic_cast<MatrixTransform *>(n);
  Matrix4f new_transform = t?t->matrix->getValue() * transform:transform;

  H3DNodeDatabase *db = H3DNodeDatabase::lookupNodeInstance(n);
  for(H3DNodeDatabase::FieldDBConstIterator i = db->fieldDBBegin();
  db->fieldDBEnd() != i; ++i) {
    Field *f = i.getField(n);
    if(SFNode *sfnode = dynamic_cast<SFNode *>(f)) {
      collectAllTriangles(sfnode->getValue(), new_transform, triangles);
    } else if(MFNode *mfnode = dynamic_cast<MFNode *>(f)) {
      for(unsigned int j = 0; j < mfnode->size(); ++j) {
        Node *n = mfnode->getValueByIndex(j);
        collectAllTriangles(n, new_transform, triangles);
      }
    }
  }
}

BEGIN_EVENT_TABLE(wxImagePanel, wxPanel)
EVT_PAINT(wxImagePanel::paintEvent)
END_EVENT_TABLE()

wxImagePanel::wxImagePanel(wxWindow* parent) : wxPanel(parent) {
}

void wxImagePanel::paintEvent(wxPaintEvent & evt) {
  wxPaintDC dc(this);
  dc.DrawBitmap(image, 0, 0, false);
}

void wxImagePanel::setImage(const wxImage& _image) {
  image = wxBitmap(_image);

  wxSize s(image.GetWidth(), image.GetHeight());
  SetMinSize(s);
  SetMaxSize(s);
}

H3DViewImage::H3DViewImage(wxWindow* parent, X3DTextureNode& _texture)
  :
  image_data(NULL),
  ViewImage(parent) {
  draw_pane = new wxImagePanel(m_imagePanel);
  m_imagePanel->GetSizer()->Add(draw_pane, 1, wxEXPAND);
  
  AUTOREF_DEBUG_NAME( texture, "H3DViewImage::texture")

  texture.reset(&_texture);
  updateImage();

  stringstream s;
  s << "View: " << _texture.getName();
  SetTitle(wxString(s.str().c_str(), wxConvUTF8));
}

H3DViewImage::~H3DViewImage() {
  if(image_data) {
    delete[] image_data;
    image_data = 0;
  }
}

void H3DViewImage::OnSave(wxCommandEvent& event) {
  #ifdef HAVE_FREEIMAGE
  auto_ptr< wxFileDialog > file_dialog(new wxFileDialog(this,
                                                        wxT("File to save as.."),
                                                        wxT(""),
                                                        wxT(""),
                                                        wxT("*.png"),
                                                        wxFD_SAVE,
                                                        wxDefaultPosition));

  if(file_dialog->ShowModal() == wxID_OK) {
    try {
      Image* image = texture->renderToImage(-1, -1);
      if(!image) {
        wxMessageBox(wxT("Failed to render texture to image!"), wxT("Error"),
                     wxOK | wxICON_EXCLAMATION);
        return;
      }

      if(!H3DUtil::saveFreeImagePNG(string(file_dialog->GetPath().mb_str()),
                                    *image)) {
        stringstream s;
        s << "Error saving png file";
        wxMessageBox(wxString(s.str().c_str(), wxConvUTF8), wxT("Error"),
                     wxOK | wxICON_EXCLAMATION);
      }
    } catch(const Exception::H3DException &e) {
      stringstream s;
      s << e;
      wxMessageBox(wxString(s.str().c_str(), wxConvUTF8), wxT("Error"),
                   wxOK | wxICON_EXCLAMATION);
    }
  }
  #endif
}

void H3DViewImage::OnRefresh(wxCommandEvent& event) {
  updateImage();
  draw_pane->Refresh();
}

void H3DViewImage::OnAutoRefresh(wxCommandEvent& event) {
  if(event.IsChecked()) {
    m_timerRefresh.Start(2000);
  } else {
    m_timerRefresh.Stop();
  }
}

void H3DViewImage::OnTimer(wxTimerEvent& event) {
  updateImage();
  draw_pane->Refresh();
}

void H3DViewImage::updateImage() {
  auto_ptr < Image > image(texture->renderToImage(-1, -1));
  if(!image.get()) {
    Console(LogLevel::Error) << "ERROR: Failed to render texture to image!" << endl;
    return;
  }

  unsigned char* rgb = new unsigned char[image->width()*image->height() * 3];

  size_t offset = 0;
  for(size_t y = 0; y < image->height(); ++y) {
    for(size_t x = 0; x < image->width(); ++x) {
      RGBA rgba = image->getPixel(x, image->height() - 1 - y);

        if( m_checkBoxChannelAlpha->GetValue() ) {
          rgb[offset++] = (unsigned char)(rgba.a * 255);
          rgb[offset++] = (unsigned char)(rgba.a * 255);
          rgb[offset++] = (unsigned char)(rgba.a * 255);
        } else {
        rgb[offset++]= m_checkBoxChannelRed->GetValue()   ? (unsigned char)(rgba.r*255) : 0;
        rgb[offset++]= m_checkBoxChannelGreen->GetValue() ? (unsigned char)(rgba.g*255) : 0;
        rgb[offset++]= m_checkBoxChannelBlue->GetValue()  ? (unsigned char)(rgba.b*255) : 0;
        }
    }
  }

  draw_pane->setImage(wxImage(image->width(), image->height(), rgb, true));
  Layout();

  delete[] image_data;
  image_data = rgb;
}
