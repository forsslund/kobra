# (c) SenseGraphics 2010. Written for MOOG
# 17.06.2010
# Convert from IndexedFaceSet to IndexedTriangleSet for those IndexedFaceSets
# that are generated by other programs and have only 3-vertex polygons.
# The program will not work if the IndexedFaceSet contains a polygon with more than
# 3 vertices.
# Syntax: h3dload ifs2its.x3d [source_file] [destination_file]
# Demo: h3dload ifs2its.x3d face.x3d face_converted.x3d

from H3DInterface import *

import itertools
import operator
import random

# Generate normals for a ITS. return a list contains normals
# This is a python-converted of the function
# IndexedFaceSet::AutoNormal::generateNormalsPerVertex
def generateNormalsPerVertex(ITS):
  coords = ITS.coord.getValue().point.getValue()
  indexes = ITS.index.getValue()
  ccw = ITS.ccw.getValue()
  normals = [Vec3f(0, 0, 0)] * len(coords)
  
  for j in range(0, len(indexes), 3):
    if j+2 >= len(indexes):
      norm = Vec3f(1, 0, 0)
    else:
      A = coords[indexes[j]]
      B = coords[indexes[j+1]]
      C = coords[indexes[j+2]]
      AB = B - A
      BC = C - B
      norm = AB % BC
      norm.normalizeSafe()
      if not ccw: norm = - norm
      normals[indexes[ j ]] += norm;
      normals[indexes[ j+1 ]] += norm;
      normals[indexes[ j+2 ]] += norm;
  for normal in normals:
    normal.normalizeSafe()

  return normals

# Calculates information about which vertices are shared.
# Input is a list of indices and the maximum index value in 
# the list.
# Returns a list of size max_index where each position contains
# a list of the indices into the index_list that uses that index. 
#
# Example: generateSharedVertexInfo( [0 1 2 1 2 3], 3 )
# will return [ [0], [1,3], [2,4], [5] ]
def generateSharedVertexInfo( index_list, max_index ):
  info_list = [ [] for i in range( max_index+1) ]
  for i, index in enumerate(index_list):
    if( index != -1 ):
      info_list[index].append( i )
  return info_list

# Calculates which vertex indices share values in a list of
# VertexData instances.
#
# input is a list of all VertexData instance to check.
# index_list_in is a list of indices to check if they are shared. 
#
# Returns a list of lists. Each list in the return list is a subset
# of index_list_in where the vertex values are shared in all VertexData
# instances given in vertex_data_sets. 
def generateEqualValueList( vertex_data_sets,
                            index_list_in,
                            equal_based_on_index = True ):
  # take a copy of the input index list
  index_list = []
  index_list.extend( index_list_in )

  return_list = []

  while( len( index_list ) > 0 ): 
    # each loop we use the first index and identify which 
    # of the indices following it is shared in the same way
    # in all VertexData sets. All indices that are shared are
    # then removed and we do it all over on the rest of the 
    # indices that were not shared
    v = index_list[0]
    index_list.pop(0)

    shared = [v]

    # add indices to be removed to this container. All values in index_list
    # that are equal to v for all vertex data sets will be added
    to_remove = []

    #
    for i, index in enumerate(index_list):
      is_shared_by_all = True
      for vertex_data in vertex_data_sets:
        if( (equal_based_on_index and vertex_data.index[v] != vertex_data.index[index] ) or
            (not(equal_based_on_index) and vertex_data.values[vertex_data.index[v]] != vertex_data.values[vertex_data.index[index] ] )):
          is_shared_by_all = False
          continue

      # index is an index that is shared.
      if( is_shared_by_all ):
        shared.append( index )
        to_remove.append( i )

    # remove all used indices from the index_list
    to_remove.reverse()
    for remove_i in to_remove:
      index_list.pop( remove_i )      

    return_list.append( shared)
  return return_list

# 2 (or more) points with same coordinate will have different
# normals since they connect to different triangles
# This function merge their normals together
def fixNormals(ITS):
  coords = ITS.coord.getValue().point.getValue()
  coords_visited = [0] * len(coords)
  normal_node = ITS.normal.getValue()
  normals = normal_node.vector.getValue()
  
  # this will take O(n^2) but it's modular (separate from the main prog)
  for i in range(len(coords)):
    if coords_visited[i]: continue
    coords_visited[i] = 1
    normal_sum = normals[i]
    adjacent_ids = [i] # list of coords that need to set to the same normal
    for j in range(i + 1, len(coords)):
      if (coords[i] - coords[j]).length() < 0.001:
        adjacent_ids += [j]
        normal_sum = normal_sum + normals[j]
        coords_visited[j] = 1
    for id in adjacent_ids:
      normals[id] = normal_sum
      normals[id].normalizeSafe()
  
  normal_node.vector.setValue(normals)


class VertexData:
  def __init__( self, field_name, index, values ):
    self.field_name = field_name
    self.index = index
    self.values = values
    self.new_index = []
    self.new_values = []

  def matchIndex( self, main_index_list, max_index ):
    self.new_index = []
    self.new_values = [self.values[0]] * (max_index + 1)
    self.new_index.extend( main_index_list )
    for i, index in enumerate(main_index_list ):
      self.new_values[index] = self.values[ self.index[i] ]

  def splitPoints( self, split_info ):
    if( len(split_info) < 2 ): return
#    print "Split ", split_info, " ", self.field_name     
    for index in split_info[0]:
      self.new_values[ self.new_index[index] ] = self.values[ self.index[index] ]
    for split_indices in split_info[1:] :
      new_index = len( self.new_values )
      self.new_values.append( self.values[ self.index[split_indices[0] ]] )
      for index in split_indices:
        self.new_index[ index ] = new_index

# Takes a IndexedFaceSet index and creates a IndexedTriangleIndex from it by
# removing -1 and triangulating quads if present. Polygons with more vertices 
# than 4 are not supported.
def triangulateIndex( indices ):
  tri_indices = []
  face_indices = []

  doing_triangulation = False
  
  # add a -1 at the end in order to ensure that the last face is handles
  # in the for loop below.
  indices.append( -1 )

  for i in indices:
    if( i == -1 ):
      if( len(face_indices) == 3 ):
        # face is triangle
        tri_indices.extend( face_indices )
      elif( len(face_indices) == 4 ):
        # face is quad
        if( not( doing_triangulation ) ):
          print "IndexedFaceSet contains quads. Creating triangulation.."
          doing_triangulation = True
        tri_indices.extend( [face_indices[0], face_indices[1], face_indices[2],
                             face_indices[0], face_indices[2], face_indices[3] ] )
        
      elif( len(face_indices) > 4 ):
        print "Unsupported face type: ", len( face_indices ), " vertices."
      face_indices = []
    else:
      face_indices.append( i )
  return tri_indices

def ifs2its(IFS):
  """
    IFS: IndexedFaceSet
    Return: IndexedTriangleSet string
  """

  COORD = IFS.coord.getValue()
  TEX_COORD = IFS.texCoord.getValue()
  NORMAL = IFS.normal.getValue()
  COLOR = IFS.color.getValue()

  coord_idxs    = triangulateIndex(IFS.coordIndex.getValue())
  coord_points  = COORD.point.getValue()
  
  share_info_coord = generateSharedVertexInfo( coord_idxs, len( coord_points ) -1)

  vertex_data_sets = []

  if( COORD ):
    print "Have coordinates"
    vd = VertexData( "coord",
                     coord_idxs, 
                     coord_points )
    vd.matchIndex( coord_idxs, len( coord_points ) - 1)
    vertex_data_sets.append( vd )
    
  if( TEX_COORD ):
    message= "Have texture coordinates"
    index = IFS.texCoordIndex.getValue()
    if( len(index) == 0 ): index = coord_idxs
    else: message += " and separate texCoordIndex."
    vd = VertexData( "texCoord", 
                     triangulateIndex( index),
                     TEX_COORD.point.getValue() )
    vd.matchIndex( coord_idxs, len( coord_points ) - 1)
    vertex_data_sets.append( vd )
    print message

  if( COLOR ):
    message = "Have color"
    index = IFS.colorIndex.getValue()
    if( len(index) == 0 ): index = coord_idxs
    else: message += " and separate colorIndex."
    vd = VertexData( "color",
                     triangulateIndex( index),
                     COLOR.color.getValue() ) 
    vd.matchIndex( coord_idxs, len( coord_points ) - 1)
    vertex_data_sets.append( vd )
    print message

  if( NORMAL ):
    message = "Have normals"
    index = IFS.normalIndex.getValue()
    if( len(index) == 0 ): index = coord_idxs
    else: message += " and separate normalIndex."
    vd = VertexData( "normal",
                     triangulateIndex( index),
                     NORMAL.vector.getValue() )
    vd.matchIndex( coord_idxs, len( coord_points ) - 1)
    vertex_data_sets.append( vd )
    print message

#  print "Shared info: ", share_info_coord
                             
  for coord_i, shared_indices in enumerate( share_info_coord ):
    # the number of indices in coordIndex that share this coordinate.
    nr_shared_points = len( shared_indices)

    if( nr_shared_points >= 2 ):
      # create a list of lists of indices that have the same value in all
      # VertexData sets.
      equal_value_indices_list = generateEqualValueList( vertex_data_sets,
                                                         shared_indices,
                                                         False )
#      print "Equal values: ", equal_value_indices_list
      for vertex_data in vertex_data_sets:
        vertex_data.splitPoints( equal_value_indices_list )
      

  # count number of new points (statistics purpose)
  count_newpoints = 0

  #if( len(tex_idxs) < len(coord_idxs) ): 
  #  print "Invalid IndexedFaceSet: not enough texCoordIndex values"
  #  return IFS


  its = createX3DNodeFromString("<IndexedTriangleSet />")[0]
  ifs_name = IFS.getName() 
  if( ifs_name != "Unnamed IndexedFaceSet" ):
    its.setName( ifs_name )

  for vertex_data in vertex_data_sets:
    if( vertex_data.field_name == "coord" ):
      coord_node = createX3DNodeFromString("<Coordinate />")[0]
      coord_node.point.setValue(vertex_data.new_values)
      its.coord.setValue(coord_node)
      points_added = len(vertex_data.new_values) - len( vertex_data.values)
      its.index.setValue(vertex_data.new_index)
      if( points_added > 0 ):
        print '%i coordinate points added. Original: %i points.' % (points_added, len( vertex_data.values ))
    elif( vertex_data.field_name == "texCoord" ):
      texcoord_node = createX3DNodeFromString("<TextureCoordinate />")[0]
      texcoord_node.point.setValue(vertex_data.new_values)
      its.texCoord.setValue(texcoord_node)
      points_added = len(vertex_data.new_values) - len( vertex_data.values)
      if( points_added > 0 ):
        print '%i texture coordinate points added. Original: %i points.' % (points_added, len( vertex_data.values ))
      elif( points_added < 0 ):
        print '%i texture coordinate points removed. Original: %i points.' % (-points_added, len( vertex_data.values ))

    elif( vertex_data.field_name == "normal" ):
      normal_node = createX3DNodeFromString("<Normal />")[0]
      normal_node.vector.setValue(vertex_data.new_values)
      its.normal.setValue(normal_node)
      points_added = len(vertex_data.new_values) - len( vertex_data.values)
      if( points_added > 0 ):
        print '%i normal vectors added. Original: %i vectors.' % (points_added, len( vertex_data.values ))
      elif( points_added < 0 ):
        print '%i normal vectors points removed. Original: %i vectors.' % (-points_added, len( vertex_data.values ))
    elif( vertex_data.field_name == "color" ):
      color_node = createX3DNodeFromString("<Color />")[0]
      color_node.color.setValue(vertex_data.new_values)
      its.color.setValue(color_node)
      points_added = len(vertex_data.new_values) - len( vertex_data.values)
      if( points_added > 0 ):
        print '%i color values added. Original: %i values.' % (points_added, len( vertex_data.values ))
      elif( points_added < 0 ):
        print '%i color values points removed. Original: %i values.' % (-points_added, len( vertex_data.values ))  

  if( not( its.normal.getValue() ) ):
    normals = generateNormalsPerVertex(its)
    normalnode = createX3DNodeFromString("<Normal />")[0]
    normalnode.vector.setValue(normals)
    its.normal.setValue(normalnode)
  
    #fixNormals(its)


  return writeNodeAsX3D(its)
