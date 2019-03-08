from H3DInterface import *
import SoftBodyUtility

indexed_hexa_set, = references.getValue()

tmp_hexa_set = SoftBodyUtility.regularHexaVolume( Vec3f( 0, 0, 0), Vec3f( 4, 4, 4 ), 4, 4, 4 )
indexed_hexa_set.coord.setValue( tmp_hexa_set.coord.getValue() )
indexed_hexa_set.index.setValue( tmp_hexa_set.index.getValue() )