/**
 * FloatSwitch.cpp
 *
 *  Created on: Aug 31, 2016
 *      Author: Jonas
 */

#include "FloatSwitch.h"

using namespace H3D;


/// Add this node to the H3DNodeDatabase system.
H3DNodeDatabase FloatSwitch::database("FloatSwitch",
                                      &(newInstance<FloatSwitch>),
                                      typeid( FloatSwitch ),
                                      &X3DChildNode::database );

namespace FloatSwitchInternals
{
FIELDDB_ELEMENT( FloatSwitch, select_a, INPUT_OUTPUT );
FIELDDB_ELEMENT( FloatSwitch, value_a, INPUT_OUTPUT );
FIELDDB_ELEMENT( FloatSwitch, value_b, INPUT_OUTPUT );
FIELDDB_ELEMENT( FloatSwitch, out, INPUT_OUTPUT );
FIELDDB_ELEMENT( FloatSwitch, out_int, INPUT_OUTPUT );
}


FloatSwitch::FloatSwitch(Inst< SFNode       > _metadata,
                         Inst< SFBoolParent  > select_a,
                         Inst< SFFloat  > value_a,
                         Inst< SFFloat  > value_b,
                         Inst< SFFloat > out,
                         Inst< SFInt32 > out_int)
  :  X3DChildNode( _metadata ),
    select_a(select_a),
    value_a(value_a),
    value_b(value_b),
    out(out), out_int(out_int)
{
  cout << "FloatSwitch Constructing"<<endl;

  type_name = "FloatSwitch";
  database.initFields( this );


  cout << "FloatSwitch Constructed"<<endl;
}

