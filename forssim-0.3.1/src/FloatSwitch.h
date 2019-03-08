/**
 * FloatSwitch.cpp (skipped having a .h file)
 *
 *  Created on: Aug 31, 2016
 *      Author: Jonas
 */
#ifndef FLOATSWITCH_H_
#define FLOATSWITCH_H_

#include <H3D/X3DChildNode.h>
#include <H3D/SFBool.h>
#include <H3D/SFFloat.h>
#include <H3D/SFInt32.h>

namespace H3D
{

class FloatSwitch : public X3DChildNode
{
public:

  class SFBoolParent:  public AutoUpdate< SFBool >{
  protected:
    virtual void update(){
      SFBool::update();
      FloatSwitch *fs = static_cast<FloatSwitch*>(getOwner());
      float fval = value? fs->value_a->getValue() : fs->value_b->getValue();
      fs->out->setValue(fval);
      fs->out_int->setValue(int(fval));
    }
  };

  FloatSwitch(Inst< SFNode       > _metadata    = 0,
              Inst< SFBoolParent > select_a = 0,
              Inst< SFFloat > value_a = 0,
              Inst< SFFloat > value_b = 0,
              Inst< SFFloat > out = 0,
              Inst< SFInt32 > out_int = 0);

  auto_ptr< SFBoolParent  > 	select_a;
  auto_ptr< SFFloat > 	value_a;
  auto_ptr< SFFloat > 	value_b;
  auto_ptr< SFFloat > 	out;
  auto_ptr< SFInt32> out_int;

  /**
     * The H3DNodeDatabase for this node
     */
  static H3D::H3DNodeDatabase database;
};

}

#endif /* FLOATSWITCH_H_ */
