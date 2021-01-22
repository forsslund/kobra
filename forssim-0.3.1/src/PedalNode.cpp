 /**
 * PedalNode.cpp
 *
 *  Created on: Apr 4, 2010
 *      Author: martin & jonas & auno
 */

#include "PedalNode.h"
#include <stdio.h>
#include <fcntl.h>
#ifdef LINUX
#include <unistd.h>
#endif

struct hiddev_event {
  unsigned hid;
  signed int value;
};

using namespace FS;

/// Add this node to the H3DNodeDatabase system.
H3D::H3DNodeDatabase PedalNode::database("PedalNode",
		&(newInstance<PedalNode>),
		typeid( PedalNode ),
		&X3DChildNode::database );

namespace PedalNodeInternals
{
	H3D::FIELDDB_ELEMENT( PedalNode, pedal_0, INPUT_OUTPUT );
	H3D::FIELDDB_ELEMENT( PedalNode, pedal_1, INPUT_OUTPUT );
	H3D::FIELDDB_ELEMENT( PedalNode, pedal_2, INPUT_OUTPUT );
    H3D::FIELDDB_ELEMENT( PedalNode, noCombo, INPUT_OUTPUT );
}

PedalNode::PedalNode(H3D::Inst< H3D::SFBool  > pedal_0,
                     H3D::Inst< H3D::SFBool  > pedal_1,
             H3D::Inst< H3D::SFBool  > pedal_2,
             H3D::Inst< H3D::SFBool  > _noCombo                     )
:pedal_0(pedal_0),
pedal_1(pedal_1),
pedal_2(pedal_2),
  noCombo(_noCombo), fd(0)
{
	cout << "Pedal Node Constructing"<<endl;

	type_name = "PedalNode";
	database.initFields( this );

    noCombo->setValue(true);

	cout << "Pedal Node Constructed"<<endl;
}

PedalNode::~PedalNode()
{
	cout<<"Pedal Node Destructed"<<endl;
}

void PedalNode::traverseSG( H3D::TraverseInfo&){

  int state[] = { 0, 0, 0 };
  bool foundMessage = false;

  //std::cout << "Pedals: " << pedal_0->getValue() << ", " << pedal_1->getValue() << ", " << pedal_2->getValue() << "\n";

#ifdef LINUX
  struct hiddev_event event;

    for (int i = 0; i < 3; i++) {

        int rcount = read(fd, &event, sizeof(struct hiddev_event));

        while (rcount > 0 && rcount!=sizeof(struct hiddev_event)){
          rcount += read(fd, &event+rcount, sizeof(struct hiddev_event)-rcount);
        }


        if (rcount > 0 && rcount==sizeof(struct hiddev_event)) {
            if(event.hid == 589899){
                // New protocol
                if(i==0){
                    state[0] = (event.value & 1);
                    state[1] = (event.value & 2)>>1;
                    state[2] = (event.value & 4)>>2;
                    foundMessage = true;
                }
                else {
                    i=100; // only 2 messages, the last being always 0
                }
            } else {
                // Old protocol
                state[(event.hid & 0xFF)-1] = event.value;
                foundMessage = true;
            }
        }

    }
#endif

    // Return already if no message, useful if no pedal is hooked up
    // and we instead control the pedal states virtually (through route in)
    if(!foundMessage) return;



    // New ordering of pedals 2019-04-27. Middle is first
    if(state[1] == 1){ // Priority middle pedal controling drill
        pedal_0->setValue(true);
        pedal_1->setValue(false);
        pedal_2->setValue(false);
        return;
    }
    else
        pedal_0->setValue(false);


    // Check for multiple pedals activated
    if(noCombo->getValue()){
        int num_pedals_on = 0;
        for(int i=0;i<3;++i){
            num_pedals_on += state[i];
        }
        if(num_pedals_on>1) return;
    }

    // Left pedal (usually controlling moving around)
    if(state[0] == 1)
        pedal_1->setValue(true);
    else
        pedal_1->setValue(false);

    // Right pedal (usually controlling elevator)
    if(state[2] == 1)
        pedal_2->setValue(true);
    else
        pedal_2->setValue(false);
}

void PedalNode::initialize()
{
	cout<<"Pedal Node Initializing"<<endl;
	pedal_0->setValue(false);
	pedal_1->setValue(false);
	pedal_2->setValue(false);

#ifdef LINUX
	fd = open("/dev/infinity-in-usb-2", O_NONBLOCK);
#endif 

	cout<<"Pedal Node Initialized"<<endl;
}
