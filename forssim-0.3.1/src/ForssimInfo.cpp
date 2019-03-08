/**
 * PedalNode.h
 *
 *  Created on: Apr 4, 2010
 *      Author: martin & jonas & auno
 */

#ifndef FORSSIMINFO
#define FORSSIMINFO

#include <H3D/X3DChildNode.h>
#include <H3D/SFString.h>

using namespace std;
using namespace H3D;

namespace FS
{

    /**
     * Version information etc, and used to tell other applications H3D
     * is up and running (couts fps...)
     */
    class ForssimInfo : public X3DChildNode
    {
    public:
        ForssimInfo(Inst< SFString> textout = 0,Inst<SFString> current_state = 0):
            textout(textout),current_state(current_state),frame_count(0){
            cout << "\n************************************\n";
            cout <<   "*  Welcome to Forssim 0.3.1        *\n";
            //         *  Build: Sep  1 2016 01:54:32     *"
            cout <<   "*  Build: " << __DATE__ << " " __TIME__ << "     *\n";
            cout <<   "************************************\n\n";
            cout << "fps: " << endl; // This is to let know we are alive
            type_name = "ForssimInfo";
            database.initFields( this );
            this->current_state->setValue("0");
        }

        static H3DNodeDatabase database;
        virtual void traverseSG( TraverseInfo &ti );
        auto_ptr< H3D::SFString > textout;
        auto_ptr< H3D::SFString > current_state;
        TimeStamp ts;
        int frame_count;
    };
}

using namespace FS;

/// Add this node to the H3DNodeDatabase system.
H3D::H3DNodeDatabase ForssimInfo::database("ForssimInfo",
        &(newInstance<ForssimInfo>),
        typeid( ForssimInfo ),
        &X3DChildNode::database );

namespace ForssimInfoInternals
{
    H3D::FIELDDB_ELEMENT( ForssimInfo, textout, INPUT_OUTPUT );
    H3D::FIELDDB_ELEMENT( ForssimInfo, current_state, INPUT_OUTPUT );
}

void ForssimInfo::traverseSG( H3D::TraverseInfo&){
    TimeStamp now;
    frame_count++;
    if(now-ts > 4){
        cout << "fps: " << frame_count/4 << endl;
        ts=now;
        frame_count=0;

        cout << "state: " << current_state->getValue() << "\n\n";
    }
}

#endif /* FORSSIMINFO */
