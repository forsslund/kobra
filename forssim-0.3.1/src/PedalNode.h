/**
 * PedalNode.h
 *
 *  Created on: Apr 4, 2010
 *      Author: martin & jonas & auno
 */

#ifndef PEDALNODE_H_
#define PEDALNODE_H_

#include <H3D/X3DChildNode.h>
#include <H3D/SFString.h>
#include <H3D/SFBool.h>

using namespace std;
using namespace H3D;

namespace FS
{

/**
 * Concerned with the Pedal of data to file
 */
class PedalNode : public H3D::X3DChildNode
{
public:
	PedalNode(H3D::Inst< H3D::SFBool  > pedal_0 = 0,
                  H3D::Inst< H3D::SFBool  > pedal_1 = 0,
                  H3D::Inst< H3D::SFBool  > pedal_2 = 0,
                  H3D::Inst< H3D::SFBool  > noCombo = 0);
	virtual ~PedalNode();

	/**
	 * Initialize the Pedal Node
	 */
	void initialize();

	/**
	 * The H3DNodeDatabase for this node
	 */
	static H3D::H3DNodeDatabase database;

	virtual void traverseSG( H3D::TraverseInfo &ti );

        int fd;

	/**
	 * The location to where logs are stored for a particular user
	 */
	auto_ptr< H3D::SFBool > 	pedal_0;
	auto_ptr< H3D::SFBool > 	pedal_1;
	auto_ptr< H3D::SFBool > 	pedal_2;
    auto_ptr< H3D::SFBool > 	noCombo;
};
}
#endif /* PEDALNODE_H_ */
