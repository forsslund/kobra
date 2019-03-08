//////////////////////////////////////////////////////////////////////////////
//    Copyright 2012 Jonas Forsslund, Sonny Chan
//
//    CDrillForce is free software; you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation; either version 2 of the License, or
//    (at your option) any later version.
//
//    CDrillForce is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with ADrillForce; if not, write to the Free Software
//    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//
//    If you wish to use the code in a proprietary project, please contact
//    jonas@forsslundsystems.se for more information.
//
//////////////////////////////////////////////////////////////////////////////

#include "CHapticDrillForce.h"
#include <H3D/X3DGroupingNode.h>
#include <H3D/Transform.h>
#include <H3D/Shape.h>
#include <H3D/Sphere.h>
#include <H3D/Material.h>
#include <H3D/Appearance.h>
#include <H3D/SFColor.h>
#include "IntervalCollisionDetector.h"
#include "WilhelmsenProjection.h"
#include "Eigen/Cholesky"
#include "Eigen/LU"

using namespace FS;

CHapticDrillForce::CHapticDrillForce(const HAPI::Matrix4 & _transform,
        VolumeModel *_volume_model,
        double _bitRadius,
        CHapticContext *_context):
        HAPIForceEffect( ),
        transform( _transform),
        volume_model( _volume_model),
        bitRadius( _bitRadius),
        context( _context){


    computeMassMatrix(&(context->m_pointShell));
}

CHapticDrillForce::~CHapticDrillForce()
{
	//Need to delete generated_force which is allocated in ADrillForce::traverseSG
    //if( generated_force )
    //	delete generated_force;
}

CHapticDrillForce::EffectOutput CHapticDrillForce::calculateForces( const EffectInput &input )
{
    H3D::TimeStamp start;

    HAPI::Vec3 f_output;
    f_output.x = 0;
    f_output.y = 0;
    f_output.z = 0;



    // check for null
    //if(!context->proxyTransform)
    //    return EffectOutput(       f_output     );

    /***** Moved to CDrillForce.cpp
    // =========================================================================
    // Create some pointshellzz
    // Access the debugsphere childs
    // =========================================================================
    std::vector<cml::vector3d>  m_pointShell;
    NodeVector theChildren = context->proxyTransform->children->getValue();

    for( MFNode::const_iterator i = theChildren.begin();
         i != theChildren.end(); i++ ) {

        // We have a group arround a transform since X3D::Create... makes a group
        Group *g = dynamic_cast< Group * >( (*i) );
        if(!g)
            continue;

        Transform *t = dynamic_cast< Transform * >( g->children->getValue()[0] );
        if(t){
            HAPI::Vec3 p = t->translation->getValue();

            // add to our list
            m_pointShell.push_back(cml::vector3d(p.x,p.y,p.z));
        }
    }
    // =========================================================================
    computeMassMatrix(&m_pointShell);
    **********/





    // Get pos & rot from candidate proxy, in GLOBAL SPACE
    HAPI::Vec3 posLocalSpace = transform*
            context->proxyPosition;
            //(context->proxyTransform->translation->getValue());
    HAPI::Rotation r(H3DUtil::ArithmeticTypes::Matrix3d(HAPI::Rotation(transform.getRotationPart())) *
            context->proxyRotation);
            //context->proxyTransform->rotation->getValue();

    MyCollisionOracle *myCollisionOracle = new MyCollisionOracle(volume_model,
                                                                 r,
                                                                 posLocalSpace);



    // Move candidate proxy to be in between proxy and haptic device
    HAPI::Vec3 proxyPos = context->proxyPosition;//context->proxyTransform->translation->getValue();
    HAPI::Rotation proxyRot(context->proxyRotation); //= context->proxyTransform->rotation->getValue();
    proxyRot.axis.normalizeSafe();



    // Note: we have to use rotation matrixes, rotation of rotation is otherwise
    // slightly undefined (gives singularities)
    HAPI::Vec3 devicePos = input.hd->getPosition();
    HAPI::Rotation deviceRot = input.hd->getOrientation();
    HAPI::Matrix3 proxyRotMatrix = HAPI::Matrix3(proxyRot);


    HAPI::Matrix3 inverseProxyRot = proxyRotMatrix.transpose();
    HAPI::Rotation deltaRot =    HAPI::Rotation(HAPI::Matrix3(deviceRot) * inverseProxyRot);


    // Create a collision detector!
    HAPI::Vec3 a_hapi = HAPI::Vec3(-proxyRot * (devicePos - proxyPos));
    HAPI::Vec3 alpha_hapi = HAPI::Vec3(deltaRot.axis * deltaRot.angle);
    cml::vector3d a = cml::vector3d(a_hapi.x,a_hapi.y,a_hapi.z);
    cml::vector3d alpha = cml::vector3d(alpha_hapi.x,alpha_hapi.y,alpha_hapi.z);

    IntervalCollisionDetector checker(context->m_pointShell,
                                  a,
                                  alpha,
                                  myCollisionOracle,
                                  volume_model->getSpacing()/2.0);


    // prune the active contact set using the current unconstrained accelerations?
    checker.pruneContacts(m_contacts);



    // put all verified contacts into the contact set for calculating constraints
    m_contactPoints.clear();
    m_contactNormals.clear();
    for (std::vector<CollisionContact>::iterator it = m_contacts.begin();
         it != m_contacts.end(); ++it)
    {
        // compute the position in the object frame

        // calculate the normal (in this case of a sphere its easy)
        cml::vector3d p = it->position;
        p.normalize();
        p = -p;


        //vector3d q = toObject * it->position + display->proxyPosition();
        m_contactPoints.push_back(it->position);
        //m_contactNormals.push_back(toProxy * gradient(q));
        m_contactNormals.push_back(p);
    }

    // if the current contact set is non-empty, test the desired acceleration
    // between the proxy and device against the constraints (in the proxy frame)
    cml::vector3d ac = a, alphac = alpha;
    if (!m_contactNormals.empty())
        constrainedAcceleration(a, alpha, ac, alphac);

    // run collision detection to compute the next contact set
    IntervalCollisionDetector icd(context->m_pointShell, ac, alphac, myCollisionOracle,
                                  volume_model->getSpacing()/2.0);
    double t = icd.detectCollisions(m_contacts);

    // move the proxy according to the constrained acceleration and earliest
    // detected collision time
    double angle = alphac.length();
    HAPI::Vec3 axis(1,0,0);
    if (angle > 0.0) {
        alphac.normalize();
        axis = HAPI::Vec3(alphac[0], alphac[1], alphac[2]);
    }


    HAPI::Rotation constrainedRot = HAPI::Rotation(axis, t*angle);
    HAPI::Vec3 constrainedPos = t * HAPI::Vec3(ac[0], ac[1], ac[2]);

    HAPI::Vec3 newCandidatePos =  proxyPos + proxyRot*constrainedPos;
    HAPI::Rotation newCandidateRot = constrainedRot*proxyRot;//proxyRot;//halfRot;// proxyRot.slerp(deviceRot,0.5);


    /*
    context->proxyTransform->translation->setValue(
                H3DUtil::ArithmeticTypes::Vec3f(newCandidatePos));
    context->proxyTransform->rotation->setValue(
                H3DUtil::ArithmeticTypes::Rotation(newCandidateRot));
                */
    context->proxyPosition = newCandidatePos;
    context->proxyRotation = H3DUtil::ArithmeticTypes::Matrix3d(newCandidateRot);



    // =========================================================================
    // Cyclic debug statistics buffer and output
    // =========================================================================
    double elapsed = H3D::TimeStamp() - start;
    context->debug_buffercount++;
    if(context->debug_buffercount > 1000){

        double sumElapsed=0;
        double maxElapsed=0;
        for(int i=0;i<1000;i++){
            sumElapsed+=context->debug_elapsedTimes[i];
            if(context->debug_elapsedTimes[i] > maxElapsed)
                maxElapsed = context->debug_elapsedTimes[i];
        }
        double mean = sumElapsed/1000.0;

        double variance = 0;
        for(int i=0;i<1000;i++){
            variance += (context->debug_elapsedTimes[i]-mean)*
                        (context->debug_elapsedTimes[i]-mean);
        }

        int sgSwithes=0;
        unsigned int currSg = context->debug_scenegraphCount[0];
        for(unsigned int i=0;i<1000;i++){
            if(context->debug_scenegraphCount[i] != currSg){
                currSg = context->debug_scenegraphCount[i];
                sgSwithes++;
            }
        }
        /*
        std::cout << "Mean time: " << mean*1000 << "ms "
                  << "variance: " << variance*1000 << "ms "
                  << "max: " << maxElapsed*1000 << "ms "
                  << "sg switches: " << sgSwithes << std::endl;
		*/
        context->debug_buffercount = 0;
    }
    context->debug_elapsedTimes[context->debug_buffercount] = elapsed;
    context->debug_scenegraphCount[context->debug_buffercount] = context->debug_currentScenegraph;
    // =========================================================================


    if(m_contacts.size()>0)
        resolvePenetrations(input.hd->getPosition());


    // Springforce!
    double k = context->springConstant;

    //f_output = k * (context->proxyTransform->translation->getValue() -
    //                input.hd->getPosition());

    f_output = k * (context->proxyPosition - input.hd->getPosition());


    // Check if we get "stuck" (pop through risk)
    if((context->proxyPosition - input.hd->getPosition()).length() > 0.05){
        std::cout << "Stuck detected!" << std::endl;
        f_output = f_output*0;
        context->proxyPosition = input.hd->getPosition();
        //context->proxyRotation = input.hd->getOrientation();
        // TODO: Check that this is not a collision position...
    }


    /*
    if(f_output.length() > 3) {
        std::cout << "Force > 3: " << f_output  << std::endl;
    }
    if(f_output.length() > 6) {
        std::cout << "Force > 6: " << f_output  << std::endl;
    }
    if(f_output.length() > 9) {
        std::cout << "Force > 9: " << f_output  << std::endl;
    }
    */

    if(f_output.length() > 5) f_output = input.hd->getLastForce();



    delete myCollisionOracle;


    context->generated_force = H3DUtil::ArithmeticTypes::Vec3f(f_output);
    return EffectOutput(       f_output     );
}
//==============================================================================



//==============================================================================
bool CHapticDrillForce::resolvePenetrations(HAPI::Vec3 devicePos)
{


    // Get pos & rot from candidate proxy, in GLOBAL SPACE
    HAPI::Vec3 posLocalSpace = transform*
            context->proxyPosition;
    HAPI::Rotation r(H3DUtil::ArithmeticTypes::Matrix3d(HAPI::Rotation(transform.getRotationPart())) *
            context->proxyRotation);




    cml::vector3d objectnormal;
    objectnormal[0]=0;
    objectnormal[1]=0;
    objectnormal[2]=0;
    for (std::vector<CollisionContact>::iterator it = m_contacts.begin();
         it != m_contacts.end(); ++it)
    {
        // compute the position in the object frame
        // TODO: decide if it->position or it->contact is the right thing to use

        // calculate the normal (in this case of a sphere its easy)
        cml::vector3d p = it->position;
        p.normalize();
        p = -p;

        objectnormal+=p;
    }
    if(objectnormal.length()>0)
        objectnormal.normalize();



    //Move away from devicePos in direction of proxy
    HAPI::Vec3 proxyPos = context->proxyPosition;//(context->proxyTransform->translation->getValue());
    HAPI::Vec3 dir(objectnormal[0],objectnormal[1],objectnormal[2]);// = proxyPos - devicePos;


    HAPI::Rotation proxyRot(context->proxyRotation);// = context->proxyTransform->rotation->getValue();
    proxyRot.axis.normalizeSafe();
    HAPI::Matrix3 proxyRotMatrix = HAPI::Matrix3(proxyRot);

    dir = proxyRotMatrix*dir;

//    context->proxyPosition = proxyPos + dir*volume_model->getSpacing()/2.0*2.1;


    bool inside = true;

    int max10 = 0;

    // Step out until we do not collide anymore
    while(inside){
        max10++;
        if(max10>9) break;
        // Take a small step
        context->proxyPosition += dir*volume_model->getSpacing()*0.1;

        posLocalSpace = transform*
                context->proxyPosition;
        r = HAPI::Rotation(H3DUtil::ArithmeticTypes::Matrix3d(HAPI::Rotation(transform.getRotationPart())) *
                context->proxyRotation);
        MyCollisionOracle oracle(volume_model,r,posLocalSpace);

        // Chechk if ANY of the points are inside
        inside = false;
        for (std::vector<CollisionContact>::iterator it = m_contacts.begin();
             it != m_contacts.end(); ++it){
             if(oracle.inside(it->position)){
                 inside = true;
                 break;
             }
        }
    }

    return true;
}
//==============================================================================



//==============================================================================
// this is the new, corrected version that uses the technique described in
// "Gauss' Least Constraints Principle and Rigid Body Simulations" by
// Redon, Kheddar, and Coquillart (2002)
void CHapticDrillForce::constrainedAcceleration(const cml::vector3d &a, const cml::vector3d &alpha,
                                                cml::vector3d &ac, cml::vector3d &alphac)
{
    // set up typing for projection algorithm
    typedef double number;
    typedef WilhelmsenProjection<number>::vector6 vector6;

    // set up target point based on desired acceleration
    //vector6 au(a[0], a[1], a[2], alpha[0], alpha[1], alpha[2]);
    vector6 au; au << a[0], a[1], a[2], alpha[0], alpha[1], alpha[2];
    vector6 s = -m_massMatrixQ * au;

    // a global scale for the projection, used for conditioning
    number scale = WilhelmsenProjection<number>::norm(s);
    if (scale > 1e-10) {
        s /= scale;
    }
    else {
        ac.zero();
        alphac.zero();
        return;
    }

    // set up constraint matrix from contact set
    std::vector<vector6> Jq;
    for (unsigned int i = 0; i < m_contactNormals.size(); ++i)
    {
        const cml::vector3d &n = m_contactNormals[i];
        cml::vector3d k = cross(m_contactPoints[i], n);

        vector6 Jqi; Jqi << n[0], n[1], n[2], k[0], k[1], k[2];
        Jq.push_back(Jqi.transpose() * m_massMatrixQinv);
    }

    // condition the set of constraints
    WilhelmsenProjection<number>::conditionGenerators(Jq);

    // solve the projection
    vector6 p = scale * WilhelmsenProjection<number>::projectCone(Jq, s);
    vector6 x = m_massMatrixQinv * p + au;

    for (int i = 0; i < 3; ++i) {
        ac[i] = x[i];
        alphac[i] = x[i+3];
    }
}
// --------------------------------------------------------------------------



// --------------------------------------------------------------------------
void CHapticDrillForce::computeMassMatrix(std::vector<cml::vector3d> *m_pointShell)
{
    // assume the object has unit mass
    m_massMatrix.setIdentity();

    // assume mass is equally distributed over all points in the point shell
    double m = 1.0 / m_pointShell->size();
    matrix33 inertia = matrix33::Zero();

    for (std::vector<cml::vector3d>::iterator it = m_pointShell->begin();
         it != m_pointShell->end(); ++it)
    {
        cml::vector3d &v = *it;
        vector3 r(v[0], v[1], v[2]);
        inertia += m * (matrix33::Identity() * r.dot(r) - r * r.transpose());
    }

    // assemble the 6x6 mass matrix
    m_massMatrix.block<3,3>(3,3) = inertia;

    // now use Cholesky factorization to decompose M into Q^T Q,
    // where Q is upper diagonal
    m_massMatrixQ = m_massMatrix.llt().matrixL().transpose();

    // invert the Q matrix for later use
    m_massMatrixQinv = m_massMatrixQ.inverse();
}
// --------------------------------------------------------------------------


