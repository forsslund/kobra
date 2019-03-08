//////////////////////////////////////////////////////////////////////////////
//    Copyright 2012 Sonny Chan
//
//    IntervalCollisionDetector is free software; you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation; either version 2 of the License, or
//    (at your option) any later version.
//
//    IntervalCollisionDetector is distributed in the hope that it will be useful,
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

#ifndef INTERVALCOLLISIONDETECTOR_H
#define INTERVALCOLLISIONDETECTOR_H

#include <vector>
#include "cml/mathlib/typedef.h"

// an "oracle" that can tell us whether or not we're inside an object
class CollisionOracle
{
public:
    virtual bool inside(const cml::vector3d &p) const = 0;
};

// a structure to store a point of contact
struct CollisionContact
{
    int             index;
    cml::vector3d   position;
    cml::vector3d   contact;
    cml::vector3d   normal;
};

class IntervalCollisionDetector
{
    const std::vector<cml::vector3d> &m_points;

    // the rigid body trajectory of the points, in local frame
    cml::vector3d   m_linear;
    double          m_displacement;
    cml::vector3d   m_axis;
    double          m_angle;

    // feature size limits for collision detection interval step
    double          m_feature;
    double          m_epsilon;

    // oracle to tell us whether or not we're inside an object
    CollisionOracle *m_oracle;

    // structure to track intervals along a point's path
    struct PointInterval
    {
        int index;
        double t1, t2;
        double tStep, tLimit;
        bool contact;
        int marked;

        PointInterval(int i = 0)
            : index(i), t1(0.0), t2(0.0), tStep(1.0), tLimit(1.0),
            contact(false), marked(-1) {}
    };

    // comparator to order point intervals by time
    struct After
    {
        bool operator()(const PointInterval &a, const PointInterval &b) {
            return a.t2 > b.t2;
        }
    };

    // transforms a point according to the given motion, at time t
    cml::vector3d pointAtTime(const cml::vector3d &p, double t);

    // estimates the arc length of a point undergoing the full motion
    double arcLength(const cml::vector3d &p);

    // tests a point for contact with the object, then refines the interval
    void testPoint(PointInterval &pi);

public:
    IntervalCollisionDetector(const std::vector<cml::vector3d> &pointShell,
                              const cml::vector3d &a, const cml::vector3d &alpha,
                              CollisionOracle *oracle, double featureSize = 0.005);

    // detects collisions along the path of motion by dividing each point's
    // path into feature-sized segments, and if contact is detected, the
    // contact time interval is bisected until a descired accuracy is reached
    //      - returns a lower bound on the time of first contact
    //      - on start, contactIndices is current contact set
    //      - on finish, contactIndices contains original items plus points
    //          within an epsilon (x2?) of colliding
    double detectCollisions(std::vector<CollisionContact> &contacts);

    // prunes out inactive points (ones not within an epsilon distance of
    // contact) from a given contact set
    void pruneContacts(std::vector<CollisionContact> &contacts);

    // attempts to move the proxy along the given vector until the set of
    // contact points no longer penetrates the isosurface
    double resolvePenetrations(const std::vector<CollisionContact> &contacts);

};

#endif // INTERVALCOLLISIONDETECTOR_H
