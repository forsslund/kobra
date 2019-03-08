//////////////////////////////////////////////////////////////////////////////
//    Copyright 2009-2019, SenseGraphics AB and Lennart Svensson
//
//    This file is part of MedX3D.
//
//    MedX3D is free software; you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation; either version 2 of the License, or
//    (at your option) any later version.
//
//    MedX3D is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with MedX3D; if not, write to the Free Software
//    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//
//    A commercial license is also available. Please contact us at 
//    www.sensegraphics.com for more information.
//
//
/// \file Integrator.h
/// \brief Contains template classes used for preintegrated volume rendering.
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef _INTEGRATOR_H_
#define _INTEGRATOR_H_

#include <vector>

//#include "DataTypes.h"

using namespace std;

template <class Tin, class Tout>
class CIntegrator
{
public:
    static Tout RiemannMean( Tout (*Lookup)(Tin, void*),
        Tin fStart,
        Tin fEnd,
        int nIntervals,
        void *pUserData );

    static Tout Trapezoid( Tout (*Lookup)(Tin, void*),
        Tin fStart,
        Tin fEnd,
        int nIntervals,
        void *pUserData );

    static Tout Trapezoid( Tout (*Lookup)(Tin, void*),
        Tin fStart,
        Tin fEnd,
        int nIntervals,
        void *pUserData,
        vector <Tout>& rafIntermediateResult );

    static Tout TrapezoidStepSize( Tout (*Lookup)(Tin, void*),
        Tin fStart,
        Tin fEnd,
        Tin fStep,
        void *pUserData );
};

template <class Tin, class Tout>
Tout CIntegrator<Tin, Tout>::RiemannMean( Tout (*Lookup)(Tin, void*),
                                          Tin fStart,
                                          Tin fEnd,
                                          int nIntervals,
                                          void *pUserData )
{
    // Save the Trapezoid sum in accum.
    // Mean is calculated I = accum / 2 / nIntervals

    Tin fX = fStart;
    Tout accum = (*Lookup)(fStart, pUserData);
    Tin fStep = (fEnd - fStart) / nIntervals;

    for ( int n = 2; n < nIntervals+1; ++n )
    {
        fX += fStep;
        accum = accum + 2.0f*(*Lookup)(fX, pUserData);
    }

    accum = accum + (*Lookup)(fEnd, pUserData);
    Tout fRiemannMean = (Tout) (accum * 0.5 * (1.0 / (Tin)(nIntervals)));

    return fRiemannMean;
}

template <class Tin, class Tout>
Tout CIntegrator<Tin, Tout>::Trapezoid( Tout (*Lookup)(Tin, void*),
                            Tin fStart,
                            Tin fEnd,
                            int nIntervals,
                            void *pUserData )
{
    // Save the Trapezoid sum in accum.
    // Integral is calculated using I = accum / 2 / nIntervals * fSpanX

    Tin fX = fStart;
    Tout accum = (*Lookup)(fStart, pUserData);
    Tin fStep = (fEnd - fStart) / nIntervals;

    for ( int n = 2; n < nIntervals+1; ++n )
    {
        fX += fStep;
        accum = accum + 2.0f*(*Lookup)(fX, pUserData);
    }

    accum = accum + (*Lookup)(fEnd, pUserData);
    Tout fIntegral = accum * 0.5 * (1.0 / (Tin)(nIntervals)) * (fEnd-fStart);

    return fIntegral;
}

template <class Tin, class Tout>
Tout CIntegrator<Tin, Tout>::Trapezoid( Tout (*Lookup)(Tin, void*),
                             Tin fStart,
                             Tin fEnd,
                             int nIntervals,
                             void *pUserData,
                             vector <Tout>& rafIntermediateResult )
{
    rafIntermediateResult.clear();

    Tin fX = fStart;
    Tout accum = (*Lookup)(fStart, pUserData);
    Tin fStep = (fEnd - fStart) / nIntervals;

    for ( int n = 2; n <= nIntervals+1; ++n )
    {
        fX += fStep;
        Tout fY = (*Lookup)(fX, pUserData);
        accum = accum + fY;
        Tout fTemp = accum * 0.5 * (1.0 / (Tout)(nIntervals)) * (fX-fStart);
        rafIntermediateResult.push_back( fTemp );

        accum += fY;
    }

    return rafIntermediateResult.back();
}

template <class Tin, class Tout>
Tout CIntegrator<Tin, Tout>::TrapezoidStepSize( Tout (*Lookup)(Tin, void*),
                                     Tin fStart,
                                     Tin fEnd,
                                     Tin fStep,
                                     void *pUserData )
{
    Tin fX, fBeforeEnd;
    Tout accum;

    accum = (*Lookup)(fStart, pUserData);
    fBeforeEnd = fEnd - fStep;
    for ( fX=fStart+fStep; fX <= fBeforeEnd; fX += fStep )
    {
        accum += 2.0f*(*Lookup)(fX, pUserData);
    }

    // The last element in the Trapezoid sum will be the second last sample
    // totally, because the final step length will probably be different.
    Tout fNextToLastY = (*Lookup)(fX, pUserData);
    Tout fLastY = (*Lookup)(fEnd, pUserData);
    accum += fNextToLastY;

    Tout fIntegral1 = accum * 0.5 * fStep;
    Tout fIntegral2 = (fNextToLastY + fLastY) * 0.5 * (fEnd - fX);

    return fIntegral1 + fIntegral2;
}

#endif // _INTEGRATOR_H_
