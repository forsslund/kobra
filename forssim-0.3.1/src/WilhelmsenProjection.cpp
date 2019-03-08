//////////////////////////////////////////////////////////////////////////////
//    Copyright 2012 Sonny Chan
//
//    WilhelmsenProjection is free software; you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation; either version 2 of the License, or
//    (at your option) any later version.
//
//    WilhelmsenProjection is distributed in the hope that it will be useful,
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

#include "WilhelmsenProjection.h"
#include <iterator>
#include <cassert>
#include <iostream>


#define USE_EIGEN


#ifdef USE_EIGEN
#include "Eigen/Dense"
#else
#include <cml/cml.h>
using namespace cml;
#endif

template <class T> std::vector<int> WilhelmsenProjection<T>::constraints;
template <class T> const T WilhelmsenProjection<T>::eps   = 1e-7;
template <class T> const T WilhelmsenProjection<T>::eps2  = 1e-14;

// --------------------------------------------------------------------------
template <class T>
#ifdef USE_EIGEN
        Eigen::Matrix<T, Eigen::Dynamic, 1>
#else
        cml::vector< T, cml::dynamic<> >
#endif
        WilhelmsenProjection<T>::projectSubspace(const std::vector<vector6> &S,
                                              const vector6 &q)
{
    // set up an nxn linear system, Ax = b, to solve for the projection
    //  if S = { e_1, e_2, ..., e_n }
    //  then the system is \sum_{i=1}^n x_i <e_i e_j> = <q e_j> for j = 1..n

    int n = int(S.size());
    vectorn result(n);

    // residuals to check which solution is better
//    T cmlR, eigenR;

#ifdef USE_EIGEN
    // use Eigen's LU factorization to solve the normal equations
    matrixn A(n, n);
    for (int i = 0; i < n; ++i)
        for (int j = i; j < n; ++j)
            A(i, j) = A(j, i) = S[i].dot(S[j]);

    vectorn b(n);
    for (int j = 0; j < n; ++j)
        b[j] = q.dot(S[j]);

    vectorn x = A.colPivHouseholderQr().solve(b);
    result = x;

//    T det = A.determinant();
//    if (fabs(det) < eps2)
//        std::cout << "Warning: det(A) = " << det << std::endl;

    // compute the residual from the solution
//    eigenR = (A*x - b).norm();

#else
    // set up the matrix A
    matrixn A(n, n);
    for (int i = 0; i < n; ++i)
        for (int j = i; j < n; ++j)
            A(i, j) = A(j, i) = dot(S[i], S[j]);

    // set up the vector b
    vectorn b(n);
    for (int j = 0; j < n; ++j)
        b[j] = dot(q, S[j]);

    // compute x by inverting the matrix A
    vectorn x = inverse(A) * b;
    result = x;

    // compute the residual from the solution
//    cmlR = length(A*x - b);
#endif


    // output a status to see which one performs better
//    static int counter = 0;
//    if (++counter > 500)
//    {
//        std::cout << "CML residual: " << cmlR << '\t'
//                << "Eigen residual:" << eigenR << std::endl;
//        counter = 0;
//    }

    return result;
}

// --------------------------------------------------------------------------
template <class T>
#ifdef USE_EIGEN
        Eigen::Matrix<T, 6, 1>
#else
        cml::vector< T, cml::fixed<6> >
#endif
        WilhelmsenProjection<T>::projectCone(const std::vector<vector6> &K,
                                             const vector6 &q)
{
    // Step 0: find an initial vector in the direction of q
    int n = int(K.size()), ki = 0;
    vector6 p = zero6();

    std::vector<vector6>    F;
    std::vector<int>        index;
    std::vector<T>          lambda;
    std::vector<bool>       used(n, false);

    for (; ki < n && dot(q, K[ki]) < eps2; ++ki);
    if (ki < n) {
        F.push_back(K[ki]);
        index.push_back(ki);
        lambda.push_back(dot(q, F[0]) / norm2(F[0]));
        p = lambda[0] * F[0];
        used[ki] = true;
    }
    else return p;

    // main algorithm iteration loop
    for (;;)
    {
        // Step 1: look for the next generator that lies between the current
        //         subcone and the target point
        vector6 eta = q - p;
        if (norm2(eta) < eps2) break;

        // this search is linear... can we do better?
        for (ki = 0; ki < n; ++ki) {
            if (used[ki]) continue;
            if (dot(eta, K[ki]) > eps) break;
        }
        if (ki < n) {
            F.push_back(K[ki]);
            index.push_back(ki);
            lambda.push_back(0.0);
            used[ki] = true;
        }
        else break;

        // auxiliary loop to reduce the subcone to the smallest face
        bool repeat;
        do
        {
            // Step 2: project the target onto the span of F and see if the point
            //         lies within the subcone generated by F
            vectorn beta = projectSubspace(F, q);
            int m = int(F.size());
            T lo = smallest(beta);

            // if the projected point is outside the subcone, we need to find
            // the nearest point in the subspace within the subcone generated by F
            if (lo < -eps)
            {
                T rho = 1.0;
                for (int i = 0; i < m; ++i)
                    if (lambda[i] - beta[i] >= eps)
                        rho = std::min(rho, lambda[i] / (lambda[i] - beta[i]));

                // compute new barycentric coordinates gamma
                for (int i = 0; i < m; ++i)
                    beta[i] = (1.0 - rho) * lambda[i] + rho * beta[i];
                lo = smallest(beta);
                repeat = true;
            }
            // otherwise the projected point is already within the subcone,
            // so just remove redundant generators from F
            else repeat = false;

            // remove generators not on the separating hyperplane (zero barycentric)
            if (lo < eps) {
                std::vector<vector6> E; E.swap(F);
                std::vector<int> edex; edex.swap(index);
                lambda.clear();
                for (int i = 0; i < m; ++i) {
                    if (beta[i] >= eps) {
                        F.push_back(E[i]);
                        index.push_back(edex[i]);
                        lambda.push_back(beta[i]);
                    }
                }
            }
            else {
                for (int i = 0; i < m; ++i)
                    lambda[i] = beta[i];
            }

            // compute the new projected point p based on F and lambda
            m = int(F.size());
            p = zero6();
            for (int i = 0; i < m; ++i)
                p += lambda[i] * F[i];

        } while (repeat);
    }

    constraints = index;

    return p;
}

// --------------------------------------------------------------------------
template <class T>
        void WilhelmsenProjection<T>::conditionGenerators(std::vector<vector6> &K)
{
    std::vector<vector6> C;
    for (typename std::vector<vector6>::iterator kt = K.begin(); kt != K.end(); ++kt)
    {
        // check vector length
        T len = norm(*kt);
        if (len < eps) continue;
        vector6 v = *kt / len;

        // check for parallel vectors
        bool indep = true;
        for (typename std::vector<vector6>::iterator ct = C.begin(); ct != C.end(); ++ct)
            if (1.0 - dot(v, *ct) < eps) { indep = false; break; }
        if (indep) C.push_back(v);
    }

    // swap K with the set of conditioned generators
    K.swap(C);
}

// --------------------------------------------------------------------------
// debug function to print out the last set of active constraints

template <class T>
        void WilhelmsenProjection<T>::printConstraints()
{
    std::cout << "Wilhelmson constraints: ";
    std::copy(constraints.begin(), constraints.end(),
              std::ostream_iterator<int>(std::cout, "  "));
    std::cout << std::endl;
}

// --------------------------------------------------------------------------
// template instantiations for double and long double data types

template class WilhelmsenProjection<double>;
template class WilhelmsenProjection<long double>;

// --------------------------------------------------------------------------
