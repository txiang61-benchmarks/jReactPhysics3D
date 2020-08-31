/*
 * ReactPhysics3D physics library, http://code.google.com/p/reactphysics3d/
 * Copyright (c) 2010-2013 Daniel Chappuis
 *
 * This software is provided 'as-is', without any express or implied warranty.
 * In no event will the authors be held liable for any damages arising from the
 * use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not claim
 *    that you wrote the original software. If you use this software in a
 *    product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 *
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 *
 * 3. This notice may not be removed or altered from any source distribution.
 *
 * This file has been modified during the port to Java and differ from the source versions.
 */
package net.smert.jreactphysics3d.collision.narrowphase.GJK;

import units.qual.Dimensionless;
import net.smert.jreactphysics3d.configuration.Defaults;
import net.smert.jreactphysics3d.mathematics.Vector3;

/**
 * This class represents a simplex which is a set of 3D points. This class is used in the GJK algorithm. This
 * implementation is based on the implementation discussed in the book "Collision Detection in 3D Environments". This
 * class implements the Johnson's algorithm for computing the point of a simplex that is closest to the origin and also
 * the smallest simplex needed to represent that closest point.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
@Dimensionless
public class Simplex {

    // Maximum length of pointsLengthSquare[i]
    private @Dimensionless float mMaxLengthSquare;

    // pointsLengthSquare[i] = (points[i].length)^2
    private final @Dimensionless float @Dimensionless [] mPointsLengthSquare = new @Dimensionless float @Dimensionless [((@Dimensionless int) (4))];

    // Cached determinant values
    private final @Dimensionless float @Dimensionless [] @Dimensionless [] mDet = new @Dimensionless float @Dimensionless [((@Dimensionless int) (16))] @Dimensionless [((@Dimensionless int) (4))];

    // norm[i][j] = (diff[i][j].length())^2
    private final @Dimensionless float @Dimensionless [] @Dimensionless [] mNormSquare = new @Dimensionless float @Dimensionless [((@Dimensionless int) (4))] @Dimensionless [((@Dimensionless int) (4))];

    // allBits = bitsCurrentSimplex | lastFoundBit;
    private @Dimensionless int mAllBits;

    // 4 bits that identify the current points of the simplex
    // For instance, 0101 means that points[1] and points[3] are in the simplex
    private @Dimensionless int mBitsCurrentSimplex;

    // Number between 1 and 4 that identify the last found support point
    private @Dimensionless int mLastFound;

    // Position of the last found support point (lastFoundBit = 0x1 << lastFound)
    private @Dimensionless int mLastFoundBit;

    // Current points
    private final @Dimensionless Vector3 @Dimensionless [] mPoints = new @Dimensionless Vector3 @Dimensionless [((@Dimensionless int) (4))];

    // Support points of object A in local coordinates
    private final @Dimensionless Vector3 @Dimensionless [] mSuppPointsA = new @Dimensionless Vector3 @Dimensionless [((@Dimensionless int) (4))];

    // Support points of object B in local coordinates
    private final @Dimensionless Vector3 @Dimensionless [] mSuppPointsB = new @Dimensionless Vector3 @Dimensionless [((@Dimensionless int) (4))];

    // diff[i][j] contains points[i] - points[j]
    private final @Dimensionless Vector3 @Dimensionless [] @Dimensionless [] mDiffLength = new @Dimensionless Vector3 @Dimensionless [((@Dimensionless int) (4))] @Dimensionless [((@Dimensionless int) (4))];

    public Simplex() {
        mAllBits = ((@Dimensionless int) (0));
        mBitsCurrentSimplex = ((@Dimensionless int) (0));

        for (@Dimensionless int i = ((@Dimensionless int) (0)); i < mPoints.length; i++) {
            mPoints[i] = new @Dimensionless Vector3();
        }
        for (@Dimensionless int i = ((@Dimensionless int) (0)); i < mSuppPointsA.length; i++) {
            mSuppPointsA[i] = new @Dimensionless Vector3();
        }
        for (@Dimensionless int i = ((@Dimensionless int) (0)); i < mSuppPointsB.length; i++) {
            mSuppPointsB[i] = new @Dimensionless Vector3();
        }
    }

    // Return the closest point "v" in the convex hull of the points in the subset
    // represented by the bits "subset"
    private @Dimensionless Vector3 computeClosestPointForSubset(@Dimensionless Simplex this, @Dimensionless int subset) {

        mMaxLengthSquare = ((@Dimensionless float) (0.0f));
        @Dimensionless
        float deltaX = ((@Dimensionless float) (0.0f));            // deltaX = sum of all det[subset][i]
        @Dimensionless
        Vector3 v = new @Dimensionless Vector3();      // Closet point v = sum(lambda_i * points[i])

        // For each four point in the possible simplex set
        for (int i = ((@Dimensionless int) (0)), bit = ((@Dimensionless int) (0x1)); i < ((@Dimensionless int) (4)); i++, bit <<= ((@Dimensionless int) (1))) {
            // If the current point is in the subset
            if (overlap(subset, bit)) {
                // deltaX = sum of all det[subset][i]
                deltaX += mDet[subset][i];

                if (mMaxLengthSquare < mPointsLengthSquare[i]) {
                    mMaxLengthSquare = mPointsLengthSquare[i];
                }

                // Closest point v = sum(lambda_i * points[i])
                v.add(new @Dimensionless Vector3(mPoints[i]).multiply(mDet[subset][i]));
            }
        }

        assert (deltaX > ((@Dimensionless float) (0.0f)));

        // Return the closet point "v" in the convex hull for the given subset
        return new @Dimensionless Vector3(v).multiply(((@Dimensionless float) (1.0f)) / deltaX);
    }

    // Compute the cached determinant values
    private void computeDeterminants(@Dimensionless Simplex this) {

        mDet[mLastFoundBit][mLastFound] = ((@Dimensionless float) (1.0f));

        // If the current simplex is not empty
        if (!isEmpty()) {

            // For each possible four points in the simplex set
            for (int i = ((@Dimensionless int) (0)), bitI = ((@Dimensionless int) (0x1)); i < ((@Dimensionless int) (4)); i++, bitI <<= ((@Dimensionless int) (1))) {

                // If the current point is in the simplex
                if (overlap(mBitsCurrentSimplex, bitI)) {
                    @Dimensionless
                    int bit2 = bitI | mLastFoundBit;

                    mDet[bit2][i] = mDiffLength[mLastFound][i].dot(mPoints[mLastFound]);
                    mDet[bit2][mLastFound] = mDiffLength[i][mLastFound].dot(mPoints[i]);

                    for (int j = ((@Dimensionless int) (0)), k, bitJ = ((@Dimensionless int) (0x1)); j < i; j++, bitJ <<= ((@Dimensionless int) (1))) {
                        if (overlap(mBitsCurrentSimplex, bitJ)) {
                            @Dimensionless
                            int bit3 = bitJ | bit2;

                            k = mNormSquare[i][j] < mNormSquare[mLastFound][j] ? i : mLastFound;
                            mDet[bit3][j] = mDet[bit2][i] * mDiffLength[k][j].dot(mPoints[i])
                                    + mDet[bit2][mLastFound]
                                    * mDiffLength[k][j].dot(mPoints[mLastFound]);

                            k = mNormSquare[j][i] < mNormSquare[mLastFound][i] ? j : mLastFound;
                            mDet[bit3][i] = mDet[bitJ | mLastFoundBit][j]
                                    * mDiffLength[k][i].dot(mPoints[j])
                                    + mDet[bitJ | mLastFoundBit][mLastFound]
                                    * mDiffLength[k][i].dot(mPoints[mLastFound]);

                            k = mNormSquare[i][mLastFound] < mNormSquare[j][mLastFound] ? i : j;
                            mDet[bit3][mLastFound] = mDet[bitJ | bitI][j]
                                    * mDiffLength[k][mLastFound].dot(mPoints[j])
                                    + mDet[bitJ | bitI][i]
                                    * mDiffLength[k][mLastFound].dot(mPoints[i]);
                        }
                    }
                }
            }

            if (mAllBits == ((@Dimensionless int) (0xf))) {
                @Dimensionless
                int k;

                k = mNormSquare[((@Dimensionless int) (1))][((@Dimensionless int) (0))] < mNormSquare[((@Dimensionless int) (2))][((@Dimensionless int) (0))]
                        ? (mNormSquare[((@Dimensionless int) (1))][((@Dimensionless int) (0))] < mNormSquare[((@Dimensionless int) (3))][((@Dimensionless int) (0))] ? ((@Dimensionless int) (1)) : ((@Dimensionless int) (3)))
                        : (mNormSquare[((@Dimensionless int) (2))][((@Dimensionless int) (0))] < mNormSquare[((@Dimensionless int) (3))][((@Dimensionless int) (0))] ? ((@Dimensionless int) (2)) : ((@Dimensionless int) (3)));
                mDet[((@Dimensionless int) (0xf))][((@Dimensionless int) (0))] = mDet[((@Dimensionless int) (0xe))][((@Dimensionless int) (1))] * mDiffLength[k][((@Dimensionless int) (0))].dot(mPoints[((@Dimensionless int) (1))])
                        + mDet[((@Dimensionless int) (0xe))][((@Dimensionless int) (2))] * mDiffLength[k][((@Dimensionless int) (0))].dot(mPoints[((@Dimensionless int) (2))])
                        + mDet[((@Dimensionless int) (0xe))][((@Dimensionless int) (3))] * mDiffLength[k][((@Dimensionless int) (0))].dot(mPoints[((@Dimensionless int) (3))]);

                k = mNormSquare[((@Dimensionless int) (0))][((@Dimensionless int) (1))] < mNormSquare[((@Dimensionless int) (2))][((@Dimensionless int) (1))]
                        ? (mNormSquare[((@Dimensionless int) (0))][((@Dimensionless int) (1))] < mNormSquare[((@Dimensionless int) (3))][((@Dimensionless int) (1))] ? ((@Dimensionless int) (0)) : ((@Dimensionless int) (3)))
                        : (mNormSquare[((@Dimensionless int) (2))][((@Dimensionless int) (1))] < mNormSquare[((@Dimensionless int) (3))][((@Dimensionless int) (1))] ? ((@Dimensionless int) (2)) : ((@Dimensionless int) (3)));
                mDet[((@Dimensionless int) (0xf))][((@Dimensionless int) (1))] = mDet[((@Dimensionless int) (0xd))][((@Dimensionless int) (0))] * mDiffLength[k][((@Dimensionless int) (1))].dot(mPoints[((@Dimensionless int) (0))])
                        + mDet[((@Dimensionless int) (0xd))][((@Dimensionless int) (2))] * mDiffLength[k][((@Dimensionless int) (1))].dot(mPoints[((@Dimensionless int) (2))])
                        + mDet[((@Dimensionless int) (0xd))][((@Dimensionless int) (3))] * mDiffLength[k][((@Dimensionless int) (1))].dot(mPoints[((@Dimensionless int) (3))]);

                k = mNormSquare[((@Dimensionless int) (0))][((@Dimensionless int) (2))] < mNormSquare[((@Dimensionless int) (1))][((@Dimensionless int) (2))]
                        ? (mNormSquare[((@Dimensionless int) (0))][((@Dimensionless int) (2))] < mNormSquare[((@Dimensionless int) (3))][((@Dimensionless int) (2))] ? ((@Dimensionless int) (0)) : ((@Dimensionless int) (3)))
                        : (mNormSquare[((@Dimensionless int) (1))][((@Dimensionless int) (2))] < mNormSquare[((@Dimensionless int) (3))][((@Dimensionless int) (2))] ? ((@Dimensionless int) (1)) : ((@Dimensionless int) (3)));
                mDet[((@Dimensionless int) (0xf))][((@Dimensionless int) (2))] = mDet[((@Dimensionless int) (0xb))][((@Dimensionless int) (0))] * mDiffLength[k][((@Dimensionless int) (2))].dot(mPoints[((@Dimensionless int) (0))])
                        + mDet[((@Dimensionless int) (0xb))][((@Dimensionless int) (1))] * mDiffLength[k][((@Dimensionless int) (2))].dot(mPoints[((@Dimensionless int) (1))])
                        + mDet[((@Dimensionless int) (0xb))][((@Dimensionless int) (3))] * mDiffLength[k][((@Dimensionless int) (2))].dot(mPoints[((@Dimensionless int) (3))]);

                k = mNormSquare[((@Dimensionless int) (0))][((@Dimensionless int) (3))] < mNormSquare[((@Dimensionless int) (1))][((@Dimensionless int) (3))]
                        ? (mNormSquare[((@Dimensionless int) (0))][((@Dimensionless int) (3))] < mNormSquare[((@Dimensionless int) (2))][((@Dimensionless int) (3))] ? ((@Dimensionless int) (0)) : ((@Dimensionless int) (2)))
                        : (mNormSquare[((@Dimensionless int) (1))][((@Dimensionless int) (3))] < mNormSquare[((@Dimensionless int) (2))][((@Dimensionless int) (3))] ? ((@Dimensionless int) (1)) : ((@Dimensionless int) (2)));
                mDet[((@Dimensionless int) (0xf))][((@Dimensionless int) (3))] = mDet[((@Dimensionless int) (0x7))][((@Dimensionless int) (0))] * mDiffLength[k][((@Dimensionless int) (3))].dot(mPoints[((@Dimensionless int) (0))])
                        + mDet[((@Dimensionless int) (0x7))][((@Dimensionless int) (1))] * mDiffLength[k][((@Dimensionless int) (3))].dot(mPoints[((@Dimensionless int) (1))])
                        + mDet[((@Dimensionless int) (0x7))][((@Dimensionless int) (2))] * mDiffLength[k][((@Dimensionless int) (3))].dot(mPoints[((@Dimensionless int) (2))]);
            }
        }
    }

    // Return true if the subset is a proper subset.
    // A proper subset X is a subset where for all point "y_i" in X, we have
    // detX_i value bigger than zero
    private @Dimensionless boolean isProperSubset(@Dimensionless Simplex this, @Dimensionless int subset) {

        // For each four point of the possible simplex set
        for (int i = ((@Dimensionless int) (0)), bit = ((@Dimensionless int) (0x1)); i < ((@Dimensionless int) (4)); i++, bit <<= ((@Dimensionless int) (1))) {
            if (overlap(subset, bit) && mDet[subset][i] <= ((@Dimensionless float) (0.0f))) {
                return false;
            }
        }

        return true;
    }

    // Return true if the bits of "b" is a subset of the bits of "a"
    private @Dimensionless boolean isSubset(@Dimensionless Simplex this, @Dimensionless int a, @Dimensionless int b) {
        return ((a & b) == a);
    }

    // Return true if the subset is a valid one for the closest point computation.
    // A subset X is valid if :
    //    1. delta(X)_i > 0 for each "i" in I_x and
    //    2. delta(X U {y_j})_j <= 0 for each "j" not in I_x_
    private @Dimensionless boolean isValidSubset(@Dimensionless Simplex this, @Dimensionless int subset) {

        // For each four point in the possible simplex set
        for (int i = ((@Dimensionless int) (0)), bit = ((@Dimensionless int) (0x1)); i < ((@Dimensionless int) (4)); i++, bit <<= ((@Dimensionless int) (1))) {
            if (overlap(mAllBits, bit)) {
                // If the current point is in the subset
                if (overlap(subset, bit)) {
                    // If one delta(X)_i is smaller or equal to zero
                    if (mDet[subset][i] <= ((@Dimensionless float) (0.0f))) {
                        // The subset is not valid
                        return false;
                    }
                } // If the point is not in the subset and the value delta(X U {y_j})_j
                // is bigger than zero
                else if (mDet[subset | bit][i] > ((@Dimensionless float) (0.0f))) {
                    // The subset is not valid
                    return false;
                }
            }
        }

        return true;
    }

    // Return true if some bits of "a" overlap with bits of "b"
    private @Dimensionless boolean overlap(@Dimensionless Simplex this, @Dimensionless int a, @Dimensionless int b) {
        return ((a & b) != ((@Dimensionless int) (0x0)));
    }

    // Update the cached values used during the GJK algorithm
    private void updateCache(@Dimensionless Simplex this) {

        // For each of the four possible points of the simplex
        for (int i = ((@Dimensionless int) (0)), bit = ((@Dimensionless int) (0x1)); i < ((@Dimensionless int) (4)); i++, bit <<= ((@Dimensionless int) (1))) {
            // If the current points is in the simplex
            if (overlap(mBitsCurrentSimplex, bit)) {

                // Compute the distance between two points in the possible simplex set
                mDiffLength[i][mLastFound] = new @Dimensionless Vector3(mPoints[i]).subtract(mPoints[mLastFound]);
                mDiffLength[mLastFound][i] = new @Dimensionless Vector3(mDiffLength[i][mLastFound]).invert();

                // Compute the squared length of the vector
                // distances from points in the possible simplex set
                mNormSquare[i][mLastFound] = mNormSquare[mLastFound][i]
                        = mDiffLength[i][mLastFound].dot(mDiffLength[i][mLastFound]);
            }
        }
    }

    // Add a new support point of (A-B) into the simplex
    // suppPointA : support point of object A in a direction -v
    // suppPointB : support point of object B in a direction v
    // point      : support point of object (A-B) => point = suppPointA - suppPointB
    public void addPoint(@Dimensionless Simplex this, @Dimensionless Vector3 point, @Dimensionless Vector3 suppPointA, @Dimensionless Vector3 suppPointB) {

        assert (!isFull());

        mLastFound = ((@Dimensionless int) (0));
        mLastFoundBit = ((@Dimensionless int) (0x1));

        // Look for the bit corresponding to one of the four point that is not in
        // the current simplex
        while (overlap(mBitsCurrentSimplex, mLastFoundBit)) {
            mLastFound++;
            mLastFoundBit <<= ((@Dimensionless int) (1));
        }

        assert (mLastFound >= ((@Dimensionless int) (0)) && mLastFound < ((@Dimensionless int) (4)));

        // Add the point into the simplex
        mPoints[mLastFound].set(point);
        mPointsLengthSquare[mLastFound] = point.dot(point);
        mAllBits = mBitsCurrentSimplex | mLastFoundBit;

        // Update the cached values
        updateCache();

        // Compute the cached determinant values
        computeDeterminants();

        // Add the support points of objects A and B
        mSuppPointsA[mLastFound].set(suppPointA);
        mSuppPointsB[mLastFound].set(suppPointB);
    }

    // Backup the closest point
    public void backupClosestPointInSimplex(@Dimensionless Simplex this, @Dimensionless Vector3 v) {

        @Dimensionless
        float minDistSquare = Defaults.DECIMAL_LARGEST;

        for (@Dimensionless int bit = mAllBits; bit != ((@Dimensionless int) (0x0)); bit--) {
            if (isSubset(bit, mAllBits) && isProperSubset(bit)) {
                @Dimensionless
                Vector3 u = computeClosestPointForSubset(bit);
                @Dimensionless
                float distSquare = u.dot(u);
                if (distSquare < minDistSquare) {
                    minDistSquare = distSquare;
                    mBitsCurrentSimplex = bit;
                    v.set(u);
                }
            }
        }
    }

    // Compute the closest point "v" to the origin of the current simplex.
    // This method executes the Jonhnson's algorithm for computing the point
    // "v" of simplex that is closest to the origin. The method returns true
    // if a closest point has been found.
    public @Dimensionless boolean computeClosestPoint(@Dimensionless Simplex this, @Dimensionless Vector3 v) {

        // For each possible simplex set
        for (@Dimensionless int subset = mBitsCurrentSimplex; subset != ((@Dimensionless int) (0x0)); subset--) {
            // If the simplex is a subset of the current simplex and is valid for the Johnson's
            // algorithm test
            if (isSubset(subset, mBitsCurrentSimplex) && isValidSubset(subset | mLastFoundBit)) {
                mBitsCurrentSimplex = subset | mLastFoundBit;                   // Add the last added point to the current simplex
                @Dimensionless
                Vector3 u = computeClosestPointForSubset(mBitsCurrentSimplex);  // Compute the closest point in the simplex
                v.set(u);
                return true;
            }
        }

        // If the simplex that contains only the last added point is valid for the Johnson's algorithm test
        if (isValidSubset(mLastFoundBit)) {
            mBitsCurrentSimplex = mLastFoundBit;                // Set the current simplex to the set that contains only the last added point
            mMaxLengthSquare = mPointsLengthSquare[mLastFound]; // Update the maximum square length
            @Dimensionless
            Vector3 u = mPoints[mLastFound];                    // The closest point of the simplex "v" is the last added point
            v.set(u);
            return true;
        }

        // The algorithm failed to found a point
        return false;
    }

    // Compute the closest points "pA" and "pB" of object A and B.
    // The points are computed as follows :
    //      pA = sum(lambda_i * a_i)    where "a_i" are the support points of object A
    //      pB = sum(lambda_i * b_i)    where "b_i" are the support points of object B
    //      with lambda_i = deltaX_i / deltaX
    public void computeClosestPointsOfAandB(@Dimensionless Simplex this, @Dimensionless Vector3 pA, @Dimensionless Vector3 pB) {

        @Dimensionless
        float deltaX = ((@Dimensionless float) (0.0f));
        pA.zero();
        pB.zero();

        // For each four points in the possible simplex set
        for (int i = ((@Dimensionless int) (0)), bit = ((@Dimensionless int) (0x1)); i < ((@Dimensionless int) (4)); i++, bit <<= ((@Dimensionless int) (1))) {
            // If the current point is part of the simplex
            if (overlap(mBitsCurrentSimplex, bit)) {
                deltaX += mDet[mBitsCurrentSimplex][i];
                pA.add(new @Dimensionless Vector3(mSuppPointsA[i]).multiply(mDet[mBitsCurrentSimplex][i]));
                pB.add(new @Dimensionless Vector3(mSuppPointsB[i]).multiply(mDet[mBitsCurrentSimplex][i]));
            }
        }

        assert (deltaX > ((@Dimensionless float) (0.0f)));
        @Dimensionless
        float factor = ((@Dimensionless float) (1.0f)) / deltaX;
        pA.multiply(factor);
        pB.multiply(factor);
    }

    // Return the maximum squared length of a point
    public @Dimensionless float getMaxLengthSquareOfAPoint(@Dimensionless Simplex this) {
        return mMaxLengthSquare;
    }

    // Return the points of the simplex
    public @Dimensionless int getSimplex(@Dimensionless Simplex this, @Dimensionless Vector3 @Dimensionless [] suppPointsA, @Dimensionless Vector3 @Dimensionless [] suppPointsB, @Dimensionless Vector3 @Dimensionless [] points) {
        @Dimensionless
        int numVertices = ((@Dimensionless int) (0));

        // For each four point in the possible simplex
        for (int i = ((@Dimensionless int) (0)), bit = ((@Dimensionless int) (0x1)); i < ((@Dimensionless int) (4)); i++, bit <<= ((@Dimensionless int) (1))) {

            // If the current point is in the simplex
            if (overlap(mBitsCurrentSimplex, bit)) {

                // Store the points
                suppPointsA[numVertices] = new @Dimensionless Vector3(this.mSuppPointsA[numVertices]);
                suppPointsB[numVertices] = new @Dimensionless Vector3(this.mSuppPointsB[numVertices]);
                points[numVertices] = new @Dimensionless Vector3(this.mPoints[numVertices]);

                numVertices++;
            }
        }

        // Return the number of points in the simplex
        return numVertices;
    }

    // Return true if the set is affinely dependent.
    // A set if affinely dependent if a point of the set
    // is an affine combination of other points in the set
    public @Dimensionless boolean isAffinelyDependent(@Dimensionless Simplex this) {

        @Dimensionless
        float sum = ((@Dimensionless float) (0.0f));

        // For each four point of the possible simplex set
        for (int i = ((@Dimensionless int) (0)), bit = ((@Dimensionless int) (0x1)); i < ((@Dimensionless int) (4)); i++, bit <<= ((@Dimensionless int) (1))) {
            if (overlap(mAllBits, bit)) {
                sum += mDet[mAllBits][i];
            }
        }

        return (sum <= ((@Dimensionless float) (0.0f)));
    }

    // Return true if the simple is empty
    public @Dimensionless boolean isEmpty(@Dimensionless Simplex this) {
        return (mBitsCurrentSimplex == ((@Dimensionless int) (0x0)));
    }

    // Return true if the simplex contains 4 points
    public @Dimensionless boolean isFull(@Dimensionless Simplex this) {
        return (mBitsCurrentSimplex == ((@Dimensionless int) (0xf)));
    }

    // Return true if the point is in the simplex
    public @Dimensionless boolean isPointInSimplex(@Dimensionless Simplex this, @Dimensionless Vector3 point) {

        // For each four possible points in the simplex
        for (int i = ((@Dimensionless int) (0)), bit = ((@Dimensionless int) (0x1)); i < ((@Dimensionless int) (4)); i++, bit <<= ((@Dimensionless int) (1))) {
            // Check if the current point is in the simplex
            if (overlap(mAllBits, bit) && point.equals(mPoints[i])) {
                return true;
            }
        }

        return false;
    }

}
