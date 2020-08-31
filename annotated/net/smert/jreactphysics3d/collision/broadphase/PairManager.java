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
package net.smert.jreactphysics3d.collision.broadphase;

import units.qual.Dimensionless;
import net.smert.jreactphysics3d.body.CollisionBody;
import net.smert.jreactphysics3d.collision.CollisionDetection;

/**
 * This class is a data-structure contains the pairs of bodies that are overlapping during the broad-phase collision
 * detection. This class implements the pair manager described by Pierre Terdiman in www.codercorner.com/SAP.pdf.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class PairManager {

    // Invalid ID
    private final static @Dimensionless int INVALID_INDEX = ((@Dimensionless int) (Integer.MAX_VALUE));

    // Hash mask for the hash function
    private @Dimensionless int hashMask;

    // Number of elements in the hash table
    private @Dimensionless int numElementsHashTable;

    // Number of overlapping pairs
    private @Dimensionless int numOverlappingPairs;

    // Hash table that contains the offset of the first pair of the list of
    // pairs with the same hash value in the "overlappingPairs" array
    private @Dimensionless int @Dimensionless [] hashTable;

    // Array that contains for each offset, the offset of the next pair with
    // the same hash value for a given same hash value
    private @Dimensionless int @Dimensionless [] offsetNextPair;

    // Array that contains the overlapping pairs
    private @Dimensionless BodyPair @Dimensionless [] overlappingPairs;

    // Reference to the collision detection
    private final @Dimensionless CollisionDetection collisionDetection;

    // Constructor
    public PairManager(CollisionDetection collisionDetection) {
        hashMask = ((@Dimensionless int) (0));
        numElementsHashTable = ((@Dimensionless int) (0));
        numOverlappingPairs = ((@Dimensionless int) (0));
        hashTable = null;
        offsetNextPair = null;
        overlappingPairs = null;
        this.collisionDetection = collisionDetection;
    }

    // This method returns an hash value for a 32 bits key.
    // using Thomas Wang's hash technique.
    // This hash function can be found at :
    // http://www.concentric.net/~ttwang/tech/inthash.htm
    private @Dimensionless int computeHash32Bits(@Dimensionless PairManager this, @Dimensionless int key) {
        key += ~(key << ((@Dimensionless int) (15)));
        key ^= (key >> ((@Dimensionless int) (10)));
        key += (key << ((@Dimensionless int) (3)));
        key ^= (key >> ((@Dimensionless int) (6)));
        key += ~(key << ((@Dimensionless int) (11)));
        key ^= (key >> ((@Dimensionless int) (16)));
        return key;
    }

    // Compute the hash value of two bodies
    private @Dimensionless int computeHashBodies(@Dimensionless PairManager this, @Dimensionless int id1, @Dimensionless int id2) {
        return computeHash32Bits(id1 | (id2 << ((@Dimensionless int) (16))));
    }

    // Compute the offset of a given pair in the array of overlapping pairs
    private @Dimensionless int computePairOffset(@Dimensionless PairManager this, @Dimensionless BodyPair pair) {
        for (@Dimensionless int i = ((@Dimensionless int) (0)); i < overlappingPairs.length; i++) {
            if (pair.equals(overlappingPairs[i])) {
                return i;
            }
        }
        return ((@Dimensionless int) (-1));
    }

    // Find a pair given two body IDs and an hash value.
    // This internal version is used to avoid computing multiple times in the
    // caller method
    private @Dimensionless BodyPair findPairWithHashValue(@Dimensionless PairManager this, @Dimensionless int id1, @Dimensionless int id2, @Dimensionless int hashValue) {

        // Check if the hash table has been allocated yet
        if (hashTable == null) {
            return null;
        }

        // Look for the pair in the set of overlapping pairs
        return lookForAPair(id1, id2, hashValue);
    }

    // Return true if pair1 and pair2 are the same
    private @Dimensionless boolean isDifferentPair(@Dimensionless PairManager this, @Dimensionless BodyPair pair1, @Dimensionless int pair2ID1, @Dimensionless int pair2ID2) {
        return (pair2ID1 != pair1.getBody1().getBodyID() || pair2ID2 != pair1.getBody2().getBodyID());
    }

    // Look for a pair in the set of overlapping pairs
    public @Dimensionless BodyPair lookForAPair(@Dimensionless PairManager this, @Dimensionless int id1, @Dimensionless int id2, @Dimensionless int hashValue) {

        // Look for the pair in the set of overlapping pairs
        @Dimensionless
        int offset = hashTable[hashValue];
        while (offset != INVALID_INDEX && isDifferentPair(overlappingPairs[offset], id1, id2)) {
            offset = offsetNextPair[offset];
        }

        // If the pair has not been found in the overlapping pairs
        if (offset == INVALID_INDEX) {
            return null;
        }

        assert (offset < numOverlappingPairs);

        // The pair has been found in the set of overlapping pairs, then
        // we return a pointer to it
        return overlappingPairs[offset];
    }

    // Reallocate more pairs
    private void reallocatePairs() {

        // Reallocate the hash table and initialize it
        hashTable = new @Dimensionless int @Dimensionless [numElementsHashTable];
        assert (hashTable != null);
        for (@Dimensionless int i = ((@Dimensionless int) (0)); i < numElementsHashTable; i++) {
            hashTable[i] = INVALID_INDEX;
        }

        // Reallocate the overlapping pairs
        @Dimensionless
        BodyPair @Dimensionless [] newOverlappingPairs = new @Dimensionless BodyPair @Dimensionless [numElementsHashTable];
        @Dimensionless
        int @Dimensionless [] newOffsetNextPair = new @Dimensionless int @Dimensionless [numElementsHashTable];

        assert (newOverlappingPairs != null);
        assert (newOffsetNextPair != null);

        // If there is already some overlapping pairs
        if (numOverlappingPairs > ((@Dimensionless int) (0))) {
            // Copy the pairs to the new location
            System.arraycopy(overlappingPairs, ((@Dimensionless int) (0)), newOverlappingPairs, ((@Dimensionless int) (0)), numOverlappingPairs);
        }

        // Recompute the hash table with the new hash values
        for (@Dimensionless int i = ((@Dimensionless int) (0)); i < numOverlappingPairs; i++) {
            @Dimensionless
            int newHashValue = computeHashBodies(overlappingPairs[i].getBody1().getBodyID(),
                    overlappingPairs[i].getBody2().getBodyID()) & hashMask;
            newOffsetNextPair[i] = hashTable[newHashValue];
            hashTable[newHashValue] = i;
        }

        // Delete the old pairs
        // Replace by the new data
        overlappingPairs = newOverlappingPairs;
        offsetNextPair = newOffsetNextPair;
    }

    // Internal method to remove a pair from the set of overlapping pair
    private void removePairWithHashValue(@Dimensionless PairManager this, @Dimensionless int id1, @Dimensionless int id2, @Dimensionless int hashValue, @Dimensionless int indexPair) {

        // Get the initial offset of the pairs with
        // the corresponding hash value
        @Dimensionless
        int offset = hashTable[hashValue];
        assert (offset != INVALID_INDEX);

        // Look for the pair in the set of overlapping pairs
        @Dimensionless
        int previousPair = INVALID_INDEX;
        while (offset != indexPair) {
            previousPair = offset;
            offset = offsetNextPair[offset];
        }

        // If the pair was the first one with this hash
        // value in the hash table
        if (previousPair == INVALID_INDEX) {
            // Replace the pair to remove in the
            // hash table by the next one
            hashTable[hashValue] = offsetNextPair[indexPair];
        } else {    // If the pair was not the first one
            // Replace the pair to remove in the
            // hash table by the next one
            assert (offsetNextPair[previousPair] == indexPair);
            offsetNextPair[previousPair] = offsetNextPair[indexPair];
        }

        @Dimensionless
        int indexLastPair = numOverlappingPairs - ((@Dimensionless int) (1));

        // If the pair to remove is the last one in the list
        if (indexPair == indexLastPair) {

            // We simply decrease the number of overlapping pairs
            numOverlappingPairs--;
        } else {    // If the pair to remove is in the middle of the list

            // Now, we want to move the last pair into the location that is
            // now free because of the pair we want to remove
            // Get the last pair
            @Dimensionless
            BodyPair lastPair = overlappingPairs[indexLastPair];
            @Dimensionless
            int lastPairHashValue = computeHashBodies(lastPair.getBody1().getBodyID(),
                    lastPair.getBody2().getBodyID()) & hashMask;

            // Compute the initial offset of the last pair
            offset = hashTable[lastPairHashValue];
            assert (offset != INVALID_INDEX);

            // Go through the pairs with the same hash value
            // and find the offset of the last pair
            @Dimensionless
            int previous = INVALID_INDEX;
            while (offset != indexLastPair) {
                previous = offset;
                offset = offsetNextPair[offset];
            }

            // If the last pair is not the first one with this hash value
            if (previous != INVALID_INDEX) {

                // Remove the offset of the last pair in the "nextOffset" array
                assert (offsetNextPair[previous] == indexLastPair);
                offsetNextPair[previous] = offsetNextPair[indexLastPair];
            } else {    // If the last pair is the first offset with this hash value

                // Remove the offset of the last pair in the "nextOffset" array
                hashTable[lastPairHashValue] = offsetNextPair[indexLastPair];
            }

            // Replace the pair to remove by the last pair in
            // the overlapping pairs array
            overlappingPairs[indexPair] = overlappingPairs[indexLastPair];
            offsetNextPair[indexPair] = hashTable[lastPairHashValue];
            hashTable[lastPairHashValue] = indexPair;

            numOverlappingPairs--;
        }
    }

    // Try to reduce the allocated memory by the pair manager
    private void shrinkMemory(@Dimensionless PairManager this) {

        // Check if the allocated memory can be reduced
        @Dimensionless
        int correctNumElementsHashTable = ComputeNextPowerOfTwo(numOverlappingPairs);
        if (numElementsHashTable == correctNumElementsHashTable) {
            return;
        }

        // Reduce the allocated memory
        numElementsHashTable = correctNumElementsHashTable;
        hashMask = numElementsHashTable - ((@Dimensionless int) (1));
        reallocatePairs();
    }

    // Add a pair of bodies in the pair manager and returns a pointer to that pair.
    // If the pair to add does not already exist in the set of
    // overlapping pairs, it will be created and if it already exists, we only
    // return a pointer to that pair.
    public BodyPair addPair(@Dimensionless PairManager this, CollisionBody body1, CollisionBody body2) {

        // Sort the bodies to have the body with smallest ID first
        // If the ID of body1 is larger than the ID of body 2
        if (body1.getBodyID() > body2.getBodyID()) {

            // Swap the two bodies pointers
            @Dimensionless
            CollisionBody temp = body2;
            body2 = body1;
            body1 = temp;
        }

        // Get the bodies IDs
        @Dimensionless
        int id1 = body1.getBodyID();
        @Dimensionless
        int id2 = body2.getBodyID();

        // Compute the hash value of the two bodies
        @Dimensionless
        int hashValue = computeHashBodies(id1, id2) & hashMask;

        // Try to find the pair in the current overlapping pairs.
        @Dimensionless
        BodyPair pair = findPairWithHashValue(id1, id2, hashValue);

        // If the pair is already in the set of overlapping pairs
        if (pair != null) {
            // We only return a pointer to that pair
            return pair;
        }

        // If we need to allocate more pairs in the set of overlapping pairs
        if (numOverlappingPairs >= numElementsHashTable) {
            // Increase the size of the hash table (always a power of two)
            numElementsHashTable = ComputeNextPowerOfTwo(numOverlappingPairs + ((@Dimensionless int) (1)));

            // Compute the new hash mask with the new hash size
            hashMask = numElementsHashTable - ((@Dimensionless int) (1));

            // Reallocate more pairs
            reallocatePairs();

            // Compute the new hash value (using the new hash size and hash mask)
            hashValue = computeHashBodies(id1, id2) & hashMask;
        }

        // Create the new overlapping pair
        if (overlappingPairs[numOverlappingPairs] == null) {
            overlappingPairs[numOverlappingPairs] = new @Dimensionless BodyPair();
        }
        @Dimensionless
        BodyPair newPair = overlappingPairs[numOverlappingPairs];
        newPair.setBody1(body1);
        newPair.setBody2(body2);

        // Put the new pair as the initial pair with this hash value
        offsetNextPair[numOverlappingPairs] = hashTable[hashValue];
        hashTable[hashValue] = numOverlappingPairs++;

        // Notify the collision detection about this new overlapping pair
        collisionDetection.broadPhaseNotifyAddedOverlappingPair(newPair);

        // Return a pointer to the new created pair
        return newPair;
    }

    // Find a pair given two body IDs
    public @Dimensionless BodyPair findPair(@Dimensionless PairManager this, @Dimensionless int id1, @Dimensionless int id2) {

        // Check if the hash table has been allocated yet
        if (hashTable == null) {
            return null;
        }

        // Sort the IDs
        if (id1 > id2) {
            @Dimensionless
            int temp = id2;
            id2 = id1;
            id1 = temp;
        }

        // Compute the hash value of the pair to find
        @Dimensionless
        int hashValue = computeHashBodies(id1, id2) & hashMask;

        // Look for the pair in the set of overlapping pairs
        return lookForAPair(id1, id2, hashValue);
    }

    // Return the number of overlapping pairs
    public @Dimensionless int getNumOverlappingPairs(@Dimensionless PairManager this) {
        return numOverlappingPairs;
    }

    // Remove a pair of bodies from the pair manager.
    // This method returns true if the pair has been found and removed.
    public boolean removePair(@Dimensionless PairManager this, int id1, int id2) {

        // Sort the bodies IDs
        if (id1 > id2) {
            @Dimensionless
            int temp = id2;
            id2 = id1;
            id1 = temp;
        }

        // Compute the hash value of the pair to remove
        @Dimensionless
        int hashValue = computeHashBodies(id1, id2) & hashMask;

        // Find the pair to remove
        @Dimensionless
        BodyPair pair = findPairWithHashValue(id1, id2, hashValue);

        // If we have not found the pair
        if (pair == null) {
            return false;
        }

        assert (pair.getBody1().getBodyID() == id1);
        assert (pair.getBody2().getBodyID() == id2);

        // Notify the collision detection about this removed overlapping pair
        collisionDetection.broadPhaseNotifyRemovedOverlappingPair(pair);

        // Remove the pair from the set of overlapping pairs
        removePairWithHashValue(id1, id2, hashValue, computePairOffset(pair));

        // Try to shrink the memory used by the pair manager
        shrinkMemory();

        return true;
    }

    // Return the next power of two of a 32bits integer using a SWAR algorithm
    public static int ComputeNextPowerOfTwo(int number) {
        number |= (number >> ((@Dimensionless int) (1)));
        number |= (number >> ((@Dimensionless int) (2)));
        number |= (number >> ((@Dimensionless int) (4)));
        number |= (number >> ((@Dimensionless int) (8)));
        number |= (number >> ((@Dimensionless int) (16)));
        return number + ((@Dimensionless int) (1));
    }

}
