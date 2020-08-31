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
package net.smert.jreactphysics3d.collision;

import units.qual.Dimensionless;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;
import net.smert.jreactphysics3d.body.CollisionBody;
import net.smert.jreactphysics3d.body.RigidBody;
import net.smert.jreactphysics3d.collision.broadphase.BodyPair;
import net.smert.jreactphysics3d.collision.broadphase.BroadPhaseAlgorithm;
import net.smert.jreactphysics3d.collision.broadphase.NoBroadPhaseAlgorithm;
import net.smert.jreactphysics3d.collision.narrowphase.GJK.GJKAlgorithm;
import net.smert.jreactphysics3d.collision.narrowphase.NarrowPhaseAlgorithm;
import net.smert.jreactphysics3d.collision.narrowphase.SphereVsSphereAlgorithm;
import net.smert.jreactphysics3d.collision.shapes.CollisionShape;
import net.smert.jreactphysics3d.collision.shapes.CollisionShapeType;
import net.smert.jreactphysics3d.constraint.ContactPointInfo;
import net.smert.jreactphysics3d.engine.CollisionWorld;
import net.smert.jreactphysics3d.engine.Profiler;

/**
 * This class computes the collision detection algorithms. We first perform a broad-phase algorithm to know which pairs
 * of bodies can collide and then we run a narrow-phase algorithm to compute the collision contacts between bodies.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class CollisionDetection {

    // Broad-phase algorithm
    private final @Dimensionless BroadPhaseAlgorithm broadPhaseAlgorithm;

    // Pointer to the physics world
    private final @Dimensionless CollisionWorld world;

    // Narrow-phase GJK algorithm
    private final @Dimensionless GJKAlgorithm narrowPhaseGJKAlgorithm;

    // Broad-phase overlapping pairs
    private final @Dimensionless Map<@Dimensionless BodyIndexPair, @Dimensionless BroadPhasePair> overlappingPairs;

    // Set of pair of bodies that cannot collide between each other
    private final @Dimensionless Set<@Dimensionless BodyIndexPair> noCollisionPairs;

    // Narrow-phase Sphere vs Sphere algorithm
    private final @Dimensionless SphereVsSphereAlgorithm narrowPhaseSphereVsSphereAlgorithm;

    // Constructor
    public CollisionDetection(@Dimensionless CollisionWorld world) {
        assert (world != null);
        broadPhaseAlgorithm = new @Dimensionless NoBroadPhaseAlgorithm(this);
        this.world = world;
        narrowPhaseGJKAlgorithm = new @Dimensionless GJKAlgorithm();
        overlappingPairs = new @Dimensionless HashMap<>();
        noCollisionPairs = new @Dimensionless HashSet<>();
        narrowPhaseSphereVsSphereAlgorithm = new @Dimensionless SphereVsSphereAlgorithm();
    }

    // Compute the broad-phase collision detection
    private void computeBroadPhase(@Dimensionless CollisionDetection this) {

        Profiler.StartProfilingBlock("CollisionDetection::computeBroadPhase()");

        // Notify the broad-phase algorithm about the bodies that have moved since last frame
        for (@Dimensionless CollisionBody it : world.getBodies()) {

            // If the body has moved
            if (it.getHasMoved()) {

                // Notify the broad-phase that the body has moved
                broadPhaseAlgorithm.updateObject(it, it.getAABB());
            }
        }
    }

    // Compute the narrow-phase collision detection
    private void computeNarrowPhase(@Dimensionless CollisionDetection this) {

        Profiler.StartProfilingBlock("CollisionDetection::computeNarrowPhase()");

        for (Map.@Dimensionless Entry pairs : overlappingPairs.entrySet()) {
            @Dimensionless
            ContactPointInfo contactInfo = new @Dimensionless ContactPointInfo();

            @Dimensionless
            BroadPhasePair pair = (@Dimensionless BroadPhasePair) pairs.getValue();
            assert (pair != null);

            @Dimensionless
            CollisionBody body1 = pair.getBody1();
            @Dimensionless
            CollisionBody body2 = pair.getBody2();

            // Update the contact cache of the overlapping pair
            world.updateOverlappingPair(pair);

            // Check if the two bodies are allowed to collide, otherwise, we do not test for collision
            if (noCollisionPairs.contains(pair.newBodiesIndexPair()) == true) {
                continue;
            }

            // Check if the two bodies are sleeping, if so, we do no test collision between them
            if (body1.isSleeping() && body2.isSleeping()) {
                continue;
            }

            // Select the narrow phase algorithm to use according to the two collision shapes
            @Dimensionless
            NarrowPhaseAlgorithm narrowPhaseAlgorithm = selectNarrowPhaseAlgorithm(
                    body1.getCollisionShape(), body2.getCollisionShape());

            // Notify the narrow-phase algorithm about the overlapping pair we are going to test
            narrowPhaseAlgorithm.setCurrentOverlappingPair(pair);

            // Use the narrow-phase collision detection algorithm to check
            // if there really is a collision
            if (narrowPhaseAlgorithm.testCollision(
                    body1.getCollisionShape(), body1.getTransform(),
                    body2.getCollisionShape(), body2.getTransform(),
                    contactInfo)) {
                assert (contactInfo != null);

                // Set the bodies of the contact
                contactInfo.setBody1((@Dimensionless RigidBody) body1);
                contactInfo.setBody2((@Dimensionless RigidBody) body2);
                assert (contactInfo.getBody1() != null);
                assert (contactInfo.getBody2() != null);

                // Notify the world about the new narrow-phase contact
                world.notifyNewContact(pair, contactInfo);

                // Delete and remove the contact info from the memory allocator
            }
        }
    }

    // Select the narrow-phase collision algorithm to use given two collision shapes
    private @Dimensionless NarrowPhaseAlgorithm selectNarrowPhaseAlgorithm(@Dimensionless CollisionDetection this, @Dimensionless CollisionShape collisionShape1, @Dimensionless CollisionShape collisionShape2) {

        // Sphere vs Sphere algorithm
        if (collisionShape1.getType() == CollisionShapeType.SPHERE && collisionShape2.getType() == CollisionShapeType.SPHERE) {
            return narrowPhaseSphereVsSphereAlgorithm;
        } else {    // GJK algorithm
            return narrowPhaseGJKAlgorithm;
        }
    }

    // Add a body to the collision detection
    public void addBody(@Dimensionless CollisionDetection this, @Dimensionless CollisionBody body) {

        // Add the body to the broad-phase
        broadPhaseAlgorithm.addObject(body, body.getAABB());
    }

    // Remove a body from the collision detection
    public void removeBody(@Dimensionless CollisionDetection this, @Dimensionless CollisionBody body) {

        // Remove the body from the broad-phase
        broadPhaseAlgorithm.removeObject(body);
    }

    // Add a pair of bodies that cannot collide with each other
    public void addNoCollisionPair(@Dimensionless CollisionDetection this, @Dimensionless CollisionBody body1, @Dimensionless CollisionBody body2) {
        noCollisionPairs.add(BroadPhasePair.ComputeBodiesIndexPair(body1, body2));
    }

    // Remove a pair of bodies that cannot collide with each other
    public void removeNoCollisionPair(@Dimensionless CollisionDetection this, @Dimensionless CollisionBody body1, @Dimensionless CollisionBody body2) {
        noCollisionPairs.remove(BroadPhasePair.ComputeBodiesIndexPair(body1, body2));
    }

    // Allow the broadphase to notify the collision detection about an overlapping pair.
    // This method is called by a broad-phase collision detection algorithm
    public void broadPhaseNotifyAddedOverlappingPair(@Dimensionless CollisionDetection this, @Dimensionless BodyPair addedPair) {

        // Get the pair of body index
        @Dimensionless
        BodyIndexPair indexPair = addedPair.newBodiesIndexPair();

        // Create the corresponding broad-phase pair object
        @Dimensionless
        BroadPhasePair broadPhasePair = new @Dimensionless BroadPhasePair(addedPair.getBody1(), addedPair.getBody2());
        assert (broadPhasePair != null);

        // Add the pair into the set of overlapping pairs (if not there yet)
        @Dimensionless
        BroadPhasePair check = overlappingPairs.put(indexPair, broadPhasePair);

        assert (check == null);

        // Notify the world about the new broad-phase overlapping pair
        world.notifyAddedOverlappingPair(broadPhasePair);
    }

    // Allow the broadphase to notify the collision detection about a removed overlapping pair
    public void broadPhaseNotifyRemovedOverlappingPair(@Dimensionless CollisionDetection this, @Dimensionless BodyPair removedPair) {

        // Get the pair of body index
        @Dimensionless
        BodyIndexPair indexPair = removedPair.newBodiesIndexPair();

        // Get the broad-phase pair
        @Dimensionless
        BroadPhasePair broadPhasePair = overlappingPairs.get(indexPair);
        assert (broadPhasePair != null);

        // Notify the world about the removed broad-phase pair
        world.notifyRemovedOverlappingPair(broadPhasePair);

        // Remove the overlapping pair from the memory allocator
        overlappingPairs.remove(indexPair);
    }

    // Compute the collision detection
    public void computeCollisionDetection(@Dimensionless CollisionDetection this) {

        Profiler.StartProfilingBlock("CollisionDetection::computeCollisionDetection()");

        // Compute the broad-phase collision detection
        computeBroadPhase();

        // Compute the narrow-phase collision detection
        computeNarrowPhase();
    }

}
