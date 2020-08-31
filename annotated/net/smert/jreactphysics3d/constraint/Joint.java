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
package net.smert.jreactphysics3d.constraint;

import units.qual.Dimensionless;
import net.smert.jreactphysics3d.body.RigidBody;
import net.smert.jreactphysics3d.configuration.JointsPositionCorrectionTechnique;
import net.smert.jreactphysics3d.engine.ConstraintSolverData;

/**
 * This abstract class represents a joint between two bodies.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public abstract class Joint {

    // Pointer to the first body of the joint
    protected @Dimensionless RigidBody mBody1;

    // Pointer to the second body of the joint
    protected @Dimensionless RigidBody mBody2;

    // Type of the joint
    protected @Dimensionless JointType mType;

    // Body 1 index in the velocity array to solve the constraint
    protected @Dimensionless int mIndexBody1;

    // Body 2 index in the velocity array to solve the constraint
    protected @Dimensionless int mIndexBody2;

    // Position correction technique used for the constraint (used for joints)
    protected @Dimensionless JointsPositionCorrectionTechnique mPositionCorrectionTechnique;

    // True if the two bodies of the constraint are allowed to collide with each other
    protected @Dimensionless boolean mIsCollisionEnabled;

    // True if the joint has already been added into an island
    protected @Dimensionless boolean mIsAlreadyInIsland;

    // Constructor
    public Joint(@Dimensionless JointInfo jointInfo) {
        mBody1 = jointInfo.body1;
        mBody2 = jointInfo.body2;
        mType = jointInfo.type;
        mPositionCorrectionTechnique = jointInfo.positionCorrectionTechnique;
        mIsCollisionEnabled = jointInfo.isCollisionEnabled;
        mIsAlreadyInIsland = false;

        assert (mBody1 != null);
        assert (mBody2 != null);
    }

    // Return the reference to the body 1
    public @Dimensionless RigidBody getBody1(@Dimensionless Joint this) {
        return mBody1;
    }

    // Return the reference to the body 2
    public @Dimensionless RigidBody getBody2(@Dimensionless Joint this) {
        return mBody2;
    }

    // Return true if the joint is active
    public @Dimensionless boolean isActive(@Dimensionless Joint this) {
        return (mBody1.isActive() && mBody2.isActive());
    }

    // Return the type of the joint
    public @Dimensionless JointType getType(@Dimensionless Joint this) {
        return mType;
    }

    // Return true if the collision between the two bodies of the joint is enabled
    public @Dimensionless boolean isCollisionEnabled(@Dimensionless Joint this) {
        return mIsCollisionEnabled;
    }

    // Return true if the joint has already been added into an island
    public @Dimensionless boolean isAlreadyInIsland(@Dimensionless Joint this) {
        return mIsAlreadyInIsland;
    }

    public void setIsAlreadyInIsland(@Dimensionless Joint this, @Dimensionless boolean isAlreadyInIsland) {
        mIsAlreadyInIsland = isAlreadyInIsland;
    }

    // Initialize before solving the joint
    public abstract void initBeforeSolve(@Dimensionless Joint this, @Dimensionless ConstraintSolverData constraintSolverData);

    // Warm start the joint (apply the previous impulse at the beginning of the step)
    public abstract void warmstart(@Dimensionless Joint this, @Dimensionless ConstraintSolverData constraintSolverData);

    // Solve the velocity constraint
    public abstract void solveVelocityConstraint(@Dimensionless Joint this, @Dimensionless ConstraintSolverData constraintSolverData);

    // Solve the position constraint
    public abstract void solvePositionConstraint(@Dimensionless Joint this, @Dimensionless ConstraintSolverData constraintSolverData);

}
