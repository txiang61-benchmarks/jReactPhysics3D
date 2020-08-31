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
import net.smert.jreactphysics3d.configuration.JointsPositionCorrectionTechnique;
import net.smert.jreactphysics3d.engine.ConstraintSolverData;
import net.smert.jreactphysics3d.mathematics.Mathematics;
import net.smert.jreactphysics3d.mathematics.Matrix2x2;
import net.smert.jreactphysics3d.mathematics.Matrix3x3;
import net.smert.jreactphysics3d.mathematics.Quaternion;
import net.smert.jreactphysics3d.mathematics.Transform;
import net.smert.jreactphysics3d.mathematics.Vector2;
import net.smert.jreactphysics3d.mathematics.Vector3;

/**
 * This class represents a slider joint. This joint has a one degree of freedom. It only allows relative translation of
 * the bodies along a single direction and no rotation.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class SliderJoint extends Joint {

    // Beta value for the position correction bias factor
    private static final @Dimensionless float BETA = ((@Dimensionless float) (0.2f));

    // Anchor point of body 1 (in local-space coordinates of body 1)
    private @Dimensionless Vector3 mLocalAnchorPointBody1;

    // Anchor point of body 2 (in local-space coordinates of body 2)
    private @Dimensionless Vector3 mLocalAnchorPointBody2;

    // Slider axis (in local-space coordinates of body 1)
    private @Dimensionless Vector3 mSliderAxisBody1;

    // Inertia tensor of body 1 (in world-space coordinates)
    private @Dimensionless Matrix3x3 mI1;

    // Inertia tensor of body 2 (in world-space coordinates)
    private @Dimensionless Matrix3x3 mI2;

    // Inverse of the initial orientation difference between the two bodies
    private @Dimensionless Quaternion mInitOrientationDifferenceInv;

    // First vector orthogonal to the slider axis local-space of body 1
    private @Dimensionless Vector3 mN1;

    // Second vector orthogonal to the slider axis and mN1 in local-space of body 1
    private @Dimensionless Vector3 mN2;

    // Vector r1 in world-space coordinates
    private @Dimensionless Vector3 mR1;

    // Vector r2 in world-space coordinates
    private @Dimensionless Vector3 mR2;

    // Cross product of r2 and n1
    private @Dimensionless Vector3 mR2CrossN1;

    // Cross product of r2 and n2
    private @Dimensionless Vector3 mR2CrossN2;

    // Cross product of r2 and the slider axis
    private @Dimensionless Vector3 mR2CrossSliderAxis;

    // Cross product of vector (r1 + u) and n1
    private @Dimensionless Vector3 mR1PlusUCrossN1;

    // Cross product of vector (r1 + u) and n2
    private @Dimensionless Vector3 mR1PlusUCrossN2;

    // Cross product of vector (r1 + u) and the slider axis
    private @Dimensionless Vector3 mR1PlusUCrossSliderAxis;

    // Bias of the 2 translation constraints
    private @Dimensionless Vector2 mBTranslation;

    // Bias of the 3 rotation constraints
    private @Dimensionless Vector3 mBRotation;

    // Bias of the lower limit constraint
    private @Dimensionless float mBLowerLimit;

    // Bias of the upper limit constraint
    private @Dimensionless float mBUpperLimit;

    // Inverse of mass matrix K=JM^-1J^t for the translation constraint (2x2 matrix)
    private @Dimensionless Matrix2x2 mInverseMassMatrixTranslationConstraint;

    // Inverse of mass matrix K=JM^-1J^t for the rotation constraint (3x3 matrix)
    private @Dimensionless Matrix3x3 mInverseMassMatrixRotationConstraint;

    // Inverse of mass matrix K=JM^-1J^t for the upper and lower limit constraints (1x1 matrix)
    private @Dimensionless float mInverseMassMatrixLimit;

    // Inverse of mass matrix K=JM^-1J^t for the motor
    private @Dimensionless float mInverseMassMatrixMotor;

    // Accumulated impulse for the 2 translation constraints
    private @Dimensionless Vector2 mImpulseTranslation;

    // Accumulated impulse for the 3 rotation constraints
    private @Dimensionless Vector3 mImpulseRotation;

    // Accumulated impulse for the lower limit constraint
    private @Dimensionless float mImpulseLowerLimit;

    // Accumulated impulse for the upper limit constraint
    private @Dimensionless float mImpulseUpperLimit;

    // Accumulated impulse for the motor
    private @Dimensionless float mImpulseMotor;

    // True if the slider limits are enabled
    private @Dimensionless boolean mIsLimitEnabled;

    // True if the motor of the joint in enabled
    private @Dimensionless boolean mIsMotorEnabled;

    // Slider axis in world-space coordinates
    private @Dimensionless Vector3 mSliderAxisWorld;

    // Lower limit (minimum translation distance)
    private @Dimensionless float mLowerLimit;

    // Upper limit (maximum translation distance)
    private @Dimensionless float mUpperLimit;

    // True if the lower limit is violated
    private @Dimensionless boolean mIsLowerLimitViolated;

    // True if the upper limit is violated
    private @Dimensionless boolean mIsUpperLimitViolated;

    // Motor speed
    private @Dimensionless float mMotorSpeed;

    // Maximum motor force (in Newtons) that can be applied to reach to desired motor speed
    private @Dimensionless float mMaxMotorForce;

    // Reset the limits
    protected void resetLimits(@Dimensionless SliderJoint this) {

        // Reset the accumulated impulses for the limits
        mImpulseLowerLimit = ((@Dimensionless float) (0.0f));
        mImpulseUpperLimit = ((@Dimensionless float) (0.0f));

        // Wake up the two bodies of the joint
        mBody1.setIsSleeping(false);
        mBody2.setIsSleeping(false);
    }

    // Constructor
    public SliderJoint(SliderJointInfo jointInfo) {
        super(jointInfo);

        mImpulseTranslation = new @Dimensionless Vector2();
        mImpulseRotation = new @Dimensionless Vector3();
        mImpulseLowerLimit = ((@Dimensionless float) (0.0f));
        mImpulseUpperLimit = ((@Dimensionless float) (0.0f));
        mImpulseMotor = ((@Dimensionless float) (0.0f));
        mIsLimitEnabled = jointInfo.isLimitEnabled;
        mIsMotorEnabled = jointInfo.isMotorEnabled;
        mLowerLimit = jointInfo.minTranslationLimit;
        mUpperLimit = jointInfo.maxTranslationLimit;
        mIsLowerLimitViolated = false;
        mIsUpperLimitViolated = false;
        mMotorSpeed = jointInfo.motorSpeed;
        mMaxMotorForce = jointInfo.maxMotorForce;

        assert (mUpperLimit >= ((@Dimensionless float) (0.0f)));
        assert (mLowerLimit <= ((@Dimensionless float) (0.0f)));
        assert (mMaxMotorForce >= ((@Dimensionless float) (0.0f)));

        // Compute the local-space anchor point for each body
        @Dimensionless
        Transform transform1 = mBody1.getTransform();
        @Dimensionless
        Transform transform2 = mBody2.getTransform();
        mLocalAnchorPointBody1 = new @Dimensionless Transform(transform1).inverse().multiply(jointInfo.anchorPointWorldSpace, new @Dimensionless Vector3());
        mLocalAnchorPointBody2 = new @Dimensionless Transform(transform2).inverse().multiply(jointInfo.anchorPointWorldSpace, new @Dimensionless Vector3());

        // Compute the inverse of the initial orientation difference between the two bodies
        mInitOrientationDifferenceInv = new @Dimensionless Quaternion(transform2.getOrientation()).multiply(new @Dimensionless Quaternion(transform1.getOrientation()).inverse());
        mInitOrientationDifferenceInv.normalize();
        mInitOrientationDifferenceInv.inverse();

        // Compute the slider axis in local-space of body 1
        new @Dimensionless Quaternion(mBody1.getTransform().getOrientation()).inverse().multiply(jointInfo.sliderAxisWorldSpace, mSliderAxisBody1);
        mSliderAxisBody1.normalize();
    }

    // Initialize before solving the constraint
    @Override
    public void initBeforeSolve(@Dimensionless SliderJoint this, @Dimensionless ConstraintSolverData constraintSolverData) {

        // Initialize the bodies index in the veloc ity array
        mIndexBody1 = constraintSolverData.mapBodyToConstrainedVelocityIndex.get(mBody1);
        mIndexBody2 = constraintSolverData.mapBodyToConstrainedVelocityIndex.get(mBody2);

        // Get the bodies positions and orientations
        @Dimensionless
        Vector3 x1 = mBody1.getTransform().getPosition();
        @Dimensionless
        Vector3 x2 = mBody2.getTransform().getPosition();
        @Dimensionless
        Quaternion orientationBody1 = mBody1.getTransform().getOrientation();
        @Dimensionless
        Quaternion orientationBody2 = mBody2.getTransform().getOrientation();

        // Get the inertia tensor of bodies
        mI1 = mBody1.getInertiaTensorInverseWorld();
        mI2 = mBody2.getInertiaTensorInverseWorld();

        // Vector from body center to the anchor point
        orientationBody1.multiply(mLocalAnchorPointBody1, mR1);
        orientationBody2.multiply(mLocalAnchorPointBody2, mR2);

        // Compute the vector u (difference between anchor points)
        @Dimensionless
        Vector3 u = new @Dimensionless Vector3(new @Dimensionless Vector3(new @Dimensionless Vector3(x2).add(mR2)).subtract(x1)).subtract(mR1);

        // Compute the two orthogonal vectors to the slider axis in world-space
        orientationBody1.multiply(mSliderAxisBody1, mSliderAxisWorld);
        mSliderAxisWorld.normalize();
        mN1 = new @Dimensionless Vector3(mSliderAxisWorld).setUnitOrthogonal();
        mN2 = new @Dimensionless Vector3(mSliderAxisWorld).cross(mN1);

        // Check if the limit constraints are violated or not
        @Dimensionless
        float uDotSliderAxis = u.dot(mSliderAxisWorld);
        @Dimensionless
        float lowerLimitError = uDotSliderAxis - mLowerLimit;
        @Dimensionless
        float upperLimitError = mUpperLimit - uDotSliderAxis;
        @Dimensionless
        boolean oldIsLowerLimitViolated = mIsLowerLimitViolated;
        mIsLowerLimitViolated = lowerLimitError <= ((@Dimensionless float) (0.0f));
        if (mIsLowerLimitViolated != oldIsLowerLimitViolated) {
            mImpulseLowerLimit = ((@Dimensionless float) (0.0f));
        }
        @Dimensionless
        boolean oldIsUpperLimitViolated = mIsUpperLimitViolated;
        mIsUpperLimitViolated = upperLimitError <= ((@Dimensionless float) (0.0f));
        if (mIsUpperLimitViolated != oldIsUpperLimitViolated) {
            mImpulseUpperLimit = ((@Dimensionless float) (0.0f));
        }

        // Compute the cross products used in the Jacobians
        mR2CrossN1 = new @Dimensionless Vector3(mR2).cross(mN1);
        mR2CrossN2 = new @Dimensionless Vector3(mR2).cross(mN2);
        mR2CrossSliderAxis = new @Dimensionless Vector3(mR2).cross(mSliderAxisWorld);
        @Dimensionless
        Vector3 r1PlusU = new @Dimensionless Vector3(mR1).add(u);
        mR1PlusUCrossN1 = new @Dimensionless Vector3(r1PlusU).cross(mN1);
        mR1PlusUCrossN2 = new @Dimensionless Vector3(r1PlusU).cross(mN2);
        mR1PlusUCrossSliderAxis = new @Dimensionless Vector3(r1PlusU).cross(mSliderAxisWorld);

        // Compute the inverse of the mass matrix K=JM^-1J^t for the 2 translation
        // constraints (2x2 matrix)
        @Dimensionless
        float sumInverseMass = ((@Dimensionless float) (0.0f));
        @Dimensionless
        Vector3 I1R1PlusUCrossN1 = new @Dimensionless Vector3();
        @Dimensionless
        Vector3 I1R1PlusUCrossN2 = new @Dimensionless Vector3();
        @Dimensionless
        Vector3 I2R2CrossN1 = new @Dimensionless Vector3();
        @Dimensionless
        Vector3 I2R2CrossN2 = new @Dimensionless Vector3();
        if (mBody1.isMotionEnabled()) {
            sumInverseMass += mBody1.getMassInverse();
            I1R1PlusUCrossN1 = mI1.multiply(mR1PlusUCrossN1, new @Dimensionless Vector3());
            I1R1PlusUCrossN2 = mI1.multiply(mR1PlusUCrossN2, new @Dimensionless Vector3());
        }
        if (mBody2.isMotionEnabled()) {
            sumInverseMass += mBody2.getMassInverse();
            I2R2CrossN1 = mI2.multiply(mR2CrossN1, new @Dimensionless Vector3());
            I2R2CrossN2 = mI2.multiply(mR2CrossN2, new @Dimensionless Vector3());
        }
        @Dimensionless
        float el11 = sumInverseMass + mR1PlusUCrossN1.dot(I1R1PlusUCrossN1) + mR2CrossN1.dot(I2R2CrossN1);
        @Dimensionless
        float el12 = mR1PlusUCrossN1.dot(I1R1PlusUCrossN2) + mR2CrossN1.dot(I2R2CrossN2);
        @Dimensionless
        float el21 = mR1PlusUCrossN2.dot(I1R1PlusUCrossN1) + mR2CrossN2.dot(I2R2CrossN1);
        @Dimensionless
        float el22 = sumInverseMass + mR1PlusUCrossN2.dot(I1R1PlusUCrossN2) + mR2CrossN2.dot(I2R2CrossN2);
        @Dimensionless
        Matrix2x2 matrixKTranslation = new @Dimensionless Matrix2x2(el11, el12, el21, el22);
        mInverseMassMatrixTranslationConstraint.zero();
        if (mBody1.isMotionEnabled() || mBody2.isMotionEnabled()) {
            mInverseMassMatrixTranslationConstraint = new @Dimensionless Matrix2x2(matrixKTranslation).inverse();
        }

        // Compute the bias "b" of the translation constraint
        mBTranslation.zero();
        @Dimensionless
        float biasFactor = (BETA / constraintSolverData.timeStep);
        if (mPositionCorrectionTechnique == JointsPositionCorrectionTechnique.BAUMGARTE_JOINTS) {
            mBTranslation.setX(u.dot(mN1));
            mBTranslation.setY(u.dot(mN2));
            mBTranslation.multiply(biasFactor);
        }

        // Compute the inverse of the mass matrix K=JM^-1J^t for the 3 rotation
        // contraints (3x3 matrix)
        mInverseMassMatrixRotationConstraint.zero();
        if (mBody1.isMotionEnabled()) {
            mInverseMassMatrixRotationConstraint.add(mI1);
        }
        if (mBody2.isMotionEnabled()) {
            mInverseMassMatrixRotationConstraint.add(mI2);
        }
        if (mBody1.isMotionEnabled() || mBody2.isMotionEnabled()) {
            mInverseMassMatrixRotationConstraint = new @Dimensionless Matrix3x3(mInverseMassMatrixRotationConstraint).inverse();
        }

        // Compute the bias "b" of the rotation constraint
        mBRotation.zero();
        if (mPositionCorrectionTechnique == JointsPositionCorrectionTechnique.BAUMGARTE_JOINTS) {
            @Dimensionless
            Quaternion currentOrientationDifference = new @Dimensionless Quaternion(orientationBody2).multiply(new @Dimensionless Quaternion(orientationBody1).inverse());
            currentOrientationDifference.normalize();
            @Dimensionless
            Quaternion qError = new @Dimensionless Quaternion(currentOrientationDifference).multiply(mInitOrientationDifferenceInv);
            @Dimensionless
            Vector3 qErrorV = new @Dimensionless Vector3();
            qError.getVectorV(qErrorV);
            mBRotation = qErrorV.multiply(biasFactor * ((@Dimensionless float) (2.0f)));
        }

        // If the limits are enabled
        if (mIsLimitEnabled && (mIsLowerLimitViolated || mIsUpperLimitViolated)) {

            // Compute the inverse of the mass matrix K=JM^-1J^t for the limits (1x1 matrix)
            mInverseMassMatrixLimit = ((@Dimensionless float) (0.0f));
            if (mBody1.isMotionEnabled()) {
                mInverseMassMatrixLimit += mBody1.getMassInverse()
                        + mR1PlusUCrossSliderAxis.dot(mI1.multiply(mR1PlusUCrossSliderAxis, new @Dimensionless Vector3()));
            }
            if (mBody2.isMotionEnabled()) {
                mInverseMassMatrixLimit += mBody2.getMassInverse()
                        + mR2CrossSliderAxis.dot(mI2.multiply(mR2CrossSliderAxis, new @Dimensionless Vector3()));
            }
            mInverseMassMatrixLimit = (mInverseMassMatrixLimit > ((@Dimensionless float) (0.0f))) ? ((@Dimensionless float) (1.0f)) / mInverseMassMatrixLimit : ((@Dimensionless float) (0.0f));

            // Compute the bias "b" of the lower limit constraint
            mBLowerLimit = ((@Dimensionless float) (0.0f));
            if (mPositionCorrectionTechnique == JointsPositionCorrectionTechnique.BAUMGARTE_JOINTS) {
                mBLowerLimit = biasFactor * lowerLimitError;
            }

            // Compute the bias "b" of the upper limit constraint
            mBUpperLimit = ((@Dimensionless float) (0.0f));
            if (mPositionCorrectionTechnique == JointsPositionCorrectionTechnique.BAUMGARTE_JOINTS) {
                mBUpperLimit = biasFactor * upperLimitError;
            }
        }

        // If the motor is enabled
        if (mIsMotorEnabled) {

            // Compute the inverse of mass matrix K=JM^-1J^t for the motor (1x1 matrix)
            mInverseMassMatrixMotor = ((@Dimensionless float) (0.0f));
            if (mBody1.isMotionEnabled()) {
                mInverseMassMatrixMotor += mBody1.getMassInverse();
            }
            if (mBody2.isMotionEnabled()) {
                mInverseMassMatrixMotor += mBody2.getMassInverse();
            }
            mInverseMassMatrixMotor = (mInverseMassMatrixMotor > ((@Dimensionless float) (0.0f))) ? ((@Dimensionless float) (1.0f)) / mInverseMassMatrixMotor : ((@Dimensionless float) (0.0f));
        }

        // If warm-starting is not enabled
        if (!constraintSolverData.isWarmStartingActive) {

            // Reset all the accumulated impulses
            mImpulseTranslation.zero();
            mImpulseRotation.zero();
            mImpulseLowerLimit = ((@Dimensionless float) (0.0f));
            mImpulseUpperLimit = ((@Dimensionless float) (0.0f));
            mImpulseMotor = ((@Dimensionless float) (0.0f));
        }
    }

    // Warm start the constraint (apply the previous impulse at the beginning of the step)
    @Override
    public void warmstart(@Dimensionless SliderJoint this, @Dimensionless ConstraintSolverData constraintSolverData) {

        // Get the velocities
        @Dimensionless
        Vector3 v1 = constraintSolverData.linearVelocities[mIndexBody1];
        @Dimensionless
        Vector3 v2 = constraintSolverData.linearVelocities[mIndexBody2];
        @Dimensionless
        Vector3 w1 = constraintSolverData.angularVelocities[mIndexBody1];
        @Dimensionless
        Vector3 w2 = constraintSolverData.angularVelocities[mIndexBody2];

        // Get the inverse mass and inverse inertia tensors of the bodies
        @Dimensionless
        float inverseMassBody1 = mBody1.getMassInverse();
        @Dimensionless
        float inverseMassBody2 = mBody2.getMassInverse();

        // Compute the impulse P=J^T * lambda for the lower and upper limits constraints
        @Dimensionless
        float impulseLimits = mImpulseUpperLimit - mImpulseLowerLimit;
        @Dimensionless
        Vector3 linearImpulseLimits = new @Dimensionless Vector3(mSliderAxisWorld).multiply(impulseLimits);

        // Compute the impulse P=J^T * lambda for the motor constraint
        @Dimensionless
        Vector3 impulseMotor = new @Dimensionless Vector3(mSliderAxisWorld).multiply(mImpulseMotor);

        if (mBody1.isMotionEnabled()) {

            // Compute the impulse P=J^T * lambda for the 2 translation constraints
            @Dimensionless
            Vector3 linearImpulseBody1 = new @Dimensionless Vector3(
                    new @Dimensionless Vector3(new @Dimensionless Vector3(mN1).invert()).multiply(mImpulseTranslation.getX())).subtract(
                            new @Dimensionless Vector3(mN2).multiply(mImpulseTranslation.getY()));
            @Dimensionless
            Vector3 angularImpulseBody1 = new @Dimensionless Vector3(
                    new @Dimensionless Vector3(new @Dimensionless Vector3(mR1PlusUCrossN1).invert()).multiply(mImpulseTranslation.getX())).subtract(
                            new @Dimensionless Vector3(mR1PlusUCrossN2).multiply(mImpulseTranslation.getY()));

            // Compute the impulse P=J^T * lambda for the 3 rotation constraints
            angularImpulseBody1.add(new @Dimensionless Vector3(mImpulseRotation).invert());

            // Compute the impulse P=J^T * lambda for the lower and upper limits constraints
            linearImpulseBody1.add(linearImpulseLimits);
            angularImpulseBody1.add(new @Dimensionless Vector3(mR1PlusUCrossSliderAxis).multiply(impulseLimits));

            // Compute the impulse P=J^T * lambda for the motor constraint
            linearImpulseBody1.add(impulseMotor);

            // Apply the impulse to the body
            v1.add(new @Dimensionless Vector3(linearImpulseBody1).multiply(inverseMassBody1));
            w1.add(mI1.multiply(angularImpulseBody1, new @Dimensionless Vector3()));
        }
        if (mBody2.isMotionEnabled()) {

            // Compute the impulse P=J^T * lambda for the 2 translation constraints
            @Dimensionless
            Vector3 linearImpulseBody2 = new @Dimensionless Vector3(
                    new @Dimensionless Vector3(new @Dimensionless Vector3(mN1).invert()).multiply(mImpulseTranslation.getX())).add(
                            new @Dimensionless Vector3(mN2).multiply(mImpulseTranslation.getY()));
            @Dimensionless
            Vector3 angularImpulseBody2 = new @Dimensionless Vector3(
                    new @Dimensionless Vector3(new @Dimensionless Vector3(mR2CrossN1).invert()).multiply(mImpulseTranslation.getX())).add(
                            new @Dimensionless Vector3(mR2CrossN2).multiply(mImpulseTranslation.getY()));

            // Compute the impulse P=J^T * lambda for the 3 rotation constraints
            angularImpulseBody2.add(mImpulseRotation);

            // Compute the impulse P=J^T * lambda for the lower and upper limits constraints
            linearImpulseBody2.add(new @Dimensionless Vector3(linearImpulseLimits).invert());
            angularImpulseBody2.add(new @Dimensionless Vector3(mR2CrossSliderAxis).multiply(-impulseLimits));

            // Compute the impulse P=J^T * lambda for the motor constraint
            linearImpulseBody2.add(new @Dimensionless Vector3(impulseMotor).invert());

            // Apply the impulse to the body
            v2.add(new @Dimensionless Vector3(linearImpulseBody2).multiply(inverseMassBody2));
            w2.add(mI2.multiply(angularImpulseBody2, new @Dimensionless Vector3()));
        }
    }

    // Solve the velocity constraint
    @Override
    public void solveVelocityConstraint(@Dimensionless SliderJoint this, @Dimensionless ConstraintSolverData constraintSolverData) {

        // Get the velocities
        @Dimensionless
        Vector3 v1 = constraintSolverData.linearVelocities[mIndexBody1];
        @Dimensionless
        Vector3 v2 = constraintSolverData.linearVelocities[mIndexBody2];
        @Dimensionless
        Vector3 w1 = constraintSolverData.angularVelocities[mIndexBody1];
        @Dimensionless
        Vector3 w2 = constraintSolverData.angularVelocities[mIndexBody2];

        // Get the inverse mass and inverse inertia tensors of the bodies
        @Dimensionless
        float inverseMassBody1 = mBody1.getMassInverse();
        @Dimensionless
        float inverseMassBody2 = mBody2.getMassInverse();

        /**
         * --------------- Translation Constraints ---------------
         */
        // Compute J*v for the 2 translation constraints
        @Dimensionless
        float el1 = -mN1.dot(v1) - w1.dot(mR1PlusUCrossN1) + mN1.dot(v2) + w2.dot(mR2CrossN1);
        @Dimensionless
        float el2 = -mN2.dot(v1) - w1.dot(mR1PlusUCrossN2) + mN2.dot(v2) + w2.dot(mR2CrossN2);
        @Dimensionless
        Vector2 JvTranslation = new @Dimensionless Vector2(el1, el2);

        // Compute the Lagrange multiplier lambda for the 2 translation constraints
        @Dimensionless
        Vector2 deltaLambda = new @Dimensionless Vector2();
        mInverseMassMatrixTranslationConstraint.multiply(new @Dimensionless Vector2(JvTranslation).invert().subtract(mBTranslation), deltaLambda);
        mImpulseTranslation.add(deltaLambda);

        if (mBody1.isMotionEnabled()) {

            // Compute the impulse P=J^T * lambda for the 2 translation constraints
            @Dimensionless
            Vector3 linearImpulseBody1 = new @Dimensionless Vector3(
                    new @Dimensionless Vector3(new @Dimensionless Vector3(mN1).invert()).multiply(deltaLambda.getX())).subtract(
                            new @Dimensionless Vector3(mN2).multiply(deltaLambda.getY()));
            @Dimensionless
            Vector3 angularImpulseBody1 = new @Dimensionless Vector3(
                    new @Dimensionless Vector3(new @Dimensionless Vector3(mR1PlusUCrossN1).invert()).multiply(deltaLambda.getX())).subtract(
                            new @Dimensionless Vector3(mR1PlusUCrossN2).multiply(deltaLambda.getY()));

            // Apply the impulse to the body
            v1.add(new @Dimensionless Vector3(linearImpulseBody1).multiply(inverseMassBody1));
            w1.add(mI1.multiply(angularImpulseBody1, new @Dimensionless Vector3()));
        }
        if (mBody2.isMotionEnabled()) {

            // Compute the impulse P=J^T * lambda for the 2 translation constraints
            @Dimensionless
            Vector3 linearImpulseBody2 = new @Dimensionless Vector3(
                    new @Dimensionless Vector3(new @Dimensionless Vector3(mN1).invert()).multiply(deltaLambda.getX())).add(
                            new @Dimensionless Vector3(mN2).multiply(deltaLambda.getY()));
            @Dimensionless
            Vector3 angularImpulseBody2 = new @Dimensionless Vector3(
                    new @Dimensionless Vector3(new @Dimensionless Vector3(mR2CrossN1).invert()).multiply(deltaLambda.getX())).add(
                            new @Dimensionless Vector3(mR2CrossN2).multiply(deltaLambda.getY()));

            // Apply the impulse to the body
            v2.add(new @Dimensionless Vector3(linearImpulseBody2).multiply(inverseMassBody2));
            w2.add(mI2.multiply(angularImpulseBody2, new @Dimensionless Vector3()));
        }

        /**
         * --------------- Rotation Constraints ---------------
         */
        // Compute J*v for the 3 rotation constraints
        @Dimensionless
        Vector3 JvRotation = new @Dimensionless Vector3(w2).subtract(w1);

        // Compute the Lagrange multiplier lambda for the 3 rotation constraints
        @Dimensionless
        Vector3 deltaLambda2 = mInverseMassMatrixRotationConstraint.multiply(
                new @Dimensionless Vector3(new @Dimensionless Vector3(JvRotation).invert()).subtract(mBRotation), new @Dimensionless Vector3());
        mImpulseRotation.add(deltaLambda2);

        if (mBody1.isMotionEnabled()) {

            // Compute the impulse P=J^T * lambda for the 3 rotation constraints
            @Dimensionless
            Vector3 angularImpulseBody1 = new @Dimensionless Vector3(deltaLambda2).invert();

            // Apply the impulse to the body
            w1.add(mI1.multiply(angularImpulseBody1, new @Dimensionless Vector3()));
        }
        if (mBody2.isMotionEnabled()) {

            // Compute the impulse P=J^T * lambda for the 3 rotation constraints
            @Dimensionless
            Vector3 angularImpulseBody2 = deltaLambda2;

            // Apply the impulse to the body
            w2.add(mI2.multiply(angularImpulseBody2, new @Dimensionless Vector3()));
        }

        /**
         * --------------- Limits Constraints ---------------
         */
        if (mIsLimitEnabled) {

            // If the lower limit is violated
            if (mIsLowerLimitViolated) {

                // Compute J*v for the lower limit constraint
                @Dimensionless
                float JvLowerLimit = mSliderAxisWorld.dot(v2) + mR2CrossSliderAxis.dot(w2)
                        - mSliderAxisWorld.dot(v1) - mR1PlusUCrossSliderAxis.dot(w1);

                // Compute the Lagrange multiplier lambda for the lower limit constraint
                @Dimensionless
                float deltaLambdaLower = mInverseMassMatrixLimit * (-JvLowerLimit - mBLowerLimit);
                @Dimensionless
                float lambdaTemp = mImpulseLowerLimit;
                mImpulseLowerLimit = Math.max(mImpulseLowerLimit + deltaLambdaLower, ((@Dimensionless float) (0.0f)));
                deltaLambdaLower = mImpulseLowerLimit - lambdaTemp;

                if (mBody1.isMotionEnabled()) {

                    // Compute the impulse P=J^T * lambda for the lower limit constraint
                    @Dimensionless
                    Vector3 linearImpulseBody1 = new @Dimensionless Vector3(mSliderAxisWorld).multiply(-deltaLambdaLower);
                    @Dimensionless
                    Vector3 angularImpulseBody1 = new @Dimensionless Vector3(mR1PlusUCrossSliderAxis).multiply(-deltaLambdaLower);

                    // Apply the impulse to the body
                    v1.add(new @Dimensionless Vector3(linearImpulseBody1).multiply(inverseMassBody1));
                    w1.add(mI1.multiply(angularImpulseBody1, new @Dimensionless Vector3()));
                }
                if (mBody2.isMotionEnabled()) {

                    // Compute the impulse P=J^T * lambda for the lower limit constraint
                    @Dimensionless
                    Vector3 linearImpulseBody2 = new @Dimensionless Vector3(mSliderAxisWorld).multiply(deltaLambdaLower);
                    @Dimensionless
                    Vector3 angularImpulseBody2 = new @Dimensionless Vector3(mR2CrossSliderAxis).multiply(deltaLambdaLower);

                    // Apply the impulse to the body
                    v2.add(new @Dimensionless Vector3(linearImpulseBody2).multiply(inverseMassBody2));
                    w2.add(mI2.multiply(angularImpulseBody2, new @Dimensionless Vector3()));
                }
            }

            // If the upper limit is violated
            if (mIsUpperLimitViolated) {

                // Compute J*v for the upper limit constraint
                @Dimensionless
                float JvUpperLimit = mSliderAxisWorld.dot(v1) + mR1PlusUCrossSliderAxis.dot(w1)
                        - mSliderAxisWorld.dot(v2) - mR2CrossSliderAxis.dot(w2);

                // Compute the Lagrange multiplier lambda for the upper limit constraint
                @Dimensionless
                float deltaLambdaUpper = mInverseMassMatrixLimit * (-JvUpperLimit - mBUpperLimit);
                @Dimensionless
                float lambdaTemp = mImpulseUpperLimit;
                mImpulseUpperLimit = Math.max(mImpulseUpperLimit + deltaLambdaUpper, ((@Dimensionless float) (0.0f)));
                deltaLambdaUpper = mImpulseUpperLimit - lambdaTemp;

                if (mBody1.isMotionEnabled()) {

                    // Compute the impulse P=J^T * lambda for the upper limit constraint
                    @Dimensionless
                    Vector3 linearImpulseBody1 = new @Dimensionless Vector3(mSliderAxisWorld).multiply(deltaLambdaUpper);
                    @Dimensionless
                    Vector3 angularImpulseBody1 = new @Dimensionless Vector3(mR1PlusUCrossSliderAxis).multiply(deltaLambdaUpper);

                    // Apply the impulse to the body
                    v1.add(new @Dimensionless Vector3(linearImpulseBody1).multiply(inverseMassBody1));
                    w1.add(mI1.multiply(angularImpulseBody1, new @Dimensionless Vector3()));
                }
                if (mBody2.isMotionEnabled()) {

                    // Compute the impulse P=J^T * lambda for the upper limit constraint
                    @Dimensionless
                    Vector3 linearImpulseBody2 = new @Dimensionless Vector3(mSliderAxisWorld).multiply(-deltaLambdaUpper);
                    @Dimensionless
                    Vector3 angularImpulseBody2 = new @Dimensionless Vector3(mR2CrossSliderAxis).multiply(-deltaLambdaUpper);

                    // Apply the impulse to the body
                    v2.add(new @Dimensionless Vector3(linearImpulseBody2).multiply(inverseMassBody2));
                    w2.add(mI2.multiply(angularImpulseBody2, new @Dimensionless Vector3()));
                }
            }
        }

        /**
         * --------------- Motor ---------------
         */
        if (mIsMotorEnabled) {

            // Compute J*v for the motor
            @Dimensionless
            float JvMotor = mSliderAxisWorld.dot(v1) - mSliderAxisWorld.dot(v2);

            // Compute the Lagrange multiplier lambda for the motor
            @Dimensionless
            float maxMotorImpulse = mMaxMotorForce * constraintSolverData.timeStep;
            @Dimensionless
            float deltaLambdaMotor = mInverseMassMatrixMotor * (-JvMotor - mMotorSpeed);
            @Dimensionless
            float lambdaTemp = mImpulseMotor;
            mImpulseMotor = Mathematics.Clamp(mImpulseMotor + deltaLambdaMotor, -maxMotorImpulse, maxMotorImpulse);
            deltaLambdaMotor = mImpulseMotor - lambdaTemp;

            if (mBody1.isMotionEnabled()) {

                // Compute the impulse P=J^T * lambda for the motor
                @Dimensionless
                Vector3 linearImpulseBody1 = new @Dimensionless Vector3(mSliderAxisWorld).multiply(deltaLambdaMotor);

                // Apply the impulse to the body
                v1.add(new @Dimensionless Vector3(linearImpulseBody1).multiply(inverseMassBody1));
            }
            if (mBody2.isMotionEnabled()) {

                // Compute the impulse P=J^T * lambda for the motor
                @Dimensionless
                Vector3 linearImpulseBody2 = new @Dimensionless Vector3(mSliderAxisWorld).multiply(-deltaLambdaMotor);

                // Apply the impulse to the body
                v2.add(new @Dimensionless Vector3(linearImpulseBody2).multiply(inverseMassBody2));
            }
        }
    }

    // Solve the position constraint (for position error correction)
    @Override
    public void solvePositionConstraint(@Dimensionless SliderJoint this, @Dimensionless ConstraintSolverData constraintSolverData) {

        // If the error position correction technique is not the non-linear-gauss-seidel, we do
        // do not execute this method
        if (mPositionCorrectionTechnique != JointsPositionCorrectionTechnique.NON_LINEAR_GAUSS_SEIDEL) {
            return;
        }

        // Get the bodies positions and orientations
        @Dimensionless
        Vector3 x1 = constraintSolverData.positions.get(mIndexBody1);
        @Dimensionless
        Vector3 x2 = constraintSolverData.positions.get(mIndexBody2);
        @Dimensionless
        Quaternion q1 = constraintSolverData.orientations.get(mIndexBody1);
        @Dimensionless
        Quaternion q2 = constraintSolverData.orientations.get(mIndexBody2);

        // Get the inverse mass and inverse inertia tensors of the bodies
        @Dimensionless
        float inverseMassBody1 = mBody1.getMassInverse();
        @Dimensionless
        float inverseMassBody2 = mBody2.getMassInverse();

        // Recompute the inertia tensor of bodies
        mI1 = mBody1.getInertiaTensorInverseWorld();
        mI2 = mBody2.getInertiaTensorInverseWorld();

        // Vector from body center to the anchor point
        q1.multiply(mLocalAnchorPointBody1, mR1);
        q2.multiply(mLocalAnchorPointBody2, mR2);

        // Compute the vector u (difference between anchor points)
        @Dimensionless
        Vector3 u = new @Dimensionless Vector3(new @Dimensionless Vector3(new @Dimensionless Vector3(x2).add(mR2)).subtract(x1)).subtract(mR1);

        // Compute the two orthogonal vectors to the slider axis in world-space
        q1.multiply(mSliderAxisBody1, mSliderAxisWorld);
        mSliderAxisWorld.normalize();
        mN1 = new @Dimensionless Vector3(mSliderAxisWorld).setUnitOrthogonal();
        mN2 = new @Dimensionless Vector3(mSliderAxisWorld).cross(mN1);

        // Check if the limit constraints are violated or not
        @Dimensionless
        float uDotSliderAxis = u.dot(mSliderAxisWorld);
        @Dimensionless
        float lowerLimitError = uDotSliderAxis - mLowerLimit;
        @Dimensionless
        float upperLimitError = mUpperLimit - uDotSliderAxis;
        mIsLowerLimitViolated = lowerLimitError <= ((@Dimensionless float) (0.0f));
        mIsUpperLimitViolated = upperLimitError <= ((@Dimensionless float) (0.0f));

        // Compute the cross products used in the Jacobians
        mR2CrossN1 = new @Dimensionless Vector3(mR2).cross(mN1);
        mR2CrossN2 = new @Dimensionless Vector3(mR2).cross(mN2);
        mR2CrossSliderAxis = new @Dimensionless Vector3(mR2).cross(mSliderAxisWorld);
        @Dimensionless
        Vector3 r1PlusU = new @Dimensionless Vector3(mR1).add(u);
        mR1PlusUCrossN1 = new @Dimensionless Vector3(r1PlusU).cross(mN1);
        mR1PlusUCrossN2 = new @Dimensionless Vector3(r1PlusU).cross(mN2);
        mR1PlusUCrossSliderAxis = new @Dimensionless Vector3(r1PlusU).cross(mSliderAxisWorld);

        /**
         * --------------- Translation Constraints ---------------
         */
        // Recompute the inverse of the mass matrix K=JM^-1J^t for the 2 translation
        // constraints (2x2 matrix)
        @Dimensionless
        float sumInverseMass = ((@Dimensionless float) (0.0f));
        @Dimensionless
        Vector3 I1R1PlusUCrossN1 = new @Dimensionless Vector3();
        @Dimensionless
        Vector3 I1R1PlusUCrossN2 = new @Dimensionless Vector3();
        @Dimensionless
        Vector3 I2R2CrossN1 = new @Dimensionless Vector3();
        @Dimensionless
        Vector3 I2R2CrossN2 = new @Dimensionless Vector3();
        if (mBody1.isMotionEnabled()) {
            sumInverseMass += mBody1.getMassInverse();
            I1R1PlusUCrossN1 = mI1.multiply(mR1PlusUCrossN1, new @Dimensionless Vector3());
            I1R1PlusUCrossN2 = mI1.multiply(mR1PlusUCrossN2, new @Dimensionless Vector3());
        }
        if (mBody2.isMotionEnabled()) {
            sumInverseMass += mBody2.getMassInverse();
            I2R2CrossN1 = mI2.multiply(mR2CrossN1, new @Dimensionless Vector3());
            I2R2CrossN2 = mI2.multiply(mR2CrossN2, new @Dimensionless Vector3());
        }
        @Dimensionless
        float el11 = sumInverseMass + mR1PlusUCrossN1.dot(I1R1PlusUCrossN1) + mR2CrossN1.dot(I2R2CrossN1);
        @Dimensionless
        float el12 = mR1PlusUCrossN1.dot(I1R1PlusUCrossN2) + mR2CrossN1.dot(I2R2CrossN2);
        @Dimensionless
        float el21 = mR1PlusUCrossN2.dot(I1R1PlusUCrossN1) + mR2CrossN2.dot(I2R2CrossN1);
        @Dimensionless
        float el22 = sumInverseMass + mR1PlusUCrossN2.dot(I1R1PlusUCrossN2) + mR2CrossN2.dot(I2R2CrossN2);
        @Dimensionless
        Matrix2x2 matrixKTranslation = new @Dimensionless Matrix2x2(el11, el12, el21, el22);
        mInverseMassMatrixTranslationConstraint.zero();
        if (mBody1.isMotionEnabled() || mBody2.isMotionEnabled()) {
            mInverseMassMatrixTranslationConstraint = new @Dimensionless Matrix2x2(matrixKTranslation).inverse();
        }

        // Compute the position error for the 2 translation constraints
        @Dimensionless
        Vector2 translationError = new @Dimensionless Vector2(u.dot(mN1), u.dot(mN2));

        // Compute the Lagrange multiplier lambda for the 2 translation constraints
        @Dimensionless
        Vector2 lambdaTranslation = new @Dimensionless Vector2();
        mInverseMassMatrixTranslationConstraint.multiply(new @Dimensionless Vector2(translationError).invert(), lambdaTranslation);

        if (mBody1.isMotionEnabled()) {

            // Compute the impulse P=J^T * lambda for the 2 translation constraints
            @Dimensionless
            Vector3 linearImpulseBody1 = new @Dimensionless Vector3(
                    new @Dimensionless Vector3(new @Dimensionless Vector3(mN1).invert()).multiply(lambdaTranslation.getX())).subtract(
                            new @Dimensionless Vector3(mN2).multiply(lambdaTranslation.getY()));
            @Dimensionless
            Vector3 angularImpulseBody1 = new @Dimensionless Vector3(
                    new @Dimensionless Vector3(new @Dimensionless Vector3(mR1PlusUCrossN1).invert()).multiply(lambdaTranslation.getX())).subtract(
                            new @Dimensionless Vector3(mR1PlusUCrossN2).multiply(lambdaTranslation.getY()));

            // Apply the impulse to the body
            @Dimensionless
            Vector3 v1 = new @Dimensionless Vector3(linearImpulseBody1).multiply(inverseMassBody1);
            @Dimensionless
            Vector3 w1 = mI1.multiply(angularImpulseBody1, new @Dimensionless Vector3());

            // Update the body position/orientation
            x1.add(v1);
            q1.add(new @Dimensionless Quaternion(w1, ((@Dimensionless float) (0.0f))).multiply(q1).multiply(((@Dimensionless float) (0.5f))));
            q1.normalize();
        }
        if (mBody2.isMotionEnabled()) {

            // Compute the impulse P=J^T * lambda for the 2 translation constraints
            @Dimensionless
            Vector3 linearImpulseBody2 = new @Dimensionless Vector3(
                    new @Dimensionless Vector3(new @Dimensionless Vector3(mN1).invert()).multiply(lambdaTranslation.getX())).add(
                            new @Dimensionless Vector3(mN2).multiply(lambdaTranslation.getY()));
            @Dimensionless
            Vector3 angularImpulseBody2 = new @Dimensionless Vector3(
                    new @Dimensionless Vector3(new @Dimensionless Vector3(mR2CrossN1).invert()).multiply(lambdaTranslation.getX())).add(
                            new @Dimensionless Vector3(mR2CrossN2).multiply(lambdaTranslation.getY()));

            // Apply the impulse to the body
            @Dimensionless
            Vector3 v2 = new @Dimensionless Vector3(linearImpulseBody2).multiply(inverseMassBody2);
            @Dimensionless
            Vector3 w2 = mI2.multiply(angularImpulseBody2, new @Dimensionless Vector3());

            // Update the body position/orientation
            x2.add(v2);
            q2.add(new @Dimensionless Quaternion(w2, ((@Dimensionless float) (0.0f))).multiply(q2).multiply(((@Dimensionless float) (0.5f))));
            q2.normalize();
        }

        /**
         * --------------- Rotation Constraints ---------------
         */
        // Compute the inverse of the mass matrix K=JM^-1J^t for the 3 rotation
        // contraints (3x3 matrix)
        mInverseMassMatrixRotationConstraint.zero();
        if (mBody1.isMotionEnabled()) {
            mInverseMassMatrixRotationConstraint.add(mI1);
        }
        if (mBody2.isMotionEnabled()) {
            mInverseMassMatrixRotationConstraint.add(mI2);
        }
        if (mBody1.isMotionEnabled() || mBody2.isMotionEnabled()) {
            mInverseMassMatrixRotationConstraint = new @Dimensionless Matrix3x3(mInverseMassMatrixRotationConstraint).inverse();
        }

        // Compute the position error for the 3 rotation constraints
        @Dimensionless
        Quaternion currentOrientationDifference = new @Dimensionless Quaternion(q2).multiply(new @Dimensionless Quaternion(q1).inverse());
        currentOrientationDifference.normalize();
        @Dimensionless
        Quaternion qError = new @Dimensionless Quaternion(currentOrientationDifference).multiply(mInitOrientationDifferenceInv);
        @Dimensionless
        Vector3 qErrorV = new @Dimensionless Vector3();
        qError.getVectorV(qErrorV);
        @Dimensionless
        Vector3 errorRotation = qErrorV.multiply(((@Dimensionless float) (2.0f)));

        // Compute the Lagrange multiplier lambda for the 3 rotation constraints
        @Dimensionless
        Vector3 lambdaRotation = mInverseMassMatrixRotationConstraint.multiply(
                new @Dimensionless Vector3(errorRotation).invert(), new @Dimensionless Vector3());

        if (mBody1.isMotionEnabled()) {

            // Compute the impulse P=J^T * lambda for the 3 rotation constraints
            @Dimensionless
            Vector3 angularImpulseBody1 = new @Dimensionless Vector3(lambdaRotation).invert();

            // Apply the impulse to the body
            @Dimensionless
            Vector3 w1 = mI1.multiply(angularImpulseBody1, new @Dimensionless Vector3());

            // Update the body position/orientation
            q1.add(new @Dimensionless Quaternion(w1, ((@Dimensionless float) (0.0f))).multiply(q1).multiply(((@Dimensionless float) (0.5f))));
            q1.normalize();
        }
        if (mBody2.isMotionEnabled()) {

            // Compute the impulse P=J^T * lambda for the 3 rotation constraints
            @Dimensionless
            Vector3 angularImpulseBody2 = lambdaRotation;

            // Apply the impulse to the body
            @Dimensionless
            Vector3 w2 = mI2.multiply(angularImpulseBody2, new @Dimensionless Vector3());

            // Update the body position/orientation
            q2.add(new @Dimensionless Quaternion(w2, ((@Dimensionless float) (0.0f))).multiply(q2).multiply(((@Dimensionless float) (0.5f))));
            q2.normalize();
        }

        /**
         * --------------- Limits Constraints ---------------
         */
        if (mIsLimitEnabled) {

            if (mIsLowerLimitViolated || mIsUpperLimitViolated) {

                // Compute the inverse of the mass matrix K=JM^-1J^t for the limits (1x1 matrix)
                mInverseMassMatrixLimit = ((@Dimensionless float) (0.0f));
                if (mBody1.isMotionEnabled()) {
                    mInverseMassMatrixLimit += mBody1.getMassInverse() + mR1PlusUCrossSliderAxis.dot(mI1.multiply(mR1PlusUCrossSliderAxis, new @Dimensionless Vector3()));
                }
                if (mBody2.isMotionEnabled()) {
                    mInverseMassMatrixLimit += mBody2.getMassInverse() + mR2CrossSliderAxis.dot(mI2.multiply(mR2CrossSliderAxis, new @Dimensionless Vector3()));
                }
                mInverseMassMatrixLimit = (mInverseMassMatrixLimit > ((@Dimensionless float) (0.0f))) ? ((@Dimensionless float) (1.0f)) / mInverseMassMatrixLimit : ((@Dimensionless float) (0.0f));
            }

            // If the lower limit is violated
            if (mIsLowerLimitViolated) {

                // Compute the Lagrange multiplier lambda for the lower limit constraint
                @Dimensionless
                float lambdaLowerLimit = mInverseMassMatrixLimit * (-lowerLimitError);

                if (mBody1.isMotionEnabled()) {

                    // Compute the impulse P=J^T * lambda for the lower limit constraint
                    @Dimensionless
                    Vector3 linearImpulseBody1 = new @Dimensionless Vector3(mSliderAxisWorld).multiply(-lambdaLowerLimit);
                    @Dimensionless
                    Vector3 angularImpulseBody1 = new @Dimensionless Vector3(mR1PlusUCrossSliderAxis).multiply(-lambdaLowerLimit);

                    // Apply the impulse to the body
                    @Dimensionless
                    Vector3 v1 = new @Dimensionless Vector3(linearImpulseBody1).multiply(inverseMassBody1);
                    @Dimensionless
                    Vector3 w1 = mI1.multiply(angularImpulseBody1, new @Dimensionless Vector3());

                    // Update the body position/orientation
                    x1.add(v1);
                    q1.add(new @Dimensionless Quaternion(w1, ((@Dimensionless float) (0.0f))).multiply(q1).multiply(((@Dimensionless float) (0.5f))));
                    q1.normalize();
                }
                if (mBody2.isMotionEnabled()) {

                    // Compute the impulse P=J^T * lambda for the lower limit constraint
                    @Dimensionless
                    Vector3 linearImpulseBody2 = new @Dimensionless Vector3(mSliderAxisWorld).multiply(lambdaLowerLimit);
                    @Dimensionless
                    Vector3 angularImpulseBody2 = new @Dimensionless Vector3(mR2CrossSliderAxis).multiply(lambdaLowerLimit);

                    // Apply the impulse to the body
                    @Dimensionless
                    Vector3 v2 = new @Dimensionless Vector3(linearImpulseBody2).multiply(inverseMassBody2);
                    @Dimensionless
                    Vector3 w2 = mI2.multiply(angularImpulseBody2, new @Dimensionless Vector3());

                    // Update the body position/orientation
                    x2.add(v2);
                    q2.add(new @Dimensionless Quaternion(w2, ((@Dimensionless float) (0.0f))).multiply(q2).multiply(((@Dimensionless float) (0.5f))));
                    q2.normalize();
                }
            }

            // If the upper limit is violated
            if (mIsUpperLimitViolated) {

                // Compute the Lagrange multiplier lambda for the upper limit constraint
                @Dimensionless
                float lambdaUpperLimit = mInverseMassMatrixLimit * (-upperLimitError);

                if (mBody1.isMotionEnabled()) {

                    // Compute the impulse P=J^T * lambda for the upper limit constraint
                    @Dimensionless
                    Vector3 linearImpulseBody1 = new @Dimensionless Vector3(mSliderAxisWorld).multiply(lambdaUpperLimit);
                    @Dimensionless
                    Vector3 angularImpulseBody1 = new @Dimensionless Vector3(mR1PlusUCrossSliderAxis).multiply(lambdaUpperLimit);

                    // Apply the impulse to the body
                    @Dimensionless
                    Vector3 v1 = new @Dimensionless Vector3(linearImpulseBody1).multiply(inverseMassBody1);
                    @Dimensionless
                    Vector3 w1 = mI1.multiply(angularImpulseBody1, new @Dimensionless Vector3());

                    // Update the body position/orientation
                    x1.add(v1);
                    q1.add(new @Dimensionless Quaternion(w1, ((@Dimensionless float) (0.0f))).multiply(q1).multiply(((@Dimensionless float) (0.5f))));
                    q1.normalize();
                }
                if (mBody2.isMotionEnabled()) {

                    // Compute the impulse P=J^T * lambda for the upper limit constraint
                    @Dimensionless
                    Vector3 linearImpulseBody2 = new @Dimensionless Vector3(mSliderAxisWorld).multiply(-lambdaUpperLimit);
                    @Dimensionless
                    Vector3 angularImpulseBody2 = new @Dimensionless Vector3(mR2CrossSliderAxis).multiply(-lambdaUpperLimit);

                    // Apply the impulse to the body
                    @Dimensionless
                    Vector3 v2 = new @Dimensionless Vector3(linearImpulseBody2).multiply(inverseMassBody2);
                    @Dimensionless
                    Vector3 w2 = mI2.multiply(angularImpulseBody2, new @Dimensionless Vector3());

                    // Update the body position/orientation
                    x2.add(v2);
                    q2.add(new @Dimensionless Quaternion(w2, ((@Dimensionless float) (0.0f))).multiply(q2).multiply(((@Dimensionless float) (0.5f))));
                    q2.normalize();
                }
            }
        }
    }

    // Return true if the limits or the joint are enabled
    public @Dimensionless boolean isLimitEnabled(@Dimensionless SliderJoint this) {
        return mIsLimitEnabled;
    }

    // Return true if the motor of the joint is enabled
    public @Dimensionless boolean isMotorEnabled(@Dimensionless SliderJoint this) {
        return mIsMotorEnabled;
    }

    // Return the minimum translation limit
    public @Dimensionless float getMinTranslationLimit(@Dimensionless SliderJoint this) {
        return mLowerLimit;
    }

    // Return the maximum translation limit
    public @Dimensionless float getMaxTranslationLimit(@Dimensionless SliderJoint this) {
        return mUpperLimit;
    }

    // Return the motor speed
    public @Dimensionless float getMotorSpeed(@Dimensionless SliderJoint this) {
        return mMotorSpeed;
    }

    // Return the maximum motor force
    public @Dimensionless float getMaxMotorForce(@Dimensionless SliderJoint this) {
        return mMaxMotorForce;
    }

    // Return the intensity of the current force applied for the joint motor
    public @Dimensionless float getMotorForce(@Dimensionless SliderJoint this, @Dimensionless float timeStep) {
        return mImpulseMotor / timeStep;
    }

    // Enable/Disable the limits of the joint
    public void enableLimit(@Dimensionless SliderJoint this, @Dimensionless boolean isLimitEnabled) {

        if (isLimitEnabled != mIsLimitEnabled) {

            mIsLimitEnabled = isLimitEnabled;

            // Reset the limits
            resetLimits();
        }
    }

    // Enable/Disable the motor of the joint
    public void enableMotor(@Dimensionless SliderJoint this, @Dimensionless boolean isMotorEnabled) {

        mIsMotorEnabled = isMotorEnabled;
        mImpulseMotor = ((@Dimensionless float) (0.0f));

        // Wake up the two bodies of the joint
        mBody1.setIsSleeping(false);
        mBody2.setIsSleeping(false);
    }

    // Return the current translation value of the joint
    public @Dimensionless float getTranslation(@Dimensionless SliderJoint this) {

        // Get the bodies positions and orientations
        @Dimensionless
        Vector3 x1 = mBody1.getTransform().getPosition();
        @Dimensionless
        Vector3 x2 = mBody2.getTransform().getPosition();
        @Dimensionless
        Quaternion q1 = mBody1.getTransform().getOrientation();
        @Dimensionless
        Quaternion q2 = mBody2.getTransform().getOrientation();

        // Compute the two anchor points in world-space coordinates
        @Dimensionless
        Vector3 vB1 = new @Dimensionless Vector3();
        q1.multiply(mLocalAnchorPointBody1, vB1);
        @Dimensionless
        Vector3 anchorBody1 = new @Dimensionless Vector3(x1).add(vB1);
        @Dimensionless
        Vector3 vB2 = new @Dimensionless Vector3();
        q2.multiply(mLocalAnchorPointBody2, vB2);
        @Dimensionless
        Vector3 anchorBody2 = new @Dimensionless Vector3(x2).add(vB2);

        // Compute the vector u (difference between anchor points)
        @Dimensionless
        Vector3 u = new @Dimensionless Vector3(anchorBody2).subtract(anchorBody1);

        // Compute the slider axis in world-space
        @Dimensionless
        Vector3 sliderAxisWorld = new @Dimensionless Vector3();
        q1.multiply(mSliderAxisBody1, sliderAxisWorld);
        sliderAxisWorld.normalize();

        // Compute and return the translation value
        return u.dot(sliderAxisWorld);
    }

    // Set the minimum translation limit
    public void setMinTranslationLimit(@Dimensionless SliderJoint this, @Dimensionless float lowerLimit) {

        assert (lowerLimit <= mUpperLimit);

        if (lowerLimit != mLowerLimit) {

            mLowerLimit = lowerLimit;

            // Reset the limits
            resetLimits();
        }
    }

    // Set the maximum translation limit
    public void setMaxTranslationLimit(@Dimensionless SliderJoint this, @Dimensionless float upperLimit) {

        assert (mLowerLimit <= upperLimit);

        if (upperLimit != mUpperLimit) {

            mUpperLimit = upperLimit;

            // Reset the limits
            resetLimits();
        }
    }

    // Set the motor speed
    public void setMotorSpeed(@Dimensionless SliderJoint this, @Dimensionless float motorSpeed) {

        if (motorSpeed != mMotorSpeed) {

            mMotorSpeed = motorSpeed;

            // Wake up the two bodies of the joint
            mBody1.setIsSleeping(false);
            mBody2.setIsSleeping(false);
        }
    }

    // Set the maximum motor force
    public void setMaxMotorForce(@Dimensionless SliderJoint this, @Dimensionless float maxMotorForce) {

        if (maxMotorForce != mMaxMotorForce) {

            assert (mMaxMotorForce >= ((@Dimensionless float) (0.0f)));
            mMaxMotorForce = maxMotorForce;

            // Wake up the two bodies of the joint
            mBody1.setIsSleeping(false);
            mBody2.setIsSleeping(false);
        }
    }

}
