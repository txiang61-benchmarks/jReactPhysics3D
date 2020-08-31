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
import net.smert.jreactphysics3d.configuration.Defaults;
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
 * This class represents a hinge joint that allows arbitrary rotation between two bodies around a single axis. This
 * joint has one degree of freedom. It can be useful to simulate doors or pendulumns.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
@Dimensionless
public class HingeJoint extends Joint {

    // Beta value for the bias factor of position correction
    private static final @Dimensionless float BETA = ((@Dimensionless float) (0.2f));

    // Anchor point of body 1 (in local-space coordinates of body 1)
    private @Dimensionless Vector3 mLocalAnchorPointBody1;

    // Anchor point of body 2 (in local-space coordinates of body 2)
    private @Dimensionless Vector3 mLocalAnchorPointBody2;

    // Hinge rotation axis (in local-space coordinates of body 1)
    private @Dimensionless Vector3 mHingeLocalAxisBody1;

    // Hinge rotation axis (in local-space coordiantes of body 2)
    private @Dimensionless Vector3 mHingeLocalAxisBody2;

    // Inertia tensor of body 1 (in world-space coordinates)
    private @Dimensionless Matrix3x3 mI1;

    // Inertia tensor of body 2 (in world-space coordinates)
    private @Dimensionless Matrix3x3 mI2;

    // Hinge rotation axis (in world-space coordinates) computed from body 1
    private @Dimensionless Vector3 mA1;

    // Vector from center of body 2 to anchor point in world-space
    private @Dimensionless Vector3 mR1World;

    // Vector from center of body 2 to anchor point in world-space
    private @Dimensionless Vector3 mR2World;

    // Cross product of vector b2 and a1
    private @Dimensionless Vector3 mB2CrossA1;

    // Cross product of vector c2 and a1;
    private @Dimensionless Vector3 mC2CrossA1;

    // Impulse for the 3 translation constraints
    private @Dimensionless Vector3 mImpulseTranslation;

    // Impulse for the 2 rotation constraints
    private @Dimensionless Vector2 mImpulseRotation;

    // Accumulated impulse for the lower limit constraint
    private @Dimensionless float mImpulseLowerLimit;

    // Accumulated impulse for the upper limit constraint
    private @Dimensionless float mImpulseUpperLimit;

    // Accumulated impulse for the motor constraint;
    private @Dimensionless float mImpulseMotor;

    // Inverse mass matrix K=JM^-1J^t for the 3 translation constraints
    private @Dimensionless Matrix3x3 mInverseMassMatrixTranslation;

    // Inverse mass matrix K=JM^-1J^t for the 2 rotation constraints
    private @Dimensionless Matrix2x2 mInverseMassMatrixRotation;

    // Inverse of mass matrix K=JM^-1J^t for the limits and motor constraints (1x1 matrix)
    private @Dimensionless float mInverseMassMatrixLimitMotor;

    // Inverse of mass matrix K=JM^-1J^t for the motor
    private @Dimensionless float mInverseMassMatrixMotor;

    // Bias vector for the error correction for the translation constraints
    private @Dimensionless Vector3 mBTranslation;

    // Bias vector for the error correction for the rotation constraints
    private @Dimensionless Vector2 mBRotation;

    // Bias of the lower limit constraint
    private @Dimensionless float mBLowerLimit;

    // Bias of the upper limit constraint
    private @Dimensionless float mBUpperLimit;

    // Inverse of the initial orientation difference between the bodies
    private @Dimensionless Quaternion mInitOrientationDifferenceInv;

    // True if the joint limits are enabled
    private @Dimensionless boolean mIsLimitEnabled;

    // True if the motor of the joint in enabled
    private @Dimensionless boolean mIsMotorEnabled;

    // Lower limit (minimum allowed rotation angle in radi)
    private @Dimensionless float mLowerLimit;

    // Upper limit (maximum translation distance)
    private @Dimensionless float mUpperLimit;

    // True if the lower limit is violated
    private @Dimensionless boolean mIsLowerLimitViolated;

    // True if the upper limit is violated
    private @Dimensionless boolean mIsUpperLimitViolated;

    // Motor speed
    private @Dimensionless float mMotorSpeed;

    // Maximum motor torque (in Newtons) that can be applied to reach to desired motor speed
    private @Dimensionless float mMaxMotorTorque;

    // Reset the limits
    private void resetLimits(@Dimensionless HingeJoint this) {

        // Reset the accumulated impulses for the limits
        mImpulseLowerLimit = ((@Dimensionless float) (0.0f));
        mImpulseUpperLimit = ((@Dimensionless float) (0.0f));

        // Wake up the two bodies of the joint
        mBody1.setIsSleeping(false);
        mBody2.setIsSleeping(false);
    }

    // Given an angle in radian, this method returns the corresponding angle in the range [-pi; pi]
    protected @Dimensionless float computeNormalizedAngle(@Dimensionless HingeJoint this, @Dimensionless float angle) {

        // Convert it into the range [-2*pi; 2*pi]
        angle = angle % Defaults.PI_TIMES_2;

        // Convert it into the range [-pi; pi]
        if (angle < -Defaults.PI) {
            return angle + Defaults.PI_TIMES_2;
        } else if (angle > Defaults.PI) {
            return angle - Defaults.PI_TIMES_2;
        } else {
            return angle;
        }
    }

    // Given an "inputAngle" in the range [-pi, pi], this method returns an
    // angle (modulo 2*pi) in the range [-2*pi; 2*pi] that is closest to one of the
    // two angle limits in arguments.
    protected @Dimensionless float computeCorrespondingAngleNearLimits(@Dimensionless HingeJoint this, @Dimensionless float inputAngle, @Dimensionless float lowerLimitAngle, @Dimensionless float upperLimitAngle) {
        if (upperLimitAngle <= lowerLimitAngle) {
            return inputAngle;
        } else if (inputAngle > upperLimitAngle) {
            @Dimensionless
            float diffToUpperLimit = Math.abs(computeNormalizedAngle(inputAngle - upperLimitAngle));
            @Dimensionless
            float diffToLowerLimit = Math.abs(computeNormalizedAngle(inputAngle - lowerLimitAngle));
            return (diffToUpperLimit > diffToLowerLimit) ? (inputAngle - Defaults.PI_TIMES_2) : inputAngle;
        } else if (inputAngle < lowerLimitAngle) {
            @Dimensionless
            float diffToUpperLimit = Math.abs(computeNormalizedAngle(upperLimitAngle - inputAngle));
            @Dimensionless
            float diffToLowerLimit = Math.abs(computeNormalizedAngle(lowerLimitAngle - inputAngle));
            return (diffToUpperLimit > diffToLowerLimit) ? inputAngle : (inputAngle + Defaults.PI_TIMES_2);
        } else {
            return inputAngle;
        }
    }

    // Compute the current angle around the hinge axis
    protected @Dimensionless float computeCurrentHingeAngle(@Dimensionless HingeJoint this, @Dimensionless Quaternion orientationBody1, @Dimensionless Quaternion orientationBody2) {

        @Dimensionless
        float hingeAngle;

        // Compute the current orientation difference between the two bodies
        @Dimensionless
        Quaternion currentOrientationDiff = new @Dimensionless Quaternion(orientationBody2).multiply(new @Dimensionless Quaternion(orientationBody1).inverse());
        currentOrientationDiff.normalize();

        // Compute the relative rotation considering the initial orientation difference
        @Dimensionless
        Quaternion relativeRotation = new @Dimensionless Quaternion(currentOrientationDiff).multiply(mInitOrientationDifferenceInv);
        relativeRotation.normalize();

        // A quaternion q = [cos(theta/2); sin(theta/2) * rotAxis] where rotAxis is a unit
        // length vector. We can extract cos(theta/2) with q.w and we can extract |sin(theta/2)| with :
        // |sin(theta/2)| = q.getVectorV().length() since rotAxis is unit length. Note that any
        // rotation can be represented by a quaternion q and -q. Therefore, if the relative rotation
        // axis is not pointing in the same direction as the hinge axis, we use the rotation -q which
        // has the same |sin(theta/2)| value but the value cos(theta/2) is sign inverted. Some details
        // about this trick is explained in the source code of OpenTissue (http://www.opentissue.org).
        @Dimensionless
        float cosHalfAngle = relativeRotation.getW();
        @Dimensionless
        Vector3 relativeRotationV = new @Dimensionless Vector3();
        relativeRotation.getVectorV(relativeRotationV);
        @Dimensionless
        float sinHalfAngleAbs = relativeRotationV.length();

        // Compute the dot product of the relative rotation axis and the hinge axis
        @Dimensionless
        float dotProduct = relativeRotationV.dot(mA1);

        // If the relative rotation axis and the hinge axis are pointing the same direction
        if (dotProduct >= ((@Dimensionless float) (0.0f))) {
            hingeAngle = ((@Dimensionless float) (2.0f)) * Mathematics.ArcTan2(sinHalfAngleAbs, cosHalfAngle);
        } else {
            hingeAngle = ((@Dimensionless float) (2.0f)) * Mathematics.ArcTan2(sinHalfAngleAbs, -cosHalfAngle);
        }

        // Convert the angle from range [-2*pi; 2*pi] into the range [-pi; pi]
        hingeAngle = computeNormalizedAngle(hingeAngle);

        // Compute and return the corresponding angle near one the two limits
        return computeCorrespondingAngleNearLimits(hingeAngle, mLowerLimit, mUpperLimit);
    }

    // Constructor
    public HingeJoint(@Dimensionless HingeJointInfo jointInfo) {
        super(jointInfo);

        mImpulseTranslation = new @Dimensionless Vector3();
        mImpulseRotation = new @Dimensionless Vector2();
        mImpulseLowerLimit = ((@Dimensionless float) (0.0f));
        mImpulseUpperLimit = ((@Dimensionless float) (0.0f));
        mImpulseMotor = ((@Dimensionless float) (0.0f));
        mIsLimitEnabled = jointInfo.isLimitEnabled;
        mIsMotorEnabled = jointInfo.isMotorEnabled;
        mLowerLimit = jointInfo.minAngleLimit;
        mUpperLimit = jointInfo.maxAngleLimit;
        mIsLowerLimitViolated = false;
        mIsUpperLimitViolated = false;
        mMotorSpeed = jointInfo.motorSpeed;
        mMaxMotorTorque = jointInfo.maxMotorTorque;

        assert (mLowerLimit <= ((@Dimensionless float) (0.0f)) && mLowerLimit >= - ((@Dimensionless float) (2.0f)) * Defaults.PI);
        assert (mUpperLimit >= ((@Dimensionless float) (0.0f)) && mUpperLimit <= ((@Dimensionless float) (2.0f)) * Defaults.PI);

        // Compute the local-space anchor point for each body
        @Dimensionless
        Transform transform1 = mBody1.getTransform();
        @Dimensionless
        Transform transform2 = mBody2.getTransform();
        mLocalAnchorPointBody1 = new @Dimensionless Transform(transform1).inverse().multiply(jointInfo.anchorPointWorldSpace, new @Dimensionless Vector3());
        mLocalAnchorPointBody2 = new @Dimensionless Transform(transform2).inverse().multiply(jointInfo.anchorPointWorldSpace, new @Dimensionless Vector3());

        // Compute the local-space hinge axis
        new @Dimensionless Quaternion(transform1.getOrientation()).inverse().multiply(jointInfo.rotationAxisWorld, mHingeLocalAxisBody1);
        new @Dimensionless Quaternion(transform2.getOrientation()).inverse().multiply(jointInfo.rotationAxisWorld, mHingeLocalAxisBody2);
        mHingeLocalAxisBody1.normalize();
        mHingeLocalAxisBody2.normalize();

        // Compute the inverse of the initial orientation difference between the two bodies
        mInitOrientationDifferenceInv = new @Dimensionless Quaternion(transform2.getOrientation()).multiply(new @Dimensionless Quaternion(transform1.getOrientation()).inverse());
        mInitOrientationDifferenceInv.normalize();
        mInitOrientationDifferenceInv.inverse();
    }

    // Initialize before solving the constraint
    @Override
    public void initBeforeSolve(@Dimensionless HingeJoint this, @Dimensionless ConstraintSolverData constraintSolverData) {

        // Initialize the bodies index in the velocity array
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

        // Compute the vector from body center to the anchor point in world-space
        orientationBody1.multiply(mLocalAnchorPointBody1, mR1World);
        orientationBody2.multiply(mLocalAnchorPointBody2, mR2World);

        // Compute the current angle around the hinge axis
        @Dimensionless
        float hingeAngle = computeCurrentHingeAngle(orientationBody1, orientationBody2);

        // Check if the limit constraints are violated or not
        @Dimensionless
        float lowerLimitError = hingeAngle - mLowerLimit;
        @Dimensionless
        float upperLimitError = mUpperLimit - hingeAngle;
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

        // Compute vectors needed in the Jacobian
        orientationBody1.multiply(mHingeLocalAxisBody1, mA1);
        @Dimensionless
        Vector3 a2 = new @Dimensionless Vector3();
        orientationBody2.multiply(mHingeLocalAxisBody2, a2);
        mA1.normalize();
        a2.normalize();
        @Dimensionless
        Vector3 b2 = new @Dimensionless Vector3(a2).setUnitOrthogonal();
        @Dimensionless
        Vector3 c2 = new @Dimensionless Vector3(a2).cross(b2);
        mB2CrossA1 = new @Dimensionless Vector3(b2).cross(mA1);
        mC2CrossA1 = new @Dimensionless Vector3(c2).cross(mA1);

        // Compute the corresponding skew-symmetric matrices
        @Dimensionless
        Matrix3x3 skewSymmetricMatrixU1 = new @Dimensionless Matrix3x3().computeSkewSymmetricMatrixForCrossProduct(mR1World);
        @Dimensionless
        Matrix3x3 skewSymmetricMatrixU2 = new @Dimensionless Matrix3x3().computeSkewSymmetricMatrixForCrossProduct(mR2World);

        // Compute the inverse mass matrix K=JM^-1J^t for the 3 translation constraints (3x3 matrix)
        @Dimensionless
        float inverseMassBodies = ((@Dimensionless float) (0.0f));
        if (mBody1.isMotionEnabled()) {
            inverseMassBodies += mBody1.getMassInverse();
        }
        if (mBody2.isMotionEnabled()) {
            inverseMassBodies += mBody2.getMassInverse();
        }
        @Dimensionless
        Matrix3x3 massMatrix = new @Dimensionless Matrix3x3(inverseMassBodies, ((@Dimensionless float) (0.0f)), ((@Dimensionless float) (0.0f)),
                ((@Dimensionless float) (0.0f)), inverseMassBodies, ((@Dimensionless float) (0.0f)),
                ((@Dimensionless float) (0.0f)), ((@Dimensionless float) (0.0f)), inverseMassBodies);
        if (mBody1.isMotionEnabled()) {
            massMatrix.add(
                    new @Dimensionless Matrix3x3(skewSymmetricMatrixU1).multiply(new @Dimensionless Matrix3x3(mI1).multiply(new @Dimensionless Matrix3x3(skewSymmetricMatrixU1).transpose())));
        }
        if (mBody2.isMotionEnabled()) {
            massMatrix.add(
                    new @Dimensionless Matrix3x3(skewSymmetricMatrixU2).multiply(new @Dimensionless Matrix3x3(mI2).multiply(new @Dimensionless Matrix3x3(skewSymmetricMatrixU2).transpose())));
        }
        mInverseMassMatrixTranslation.zero();
        if (mBody1.isMotionEnabled() || mBody2.isMotionEnabled()) {
            mInverseMassMatrixTranslation = new @Dimensionless Matrix3x3(massMatrix).inverse();
        }

        // Compute the bias "b" of the translation constraints
        mBTranslation.zero();
        @Dimensionless
        float biasFactor = (BETA / constraintSolverData.timeStep);
        if (mPositionCorrectionTechnique == JointsPositionCorrectionTechnique.BAUMGARTE_JOINTS) {
            mBTranslation = new @Dimensionless Vector3(
                    new @Dimensionless Vector3(new @Dimensionless Vector3(new @Dimensionless Vector3(x2).add(mR2World)).subtract(x1)).subtract(mR1World)).multiply(biasFactor);
        }

        // Compute the inverse mass matrix K=JM^-1J^t for the 2 rotation constraints (2x2 matrix)
        @Dimensionless
        Vector3 I1B2CrossA1 = new @Dimensionless Vector3();
        @Dimensionless
        Vector3 I1C2CrossA1 = new @Dimensionless Vector3();
        @Dimensionless
        Vector3 I2B2CrossA1 = new @Dimensionless Vector3();
        @Dimensionless
        Vector3 I2C2CrossA1 = new @Dimensionless Vector3();
        if (mBody1.isMotionEnabled()) {
            I1B2CrossA1 = mI1.multiply(mB2CrossA1, new @Dimensionless Vector3());
            I1C2CrossA1 = mI1.multiply(mC2CrossA1, new @Dimensionless Vector3());
        }
        if (mBody2.isMotionEnabled()) {
            I2B2CrossA1 = mI2.multiply(mB2CrossA1, new @Dimensionless Vector3());
            I2C2CrossA1 = mI2.multiply(mC2CrossA1, new @Dimensionless Vector3());
        }
        @Dimensionless
        float el11 = mB2CrossA1.dot(I1B2CrossA1) + mB2CrossA1.dot(I2B2CrossA1);
        @Dimensionless
        float el12 = mB2CrossA1.dot(I1C2CrossA1) + mB2CrossA1.dot(I2C2CrossA1);
        @Dimensionless
        float el21 = mC2CrossA1.dot(I1B2CrossA1) + mC2CrossA1.dot(I2B2CrossA1);
        @Dimensionless
        float el22 = mC2CrossA1.dot(I1C2CrossA1) + mC2CrossA1.dot(I2C2CrossA1);
        @Dimensionless
        Matrix2x2 matrixKRotation = new @Dimensionless Matrix2x2(el11, el12, el21, el22);
        mInverseMassMatrixRotation.zero();
        if (mBody1.isMotionEnabled() || mBody2.isMotionEnabled()) {
            mInverseMassMatrixRotation = new @Dimensionless Matrix2x2(matrixKRotation).inverse();
        }

        // Compute the bias "b" of the rotation constraints
        mBRotation.zero();
        if (mPositionCorrectionTechnique == JointsPositionCorrectionTechnique.BAUMGARTE_JOINTS) {
            mBRotation = new @Dimensionless Vector2(mA1.dot(b2), mA1.dot(c2)).multiply(biasFactor);
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

        // If the motor or limits are enabled
        if (mIsMotorEnabled || (mIsLimitEnabled && (mIsLowerLimitViolated || mIsUpperLimitViolated))) {

            // Compute the inverse of the mass matrix K=JM^-1J^t for the limits and motor (1x1 matrix)
            mInverseMassMatrixLimitMotor = ((@Dimensionless float) (0.0f));
            if (mBody1.isMotionEnabled()) {
                mInverseMassMatrixLimitMotor += mA1.dot(mI1.multiply(mA1, new @Dimensionless Vector3()));
            }
            if (mBody2.isMotionEnabled()) {
                mInverseMassMatrixLimitMotor += mA1.dot(mI2.multiply(mA1, new @Dimensionless Vector3()));
            }
            mInverseMassMatrixLimitMotor = (mInverseMassMatrixLimitMotor > ((@Dimensionless float) (0.0f))) ? ((@Dimensionless float) (1.0f)) / mInverseMassMatrixLimitMotor : ((@Dimensionless float) (0.0f));

            if (mIsLimitEnabled) {

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
        }
    }

    // Warm start the constraint (apply the previous impulse at the beginning of the step)
    @Override
    public void warmstart(@Dimensionless HingeJoint this, @Dimensionless ConstraintSolverData constraintSolverData) {

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

        // Compute the impulse P=J^T * lambda for the 2 rotation constraints
        @Dimensionless
        Vector3 rotationImpulse = new @Dimensionless Vector3(
                new @Dimensionless Vector3(new @Dimensionless Vector3(mB2CrossA1).invert()).multiply(mImpulseRotation.getX())).subtract(
                        new @Dimensionless Vector3(mC2CrossA1).multiply(mImpulseRotation.getY()));

        // Compute the impulse P=J^T * lambda for the lower and upper limits constraints
        @Dimensionless
        Vector3 limitsImpulse = new @Dimensionless Vector3(mA1).multiply(mImpulseUpperLimit - mImpulseLowerLimit);

        // Compute the impulse P=J^T * lambda for the motor constraint
        @Dimensionless
        Vector3 motorImpulse = new @Dimensionless Vector3(mA1).multiply(-mImpulseMotor);

        if (mBody1.isMotionEnabled()) {

            // Compute the impulse P=J^T * lambda for the 3 translation constraints
            @Dimensionless
            Vector3 linearImpulseBody1 = new @Dimensionless Vector3(mImpulseTranslation).invert();
            @Dimensionless
            Vector3 angularImpulseBody1 = new @Dimensionless Vector3(mImpulseTranslation).cross(mR1World);

            // Compute the impulse P=J^T * lambda for the 2 rotation constraints
            angularImpulseBody1.add(rotationImpulse);

            // Compute the impulse P=J^T * lambda for the lower and upper limits constraints
            angularImpulseBody1.add(limitsImpulse);

            // Compute the impulse P=J^T * lambda for the motor constraint
            angularImpulseBody1.add(motorImpulse);

            // Apply the impulse to the body
            v1.add(new @Dimensionless Vector3(linearImpulseBody1).multiply(inverseMassBody1));
            w1.add(mI1.multiply(angularImpulseBody1, new @Dimensionless Vector3()));
        }
        if (mBody2.isMotionEnabled()) {

            // Compute the impulse P=J^T * lambda for the 3 translation constraints
            @Dimensionless
            Vector3 linearImpulseBody2 = mImpulseTranslation;
            @Dimensionless
            Vector3 angularImpulseBody2 = new @Dimensionless Vector3(mImpulseTranslation).cross(mR2World).invert();

            // Compute the impulse P=J^T * lambda for the 2 rotation constraints
            angularImpulseBody2.add(new @Dimensionless Vector3(rotationImpulse).invert());

            // Compute the impulse P=J^T * lambda for the lower and upper limits constraints
            angularImpulseBody2.add(new @Dimensionless Vector3(limitsImpulse).invert());

            // Compute the impulse P=J^T * lambda for the motor constraint
            angularImpulseBody2.add(new @Dimensionless Vector3(motorImpulse).invert());

            // Apply the impulse to the body
            v2.add(new @Dimensionless Vector3(linearImpulseBody2).multiply(inverseMassBody2));
            w2.add(mI2.multiply(angularImpulseBody2, new @Dimensionless Vector3()));
        }
    }

    // Solve the velocity constraint
    @Override
    public void solveVelocityConstraint(@Dimensionless HingeJoint this, @Dimensionless ConstraintSolverData constraintSolverData) {

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
        // Compute J*v
        @Dimensionless
        Vector3 JvTranslation = new @Dimensionless Vector3(
                new @Dimensionless Vector3(new @Dimensionless Vector3(v2).add(new @Dimensionless Vector3(w2).cross(mR2World))).subtract(v1)).subtract(new @Dimensionless Vector3(w1).cross(mR1World));

        // Compute the Lagrange multiplier lambda
        @Dimensionless
        Vector3 deltaLambdaTranslation = mInverseMassMatrixTranslation.multiply(
                new @Dimensionless Vector3(new @Dimensionless Vector3(JvTranslation).invert()).subtract(mBTranslation), new @Dimensionless Vector3());
        mImpulseTranslation.add(deltaLambdaTranslation);

        if (mBody1.isMotionEnabled()) {

            // Compute the impulse P=J^T * lambda
            @Dimensionless
            Vector3 linearImpulseBody1 = new @Dimensionless Vector3(deltaLambdaTranslation).invert();
            @Dimensionless
            Vector3 angularImpulseBody1 = new @Dimensionless Vector3(deltaLambdaTranslation).cross(mR1World);

            // Apply the impulse to the body
            v1.add(new @Dimensionless Vector3(linearImpulseBody1).multiply(inverseMassBody1));
            w1.add(mI1.multiply(angularImpulseBody1, new @Dimensionless Vector3()));
        }
        if (mBody2.isMotionEnabled()) {

            // Compute the impulse P=J^T * lambda
            @Dimensionless
            Vector3 linearImpulseBody2 = deltaLambdaTranslation;
            @Dimensionless
            Vector3 angularImpulseBody2 = new @Dimensionless Vector3(deltaLambdaTranslation).cross(mR2World).invert();

            // Apply the impulse to the body
            v2.add(new @Dimensionless Vector3(linearImpulseBody2).multiply(inverseMassBody2));
            w2.add(mI2.multiply(angularImpulseBody2, new @Dimensionless Vector3()));
        }

        /**
         * --------------- Rotation Constraints ---------------
         */
        // Compute J*v for the 2 rotation constraints
        @Dimensionless
        Vector2 JvRotation = new @Dimensionless Vector2(-mB2CrossA1.dot(w1) + mB2CrossA1.dot(w2), -mC2CrossA1.dot(w1) + mC2CrossA1.dot(w2));

        // Compute the Lagrange multiplier lambda for the 2 rotation constraints
        @Dimensionless
        Vector2 deltaLambdaRotation = new @Dimensionless Vector2();
        mInverseMassMatrixRotation.multiply(new @Dimensionless Vector2(JvRotation).invert().subtract(mBRotation), deltaLambdaRotation);
        mImpulseRotation.add(deltaLambdaRotation);

        if (mBody1.isMotionEnabled()) {

            // Compute the impulse P=J^T * lambda for the 2 rotation constraints
            @Dimensionless
            Vector3 angularImpulseBody1 = new @Dimensionless Vector3(
                    new @Dimensionless Vector3(new @Dimensionless Vector3(mB2CrossA1).invert()).multiply(deltaLambdaRotation.getX())).subtract(
                            new @Dimensionless Vector3(mC2CrossA1).multiply(deltaLambdaRotation.getY()));

            // Apply the impulse to the body
            w1.add(mI1.multiply(angularImpulseBody1, new @Dimensionless Vector3()));
        }
        if (mBody2.isMotionEnabled()) {

            // Compute the impulse P=J^T * lambda for the 2 rotation constraints
            @Dimensionless
            Vector3 angularImpulseBody2 = new @Dimensionless Vector3(
                    new @Dimensionless Vector3(mB2CrossA1).multiply(deltaLambdaRotation.getX())).add(
                            new @Dimensionless Vector3(mC2CrossA1).multiply(deltaLambdaRotation.getY()));

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
                float JvLowerLimit = new @Dimensionless Vector3(w2).subtract(w1).dot(mA1);

                // Compute the Lagrange multiplier lambda for the lower limit constraint
                @Dimensionless
                float deltaLambdaLower = mInverseMassMatrixLimitMotor * (-JvLowerLimit - mBLowerLimit);
                @Dimensionless
                float lambdaTemp = mImpulseLowerLimit;
                mImpulseLowerLimit = Math.max(mImpulseLowerLimit + deltaLambdaLower, ((@Dimensionless float) (0.0f)));
                deltaLambdaLower = mImpulseLowerLimit - lambdaTemp;

                if (mBody1.isMotionEnabled()) {

                    // Compute the impulse P=J^T * lambda for the lower limit constraint
                    @Dimensionless
                    Vector3 angularImpulseBody1 = new @Dimensionless Vector3(mA1).multiply(-deltaLambdaLower);

                    // Apply the impulse to the body
                    w1.add(mI1.multiply(angularImpulseBody1, new @Dimensionless Vector3()));
                }
                if (mBody2.isMotionEnabled()) {

                    // Compute the impulse P=J^T * lambda for the lower limit constraint
                    @Dimensionless
                    Vector3 angularImpulseBody2 = new @Dimensionless Vector3(mA1).multiply(deltaLambdaLower);

                    // Apply the impulse to the body
                    w2.add(mI2.multiply(angularImpulseBody2, new @Dimensionless Vector3()));
                }
            }

            // If the upper limit is violated
            if (mIsUpperLimitViolated) {

                // Compute J*v for the upper limit constraint
                @Dimensionless
                float JvUpperLimit = -new @Dimensionless Vector3(w2).subtract(w1).dot(mA1);

                // Compute the Lagrange multiplier lambda for the upper limit constraint
                @Dimensionless
                float deltaLambdaUpper = mInverseMassMatrixLimitMotor * (-JvUpperLimit - mBUpperLimit);
                @Dimensionless
                float lambdaTemp = mImpulseUpperLimit;
                mImpulseUpperLimit = Math.max(mImpulseUpperLimit + deltaLambdaUpper, ((@Dimensionless float) (0.0f)));
                deltaLambdaUpper = mImpulseUpperLimit - lambdaTemp;

                if (mBody1.isMotionEnabled()) {

                    // Compute the impulse P=J^T * lambda for the upper limit constraint
                    @Dimensionless
                    Vector3 angularImpulseBody1 = new @Dimensionless Vector3(mA1).multiply(deltaLambdaUpper);

                    // Apply the impulse to the body
                    w1.add(mI1.multiply(angularImpulseBody1, new @Dimensionless Vector3()));
                }
                if (mBody2.isMotionEnabled()) {

                    // Compute the impulse P=J^T * lambda for the upper limit constraint
                    @Dimensionless
                    Vector3 angularImpulseBody2 = new @Dimensionless Vector3(mA1).multiply(-deltaLambdaUpper);

                    // Apply the impulse to the body
                    w2.add(mI2.multiply(angularImpulseBody2, new @Dimensionless Vector3()));
                }
            }
        }

        /**
         * --------------- Motor ---------------
         */
        // If the motor is enabled
        if (mIsMotorEnabled) {

            // Compute J*v for the motor
            @Dimensionless
            float JvMotor = mA1.dot(new @Dimensionless Vector3(w1).subtract(w2));

            // Compute the Lagrange multiplier lambda for the motor
            @Dimensionless
            float maxMotorImpulse = mMaxMotorTorque * constraintSolverData.timeStep;
            @Dimensionless
            float deltaLambdaMotor = mInverseMassMatrixLimitMotor * (-JvMotor - mMotorSpeed);
            @Dimensionless
            float lambdaTemp = mImpulseMotor;
            mImpulseMotor = Mathematics.Clamp(mImpulseMotor + deltaLambdaMotor, -maxMotorImpulse, maxMotorImpulse);
            deltaLambdaMotor = mImpulseMotor - lambdaTemp;

            if (mBody1.isMotionEnabled()) {

                // Compute the impulse P=J^T * lambda for the motor
                @Dimensionless
                Vector3 angularImpulseBody1 = new @Dimensionless Vector3(mA1).multiply(-deltaLambdaMotor);

                // Apply the impulse to the body
                w1.add(mI1.multiply(angularImpulseBody1, new @Dimensionless Vector3()));
            }
            if (mBody2.isMotionEnabled()) {

                // Compute the impulse P=J^T * lambda for the motor
                @Dimensionless
                Vector3 angularImpulseBody2 = new @Dimensionless Vector3(mA1).multiply(deltaLambdaMotor);

                // Apply the impulse to the body
                w2.add(mI2.multiply(angularImpulseBody2, new @Dimensionless Vector3()));
            }
        }
    }

    // Solve the position constraint (for position error correction)
    @Override
    public void solvePositionConstraint(@Dimensionless HingeJoint this, @Dimensionless ConstraintSolverData constraintSolverData) {

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

        // Recompute the inverse inertia tensors
        mI1 = mBody1.getInertiaTensorInverseWorld();
        mI2 = mBody2.getInertiaTensorInverseWorld();

        // Compute the vector from body center to the anchor point in world-space
        q1.multiply(mLocalAnchorPointBody1, mR1World);
        q2.multiply(mLocalAnchorPointBody2, mR2World);

        // Compute the current angle around the hinge axis
        @Dimensionless
        float hingeAngle = computeCurrentHingeAngle(q1, q2);

        // Check if the limit constraints are violated or not
        @Dimensionless
        float lowerLimitError = hingeAngle - mLowerLimit;
        @Dimensionless
        float upperLimitError = mUpperLimit - hingeAngle;
        mIsLowerLimitViolated = lowerLimitError <= ((@Dimensionless float) (0.0f));
        mIsUpperLimitViolated = upperLimitError <= ((@Dimensionless float) (0.0f));

        // Compute vectors needed in the Jacobian
        q1.multiply(mHingeLocalAxisBody1, mA1);
        @Dimensionless
        Vector3 a2 = new @Dimensionless Vector3();
        q2.multiply(mHingeLocalAxisBody2, a2);
        mA1.normalize();
        a2.normalize();
        @Dimensionless
        Vector3 b2 = new @Dimensionless Vector3(a2).setUnitOrthogonal();
        @Dimensionless
        Vector3 c2 = new @Dimensionless Vector3(a2).cross(b2);
        mB2CrossA1 = new @Dimensionless Vector3(b2).cross(mA1);
        mC2CrossA1 = new @Dimensionless Vector3(c2).cross(mA1);

        // Compute the corresponding skew-symmetric matrices
        @Dimensionless
        Matrix3x3 skewSymmetricMatrixU1 = new @Dimensionless Matrix3x3().computeSkewSymmetricMatrixForCrossProduct(mR1World);
        @Dimensionless
        Matrix3x3 skewSymmetricMatrixU2 = new @Dimensionless Matrix3x3().computeSkewSymmetricMatrixForCrossProduct(mR2World);

        /**
         * --------------- Translation Constraints ---------------
         */
        // Compute the matrix K=JM^-1J^t (3x3 matrix) for the 3 translation constraints
        @Dimensionless
        float inverseMassBodies = ((@Dimensionless float) (0.0f));
        if (mBody1.isMotionEnabled()) {
            inverseMassBodies += mBody1.getMassInverse();
        }
        if (mBody2.isMotionEnabled()) {
            inverseMassBodies += mBody2.getMassInverse();
        }
        @Dimensionless
        Matrix3x3 massMatrix = new @Dimensionless Matrix3x3(inverseMassBodies, ((@Dimensionless float) (0.0f)), ((@Dimensionless float) (0.0f)),
                ((@Dimensionless float) (0.0f)), inverseMassBodies, ((@Dimensionless float) (0.0f)),
                ((@Dimensionless float) (0.0f)), ((@Dimensionless float) (0.0f)), inverseMassBodies);
        if (mBody1.isMotionEnabled()) {
            massMatrix.add(
                    new @Dimensionless Matrix3x3(skewSymmetricMatrixU1).multiply(new @Dimensionless Matrix3x3(mI1).multiply(new @Dimensionless Matrix3x3(skewSymmetricMatrixU1).transpose())));
        }
        if (mBody2.isMotionEnabled()) {
            massMatrix.add(
                    new @Dimensionless Matrix3x3(skewSymmetricMatrixU2).multiply(new @Dimensionless Matrix3x3(mI2).multiply(new @Dimensionless Matrix3x3(skewSymmetricMatrixU2).transpose())));
        }
        mInverseMassMatrixTranslation.zero();
        if (mBody1.isMotionEnabled() || mBody2.isMotionEnabled()) {
            mInverseMassMatrixTranslation = new @Dimensionless Matrix3x3(massMatrix).inverse();
        }

        // Compute position error for the 3 translation constraints
        @Dimensionless
        Vector3 errorTranslation = new @Dimensionless Vector3(
                new @Dimensionless Vector3(new @Dimensionless Vector3(x2).add(mR2World)).subtract(x1)).subtract(mR1World);

        // Compute the Lagrange multiplier lambda
        @Dimensionless
        Vector3 lambdaTranslation = mInverseMassMatrixTranslation.multiply(
                new @Dimensionless Vector3(errorTranslation).invert(), new @Dimensionless Vector3());

        // Apply the impulse to the bodies of the joint
        if (mBody1.isMotionEnabled()) {

            // Compute the impulse
            @Dimensionless
            Vector3 linearImpulseBody1 = new @Dimensionless Vector3(lambdaTranslation).invert();
            @Dimensionless
            Vector3 angularImpulseBody1 = new @Dimensionless Vector3(lambdaTranslation).cross(mR1World);

            // Compute the pseudo velocity
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

            // Compute the impulse
            @Dimensionless
            Vector3 linearImpulseBody2 = lambdaTranslation;
            @Dimensionless
            Vector3 angularImpulseBody2 = new @Dimensionless Vector3(lambdaTranslation).cross(mR2World).invert();

            // Compute the pseudo velocity
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
        // Compute the inverse mass matrix K=JM^-1J^t for the 2 rotation constraints (2x2 matrix)
        @Dimensionless
        Vector3 I1B2CrossA1 = new @Dimensionless Vector3();
        @Dimensionless
        Vector3 I1C2CrossA1 = new @Dimensionless Vector3();
        @Dimensionless
        Vector3 I2B2CrossA1 = new @Dimensionless Vector3();
        @Dimensionless
        Vector3 I2C2CrossA1 = new @Dimensionless Vector3();
        if (mBody1.isMotionEnabled()) {
            I1B2CrossA1 = mI1.multiply(mB2CrossA1, new @Dimensionless Vector3());
            I1C2CrossA1 = mI1.multiply(mC2CrossA1, new @Dimensionless Vector3());
        }
        if (mBody2.isMotionEnabled()) {
            I2B2CrossA1 = mI2.multiply(mB2CrossA1, new @Dimensionless Vector3());
            I2C2CrossA1 = mI2.multiply(mC2CrossA1, new @Dimensionless Vector3());
        }
        @Dimensionless
        float el11 = mB2CrossA1.dot(I1B2CrossA1) + mB2CrossA1.dot(I2B2CrossA1);
        @Dimensionless
        float el12 = mB2CrossA1.dot(I1C2CrossA1) + mB2CrossA1.dot(I2C2CrossA1);
        @Dimensionless
        float el21 = mC2CrossA1.dot(I1B2CrossA1) + mC2CrossA1.dot(I2B2CrossA1);
        @Dimensionless
        float el22 = mC2CrossA1.dot(I1C2CrossA1) + mC2CrossA1.dot(I2C2CrossA1);
        @Dimensionless
        Matrix2x2 matrixKRotation = new @Dimensionless Matrix2x2(el11, el12, el21, el22);
        mInverseMassMatrixRotation.zero();
        if (mBody1.isMotionEnabled() || mBody2.isMotionEnabled()) {
            mInverseMassMatrixRotation = new @Dimensionless Matrix2x2(matrixKRotation).inverse();
        }

        // Compute the position error for the 3 rotation constraints
        @Dimensionless
        Vector2 errorRotation = new @Dimensionless Vector2(mA1.dot(b2), mA1.dot(c2));

        // Compute the Lagrange multiplier lambda for the 3 rotation constraints
        @Dimensionless
        Vector2 lambdaRotation = new @Dimensionless Vector2();
        mInverseMassMatrixRotation.multiply(new @Dimensionless Vector2(errorRotation).invert(), lambdaRotation);

        // Apply the impulse to the bodies of the joint
        if (mBody1.isMotionEnabled()) {

            // Compute the impulse P=J^T * lambda for the 3 rotation constraints
            @Dimensionless
            Vector3 angularImpulseBody1 = new @Dimensionless Vector3(
                    new @Dimensionless Vector3(new @Dimensionless Vector3(mB2CrossA1).invert()).multiply(lambdaRotation.getX())).subtract(
                            new @Dimensionless Vector3(mC2CrossA1).multiply(lambdaRotation.getY()));

            // Compute the pseudo velocity
            @Dimensionless
            Vector3 w1 = mI1.multiply(angularImpulseBody1, new @Dimensionless Vector3());

            // Update the body position/orientation
            q1.add(new @Dimensionless Quaternion(w1, ((@Dimensionless float) (0.0f))).multiply(q1).multiply(((@Dimensionless float) (0.5f))));
            q1.normalize();
        }
        if (mBody2.isMotionEnabled()) {

            // Compute the impulse
            @Dimensionless
            Vector3 angularImpulseBody2 = new @Dimensionless Vector3(
                    new @Dimensionless Vector3(mB2CrossA1).multiply(lambdaRotation.getX())).add(
                            new @Dimensionless Vector3(mC2CrossA1).multiply(lambdaRotation.getY()));

            // Compute the pseudo velocity
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
                mInverseMassMatrixLimitMotor = ((@Dimensionless float) (0.0f));
                if (mBody1.isMotionEnabled()) {
                    mInverseMassMatrixLimitMotor += mA1.dot(mI1.multiply(mA1, new @Dimensionless Vector3()));
                }
                if (mBody2.isMotionEnabled()) {
                    mInverseMassMatrixLimitMotor += mA1.dot(mI2.multiply(mA1, new @Dimensionless Vector3()));
                }
                mInverseMassMatrixLimitMotor = (mInverseMassMatrixLimitMotor > ((@Dimensionless float) (0.0f))) ? ((@Dimensionless float) (1.0f)) / mInverseMassMatrixLimitMotor : ((@Dimensionless float) (0.0f));
            }

            // If the lower limit is violated
            if (mIsLowerLimitViolated) {

                // Compute the Lagrange multiplier lambda for the lower limit constraint
                @Dimensionless
                float lambdaLowerLimit = mInverseMassMatrixLimitMotor * (-lowerLimitError);

                // Apply the impulse to the bodies of the joint
                if (mBody1.isMotionEnabled()) {

                    // Compute the impulse P=J^T * lambda
                    @Dimensionless
                    Vector3 angularImpulseBody1 = new @Dimensionless Vector3(mA1).multiply(-lambdaLowerLimit);

                    // Compute the pseudo velocity
                    @Dimensionless
                    Vector3 w1 = mI1.multiply(angularImpulseBody1, new @Dimensionless Vector3());

                    // Update the body position/orientation
                    q1.add(new @Dimensionless Quaternion(w1, ((@Dimensionless float) (0.0f))).multiply(q1).multiply(((@Dimensionless float) (0.5f))));
                    q1.normalize();
                }
                if (mBody2.isMotionEnabled()) {

                    // Compute the impulse P=J^T * lambda
                    @Dimensionless
                    Vector3 angularImpulseBody2 = new @Dimensionless Vector3(mA1).multiply(lambdaLowerLimit);

                    // Compute the pseudo velocity
                    @Dimensionless
                    Vector3 w2 = mI2.multiply(angularImpulseBody2, new @Dimensionless Vector3());

                    // Update the body position/orientation
                    q2.add(new @Dimensionless Quaternion(w2, ((@Dimensionless float) (0.0f))).multiply(q2).multiply(((@Dimensionless float) (0.5f))));
                    q2.normalize();
                }
            }

            // If the upper limit is violated
            if (mIsUpperLimitViolated) {

                // Compute the Lagrange multiplier lambda for the upper limit constraint
                @Dimensionless
                float lambdaUpperLimit = mInverseMassMatrixLimitMotor * (-upperLimitError);

                // Apply the impulse to the bodies of the joint
                if (mBody1.isMotionEnabled()) {

                    // Compute the impulse P=J^T * lambda
                    @Dimensionless
                    Vector3 angularImpulseBody1 = new @Dimensionless Vector3(mA1).multiply(lambdaUpperLimit);

                    // Compute the pseudo velocity
                    @Dimensionless
                    Vector3 w1 = mI1.multiply(angularImpulseBody1, new @Dimensionless Vector3());

                    // Update the body position/orientation
                    q1.add(new @Dimensionless Quaternion(w1, ((@Dimensionless float) (0.0f))).multiply(q1).multiply(((@Dimensionless float) (0.5f))));
                    q1.normalize();
                }
                if (mBody2.isMotionEnabled()) {

                    // Compute the impulse P=J^T * lambda
                    @Dimensionless
                    Vector3 angularImpulseBody2 = new @Dimensionless Vector3(mA1).multiply(-lambdaUpperLimit);

                    // Compute the pseudo velocity
                    @Dimensionless
                    Vector3 w2 = mI2.multiply(angularImpulseBody2, new @Dimensionless Vector3());

                    // Update the body position/orientation
                    q2.add(new @Dimensionless Quaternion(w2, ((@Dimensionless float) (0.0f))).multiply(q2).multiply(((@Dimensionless float) (0.5f))));
                    q2.normalize();
                }
            }
        }
    }

    // Return true if the limits or the joint are enabled
    public @Dimensionless boolean isLimitEnabled(@Dimensionless HingeJoint this) {
        return mIsLimitEnabled;
    }

    // Return true if the motor of the joint is enabled
    public @Dimensionless boolean isMotorEnabled(@Dimensionless HingeJoint this) {
        return mIsMotorEnabled;
    }

    // Return the minimum angle limit
    public @Dimensionless float getMinAngleLimit(@Dimensionless HingeJoint this) {
        return mLowerLimit;
    }

    // Return the maximum angle limit
    public @Dimensionless float getMaxAngleLimit(@Dimensionless HingeJoint this) {
        return mUpperLimit;
    }

    // Return the motor speed
    public @Dimensionless float getMotorSpeed(@Dimensionless HingeJoint this) {
        return mMotorSpeed;
    }

    // Return the maximum motor torque
    public @Dimensionless float getMaxMotorTorque(@Dimensionless HingeJoint this) {
        return mMaxMotorTorque;
    }

    // Return the intensity of the current torque applied for the joint motor
    public @Dimensionless float getMotorTorque(@Dimensionless HingeJoint this, @Dimensionless float timeStep) {
        return mImpulseMotor / timeStep;
    }

    // Enable/Disable the limits of the joint
    public void enableLimit(@Dimensionless HingeJoint this, @Dimensionless boolean isLimitEnabled) {

        if (isLimitEnabled != mIsLimitEnabled) {

            mIsLimitEnabled = isLimitEnabled;

            // Reset the limits
            resetLimits();
        }
    }

    // Enable/Disable the motor of the joint
    public void enableMotor(@Dimensionless HingeJoint this, @Dimensionless boolean isMotorEnabled) {

        mIsMotorEnabled = isMotorEnabled;
        mImpulseMotor = ((@Dimensionless float) (0.0f));

        // Wake up the two bodies of the joint
        mBody1.setIsSleeping(false);
        mBody2.setIsSleeping(false);
    }

    // Set the minimum angle limit
    public void setMinAngleLimit(@Dimensionless HingeJoint this, @Dimensionless float lowerLimit) {

        assert (mLowerLimit <= ((@Dimensionless float) (0.0f)) && mLowerLimit >= - ((@Dimensionless double) (2.0)) * Defaults.PI);

        if (lowerLimit != mLowerLimit) {

            mLowerLimit = lowerLimit;

            // Reset the limits
            resetLimits();
        }
    }

    // Set the maximum angle limit
    public void setMaxAngleLimit(@Dimensionless HingeJoint this, @Dimensionless float upperLimit) {

        assert (upperLimit >= ((@Dimensionless float) (0.0f)) && upperLimit <= ((@Dimensionless float) (2.0f)) * Defaults.PI);

        if (upperLimit != mUpperLimit) {

            mUpperLimit = upperLimit;

            // Reset the limits
            resetLimits();
        }
    }

    // Set the motor speed
    public void setMotorSpeed(@Dimensionless HingeJoint this, @Dimensionless float motorSpeed) {

        if (motorSpeed != mMotorSpeed) {

            mMotorSpeed = motorSpeed;

            // Wake up the two bodies of the joint
            mBody1.setIsSleeping(false);
            mBody2.setIsSleeping(false);
        }
    }

    // Set the maximum motor torque
    public void setMaxMotorTorque(@Dimensionless HingeJoint this, @Dimensionless float maxMotorTorque) {

        if (maxMotorTorque != mMaxMotorTorque) {

            assert (mMaxMotorTorque >= ((@Dimensionless float) (0.0f)));
            mMaxMotorTorque = maxMotorTorque;

            // Wake up the two bodies of the joint
            mBody1.setIsSleeping(false);
            mBody2.setIsSleeping(false);
        }
    }

}
