﻿using System;
using BEPUphysics.Entities;
 
using BEPUutilities;
using FixMath.NET;

namespace BEPUphysics.Constraints.TwoEntity.Joints
{
    /// <summary>
    /// Constrains two bodies so that they can rotate relative to each other like a modified door hinge.
    /// Instead of removing two degrees of freedom, only one is removed so that the second connection to the constraint can twist.
    /// </summary>
    public class SwivelHingeAngularJoint : Joint, I1DImpulseConstraintWithError, I1DJacobianConstraint
    {
        private Fix64 accumulatedImpulse;
        private Fix64 biasVelocity;
        private Vector3 jacobianA, jacobianB;
        private Fix64 error;
        private Vector3 localHingeAxis;
        private Vector3 localTwistAxis;
        private Vector3 worldHingeAxis;
        private Vector3 worldTwistAxis;
        private Fix64 velocityToImpulse;

        /// <summary>
        /// Constructs a new constraint which allows relative angular motion around a hinge axis and a twist axis.
        /// To finish the initialization, specify the connections (ConnectionA and ConnectionB) 
        /// as well as the WorldHingeAxis and WorldTwistAxis (or their entity-local versions).
        /// This constructor sets the constraint's IsActive property to false by default.
        /// </summary>
        public SwivelHingeAngularJoint()
        {
            IsActive = false;
        }

        /// <summary>
        /// Constructs a new constraint which allows relative angular motion around a hinge axis and a twist axis.
        /// </summary>
        /// <param name="connectionA">First connection of the pair.</param>
        /// <param name="connectionB">Second connection of the pair.</param>
        /// <param name="worldHingeAxis">Hinge axis attached to connectionA.
        /// The connected entities will be able to rotate around this axis relative to each other.</param>
        /// <param name="worldTwistAxis">Twist axis attached to connectionB.
        /// The connected entities will be able to rotate around this axis relative to each other.</param>
        public SwivelHingeAngularJoint(Entity connectionA, Entity connectionB, Vector3 worldHingeAxis, Vector3 worldTwistAxis)
        {
            ConnectionA = connectionA;
            ConnectionB = connectionB;
            WorldHingeAxis = worldHingeAxis;
            WorldTwistAxis = worldTwistAxis;
        }

        /// <summary>
        /// Gets or sets the hinge axis attached to entity A in its local space.
        /// </summary>
        public Vector3 LocalHingeAxis
        {
            get { return localHingeAxis; }
            set
            {
                localHingeAxis = Vector3.Normalize(value);
                Matrix3x3.Transform(ref localHingeAxis, ref connectionA.orientationMatrix, out worldHingeAxis);
            }
        }

        /// <summary>
        /// Gets or sets the twist axis attached to entity B in its local space.
        /// </summary>
        public Vector3 LocalTwistAxis
        {
            get { return localTwistAxis; }
            set
            {
                localTwistAxis = Vector3.Normalize(value);
                Matrix3x3.Transform(ref localTwistAxis, ref connectionB.orientationMatrix, out worldTwistAxis);
            }
        }

        /// <summary>
        /// Gets or sets the hinge axis attached to entity A in world space.
        /// </summary>
        public Vector3 WorldHingeAxis
        {
            get { return worldHingeAxis; }
            set
            {
                worldHingeAxis = Vector3.Normalize(value);
                Quaternion conjugate;
                Quaternion.Conjugate(ref connectionA.orientation, out conjugate);
                Quaternion.Transform(ref worldHingeAxis, ref conjugate, out localHingeAxis);
            }
        }

        /// <summary>
        /// Gets or sets the axis attached to the first connected entity in world space.
        /// </summary>
        public Vector3 WorldTwistAxis
        {
            get { return worldTwistAxis; }
            set
            {
                worldTwistAxis = Vector3.Normalize(value);
                Quaternion conjugate;
                Quaternion.Conjugate(ref connectionB.orientation, out conjugate);
                Quaternion.Transform(ref worldTwistAxis, ref conjugate, out localTwistAxis);
            }
        }

        #region I1DImpulseConstraintWithError Members

        /// <summary>
        /// Gets the current relative velocity between the connected entities with respect to the constraint.
        /// </summary>
        public Fix64 RelativeVelocity
        {
            get
            {
                Fix64 velocityA, velocityB;
                //Find the velocity contribution from each connection
                Vector3.Dot(ref connectionA.angularVelocity, ref jacobianA, out velocityA);
                Vector3.Dot(ref connectionB.angularVelocity, ref jacobianB, out velocityB);
                return velocityA + velocityB;
            }
        }

        /// <summary>
        /// Gets the total impulse applied by this constraint.
        /// </summary>
        public Fix64 TotalImpulse
        {
            get { return accumulatedImpulse; }
        }

        /// <summary>
        /// Gets the current constraint error.
        /// </summary>
        public Fix64 Error
        {
            get { return error; }
        }

        #endregion

        #region I1DJacobianConstraint Members

        /// <summary>
        /// Gets the linear jacobian entry for the first connected entity.
        /// </summary>
        /// <param name="jacobian">Linear jacobian entry for the first connected entity.</param>
        public void GetLinearJacobianA(out Vector3 jacobian)
        {
            jacobian = Toolbox.ZeroVector;
        }

        /// <summary>
        /// Gets the linear jacobian entry for the second connected entity.
        /// </summary>
        /// <param name="jacobian">Linear jacobian entry for the second connected entity.</param>
        public void GetLinearJacobianB(out Vector3 jacobian)
        {
            jacobian = Toolbox.ZeroVector;
        }

        /// <summary>
        /// Gets the angular jacobian entry for the first connected entity.
        /// </summary>
        /// <param name="jacobian">Angular jacobian entry for the first connected entity.</param>
        public void GetAngularJacobianA(out Vector3 jacobian)
        {
            jacobian = jacobianA;
        }

        /// <summary>
        /// Gets the angular jacobian entry for the second connected entity.
        /// </summary>
        /// <param name="jacobian">Angular jacobian entry for the second connected entity.</param>
        public void GetAngularJacobianB(out Vector3 jacobian)
        {
            jacobian = jacobianB;
        }

        /// <summary>
        /// Gets the mass matrix of the constraint.
        /// </summary>
        /// <param name="outputMassMatrix">Constraint's mass matrix.</param>
        public void GetMassMatrix(out Fix64 outputMassMatrix)
        {
            outputMassMatrix = velocityToImpulse;
        }

        #endregion

        /// <summary>
        /// Solves for velocity.
        /// </summary>
        public override Fix64 SolveIteration()
        {
            Fix64 velocityA, velocityB;
            //Find the velocity contribution from each connection
            Vector3.Dot(ref connectionA.angularVelocity, ref jacobianA, out velocityA);
            Vector3.Dot(ref connectionB.angularVelocity, ref jacobianB, out velocityB);
            //Add in the constraint space bias velocity
            Fix64 lambda = -(velocityA + velocityB) - biasVelocity - softness * accumulatedImpulse;

            //Transform to an impulse
            lambda *= velocityToImpulse;

            //Accumulate the impulse
            accumulatedImpulse += lambda;

            //Apply the impulse
            Vector3 impulse;
            if (connectionA.isDynamic)
            {
                Vector3.Multiply(ref jacobianA, lambda, out impulse);
                connectionA.ApplyAngularImpulse(ref impulse);
            }
            if (connectionB.isDynamic)
            {
                Vector3.Multiply(ref jacobianB, lambda, out impulse);
                connectionB.ApplyAngularImpulse(ref impulse);
            }

            return (Fix64.Abs(lambda));
        }

        /// <summary>
        /// Do any necessary computations to prepare the constraint for this frame.
        /// </summary>
        /// <param name="dt">Simulation step length.</param>
        public override void Update(Fix64 dt)
        {
            //Transform the axes into world space.
            Matrix3x3.Transform(ref localHingeAxis, ref connectionA.orientationMatrix, out worldHingeAxis);
            Matrix3x3.Transform(ref localTwistAxis, ref connectionB.orientationMatrix, out worldTwistAxis);

            //****** VELOCITY BIAS ******//
            Vector3.Dot(ref worldHingeAxis, ref worldTwistAxis, out error);
            //Compute the correction velocity.

            Fix64 errorReduction;
            springSettings.ComputeErrorReductionAndSoftness(dt, F64.C1 / dt, out errorReduction, out softness);

            biasVelocity = MathHelper.Clamp(error * errorReduction, -maxCorrectiveVelocity, maxCorrectiveVelocity);

            //Compute the jacobian
            Vector3.Cross(ref worldHingeAxis, ref worldTwistAxis, out jacobianA);
            Fix64 length = jacobianA.LengthSquared();
            if (length > Toolbox.Epsilon)
                Vector3.Divide(ref jacobianA, Fix64.Sqrt(length), out jacobianA);
            else
                jacobianA = new Vector3();
            jacobianB.X = -jacobianA.X;
            jacobianB.Y = -jacobianA.Y;
            jacobianB.Z = -jacobianA.Z;


            //****** EFFECTIVE MASS MATRIX ******//
            //Connection A's contribution to the mass matrix
            Fix64 entryA;
            Vector3 transformedAxis;
            if (connectionA.isDynamic)
            {
                Matrix3x3.Transform(ref jacobianA, ref connectionA.inertiaTensorInverse, out transformedAxis);
                Vector3.Dot(ref transformedAxis, ref jacobianA, out entryA);
            }
            else
                entryA = F64.C0;

            //Connection B's contribution to the mass matrix
            Fix64 entryB;
            if (connectionB.isDynamic)
            {
                Matrix3x3.Transform(ref jacobianB, ref connectionB.inertiaTensorInverse, out transformedAxis);
                Vector3.Dot(ref transformedAxis, ref jacobianB, out entryB);
            }
            else
                entryB = F64.C0;

            //Compute the inverse mass matrix
            velocityToImpulse = F64.C1 / (softness + entryA + entryB);

            
        }

        /// <summary>
        /// Performs any pre-solve iteration work that needs exclusive
        /// access to the members of the solver updateable.
        /// Usually, this is used for applying warmstarting impulses.
        /// </summary>
        public override void ExclusiveUpdate()
        {
            //****** WARM STARTING ******//
            //Apply accumulated impulse
            Vector3 impulse;
            if (connectionA.isDynamic)
            {
                Vector3.Multiply(ref jacobianA, accumulatedImpulse, out impulse);
                connectionA.ApplyAngularImpulse(ref impulse);
            }
            if (connectionB.isDynamic)
            {
                Vector3.Multiply(ref jacobianB, accumulatedImpulse, out impulse);
                connectionB.ApplyAngularImpulse(ref impulse);
            }
        }
    }
}