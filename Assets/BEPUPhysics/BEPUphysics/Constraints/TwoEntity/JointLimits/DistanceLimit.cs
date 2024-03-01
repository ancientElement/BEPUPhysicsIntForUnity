using System;
using BEPUphysics.Entities;

using BEPUutilities;
using FixMath.NET;

namespace BEPUphysics.Constraints.TwoEntity.JointLimits
{
    /// <summary>
    /// A modified distance constraint allowing a range of lengths between two anchor points.
    /// </summary>
    public class DistanceLimit : JointLimit, I1DImpulseConstraintWithError, I1DJacobianConstraint
    {
        private Fix64 accumulatedImpulse;
        private Vector3 anchorA;

        private Vector3 anchorB;
        private Fix64 biasVelocity;
        private Vector3 jAngularA, jAngularB;
        private Vector3 jLinearA, jLinearB;
        private Fix64 error;

        private Vector3 localAnchorA;

        private Vector3 localAnchorB;

        /// <summary>
        /// Maximum distance allowed between the anchors.
        /// </summary>
        protected Fix64 maximumLength;

        /// <summary>
        /// Minimum distance maintained between the anchors.
        /// </summary>
        protected Fix64 minimumLength;


        private Vector3 offsetA, offsetB;
        private Fix64 velocityToImpulse;

        /// <summary>
        /// Constructs a distance limit joint.
        /// To finish the initialization, specify the connections (ConnectionA and ConnectionB) 
        /// as well as the WorldAnchorA and WorldAnchorB (or their entity-local versions)
        /// and the MinimumLength and MaximumLength.
        /// This constructor sets the constraint's IsActive property to false by default.
        /// </summary>
        public DistanceLimit()
        {
            IsActive = false;
        }

        /// <summary>
        /// Constructs a distance limit joint.
        /// </summary>
        /// <param name="connectionA">First body connected to the distance limit.</param>
        /// <param name="connectionB">Second body connected to the distance limit.</param>
        /// <param name="anchorA">Connection to the spring from the first connected body in world space.</param>
        /// <param name="anchorB"> Connection to the spring from the second connected body in world space.</param>
        /// <param name="minimumLength">Minimum distance maintained between the anchors.</param>
        /// <param name="maximumLength">Maximum distance allowed between the anchors.</param>
        public DistanceLimit(Entity connectionA, Entity connectionB, Vector3 anchorA, Vector3 anchorB, Fix64 minimumLength, Fix64 maximumLength)
        {
            ConnectionA = connectionA;
            ConnectionB = connectionB;
            MinimumLength = minimumLength;
            MaximumLength = maximumLength;

            WorldAnchorA = anchorA;
            WorldAnchorB = anchorB;
        }

        /// <summary>
        /// Gets or sets the first entity's connection point in local space.
        /// </summary>
        public Vector3 LocalAnchorA
        {
            get { return localAnchorA; }
            set
            {
                localAnchorA = value;
                Matrix3x3.Transform(ref localAnchorA, ref connectionA.orientationMatrix, out anchorA);
                anchorA += connectionA.position;
            }
        }

        /// <summary>
        /// Gets or sets the first entity's connection point in local space.
        /// </summary>
        public Vector3 LocalAnchorB
        {
            get { return localAnchorB; }
            set
            {
                localAnchorB = value;
                Matrix3x3.Transform(ref localAnchorB, ref connectionB.orientationMatrix, out anchorB);
                anchorB += connectionB.position;
            }
        }

        /// <summary>
        /// Gets or sets the maximum distance allowed between the anchors.
        /// </summary>
        public Fix64 MaximumLength
        {
            get { return maximumLength; }
            set
            {
                maximumLength = MathHelper.Max(F64.C0, value);
                minimumLength = MathHelper.Min(minimumLength, maximumLength);
            }
        }

        /// <summary>
        /// Gets or sets the minimum distance maintained between the anchors.
        /// </summary>
        public Fix64 MinimumLength
        {
            get { return minimumLength; }
            set
            {
                minimumLength = MathHelper.Max(F64.C0, value);
                maximumLength = MathHelper.Max(minimumLength, maximumLength);
            }
        }

        /// <summary>
        /// Gets or sets the connection to the distance constraint from the first connected body in world space.
        /// </summary>
        public Vector3 WorldAnchorA
        {
            get { return anchorA; }
            set
            {
                anchorA = value;
                localAnchorA = Quaternion.Transform(anchorA - connectionA.position, Quaternion.Conjugate(connectionA.orientation));
            }
        }

        /// <summary>
        /// Gets or sets the connection to the distance constraint from the second connected body in world space.
        /// </summary>
        public Vector3 WorldAnchorB
        {
            get { return anchorB; }
            set
            {
                anchorB = value;
                localAnchorB = Quaternion.Transform(anchorB - connectionB.position, Quaternion.Conjugate(connectionB.orientation));
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
                if (isLimitActive)
                {
                    Fix64 lambda, dot;
                    Vector3.Dot(ref jLinearA, ref connectionA.linearVelocity, out lambda);
                    Vector3.Dot(ref jAngularA, ref connectionA.angularVelocity, out dot);
                    lambda += dot;
                    Vector3.Dot(ref jLinearB, ref connectionB.linearVelocity, out dot);
                    lambda += dot;
                    Vector3.Dot(ref jAngularB, ref connectionB.angularVelocity, out dot);
                    lambda += dot;
                    return lambda;
                }
                return F64.C0;
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
            jacobian = jLinearA;
        }

        /// <summary>
        /// Gets the linear jacobian entry for the second connected entity.
        /// </summary>
        /// <param name="jacobian">Linear jacobian entry for the second connected entity.</param>
        public void GetLinearJacobianB(out Vector3 jacobian)
        {
            jacobian = jLinearB;
        }

        /// <summary>
        /// Gets the angular jacobian entry for the first connected entity.
        /// </summary>
        /// <param name="jacobian">Angular jacobian entry for the first connected entity.</param>
        public void GetAngularJacobianA(out Vector3 jacobian)
        {
            jacobian = jAngularA;
        }

        /// <summary>
        /// Gets the angular jacobian entry for the second connected entity.
        /// </summary>
        /// <param name="jacobian">Angular jacobian entry for the second connected entity.</param>
        public void GetAngularJacobianB(out Vector3 jacobian)
        {
            jacobian = jAngularB;
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
        /// Calculates and applies corrective impulses.
        /// Called automatically by space.
        /// </summary>
        public override Fix64 SolveIteration()
        {
            //Compute the current relative velocity.
            Fix64 lambda, dot;
            Vector3.Dot(ref jLinearA, ref connectionA.linearVelocity, out lambda);
            Vector3.Dot(ref jAngularA, ref connectionA.angularVelocity, out dot);
            lambda += dot;
            Vector3.Dot(ref jLinearB, ref connectionB.linearVelocity, out dot);
            lambda += dot;
            Vector3.Dot(ref jAngularB, ref connectionB.angularVelocity, out dot);
            lambda += dot;

            //Add in the constraint space bias velocity
            lambda = -lambda + biasVelocity - softness * accumulatedImpulse;

            //Transform to an impulse
            lambda *= velocityToImpulse;

            //Clamp accumulated impulse (can't go negative)
            Fix64 previousAccumulatedImpulse = accumulatedImpulse;
            accumulatedImpulse = MathHelper.Max(accumulatedImpulse + lambda, F64.C0);
            lambda = accumulatedImpulse - previousAccumulatedImpulse;

            //Apply the impulse
            Vector3 impulse;
            if (connectionA.isDynamic)
            {
                Vector3.Multiply(ref jLinearA, lambda, out impulse);
                connectionA.ApplyLinearImpulse(ref impulse);
                Vector3.Multiply(ref jAngularA, lambda, out impulse);
                connectionA.ApplyAngularImpulse(ref impulse);
            }
            if (connectionB.isDynamic)
            {
                Vector3.Multiply(ref jLinearB, lambda, out impulse);
                connectionB.ApplyLinearImpulse(ref impulse);
                Vector3.Multiply(ref jAngularB, lambda, out impulse);
                connectionB.ApplyAngularImpulse(ref impulse);
            }

            return (Fix64.Abs(lambda));
        }

        /// <summary>
        /// Calculates necessary information for velocity solving.
        /// </summary>
        /// <param name="dt">Time in seconds since the last update.</param>
        public override void Update(Fix64 dt)
        {
            //Transform the anchors and offsets into world space.
            Matrix3x3.Transform(ref localAnchorA, ref connectionA.orientationMatrix, out offsetA);
            Matrix3x3.Transform(ref localAnchorB, ref connectionB.orientationMatrix, out offsetB);
            Vector3.Add(ref connectionA.position, ref offsetA, out anchorA);
            Vector3.Add(ref connectionB.position, ref offsetB, out anchorB);

            //Compute the distance.
            Vector3 separation;
            Vector3.Subtract(ref anchorB, ref anchorA, out separation);
            Fix64 distance = separation.Length();
            if (distance <= maximumLength && distance >= minimumLength)
            {
                isActiveInSolver = false;
                accumulatedImpulse = F64.C0;
                error = F64.C0;
                isLimitActive = false;
                return;
            }
            isLimitActive = true;


            //Compute jacobians
            if (distance > maximumLength)
            {
                //If it's beyond the max, all of the jacobians are reversed compared to what they are when it's below the min.
                if (distance > Toolbox.Epsilon)
                {
                    jLinearA.X = separation.X / distance;
                    jLinearA.Y = separation.Y / distance;
                    jLinearA.Z = separation.Z / distance;
                }
                else
                    jLinearB = Toolbox.ZeroVector;

                jLinearB.X = -jLinearA.X;
                jLinearB.Y = -jLinearA.Y;
                jLinearB.Z = -jLinearA.Z;

                Vector3.Cross(ref jLinearA, ref offsetA, out jAngularA);
                //Still need to negate angular A.  It's done after the effective mass matrix.
                Vector3.Cross(ref jLinearA, ref offsetB, out jAngularB);
            }
            else
            {
                if (distance > Toolbox.Epsilon)
                {
                    jLinearB.X = separation.X / distance;
                    jLinearB.Y = separation.Y / distance;
                    jLinearB.Z = separation.Z / distance;
                }
                else
                    jLinearB = Toolbox.ZeroVector;

                jLinearA.X = -jLinearB.X;
                jLinearA.Y = -jLinearB.Y;
                jLinearA.Z = -jLinearB.Z;

                Vector3.Cross(ref offsetA, ref jLinearB, out jAngularA);
                //Still need to negate angular A.  It's done after the effective mass matrix.
                Vector3.Cross(ref offsetB, ref jLinearB, out jAngularB);
            }


            //Debug.WriteLine("BiasVelocity: " + biasVelocity);


            //Compute effective mass matrix
            if (connectionA.isDynamic && connectionB.isDynamic)
            {
                Vector3 aAngular;
                Matrix3x3.Transform(ref jAngularA, ref connectionA.localInertiaTensorInverse, out aAngular);
                Vector3.Cross(ref aAngular, ref offsetA, out aAngular);
                Vector3 bAngular;
                Matrix3x3.Transform(ref jAngularB, ref connectionB.localInertiaTensorInverse, out bAngular);
                Vector3.Cross(ref bAngular, ref offsetB, out bAngular);
                Vector3.Add(ref aAngular, ref bAngular, out aAngular);
                Vector3.Dot(ref aAngular, ref jLinearB, out velocityToImpulse);
                velocityToImpulse += connectionA.inverseMass + connectionB.inverseMass;
            }
            else if (connectionA.isDynamic)
            {
                Vector3 aAngular;
                Matrix3x3.Transform(ref jAngularA, ref connectionA.localInertiaTensorInverse, out aAngular);
                Vector3.Cross(ref aAngular, ref offsetA, out aAngular);
                Vector3.Dot(ref aAngular, ref jLinearB, out velocityToImpulse);
                velocityToImpulse += connectionA.inverseMass;
            }
            else if (connectionB.isDynamic)
            {
                Vector3 bAngular;
                Matrix3x3.Transform(ref jAngularB, ref connectionB.localInertiaTensorInverse, out bAngular);
                Vector3.Cross(ref bAngular, ref offsetB, out bAngular);
                Vector3.Dot(ref bAngular, ref jLinearB, out velocityToImpulse);
                velocityToImpulse += connectionB.inverseMass;
            }
            else
            {
                //No point in trying to solve with two kinematics.
                isActiveInSolver = false;
                accumulatedImpulse = F64.C0;
                return;
            }

            Fix64 errorReduction;
            springSettings.ComputeErrorReductionAndSoftness(dt, F64.C1 / dt, out errorReduction, out softness);

            velocityToImpulse = F64.C1 / (softness + velocityToImpulse);
            //Finish computing jacobian; it's down here as an optimization (since it didn't need to be negated in mass matrix)
            jAngularA.X = -jAngularA.X;
            jAngularA.Y = -jAngularA.Y;
            jAngularA.Z = -jAngularA.Z;

            //Compute bias velocity
            if (distance > maximumLength)
                error = MathHelper.Max(F64.C0, distance - maximumLength - Margin);
            else
                error = MathHelper.Max(F64.C0, minimumLength - Margin - distance);
            biasVelocity = MathHelper.Min(errorReduction * error, maxCorrectiveVelocity);
            if (bounciness > F64.C0)
            {
                //Compute currently relative velocity for bounciness.
                Fix64 relativeVelocity, dot;
                Vector3.Dot(ref jLinearA, ref connectionA.linearVelocity, out relativeVelocity);
                Vector3.Dot(ref jAngularA, ref connectionA.angularVelocity, out dot);
                relativeVelocity += dot;
                Vector3.Dot(ref jLinearB, ref connectionB.linearVelocity, out dot);
                relativeVelocity += dot;
                Vector3.Dot(ref jAngularB, ref connectionB.angularVelocity, out dot);
                relativeVelocity += dot;
                biasVelocity = MathHelper.Max(biasVelocity, ComputeBounceVelocity(-relativeVelocity));
            }


        }

        /// <summary>
        /// Performs any pre-solve iteration work that needs exclusive
        /// access to the members of the solver updateable.
        /// Usually, this is used for applying warmstarting impulses.
        /// </summary>
        public override void ExclusiveUpdate()
        {
            //Warm starting
            Vector3 impulse;
            if (connectionA.isDynamic)
            {
                Vector3.Multiply(ref jLinearA, accumulatedImpulse, out impulse);
                connectionA.ApplyLinearImpulse(ref impulse);
                Vector3.Multiply(ref jAngularA, accumulatedImpulse, out impulse);
                connectionA.ApplyAngularImpulse(ref impulse);
            }
            if (connectionB.isDynamic)
            {
                Vector3.Multiply(ref jLinearB, accumulatedImpulse, out impulse);
                connectionB.ApplyLinearImpulse(ref impulse);
                Vector3.Multiply(ref jAngularB, accumulatedImpulse, out impulse);
                connectionB.ApplyAngularImpulse(ref impulse);
            }
        }
    }
}