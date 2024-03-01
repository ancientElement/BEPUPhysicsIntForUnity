using System;
using BEPUphysics.Entities;
using BEPUutilities;
 
using System.Diagnostics;
using FixMath.NET;

namespace BEPUphysics.Constraints.TwoEntity.Joints
{
    /// <summary>
    /// Connects two entities with a spherical joint.  Acts like an unrestricted shoulder joint.
    /// </summary>
    public class BallSocketJoint : Joint, I3DImpulseConstraintWithError, I3DJacobianConstraint
    {
        private Vector3 accumulatedImpulse;
        private Vector3 biasVelocity;
        private Vector3 localAnchorA;
        private Vector3 localAnchorB;
        private Matrix3x3 massMatrix;
        private Vector3 error;
        private Matrix3x3 rACrossProduct;
        private Matrix3x3 rBCrossProduct;
        private Vector3 worldOffsetA, worldOffsetB;

        /// <summary>
        /// Constructs a spherical joint.
        /// To finish the initialization, specify the connections (ConnectionA and ConnectionB) 
        /// as well as the offsets (OffsetA, OffsetB or LocalOffsetA, LocalOffsetB).
        /// This constructor sets the constraint's IsActive property to false by default.
        /// </summary>
        public BallSocketJoint()
        {
            IsActive = false;
        }

        /// <summary>
        /// Constructs a spherical joint.
        /// </summary>
        /// <param name="connectionA">First connected entity.</param>
        /// <param name="connectionB">Second connected entity.</param>
        /// <param name="anchorLocation">Location of the socket.</param>
        public BallSocketJoint(Entity connectionA, Entity connectionB, Vector3 anchorLocation)
        {
            ConnectionA = connectionA;
            ConnectionB = connectionB;

            OffsetA = anchorLocation - ConnectionA.position;
            OffsetB = anchorLocation - ConnectionB.position;
        }

        /// <summary>
        /// Gets or sets the offset from the first entity's center of mass to the anchor point in its local space.
        /// </summary>
        public Vector3 LocalOffsetA
        {
            get { return localAnchorA; }
            set
            {
                localAnchorA = value;
                Matrix3x3.Transform(ref localAnchorA, ref connectionA.orientationMatrix, out worldOffsetA); 
            }
        }

        /// <summary>
        /// Gets or sets the offset from the second entity's center of mass to the anchor point in its local space.
        /// </summary>
        public Vector3 LocalOffsetB
        {
            get { return localAnchorB; }
            set
            {
                localAnchorB = value;
                Matrix3x3.Transform(ref localAnchorB, ref connectionB.orientationMatrix, out worldOffsetB); 
            }
        }

        /// <summary>
        /// Gets or sets the offset from the first entity's center of mass to the anchor point in world space.
        /// </summary>
        public Vector3 OffsetA
        {
            get { return worldOffsetA; }
            set
            {
                worldOffsetA = value;
                Matrix3x3.TransformTranspose(ref worldOffsetA, ref connectionA.orientationMatrix, out localAnchorA);
            }
        }

        /// <summary>
        /// Gets or sets the offset from the second entity's center of mass to the anchor point in world space.
        /// </summary>
        public Vector3 OffsetB
        {
            get { return worldOffsetB; }
            set
            {
                worldOffsetB = value;
                Matrix3x3.TransformTranspose(ref worldOffsetB, ref connectionB.orientationMatrix, out localAnchorB);
            }
        }

        #region I3DImpulseConstraintWithError Members

        /// <summary>
        /// Gets the current relative velocity between the connected entities with respect to the constraint.
        /// </summary>
        public Vector3 RelativeVelocity
        {
            get
            {
                Vector3 cross;
                Vector3 aVel, bVel;
                Vector3.Cross(ref connectionA.angularVelocity, ref worldOffsetA, out cross);
                Vector3.Add(ref connectionA.linearVelocity, ref cross, out aVel);
                Vector3.Cross(ref connectionB.angularVelocity, ref worldOffsetB, out cross);
                Vector3.Add(ref connectionB.linearVelocity, ref cross, out bVel);
                return aVel - bVel;
            }
        }

        /// <summary>
        /// Gets the total impulse applied by this constraint.
        /// </summary>
        public Vector3 TotalImpulse
        {
            get { return accumulatedImpulse; }
        }

        /// <summary>
        /// Gets the current constraint error.
        /// </summary>
        public Vector3 Error
        {
            get { return error; }
        }

        #endregion

        #region I3DJacobianConstraint Members

        /// <summary>
        /// Gets the linear jacobian entry for the first connected entity.
        /// </summary>
        /// <param name="jacobianX">First linear jacobian entry for the first connected entity.</param>
        /// <param name="jacobianY">Second linear jacobian entry for the first connected entity.</param>
        /// <param name="jacobianZ">Third linear jacobian entry for the first connected entity.</param>
        public void GetLinearJacobianA(out Vector3 jacobianX, out Vector3 jacobianY, out Vector3 jacobianZ)
        {
            jacobianX = Toolbox.RightVector;
            jacobianY = Toolbox.UpVector;
            jacobianZ = Toolbox.BackVector;
        }

        /// <summary>
        /// Gets the linear jacobian entry for the second connected entity.
        /// </summary>
        /// <param name="jacobianX">First linear jacobian entry for the second connected entity.</param>
        /// <param name="jacobianY">Second linear jacobian entry for the second connected entity.</param>
        /// <param name="jacobianZ">Third linear jacobian entry for the second connected entity.</param>
        public void GetLinearJacobianB(out Vector3 jacobianX, out Vector3 jacobianY, out Vector3 jacobianZ)
        {
            jacobianX = Toolbox.RightVector;
            jacobianY = Toolbox.UpVector;
            jacobianZ = Toolbox.BackVector;
        }

        /// <summary>
        /// Gets the angular jacobian entry for the first connected entity.
        /// </summary>
        /// <param name="jacobianX">First angular jacobian entry for the first connected entity.</param>
        /// <param name="jacobianY">Second angular jacobian entry for the first connected entity.</param>
        /// <param name="jacobianZ">Third angular jacobian entry for the first connected entity.</param>
        public void GetAngularJacobianA(out Vector3 jacobianX, out Vector3 jacobianY, out Vector3 jacobianZ)
        {
            jacobianX = rACrossProduct.Right;
            jacobianY = rACrossProduct.Up;
            jacobianZ = rACrossProduct.Forward;
        }

        /// <summary>
        /// Gets the angular jacobian entry for the second connected entity.
        /// </summary>
        /// <param name="jacobianX">First angular jacobian entry for the second connected entity.</param>
        /// <param name="jacobianY">Second angular jacobian entry for the second connected entity.</param>
        /// <param name="jacobianZ">Third angular jacobian entry for the second connected entity.</param>
        public void GetAngularJacobianB(out Vector3 jacobianX, out Vector3 jacobianY, out Vector3 jacobianZ)
        {
            jacobianX = rBCrossProduct.Right;
            jacobianY = rBCrossProduct.Up;
            jacobianZ = rBCrossProduct.Forward;
        }

        /// <summary>
        /// Gets the mass matrix of the constraint.
        /// </summary>
        /// <param name="outputMassMatrix">Constraint's mass matrix.</param>
        public void GetMassMatrix(out Matrix3x3 outputMassMatrix)
        {
            outputMassMatrix = massMatrix;
        }

        #endregion


        /// <summary>
        /// Calculates necessary information for velocity solving.
        /// Called by preStep(Fix64 dt)
        /// </summary>
        /// <param name="dt">Time in seconds since the last update.</param>
        public override void Update(Fix64 dt)
        {
            Matrix3x3.Transform(ref localAnchorA, ref connectionA.orientationMatrix, out worldOffsetA);
            Matrix3x3.Transform(ref localAnchorB, ref connectionB.orientationMatrix, out worldOffsetB);


            Fix64 errorReductionParameter;
            springSettings.ComputeErrorReductionAndSoftness(dt, F64.C1 / dt, out errorReductionParameter, out softness);

            //Mass Matrix
            Matrix3x3 k;
            Matrix3x3 linearComponent;
            Matrix3x3.CreateCrossProduct(ref worldOffsetA, out rACrossProduct);
            Matrix3x3.CreateCrossProduct(ref worldOffsetB, out rBCrossProduct);
            if (connectionA.isDynamic && connectionB.isDynamic)
            {
                Matrix3x3.CreateScale(connectionA.inverseMass + connectionB.inverseMass, out linearComponent);
                Matrix3x3 angularComponentA, angularComponentB;
                Matrix3x3.Multiply(ref rACrossProduct, ref connectionA.inertiaTensorInverse, out angularComponentA);
                Matrix3x3.Multiply(ref rBCrossProduct, ref connectionB.inertiaTensorInverse, out angularComponentB);
                Matrix3x3.Multiply(ref angularComponentA, ref rACrossProduct, out angularComponentA);
                Matrix3x3.Multiply(ref angularComponentB, ref rBCrossProduct, out angularComponentB);
                Matrix3x3.Subtract(ref linearComponent, ref angularComponentA, out k);
                Matrix3x3.Subtract(ref k, ref angularComponentB, out k);
            }
            else if (connectionA.isDynamic && !connectionB.isDynamic)
            {
                Matrix3x3.CreateScale(connectionA.inverseMass, out linearComponent);
                Matrix3x3 angularComponentA;
                Matrix3x3.Multiply(ref rACrossProduct, ref connectionA.inertiaTensorInverse, out angularComponentA);
                Matrix3x3.Multiply(ref angularComponentA, ref rACrossProduct, out angularComponentA);
                Matrix3x3.Subtract(ref linearComponent, ref angularComponentA, out k);
            }
            else if (!connectionA.isDynamic && connectionB.isDynamic)
            {
                Matrix3x3.CreateScale(connectionB.inverseMass, out linearComponent);
                Matrix3x3 angularComponentB;
                Matrix3x3.Multiply(ref rBCrossProduct, ref connectionB.inertiaTensorInverse, out angularComponentB);
                Matrix3x3.Multiply(ref angularComponentB, ref rBCrossProduct, out angularComponentB);
                Matrix3x3.Subtract(ref linearComponent, ref angularComponentB, out k);
            }
            else
            {
                throw new InvalidOperationException("Cannot constrain two kinematic bodies.");
            }
            k.M11 += softness;
            k.M22 += softness;
            k.M33 += softness;
            Matrix3x3.Invert(ref k, out massMatrix);

            Vector3.Add(ref connectionB.position, ref worldOffsetB, out error);
            Vector3.Subtract(ref error, ref connectionA.position, out error);
            Vector3.Subtract(ref error, ref worldOffsetA, out error);


            Vector3.Multiply(ref error, -errorReductionParameter, out biasVelocity);

            //Ensure that the corrective velocity doesn't exceed the max.
            Fix64 length = biasVelocity.LengthSquared();
            if (length > maxCorrectiveVelocitySquared)
            {
                Fix64 multiplier = maxCorrectiveVelocity / Fix64.Sqrt(length);
                biasVelocity.X *= multiplier;
                biasVelocity.Y *= multiplier;
                biasVelocity.Z *= multiplier;
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
            //Constraint.applyImpulse(myConnectionA, myConnectionB, ref rA, ref rB, ref accumulatedImpulse);
#if !WINDOWS
            Vector3 linear = new Vector3();
#else
            Vector3 linear;
#endif
            if (connectionA.isDynamic)
            {
                linear.X = -accumulatedImpulse.X;
                linear.Y = -accumulatedImpulse.Y;
                linear.Z = -accumulatedImpulse.Z;
                connectionA.ApplyLinearImpulse(ref linear);
                Vector3 taImpulse;
                Vector3.Cross(ref worldOffsetA, ref linear, out taImpulse);
                connectionA.ApplyAngularImpulse(ref taImpulse);
            }
            if (connectionB.isDynamic)
            {
                connectionB.ApplyLinearImpulse(ref accumulatedImpulse);
                Vector3 tbImpulse;
                Vector3.Cross(ref worldOffsetB, ref accumulatedImpulse, out tbImpulse);
                connectionB.ApplyAngularImpulse(ref tbImpulse);
            }
        }


        /// <summary>
        /// Calculates and applies corrective impulses.
        /// Called automatically by space.
        /// </summary>
        public override Fix64 SolveIteration()
        {
#if !WINDOWS
            Vector3 lambda = new Vector3();
#else
            Vector3 lambda;
#endif

            //Velocity along the length.
            Vector3 cross;
            Vector3 aVel, bVel;
            Vector3.Cross(ref connectionA.angularVelocity, ref worldOffsetA, out cross);
            Vector3.Add(ref connectionA.linearVelocity, ref cross, out aVel);
            Vector3.Cross(ref connectionB.angularVelocity, ref worldOffsetB, out cross);
            Vector3.Add(ref connectionB.linearVelocity, ref cross, out bVel);

            lambda.X = aVel.X - bVel.X + biasVelocity.X - softness * accumulatedImpulse.X;
            lambda.Y = aVel.Y - bVel.Y + biasVelocity.Y - softness * accumulatedImpulse.Y;
            lambda.Z = aVel.Z - bVel.Z + biasVelocity.Z - softness * accumulatedImpulse.Z;

            //Turn the velocity into an impulse.
            Matrix3x3.Transform(ref lambda, ref massMatrix, out lambda);

            //Accumulate the impulse
            Vector3.Add(ref accumulatedImpulse, ref lambda, out accumulatedImpulse);

            //Apply the impulse
            //Constraint.applyImpulse(myConnectionA, myConnectionB, ref rA, ref rB, ref impulse);
#if !WINDOWS
            Vector3 linear = new Vector3();
#else
            Vector3 linear;
#endif
            if (connectionA.isDynamic)
            {
                linear.X = -lambda.X;
                linear.Y = -lambda.Y;
                linear.Z = -lambda.Z;
                connectionA.ApplyLinearImpulse(ref linear);
                Vector3 taImpulse;
                Vector3.Cross(ref worldOffsetA, ref linear, out taImpulse);
                connectionA.ApplyAngularImpulse(ref taImpulse);
            }
            if (connectionB.isDynamic)
            {
                connectionB.ApplyLinearImpulse(ref lambda);
                Vector3 tbImpulse;
                Vector3.Cross(ref worldOffsetB, ref lambda, out tbImpulse);
                connectionB.ApplyAngularImpulse(ref tbImpulse);
            }

            return (Fix64.Abs(lambda.X) +
					Fix64.Abs(lambda.Y) +
					Fix64.Abs(lambda.Z));
        }
    }
}