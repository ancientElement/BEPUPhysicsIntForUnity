﻿using System;
using BEPUphysics.BroadPhaseEntries.MobileCollidables;

using BEPUutilities;
using FixMath.NET;

namespace BEPUphysics.CollisionShapes.ConvexShapes
{

    ///<summary>
    /// Triangle collision shape.
    ///</summary>
    public class TriangleShape : ConvexShape
    {
        internal Vector3 vA, vB, vC;

        ///<summary>
        /// Gets or sets the first vertex of the triangle shape.
        ///</summary>
        public Vector3 VertexA
        {
            get
            {
                return vA;
            }
            set
            {
                vA = value;
                OnShapeChanged();
            }
        }

        ///<summary>
        /// Gets or sets the second vertex of the triangle shape.
        ///</summary>
        public Vector3 VertexB
        {
            get
            {
                return vB;
            }
            set
            {
                vB = value;
                OnShapeChanged();
            }
        }

        ///<summary>
        /// Gets or sets the third vertex of the triangle shape.
        ///</summary>
        public Vector3 VertexC
        {
            get
            {
                return vC;
            }
            set
            {
                vC = value;
                OnShapeChanged();
            }
        }

        internal TriangleSidedness sidedness;
        ///<summary>
        /// Gets or sets the sidedness of the triangle.
        ///</summary>
        public TriangleSidedness Sidedness
        {
            get { return sidedness; }
            set
            {
                sidedness = value;
                OnShapeChanged();
            }
        }

        ///<summary>
        /// Constructs a triangle shape without initializing it.
        /// This is useful for systems that re-use a triangle shape repeatedly and do not care about its properties.
        ///</summary>
        public TriangleShape()
        {
            //Triangles are often used in special situations where the vertex locations are changed directly.  This constructor assists with that.
        }

        ///<summary>
        /// Constructs a triangle shape.
        /// The vertices will be recentered.  If the center is needed, use the other constructor.
        ///</summary>
        ///<param name="vA">First vertex in the triangle.</param>
        ///<param name="vB">Second vertex in the triangle.</param>
        ///<param name="vC">Third vertex in the triangle.</param>
        public TriangleShape(Vector3 vA, Vector3 vB, Vector3 vC)
        {
            //Recenter.  Convexes should contain the origin.
            Vector3 center = (vA + vB + vC) / F64.C3;
            this.vA = vA - center;
            this.vB = vB - center;
            this.vC = vC - center;
            UpdateConvexShapeInfo(ComputeDescription(this.vA, this.vB, this.vC, collisionMargin));
        }

        ///<summary>
        /// Constructs a triangle shape.
        /// The vertices will be recentered.
        ///</summary>
        ///<param name="vA">First vertex in the triangle.</param>
        ///<param name="vB">Second vertex in the triangle.</param>
        ///<param name="vC">Third vertex in the triangle.</param>
        ///<param name="center">Computed center of the triangle.</param>
        public TriangleShape(Vector3 vA, Vector3 vB, Vector3 vC, out Vector3 center)
        {
            //Recenter.  Convexes should contain the origin.
            center = (vA + vB + vC) / F64.C3;
            this.vA = vA - center;
            this.vB = vB - center;
            this.vC = vC - center;
            UpdateConvexShapeInfo(ComputeDescription(this.vA, this.vB, this.vC, collisionMargin));
        }

        ///<summary>
        /// Constructs a triangle shape from cached data.
        ///</summary>
        ///<param name="vA">First vertex in the triangle.</param>
        ///<param name="vB">Second vertex in the triangle.</param>
        ///<param name="vC">Third vertex in the triangle.</param>
        /// <param name="description">Cached information about the shape. Assumed to be correct; no extra processing or validation is performed.</param>
        public TriangleShape(Vector3 vA, Vector3 vB, Vector3 vC, ConvexShapeDescription description)
        {
            //Recenter.  Convexes should contain the origin.
            var center = (vA + vB + vC) / F64.C3;
            this.vA = vA - center;
            this.vB = vB - center;
            this.vC = vC - center;
            UpdateConvexShapeInfo(description);
        }




        /// <summary>
        /// Computes a convex shape description for a TransformableShape.
        /// </summary>
        ///<param name="vA">First local vertex in the triangle.</param>
        ///<param name="vB">Second local vertex in the triangle.</param>
        ///<param name="vC">Third local vertex in the triangle.</param>
        ///<param name="collisionMargin">Collision margin of the shape.</param>
        /// <returns>Description required to define a convex shape.</returns>
        public static ConvexShapeDescription ComputeDescription(Vector3 vA, Vector3 vB, Vector3 vC, Fix64 collisionMargin)
        {
            ConvexShapeDescription description;
            // A triangle by itself technically has no volume, but shapes try to include the collision margin in the volume when feasible (e.g. BoxShape).
            //Plus, it's convenient to have a nonzero volume for buoyancy.
            var doubleArea = Vector3.Cross(vB - vA, vC - vA).Length();
            description.EntityShapeVolume.Volume = doubleArea * collisionMargin;

            //Compute the inertia tensor.
            var v = new Matrix3x3(
                vA.X, vA.Y, vA.Z,
                vB.X, vB.Y, vB.Z,
                vC.X, vC.Y, vC.Z);
            var s = new Matrix3x3(
				F64.C2, F64.C1, F64.C1,
				F64.C1, F64.C2, F64.C1,
				F64.C1, F64.C1, F64.C2);

            Matrix3x3.MultiplyTransposed(ref v, ref s, out description.EntityShapeVolume.VolumeDistribution);
            Matrix3x3.Multiply(ref description.EntityShapeVolume.VolumeDistribution, ref v, out description.EntityShapeVolume.VolumeDistribution);
            var scaling = doubleArea / F64.C24;
            Matrix3x3.Multiply(ref description.EntityShapeVolume.VolumeDistribution, -scaling, out description.EntityShapeVolume.VolumeDistribution);

            //The square-of-sum term is ignored since the parameters should already be localized (and so would sum to zero).
            var sums = scaling * (vA.LengthSquared() + vB.LengthSquared() + vC.LengthSquared());
            description.EntityShapeVolume.VolumeDistribution.M11 += sums;
            description.EntityShapeVolume.VolumeDistribution.M22 += sums;
            description.EntityShapeVolume.VolumeDistribution.M33 += sums;

            description.MinimumRadius = collisionMargin;
            description.MaximumRadius = collisionMargin + MathHelper.Max(vA.Length(), MathHelper.Max(vB.Length(), vC.Length()));
            description.CollisionMargin = collisionMargin;
            return description;
        }

        /// <summary>
        /// Gets the bounding box of the shape given a transform.
        /// </summary>
        /// <param name="shapeTransform">Transform to use.</param>
        /// <param name="boundingBox">Bounding box of the transformed shape.</param>
        public override void GetBoundingBox(ref RigidTransform shapeTransform, out BoundingBox boundingBox)
        {
            Vector3 a, b, c;

            Matrix3x3 o;
            Matrix3x3.CreateFromQuaternion(ref shapeTransform.Orientation, out o);
            Matrix3x3.Transform(ref vA, ref o, out a);
            Matrix3x3.Transform(ref vB, ref o, out b);
            Matrix3x3.Transform(ref vC, ref o, out c);

            Vector3.Min(ref a, ref b, out boundingBox.Min);
            Vector3.Min(ref c, ref boundingBox.Min, out boundingBox.Min);

            Vector3.Max(ref a, ref b, out boundingBox.Max);
            Vector3.Max(ref c, ref boundingBox.Max, out boundingBox.Max);

            boundingBox.Min.X += shapeTransform.Position.X - collisionMargin;
            boundingBox.Min.Y += shapeTransform.Position.Y - collisionMargin;
            boundingBox.Min.Z += shapeTransform.Position.Z - collisionMargin;
            boundingBox.Max.X += shapeTransform.Position.X + collisionMargin;
            boundingBox.Max.Y += shapeTransform.Position.Y + collisionMargin;
            boundingBox.Max.Z += shapeTransform.Position.Z + collisionMargin;
        }


        ///<summary>
        /// Gets the extreme point of the shape in local space in a given direction.
        ///</summary>
        ///<param name="direction">Direction to find the extreme point in.</param>
        ///<param name="extremePoint">Extreme point on the shape.</param>
        public override void GetLocalExtremePointWithoutMargin(ref Vector3 direction, out Vector3 extremePoint)
        {
            Fix64 dotA, dotB, dotC;
            Vector3.Dot(ref direction, ref vA, out dotA);
            Vector3.Dot(ref direction, ref vB, out dotB);
            Vector3.Dot(ref direction, ref vC, out dotC);
            if (dotA > dotB && dotA > dotC)
            {
                extremePoint = vA;
            }
            else if (dotB > dotC) //vA is not the most extreme point.
            {
                extremePoint = vB;
            }
            else
            {
                extremePoint = vC;
            }
        }

        /// <summary>
        /// Computes the volume distribution of the triangle.
        /// The volume distribution can be used to compute inertia tensors when
        /// paired with mass and other tuning factors.
        /// </summary>
        ///<param name="vA">First vertex in the triangle.</param>
        ///<param name="vB">Second vertex in the triangle.</param>
        ///<param name="vC">Third vertex in the triangle.</param>
        /// <returns>Volume distribution of the shape.</returns>
        public static Matrix3x3 ComputeVolumeDistribution(Vector3 vA, Vector3 vB, Vector3 vC)
        {
            Vector3 center = (vA + vB + vC) * F64.OneThird;

            //Calculate distribution of mass.

            Fix64 massPerPoint = F64.OneThird;

            //Subtract the position from the distribution, moving into a 'body space' relative to itself.
            //        [ (j * j + z * z)  (-j * j)  (-j * z) ]
            //I = I + [ (-j * j)  (j * j + z * z)  (-j * z) ]
            //	      [ (-j * z)  (-j * z)  (j * j + j * j) ]

            Fix64 i = vA.X - center.X;
            Fix64 j = vA.Y - center.Y;
            Fix64 k = vA.Z - center.Z;
            //localInertiaTensor += new Matrix(j * j + k * k, -j * j, -j * k, 0, -j * j, j * j + k * k, -j * k, 0, -j * k, -j * k, j * j + j * j, 0, 0, 0, 0, 0); //No mass per point.
            var volumeDistribution = new Matrix3x3(massPerPoint * (j * j + k * k), massPerPoint * (-i * j), massPerPoint * (-i * k),
                                                   massPerPoint * (-i * j), massPerPoint * (i * i + k * k), massPerPoint * (-j * k),
                                                   massPerPoint * (-i * k), massPerPoint * (-j * k), massPerPoint * (i * i + j * j));

            i = vB.X - center.X;
            j = vB.Y - center.Y;
            k = vB.Z - center.Z;
            var pointContribution = new Matrix3x3(massPerPoint * (j * j + k * k), massPerPoint * (-i * j), massPerPoint * (-i * k),
                                                  massPerPoint * (-i * j), massPerPoint * (i * i + k * k), massPerPoint * (-j * k),
                                                  massPerPoint * (-i * k), massPerPoint * (-j * k), massPerPoint * (i * i + j * j));
            Matrix3x3.Add(ref volumeDistribution, ref pointContribution, out volumeDistribution);

            i = vC.X - center.X;
            j = vC.Y - center.Y;
            k = vC.Z - center.Z;
            pointContribution = new Matrix3x3(massPerPoint * (j * j + k * k), massPerPoint * (-i * j), massPerPoint * (-i * k),
                                              massPerPoint * (-i * j), massPerPoint * (i * i + k * k), massPerPoint * (-j * k),
                                              massPerPoint * (-i * k), massPerPoint * (-j * k), massPerPoint * (i * i + j * j));
            Matrix3x3.Add(ref volumeDistribution, ref pointContribution, out volumeDistribution);
            return volumeDistribution;
        }

        ///<summary>
        /// Gets the normal of the triangle shape in its local space.
        ///</summary>
        ///<returns>The local normal.</returns>
        public Vector3 GetLocalNormal()
        {
            Vector3 normal;
            Vector3 vAvB;
            Vector3 vAvC;
            Vector3.Subtract(ref vB, ref vA, out vAvB);
            Vector3.Subtract(ref vC, ref vA, out vAvC);
            Vector3.Cross(ref vAvB, ref vAvC, out normal);
            normal.Normalize();
            return normal;
        }

        /// <summary>
        /// Gets the normal of the triangle in world space.
        /// </summary>
        /// <param name="transform">World transform.</param>
        /// <returns>Normal of the triangle in world space.</returns>
        public Vector3 GetNormal(RigidTransform transform)
        {
            Vector3 normal = GetLocalNormal();
            Quaternion.Transform(ref normal, ref transform.Orientation, out normal);
            return normal;
        }

        /// <summary>
        /// Gets the intersection between the triangle and the ray.
        /// </summary>
        /// <param name="ray">Ray to test against the triangle.</param>
        /// <param name="transform">Transform to apply to the triangle shape for the test.</param>
        /// <param name="maximumLength">Maximum distance to travel in units of the direction vector's length.</param>
        /// <param name="hit">Hit data of the ray cast, if any.</param>
        /// <returns>Whether or not the ray hit the target.</returns>
        public override bool RayTest(ref Ray ray, ref RigidTransform transform, Fix64 maximumLength, out RayHit hit)
        {
            Matrix3x3 orientation;
            Matrix3x3.CreateFromQuaternion(ref transform.Orientation, out orientation);
            Ray localRay;
            Quaternion conjugate;
            Quaternion.Conjugate(ref transform.Orientation, out conjugate);
            Quaternion.Transform(ref ray.Direction, ref conjugate, out localRay.Direction);
            Vector3.Subtract(ref ray.Position, ref transform.Position, out localRay.Position);
            Quaternion.Transform(ref localRay.Position, ref conjugate, out localRay.Position);

            bool toReturn = Toolbox.FindRayTriangleIntersection(ref localRay, maximumLength, sidedness, ref vA, ref vB, ref vC, out hit);
            //Move the hit back into world space.
            Vector3.Multiply(ref ray.Direction, hit.T, out hit.Location);
            Vector3.Add(ref ray.Position, ref hit.Location, out hit.Location);
            Quaternion.Transform(ref hit.Normal, ref transform.Orientation, out hit.Normal);
            return toReturn;
        }

        /// <summary>
        /// Returns a <see cref="T:System.String"/> that represents the current <see cref="T:System.Object"/>.
        /// </summary>
        /// <returns>
        /// A <see cref="T:System.String"/> that represents the current <see cref="T:System.Object"/>.
        /// </returns>
        /// <filterpriority>2</filterpriority>
        public override string ToString()
        {
            return vA + ", " + vB + ", " + vC;
        }

        /// <summary>
        /// Retrieves an instance of an EntityCollidable that uses this EntityShape.  Mainly used by compound bodies.
        /// </summary>
        /// <returns>EntityCollidable that uses this shape.</returns>
        public override EntityCollidable GetCollidableInstance()
        {
            return new ConvexCollidable<TriangleShape>(this);
        }

    }

}
