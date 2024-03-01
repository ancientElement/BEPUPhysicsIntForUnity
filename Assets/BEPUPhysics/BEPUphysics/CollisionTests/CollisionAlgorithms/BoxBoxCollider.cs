﻿using System;
using System.Runtime.InteropServices;
using BEPUutilities.DataStructures;
using BEPUutilities;
 
using BEPUphysics.CollisionShapes.ConvexShapes;
using FixMath.NET;

namespace BEPUphysics.CollisionTests.CollisionAlgorithms
{
    /// <summary>
    /// Stores basic data used by some collision systems.
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct BoxContactData : IEquatable<BoxContactData>
    {
        /// <summary>
        /// Position of the candidate contact.
        /// </summary>
        public Vector3 Position;

        /// <summary>
        /// Depth of the candidate contact.
        /// </summary>
        public Fix64 Depth;

        /// <summary>
        /// Id of the candidate contact.
        /// </summary>
        public int Id;

        #region IEquatable<BoxContactData> Members

        /// <summary>
        /// Returns true if the other data has the same id.
        /// </summary>
        /// <param name="other">Data to compare.</param>
        /// <returns>True if the other data has the same id, false otherwise.</returns>
        public bool Equals(BoxContactData other)
        {
            return Id == other.Id;
        }

        #endregion
    }

    /// <summary>
    /// Basic storage structure for contact data.
    /// Designed for performance critical code and pointer access.
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct BoxContactDataCache
    {
        public BoxContactData D1;
        public BoxContactData D2;
        public BoxContactData D3;
        public BoxContactData D4;

        public BoxContactData D5;
        public BoxContactData D6;
        public BoxContactData D7;
        public BoxContactData D8;


        /// <summary>
        /// Number of elements in the cache.
        /// </summary>
        public byte Count;

#if ALLOWUNSAFE
        /// <summary>
        /// Removes an item at the given index.
        /// </summary>
        /// <param name="index">Index to remove.</param>
        public unsafe void RemoveAt(int index)
        {
            BoxContactDataCache copy = this;
            BoxContactData* pointer = &copy.D1;
            pointer[index] = pointer[Count - 1];
            this = copy;
            Count--;
        }
#endif
    }


    /// <summary>
    /// Contains helper methods for testing collisions between boxes.
    /// </summary>
    public static class BoxBoxCollider
    {
        /// <summary>
        /// Determines if the two boxes are colliding.
        /// </summary>
        /// <param name="a">First box to collide.</param>
        /// <param name="b">Second box to collide.</param>
        /// <param name="transformA">Transform to apply to shape a.</param>
        /// <param name="transformB">Transform to apply to shape b.</param>
        /// <returns>Whether or not the boxes collide.</returns>
        public static bool AreBoxesColliding(BoxShape a, BoxShape b, ref RigidTransform transformA, ref RigidTransform transformB)
        {
            Fix64 aX = a.HalfWidth;
            Fix64 aY = a.HalfHeight;
            Fix64 aZ = a.HalfLength;

            Fix64 bX = b.HalfWidth;
            Fix64 bY = b.HalfHeight;
            Fix64 bZ = b.HalfLength;

            //Relative rotation from A to B.
            Matrix3x3 bR;

            Matrix3x3 aO;
            Matrix3x3.CreateFromQuaternion(ref transformA.Orientation, out aO);
            Matrix3x3 bO;
            Matrix3x3.CreateFromQuaternion(ref transformB.Orientation, out bO);

            //Relative translation rotated into A's configuration space.
            Vector3 t;
            Vector3.Subtract(ref transformB.Position, ref transformA.Position, out t);

            bR.M11 = aO.M11 * bO.M11 + aO.M12 * bO.M12 + aO.M13 * bO.M13;
            bR.M12 = aO.M11 * bO.M21 + aO.M12 * bO.M22 + aO.M13 * bO.M23;
            bR.M13 = aO.M11 * bO.M31 + aO.M12 * bO.M32 + aO.M13 * bO.M33;
            Matrix3x3 absBR;
            //Epsilons are added to deal with near-parallel edges.
            absBR.M11 = Fix64.Abs(bR.M11) + Toolbox.Epsilon;
            absBR.M12 = Fix64.Abs(bR.M12) + Toolbox.Epsilon;
            absBR.M13 = Fix64.Abs(bR.M13) + Toolbox.Epsilon;
            Fix64 tX = t.X;
            t.X = t.X * aO.M11 + t.Y * aO.M12 + t.Z * aO.M13;

            //Test the axes defines by entity A's rotation matrix.
            //A.X
            Fix64 rb = bX * absBR.M11 + bY * absBR.M12 + bZ * absBR.M13;
            if (Fix64.Abs(t.X) > aX + rb)
                return false;
            bR.M21 = aO.M21 * bO.M11 + aO.M22 * bO.M12 + aO.M23 * bO.M13;
            bR.M22 = aO.M21 * bO.M21 + aO.M22 * bO.M22 + aO.M23 * bO.M23;
            bR.M23 = aO.M21 * bO.M31 + aO.M22 * bO.M32 + aO.M23 * bO.M33;
            absBR.M21 = Fix64.Abs(bR.M21) + Toolbox.Epsilon;
            absBR.M22 = Fix64.Abs(bR.M22) + Toolbox.Epsilon;
            absBR.M23 = Fix64.Abs(bR.M23) + Toolbox.Epsilon;
            Fix64 tY = t.Y;
            t.Y = tX * aO.M21 + t.Y * aO.M22 + t.Z * aO.M23;

            //A.Y
            rb = bX * absBR.M21 + bY * absBR.M22 + bZ * absBR.M23;
            if (Fix64.Abs(t.Y) > aY + rb)
                return false;

            bR.M31 = aO.M31 * bO.M11 + aO.M32 * bO.M12 + aO.M33 * bO.M13;
            bR.M32 = aO.M31 * bO.M21 + aO.M32 * bO.M22 + aO.M33 * bO.M23;
            bR.M33 = aO.M31 * bO.M31 + aO.M32 * bO.M32 + aO.M33 * bO.M33;
            absBR.M31 = Fix64.Abs(bR.M31) + Toolbox.Epsilon;
            absBR.M32 = Fix64.Abs(bR.M32) + Toolbox.Epsilon;
            absBR.M33 = Fix64.Abs(bR.M33) + Toolbox.Epsilon;
            t.Z = tX * aO.M31 + tY * aO.M32 + t.Z * aO.M33;

            //A.Z
            rb = bX * absBR.M31 + bY * absBR.M32 + bZ * absBR.M33;
            if (Fix64.Abs(t.Z) > aZ + rb)
                return false;

            //Test the axes defines by entity B's rotation matrix.
            //B.X
            Fix64 ra = aX * absBR.M11 + aY * absBR.M21 + aZ * absBR.M31;
            if (Fix64.Abs(t.X * bR.M11 + t.Y * bR.M21 + t.Z * bR.M31) > ra + bX)
                return false;

            //B.Y
            ra = aX * absBR.M12 + aY * absBR.M22 + aZ * absBR.M32;
            if (Fix64.Abs(t.X * bR.M12 + t.Y * bR.M22 + t.Z * bR.M32) > ra + bY)
                return false;

            //B.Z
            ra = aX * absBR.M13 + aY * absBR.M23 + aZ * absBR.M33;
            if (Fix64.Abs(t.X * bR.M13 + t.Y * bR.M23 + t.Z * bR.M33) > ra + bZ)
                return false;

            //Now for the edge-edge cases.
            //A.X x B.X
            ra = aY * absBR.M31 + aZ * absBR.M21;
            rb = bY * absBR.M13 + bZ * absBR.M12;
            if (Fix64.Abs(t.Z * bR.M21 - t.Y * bR.M31) > ra + rb)
                return false;

            //A.X x B.Y
            ra = aY * absBR.M32 + aZ * absBR.M22;
            rb = bX * absBR.M13 + bZ * absBR.M11;
            if (Fix64.Abs(t.Z * bR.M22 - t.Y * bR.M32) > ra + rb)
                return false;

            //A.X x B.Z
            ra = aY * absBR.M33 + aZ * absBR.M23;
            rb = bX * absBR.M12 + bY * absBR.M11;
            if (Fix64.Abs(t.Z * bR.M23 - t.Y * bR.M33) > ra + rb)
                return false;


            //A.Y x B.X
            ra = aX * absBR.M31 + aZ * absBR.M11;
            rb = bY * absBR.M23 + bZ * absBR.M22;
            if (Fix64.Abs(t.X * bR.M31 - t.Z * bR.M11) > ra + rb)
                return false;

            //A.Y x B.Y
            ra = aX * absBR.M32 + aZ * absBR.M12;
            rb = bX * absBR.M23 + bZ * absBR.M21;
            if (Fix64.Abs(t.X * bR.M32 - t.Z * bR.M12) > ra + rb)
                return false;

            //A.Y x B.Z
            ra = aX * absBR.M33 + aZ * absBR.M13;
            rb = bX * absBR.M22 + bY * absBR.M21;
            if (Fix64.Abs(t.X * bR.M33 - t.Z * bR.M13) > ra + rb)
                return false;

            //A.Z x B.X
            ra = aX * absBR.M21 + aY * absBR.M11;
            rb = bY * absBR.M33 + bZ * absBR.M32;
            if (Fix64.Abs(t.Y * bR.M11 - t.X * bR.M21) > ra + rb)
                return false;

            //A.Z x B.Y
            ra = aX * absBR.M22 + aY * absBR.M12;
            rb = bX * absBR.M33 + bZ * absBR.M31;
            if (Fix64.Abs(t.Y * bR.M12 - t.X * bR.M22) > ra + rb)
                return false;

            //A.Z x B.Z
            ra = aX * absBR.M23 + aY * absBR.M13;
            rb = bX * absBR.M32 + bY * absBR.M31;
            if (Fix64.Abs(t.Y * bR.M13 - t.X * bR.M23) > ra + rb)
                return false;

            return true;
        }

        /// <summary>
        /// Determines if the two boxes are colliding.
        /// </summary>
        /// <param name="a">First box to collide.</param>
        /// <param name="b">Second box to collide.</param>
        /// <param name="separationDistance">Distance of separation.</param>
        /// <param name="separatingAxis">Axis of separation.</param>
        /// <param name="transformA">Transform to apply to shape A.</param>
        /// <param name="transformB">Transform to apply to shape B.</param>
        /// <returns>Whether or not the boxes collide.</returns>
        public static bool AreBoxesColliding(BoxShape a, BoxShape b, ref RigidTransform transformA, ref RigidTransform transformB, out Fix64 separationDistance, out Vector3 separatingAxis)
        {
            Fix64 aX = a.HalfWidth;
            Fix64 aY = a.HalfHeight;
            Fix64 aZ = a.HalfLength;

            Fix64 bX = b.HalfWidth;
            Fix64 bY = b.HalfHeight;
            Fix64 bZ = b.HalfLength;

            //Relative rotation from A to B.
            Matrix3x3 bR;

            Matrix3x3 aO;
            Matrix3x3.CreateFromQuaternion(ref transformA.Orientation, out aO);
            Matrix3x3 bO;
            Matrix3x3.CreateFromQuaternion(ref transformB.Orientation, out bO);

            //Relative translation rotated into A's configuration space.
            Vector3 t;
            Vector3.Subtract(ref transformB.Position, ref transformA.Position, out t);

            #region A Face Normals

            bR.M11 = aO.M11 * bO.M11 + aO.M12 * bO.M12 + aO.M13 * bO.M13;
            bR.M12 = aO.M11 * bO.M21 + aO.M12 * bO.M22 + aO.M13 * bO.M23;
            bR.M13 = aO.M11 * bO.M31 + aO.M12 * bO.M32 + aO.M13 * bO.M33;
            Matrix3x3 absBR;
            //Epsilons are added to deal with near-parallel edges.
            absBR.M11 = Fix64.Abs(bR.M11) + Toolbox.Epsilon;
            absBR.M12 = Fix64.Abs(bR.M12) + Toolbox.Epsilon;
            absBR.M13 = Fix64.Abs(bR.M13) + Toolbox.Epsilon;
            Fix64 tX = t.X;
            t.X = t.X * aO.M11 + t.Y * aO.M12 + t.Z * aO.M13;

            //Test the axes defines by entity A's rotation matrix.
            //A.X
            Fix64 rarb = aX + bX * absBR.M11 + bY * absBR.M12 + bZ * absBR.M13;
            if (t.X > rarb)
            {
                separationDistance = t.X - rarb;
                separatingAxis = new Vector3(aO.M11, aO.M12, aO.M13);
                return false;
            }
            if (t.X < -rarb)
            {
                separationDistance = -t.X - rarb;
                separatingAxis = new Vector3(-aO.M11, -aO.M12, -aO.M13);
                return false;
            }


            bR.M21 = aO.M21 * bO.M11 + aO.M22 * bO.M12 + aO.M23 * bO.M13;
            bR.M22 = aO.M21 * bO.M21 + aO.M22 * bO.M22 + aO.M23 * bO.M23;
            bR.M23 = aO.M21 * bO.M31 + aO.M22 * bO.M32 + aO.M23 * bO.M33;
            absBR.M21 = Fix64.Abs(bR.M21) + Toolbox.Epsilon;
            absBR.M22 = Fix64.Abs(bR.M22) + Toolbox.Epsilon;
            absBR.M23 = Fix64.Abs(bR.M23) + Toolbox.Epsilon;
            Fix64 tY = t.Y;
            t.Y = tX * aO.M21 + t.Y * aO.M22 + t.Z * aO.M23;

            //A.Y
            rarb = aY + bX * absBR.M21 + bY * absBR.M22 + bZ * absBR.M23;
            if (t.Y > rarb)
            {
                separationDistance = t.Y - rarb;
                separatingAxis = new Vector3(aO.M21, aO.M22, aO.M23);
                return false;
            }
            if (t.Y < -rarb)
            {
                separationDistance = -t.Y - rarb;
                separatingAxis = new Vector3(-aO.M21, -aO.M22, -aO.M23);
                return false;
            }

            bR.M31 = aO.M31 * bO.M11 + aO.M32 * bO.M12 + aO.M33 * bO.M13;
            bR.M32 = aO.M31 * bO.M21 + aO.M32 * bO.M22 + aO.M33 * bO.M23;
            bR.M33 = aO.M31 * bO.M31 + aO.M32 * bO.M32 + aO.M33 * bO.M33;
            absBR.M31 = Fix64.Abs(bR.M31) + Toolbox.Epsilon;
            absBR.M32 = Fix64.Abs(bR.M32) + Toolbox.Epsilon;
            absBR.M33 = Fix64.Abs(bR.M33) + Toolbox.Epsilon;
            t.Z = tX * aO.M31 + tY * aO.M32 + t.Z * aO.M33;

            //A.Z
            rarb = aZ + bX * absBR.M31 + bY * absBR.M32 + bZ * absBR.M33;
            if (t.Z > rarb)
            {
                separationDistance = t.Z - rarb;
                separatingAxis = new Vector3(aO.M31, aO.M32, aO.M33);
                return false;
            }
            if (t.Z < -rarb)
            {
                separationDistance = -t.Z - rarb;
                separatingAxis = new Vector3(-aO.M31, -aO.M32, -aO.M33);
                return false;
            }

            #endregion

            #region B Face Normals

            //Test the axes defines by entity B's rotation matrix.
            //B.X
            rarb = bX + aX * absBR.M11 + aY * absBR.M21 + aZ * absBR.M31;
            Fix64 tl = t.X * bR.M11 + t.Y * bR.M21 + t.Z * bR.M31;
            if (tl > rarb)
            {
                separationDistance = tl - rarb;
                separatingAxis = new Vector3(bO.M11, bO.M12, bO.M13);
                return false;
            }
            if (tl < -rarb)
            {
                separationDistance = -tl - rarb;
                separatingAxis = new Vector3(-bO.M11, -bO.M12, -bO.M13);
                return false;
            }

            //B.Y
            rarb = bY + aX * absBR.M12 + aY * absBR.M22 + aZ * absBR.M32;
            tl = t.X * bR.M12 + t.Y * bR.M22 + t.Z * bR.M32;
            if (tl > rarb)
            {
                separationDistance = tl - rarb;
                separatingAxis = new Vector3(bO.M21, bO.M22, bO.M23);
                return false;
            }
            if (tl < -rarb)
            {
                separationDistance = -tl - rarb;
                separatingAxis = new Vector3(-bO.M21, -bO.M22, -bO.M23);
                return false;
            }


            //B.Z
            rarb = bZ + aX * absBR.M13 + aY * absBR.M23 + aZ * absBR.M33;
            tl = t.X * bR.M13 + t.Y * bR.M23 + t.Z * bR.M33;
            if (tl > rarb)
            {
                separationDistance = tl - rarb;
                separatingAxis = new Vector3(bO.M31, bO.M32, bO.M33);
                return false;
            }
            if (tl < -rarb)
            {
                separationDistance = -tl - rarb;
                separatingAxis = new Vector3(-bO.M31, -bO.M32, -bO.M33);
                return false;
            }

            #endregion

            #region A.X x B.()

            //Now for the edge-edge cases.
            //A.X x B.X
            rarb = aY * absBR.M31 + aZ * absBR.M21 +
                   bY * absBR.M13 + bZ * absBR.M12;
            tl = t.Z * bR.M21 - t.Y * bR.M31;
            if (tl > rarb)
            {
                separationDistance = tl - rarb;
                separatingAxis = new Vector3(aO.M12 * bO.M13 - aO.M13 * bO.M12,
                                             aO.M13 * bO.M11 - aO.M11 * bO.M13,
                                             aO.M11 * bO.M12 - aO.M12 * bO.M11);
                return false;
            }
            if (tl < -rarb)
            {
                separationDistance = -tl - rarb;
                separatingAxis = new Vector3(bO.M12 * aO.M13 - bO.M13 * aO.M12,
                                             bO.M13 * aO.M11 - bO.M11 * aO.M13,
                                             bO.M11 * aO.M12 - bO.M12 * aO.M11);
                return false;
            }

            //A.X x B.Y
            rarb = aY * absBR.M32 + aZ * absBR.M22 +
                   bX * absBR.M13 + bZ * absBR.M11;
            tl = t.Z * bR.M22 - t.Y * bR.M32;
            if (tl > rarb)
            {
                separationDistance = tl - rarb;
                separatingAxis = new Vector3(aO.M12 * bO.M23 - aO.M13 * bO.M22,
                                             aO.M13 * bO.M21 - aO.M11 * bO.M23,
                                             aO.M11 * bO.M22 - aO.M12 * bO.M21);
                return false;
            }
            if (tl < -rarb)
            {
                separationDistance = -tl - rarb;
                separatingAxis = new Vector3(bO.M22 * aO.M13 - bO.M23 * aO.M12,
                                             bO.M23 * aO.M11 - bO.M21 * aO.M13,
                                             bO.M21 * aO.M12 - bO.M22 * aO.M11);
                return false;
            }

            //A.X x B.Z
            rarb = aY * absBR.M33 + aZ * absBR.M23 +
                   bX * absBR.M12 + bY * absBR.M11;
            tl = t.Z * bR.M23 - t.Y * bR.M33;
            if (tl > rarb)
            {
                separationDistance = tl - rarb;
                separatingAxis = new Vector3(aO.M12 * bO.M33 - aO.M13 * bO.M32,
                                             aO.M13 * bO.M31 - aO.M11 * bO.M33,
                                             aO.M11 * bO.M32 - aO.M12 * bO.M31);
                return false;
            }
            if (tl < -rarb)
            {
                separationDistance = -tl - rarb;
                separatingAxis = new Vector3(bO.M32 * aO.M13 - bO.M33 * aO.M12,
                                             bO.M33 * aO.M11 - bO.M31 * aO.M13,
                                             bO.M31 * aO.M12 - bO.M32 * aO.M11);
                return false;
            }

            #endregion

            #region A.Y x B.()

            //A.Y x B.X
            rarb = aX * absBR.M31 + aZ * absBR.M11 +
                   bY * absBR.M23 + bZ * absBR.M22;
            tl = t.X * bR.M31 - t.Z * bR.M11;
            if (tl > rarb)
            {
                separationDistance = tl - rarb;
                separatingAxis = new Vector3(aO.M22 * bO.M13 - aO.M23 * bO.M12,
                                             aO.M23 * bO.M11 - aO.M21 * bO.M13,
                                             aO.M21 * bO.M12 - aO.M22 * bO.M11);
                return false;
            }
            if (tl < -rarb)
            {
                separationDistance = -tl - rarb;
                separatingAxis = new Vector3(bO.M12 * aO.M23 - bO.M13 * aO.M22,
                                             bO.M13 * aO.M21 - bO.M11 * aO.M23,
                                             bO.M11 * aO.M22 - bO.M12 * aO.M21);
                return false;
            }

            //A.Y x B.Y
            rarb = aX * absBR.M32 + aZ * absBR.M12 +
                   bX * absBR.M23 + bZ * absBR.M21;
            tl = t.X * bR.M32 - t.Z * bR.M12;
            if (tl > rarb)
            {
                separationDistance = tl - rarb;
                separatingAxis = new Vector3(aO.M22 * bO.M23 - aO.M23 * bO.M22,
                                             aO.M23 * bO.M21 - aO.M21 * bO.M23,
                                             aO.M21 * bO.M22 - aO.M22 * bO.M21);
                return false;
            }
            if (tl < -rarb)
            {
                separationDistance = -tl - rarb;
                separatingAxis = new Vector3(bO.M22 * aO.M23 - bO.M23 * aO.M22,
                                             bO.M23 * aO.M21 - bO.M21 * aO.M23,
                                             bO.M21 * aO.M22 - bO.M22 * aO.M21);
                return false;
            }

            //A.Y x B.Z
            rarb = aX * absBR.M33 + aZ * absBR.M13 +
                   bX * absBR.M22 + bY * absBR.M21;
            tl = t.X * bR.M33 - t.Z * bR.M13;
            if (tl > rarb)
            {
                separationDistance = tl - rarb;
                separatingAxis = new Vector3(aO.M22 * bO.M33 - aO.M23 * bO.M32,
                                             aO.M23 * bO.M31 - aO.M21 * bO.M33,
                                             aO.M21 * bO.M32 - aO.M22 * bO.M31);
                return false;
            }
            if (tl < -rarb)
            {
                separationDistance = -tl - rarb;
                separatingAxis = new Vector3(bO.M32 * aO.M23 - bO.M33 * aO.M22,
                                             bO.M33 * aO.M21 - bO.M31 * aO.M23,
                                             bO.M31 * aO.M22 - bO.M32 * aO.M21);
                return false;
            }

            #endregion

            #region A.Z x B.()

            //A.Z x B.X
            rarb = aX * absBR.M21 + aY * absBR.M11 +
                   bY * absBR.M33 + bZ * absBR.M32;
            tl = t.Y * bR.M11 - t.X * bR.M21;
            if (tl > rarb)
            {
                separationDistance = tl - rarb;
                separatingAxis = new Vector3(aO.M32 * bO.M13 - aO.M33 * bO.M12,
                                             aO.M33 * bO.M11 - aO.M31 * bO.M13,
                                             aO.M31 * bO.M12 - aO.M32 * bO.M11);
                return false;
            }
            if (tl < -rarb)
            {
                separationDistance = -tl - rarb;
                separatingAxis = new Vector3(bO.M12 * aO.M33 - bO.M13 * aO.M32,
                                             bO.M13 * aO.M31 - bO.M11 * aO.M33,
                                             bO.M11 * aO.M32 - bO.M12 * aO.M31);
                return false;
            }

            //A.Z x B.Y
            rarb = aX * absBR.M22 + aY * absBR.M12 +
                   bX * absBR.M33 + bZ * absBR.M31;
            tl = t.Y * bR.M12 - t.X * bR.M22;
            if (tl > rarb)
            {
                separationDistance = tl - rarb;
                separatingAxis = new Vector3(aO.M32 * bO.M23 - aO.M33 * bO.M22,
                                             aO.M33 * bO.M21 - aO.M31 * bO.M23,
                                             aO.M31 * bO.M22 - aO.M32 * bO.M21);
                return false;
            }
            if (tl < -rarb)
            {
                separationDistance = -tl - rarb;
                separatingAxis = new Vector3(bO.M22 * aO.M33 - bO.M23 * aO.M32,
                                             bO.M23 * aO.M31 - bO.M21 * aO.M33,
                                             bO.M21 * aO.M32 - bO.M22 * aO.M31);
                return false;
            }

            //A.Z x B.Z
            rarb = aX * absBR.M23 + aY * absBR.M13 +
                   bX * absBR.M32 + bY * absBR.M31;
            tl = t.Y * bR.M13 - t.X * bR.M23;
            if (tl > rarb)
            {
                separationDistance = tl - rarb;
                separatingAxis = new Vector3(aO.M32 * bO.M33 - aO.M33 * bO.M32,
                                             aO.M33 * bO.M31 - aO.M31 * bO.M33,
                                             aO.M31 * bO.M32 - aO.M32 * bO.M31);
                return false;
            }
            if (tl < -rarb)
            {
                separationDistance = -tl - rarb;
                separatingAxis = new Vector3(bO.M32 * aO.M33 - bO.M33 * aO.M32,
                                             bO.M33 * aO.M31 - bO.M31 * aO.M33,
                                             bO.M31 * aO.M32 - bO.M32 * aO.M31);
                return false;
            }

            #endregion

            separationDistance = F64.C0;
            separatingAxis = Vector3.Zero;
            return true;
        }

        /// <summary>
        /// Determines if the two boxes are colliding, including penetration depth data.
        /// </summary>
        /// <param name="a">First box to collide.</param>
        /// <param name="b">Second box to collide.</param>
        /// <param name="distance">Distance of separation or penetration.</param>
        /// <param name="axis">Axis of separation or penetration.</param>
        /// <param name="transformA">Transform to apply to shape A.</param>
        /// <param name="transformB">Transform to apply to shape B.</param>
        /// <returns>Whether or not the boxes collide.</returns>
        public static bool AreBoxesCollidingWithPenetration(BoxShape a, BoxShape b, ref RigidTransform transformA, ref RigidTransform transformB, out Fix64 distance, out Vector3 axis)
        {
            Fix64 aX = a.HalfWidth;
            Fix64 aY = a.HalfHeight;
            Fix64 aZ = a.HalfLength;

            Fix64 bX = b.HalfWidth;
            Fix64 bY = b.HalfHeight;
            Fix64 bZ = b.HalfLength;

            //Relative rotation from A to B.
            Matrix3x3 bR;

            Matrix3x3 aO;
            Matrix3x3.CreateFromQuaternion(ref transformA.Orientation, out aO);
            Matrix3x3 bO;
            Matrix3x3.CreateFromQuaternion(ref transformB.Orientation, out bO);

            //Relative translation rotated into A's configuration space.
            Vector3 t;
            Vector3.Subtract(ref transformB.Position, ref transformA.Position, out t);

            Fix64 tempDistance;
            Fix64 minimumDistance = -Fix64.MaxValue;
            var minimumAxis = new Vector3();

            #region A Face Normals

            bR.M11 = aO.M11 * bO.M11 + aO.M12 * bO.M12 + aO.M13 * bO.M13;
            bR.M12 = aO.M11 * bO.M21 + aO.M12 * bO.M22 + aO.M13 * bO.M23;
            bR.M13 = aO.M11 * bO.M31 + aO.M12 * bO.M32 + aO.M13 * bO.M33;
            Matrix3x3 absBR;
            //Epsilons are added to deal with near-parallel edges.
            absBR.M11 = Fix64.Abs(bR.M11) + Toolbox.Epsilon;
            absBR.M12 = Fix64.Abs(bR.M12) + Toolbox.Epsilon;
            absBR.M13 = Fix64.Abs(bR.M13) + Toolbox.Epsilon;
            Fix64 tX = t.X;
            t.X = t.X * aO.M11 + t.Y * aO.M12 + t.Z * aO.M13;

            //Test the axes defines by entity A's rotation matrix.
            //A.X
            Fix64 rarb = aX + bX * absBR.M11 + bY * absBR.M12 + bZ * absBR.M13;
            if (t.X > rarb)
            {
                distance = t.X - rarb;
                axis = new Vector3(aO.M11, aO.M12, aO.M13);
                return false;
            }
            if (t.X < -rarb)
            {
                distance = -t.X - rarb;
                axis = new Vector3(-aO.M11, -aO.M12, -aO.M13);
                return false;
            }
            //Inside
            if (t.X > F64.C0)
            {
                tempDistance = t.X - rarb;
                if (tempDistance > minimumDistance)
                {
                    minimumDistance = tempDistance;
                    minimumAxis = new Vector3(aO.M11, aO.M12, aO.M13);
                }
            }
            else
            {
                tempDistance = -t.X - rarb;
                if (tempDistance > minimumDistance)
                {
                    minimumDistance = tempDistance;
                    minimumAxis = new Vector3(-aO.M11, -aO.M12, -aO.M13);
                }
            }


            bR.M21 = aO.M21 * bO.M11 + aO.M22 * bO.M12 + aO.M23 * bO.M13;
            bR.M22 = aO.M21 * bO.M21 + aO.M22 * bO.M22 + aO.M23 * bO.M23;
            bR.M23 = aO.M21 * bO.M31 + aO.M22 * bO.M32 + aO.M23 * bO.M33;
            absBR.M21 = Fix64.Abs(bR.M21) + Toolbox.Epsilon;
            absBR.M22 = Fix64.Abs(bR.M22) + Toolbox.Epsilon;
            absBR.M23 = Fix64.Abs(bR.M23) + Toolbox.Epsilon;
            Fix64 tY = t.Y;
            t.Y = tX * aO.M21 + t.Y * aO.M22 + t.Z * aO.M23;

            //A.Y
            rarb = aY + bX * absBR.M21 + bY * absBR.M22 + bZ * absBR.M23;
            if (t.Y > rarb)
            {
                distance = t.Y - rarb;
                axis = new Vector3(aO.M21, aO.M22, aO.M23);
                return false;
            }
            if (t.Y < -rarb)
            {
                distance = -t.Y - rarb;
                axis = new Vector3(-aO.M21, -aO.M22, -aO.M23);
                return false;
            }
            //Inside
            if (t.Y > F64.C0)
            {
                tempDistance = t.Y - rarb;
                if (tempDistance > minimumDistance)
                {
                    minimumDistance = tempDistance;
                    minimumAxis = new Vector3(aO.M21, aO.M22, aO.M23);
                }
            }
            else
            {
                tempDistance = -t.Y - rarb;
                if (tempDistance > minimumDistance)
                {
                    minimumDistance = tempDistance;
                    minimumAxis = new Vector3(-aO.M21, -aO.M22, -aO.M23);
                }
            }

            bR.M31 = aO.M31 * bO.M11 + aO.M32 * bO.M12 + aO.M33 * bO.M13;
            bR.M32 = aO.M31 * bO.M21 + aO.M32 * bO.M22 + aO.M33 * bO.M23;
            bR.M33 = aO.M31 * bO.M31 + aO.M32 * bO.M32 + aO.M33 * bO.M33;
            absBR.M31 = Fix64.Abs(bR.M31) + Toolbox.Epsilon;
            absBR.M32 = Fix64.Abs(bR.M32) + Toolbox.Epsilon;
            absBR.M33 = Fix64.Abs(bR.M33) + Toolbox.Epsilon;
            t.Z = tX * aO.M31 + tY * aO.M32 + t.Z * aO.M33;

            //A.Z
            rarb = aZ + bX * absBR.M31 + bY * absBR.M32 + bZ * absBR.M33;
            if (t.Z > rarb)
            {
                distance = t.Z - rarb;
                axis = new Vector3(aO.M31, aO.M32, aO.M33);
                return false;
            }
            if (t.Z < -rarb)
            {
                distance = -t.Z - rarb;
                axis = new Vector3(-aO.M31, -aO.M32, -aO.M33);
                return false;
            }
            //Inside
            if (t.Z > F64.C0)
            {
                tempDistance = t.Z - rarb;
                if (tempDistance > minimumDistance)
                {
                    minimumDistance = tempDistance;
                    minimumAxis = new Vector3(aO.M31, aO.M32, aO.M33);
                }
            }
            else
            {
                tempDistance = -t.Z - rarb;
                if (tempDistance > minimumDistance)
                {
                    minimumDistance = tempDistance;
                    minimumAxis = new Vector3(-aO.M31, -aO.M32, -aO.M33);
                }
            }

            #endregion

            #region B Face Normals

            //Test the axes defines by entity B's rotation matrix.
            //B.X
            rarb = bX + aX * absBR.M11 + aY * absBR.M21 + aZ * absBR.M31;
            Fix64 tl = t.X * bR.M11 + t.Y * bR.M21 + t.Z * bR.M31;
            if (tl > rarb)
            {
                distance = tl - rarb;
                axis = new Vector3(bO.M11, bO.M12, bO.M13);
                return false;
            }
            if (tl < -rarb)
            {
                distance = -tl - rarb;
                axis = new Vector3(-bO.M11, -bO.M12, -bO.M13);
                return false;
            }
            //Inside
            if (tl > F64.C0)
            {
                tempDistance = tl - rarb;
                if (tempDistance > minimumDistance)
                {
                    minimumDistance = tempDistance;
                    minimumAxis = new Vector3(bO.M11, bO.M12, bO.M13);
                }
            }
            else
            {
                tempDistance = -tl - rarb;
                if (tempDistance > minimumDistance)
                {
                    minimumDistance = tempDistance;
                    minimumAxis = new Vector3(-bO.M11, -bO.M12, -bO.M13);
                }
            }

            //B.Y
            rarb = bY + aX * absBR.M12 + aY * absBR.M22 + aZ * absBR.M32;
            tl = t.X * bR.M12 + t.Y * bR.M22 + t.Z * bR.M32;
            if (tl > rarb)
            {
                distance = tl - rarb;
                axis = new Vector3(bO.M21, bO.M22, bO.M23);
                return false;
            }
            if (tl < -rarb)
            {
                distance = -tl - rarb;
                axis = new Vector3(-bO.M21, -bO.M22, -bO.M23);
                return false;
            }
            //Inside
            if (tl > F64.C0)
            {
                tempDistance = tl - rarb;
                if (tempDistance > minimumDistance)
                {
                    minimumDistance = tempDistance;
                    minimumAxis = new Vector3(bO.M21, bO.M22, bO.M23);
                }
            }
            else
            {
                tempDistance = -tl - rarb;
                if (tempDistance > minimumDistance)
                {
                    minimumDistance = tempDistance;
                    minimumAxis = new Vector3(-bO.M21, -bO.M22, -bO.M23);
                }
            }

            //B.Z
            rarb = bZ + aX * absBR.M13 + aY * absBR.M23 + aZ * absBR.M33;
            tl = t.X * bR.M13 + t.Y * bR.M23 + t.Z * bR.M33;
            if (tl > rarb)
            {
                distance = tl - rarb;
                axis = new Vector3(bO.M31, bO.M32, bO.M33);
                return false;
            }
            if (tl < -rarb)
            {
                distance = -tl - rarb;
                axis = new Vector3(-bO.M31, -bO.M32, -bO.M33);
                return false;
            }
            //Inside
            if (tl > F64.C0)
            {
                tempDistance = tl - rarb;
                if (tempDistance > minimumDistance)
                {
                    minimumDistance = tempDistance;
                    minimumAxis = new Vector3(bO.M31, bO.M32, bO.M33);
                }
            }
            else
            {
                tempDistance = -tl - rarb;
                if (tempDistance > minimumDistance)
                {
                    minimumDistance = tempDistance;
                    minimumAxis = new Vector3(-bO.M31, -bO.M32, -bO.M33);
                }
            }

            #endregion

            Fix64 axisLengthInverse;
            Vector3 tempAxis;

            #region A.X x B.()

            //Now for the edge-edge cases.
            //A.X x B.X
            rarb = aY * absBR.M31 + aZ * absBR.M21 +
                   bY * absBR.M13 + bZ * absBR.M12;
            tl = t.Z * bR.M21 - t.Y * bR.M31;
            if (tl > rarb)
            {
                distance = tl - rarb;
                axis = new Vector3(aO.M12 * bO.M13 - aO.M13 * bO.M12,
                                   aO.M13 * bO.M11 - aO.M11 * bO.M13,
                                   aO.M11 * bO.M12 - aO.M12 * bO.M11);
                return false;
            }
            if (tl < -rarb)
            {
                distance = -tl - rarb;
                axis = new Vector3(bO.M12 * aO.M13 - bO.M13 * aO.M12,
                                   bO.M13 * aO.M11 - bO.M11 * aO.M13,
                                   bO.M11 * aO.M12 - bO.M12 * aO.M11);
                return false;
            }
            //Inside
            if (tl > F64.C0)
            {
                tempAxis = new Vector3(aO.M12 * bO.M13 - aO.M13 * bO.M12,
                                       aO.M13 * bO.M11 - aO.M11 * bO.M13,
                                       aO.M11 * bO.M12 - aO.M12 * bO.M11);
                axisLengthInverse = F64.C1 / tempAxis.Length();
                tempDistance = (tl - rarb) * axisLengthInverse;
                if (tempDistance > minimumDistance)
                {
                    minimumDistance = tempDistance;
                    tempAxis.X *= axisLengthInverse;
                    tempAxis.Y *= axisLengthInverse;
                    tempAxis.Z *= axisLengthInverse;
                    minimumAxis = tempAxis;
                }
            }
            else
            {
                tempAxis = new Vector3(bO.M12 * aO.M13 - bO.M13 * aO.M12,
                                       bO.M13 * aO.M11 - bO.M11 * aO.M13,
                                       bO.M11 * aO.M12 - bO.M12 * aO.M11);
                axisLengthInverse = F64.C1 / tempAxis.Length();
                tempDistance = (-tl - rarb) * axisLengthInverse;
                if (tempDistance > minimumDistance)
                {
                    minimumDistance = tempDistance;
                    tempAxis.X *= axisLengthInverse;
                    tempAxis.Y *= axisLengthInverse;
                    tempAxis.Z *= axisLengthInverse;
                    minimumAxis = tempAxis;
                }
            }

            //A.X x B.Y
            rarb = aY * absBR.M32 + aZ * absBR.M22 +
                   bX * absBR.M13 + bZ * absBR.M11;
            tl = t.Z * bR.M22 - t.Y * bR.M32;
            if (tl > rarb)
            {
                distance = tl - rarb;
                axis = new Vector3(aO.M12 * bO.M23 - aO.M13 * bO.M22,
                                   aO.M13 * bO.M21 - aO.M11 * bO.M23,
                                   aO.M11 * bO.M22 - aO.M12 * bO.M21);
                return false;
            }
            if (tl < -rarb)
            {
                distance = -tl - rarb;
                axis = new Vector3(bO.M22 * aO.M13 - bO.M23 * aO.M12,
                                   bO.M23 * aO.M11 - bO.M21 * aO.M13,
                                   bO.M21 * aO.M12 - bO.M22 * aO.M11);
                return false;
            }
            //Inside
            if (tl > F64.C0)
            {
                tempAxis = new Vector3(aO.M12 * bO.M23 - aO.M13 * bO.M22,
                                       aO.M13 * bO.M21 - aO.M11 * bO.M23,
                                       aO.M11 * bO.M22 - aO.M12 * bO.M21);
                axisLengthInverse = F64.C1 / tempAxis.Length();
                tempDistance = (tl - rarb) * axisLengthInverse;
                if (tempDistance > minimumDistance)
                {
                    minimumDistance = tempDistance;
                    tempAxis.X *= axisLengthInverse;
                    tempAxis.Y *= axisLengthInverse;
                    tempAxis.Z *= axisLengthInverse;
                    minimumAxis = tempAxis;
                }
            }
            else
            {
                tempAxis = new Vector3(bO.M22 * aO.M13 - bO.M23 * aO.M12,
                                       bO.M23 * aO.M11 - bO.M21 * aO.M13,
                                       bO.M21 * aO.M12 - bO.M22 * aO.M11);
                axisLengthInverse = F64.C1 / tempAxis.Length();
                tempDistance = (-tl - rarb) * axisLengthInverse;
                if (tempDistance > minimumDistance)
                {
                    minimumDistance = tempDistance;
                    tempAxis.X *= axisLengthInverse;
                    tempAxis.Y *= axisLengthInverse;
                    tempAxis.Z *= axisLengthInverse;
                    minimumAxis = tempAxis;
                }
            }

            //A.X x B.Z
            rarb = aY * absBR.M33 + aZ * absBR.M23 +
                   bX * absBR.M12 + bY * absBR.M11;
            tl = t.Z * bR.M23 - t.Y * bR.M33;
            if (tl > rarb)
            {
                distance = tl - rarb;
                axis = new Vector3(aO.M12 * bO.M33 - aO.M13 * bO.M32,
                                   aO.M13 * bO.M31 - aO.M11 * bO.M33,
                                   aO.M11 * bO.M32 - aO.M12 * bO.M31);
                return false;
            }
            if (tl < -rarb)
            {
                distance = -tl - rarb;
                axis = new Vector3(bO.M32 * aO.M13 - bO.M33 * aO.M12,
                                   bO.M33 * aO.M11 - bO.M31 * aO.M13,
                                   bO.M31 * aO.M12 - bO.M32 * aO.M11);
                return false;
            }
            //Inside
            if (tl > F64.C0)
            {
                tempAxis = new Vector3(aO.M12 * bO.M33 - aO.M13 * bO.M32,
                                       aO.M13 * bO.M31 - aO.M11 * bO.M33,
                                       aO.M11 * bO.M32 - aO.M12 * bO.M31);
                axisLengthInverse = F64.C1 / tempAxis.Length();
                tempDistance = (tl - rarb) * axisLengthInverse;
                if (tempDistance > minimumDistance)
                {
                    minimumDistance = tempDistance;
                    tempAxis.X *= axisLengthInverse;
                    tempAxis.Y *= axisLengthInverse;
                    tempAxis.Z *= axisLengthInverse;
                    minimumAxis = tempAxis;
                }
            }
            else
            {
                tempAxis = new Vector3(bO.M32 * aO.M13 - bO.M33 * aO.M12,
                                       bO.M33 * aO.M11 - bO.M31 * aO.M13,
                                       bO.M31 * aO.M12 - bO.M32 * aO.M11);
                axisLengthInverse = F64.C1 / tempAxis.Length();
                tempDistance = (-tl - rarb) * axisLengthInverse;
                if (tempDistance > minimumDistance)
                {
                    minimumDistance = tempDistance;
                    tempAxis.X *= axisLengthInverse;
                    tempAxis.Y *= axisLengthInverse;
                    tempAxis.Z *= axisLengthInverse;
                    minimumAxis = tempAxis;
                }
            }

            #endregion

            #region A.Y x B.()

            //A.Y x B.X
            rarb = aX * absBR.M31 + aZ * absBR.M11 +
                   bY * absBR.M23 + bZ * absBR.M22;
            tl = t.X * bR.M31 - t.Z * bR.M11;
            if (tl > rarb)
            {
                distance = tl - rarb;
                axis = new Vector3(aO.M22 * bO.M13 - aO.M23 * bO.M12,
                                   aO.M23 * bO.M11 - aO.M21 * bO.M13,
                                   aO.M21 * bO.M12 - aO.M22 * bO.M11);
                return false;
            }
            if (tl < -rarb)
            {
                distance = -tl - rarb;
                axis = new Vector3(bO.M12 * aO.M23 - bO.M13 * aO.M22,
                                   bO.M13 * aO.M21 - bO.M11 * aO.M23,
                                   bO.M11 * aO.M22 - bO.M12 * aO.M21);
                return false;
            }
            //Inside
            if (tl > F64.C0)
            {
                tempAxis = new Vector3(aO.M22 * bO.M13 - aO.M23 * bO.M12,
                                       aO.M23 * bO.M11 - aO.M21 * bO.M13,
                                       aO.M21 * bO.M12 - aO.M22 * bO.M11);
                axisLengthInverse = F64.C1 / tempAxis.Length();
                tempDistance = (tl - rarb) * axisLengthInverse;
                if (tempDistance > minimumDistance)
                {
                    minimumDistance = tempDistance;
                    tempAxis.X *= axisLengthInverse;
                    tempAxis.Y *= axisLengthInverse;
                    tempAxis.Z *= axisLengthInverse;
                    minimumAxis = tempAxis;
                }
            }
            else
            {
                tempAxis = new Vector3(bO.M12 * aO.M23 - bO.M13 * aO.M22,
                                       bO.M13 * aO.M21 - bO.M11 * aO.M23,
                                       bO.M11 * aO.M22 - bO.M12 * aO.M21);
                axisLengthInverse = F64.C1 / tempAxis.Length();
                tempDistance = (-tl - rarb) * axisLengthInverse;
                if (tempDistance > minimumDistance)
                {
                    minimumDistance = tempDistance;
                    tempAxis.X *= axisLengthInverse;
                    tempAxis.Y *= axisLengthInverse;
                    tempAxis.Z *= axisLengthInverse;
                    minimumAxis = tempAxis;
                }
            }

            //A.Y x B.Y
            rarb = aX * absBR.M32 + aZ * absBR.M12 +
                   bX * absBR.M23 + bZ * absBR.M21;
            tl = t.X * bR.M32 - t.Z * bR.M12;
            if (tl > rarb)
            {
                distance = tl - rarb;
                axis = new Vector3(aO.M22 * bO.M23 - aO.M23 * bO.M22,
                                   aO.M23 * bO.M21 - aO.M21 * bO.M23,
                                   aO.M21 * bO.M22 - aO.M22 * bO.M21);
                return false;
            }
            if (tl < -rarb)
            {
                distance = -tl - rarb;
                axis = new Vector3(bO.M22 * aO.M23 - bO.M23 * aO.M22,
                                   bO.M23 * aO.M21 - bO.M21 * aO.M23,
                                   bO.M21 * aO.M22 - bO.M22 * aO.M21);
                return false;
            }
            //Inside
            if (tl > F64.C0)
            {
                tempAxis = new Vector3(aO.M22 * bO.M23 - aO.M23 * bO.M22,
                                       aO.M23 * bO.M21 - aO.M21 * bO.M23,
                                       aO.M21 * bO.M22 - aO.M22 * bO.M21);
                axisLengthInverse = F64.C1 / tempAxis.Length();
                tempDistance = (tl - rarb) * axisLengthInverse;
                if (tempDistance > minimumDistance)
                {
                    minimumDistance = tempDistance;
                    tempAxis.X *= axisLengthInverse;
                    tempAxis.Y *= axisLengthInverse;
                    tempAxis.Z *= axisLengthInverse;
                    minimumAxis = tempAxis;
                }
            }
            else
            {
                tempAxis = new Vector3(bO.M22 * aO.M23 - bO.M23 * aO.M22,
                                       bO.M23 * aO.M21 - bO.M21 * aO.M23,
                                       bO.M21 * aO.M22 - bO.M22 * aO.M21);
                axisLengthInverse = F64.C1 / tempAxis.Length();
                tempDistance = (-tl - rarb) * axisLengthInverse;
                if (tempDistance > minimumDistance)
                {
                    minimumDistance = tempDistance;
                    tempAxis.X *= axisLengthInverse;
                    tempAxis.Y *= axisLengthInverse;
                    tempAxis.Z *= axisLengthInverse;
                    minimumAxis = tempAxis;
                }
            }

            //A.Y x B.Z
            rarb = aX * absBR.M33 + aZ * absBR.M13 +
                   bX * absBR.M22 + bY * absBR.M21;
            tl = t.X * bR.M33 - t.Z * bR.M13;
            if (tl > rarb)
            {
                distance = tl - rarb;
                axis = new Vector3(aO.M22 * bO.M33 - aO.M23 * bO.M32,
                                   aO.M23 * bO.M31 - aO.M21 * bO.M33,
                                   aO.M21 * bO.M32 - aO.M22 * bO.M31);
                return false;
            }
            if (tl < -rarb)
            {
                distance = -tl - rarb;
                axis = new Vector3(bO.M32 * aO.M23 - bO.M33 * aO.M22,
                                   bO.M33 * aO.M21 - bO.M31 * aO.M23,
                                   bO.M31 * aO.M22 - bO.M32 * aO.M21);
                return false;
            }
            //Inside
            if (tl > F64.C0)
            {
                tempAxis = new Vector3(aO.M22 * bO.M33 - aO.M23 * bO.M32,
                                       aO.M23 * bO.M31 - aO.M21 * bO.M33,
                                       aO.M21 * bO.M32 - aO.M22 * bO.M31);
                axisLengthInverse = F64.C1 / tempAxis.Length();
                tempDistance = (tl - rarb) * axisLengthInverse;
                if (tempDistance > minimumDistance)
                {
                    minimumDistance = tempDistance;
                    tempAxis.X *= axisLengthInverse;
                    tempAxis.Y *= axisLengthInverse;
                    tempAxis.Z *= axisLengthInverse;
                    minimumAxis = tempAxis;
                }
            }
            else
            {
                tempAxis = new Vector3(bO.M32 * aO.M23 - bO.M33 * aO.M22,
                                       bO.M33 * aO.M21 - bO.M31 * aO.M23,
                                       bO.M31 * aO.M22 - bO.M32 * aO.M21);
                axisLengthInverse = F64.C1 / tempAxis.Length();
                tempDistance = (-tl - rarb) * axisLengthInverse;
                if (tempDistance > minimumDistance)
                {
                    minimumDistance = tempDistance;
                    tempAxis.X *= axisLengthInverse;
                    tempAxis.Y *= axisLengthInverse;
                    tempAxis.Z *= axisLengthInverse;
                    minimumAxis = tempAxis;
                }
            }

            #endregion

            #region A.Z x B.()

            //A.Z x B.X
            rarb = aX * absBR.M21 + aY * absBR.M11 +
                   bY * absBR.M33 + bZ * absBR.M32;
            tl = t.Y * bR.M11 - t.X * bR.M21;
            if (tl > rarb)
            {
                distance = tl - rarb;
                axis = new Vector3(aO.M32 * bO.M13 - aO.M33 * bO.M12,
                                   aO.M33 * bO.M11 - aO.M31 * bO.M13,
                                   aO.M31 * bO.M12 - aO.M32 * bO.M11);
                return false;
            }
            if (tl < -rarb)
            {
                distance = -tl - rarb;
                axis = new Vector3(bO.M12 * aO.M33 - bO.M13 * aO.M32,
                                   bO.M13 * aO.M31 - bO.M11 * aO.M33,
                                   bO.M11 * aO.M32 - bO.M12 * aO.M31);
                return false;
            }
            //Inside
            if (tl > F64.C0)
            {
                tempAxis = new Vector3(aO.M32 * bO.M13 - aO.M33 * bO.M12,
                                       aO.M33 * bO.M11 - aO.M31 * bO.M13,
                                       aO.M31 * bO.M12 - aO.M32 * bO.M11);
                axisLengthInverse = F64.C1 / tempAxis.Length();
                tempDistance = (tl - rarb) * axisLengthInverse;
                if (tempDistance > minimumDistance)
                {
                    minimumDistance = tempDistance;
                    tempAxis.X *= axisLengthInverse;
                    tempAxis.Y *= axisLengthInverse;
                    tempAxis.Z *= axisLengthInverse;
                    minimumAxis = tempAxis;
                }
            }
            else
            {
                tempAxis = new Vector3(bO.M12 * aO.M33 - bO.M13 * aO.M32,
                                       bO.M13 * aO.M31 - bO.M11 * aO.M33,
                                       bO.M11 * aO.M32 - bO.M12 * aO.M31);
                axisLengthInverse = F64.C1 / tempAxis.Length();
                tempDistance = (-tl - rarb) * axisLengthInverse;
                if (tempDistance > minimumDistance)
                {
                    minimumDistance = tempDistance;
                    tempAxis.X *= axisLengthInverse;
                    tempAxis.Y *= axisLengthInverse;
                    tempAxis.Z *= axisLengthInverse;
                    minimumAxis = tempAxis;
                }
            }

            //A.Z x B.Y
            rarb = aX * absBR.M22 + aY * absBR.M12 +
                   bX * absBR.M33 + bZ * absBR.M31;
            tl = t.Y * bR.M12 - t.X * bR.M22;
            if (tl > rarb)
            {
                distance = tl - rarb;
                axis = new Vector3(aO.M32 * bO.M23 - aO.M33 * bO.M22,
                                   aO.M33 * bO.M21 - aO.M31 * bO.M23,
                                   aO.M31 * bO.M22 - aO.M32 * bO.M21);
                return false;
            }
            if (tl < -rarb)
            {
                distance = -tl - rarb;
                axis = new Vector3(bO.M22 * aO.M33 - bO.M23 * aO.M32,
                                   bO.M23 * aO.M31 - bO.M21 * aO.M33,
                                   bO.M21 * aO.M32 - bO.M22 * aO.M31);
                return false;
            }
            //Inside
            if (tl > F64.C0)
            {
                tempAxis = new Vector3(aO.M32 * bO.M23 - aO.M33 * bO.M22,
                                       aO.M33 * bO.M21 - aO.M31 * bO.M23,
                                       aO.M31 * bO.M22 - aO.M32 * bO.M21);
                axisLengthInverse = F64.C1 / tempAxis.Length();
                tempDistance = (tl - rarb) * axisLengthInverse;
                if (tempDistance > minimumDistance)
                {
                    minimumDistance = tempDistance;
                    tempAxis.X *= axisLengthInverse;
                    tempAxis.Y *= axisLengthInverse;
                    tempAxis.Z *= axisLengthInverse;
                    minimumAxis = tempAxis;
                }
            }
            else
            {
                tempAxis = new Vector3(bO.M22 * aO.M33 - bO.M23 * aO.M32,
                                       bO.M23 * aO.M31 - bO.M21 * aO.M33,
                                       bO.M21 * aO.M32 - bO.M22 * aO.M31);
                axisLengthInverse = F64.C1 / tempAxis.Length();
                tempDistance = (-tl - rarb) * axisLengthInverse;
                if (tempDistance > minimumDistance)
                {
                    minimumDistance = tempDistance;
                    tempAxis.X *= axisLengthInverse;
                    tempAxis.Y *= axisLengthInverse;
                    tempAxis.Z *= axisLengthInverse;
                    minimumAxis = tempAxis;
                }
            }

            //A.Z x B.Z
            rarb = aX * absBR.M23 + aY * absBR.M13 +
                   bX * absBR.M32 + bY * absBR.M31;
            tl = t.Y * bR.M13 - t.X * bR.M23;
            if (tl > rarb)
            {
                distance = tl - rarb;
                axis = new Vector3(aO.M32 * bO.M33 - aO.M33 * bO.M32,
                                   aO.M33 * bO.M31 - aO.M31 * bO.M33,
                                   aO.M31 * bO.M32 - aO.M32 * bO.M31);
                return false;
            }
            if (tl < -rarb)
            {
                distance = -tl - rarb;
                axis = new Vector3(bO.M32 * aO.M33 - bO.M33 * aO.M32,
                                   bO.M33 * aO.M31 - bO.M31 * aO.M33,
                                   bO.M31 * aO.M32 - bO.M32 * aO.M31);
                return false;
            }
            //Inside
            if (tl > F64.C0)
            {
                tempAxis = new Vector3(aO.M32 * bO.M33 - aO.M33 * bO.M32,
                                       aO.M33 * bO.M31 - aO.M31 * bO.M33,
                                       aO.M31 * bO.M32 - aO.M32 * bO.M31);
                axisLengthInverse = F64.C1 / tempAxis.Length();
                tempDistance = (tl - rarb) * axisLengthInverse;
                if (tempDistance > minimumDistance)
                {
                    minimumDistance = tempDistance;
                    tempAxis.X *= axisLengthInverse;
                    tempAxis.Y *= axisLengthInverse;
                    tempAxis.Z *= axisLengthInverse;
                    minimumAxis = tempAxis;
                }
            }
            else
            {
                tempAxis = new Vector3(bO.M32 * aO.M33 - bO.M33 * aO.M32,
                                       bO.M33 * aO.M31 - bO.M31 * aO.M33,
                                       bO.M31 * aO.M32 - bO.M32 * aO.M31);
                axisLengthInverse = F64.C1 / tempAxis.Length();
                tempDistance = (-tl - rarb) * axisLengthInverse;
                if (tempDistance > minimumDistance)
                {
                    minimumDistance = tempDistance;
                    tempAxis.X *= axisLengthInverse;
                    tempAxis.Y *= axisLengthInverse;
                    tempAxis.Z *= axisLengthInverse;
                    minimumAxis = tempAxis;
                }
            }

            #endregion

            distance = minimumDistance;
            axis = minimumAxis;
            return true;
        }

#if ALLOWUNSAFE
        /// <summary>
        /// Determines if the two boxes are colliding and computes contact data.
        /// </summary>
        /// <param name="a">First box to collide.</param>
        /// <param name="b">Second box to collide.</param>
        /// <param name="distance">Distance of separation or penetration.</param>
        /// <param name="axis">Axis of separation or penetration.</param>
        /// <param name="contactData">Computed contact data.</param>
        /// <param name="transformA">Transform to apply to shape A.</param>
        /// <param name="transformB">Transform to apply to shape B.</param>
        /// <returns>Whether or not the boxes collide.</returns>
        public static unsafe bool AreBoxesColliding(BoxShape a, BoxShape b, ref RigidTransform transformA, ref RigidTransform transformB, out Fix64 distance, out Vector3 axis, out TinyStructList<BoxContactData> contactData)
        {
            BoxContactDataCache tempData;
            bool toReturn = AreBoxesColliding(a, b, ref transformA, ref transformB, out distance, out axis, out tempData);
            BoxContactData* dataPointer = &tempData.D1;
            contactData = new TinyStructList<BoxContactData>();
            for (int i = 0; i < tempData.Count; i++)
            {
                contactData.Add(ref dataPointer[i]);
            }
            return toReturn;
        }
#endif

        /// <summary>
        /// Determines if the two boxes are colliding and computes contact data.
        /// </summary>
        /// <param name="a">First box to collide.</param>
        /// <param name="b">Second box to collide.</param>
        /// <param name="distance">Distance of separation or penetration.</param>
        /// <param name="axis">Axis of separation or penetration.</param>
        /// <param name="contactData">Contact positions, depths, and ids.</param>
        /// <param name="transformA">Transform to apply to shape A.</param>
        /// <param name="transformB">Transform to apply to shape B.</param>
        /// <returns>Whether or not the boxes collide.</returns>
#if ALLOWUNSAFE
        public static bool AreBoxesColliding(BoxShape a, BoxShape b, ref RigidTransform transformA, ref RigidTransform transformB, out Fix64 distance, out Vector3 axis, out BoxContactDataCache contactData)
#else
        public static bool AreBoxesColliding(BoxShape a, BoxShape b, ref RigidTransform transformA, ref RigidTransform transformB, out Fix64 distance, out Vector3 axis, out TinyStructList<BoxContactData> contactData)
#endif
        {
            Fix64 aX = a.HalfWidth;
            Fix64 aY = a.HalfHeight;
            Fix64 aZ = a.HalfLength;

            Fix64 bX = b.HalfWidth;
            Fix64 bY = b.HalfHeight;
            Fix64 bZ = b.HalfLength;

#if ALLOWUNSAFE
            contactData = new BoxContactDataCache();
#else
            contactData = new TinyStructList<BoxContactData>();
#endif
            //Relative rotation from A to B.
            Matrix3x3 bR;

            Matrix3x3 aO;
            Matrix3x3.CreateFromQuaternion(ref transformA.Orientation, out aO);
            Matrix3x3 bO;
            Matrix3x3.CreateFromQuaternion(ref transformB.Orientation, out bO);

            //Relative translation rotated into A's configuration space.
            Vector3 t;
            Vector3.Subtract(ref transformB.Position, ref transformA.Position, out t);

            Fix64 tempDistance;
            Fix64 minimumDistance = -Fix64.MaxValue;
            var minimumAxis = new Vector3();
            byte minimumFeature = 2; //2 means edge.  0-> A face, 1 -> B face.

            #region A Face Normals

            bR.M11 = aO.M11 * bO.M11 + aO.M12 * bO.M12 + aO.M13 * bO.M13;
            bR.M12 = aO.M11 * bO.M21 + aO.M12 * bO.M22 + aO.M13 * bO.M23;
            bR.M13 = aO.M11 * bO.M31 + aO.M12 * bO.M32 + aO.M13 * bO.M33;
            Matrix3x3 absBR;
            //Epsilons are added to deal with near-parallel edges.
            absBR.M11 = Fix64.Abs(bR.M11) + Toolbox.Epsilon;
            absBR.M12 = Fix64.Abs(bR.M12) + Toolbox.Epsilon;
            absBR.M13 = Fix64.Abs(bR.M13) + Toolbox.Epsilon;
            Fix64 tX = t.X;
            t.X = t.X * aO.M11 + t.Y * aO.M12 + t.Z * aO.M13;

            //Test the axes defines by entity A's rotation matrix.
            //A.X
            Fix64 rarb = aX + bX * absBR.M11 + bY * absBR.M12 + bZ * absBR.M13;
            if (t.X > rarb)
            {
                distance = t.X - rarb;
                axis = new Vector3(-aO.M11, -aO.M12, -aO.M13);
                return false;
            }
            if (t.X < -rarb)
            {
                distance = -t.X - rarb;
                axis = new Vector3(aO.M11, aO.M12, aO.M13);
                return false;
            }
            //Inside
            if (t.X > F64.C0)
            {
                tempDistance = t.X - rarb;
                if (tempDistance > minimumDistance)
                {
                    minimumDistance = tempDistance;
                    minimumAxis = new Vector3(-aO.M11, -aO.M12, -aO.M13);
                    minimumFeature = 0;
                }
            }
            else
            {
                tempDistance = -t.X - rarb;
                if (tempDistance > minimumDistance)
                {
                    minimumDistance = tempDistance;
                    minimumAxis = new Vector3(aO.M11, aO.M12, aO.M13);
                    minimumFeature = 0;
                }
            }


            bR.M21 = aO.M21 * bO.M11 + aO.M22 * bO.M12 + aO.M23 * bO.M13;
            bR.M22 = aO.M21 * bO.M21 + aO.M22 * bO.M22 + aO.M23 * bO.M23;
            bR.M23 = aO.M21 * bO.M31 + aO.M22 * bO.M32 + aO.M23 * bO.M33;
            absBR.M21 = Fix64.Abs(bR.M21) + Toolbox.Epsilon;
            absBR.M22 = Fix64.Abs(bR.M22) + Toolbox.Epsilon;
            absBR.M23 = Fix64.Abs(bR.M23) + Toolbox.Epsilon;
            Fix64 tY = t.Y;
            t.Y = tX * aO.M21 + t.Y * aO.M22 + t.Z * aO.M23;

            //A.Y
            rarb = aY + bX * absBR.M21 + bY * absBR.M22 + bZ * absBR.M23;
            if (t.Y > rarb)
            {
                distance = t.Y - rarb;
                axis = new Vector3(-aO.M21, -aO.M22, -aO.M23);
                return false;
            }
            if (t.Y < -rarb)
            {
                distance = -t.Y - rarb;
                axis = new Vector3(aO.M21, aO.M22, aO.M23);
                return false;
            }
            //Inside
            if (t.Y > F64.C0)
            {
                tempDistance = t.Y - rarb;
                if (tempDistance > minimumDistance)
                {
                    minimumDistance = tempDistance;
                    minimumAxis = new Vector3(-aO.M21, -aO.M22, -aO.M23);
                    minimumFeature = 0;
                }
            }
            else
            {
                tempDistance = -t.Y - rarb;
                if (tempDistance > minimumDistance)
                {
                    minimumDistance = tempDistance;
                    minimumAxis = new Vector3(aO.M21, aO.M22, aO.M23);
                    minimumFeature = 0;
                }
            }

            bR.M31 = aO.M31 * bO.M11 + aO.M32 * bO.M12 + aO.M33 * bO.M13;
            bR.M32 = aO.M31 * bO.M21 + aO.M32 * bO.M22 + aO.M33 * bO.M23;
            bR.M33 = aO.M31 * bO.M31 + aO.M32 * bO.M32 + aO.M33 * bO.M33;
            absBR.M31 = Fix64.Abs(bR.M31) + Toolbox.Epsilon;
            absBR.M32 = Fix64.Abs(bR.M32) + Toolbox.Epsilon;
            absBR.M33 = Fix64.Abs(bR.M33) + Toolbox.Epsilon;
            t.Z = tX * aO.M31 + tY * aO.M32 + t.Z * aO.M33;

            //A.Z
            rarb = aZ + bX * absBR.M31 + bY * absBR.M32 + bZ * absBR.M33;
            if (t.Z > rarb)
            {
                distance = t.Z - rarb;
                axis = new Vector3(-aO.M31, -aO.M32, -aO.M33);
                return false;
            }
            if (t.Z < -rarb)
            {
                distance = -t.Z - rarb;
                axis = new Vector3(aO.M31, aO.M32, aO.M33);
                return false;
            }
            //Inside
            if (t.Z > F64.C0)
            {
                tempDistance = t.Z - rarb;
                if (tempDistance > minimumDistance)
                {
                    minimumDistance = tempDistance;
                    minimumAxis = new Vector3(-aO.M31, -aO.M32, -aO.M33);
                    minimumFeature = 0;
                }
            }
            else
            {
                tempDistance = -t.Z - rarb;
                if (tempDistance > minimumDistance)
                {
                    minimumDistance = tempDistance;
                    minimumAxis = new Vector3(aO.M31, aO.M32, aO.M33);
                    minimumFeature = 0;
                }
            }

			#endregion

			Fix64 antiBBias = F64.C0p01;
            minimumDistance += antiBBias;

            #region B Face Normals

            //Test the axes defines by entity B's rotation matrix.
            //B.X
            rarb = bX + aX * absBR.M11 + aY * absBR.M21 + aZ * absBR.M31;
            Fix64 tl = t.X * bR.M11 + t.Y * bR.M21 + t.Z * bR.M31;
            if (tl > rarb)
            {
                distance = tl - rarb;
                axis = new Vector3(-bO.M11, -bO.M12, -bO.M13);
                return false;
            }
            if (tl < -rarb)
            {
                distance = -tl - rarb;
                axis = new Vector3(bO.M11, bO.M12, bO.M13);
                return false;
            }
            //Inside
            if (tl > F64.C0)
            {
                tempDistance = tl - rarb;
                if (tempDistance > minimumDistance)
                {
                    minimumDistance = tempDistance;
                    minimumAxis = new Vector3(-bO.M11, -bO.M12, -bO.M13);
                    minimumFeature = 1;
                }
            }
            else
            {
                tempDistance = -tl - rarb;
                if (tempDistance > minimumDistance)
                {
                    minimumDistance = tempDistance;
                    minimumAxis = new Vector3(bO.M11, bO.M12, bO.M13);
                    minimumFeature = 1;
                }
            }

            //B.Y
            rarb = bY + aX * absBR.M12 + aY * absBR.M22 + aZ * absBR.M32;
            tl = t.X * bR.M12 + t.Y * bR.M22 + t.Z * bR.M32;
            if (tl > rarb)
            {
                distance = tl - rarb;
                axis = new Vector3(-bO.M21, -bO.M22, -bO.M23);
                return false;
            }
            if (tl < -rarb)
            {
                distance = -tl - rarb;
                axis = new Vector3(bO.M21, bO.M22, bO.M23);
                return false;
            }
            //Inside
            if (tl > F64.C0)
            {
                tempDistance = tl - rarb;
                if (tempDistance > minimumDistance)
                {
                    minimumDistance = tempDistance;
                    minimumAxis = new Vector3(-bO.M21, -bO.M22, -bO.M23);
                    minimumFeature = 1;
                }
            }
            else
            {
                tempDistance = -tl - rarb;
                if (tempDistance > minimumDistance)
                {
                    minimumDistance = tempDistance;
                    minimumAxis = new Vector3(bO.M21, bO.M22, bO.M23);
                    minimumFeature = 1;
                }
            }

            //B.Z
            rarb = bZ + aX * absBR.M13 + aY * absBR.M23 + aZ * absBR.M33;
            tl = t.X * bR.M13 + t.Y * bR.M23 + t.Z * bR.M33;
            if (tl > rarb)
            {
                distance = tl - rarb;
                axis = new Vector3(-bO.M31, -bO.M32, -bO.M33);
                return false;
            }
            if (tl < -rarb)
            {
                distance = -tl - rarb;
                axis = new Vector3(bO.M31, bO.M32, bO.M33);
                return false;
            }
            //Inside
            if (tl > F64.C0)
            {
                tempDistance = tl - rarb;
                if (tempDistance > minimumDistance)
                {
                    minimumDistance = tempDistance;
                    minimumAxis = new Vector3(-bO.M31, -bO.M32, -bO.M33);
                    minimumFeature = 1;
                }
            }
            else
            {
                tempDistance = -tl - rarb;
                if (tempDistance > minimumDistance)
                {
                    minimumDistance = tempDistance;
                    minimumAxis = new Vector3(bO.M31, bO.M32, bO.M33);
                    minimumFeature = 1;
                }
            }

            #endregion

            if (minimumFeature != 1)
                minimumDistance -= antiBBias;

            Fix64 antiEdgeBias = F64.C0p01;
            minimumDistance += antiEdgeBias;
            Fix64 axisLengthInverse;
            Vector3 tempAxis;

            #region A.X x B.()

            //Now for the edge-edge cases.
            //A.X x B.X
            rarb = aY * absBR.M31 + aZ * absBR.M21 +
                   bY * absBR.M13 + bZ * absBR.M12;
            tl = t.Z * bR.M21 - t.Y * bR.M31;
            if (tl > rarb)
            {
                distance = tl - rarb;
                axis = new Vector3(bO.M12 * aO.M13 - bO.M13 * aO.M12,
                                   bO.M13 * aO.M11 - bO.M11 * aO.M13,
                                   bO.M11 * aO.M12 - bO.M12 * aO.M11);
                return false;
            }
            if (tl < -rarb)
            {
                distance = -tl - rarb;
                axis = new Vector3(aO.M12 * bO.M13 - aO.M13 * bO.M12,
                                   aO.M13 * bO.M11 - aO.M11 * bO.M13,
                                   aO.M11 * bO.M12 - aO.M12 * bO.M11);
                return false;
            }
            //Inside
            if (tl > F64.C0)
            {
                tempAxis = new Vector3(bO.M12 * aO.M13 - bO.M13 * aO.M12,
                                       bO.M13 * aO.M11 - bO.M11 * aO.M13,
                                       bO.M11 * aO.M12 - bO.M12 * aO.M11);

                axisLengthInverse = F64.C1 / tempAxis.Length();
                tempDistance = (tl - rarb) * axisLengthInverse;
                if (tempDistance > minimumDistance)
                {
                    minimumFeature = 2;
                    minimumDistance = tempDistance;
                    tempAxis.X *= axisLengthInverse;
                    tempAxis.Y *= axisLengthInverse;
                    tempAxis.Z *= axisLengthInverse;
                    minimumAxis = tempAxis;
                }
            }
            else
            {
                tempAxis = new Vector3(aO.M12 * bO.M13 - aO.M13 * bO.M12,
                                       aO.M13 * bO.M11 - aO.M11 * bO.M13,
                                       aO.M11 * bO.M12 - aO.M12 * bO.M11);
                axisLengthInverse = F64.C1 / tempAxis.Length();
                tempDistance = (-tl - rarb) * axisLengthInverse;
                if (tempDistance > minimumDistance)
                {
                    minimumFeature = 2;
                    minimumDistance = tempDistance;
                    tempAxis.X *= axisLengthInverse;
                    tempAxis.Y *= axisLengthInverse;
                    tempAxis.Z *= axisLengthInverse;
                    minimumAxis = tempAxis;
                }
            }

            //A.X x B.Y
            rarb = aY * absBR.M32 + aZ * absBR.M22 +
                   bX * absBR.M13 + bZ * absBR.M11;
            tl = t.Z * bR.M22 - t.Y * bR.M32;
            if (tl > rarb)
            {
                distance = tl - rarb;
                axis = new Vector3(bO.M22 * aO.M13 - bO.M23 * aO.M12,
                                   bO.M23 * aO.M11 - bO.M21 * aO.M13,
                                   bO.M21 * aO.M12 - bO.M22 * aO.M11);
                return false;
            }
            if (tl < -rarb)
            {
                distance = -tl - rarb;
                axis = new Vector3(aO.M12 * bO.M23 - aO.M13 * bO.M22,
                                   aO.M13 * bO.M21 - aO.M11 * bO.M23,
                                   aO.M11 * bO.M22 - aO.M12 * bO.M21);

                return false;
            }
            //Inside
            if (tl > F64.C0)
            {
                tempAxis = new Vector3(bO.M22 * aO.M13 - bO.M23 * aO.M12,
                                       bO.M23 * aO.M11 - bO.M21 * aO.M13,
                                       bO.M21 * aO.M12 - bO.M22 * aO.M11);

                axisLengthInverse = F64.C1 / tempAxis.Length();
                tempDistance = (tl - rarb) * axisLengthInverse;
                if (tempDistance > minimumDistance)
                {
                    minimumFeature = 2;
                    minimumDistance = tempDistance;
                    tempAxis.X *= axisLengthInverse;
                    tempAxis.Y *= axisLengthInverse;
                    tempAxis.Z *= axisLengthInverse;
                    minimumAxis = tempAxis;
                }
            }
            else
            {
                tempAxis = new Vector3(aO.M12 * bO.M23 - aO.M13 * bO.M22,
                                       aO.M13 * bO.M21 - aO.M11 * bO.M23,
                                       aO.M11 * bO.M22 - aO.M12 * bO.M21);
                axisLengthInverse = F64.C1 / tempAxis.Length();
                tempDistance = (-tl - rarb) * axisLengthInverse;
                if (tempDistance > minimumDistance)
                {
                    minimumFeature = 2;
                    minimumDistance = tempDistance;
                    tempAxis.X *= axisLengthInverse;
                    tempAxis.Y *= axisLengthInverse;
                    tempAxis.Z *= axisLengthInverse;
                    minimumAxis = tempAxis;
                }
            }

            //A.X x B.Z
            rarb = aY * absBR.M33 + aZ * absBR.M23 +
                   bX * absBR.M12 + bY * absBR.M11;
            tl = t.Z * bR.M23 - t.Y * bR.M33;
            if (tl > rarb)
            {
                distance = tl - rarb;
                axis = new Vector3(bO.M32 * aO.M13 - bO.M33 * aO.M12,
                                   bO.M33 * aO.M11 - bO.M31 * aO.M13,
                                   bO.M31 * aO.M12 - bO.M32 * aO.M11);
                return false;
            }
            if (tl < -rarb)
            {
                distance = -tl - rarb;

                axis = new Vector3(aO.M12 * bO.M33 - aO.M13 * bO.M32,
                                   aO.M13 * bO.M31 - aO.M11 * bO.M33,
                                   aO.M11 * bO.M32 - aO.M12 * bO.M31);
                return false;
            }
            //Inside
            if (tl > F64.C0)
            {
                tempAxis = new Vector3(bO.M32 * aO.M13 - bO.M33 * aO.M12,
                                       bO.M33 * aO.M11 - bO.M31 * aO.M13,
                                       bO.M31 * aO.M12 - bO.M32 * aO.M11);
                axisLengthInverse = F64.C1 / tempAxis.Length();
                tempDistance = (tl - rarb) * axisLengthInverse;
                if (tempDistance > minimumDistance)
                {
                    minimumFeature = 2;
                    minimumDistance = tempDistance;
                    tempAxis.X *= axisLengthInverse;
                    tempAxis.Y *= axisLengthInverse;
                    tempAxis.Z *= axisLengthInverse;
                    minimumAxis = tempAxis;
                }
            }
            else
            {
                tempAxis = new Vector3(aO.M12 * bO.M33 - aO.M13 * bO.M32,
                                       aO.M13 * bO.M31 - aO.M11 * bO.M33,
                                       aO.M11 * bO.M32 - aO.M12 * bO.M31);

                axisLengthInverse = F64.C1 / tempAxis.Length();
                tempDistance = (-tl - rarb) * axisLengthInverse;
                if (tempDistance > minimumDistance)
                {
                    minimumFeature = 2;
                    minimumDistance = tempDistance;
                    tempAxis.X *= axisLengthInverse;
                    tempAxis.Y *= axisLengthInverse;
                    tempAxis.Z *= axisLengthInverse;
                    minimumAxis = tempAxis;
                }
            }

            #endregion

            #region A.Y x B.()

            //A.Y x B.X
            rarb = aX * absBR.M31 + aZ * absBR.M11 +
                   bY * absBR.M23 + bZ * absBR.M22;
            tl = t.X * bR.M31 - t.Z * bR.M11;
            if (tl > rarb)
            {
                distance = tl - rarb;
                axis = new Vector3(bO.M12 * aO.M23 - bO.M13 * aO.M22,
                                   bO.M13 * aO.M21 - bO.M11 * aO.M23,
                                   bO.M11 * aO.M22 - bO.M12 * aO.M21);
                return false;
            }
            if (tl < -rarb)
            {
                distance = -tl - rarb;
                axis = new Vector3(aO.M22 * bO.M13 - aO.M23 * bO.M12,
                                   aO.M23 * bO.M11 - aO.M21 * bO.M13,
                                   aO.M21 * bO.M12 - aO.M22 * bO.M11);

                return false;
            }
            //Inside
            if (tl > F64.C0)
            {
                tempAxis = new Vector3(bO.M12 * aO.M23 - bO.M13 * aO.M22,
                                       bO.M13 * aO.M21 - bO.M11 * aO.M23,
                                       bO.M11 * aO.M22 - bO.M12 * aO.M21);
                axisLengthInverse = F64.C1 / tempAxis.Length();
                tempDistance = (tl - rarb) * axisLengthInverse;
                if (tempDistance > minimumDistance)
                {
                    minimumFeature = 2;
                    minimumDistance = tempDistance;
                    tempAxis.X *= axisLengthInverse;
                    tempAxis.Y *= axisLengthInverse;
                    tempAxis.Z *= axisLengthInverse;
                    minimumAxis = tempAxis;
                }
            }
            else
            {
                tempAxis = new Vector3(aO.M22 * bO.M13 - aO.M23 * bO.M12,
                                       aO.M23 * bO.M11 - aO.M21 * bO.M13,
                                       aO.M21 * bO.M12 - aO.M22 * bO.M11);

                axisLengthInverse = F64.C1 / tempAxis.Length();
                tempDistance = (-tl - rarb) * axisLengthInverse;
                if (tempDistance > minimumDistance)
                {
                    minimumFeature = 2;
                    minimumDistance = tempDistance;
                    tempAxis.X *= axisLengthInverse;
                    tempAxis.Y *= axisLengthInverse;
                    tempAxis.Z *= axisLengthInverse;
                    minimumAxis = tempAxis;
                }
            }

            //A.Y x B.Y
            rarb = aX * absBR.M32 + aZ * absBR.M12 +
                   bX * absBR.M23 + bZ * absBR.M21;
            tl = t.X * bR.M32 - t.Z * bR.M12;
            if (tl > rarb)
            {
                distance = tl - rarb;
                axis = new Vector3(bO.M22 * aO.M23 - bO.M23 * aO.M22,
                                   bO.M23 * aO.M21 - bO.M21 * aO.M23,
                                   bO.M21 * aO.M22 - bO.M22 * aO.M21);
                return false;
            }
            if (tl < -rarb)
            {
                distance = -tl - rarb;

                axis = new Vector3(aO.M22 * bO.M23 - aO.M23 * bO.M22,
                                   aO.M23 * bO.M21 - aO.M21 * bO.M23,
                                   aO.M21 * bO.M22 - aO.M22 * bO.M21);
                return false;
            }
            //Inside
            if (tl > F64.C0)
            {
                tempAxis = new Vector3(bO.M22 * aO.M23 - bO.M23 * aO.M22,
                                       bO.M23 * aO.M21 - bO.M21 * aO.M23,
                                       bO.M21 * aO.M22 - bO.M22 * aO.M21);
                axisLengthInverse = F64.C1 / tempAxis.Length();
                tempDistance = (tl - rarb) * axisLengthInverse;
                if (tempDistance > minimumDistance)
                {
                    minimumFeature = 2;
                    minimumDistance = tempDistance;
                    tempAxis.X *= axisLengthInverse;
                    tempAxis.Y *= axisLengthInverse;
                    tempAxis.Z *= axisLengthInverse;
                    minimumAxis = tempAxis;
                }
            }
            else
            {
                tempAxis = new Vector3(aO.M22 * bO.M23 - aO.M23 * bO.M22,
                                       aO.M23 * bO.M21 - aO.M21 * bO.M23,
                                       aO.M21 * bO.M22 - aO.M22 * bO.M21);

                axisLengthInverse = F64.C1 / tempAxis.Length();
                tempDistance = (-tl - rarb) * axisLengthInverse;
                if (tempDistance > minimumDistance)
                {
                    minimumFeature = 2;
                    minimumDistance = tempDistance;
                    tempAxis.X *= axisLengthInverse;
                    tempAxis.Y *= axisLengthInverse;
                    tempAxis.Z *= axisLengthInverse;
                    minimumAxis = tempAxis;
                }
            }

            //A.Y x B.Z
            rarb = aX * absBR.M33 + aZ * absBR.M13 +
                   bX * absBR.M22 + bY * absBR.M21;
            tl = t.X * bR.M33 - t.Z * bR.M13;
            if (tl > rarb)
            {
                distance = tl - rarb;
                axis = new Vector3(bO.M32 * aO.M23 - bO.M33 * aO.M22,
                                   bO.M33 * aO.M21 - bO.M31 * aO.M23,
                                   bO.M31 * aO.M22 - bO.M32 * aO.M21);
                return false;
            }
            if (tl < -rarb)
            {
                distance = -tl - rarb;

                axis = new Vector3(aO.M22 * bO.M33 - aO.M23 * bO.M32,
                                   aO.M23 * bO.M31 - aO.M21 * bO.M33,
                                   aO.M21 * bO.M32 - aO.M22 * bO.M31);
                return false;
            }
            //Inside
            if (tl > F64.C0)
            {
                tempAxis = new Vector3(bO.M32 * aO.M23 - bO.M33 * aO.M22,
                                       bO.M33 * aO.M21 - bO.M31 * aO.M23,
                                       bO.M31 * aO.M22 - bO.M32 * aO.M21);
                axisLengthInverse = F64.C1 / tempAxis.Length();
                tempDistance = (tl - rarb) * axisLengthInverse;
                if (tempDistance > minimumDistance)
                {
                    minimumFeature = 2;
                    minimumDistance = tempDistance;
                    tempAxis.X *= axisLengthInverse;
                    tempAxis.Y *= axisLengthInverse;
                    tempAxis.Z *= axisLengthInverse;
                    minimumAxis = tempAxis;
                }
            }
            else
            {
                tempAxis = new Vector3(aO.M22 * bO.M33 - aO.M23 * bO.M32,
                                       aO.M23 * bO.M31 - aO.M21 * bO.M33,
                                       aO.M21 * bO.M32 - aO.M22 * bO.M31);

                axisLengthInverse = F64.C1 / tempAxis.Length();
                tempDistance = (-tl - rarb) * axisLengthInverse;
                if (tempDistance > minimumDistance)
                {
                    minimumFeature = 2;
                    minimumDistance = tempDistance;
                    tempAxis.X *= axisLengthInverse;
                    tempAxis.Y *= axisLengthInverse;
                    tempAxis.Z *= axisLengthInverse;
                    minimumAxis = tempAxis;
                }
            }

            #endregion

            #region A.Z x B.()

            //A.Z x B.X
            rarb = aX * absBR.M21 + aY * absBR.M11 +
                   bY * absBR.M33 + bZ * absBR.M32;
            tl = t.Y * bR.M11 - t.X * bR.M21;
            if (tl > rarb)
            {
                distance = tl - rarb;
                axis = new Vector3(bO.M12 * aO.M33 - bO.M13 * aO.M32,
                                   bO.M13 * aO.M31 - bO.M11 * aO.M33,
                                   bO.M11 * aO.M32 - bO.M12 * aO.M31);
                return false;
            }
            if (tl < -rarb)
            {
                distance = -tl - rarb;

                axis = new Vector3(aO.M32 * bO.M13 - aO.M33 * bO.M12,
                                   aO.M33 * bO.M11 - aO.M31 * bO.M13,
                                   aO.M31 * bO.M12 - aO.M32 * bO.M11);
                return false;
            }
            //Inside
            if (tl > F64.C0)
            {
                tempAxis = new Vector3(bO.M12 * aO.M33 - bO.M13 * aO.M32,
                                       bO.M13 * aO.M31 - bO.M11 * aO.M33,
                                       bO.M11 * aO.M32 - bO.M12 * aO.M31);
                axisLengthInverse = F64.C1 / tempAxis.Length();
                tempDistance = (tl - rarb) * axisLengthInverse;
                if (tempDistance > minimumDistance)
                {
                    minimumFeature = 2;
                    minimumDistance = tempDistance;
                    tempAxis.X *= axisLengthInverse;
                    tempAxis.Y *= axisLengthInverse;
                    tempAxis.Z *= axisLengthInverse;
                    minimumAxis = tempAxis;
                }
            }
            else
            {
                tempAxis = new Vector3(aO.M32 * bO.M13 - aO.M33 * bO.M12,
                                       aO.M33 * bO.M11 - aO.M31 * bO.M13,
                                       aO.M31 * bO.M12 - aO.M32 * bO.M11);
                axisLengthInverse = F64.C1 / tempAxis.Length();
                tempDistance = (-tl - rarb) * axisLengthInverse;
                if (tempDistance > minimumDistance)
                {
                    minimumFeature = 2;
                    minimumDistance = tempDistance;
                    tempAxis.X *= axisLengthInverse;
                    tempAxis.Y *= axisLengthInverse;
                    tempAxis.Z *= axisLengthInverse;
                    minimumAxis = tempAxis;
                }
            }

            //A.Z x B.Y
            rarb = aX * absBR.M22 + aY * absBR.M12 +
                   bX * absBR.M33 + bZ * absBR.M31;
            tl = t.Y * bR.M12 - t.X * bR.M22;
            if (tl > rarb)
            {
                distance = tl - rarb;
                axis = new Vector3(bO.M22 * aO.M33 - bO.M23 * aO.M32,
                                   bO.M23 * aO.M31 - bO.M21 * aO.M33,
                                   bO.M21 * aO.M32 - bO.M22 * aO.M31);
                return false;
            }
            if (tl < -rarb)
            {
                distance = -tl - rarb;

                axis = new Vector3(aO.M32 * bO.M23 - aO.M33 * bO.M22,
                                   aO.M33 * bO.M21 - aO.M31 * bO.M23,
                                   aO.M31 * bO.M22 - aO.M32 * bO.M21);
                return false;
            }
            //Inside
            if (tl > F64.C0)
            {
                tempAxis = new Vector3(bO.M22 * aO.M33 - bO.M23 * aO.M32,
                                       bO.M23 * aO.M31 - bO.M21 * aO.M33,
                                       bO.M21 * aO.M32 - bO.M22 * aO.M31);
                axisLengthInverse = F64.C1 / tempAxis.Length();
                tempDistance = (tl - rarb) * axisLengthInverse;
                if (tempDistance > minimumDistance)
                {
                    minimumFeature = 2;
                    minimumDistance = tempDistance;
                    tempAxis.X *= axisLengthInverse;
                    tempAxis.Y *= axisLengthInverse;
                    tempAxis.Z *= axisLengthInverse;
                    minimumAxis = tempAxis;
                }
            }
            else
            {
                tempAxis = new Vector3(aO.M32 * bO.M23 - aO.M33 * bO.M22,
                                       aO.M33 * bO.M21 - aO.M31 * bO.M23,
                                       aO.M31 * bO.M22 - aO.M32 * bO.M21);
                axisLengthInverse = F64.C1 / tempAxis.Length();
                tempDistance = (-tl - rarb) * axisLengthInverse;
                if (tempDistance > minimumDistance)
                {
                    minimumFeature = 2;
                    minimumDistance = tempDistance;
                    tempAxis.X *= axisLengthInverse;
                    tempAxis.Y *= axisLengthInverse;
                    tempAxis.Z *= axisLengthInverse;
                    minimumAxis = tempAxis;
                }
            }

            //A.Z x B.Z
            rarb = aX * absBR.M23 + aY * absBR.M13 +
                   bX * absBR.M32 + bY * absBR.M31;
            tl = t.Y * bR.M13 - t.X * bR.M23;
            if (tl > rarb)
            {
                distance = tl - rarb;
                axis = new Vector3(bO.M32 * aO.M33 - bO.M33 * aO.M32,
                                   bO.M33 * aO.M31 - bO.M31 * aO.M33,
                                   bO.M31 * aO.M32 - bO.M32 * aO.M31);
                return false;
            }
            if (tl < -rarb)
            {
                distance = -tl - rarb;
                axis = new Vector3(aO.M32 * bO.M33 - aO.M33 * bO.M32,
                                   aO.M33 * bO.M31 - aO.M31 * bO.M33,
                                   aO.M31 * bO.M32 - aO.M32 * bO.M31);
                return false;
            }
            //Inside
            if (tl > F64.C0)
            {
                tempAxis = new Vector3(bO.M32 * aO.M33 - bO.M33 * aO.M32,
                                       bO.M33 * aO.M31 - bO.M31 * aO.M33,
                                       bO.M31 * aO.M32 - bO.M32 * aO.M31);
                axisLengthInverse = F64.C1 / tempAxis.Length();
                tempDistance = (tl - rarb) * axisLengthInverse;
                if (tempDistance > minimumDistance)
                {
                    minimumFeature = 2;
                    minimumDistance = tempDistance;
                    tempAxis.X *= axisLengthInverse;
                    tempAxis.Y *= axisLengthInverse;
                    tempAxis.Z *= axisLengthInverse;
                    minimumAxis = tempAxis;
                }
            }
            else
            {
                tempAxis = new Vector3(aO.M32 * bO.M33 - aO.M33 * bO.M32,
                                       aO.M33 * bO.M31 - aO.M31 * bO.M33,
                                       aO.M31 * bO.M32 - aO.M32 * bO.M31);

                axisLengthInverse = F64.C1 / tempAxis.Length();
                tempDistance = (-tl - rarb) * axisLengthInverse;
                if (tempDistance > minimumDistance)
                {
                    minimumFeature = 2;
                    minimumDistance = tempDistance;
                    tempAxis.X *= axisLengthInverse;
                    tempAxis.Y *= axisLengthInverse;
                    tempAxis.Z *= axisLengthInverse;
                    minimumAxis = tempAxis;
                }
            }

            #endregion

            if (minimumFeature == 2)
            {

                //Edge-edge contact conceptually only has one contact, but allowing it to create multiple due to penetration is more robust.
                GetEdgeEdgeContact(a, b, ref transformA.Position, ref aO, ref transformB.Position, ref bO, minimumDistance, ref minimumAxis, out contactData);

                //Vector3 position;
                //Fix64 depth;
                //int id;
                //                GetEdgeEdgeContact(a, b, ref transformA.Position, ref aO, ref transformB.Position, ref bO, ref minimumAxis, out position, out id);
                //#if ALLOWUNSAFE
                //                contactData.D1.Position = position;
                //                contactData.D1.Depth = minimumDistance; 
                //                contactData.D1.Id = id;
                //                contactData.Count = 1;
                //#else
                //                var toAdd = new BoxContactData();
                //                toAdd.Position = position;
                //                toAdd.Depth = minimumDistance;
                //                toAdd.Id = id;
                //                contactData.Add(ref toAdd);
                //#endif
            }
            else
            {
                minimumDistance -= antiEdgeBias;
                GetFaceContacts(a, b, ref transformA.Position, ref aO, ref transformB.Position, ref bO, minimumFeature == 0, ref minimumAxis, out contactData);

            }

            distance = minimumDistance;
            axis = minimumAxis;
            return true;
        }

#if ALLOWUNSAFE
        internal static void GetEdgeEdgeContact(BoxShape a, BoxShape b, ref Vector3 positionA, ref Matrix3x3 orientationA, ref Vector3 positionB, ref Matrix3x3 orientationB, Fix64 depth, ref Vector3 mtd, out BoxContactDataCache contactData)
#else
        internal static void GetEdgeEdgeContact(BoxShape a, BoxShape b, ref Vector3 positionA, ref Matrix3x3 orientationA, ref Vector3 positionB, ref Matrix3x3 orientationB, Fix64 depth, ref Vector3 mtd, out TinyStructList<BoxContactData> contactData)
#endif
        {
            //Edge-edge contacts conceptually can only create one contact in perfectly rigid collisions.
            //However, this is a discrete approximation of rigidity; things can penetrate each other.
            //If edge-edge only returns a single contact, there's a good chance that the box will get into
            //an oscillating state when under pressure.

            //To avoid the oscillation, we may sometimes need two edge contacts.
            //To determine which edges to use, compute 8 dot products.
            //One for each edge parallel to the contributing axis on each of the two shapes.
            //The resulting cases are:
            //One edge on A touching one edge on B.
            //Two edges on A touching one edge on B.
            //One edge on A touching two edges on B.
            //Two edges on A touching two edges on B.

            //The three latter cases SHOULD be covered by the face-contact system, but in practice,
            //they are not sufficiently covered because the system decides that the single edge-edge pair
            //should be used and drops the other contacts, producting the aforementioned oscillation.

            //All edge cross products result in the MTD, so no recalculation is necessary.

            //Of the four edges which are aligned with the local edge axis, pick the two
            //who have vertices which, when dotted with the local mtd, are greatest.

            //Compute the closest points between each edge pair.  For two edges each,
            //this comes out to four total closest point tests.
            //This is not a traditional closest point between segments test.
            //Completely ignore the pair if the closest points turn out to be beyond the intervals of the segments.

            //Use the offsets found from each test.
            //Test the A to B offset against the MTD, which is also known to be oriented in a certain way.
            //That known directionality allows easy computation of depth using MTD dot offset.
            //Do not use any contacts which have negative depth/positive distance.


            //Put the minimum translation direction into the local space of each object.
            Vector3 mtdA, mtdB;
            Vector3 negatedMtd;
            Vector3.Negate(ref mtd, out negatedMtd);
            Matrix3x3.TransformTranspose(ref negatedMtd, ref orientationA, out mtdA);
            Matrix3x3.TransformTranspose(ref mtd, ref orientationB, out mtdB);


#if !WINDOWS
            Vector3 edgeAStart1 = new Vector3(), edgeAEnd1 = new Vector3(), edgeAStart2 = new Vector3(), edgeAEnd2 = new Vector3();
            Vector3 edgeBStart1 = new Vector3(), edgeBEnd1 = new Vector3(), edgeBStart2 = new Vector3(), edgeBEnd2 = new Vector3();
#else
            Vector3 edgeAStart1, edgeAEnd1, edgeAStart2, edgeAEnd2;
            Vector3 edgeBStart1, edgeBEnd1, edgeBStart2, edgeBEnd2;
#endif
            Fix64 aHalfWidth = a.halfWidth;
            Fix64 aHalfHeight = a.halfHeight;
            Fix64 aHalfLength = a.halfLength;

            Fix64 bHalfWidth = b.halfWidth;
            Fix64 bHalfHeight = b.halfHeight;
            Fix64 bHalfLength = b.halfLength;

            //Letter stands for owner.  Number stands for edge (1 or 2).
            int edgeAStart1Id, edgeAEnd1Id, edgeAStart2Id, edgeAEnd2Id;
            int edgeBStart1Id, edgeBEnd1Id, edgeBStart2Id, edgeBEnd2Id;

            //This is an edge-edge collision, so one (AND ONLY ONE) of the components in the 
            //local direction must be very close to zero.  We can use an arbitrary fixed 
            //epsilon because the mtd is always unit length.

            #region Edge A

            if (Fix64.Abs(mtdA.X) < Toolbox.Epsilon)
            {
                //mtd is in the Y-Z plane.
                //Perform an implicit dot with the edge location relative to the center.
                //Find the two edges furthest in the direction of the mtdA.
                var dots = new TinyList<Fix64>();
                dots.Add(-aHalfHeight * mtdA.Y - aHalfLength * mtdA.Z);
                dots.Add(-aHalfHeight * mtdA.Y + aHalfLength * mtdA.Z);
                dots.Add(aHalfHeight * mtdA.Y - aHalfLength * mtdA.Z);
                dots.Add(aHalfHeight * mtdA.Y + aHalfLength * mtdA.Z);

                //Find the first and second highest indices.
                int highestIndex, secondHighestIndex;
                FindHighestIndices(ref dots, out highestIndex, out secondHighestIndex);
                //Use the indices to compute the edges.
                GetEdgeData(highestIndex, 0, aHalfWidth, aHalfHeight, aHalfLength, out edgeAStart1, out edgeAEnd1, out edgeAStart1Id, out edgeAEnd1Id);
                GetEdgeData(secondHighestIndex, 0, aHalfWidth, aHalfHeight, aHalfLength, out edgeAStart2, out edgeAEnd2, out edgeAStart2Id, out edgeAEnd2Id);


            }
            else if (Fix64.Abs(mtdA.Y) < Toolbox.Epsilon)
            {
                //mtd is in the X-Z plane
                //Perform an implicit dot with the edge location relative to the center.
                //Find the two edges furthest in the direction of the mtdA.
                var dots = new TinyList<Fix64>();
                dots.Add(-aHalfWidth * mtdA.X - aHalfLength * mtdA.Z);
                dots.Add(-aHalfWidth * mtdA.X + aHalfLength * mtdA.Z);
                dots.Add(aHalfWidth * mtdA.X - aHalfLength * mtdA.Z);
                dots.Add(aHalfWidth * mtdA.X + aHalfLength * mtdA.Z);

                //Find the first and second highest indices.
                int highestIndex, secondHighestIndex;
                FindHighestIndices(ref dots, out highestIndex, out secondHighestIndex);
                //Use the indices to compute the edges.
                GetEdgeData(highestIndex, 1, aHalfWidth, aHalfHeight, aHalfLength, out edgeAStart1, out edgeAEnd1, out edgeAStart1Id, out edgeAEnd1Id);
                GetEdgeData(secondHighestIndex, 1, aHalfWidth, aHalfHeight, aHalfLength, out edgeAStart2, out edgeAEnd2, out edgeAStart2Id, out edgeAEnd2Id);
            }
            else
            {
                //mtd is in the X-Y plane
                //Perform an implicit dot with the edge location relative to the center.
                //Find the two edges furthest in the direction of the mtdA.
                var dots = new TinyList<Fix64>();
                dots.Add(-aHalfWidth * mtdA.X - aHalfHeight * mtdA.Y);
                dots.Add(-aHalfWidth * mtdA.X + aHalfHeight * mtdA.Y);
                dots.Add(aHalfWidth * mtdA.X - aHalfHeight * mtdA.Y);
                dots.Add(aHalfWidth * mtdA.X + aHalfHeight * mtdA.Y);

                //Find the first and second highest indices.
                int highestIndex, secondHighestIndex;
                FindHighestIndices(ref dots, out highestIndex, out secondHighestIndex);
                //Use the indices to compute the edges.
                GetEdgeData(highestIndex, 2, aHalfWidth, aHalfHeight, aHalfLength, out edgeAStart1, out edgeAEnd1, out edgeAStart1Id, out edgeAEnd1Id);
                GetEdgeData(secondHighestIndex, 2, aHalfWidth, aHalfHeight, aHalfLength, out edgeAStart2, out edgeAEnd2, out edgeAStart2Id, out edgeAEnd2Id);
            }

            #endregion

            #region Edge B

            if (Fix64.Abs(mtdB.X) < Toolbox.Epsilon)
            {
                //mtd is in the Y-Z plane.
                //Perform an implicit dot with the edge location relative to the center.
                //Find the two edges furthest in the direction of the mtdB.
                var dots = new TinyList<Fix64>();
                dots.Add(-bHalfHeight * mtdB.Y - bHalfLength * mtdB.Z);
                dots.Add(-bHalfHeight * mtdB.Y + bHalfLength * mtdB.Z);
                dots.Add(bHalfHeight * mtdB.Y - bHalfLength * mtdB.Z);
                dots.Add(bHalfHeight * mtdB.Y + bHalfLength * mtdB.Z);

                //Find the first and second highest indices.
                int highestIndex, secondHighestIndex;
                FindHighestIndices(ref dots, out highestIndex, out secondHighestIndex);
                //Use the indices to compute the edges.
                GetEdgeData(highestIndex, 0, bHalfWidth, bHalfHeight, bHalfLength, out edgeBStart1, out edgeBEnd1, out edgeBStart1Id, out edgeBEnd1Id);
                GetEdgeData(secondHighestIndex, 0, bHalfWidth, bHalfHeight, bHalfLength, out edgeBStart2, out edgeBEnd2, out edgeBStart2Id, out edgeBEnd2Id);


            }
            else if (Fix64.Abs(mtdB.Y) < Toolbox.Epsilon)
            {
                //mtd is in the X-Z plane
                //Perform an implicit dot with the edge location relative to the center.
                //Find the two edges furthest in the direction of the mtdB.
                var dots = new TinyList<Fix64>();
                dots.Add(-bHalfWidth * mtdB.X - bHalfLength * mtdB.Z);
                dots.Add(-bHalfWidth * mtdB.X + bHalfLength * mtdB.Z);
                dots.Add(bHalfWidth * mtdB.X - bHalfLength * mtdB.Z);
                dots.Add(bHalfWidth * mtdB.X + bHalfLength * mtdB.Z);

                //Find the first and second highest indices.
                int highestIndex, secondHighestIndex;
                FindHighestIndices(ref dots, out highestIndex, out secondHighestIndex);
                //Use the indices to compute the edges.
                GetEdgeData(highestIndex, 1, bHalfWidth, bHalfHeight, bHalfLength, out edgeBStart1, out edgeBEnd1, out edgeBStart1Id, out edgeBEnd1Id);
                GetEdgeData(secondHighestIndex, 1, bHalfWidth, bHalfHeight, bHalfLength, out edgeBStart2, out edgeBEnd2, out edgeBStart2Id, out edgeBEnd2Id);
            }
            else
            {
                //mtd is in the X-Y plane
                //Perform an implicit dot with the edge location relative to the center.
                //Find the two edges furthest in the direction of the mtdB.
                var dots = new TinyList<Fix64>();
                dots.Add(-bHalfWidth * mtdB.X - bHalfHeight * mtdB.Y);
                dots.Add(-bHalfWidth * mtdB.X + bHalfHeight * mtdB.Y);
                dots.Add(bHalfWidth * mtdB.X - bHalfHeight * mtdB.Y);
                dots.Add(bHalfWidth * mtdB.X + bHalfHeight * mtdB.Y);

                //Find the first and second highest indices.
                int highestIndex, secondHighestIndex;
                FindHighestIndices(ref dots, out highestIndex, out secondHighestIndex);
                //Use the indices to compute the edges.
                GetEdgeData(highestIndex, 2, bHalfWidth, bHalfHeight, bHalfLength, out edgeBStart1, out edgeBEnd1, out edgeBStart1Id, out edgeBEnd1Id);
                GetEdgeData(secondHighestIndex, 2, bHalfWidth, bHalfHeight, bHalfLength, out edgeBStart2, out edgeBEnd2, out edgeBStart2Id, out edgeBEnd2Id);
            }

            #endregion


            Matrix3x3.Transform(ref edgeAStart1, ref orientationA, out edgeAStart1);
            Matrix3x3.Transform(ref edgeAEnd1, ref orientationA, out edgeAEnd1);
            Matrix3x3.Transform(ref edgeBStart1, ref orientationB, out edgeBStart1);
            Matrix3x3.Transform(ref edgeBEnd1, ref orientationB, out edgeBEnd1);

            Matrix3x3.Transform(ref edgeAStart2, ref orientationA, out edgeAStart2);
            Matrix3x3.Transform(ref edgeAEnd2, ref orientationA, out edgeAEnd2);
            Matrix3x3.Transform(ref edgeBStart2, ref orientationB, out edgeBStart2);
            Matrix3x3.Transform(ref edgeBEnd2, ref orientationB, out edgeBEnd2);

            Vector3.Add(ref edgeAStart1, ref positionA, out edgeAStart1);
            Vector3.Add(ref edgeAEnd1, ref positionA, out edgeAEnd1);
            Vector3.Add(ref edgeBStart1, ref positionB, out edgeBStart1);
            Vector3.Add(ref edgeBEnd1, ref positionB, out edgeBEnd1);

            Vector3.Add(ref edgeAStart2, ref positionA, out edgeAStart2);
            Vector3.Add(ref edgeAEnd2, ref positionA, out edgeAEnd2);
            Vector3.Add(ref edgeBStart2, ref positionB, out edgeBStart2);
            Vector3.Add(ref edgeBEnd2, ref positionB, out edgeBEnd2);

            Vector3 onA, onB;
            Vector3 offset;
            Fix64 dot;
#if ALLOWUNSAFE
            var tempContactData = new BoxContactDataCache();
            unsafe
            {
                var contactDataPointer = &tempContactData.D1;
#else
            contactData = new TinyStructList<BoxContactData>();
#endif

            //Go through the pairs and add any contacts with positive depth that are within the segments' intervals.

            if (GetClosestPointsBetweenSegments(ref edgeAStart1, ref edgeAEnd1, ref edgeBStart1, ref edgeBEnd1, out onA, out onB))
            {
                Vector3.Subtract(ref onA, ref onB, out offset);
                Vector3.Dot(ref offset, ref mtd, out dot);
                if (dot < F64.C0) //Distance must be negative.
                {
                    BoxContactData data;
                    data.Position = onA;
                    data.Depth = dot;
                    data.Id = GetContactId(edgeAStart1Id, edgeAEnd1Id, edgeBStart1Id, edgeBEnd1Id);
#if ALLOWUNSAFE
                        contactDataPointer[tempContactData.Count] = data;
                        tempContactData.Count++;
#else
                    contactData.Add(ref data);
#endif
                }

            }
            if (GetClosestPointsBetweenSegments(ref edgeAStart1, ref edgeAEnd1, ref edgeBStart2, ref edgeBEnd2, out onA, out onB))
            {
                Vector3.Subtract(ref onA, ref onB, out offset);
                Vector3.Dot(ref offset, ref mtd, out dot);
                if (dot < F64.C0) //Distance must be negative.
                {
                    BoxContactData data;
                    data.Position = onA;
                    data.Depth = dot;
                    data.Id = GetContactId(edgeAStart1Id, edgeAEnd1Id, edgeBStart2Id, edgeBEnd2Id);
#if ALLOWUNSAFE
                        contactDataPointer[tempContactData.Count] = data;
                        tempContactData.Count++;
#else
                    contactData.Add(ref data);
#endif
                }

            }
            if (GetClosestPointsBetweenSegments(ref edgeAStart2, ref edgeAEnd2, ref edgeBStart1, ref edgeBEnd1, out onA, out onB))
            {
                Vector3.Subtract(ref onA, ref onB, out offset);
                Vector3.Dot(ref offset, ref mtd, out dot);
                if (dot < F64.C0) //Distance must be negative.
                {
                    BoxContactData data;
                    data.Position = onA;
                    data.Depth = dot;
                    data.Id = GetContactId(edgeAStart2Id, edgeAEnd2Id, edgeBStart1Id, edgeBEnd1Id);
#if ALLOWUNSAFE
                        contactDataPointer[tempContactData.Count] = data;
                        tempContactData.Count++;
#else
                    contactData.Add(ref data);
#endif
                }

            }
            if (GetClosestPointsBetweenSegments(ref edgeAStart2, ref edgeAEnd2, ref edgeBStart2, ref edgeBEnd2, out onA, out onB))
            {
                Vector3.Subtract(ref onA, ref onB, out offset);
                Vector3.Dot(ref offset, ref mtd, out dot);
                if (dot < F64.C0) //Distance must be negative.
                {
                    BoxContactData data;
                    data.Position = onA;
                    data.Depth = dot;
                    data.Id = GetContactId(edgeAStart2Id, edgeAEnd2Id, edgeBStart2Id, edgeBEnd2Id);
#if ALLOWUNSAFE
                        contactDataPointer[tempContactData.Count] = data;
                        tempContactData.Count++;
#else
                    contactData.Add(ref data);
#endif
                }

            }
#if ALLOWUNSAFE
            }
            contactData = tempContactData;
#endif

        }

        private static void GetEdgeData(int index, int axis, Fix64 x, Fix64 y, Fix64 z, out Vector3 edgeStart, out Vector3 edgeEnd, out int edgeStartId, out int edgeEndId)
        {
            //Index defines which edge to use.
            //They follow this pattern:
            //0: --
            //1: -+
            //2: +-
            //3: ++

            //The axis index determines the dimensions to use.
            //0: plane with normal X
            //1: plane with normal Y
            //2: plane with normal Z

#if !WINDOWS
            edgeStart = new Vector3();
            edgeEnd = new Vector3();
#endif

            switch (index + axis * 4)
            {
                case 0:
                    //X--
                    edgeStart.X = -x;
                    edgeStart.Y = -y;
                    edgeStart.Z = -z;
                    edgeStartId = 0; //000

                    edgeEnd.X = x;
                    edgeEnd.Y = -y;
                    edgeEnd.Z = -z;
                    edgeEndId = 4; //100
                    break;
                case 1:
                    //X-+
                    edgeStart.X = -x;
                    edgeStart.Y = -y;
                    edgeStart.Z = z;
                    edgeStartId = 1; //001

                    edgeEnd.X = x;
                    edgeEnd.Y = -y;
                    edgeEnd.Z = z;
                    edgeEndId = 5; //101
                    break;
                case 2:
                    //X+-
                    edgeStart.X = -x;
                    edgeStart.Y = y;
                    edgeStart.Z = -z;
                    edgeStartId = 2; //010

                    edgeEnd.X = x;
                    edgeEnd.Y = y;
                    edgeEnd.Z = -z;
                    edgeEndId = 6; //110
                    break;
                case 3:
                    //X++
                    edgeStart.X = -x;
                    edgeStart.Y = y;
                    edgeStart.Z = z;
                    edgeStartId = 3; //011

                    edgeEnd.X = x;
                    edgeEnd.Y = y;
                    edgeEnd.Z = z;
                    edgeEndId = 7; //111
                    break;
                case 4:
                    //-Y-
                    edgeStart.X = -x;
                    edgeStart.Y = -y;
                    edgeStart.Z = -z;
                    edgeStartId = 0; //000

                    edgeEnd.X = -x;
                    edgeEnd.Y = y;
                    edgeEnd.Z = -z;
                    edgeEndId = 2; //010
                    break;
                case 5:
                    //-Y+
                    edgeStart.X = -x;
                    edgeStart.Y = -y;
                    edgeStart.Z = z;
                    edgeStartId = 1; //001

                    edgeEnd.X = -x;
                    edgeEnd.Y = y;
                    edgeEnd.Z = z;
                    edgeEndId = 3; //011
                    break;
                case 6:
                    //+Y-
                    edgeStart.X = x;
                    edgeStart.Y = -y;
                    edgeStart.Z = -z;
                    edgeStartId = 4; //100

                    edgeEnd.X = x;
                    edgeEnd.Y = y;
                    edgeEnd.Z = -z;
                    edgeEndId = 6; //110
                    break;
                case 7:
                    //+Y+
                    edgeStart.X = x;
                    edgeStart.Y = -y;
                    edgeStart.Z = z;
                    edgeStartId = 5; //101

                    edgeEnd.X = x;
                    edgeEnd.Y = y;
                    edgeEnd.Z = z;
                    edgeEndId = 7; //111
                    break;
                case 8:
                    //--Z
                    edgeStart.X = -x;
                    edgeStart.Y = -y;
                    edgeStart.Z = -z;
                    edgeStartId = 0; //000

                    edgeEnd.X = -x;
                    edgeEnd.Y = -y;
                    edgeEnd.Z = z;
                    edgeEndId = 1; //001
                    break;
                case 9:
                    //-+Z
                    edgeStart.X = -x;
                    edgeStart.Y = y;
                    edgeStart.Z = -z;
                    edgeStartId = 2; //010

                    edgeEnd.X = -x;
                    edgeEnd.Y = y;
                    edgeEnd.Z = z;
                    edgeEndId = 3; //011
                    break;
                case 10:
                    //+-Z
                    edgeStart.X = x;
                    edgeStart.Y = -y;
                    edgeStart.Z = -z;
                    edgeStartId = 4; //100

                    edgeEnd.X = x;
                    edgeEnd.Y = -y;
                    edgeEnd.Z = z;
                    edgeEndId = 5; //101
                    break;
                case 11:
                    //++Z
                    edgeStart.X = x;
                    edgeStart.Y = y;
                    edgeStart.Z = -z;
                    edgeStartId = 6; //110

                    edgeEnd.X = x;
                    edgeEnd.Y = y;
                    edgeEnd.Z = z;
                    edgeEndId = 7; //111
                    break;
                default:
                    throw new ArgumentException("Invalid index or axis.");
            }
        }

        static void FindHighestIndices(ref TinyList<Fix64> dots, out int highestIndex, out int secondHighestIndex)
        {
            highestIndex = 0;
            Fix64 highestValue = dots[0];
            for (int i = 1; i < 4; i++)
            {
                Fix64 dot = dots[i];
                if (dot > highestValue)
                {
                    highestIndex = i;
                    highestValue = dot;
                }
            }
            secondHighestIndex = 0;
            Fix64 secondHighestValue = -Fix64.MaxValue;
            for (int i = 0; i < 4; i++)
            {
                Fix64 dot = dots[i];
                if (i != highestIndex && dot > secondHighestValue)
                {
                    secondHighestIndex = i;
                    secondHighestValue = dot;
                }
            }
        }

        /// <summary>
        /// Computes closest points c1 and c2 betwen segments p1q1 and p2q2.
        /// </summary>
        /// <param name="p1">First point of first segment.</param>
        /// <param name="q1">Second point of first segment.</param>
        /// <param name="p2">First point of second segment.</param>
        /// <param name="q2">Second point of second segment.</param>
        /// <param name="c1">Closest point on first segment.</param>
        /// <param name="c2">Closest point on second segment.</param>
        static bool GetClosestPointsBetweenSegments(ref Vector3 p1, ref Vector3 q1, ref Vector3 p2, ref Vector3 q2,
                                                           out Vector3 c1, out Vector3 c2)
        {
            //Segment direction vectors
            Vector3 d1;
            Vector3.Subtract(ref q1, ref p1, out d1);
            Vector3 d2;
            Vector3.Subtract(ref q2, ref p2, out d2);
            Vector3 r;
            Vector3.Subtract(ref p1, ref p2, out r);
            //distance
            Fix64 a = d1.LengthSquared();
            Fix64 e = d2.LengthSquared();
            Fix64 f;
            Vector3.Dot(ref d2, ref r, out f);

            Fix64 s, t;

            if (a <= Toolbox.Epsilon && e <= Toolbox.Epsilon)
            {
                //These segments are more like points.
                c1 = p1;
                c2 = p2;
                return false;
            }
            if (a <= Toolbox.Epsilon)
            {
                // First segment is basically a point.
                s = F64.C0;
                t = f / e;
                if (t < F64.C0 || t > F64.C1)
                {
                    c1 = new Vector3();
                    c2 = new Vector3();
                    return false;
                }
            }
            else
            {
                Fix64 c = Vector3.Dot(d1, r);
                if (e <= Toolbox.Epsilon)
                {
                    // Second segment is basically a point.
                    t = F64.C0;
                    s = MathHelper.Clamp(-c / a, F64.C0, F64.C1);
                }
                else
                {
                    Fix64 b = Vector3.Dot(d1, d2);
                    Fix64 denom = a * e - b * b;

                    // If segments not parallel, compute closest point on L1 to L2, and
                    // clamp to segment S1. Else pick some s (here .5f)
                    if (denom != F64.C0)
                    {
                        s = (b * f - c * e) / denom;
                        if (s < F64.C0 || s > F64.C1)
                        {
                            //Closest point would be outside of the segment.
                            c1 = new Vector3();
                            c2 = new Vector3();
                            return false;
                        }
                    }
                    else //Parallel, just use .5f
                        s = F64.C0p5;


                    t = (b * s + f) / e;

                    if (t < F64.C0 || t > F64.C1)
                    {
                        //Closest point would be outside of the segment.
                        c1 = new Vector3();
                        c2 = new Vector3();
                        return false;
                    }
                }
            }

            Vector3.Multiply(ref d1, s, out c1);
            Vector3.Add(ref c1, ref p1, out c1);
            Vector3.Multiply(ref d2, t, out c2);
            Vector3.Add(ref c2, ref p2, out c2);
            return true;
        }

        //        internal static void GetEdgeEdgeContact(BoxShape a, BoxShape b, ref Vector3 positionA, ref Matrix3X3 orientationA, ref Vector3 positionB, ref Matrix3X3 orientationB, Fix64 depth, ref Vector3 mtd, out TinyStructList<BoxContactData> contactData)
        //        {
        //            //Put the minimum translation direction into the local space of each object.
        //            Vector3 mtdA, mtdB;
        //            Vector3 negatedMtd;
        //            Vector3.Negate(ref mtd, out negatedMtd);
        //            Matrix3X3.TransformTranspose(ref negatedMtd, ref orientationA, out mtdA);
        //            Matrix3X3.TransformTranspose(ref mtd, ref orientationB, out mtdB);


        //#if !WINDOWS
        //            Vector3 edgeA1 = new Vector3(), edgeA2 = new Vector3();
        //            Vector3 edgeB1 = new Vector3(), edgeB2 = new Vector3();
        //#else
        //            Vector3 edgeA1, edgeA2;
        //            Vector3 edgeB1, edgeB2;
        //#endif
        //            Fix64 aHalfWidth = a.halfWidth;
        //            Fix64 aHalfHeight = a.halfHeight;
        //            Fix64 aHalfLength = a.halfLength;

        //            Fix64 bHalfWidth = b.halfWidth;
        //            Fix64 bHalfHeight = b.halfHeight;
        //            Fix64 bHalfLength = b.halfLength;

        //            int edgeA1Id, edgeA2Id;
        //            int edgeB1Id, edgeB2Id;

        //            //This is an edge-edge collision, so one (AND ONLY ONE) of the components in the 
        //            //local direction must be very close to zero.  We can use an arbitrary fixed 
        //            //epsilon because the mtd is always unit length.

        //            #region Edge A

        //            if (Fix64.Abs(mtdA.X) < Toolbox.Epsilon)
        //            {
        //                //mtd is in the Y-Z plane.
        //                if (mtdA.Y > 0)
        //                {
        //                    if (mtdA.Z > 0)
        //                    {
        //                        //++
        //                        edgeA1.X = -aHalfWidth;
        //                        edgeA1.Y = aHalfHeight;
        //                        edgeA1.Z = aHalfLength;

        //                        edgeA2.X = aHalfWidth;
        //                        edgeA2.Y = aHalfHeight;
        //                        edgeA2.Z = aHalfLength;

        //                        edgeA1Id = 6;
        //                        edgeA2Id = 7;
        //                    }
        //                    else
        //                    {
        //                        //+-
        //                        edgeA1.X = -aHalfWidth;
        //                        edgeA1.Y = aHalfHeight;
        //                        edgeA1.Z = -aHalfLength;

        //                        edgeA2.X = aHalfWidth;
        //                        edgeA2.Y = aHalfHeight;
        //                        edgeA2.Z = -aHalfLength;

        //                        edgeA1Id = 2;
        //                        edgeA2Id = 3;
        //                    }
        //                }
        //                else
        //                {
        //                    if (mtdA.Z > 0)
        //                    {
        //                        //-+
        //                        edgeA1.X = -aHalfWidth;
        //                        edgeA1.Y = -aHalfHeight;
        //                        edgeA1.Z = aHalfLength;

        //                        edgeA2.X = aHalfWidth;
        //                        edgeA2.Y = -aHalfHeight;
        //                        edgeA2.Z = aHalfLength;

        //                        edgeA1Id = 4;
        //                        edgeA2Id = 5;
        //                    }
        //                    else
        //                    {
        //                        //--
        //                        edgeA1.X = -aHalfWidth;
        //                        edgeA1.Y = -aHalfHeight;
        //                        edgeA1.Z = -aHalfLength;

        //                        edgeA2.X = aHalfWidth;
        //                        edgeA2.Y = -aHalfHeight;
        //                        edgeA2.Z = -aHalfLength;

        //                        edgeA1Id = 0;
        //                        edgeA2Id = 1;
        //                    }
        //                }
        //            }
        //            else if (Fix64.Abs(mtdA.Y) < Toolbox.Epsilon)
        //            {
        //                //mtd is in the X-Z plane
        //                if (mtdA.X > 0)
        //                {
        //                    if (mtdA.Z > 0)
        //                    {
        //                        //++
        //                        edgeA1.X = aHalfWidth;
        //                        edgeA1.Y = -aHalfHeight;
        //                        edgeA1.Z = aHalfLength;

        //                        edgeA2.X = aHalfWidth;
        //                        edgeA2.Y = aHalfHeight;
        //                        edgeA2.Z = aHalfLength;

        //                        edgeA1Id = 5;
        //                        edgeA2Id = 7;
        //                    }
        //                    else
        //                    {
        //                        //+-
        //                        edgeA1.X = aHalfWidth;
        //                        edgeA1.Y = -aHalfHeight;
        //                        edgeA1.Z = -aHalfLength;

        //                        edgeA2.X = aHalfWidth;
        //                        edgeA2.Y = aHalfHeight;
        //                        edgeA2.Z = -aHalfLength;

        //                        edgeA1Id = 1;
        //                        edgeA2Id = 3;
        //                    }
        //                }
        //                else
        //                {
        //                    if (mtdA.Z > 0)
        //                    {
        //                        //-+
        //                        edgeA1.X = -aHalfWidth;
        //                        edgeA1.Y = -aHalfHeight;
        //                        edgeA1.Z = aHalfLength;

        //                        edgeA2.X = -aHalfWidth;
        //                        edgeA2.Y = aHalfHeight;
        //                        edgeA2.Z = aHalfLength;

        //                        edgeA1Id = 4;
        //                        edgeA2Id = 6;
        //                    }
        //                    else
        //                    {
        //                        //--
        //                        edgeA1.X = -aHalfWidth;
        //                        edgeA1.Y = -aHalfHeight;
        //                        edgeA1.Z = -aHalfLength;

        //                        edgeA2.X = -aHalfWidth;
        //                        edgeA2.Y = aHalfHeight;
        //                        edgeA2.Z = -aHalfLength;

        //                        edgeA1Id = 0;
        //                        edgeA2Id = 2;
        //                    }
        //                }
        //            }
        //            else
        //            {
        //                //mtd is in the X-Y plane
        //                if (mtdA.X > 0)
        //                {
        //                    if (mtdA.Y > 0)
        //                    {
        //                        //++
        //                        edgeA1.X = aHalfWidth;
        //                        edgeA1.Y = aHalfHeight;
        //                        edgeA1.Z = -aHalfLength;

        //                        edgeA2.X = aHalfWidth;
        //                        edgeA2.Y = aHalfHeight;
        //                        edgeA2.Z = aHalfLength;

        //                        edgeA1Id = 3;
        //                        edgeA2Id = 7;
        //                    }
        //                    else
        //                    {
        //                        //+-
        //                        edgeA1.X = aHalfWidth;
        //                        edgeA1.Y = -aHalfHeight;
        //                        edgeA1.Z = -aHalfLength;

        //                        edgeA2.X = aHalfWidth;
        //                        edgeA2.Y = -aHalfHeight;
        //                        edgeA2.Z = aHalfLength;

        //                        edgeA1Id = 1;
        //                        edgeA2Id = 5;
        //                    }
        //                }
        //                else
        //                {
        //                    if (mtdA.Y > 0)
        //                    {
        //                        //-+
        //                        edgeA1.X = -aHalfWidth;
        //                        edgeA1.Y = aHalfHeight;
        //                        edgeA1.Z = -aHalfLength;

        //                        edgeA2.X = -aHalfWidth;
        //                        edgeA2.Y = aHalfHeight;
        //                        edgeA2.Z = aHalfLength;

        //                        edgeA1Id = 2;
        //                        edgeA2Id = 6;
        //                    }
        //                    else
        //                    {
        //                        //--
        //                        edgeA1.X = -aHalfWidth;
        //                        edgeA1.Y = -aHalfHeight;
        //                        edgeA1.Z = -aHalfLength;

        //                        edgeA2.X = -aHalfWidth;
        //                        edgeA2.Y = -aHalfHeight;
        //                        edgeA2.Z = aHalfLength;

        //                        edgeA1Id = 0;
        //                        edgeA2Id = 4;
        //                    }
        //                }
        //            }

        //            #endregion

        //            #region Edge B

        //            if (Fix64.Abs(mtdB.X) < Toolbox.Epsilon)
        //            {
        //                //mtd is in the Y-Z plane.
        //                if (mtdB.Y > 0)
        //                {
        //                    if (mtdB.Z > 0)
        //                    {
        //                        //++
        //                        edgeB1.X = -bHalfWidth;
        //                        edgeB1.Y = bHalfHeight;
        //                        edgeB1.Z = bHalfLength;

        //                        edgeB2.X = bHalfWidth;
        //                        edgeB2.Y = bHalfHeight;
        //                        edgeB2.Z = bHalfLength;

        //                        edgeB1Id = 6;
        //                        edgeB2Id = 7;
        //                    }
        //                    else
        //                    {
        //                        //+-
        //                        edgeB1.X = -bHalfWidth;
        //                        edgeB1.Y = bHalfHeight;
        //                        edgeB1.Z = -bHalfLength;

        //                        edgeB2.X = bHalfWidth;
        //                        edgeB2.Y = bHalfHeight;
        //                        edgeB2.Z = -bHalfLength;

        //                        edgeB1Id = 2;
        //                        edgeB2Id = 3;
        //                    }
        //                }
        //                else
        //                {
        //                    if (mtdB.Z > 0)
        //                    {
        //                        //-+
        //                        edgeB1.X = -bHalfWidth;
        //                        edgeB1.Y = -bHalfHeight;
        //                        edgeB1.Z = bHalfLength;

        //                        edgeB2.X = bHalfWidth;
        //                        edgeB2.Y = -bHalfHeight;
        //                        edgeB2.Z = bHalfLength;

        //                        edgeB1Id = 4;
        //                        edgeB2Id = 5;
        //                    }
        //                    else
        //                    {
        //                        //--
        //                        edgeB1.X = -bHalfWidth;
        //                        edgeB1.Y = -bHalfHeight;
        //                        edgeB1.Z = -bHalfLength;

        //                        edgeB2.X = bHalfWidth;
        //                        edgeB2.Y = -bHalfHeight;
        //                        edgeB2.Z = -bHalfLength;

        //                        edgeB1Id = 0;
        //                        edgeB2Id = 1;
        //                    }
        //                }
        //            }
        //            else if (Fix64.Abs(mtdB.Y) < Toolbox.Epsilon)
        //            {
        //                //mtd is in the X-Z plane
        //                if (mtdB.X > 0)
        //                {
        //                    if (mtdB.Z > 0)
        //                    {
        //                        //++
        //                        edgeB1.X = bHalfWidth;
        //                        edgeB1.Y = -bHalfHeight;
        //                        edgeB1.Z = bHalfLength;

        //                        edgeB2.X = bHalfWidth;
        //                        edgeB2.Y = bHalfHeight;
        //                        edgeB2.Z = bHalfLength;

        //                        edgeB1Id = 5;
        //                        edgeB2Id = 7;
        //                    }
        //                    else
        //                    {
        //                        //+-
        //                        edgeB1.X = bHalfWidth;
        //                        edgeB1.Y = -bHalfHeight;
        //                        edgeB1.Z = -bHalfLength;

        //                        edgeB2.X = bHalfWidth;
        //                        edgeB2.Y = bHalfHeight;
        //                        edgeB2.Z = -bHalfLength;

        //                        edgeB1Id = 1;
        //                        edgeB2Id = 3;
        //                    }
        //                }
        //                else
        //                {
        //                    if (mtdB.Z > 0)
        //                    {
        //                        //-+
        //                        edgeB1.X = -bHalfWidth;
        //                        edgeB1.Y = -bHalfHeight;
        //                        edgeB1.Z = bHalfLength;

        //                        edgeB2.X = -bHalfWidth;
        //                        edgeB2.Y = bHalfHeight;
        //                        edgeB2.Z = bHalfLength;

        //                        edgeB1Id = 4;
        //                        edgeB2Id = 6;
        //                    }
        //                    else
        //                    {
        //                        //--
        //                        edgeB1.X = -bHalfWidth;
        //                        edgeB1.Y = -bHalfHeight;
        //                        edgeB1.Z = -bHalfLength;

        //                        edgeB2.X = -bHalfWidth;
        //                        edgeB2.Y = bHalfHeight;
        //                        edgeB2.Z = -bHalfLength;

        //                        edgeB1Id = 0;
        //                        edgeB2Id = 2;
        //                    }
        //                }
        //            }
        //            else
        //            {
        //                //mtd is in the X-Y plane
        //                if (mtdB.X > 0)
        //                {
        //                    if (mtdB.Y > 0)
        //                    {
        //                        //++
        //                        edgeB1.X = bHalfWidth;
        //                        edgeB1.Y = bHalfHeight;
        //                        edgeB1.Z = -bHalfLength;

        //                        edgeB2.X = bHalfWidth;
        //                        edgeB2.Y = bHalfHeight;
        //                        edgeB2.Z = bHalfLength;

        //                        edgeB1Id = 3;
        //                        edgeB2Id = 7;
        //                    }
        //                    else
        //                    {
        //                        //+-
        //                        edgeB1.X = bHalfWidth;
        //                        edgeB1.Y = -bHalfHeight;
        //                        edgeB1.Z = -bHalfLength;

        //                        edgeB2.X = bHalfWidth;
        //                        edgeB2.Y = -bHalfHeight;
        //                        edgeB2.Z = bHalfLength;

        //                        edgeB1Id = 1;
        //                        edgeB2Id = 5;
        //                    }
        //                }
        //                else
        //                {
        //                    if (mtdB.Y > 0)
        //                    {
        //                        //-+
        //                        edgeB1.X = -bHalfWidth;
        //                        edgeB1.Y = bHalfHeight;
        //                        edgeB1.Z = -bHalfLength;

        //                        edgeB2.X = -bHalfWidth;
        //                        edgeB2.Y = bHalfHeight;
        //                        edgeB2.Z = bHalfLength;

        //                        edgeB1Id = 2;
        //                        edgeB2Id = 6;
        //                    }
        //                    else
        //                    {
        //                        //--
        //                        edgeB1.X = -bHalfWidth;
        //                        edgeB1.Y = -bHalfHeight;
        //                        edgeB1.Z = -bHalfLength;

        //                        edgeB2.X = -bHalfWidth;
        //                        edgeB2.Y = -bHalfHeight;
        //                        edgeB2.Z = bHalfLength;

        //                        edgeB1Id = 0;
        //                        edgeB2Id = 4;
        //                    }
        //                }
        //            }

        //            #endregion

        //            //TODO: Since the above uniquely identifies the edge from each box based on two vertices,
        //            //get the edge feature id from vertexA id combined with vertexB id.
        //            //Vertex id's are 3 bit binary 'numbers' because ---, --+, -+-, etc.


        //            Matrix3X3.Transform(ref edgeA1, ref orientationA, out edgeA1);
        //            Matrix3X3.Transform(ref edgeA2, ref orientationA, out edgeA2);
        //            Matrix3X3.Transform(ref edgeB1, ref orientationB, out edgeB1);
        //            Matrix3X3.Transform(ref edgeB2, ref orientationB, out edgeB2);
        //            Vector3.Add(ref edgeA1, ref positionA, out edgeA1);
        //            Vector3.Add(ref edgeA2, ref positionA, out edgeA2);
        //            Vector3.Add(ref edgeB1, ref positionB, out edgeB1);
        //            Vector3.Add(ref edgeB2, ref positionB, out edgeB2);

        //            Fix64 s, t;
        //            Vector3 onA, onB;
        //            Toolbox.GetClosestPointsBetweenSegments(ref edgeA1, ref edgeA2, ref edgeB1, ref edgeB2, out s, out t, out onA, out onB);
        //            //Vector3.Add(ref onA, ref onB, out point);
        //            //Vector3.Multiply(ref point, .5f, out point);
        //            point = onA;

        //            //depth = (onB.X - onA.X) * mtd.X + (onB.Y - onA.Y) * mtd.Y + (onB.Z - onA.Z) * mtd.Z;

        //            id = GetContactId(edgeA1Id, edgeA2Id, edgeB1Id, edgeB2Id);
        //        }

#if ALLOWUNSAFE
        internal static void GetFaceContacts(BoxShape a, BoxShape b, ref Vector3 positionA, ref Matrix3x3 orientationA, ref Vector3 positionB, ref Matrix3x3 orientationB, bool aIsFaceOwner, ref Vector3 mtd, out BoxContactDataCache contactData)
#else
        internal static void GetFaceContacts(BoxShape a, BoxShape b, ref Vector3 positionA, ref Matrix3x3 orientationA, ref Vector3 positionB, ref Matrix3x3 orientationB, bool aIsFaceOwner, ref Vector3 mtd, out TinyStructList<BoxContactData> contactData)
#endif
        {
            Fix64 aHalfWidth = a.halfWidth;
            Fix64 aHalfHeight = a.halfHeight;
            Fix64 aHalfLength = a.halfLength;

            Fix64 bHalfWidth = b.halfWidth;
            Fix64 bHalfHeight = b.halfHeight;
            Fix64 bHalfLength = b.halfLength;

            BoxFace aBoxFace, bBoxFace;

            Vector3 negatedMtd;
            Vector3.Negate(ref mtd, out negatedMtd);
            GetNearestFace(ref positionA, ref orientationA, ref negatedMtd, aHalfWidth, aHalfHeight, aHalfLength, out aBoxFace);


            GetNearestFace(ref positionB, ref orientationB, ref mtd, bHalfWidth, bHalfHeight, bHalfLength, out bBoxFace);

            if (aIsFaceOwner)
                ClipFacesDirect(ref aBoxFace, ref bBoxFace, ref negatedMtd, out contactData);
            else
                ClipFacesDirect(ref bBoxFace, ref aBoxFace, ref mtd, out contactData);

            if (contactData.Count > 4)
                PruneContactsMaxDistance(ref mtd, contactData, out contactData);
        }

#if ALLOWUNSAFE
        private static unsafe void PruneContactsMaxDistance(ref Vector3 mtd, BoxContactDataCache input, out BoxContactDataCache output)
        {
            BoxContactData* data = &input.D1;
            int count = input.Count;
            //TODO: THE FOLLOWING has a small issue in release mode.
            //Find the deepest point.
            Fix64 deepestDepth = -1;
            int deepestIndex = 0;
            for (int i = 0; i < count; i++)
            {
                if (data[i].Depth > deepestDepth)
                {
                    deepestDepth = data[i].Depth;
                    deepestIndex = i;
                }
            }

            //Identify the furthest point away from the deepest index.
            Fix64 furthestDistance = -1;
            int furthestIndex = 0;
            for (int i = 0; i < count; i++)
            {
                Fix64 distance;
                Vector3.DistanceSquared(ref data[deepestIndex].Position, ref data[i].Position, out distance);
                if (distance > furthestDistance)
                {
                    furthestDistance = distance;
                    furthestIndex = i;
                }

            }

            Vector3 xAxis;
            Vector3.Subtract(ref data[furthestIndex].Position, ref data[deepestIndex].Position, out xAxis);

            Vector3 yAxis;
            Vector3.Cross(ref mtd, ref xAxis, out yAxis);

            Fix64 minY;
            Fix64 maxY;
            int minYindex = 0;
            int maxYindex = 0;

            Vector3.Dot(ref data[0].Position, ref yAxis, out minY);
            maxY = minY;
            for (int i = 1; i < count; i++)
            {
                Fix64 dot;
                Vector3.Dot(ref yAxis, ref data[i].Position, out dot);
                if (dot < minY)
                {
                    minY = dot;
                    minYindex = i;
                }
                else if (dot > maxY)
                {
                    maxY = dot;
                    maxYindex = i;
                }
            }

            output = new BoxContactDataCache
                         {
                             Count = 4,
                             D1 = data[deepestIndex],
                             D2 = data[furthestIndex],
                             D3 = data[minYindex],
                             D4 = data[maxYindex]
                         };


            //Vector3 v;
            //var maximumOffset = new Vector3();
            //int maxIndexA = -1, maxIndexB = -1;
            //Fix64 temp;
            //Fix64 maximumDistanceSquared = -Fix64.MaxValue;
            //for (int i = 0; i < count; i++)
            //{
            //    for (int j = i + 1; j < count; j++)
            //    {
            //        Vector3.Subtract(ref data[j].Position, ref data[i].Position, out v);
            //        temp = v.LengthSquared();
            //        if (temp > maximumDistanceSquared)
            //        {
            //            maximumDistanceSquared = temp;
            //            maxIndexA = i;
            //            maxIndexB = j;
            //            maximumOffset = v;
            //        }
            //    }
            //}

            //Vector3 otherDirection;
            //Vector3.Cross(ref mtd, ref maximumOffset, out otherDirection);
            //int minimumIndex = -1, maximumIndex = -1;
            //Fix64 minimumDistance = Fix64.MaxValue, maximumDistance = -Fix64.MaxValue;

            //for (int i = 0; i < count; i++)
            //{
            //    if (i != maxIndexA && i != maxIndexB)
            //    {
            //        Vector3.Dot(ref data[i].Position, ref otherDirection, out temp);
            //        if (temp > maximumDistance)
            //        {
            //            maximumDistance = temp;
            //            maximumIndex = i;
            //        }
            //        if (temp < minimumDistance)
            //        {
            //            minimumDistance = temp;
            //            minimumIndex = i;
            //        }
            //    }
            //}

            //output = new BoxContactDataCache();
            //output.Count = 4;
            //output.D1 = data[maxIndexA];
            //output.D2 = data[maxIndexB];
            //output.D3 = data[minimumIndex];
            //output.D4 = data[maximumIndex];
        }
#else
        private static void PruneContactsMaxDistance(ref Vector3 mtd, TinyStructList<BoxContactData> input, out TinyStructList<BoxContactData> output)
        {
            int count = input.Count;
            //Find the deepest point.
            BoxContactData data, deepestData;
            input.Get(0, out deepestData);
            for (int i = 1; i < count; i++)
            {
                input.Get(i, out data);
                if (data.Depth > deepestData.Depth)
                {
                    deepestData = data;
                }
            }

            //Identify the furthest point away from the deepest index.
            BoxContactData furthestData;
            input.Get(0, out furthestData);
            Fix64 furthestDistance;
            Vector3.DistanceSquared(ref deepestData.Position, ref furthestData.Position, out furthestDistance);
            for (int i = 1; i < count; i++)
            {
                input.Get(i, out data);
                Fix64 distance;
                Vector3.DistanceSquared(ref deepestData.Position, ref data.Position, out distance);
                if (distance > furthestDistance)
                {
                    furthestDistance = distance;
                    furthestData = data;
                }

            }

            Vector3 xAxis;
            Vector3.Subtract(ref furthestData.Position, ref deepestData.Position, out xAxis);

            Vector3 yAxis;
            Vector3.Cross(ref mtd, ref xAxis, out yAxis);

            Fix64 minY;
            Fix64 maxY;
            BoxContactData minData, maxData;
            input.Get(0, out minData);
            maxData = minData;

            Vector3.Dot(ref minData.Position, ref yAxis, out minY);
            maxY = minY;
            for (int i = 1; i < count; i++)
            {
                input.Get(i, out data);
                Fix64 dot;
                Vector3.Dot(ref yAxis, ref data.Position, out dot);
                if (dot < minY)
                {
                    minY = dot;
                    minData = data;
                }
                else if (dot > maxY)
                {
                    maxY = dot;
                    maxData = data;
                }
            }

            output = new TinyStructList<BoxContactData>();
            output.Add(ref deepestData);
            output.Add(ref furthestData);
            output.Add(ref minData);
            output.Add(ref maxData);


            //int count = input.Count;
            //Vector3 v;
            //var maximumOffset = new Vector3();
            //int maxIndexA = -1, maxIndexB = -1;
            //Fix64 temp;
            //Fix64 maximumDistanceSquared = -Fix64.MaxValue;
            //BoxContactData itemA, itemB;
            //for (int i = 0; i < count; i++)
            //{
            //    for (int j = i + 1; j < count; j++)
            //    {
            //        input.Get(j, out itemB);
            //        input.Get(i, out itemA);
            //        Vector3.Subtract(ref itemB.Position, ref itemA.Position, out v);
            //        temp = v.LengthSquared();
            //        if (temp > maximumDistanceSquared)
            //        {
            //            maximumDistanceSquared = temp;
            //            maxIndexA = i;
            //            maxIndexB = j;
            //            maximumOffset = v;
            //        }
            //    }
            //}

            //Vector3 otherDirection;
            //Vector3.Cross(ref mtd, ref maximumOffset, out otherDirection);
            //int minimumIndex = -1, maximumIndex = -1;
            //Fix64 minimumDistance = Fix64.MaxValue, maximumDistance = -Fix64.MaxValue;

            //for (int i = 0; i < count; i++)
            //{
            //    if (i != maxIndexA && i != maxIndexB)
            //    {
            //        input.Get(i, out itemA);
            //        Vector3.Dot(ref itemA.Position, ref otherDirection, out temp);
            //        if (temp > maximumDistance)
            //        {
            //            maximumDistance = temp;
            //            maximumIndex = i;
            //        }
            //        if (temp < minimumDistance)
            //        {
            //            minimumDistance = temp;
            //            minimumIndex = i;
            //        }
            //    }
            //}

            //output = new TinyStructList<BoxContactData>();
            //input.Get(maxIndexA, out itemA);
            //output.Add(ref itemA);
            //input.Get(maxIndexB, out itemA);
            //output.Add(ref itemA);
            //input.Get(minimumIndex, out itemA);
            //output.Add(ref itemA);
            //input.Get(maximumIndex, out itemA);
            //output.Add(ref itemA);
        }
#endif

#if ALLOWUNSAFE
        private static unsafe void ClipFacesDirect(ref BoxFace clipFace, ref BoxFace face, ref Vector3 mtd, out BoxContactDataCache outputData)
        {
            var contactData = new BoxContactDataCache();
            BoxContactDataCache tempData; //Local version.
            BoxContactData* data = &contactData.D1;
            BoxContactData* temp = &tempData.D1;

            //Local directions on the clip face.  Their length is equal to the length of an edge.
            Vector3 clipX, clipY;
            Vector3.Subtract(ref clipFace.V4, ref clipFace.V3, out clipX);
            Vector3.Subtract(ref clipFace.V2, ref clipFace.V3, out clipY);
            Fix64 inverseClipWidth = F64.C1 / clipFace.Width;
            Fix64 inverseClipHeight = F64.C1 / clipFace.Height;
            Fix64 inverseClipWidthSquared = inverseClipWidth * inverseClipWidth;
            clipX.X *= inverseClipWidthSquared;
            clipX.Y *= inverseClipWidthSquared;
            clipX.Z *= inverseClipWidthSquared;
            Fix64 inverseClipHeightSquared = inverseClipHeight * inverseClipHeight;
            clipY.X *= inverseClipHeightSquared;
            clipY.Y *= inverseClipHeightSquared;
            clipY.Z *= inverseClipHeightSquared;

            //Local directions on the opposing face.  Their length is equal to the length of an edge.
            Vector3 faceX, faceY;
            Vector3.Subtract(ref face.V4, ref face.V3, out faceX);
            Vector3.Subtract(ref face.V2, ref face.V3, out faceY);
            Fix64 inverseFaceWidth = F64.C1 / face.Width;
            Fix64 inverseFaceHeight = F64.C1 / face.Height;
            Fix64 inverseFaceWidthSquared = inverseFaceWidth * inverseFaceWidth;
            faceX.X *= inverseFaceWidthSquared;
            faceX.Y *= inverseFaceWidthSquared;
            faceX.Z *= inverseFaceWidthSquared;
            Fix64 inverseFaceHeightSquared = inverseFaceHeight * inverseFaceHeight;
            faceY.X *= inverseFaceHeightSquared;
            faceY.Y *= inverseFaceHeightSquared;
            faceY.Z *= inverseFaceHeightSquared;

            Vector3 clipCenter;
            Vector3.Add(ref clipFace.V1, ref clipFace.V3, out clipCenter);
            //Defer division until after dot product (2 multiplies instead of 3)
            Fix64 clipCenterX, clipCenterY;
            Vector3.Dot(ref clipCenter, ref clipX, out clipCenterX);
            Vector3.Dot(ref clipCenter, ref clipY, out clipCenterY);
            clipCenterX *= F64.C0p5;
            clipCenterY *= F64.C0p5;

            Vector3 faceCenter;
            Vector3.Add(ref face.V1, ref face.V3, out faceCenter);
            //Defer division until after dot product (2 multiplies instead of 3)
            Fix64 faceCenterX, faceCenterY;
            Vector3.Dot(ref faceCenter, ref faceX, out faceCenterX);
            Vector3.Dot(ref faceCenter, ref faceY, out faceCenterY);
            faceCenterX *= F64.C0p5;
            faceCenterY *= F64.C0p5;

            //To test bounds, recall that clipX is the length of the X edge.
            //Going from the center to the max or min goes half of the length of X edge, or +/- 0.5.
            //Bias could be added here.
            //const Fix64 extent = .5f; //.5f is the default, extra could be added for robustness or speed.
            Fix64 extentX = F64.C0p5 + F64.C0p01 * inverseClipWidth;
            Fix64 extentY = F64.C0p5 + F64.C0p01 * inverseClipHeight;
            //Fix64 extentX = .5f + .01f * inverseClipXLength;
            //Fix64 extentY = .5f + .01f * inverseClipYLength;
            Fix64 clipCenterMaxX = clipCenterX + extentX;
            Fix64 clipCenterMaxY = clipCenterY + extentY;
            Fix64 clipCenterMinX = clipCenterX - extentX;
            Fix64 clipCenterMinY = clipCenterY - extentY;

            extentX = F64.C0p5 + F64.C0p01 * inverseFaceWidth;
            extentY = F64.C0p5 + F64.C0p01 * inverseFaceHeight;
            //extentX = .5f + .01f * inverseFaceXLength;
            //extentY = .5f + .01f * inverseFaceYLength;
            Fix64 faceCenterMaxX = faceCenterX + extentX;
            Fix64 faceCenterMaxY = faceCenterY + extentY;
            Fix64 faceCenterMinX = faceCenterX - extentX;
            Fix64 faceCenterMinY = faceCenterY - extentY;

            //Find out where the opposing face is.
            Fix64 dotX, dotY;

            //The four edges can be thought of as minX, maxX, minY and maxY.

            //Face v1
            Vector3.Dot(ref clipX, ref face.V1, out dotX);
            bool v1MaxXInside = dotX < clipCenterMaxX;
            bool v1MinXInside = dotX > clipCenterMinX;
            Vector3.Dot(ref clipY, ref face.V1, out dotY);
            bool v1MaxYInside = dotY < clipCenterMaxY;
            bool v1MinYInside = dotY > clipCenterMinY;

            //Face v2
            Vector3.Dot(ref clipX, ref face.V2, out dotX);
            bool v2MaxXInside = dotX < clipCenterMaxX;
            bool v2MinXInside = dotX > clipCenterMinX;
            Vector3.Dot(ref clipY, ref face.V2, out dotY);
            bool v2MaxYInside = dotY < clipCenterMaxY;
            bool v2MinYInside = dotY > clipCenterMinY;

            //Face v3
            Vector3.Dot(ref clipX, ref face.V3, out dotX);
            bool v3MaxXInside = dotX < clipCenterMaxX;
            bool v3MinXInside = dotX > clipCenterMinX;
            Vector3.Dot(ref clipY, ref face.V3, out dotY);
            bool v3MaxYInside = dotY < clipCenterMaxY;
            bool v3MinYInside = dotY > clipCenterMinY;

            //Face v4
            Vector3.Dot(ref clipX, ref face.V4, out dotX);
            bool v4MaxXInside = dotX < clipCenterMaxX;
            bool v4MinXInside = dotX > clipCenterMinX;
            Vector3.Dot(ref clipY, ref face.V4, out dotY);
            bool v4MaxYInside = dotY < clipCenterMaxY;
            bool v4MinYInside = dotY > clipCenterMinY;

            //Find out where the clip face is.
            //Clip v1
            Vector3.Dot(ref faceX, ref clipFace.V1, out dotX);
            bool clipv1MaxXInside = dotX < faceCenterMaxX;
            bool clipv1MinXInside = dotX > faceCenterMinX;
            Vector3.Dot(ref faceY, ref clipFace.V1, out dotY);
            bool clipv1MaxYInside = dotY < faceCenterMaxY;
            bool clipv1MinYInside = dotY > faceCenterMinY;

            //Clip v2
            Vector3.Dot(ref faceX, ref clipFace.V2, out dotX);
            bool clipv2MaxXInside = dotX < faceCenterMaxX;
            bool clipv2MinXInside = dotX > faceCenterMinX;
            Vector3.Dot(ref faceY, ref clipFace.V2, out dotY);
            bool clipv2MaxYInside = dotY < faceCenterMaxY;
            bool clipv2MinYInside = dotY > faceCenterMinY;

            //Clip v3
            Vector3.Dot(ref faceX, ref clipFace.V3, out dotX);
            bool clipv3MaxXInside = dotX < faceCenterMaxX;
            bool clipv3MinXInside = dotX > faceCenterMinX;
            Vector3.Dot(ref faceY, ref clipFace.V3, out dotY);
            bool clipv3MaxYInside = dotY < faceCenterMaxY;
            bool clipv3MinYInside = dotY > faceCenterMinY;

            //Clip v4
            Vector3.Dot(ref faceX, ref clipFace.V4, out dotX);
            bool clipv4MaxXInside = dotX < faceCenterMaxX;
            bool clipv4MinXInside = dotX > faceCenterMinX;
            Vector3.Dot(ref faceY, ref clipFace.V4, out dotY);
            bool clipv4MaxYInside = dotY < faceCenterMaxY;
            bool clipv4MinYInside = dotY > faceCenterMinY;

        #region Face Vertices

            if (v1MinXInside && v1MaxXInside && v1MinYInside && v1MaxYInside)
            {
                data[contactData.Count].Position = face.V1;
                data[contactData.Count].Id = face.Id1;
                contactData.Count++;
            }

            if (v2MinXInside && v2MaxXInside && v2MinYInside && v2MaxYInside)
            {
                data[contactData.Count].Position = face.V2;
                data[contactData.Count].Id = face.Id2;
                contactData.Count++;
            }

            if (v3MinXInside && v3MaxXInside && v3MinYInside && v3MaxYInside)
            {
                data[contactData.Count].Position = face.V3;
                data[contactData.Count].Id = face.Id3;
                contactData.Count++;
            }

            if (v4MinXInside && v4MaxXInside && v4MinYInside && v4MaxYInside)
            {
                data[contactData.Count].Position = face.V4;
                data[contactData.Count].Id = face.Id4;
                contactData.Count++;
            }

            #endregion

            //Compute depths.
            tempData = contactData;
            contactData.Count = 0;
            Fix64 depth;
            Fix64 clipFaceDot, faceDot;
            Vector3.Dot(ref clipFace.V1, ref mtd, out clipFaceDot);
            for (int i = 0; i < tempData.Count; i++)
            {
                Vector3.Dot(ref temp[i].Position, ref mtd, out faceDot);
                depth = faceDot - clipFaceDot;
                if (depth <= F64.C0)
                {
                    data[contactData.Count].Position = temp[i].Position;
                    data[contactData.Count].Depth = depth;
                    data[contactData.Count].Id = temp[i].Id;
                    contactData.Count++;
                }
            }

            byte previousCount = contactData.Count;
            if (previousCount >= 4) //Early finish :)
            {
                outputData = contactData;
                return;
            }

        #region Clip face vertices

            Vector3 v;
            Fix64 a, b;
            Vector3.Dot(ref face.V1, ref face.Normal, out b);
            //CLIP FACE
            if (clipv1MinXInside && clipv1MaxXInside && clipv1MinYInside && clipv1MaxYInside)
            {
                Vector3.Dot(ref clipFace.V1, ref face.Normal, out a);
                Vector3.Multiply(ref face.Normal, a - b, out v);
                Vector3.Subtract(ref clipFace.V1, ref v, out v);
                data[contactData.Count].Position = v;
                data[contactData.Count].Id = clipFace.Id1 + 8;
                contactData.Count++;
            }

            if (clipv2MinXInside && clipv2MaxXInside && clipv2MinYInside && clipv2MaxYInside)
            {
                Vector3.Dot(ref clipFace.V2, ref face.Normal, out a);
                Vector3.Multiply(ref face.Normal, a - b, out v);
                Vector3.Subtract(ref clipFace.V2, ref v, out v);
                data[contactData.Count].Position = v;
                data[contactData.Count].Id = clipFace.Id2 + 8;
                contactData.Count++;
            }

            if (clipv3MinXInside && clipv3MaxXInside && clipv3MinYInside && clipv3MaxYInside)
            {
                Vector3.Dot(ref clipFace.V3, ref face.Normal, out a);
                Vector3.Multiply(ref face.Normal, a - b, out v);
                Vector3.Subtract(ref clipFace.V3, ref v, out v);
                data[contactData.Count].Position = v;
                data[contactData.Count].Id = clipFace.Id3 + 8;
                contactData.Count++;
            }

            if (clipv4MinXInside && clipv4MaxXInside && clipv4MinYInside && clipv4MaxYInside)
            {
                Vector3.Dot(ref clipFace.V4, ref face.Normal, out a);
                Vector3.Multiply(ref face.Normal, a - b, out v);
                Vector3.Subtract(ref clipFace.V4, ref v, out v);
                data[contactData.Count].Position = v;
                data[contactData.Count].Id = clipFace.Id4 + 8;
                contactData.Count++;
            }

            #endregion

            //Compute depths.
            tempData = contactData;
            contactData.Count = previousCount;

            for (int i = previousCount; i < tempData.Count; i++)
            {
                Vector3.Dot(ref temp[i].Position, ref mtd, out faceDot);
                depth = faceDot - clipFaceDot;
                if (depth <= F64.C0)
                {
                    data[contactData.Count].Position = temp[i].Position;
                    data[contactData.Count].Depth = depth;
                    data[contactData.Count].Id = temp[i].Id;
                    contactData.Count++;
                }
            }

            previousCount = contactData.Count;
            if (previousCount >= 4) //Early finish :)
            {
                outputData = contactData;
                return;
            }

            //Intersect edges.

            //maxX maxY -> v1
            //minX maxY -> v2
            //minX minY -> v3
            //maxX minY -> v4

            //Once we get here there can only be 3 contacts or less.
            //Once 4 possible contacts have been added, switch to using safe increments.
            //Fix64 dot;

        #region CLIP EDGE: v1 v2

            FaceEdge clipEdge;
            clipFace.GetEdge(0, out clipEdge);
            if (!v1MaxYInside)
            {
                if (v2MaxYInside)
                {
                    //ComputeIntersection(ref face.V1, ref face.V2, ref clipEdge, out v);
                    //Vector3.Dot(ref clipX, ref v, out dot);
                    //if (dot > clipCenterMinX && dot < clipCenterMaxX)
                    if (ComputeIntersection(ref face.V1, ref face.V2, ref clipEdge, out v))
                    {
                        data[contactData.Count].Position = v;
                        data[contactData.Count].Id = GetContactId(face.Id1, face.Id2, ref clipEdge);
                        contactData.Count++;
                    }
                }
                if (v4MaxYInside)
                {
                    //ComputeIntersection(ref face.V4, ref face.V1, ref clipEdge, out v);
                    //Vector3.Dot(ref clipX, ref v, out dot);
                    //if (dot > clipCenterMinX && dot < clipCenterMaxX)
                    if (ComputeIntersection(ref face.V4, ref face.V1, ref clipEdge, out v))
                    {
                        data[contactData.Count].Position = v;
                        data[contactData.Count].Id = GetContactId(face.Id4, face.Id1, ref clipEdge);
                        contactData.Count++;
                    }
                }
            }
            if (!v2MaxYInside)
            {
                if (v1MaxYInside)
                {
                    //ComputeIntersection(ref face.V1, ref face.V2, ref clipEdge, out v);
                    //Vector3.Dot(ref clipX, ref v, out dot);
                    //if (dot > clipCenterMinX && dot < clipCenterMaxX)
                    if (ComputeIntersection(ref face.V1, ref face.V2, ref clipEdge, out v))
                    {
                        data[contactData.Count].Position = v;
                        data[contactData.Count].Id = GetContactId(face.Id1, face.Id2, ref clipEdge);
                        contactData.Count++;
                    }
                }
                if (v3MaxYInside)
                {
                    //ComputeIntersection(ref face.V2, ref face.V3, ref clipEdge, out v);
                    //Vector3.Dot(ref clipX, ref v, out dot);
                    //if (dot > clipCenterMinX && dot < clipCenterMaxX)
                    if (ComputeIntersection(ref face.V2, ref face.V3, ref clipEdge, out v))
                    {
                        data[contactData.Count].Position = v;
                        data[contactData.Count].Id = GetContactId(face.Id2, face.Id3, ref clipEdge);
                        contactData.Count++;
                    }
                }
            }
            if (!v3MaxYInside)
            {
                if (v2MaxYInside)
                {
                    //ComputeIntersection(ref face.V2, ref face.V3, ref clipEdge, out v);
                    //Vector3.Dot(ref clipX, ref v, out dot);
                    //if (dot > clipCenterMinX && dot < clipCenterMaxX)
                    if (ComputeIntersection(ref face.V2, ref face.V3, ref clipEdge, out v))
                    {
                        data[contactData.Count].Position = v;
                        data[contactData.Count].Id = GetContactId(face.Id2, face.Id3, ref clipEdge);
                        contactData.Count++;
                    }
                }
                if (v4MaxYInside && contactData.Count < 8)
                {
                    //ComputeIntersection(ref face.V3, ref face.V4, ref clipEdge, out v);
                    //Vector3.Dot(ref clipX, ref v, out dot);
                    //if (dot > clipCenterMinX && dot < clipCenterMaxX)
                    if (ComputeIntersection(ref face.V3, ref face.V4, ref clipEdge, out v))
                    {
                        data[contactData.Count].Position = v;
                        data[contactData.Count].Id = GetContactId(face.Id3, face.Id4, ref clipEdge);
                        contactData.Count++;
                    }
                }
            }
            if (!v4MaxYInside)
            {
                if (v1MaxYInside && contactData.Count < 8)
                {
                    //ComputeIntersection(ref face.V4, ref face.V1, ref clipEdge, out v);
                    //Vector3.Dot(ref clipX, ref v, out dot);
                    //if (dot > clipCenterMinX && dot < clipCenterMaxX)
                    if (ComputeIntersection(ref face.V4, ref face.V1, ref clipEdge, out v))
                    {
                        data[contactData.Count].Position = v;
                        data[contactData.Count].Id = GetContactId(face.Id4, face.Id1, ref clipEdge);
                        contactData.Count++;
                    }
                }
                if (v3MaxYInside && contactData.Count < 8)
                {
                    //ComputeIntersection(ref face.V3, ref face.V4, ref clipEdge, out v);
                    //Vector3.Dot(ref clipX, ref v, out dot);
                    //if (dot > clipCenterMinX && dot < clipCenterMaxX)
                    if (ComputeIntersection(ref face.V3, ref face.V4, ref clipEdge, out v))
                    {
                        data[contactData.Count].Position = v;
                        data[contactData.Count].Id = GetContactId(face.Id3, face.Id4, ref clipEdge);
                        contactData.Count++;
                    }
                }
            }

            #endregion

        #region CLIP EDGE: v2 v3

            clipFace.GetEdge(1, out clipEdge);
            if (!v1MinXInside)
            {
                if (v2MinXInside && contactData.Count < 8)
                {
                    //test v1-v2 against minXminY-minXmaxY
                    //ComputeIntersection(ref face.V1, ref face.V2, ref clipEdge, out v);
                    //Vector3.Dot(ref clipY, ref v, out dot);
                    //if (dot > clipCenterMinY && dot < clipCenterMaxY)
                    if (ComputeIntersection(ref face.V1, ref face.V2, ref clipEdge, out v))
                    {
                        data[contactData.Count].Position = v;
                        data[contactData.Count].Id = GetContactId(face.Id1, face.Id2, ref clipEdge);
                        contactData.Count++;
                    }
                }
                if (v4MinXInside && contactData.Count < 8)
                {
                    //test v1-v3 against minXminY-minXmaxY
                    //ComputeIntersection(ref face.V4, ref face.V1, ref clipEdge, out v);
                    //Vector3.Dot(ref clipY, ref v, out dot);
                    //if (dot > clipCenterMinY && dot < clipCenterMaxY)
                    if (ComputeIntersection(ref face.V4, ref face.V1, ref clipEdge, out v))
                    {
                        data[contactData.Count].Position = v;
                        data[contactData.Count].Id = GetContactId(face.Id4, face.Id1, ref clipEdge);
                        contactData.Count++;
                    }
                }
            }
            if (!v2MinXInside)
            {
                if (v1MinXInside && contactData.Count < 8)
                {
                    //test v1-v2 against minXminY-minXmaxY
                    //ComputeIntersection(ref face.V1, ref face.V2, ref clipEdge, out v);
                    //Vector3.Dot(ref clipY, ref v, out dot);
                    //if (dot > clipCenterMinY && dot < clipCenterMaxY)
                    if (ComputeIntersection(ref face.V1, ref face.V2, ref clipEdge, out v))
                    {
                        data[contactData.Count].Position = v;
                        data[contactData.Count].Id = GetContactId(face.Id1, face.Id2, ref clipEdge);
                        contactData.Count++;
                    }
                }
                if (v3MinXInside && contactData.Count < 8)
                {
                    //test v2-v4 against minXminY-minXmaxY
                    //ComputeIntersection(ref face.V2, ref face.V3, ref clipEdge, out v);
                    //Vector3.Dot(ref clipY, ref v, out dot);
                    //if (dot > clipCenterMinY && dot < clipCenterMaxY)
                    if (ComputeIntersection(ref face.V2, ref face.V3, ref clipEdge, out v))
                    {
                        data[contactData.Count].Position = v;
                        data[contactData.Count].Id = GetContactId(face.Id2, face.Id3, ref clipEdge);
                        contactData.Count++;
                    }
                }
            }
            if (!v3MinXInside)
            {
                if (v2MinXInside && contactData.Count < 8)
                {
                    //test v1-v3 against minXminY-minXmaxY
                    //ComputeIntersection(ref face.V2, ref face.V3, ref clipEdge, out v);
                    //Vector3.Dot(ref clipY, ref v, out dot);
                    //if (dot > clipCenterMinY && dot < clipCenterMaxY)
                    if (ComputeIntersection(ref face.V2, ref face.V3, ref clipEdge, out v))
                    {
                        data[contactData.Count].Position = v;
                        data[contactData.Count].Id = GetContactId(face.Id2, face.Id3, ref clipEdge);
                        contactData.Count++;
                    }
                }
                if (v4MinXInside && contactData.Count < 8)
                {
                    //test v3-v4 against minXminY-minXmaxY
                    //ComputeIntersection(ref face.V3, ref face.V4, ref clipEdge, out v);
                    //Vector3.Dot(ref clipY, ref v, out dot);
                    //if (dot > clipCenterMinY && dot < clipCenterMaxY)
                    if (ComputeIntersection(ref face.V3, ref face.V4, ref clipEdge, out v))
                    {
                        data[contactData.Count].Position = v;
                        data[contactData.Count].Id = GetContactId(face.Id3, face.Id4, ref clipEdge);
                        contactData.Count++;
                    }
                }
            }
            if (!v4MinXInside)
            {
                if (v1MinXInside && contactData.Count < 8)
                {
                    //test v2-v4 against minXminY-minXmaxY
                    //ComputeIntersection(ref face.V4, ref face.V1, ref clipEdge, out v);
                    //Vector3.Dot(ref clipY, ref v, out dot);
                    //if (dot > clipCenterMinY && dot < clipCenterMaxY)
                    if (ComputeIntersection(ref face.V4, ref face.V1, ref clipEdge, out v))
                    {
                        data[contactData.Count].Position = v;
                        data[contactData.Count].Id = GetContactId(face.Id4, face.Id1, ref clipEdge);
                        contactData.Count++;
                    }
                }
                if (v3MinXInside && contactData.Count < 8)
                {
                    //test v3-v4 against minXminY-minXmaxY
                    //ComputeIntersection(ref face.V3, ref face.V4, ref clipEdge, out v);
                    //Vector3.Dot(ref clipY, ref v, out dot);
                    //if (dot > clipCenterMinY && dot < clipCenterMaxY)
                    if (ComputeIntersection(ref face.V3, ref face.V4, ref clipEdge, out v))
                    {
                        data[contactData.Count].Position = v;
                        data[contactData.Count].Id = GetContactId(face.Id3, face.Id4, ref clipEdge);
                        contactData.Count++;
                    }
                }
            }

            #endregion

        #region CLIP EDGE: v3 v4

            clipFace.GetEdge(2, out clipEdge);
            if (!v1MinYInside)
            {
                if (v2MinYInside && contactData.Count < 8)
                {
                    //ComputeIntersection(ref face.V1, ref face.V2, ref clipEdge, out v);
                    //Vector3.Dot(ref clipX, ref v, out dot);
                    //if (dot > clipCenterMinX && dot < clipCenterMaxX)
                    if (ComputeIntersection(ref face.V1, ref face.V2, ref clipEdge, out v))
                    {
                        data[contactData.Count].Position = v;
                        data[contactData.Count].Id = GetContactId(face.Id1, face.Id2, ref clipEdge);
                        contactData.Count++;
                    }
                }
                if (v4MinYInside && contactData.Count < 8)
                {
                    //ComputeIntersection(ref face.V4, ref face.V1, ref clipEdge, out v);
                    //Vector3.Dot(ref clipX, ref v, out dot);
                    //if (dot > clipCenterMinX && dot < clipCenterMaxX)
                    if (ComputeIntersection(ref face.V4, ref face.V1, ref clipEdge, out v))
                    {
                        data[contactData.Count].Position = v;
                        data[contactData.Count].Id = GetContactId(face.Id4, face.Id1, ref clipEdge);
                        contactData.Count++;
                    }
                }
            }
            if (!v2MinYInside)
            {
                if (v1MinYInside && contactData.Count < 8)
                {
                    //ComputeIntersection(ref face.V1, ref face.V2, ref clipEdge, out v);
                    //Vector3.Dot(ref clipX, ref v, out dot);
                    //if (dot > clipCenterMinX && dot < clipCenterMaxX)
                    if (ComputeIntersection(ref face.V1, ref face.V2, ref clipEdge, out v))
                    {
                        data[contactData.Count].Position = v;
                        data[contactData.Count].Id = GetContactId(face.Id1, face.Id2, ref clipEdge);
                        contactData.Count++;
                    }
                }
                if (v3MinYInside && contactData.Count < 8)
                {
                    //ComputeIntersection(ref face.V2, ref face.V3, ref clipEdge, out v);
                    //Vector3.Dot(ref clipX, ref v, out dot);
                    //if (dot > clipCenterMinX && dot < clipCenterMaxX)
                    if (ComputeIntersection(ref face.V2, ref face.V3, ref clipEdge, out v))
                    {
                        data[contactData.Count].Position = v;
                        data[contactData.Count].Id = GetContactId(face.Id2, face.Id3, ref clipEdge);
                        contactData.Count++;
                    }
                }
            }
            if (!v3MinYInside)
            {
                if (v2MinYInside && contactData.Count < 8)
                {
                    //ComputeIntersection(ref face.V2, ref face.V3, ref clipEdge, out v);
                    //Vector3.Dot(ref clipX, ref v, out dot);
                    //if (dot > clipCenterMinX && dot < clipCenterMaxX)
                    if (ComputeIntersection(ref face.V2, ref face.V3, ref clipEdge, out v))
                    {
                        data[contactData.Count].Position = v;
                        data[contactData.Count].Id = GetContactId(face.Id2, face.Id3, ref clipEdge);
                        contactData.Count++;
                    }
                }
                if (v4MinYInside && contactData.Count < 8)
                {
                    //ComputeIntersection(ref face.V3, ref face.V4, ref clipEdge, out v);
                    //Vector3.Dot(ref clipX, ref v, out dot);
                    //if (dot > clipCenterMinX && dot < clipCenterMaxX)
                    if (ComputeIntersection(ref face.V3, ref face.V4, ref clipEdge, out v))
                    {
                        data[contactData.Count].Position = v;
                        data[contactData.Count].Id = GetContactId(face.Id3, face.Id4, ref clipEdge);
                        contactData.Count++;
                    }
                }
            }
            if (!v4MinYInside)
            {
                if (v3MinYInside && contactData.Count < 8)
                {
                    //ComputeIntersection(ref face.V3, ref face.V4, ref clipEdge, out v);
                    //Vector3.Dot(ref clipX, ref v, out dot);
                    //if (dot > clipCenterMinX && dot < clipCenterMaxX)
                    if (ComputeIntersection(ref face.V3, ref face.V4, ref clipEdge, out v))
                    {
                        data[contactData.Count].Position = v;
                        data[contactData.Count].Id = GetContactId(face.Id3, face.Id4, ref clipEdge);
                        contactData.Count++;
                    }
                }
                if (v1MinYInside && contactData.Count < 8)
                {
                    //ComputeIntersection(ref face.V4, ref face.V1, ref clipEdge, out v);
                    //Vector3.Dot(ref clipX, ref v, out dot);
                    //if (dot > clipCenterMinX && dot < clipCenterMaxX)
                    if (ComputeIntersection(ref face.V4, ref face.V1, ref clipEdge, out v))
                    {
                        data[contactData.Count].Position = v;
                        data[contactData.Count].Id = GetContactId(face.Id4, face.Id1, ref clipEdge);
                        contactData.Count++;
                    }
                }
            }

            #endregion

        #region CLIP EDGE: v4 v1

            clipFace.GetEdge(3, out clipEdge);
            if (!v1MaxXInside)
            {
                if (v2MaxXInside && contactData.Count < 8)
                {
                    //ComputeIntersection(ref face.V1, ref face.V2, ref clipEdge, out v);
                    //Vector3.Dot(ref clipY, ref v, out dot);
                    //if (dot > clipCenterMinY && dot < clipCenterMaxY)
                    if (ComputeIntersection(ref face.V1, ref face.V2, ref clipEdge, out v))
                    {
                        data[contactData.Count].Position = v;
                        data[contactData.Count].Id = GetContactId(face.Id1, face.Id2, ref clipEdge);
                        contactData.Count++;
                    }
                }
                if (v4MaxXInside && contactData.Count < 8)
                {
                    //ComputeIntersection(ref face.V4, ref face.V1, ref clipEdge, out v);
                    //Vector3.Dot(ref clipY, ref v, out dot);
                    //if (dot > clipCenterMinY && dot < clipCenterMaxY)
                    if (ComputeIntersection(ref face.V4, ref face.V1, ref clipEdge, out v))
                    {
                        data[contactData.Count].Position = v;
                        data[contactData.Count].Id = GetContactId(face.Id4, face.Id1, ref clipEdge);
                        contactData.Count++;
                    }
                }
            }
            if (!v2MaxXInside)
            {
                if (v1MaxXInside && contactData.Count < 8)
                {
                    //ComputeIntersection(ref face.V1, ref face.V2, ref clipEdge, out v);
                    //Vector3.Dot(ref clipY, ref v, out dot);
                    //if (dot > clipCenterMinY && dot < clipCenterMaxY)
                    if (ComputeIntersection(ref face.V1, ref face.V2, ref clipEdge, out v))
                    {
                        data[contactData.Count].Position = v;
                        data[contactData.Count].Id = GetContactId(face.Id1, face.Id2, ref clipEdge);
                        contactData.Count++;
                    }
                }
                if (v3MaxXInside && contactData.Count < 8)
                {
                    //ComputeIntersection(ref face.V2, ref face.V3, ref clipEdge, out v);
                    //Vector3.Dot(ref clipY, ref v, out dot);
                    //if (dot > clipCenterMinY && dot < clipCenterMaxY)
                    if (ComputeIntersection(ref face.V2, ref face.V3, ref clipEdge, out v))
                    {
                        data[contactData.Count].Position = v;
                        data[contactData.Count].Id = GetContactId(face.Id2, face.Id3, ref clipEdge);
                        contactData.Count++;
                    }
                }
            }
            if (!v3MaxXInside)
            {
                if (v2MaxXInside && contactData.Count < 8)
                {
                    //ComputeIntersection(ref face.V2, ref face.V3, ref clipEdge, out v);
                    //Vector3.Dot(ref clipY, ref v, out dot);
                    //if (dot > clipCenterMinY && dot < clipCenterMaxY)
                    if (ComputeIntersection(ref face.V2, ref face.V3, ref clipEdge, out v))
                    {
                        data[contactData.Count].Position = v;
                        data[contactData.Count].Id = GetContactId(face.Id2, face.Id3, ref clipEdge);
                        contactData.Count++;
                    }
                }
                if (v4MaxXInside && contactData.Count < 8)
                {
                    //ComputeIntersection(ref face.V3, ref face.V4, ref clipEdge, out v);
                    //Vector3.Dot(ref clipY, ref v, out dot);
                    //if (dot > clipCenterMinY && dot < clipCenterMaxY)
                    if (ComputeIntersection(ref face.V3, ref face.V4, ref clipEdge, out v))
                    {
                        data[contactData.Count].Position = v;
                        data[contactData.Count].Id = GetContactId(face.Id3, face.Id4, ref clipEdge);
                        contactData.Count++;
                    }
                }
            }
            if (!v4MaxXInside)
            {
                if (v1MaxXInside && contactData.Count < 8)
                {
                    //ComputeIntersection(ref face.V4, ref face.V1, ref clipEdge, out v);
                    //Vector3.Dot(ref clipY, ref v, out dot);
                    //if (dot > clipCenterMinY && dot < clipCenterMaxY)
                    if (ComputeIntersection(ref face.V4, ref face.V1, ref clipEdge, out v))
                    {
                        data[contactData.Count].Position = v;
                        data[contactData.Count].Id = GetContactId(face.Id4, face.Id1, ref clipEdge);
                        contactData.Count++;
                    }
                }
                if (v3MaxXInside && contactData.Count < 8)
                {
                    //ComputeIntersection(ref face.V3, ref face.V4, ref clipEdge, out v);
                    //Vector3.Dot(ref clipY, ref v, out dot);
                    //if (dot > clipCenterMinY && dot < clipCenterMaxY)
                    if (ComputeIntersection(ref face.V3, ref face.V4, ref clipEdge, out v))
                    {
                        data[contactData.Count].Position = v;
                        data[contactData.Count].Id = GetContactId(face.Id3, face.Id4, ref clipEdge);
                        contactData.Count++;
                    }
                }
            }

            #endregion

            //Compute depths.
            tempData = contactData;
            contactData.Count = previousCount;

            for (int i = previousCount; i < tempData.Count; i++)
            {
                Vector3.Dot(ref temp[i].Position, ref mtd, out faceDot);
                depth = faceDot - clipFaceDot;
                if (depth <= F64.C0)
                {
                    data[contactData.Count].Position = temp[i].Position;
                    data[contactData.Count].Depth = depth;
                    data[contactData.Count].Id = temp[i].Id;
                    contactData.Count++;
                }
            }
            outputData = contactData;
        }
#else
        private static void ClipFacesDirect(ref BoxFace clipFace, ref BoxFace face, ref Vector3 mtd, out TinyStructList<BoxContactData> contactData)
        {
            contactData = new TinyStructList<BoxContactData>();

            //Local directions on the clip face.  Their length is equal to the length of an edge.
            Vector3 clipX, clipY;
            Vector3.Subtract(ref clipFace.V4, ref clipFace.V3, out clipX);
            Vector3.Subtract(ref clipFace.V2, ref clipFace.V3, out clipY);
            Fix64 inverseClipWidth = 1 / clipFace.Width;
            Fix64 inverseClipHeight = 1 / clipFace.Height;
            Fix64 inverseClipWidthSquared = inverseClipWidth * inverseClipWidth;
            clipX.X *= inverseClipWidthSquared;
            clipX.Y *= inverseClipWidthSquared;
            clipX.Z *= inverseClipWidthSquared;
            Fix64 inverseClipHeightSquared = inverseClipHeight * inverseClipHeight;
            clipY.X *= inverseClipHeightSquared;
            clipY.Y *= inverseClipHeightSquared;
            clipY.Z *= inverseClipHeightSquared;

            //Local directions on the opposing face.  Their length is equal to the length of an edge.
            Vector3 faceX, faceY;
            Vector3.Subtract(ref face.V4, ref face.V3, out faceX);
            Vector3.Subtract(ref face.V2, ref face.V3, out faceY);
            Fix64 inverseFaceWidth = 1 / face.Width;
            Fix64 inverseFaceHeight = 1 / face.Height;
            Fix64 inverseFaceWidthSquared = inverseFaceWidth * inverseFaceWidth;
            faceX.X *= inverseFaceWidthSquared;
            faceX.Y *= inverseFaceWidthSquared;
            faceX.Z *= inverseFaceWidthSquared;
            Fix64 inverseFaceHeightSquared = inverseFaceHeight * inverseFaceHeight;
            faceY.X *= inverseFaceHeightSquared;
            faceY.Y *= inverseFaceHeightSquared;
            faceY.Z *= inverseFaceHeightSquared;

            Vector3 clipCenter;
            Vector3.Add(ref clipFace.V1, ref clipFace.V3, out clipCenter);
            //Defer division until after dot product (2 multiplies instead of 3)
            Fix64 clipCenterX, clipCenterY;
            Vector3.Dot(ref clipCenter, ref clipX, out clipCenterX);
            Vector3.Dot(ref clipCenter, ref clipY, out clipCenterY);
            clipCenterX *= .5f;
            clipCenterY *= .5f;

            Vector3 faceCenter;
            Vector3.Add(ref face.V1, ref face.V3, out faceCenter);
            //Defer division until after dot product (2 multiplies instead of 3)
            Fix64 faceCenterX, faceCenterY;
            Vector3.Dot(ref faceCenter, ref faceX, out faceCenterX);
            Vector3.Dot(ref faceCenter, ref faceY, out faceCenterY);
            faceCenterX *= .5f;
            faceCenterY *= .5f;

            //To test bounds, recall that clipX is the length of the X edge.
            //Going from the center to the max or min goes half of the length of X edge, or +/- 0.5.
            //Bias could be added here.
            //const Fix64 extent = .5f; //.5f is the default, extra could be added for robustness or speed.
            Fix64 extentX = .5f + .01f * inverseClipWidth;
            Fix64 extentY = .5f + .01f * inverseClipHeight;
            //Fix64 extentX = .5f + .01f * inverseClipXLength;
            //Fix64 extentY = .5f + .01f * inverseClipYLength;
            Fix64 clipCenterMaxX = clipCenterX + extentX;
            Fix64 clipCenterMaxY = clipCenterY + extentY;
            Fix64 clipCenterMinX = clipCenterX - extentX;
            Fix64 clipCenterMinY = clipCenterY - extentY;

            extentX = .5f + .01f * inverseFaceWidth;
            extentY = .5f + .01f * inverseFaceHeight;
            //extentX = .5f + .01f * inverseFaceXLength;
            //extentY = .5f + .01f * inverseFaceYLength;
            Fix64 faceCenterMaxX = faceCenterX + extentX;
            Fix64 faceCenterMaxY = faceCenterY + extentY;
            Fix64 faceCenterMinX = faceCenterX - extentX;
            Fix64 faceCenterMinY = faceCenterY - extentY;

            //Find out where the opposing face is.
            Fix64 dotX, dotY;

            //The four edges can be thought of as minX, maxX, minY and maxY.

            //Face v1
            Vector3.Dot(ref clipX, ref face.V1, out dotX);
            bool v1MaxXInside = dotX < clipCenterMaxX;
            bool v1MinXInside = dotX > clipCenterMinX;
            Vector3.Dot(ref clipY, ref face.V1, out dotY);
            bool v1MaxYInside = dotY < clipCenterMaxY;
            bool v1MinYInside = dotY > clipCenterMinY;

            //Face v2
            Vector3.Dot(ref clipX, ref face.V2, out dotX);
            bool v2MaxXInside = dotX < clipCenterMaxX;
            bool v2MinXInside = dotX > clipCenterMinX;
            Vector3.Dot(ref clipY, ref face.V2, out dotY);
            bool v2MaxYInside = dotY < clipCenterMaxY;
            bool v2MinYInside = dotY > clipCenterMinY;

            //Face v3
            Vector3.Dot(ref clipX, ref face.V3, out dotX);
            bool v3MaxXInside = dotX < clipCenterMaxX;
            bool v3MinXInside = dotX > clipCenterMinX;
            Vector3.Dot(ref clipY, ref face.V3, out dotY);
            bool v3MaxYInside = dotY < clipCenterMaxY;
            bool v3MinYInside = dotY > clipCenterMinY;

            //Face v4
            Vector3.Dot(ref clipX, ref face.V4, out dotX);
            bool v4MaxXInside = dotX < clipCenterMaxX;
            bool v4MinXInside = dotX > clipCenterMinX;
            Vector3.Dot(ref clipY, ref face.V4, out dotY);
            bool v4MaxYInside = dotY < clipCenterMaxY;
            bool v4MinYInside = dotY > clipCenterMinY;

            //Find out where the clip face is.
            //Clip v1
            Vector3.Dot(ref faceX, ref clipFace.V1, out dotX);
            bool clipv1MaxXInside = dotX < faceCenterMaxX;
            bool clipv1MinXInside = dotX > faceCenterMinX;
            Vector3.Dot(ref faceY, ref clipFace.V1, out dotY);
            bool clipv1MaxYInside = dotY < faceCenterMaxY;
            bool clipv1MinYInside = dotY > faceCenterMinY;

            //Clip v2
            Vector3.Dot(ref faceX, ref clipFace.V2, out dotX);
            bool clipv2MaxXInside = dotX < faceCenterMaxX;
            bool clipv2MinXInside = dotX > faceCenterMinX;
            Vector3.Dot(ref faceY, ref clipFace.V2, out dotY);
            bool clipv2MaxYInside = dotY < faceCenterMaxY;
            bool clipv2MinYInside = dotY > faceCenterMinY;

            //Clip v3
            Vector3.Dot(ref faceX, ref clipFace.V3, out dotX);
            bool clipv3MaxXInside = dotX < faceCenterMaxX;
            bool clipv3MinXInside = dotX > faceCenterMinX;
            Vector3.Dot(ref faceY, ref clipFace.V3, out dotY);
            bool clipv3MaxYInside = dotY < faceCenterMaxY;
            bool clipv3MinYInside = dotY > faceCenterMinY;

            //Clip v4
            Vector3.Dot(ref faceX, ref clipFace.V4, out dotX);
            bool clipv4MaxXInside = dotX < faceCenterMaxX;
            bool clipv4MinXInside = dotX > faceCenterMinX;
            Vector3.Dot(ref faceY, ref clipFace.V4, out dotY);
            bool clipv4MaxYInside = dotY < faceCenterMaxY;
            bool clipv4MinYInside = dotY > faceCenterMinY;

            #region Face Vertices
            BoxContactData item = new BoxContactData();
            if (v1MinXInside && v1MaxXInside && v1MinYInside && v1MaxYInside)
            {
                item.Position = face.V1;
                item.Id = face.Id1;
                contactData.Add(ref item);
            }

            if (v2MinXInside && v2MaxXInside && v2MinYInside && v2MaxYInside)
            {
                item.Position = face.V2;
                item.Id = face.Id2;
                contactData.Add(ref item);
            }

            if (v3MinXInside && v3MaxXInside && v3MinYInside && v3MaxYInside)
            {
                item.Position = face.V3;
                item.Id = face.Id3;
                contactData.Add(ref item);
            }

            if (v4MinXInside && v4MaxXInside && v4MinYInside && v4MaxYInside)
            {
                item.Position = face.V4;
                item.Id = face.Id4;
                contactData.Add(ref item);
            }

            #endregion

            //Compute depths.
            TinyStructList<BoxContactData> tempData = contactData;
            contactData.Clear();
            Fix64 clipFaceDot, faceDot;
            Vector3.Dot(ref clipFace.V1, ref mtd, out clipFaceDot);
            for (int i = 0; i < tempData.Count; i++)
            {
                tempData.Get(i, out item);
                Vector3.Dot(ref item.Position, ref mtd, out faceDot);
                item.Depth = faceDot - clipFaceDot;
                if (item.Depth <= 0)
                {
                    contactData.Add(ref item);
                }
            }

            int previousCount = contactData.Count;
            if (previousCount >= 4) //Early finish :)
            {
                return;
            }

            #region Clip face vertices

            Vector3 v;
            Fix64 a, b;
            Vector3.Dot(ref face.V1, ref face.Normal, out b);
            //CLIP FACE
            if (clipv1MinXInside && clipv1MaxXInside && clipv1MinYInside && clipv1MaxYInside)
            {
                Vector3.Dot(ref clipFace.V1, ref face.Normal, out a);
                Vector3.Multiply(ref face.Normal, a - b, out v);
                Vector3.Subtract(ref clipFace.V1, ref v, out v);
                item.Position = v;
                item.Id = clipFace.Id1 + 8;
                contactData.Add(ref item);
            }

            if (clipv2MinXInside && clipv2MaxXInside && clipv2MinYInside && clipv2MaxYInside)
            {
                Vector3.Dot(ref clipFace.V2, ref face.Normal, out a);
                Vector3.Multiply(ref face.Normal, a - b, out v);
                Vector3.Subtract(ref clipFace.V2, ref v, out v);
                item.Position = v;
                item.Id = clipFace.Id2 + 8;
                contactData.Add(ref item);
            }

            if (clipv3MinXInside && clipv3MaxXInside && clipv3MinYInside && clipv3MaxYInside)
            {
                Vector3.Dot(ref clipFace.V3, ref face.Normal, out a);
                Vector3.Multiply(ref face.Normal, a - b, out v);
                Vector3.Subtract(ref clipFace.V3, ref v, out v);
                item.Position = v;
                item.Id = clipFace.Id3 + 8;
                contactData.Add(ref item);
            }

            if (clipv4MinXInside && clipv4MaxXInside && clipv4MinYInside && clipv4MaxYInside)
            {
                Vector3.Dot(ref clipFace.V4, ref face.Normal, out a);
                Vector3.Multiply(ref face.Normal, a - b, out v);
                Vector3.Subtract(ref clipFace.V4, ref v, out v);
                item.Position = v;
                item.Id = clipFace.Id4 + 8;
                contactData.Add(ref item);
            }

            #endregion

            //Compute depths.
            int postClipCount = contactData.Count;
            tempData = contactData;
            for (int i = postClipCount - 1; i >= previousCount; i--) //TODO: >=?
                contactData.RemoveAt(i);


            for (int i = previousCount; i < tempData.Count; i++)
            {
                tempData.Get(i, out item);
                Vector3.Dot(ref item.Position, ref mtd, out faceDot);
                item.Depth = faceDot - clipFaceDot;
                if (item.Depth <= 0)
                {
                    contactData.Add(ref item);
                }
            }

            previousCount = contactData.Count;
            if (previousCount >= 4) //Early finish :)
            {
                return;
            }
            //Intersect edges.

            //maxX maxY -> v1
            //minX maxY -> v2
            //minX minY -> v3
            //maxX minY -> v4

            //Once we get here there can only be 3 contacts or less.
            //Once 4 possible contacts have been added, switch to using safe increments.
            //Fix64 dot;

            #region CLIP EDGE: v1 v2

            FaceEdge clipEdge;
            clipFace.GetEdge(0, out clipEdge);
            if (!v1MaxYInside)
            {
                if (v2MaxYInside)
                {
                    //ComputeIntersection(ref face.V1, ref face.V2, ref clipEdge, out v);
                    //Vector3.Dot(ref clipX, ref v, out dot);
                    //if (dot > clipCenterMinX && dot < clipCenterMaxX)
                    if (ComputeIntersection(ref face.V1, ref face.V2, ref clipEdge, out v))
                    {
                        item.Position = v;
                        item.Id = GetContactId(face.Id1, face.Id2, ref clipEdge);
                        contactData.Add(ref item);
                    }
                }
                if (v4MaxYInside)
                {
                    //ComputeIntersection(ref face.V4, ref face.V1, ref clipEdge, out v);
                    //Vector3.Dot(ref clipX, ref v, out dot);
                    //if (dot > clipCenterMinX && dot < clipCenterMaxX)
                    if (ComputeIntersection(ref face.V4, ref face.V1, ref clipEdge, out v))
                    {
                        item.Position = v;
                        item.Id = GetContactId(face.Id4, face.Id1, ref clipEdge);
                        contactData.Add(ref item);
                    }
                }
            }
            if (!v2MaxYInside)
            {
                if (v1MaxYInside)
                {
                    //ComputeIntersection(ref face.V1, ref face.V2, ref clipEdge, out v);
                    //Vector3.Dot(ref clipX, ref v, out dot);
                    //if (dot > clipCenterMinX && dot < clipCenterMaxX)
                    if (ComputeIntersection(ref face.V1, ref face.V2, ref clipEdge, out v))
                    {
                        item.Position = v;
                        item.Id = GetContactId(face.Id1, face.Id2, ref clipEdge);
                        contactData.Add(ref item);
                    }
                }
                if (v3MaxYInside)
                {
                    //ComputeIntersection(ref face.V2, ref face.V3, ref clipEdge, out v);
                    //Vector3.Dot(ref clipX, ref v, out dot);
                    //if (dot > clipCenterMinX && dot < clipCenterMaxX)
                    if (ComputeIntersection(ref face.V2, ref face.V3, ref clipEdge, out v))
                    {
                        item.Position = v;
                        item.Id = GetContactId(face.Id2, face.Id3, ref clipEdge);
                        contactData.Add(ref item);
                    }
                }
            }
            if (!v3MaxYInside)
            {
                if (v2MaxYInside)
                {
                    //ComputeIntersection(ref face.V2, ref face.V3, ref clipEdge, out v);
                    //Vector3.Dot(ref clipX, ref v, out dot);
                    //if (dot > clipCenterMinX && dot < clipCenterMaxX)
                    if (ComputeIntersection(ref face.V2, ref face.V3, ref clipEdge, out v))
                    {
                        item.Position = v;
                        item.Id = GetContactId(face.Id2, face.Id3, ref clipEdge);
                        contactData.Add(ref item);
                    }
                }
                if (v4MaxYInside && contactData.Count < 8)
                {
                    //ComputeIntersection(ref face.V3, ref face.V4, ref clipEdge, out v);
                    //Vector3.Dot(ref clipX, ref v, out dot);
                    //if (dot > clipCenterMinX && dot < clipCenterMaxX)
                    if (ComputeIntersection(ref face.V3, ref face.V4, ref clipEdge, out v))
                    {
                        item.Position = v;
                        item.Id = GetContactId(face.Id3, face.Id4, ref clipEdge);
                        contactData.Add(ref item);
                    }
                }
            }
            if (!v4MaxYInside)
            {
                if (v1MaxYInside && contactData.Count < 8)
                {
                    //ComputeIntersection(ref face.V4, ref face.V1, ref clipEdge, out v);
                    //Vector3.Dot(ref clipX, ref v, out dot);
                    //if (dot > clipCenterMinX && dot < clipCenterMaxX)
                    if (ComputeIntersection(ref face.V4, ref face.V1, ref clipEdge, out v))
                    {
                        item.Position = v;
                        item.Id = GetContactId(face.Id4, face.Id1, ref clipEdge);
                        contactData.Add(ref item);
                    }
                }
                if (v3MaxYInside && contactData.Count < 8)
                {
                    //ComputeIntersection(ref face.V3, ref face.V4, ref clipEdge, out v);
                    //Vector3.Dot(ref clipX, ref v, out dot);
                    //if (dot > clipCenterMinX && dot < clipCenterMaxX)
                    if (ComputeIntersection(ref face.V3, ref face.V4, ref clipEdge, out v))
                    {
                        item.Position = v;
                        item.Id = GetContactId(face.Id3, face.Id4, ref clipEdge);
                        contactData.Add(ref item);
                    }
                }
            }

            #endregion

            #region CLIP EDGE: v2 v3

            clipFace.GetEdge(1, out clipEdge);
            if (!v1MinXInside)
            {
                if (v2MinXInside && contactData.Count < 8)
                {
                    //test v1-v2 against minXminY-minXmaxY
                    //ComputeIntersection(ref face.V1, ref face.V2, ref clipEdge, out v);
                    //Vector3.Dot(ref clipY, ref v, out dot);
                    //if (dot > clipCenterMinY && dot < clipCenterMaxY)
                    if (ComputeIntersection(ref face.V1, ref face.V2, ref clipEdge, out v))
                    {
                        item.Position = v;
                        item.Id = GetContactId(face.Id1, face.Id2, ref clipEdge);
                        contactData.Add(ref item);
                    }
                }
                if (v4MinXInside && contactData.Count < 8)
                {
                    //test v1-v3 against minXminY-minXmaxY
                    //ComputeIntersection(ref face.V4, ref face.V1, ref clipEdge, out v);
                    //Vector3.Dot(ref clipY, ref v, out dot);
                    //if (dot > clipCenterMinY && dot < clipCenterMaxY)
                    if (ComputeIntersection(ref face.V4, ref face.V1, ref clipEdge, out v))
                    {
                        item.Position = v;
                        item.Id = GetContactId(face.Id4, face.Id1, ref clipEdge);
                        contactData.Add(ref item);
                    }
                }
            }
            if (!v2MinXInside)
            {
                if (v1MinXInside && contactData.Count < 8)
                {
                    //test v1-v2 against minXminY-minXmaxY
                    //ComputeIntersection(ref face.V1, ref face.V2, ref clipEdge, out v);
                    //Vector3.Dot(ref clipY, ref v, out dot);
                    //if (dot > clipCenterMinY && dot < clipCenterMaxY)
                    if (ComputeIntersection(ref face.V1, ref face.V2, ref clipEdge, out v))
                    {
                        item.Position = v;
                        item.Id = GetContactId(face.Id1, face.Id2, ref clipEdge);
                        contactData.Add(ref item);
                    }
                }
                if (v3MinXInside && contactData.Count < 8)
                {
                    //test v2-v4 against minXminY-minXmaxY
                    //ComputeIntersection(ref face.V2, ref face.V3, ref clipEdge, out v);
                    //Vector3.Dot(ref clipY, ref v, out dot);
                    //if (dot > clipCenterMinY && dot < clipCenterMaxY)
                    if (ComputeIntersection(ref face.V2, ref face.V3, ref clipEdge, out v))
                    {
                        item.Position = v;
                        item.Id = GetContactId(face.Id2, face.Id3, ref clipEdge);
                        contactData.Add(ref item);
                    }
                }
            }
            if (!v3MinXInside)
            {
                if (v2MinXInside && contactData.Count < 8)
                {
                    //test v1-v3 against minXminY-minXmaxY
                    //ComputeIntersection(ref face.V2, ref face.V3, ref clipEdge, out v);
                    //Vector3.Dot(ref clipY, ref v, out dot);
                    //if (dot > clipCenterMinY && dot < clipCenterMaxY)
                    if (ComputeIntersection(ref face.V2, ref face.V3, ref clipEdge, out v))
                    {
                        item.Position = v;
                        item.Id = GetContactId(face.Id2, face.Id3, ref clipEdge);
                        contactData.Add(ref item);
                    }
                }
                if (v4MinXInside && contactData.Count < 8)
                {
                    //test v3-v4 against minXminY-minXmaxY
                    //ComputeIntersection(ref face.V3, ref face.V4, ref clipEdge, out v);
                    //Vector3.Dot(ref clipY, ref v, out dot);
                    //if (dot > clipCenterMinY && dot < clipCenterMaxY)
                    if (ComputeIntersection(ref face.V3, ref face.V4, ref clipEdge, out v))
                    {
                        item.Position = v;
                        item.Id = GetContactId(face.Id3, face.Id4, ref clipEdge);
                        contactData.Add(ref item);
                    }
                }
            }
            if (!v4MinXInside)
            {
                if (v1MinXInside && contactData.Count < 8)
                {
                    //test v2-v4 against minXminY-minXmaxY
                    //ComputeIntersection(ref face.V4, ref face.V1, ref clipEdge, out v);
                    //Vector3.Dot(ref clipY, ref v, out dot);
                    //if (dot > clipCenterMinY && dot < clipCenterMaxY)
                    if (ComputeIntersection(ref face.V4, ref face.V1, ref clipEdge, out v))
                    {
                        item.Position = v;
                        item.Id = GetContactId(face.Id4, face.Id1, ref clipEdge);
                        contactData.Add(ref item);
                    }
                }
                if (v3MinXInside && contactData.Count < 8)
                {
                    //test v3-v4 against minXminY-minXmaxY
                    //ComputeIntersection(ref face.V3, ref face.V4, ref clipEdge, out v);
                    //Vector3.Dot(ref clipY, ref v, out dot);
                    //if (dot > clipCenterMinY && dot < clipCenterMaxY)
                    if (ComputeIntersection(ref face.V3, ref face.V4, ref clipEdge, out v))
                    {
                        item.Position = v;
                        item.Id = GetContactId(face.Id3, face.Id4, ref clipEdge);
                        contactData.Add(ref item);
                    }
                }
            }

            #endregion

            #region CLIP EDGE: v3 v4

            clipFace.GetEdge(2, out clipEdge);
            if (!v1MinYInside)
            {
                if (v2MinYInside && contactData.Count < 8)
                {
                    //ComputeIntersection(ref face.V1, ref face.V2, ref clipEdge, out v);
                    //Vector3.Dot(ref clipX, ref v, out dot);
                    //if (dot > clipCenterMinX && dot < clipCenterMaxX)
                    if (ComputeIntersection(ref face.V1, ref face.V2, ref clipEdge, out v))
                    {
                        item.Position = v;
                        item.Id = GetContactId(face.Id1, face.Id2, ref clipEdge);
                        contactData.Add(ref item);
                    }
                }
                if (v4MinYInside && contactData.Count < 8)
                {
                    //ComputeIntersection(ref face.V4, ref face.V1, ref clipEdge, out v);
                    //Vector3.Dot(ref clipX, ref v, out dot);
                    //if (dot > clipCenterMinX && dot < clipCenterMaxX)
                    if (ComputeIntersection(ref face.V4, ref face.V1, ref clipEdge, out v))
                    {
                        item.Position = v;
                        item.Id = GetContactId(face.Id4, face.Id1, ref clipEdge);
                        contactData.Add(ref item);
                    }
                }
            }
            if (!v2MinYInside)
            {
                if (v1MinYInside && contactData.Count < 8)
                {
                    //ComputeIntersection(ref face.V1, ref face.V2, ref clipEdge, out v);
                    //Vector3.Dot(ref clipX, ref v, out dot);
                    //if (dot > clipCenterMinX && dot < clipCenterMaxX)
                    if (ComputeIntersection(ref face.V1, ref face.V2, ref clipEdge, out v))
                    {
                        item.Position = v;
                        item.Id = GetContactId(face.Id1, face.Id2, ref clipEdge);
                        contactData.Add(ref item);
                    }
                }
                if (v3MinYInside && contactData.Count < 8)
                {
                    //ComputeIntersection(ref face.V2, ref face.V3, ref clipEdge, out v);
                    //Vector3.Dot(ref clipX, ref v, out dot);
                    //if (dot > clipCenterMinX && dot < clipCenterMaxX)
                    if (ComputeIntersection(ref face.V2, ref face.V3, ref clipEdge, out v))
                    {
                        item.Position = v;
                        item.Id = GetContactId(face.Id2, face.Id3, ref clipEdge);
                        contactData.Add(ref item);
                    }
                }
            }
            if (!v3MinYInside)
            {
                if (v2MinYInside && contactData.Count < 8)
                {
                    //ComputeIntersection(ref face.V2, ref face.V3, ref clipEdge, out v);
                    //Vector3.Dot(ref clipX, ref v, out dot);
                    //if (dot > clipCenterMinX && dot < clipCenterMaxX)
                    if (ComputeIntersection(ref face.V2, ref face.V3, ref clipEdge, out v))
                    {
                        item.Position = v;
                        item.Id = GetContactId(face.Id2, face.Id3, ref clipEdge);
                        contactData.Add(ref item);
                    }
                }
                if (v4MinYInside && contactData.Count < 8)
                {
                    //ComputeIntersection(ref face.V3, ref face.V4, ref clipEdge, out v);
                    //Vector3.Dot(ref clipX, ref v, out dot);
                    //if (dot > clipCenterMinX && dot < clipCenterMaxX)
                    if (ComputeIntersection(ref face.V3, ref face.V4, ref clipEdge, out v))
                    {
                        item.Position = v;
                        item.Id = GetContactId(face.Id3, face.Id4, ref clipEdge);
                        contactData.Add(ref item);
                    }
                }
            }
            if (!v4MinYInside)
            {
                if (v3MinYInside && contactData.Count < 8)
                {
                    //ComputeIntersection(ref face.V3, ref face.V4, ref clipEdge, out v);
                    //Vector3.Dot(ref clipX, ref v, out dot);
                    //if (dot > clipCenterMinX && dot < clipCenterMaxX)
                    if (ComputeIntersection(ref face.V3, ref face.V4, ref clipEdge, out v))
                    {
                        item.Position = v;
                        item.Id = GetContactId(face.Id3, face.Id4, ref clipEdge);
                        contactData.Add(ref item);
                    }
                }
                if (v1MinYInside && contactData.Count < 8)
                {
                    //ComputeIntersection(ref face.V4, ref face.V1, ref clipEdge, out v);
                    //Vector3.Dot(ref clipX, ref v, out dot);
                    //if (dot > clipCenterMinX && dot < clipCenterMaxX)
                    if (ComputeIntersection(ref face.V4, ref face.V1, ref clipEdge, out v))
                    {
                        item.Position = v;
                        item.Id = GetContactId(face.Id4, face.Id1, ref clipEdge);
                        contactData.Add(ref item);
                    }
                }
            }

            #endregion

            #region CLIP EDGE: v4 v1

            clipFace.GetEdge(3, out clipEdge);
            if (!v1MaxXInside)
            {
                if (v2MaxXInside && contactData.Count < 8)
                {
                    //ComputeIntersection(ref face.V1, ref face.V2, ref clipEdge, out v);
                    //Vector3.Dot(ref clipY, ref v, out dot);
                    //if (dot > clipCenterMinY && dot < clipCenterMaxY)
                    if (ComputeIntersection(ref face.V1, ref face.V2, ref clipEdge, out v))
                    {
                        item.Position = v;
                        item.Id = GetContactId(face.Id1, face.Id2, ref clipEdge);
                        contactData.Add(ref item);
                    }
                }
                if (v4MaxXInside && contactData.Count < 8)
                {
                    //ComputeIntersection(ref face.V4, ref face.V1, ref clipEdge, out v);
                    //Vector3.Dot(ref clipY, ref v, out dot);
                    //if (dot > clipCenterMinY && dot < clipCenterMaxY)
                    if (ComputeIntersection(ref face.V4, ref face.V1, ref clipEdge, out v))
                    {
                        item.Position = v;
                        item.Id = GetContactId(face.Id4, face.Id1, ref clipEdge);
                        contactData.Add(ref item);
                    }
                }
            }
            if (!v2MaxXInside)
            {
                if (v1MaxXInside && contactData.Count < 8)
                {
                    //ComputeIntersection(ref face.V1, ref face.V2, ref clipEdge, out v);
                    //Vector3.Dot(ref clipY, ref v, out dot);
                    //if (dot > clipCenterMinY && dot < clipCenterMaxY)
                    if (ComputeIntersection(ref face.V1, ref face.V2, ref clipEdge, out v))
                    {
                        item.Position = v;
                        item.Id = GetContactId(face.Id1, face.Id2, ref clipEdge);
                        contactData.Add(ref item);
                    }
                }
                if (v3MaxXInside && contactData.Count < 8)
                {
                    //ComputeIntersection(ref face.V2, ref face.V3, ref clipEdge, out v);
                    //Vector3.Dot(ref clipY, ref v, out dot);
                    //if (dot > clipCenterMinY && dot < clipCenterMaxY)
                    if (ComputeIntersection(ref face.V2, ref face.V3, ref clipEdge, out v))
                    {
                        item.Position = v;
                        item.Id = GetContactId(face.Id2, face.Id3, ref clipEdge);
                        contactData.Add(ref item);
                    }
                }
            }
            if (!v3MaxXInside)
            {
                if (v2MaxXInside && contactData.Count < 8)
                {
                    //ComputeIntersection(ref face.V2, ref face.V3, ref clipEdge, out v);
                    //Vector3.Dot(ref clipY, ref v, out dot);
                    //if (dot > clipCenterMinY && dot < clipCenterMaxY)
                    if (ComputeIntersection(ref face.V2, ref face.V3, ref clipEdge, out v))
                    {
                        item.Position = v;
                        item.Id = GetContactId(face.Id2, face.Id3, ref clipEdge);
                        contactData.Add(ref item);
                    }
                }
                if (v4MaxXInside && contactData.Count < 8)
                {
                    //ComputeIntersection(ref face.V3, ref face.V4, ref clipEdge, out v);
                    //Vector3.Dot(ref clipY, ref v, out dot);
                    //if (dot > clipCenterMinY && dot < clipCenterMaxY)
                    if (ComputeIntersection(ref face.V3, ref face.V4, ref clipEdge, out v))
                    {
                        item.Position = v;
                        item.Id = GetContactId(face.Id3, face.Id4, ref clipEdge);
                        contactData.Add(ref item);
                    }
                }
            }
            if (!v4MaxXInside)
            {
                if (v1MaxXInside && contactData.Count < 8)
                {
                    //ComputeIntersection(ref face.V4, ref face.V1, ref clipEdge, out v);
                    //Vector3.Dot(ref clipY, ref v, out dot);
                    //if (dot > clipCenterMinY && dot < clipCenterMaxY)
                    if (ComputeIntersection(ref face.V4, ref face.V1, ref clipEdge, out v))
                    {
                        item.Position = v;
                        item.Id = GetContactId(face.Id4, face.Id1, ref clipEdge);
                        contactData.Add(ref item);
                    }
                }
                if (v3MaxXInside && contactData.Count < 8)
                {
                    //ComputeIntersection(ref face.V3, ref face.V4, ref clipEdge, out v);
                    //Vector3.Dot(ref clipY, ref v, out dot);
                    //if (dot > clipCenterMinY && dot < clipCenterMaxY)
                    if (ComputeIntersection(ref face.V3, ref face.V4, ref clipEdge, out v))
                    {
                        item.Position = v;
                        item.Id = GetContactId(face.Id3, face.Id4, ref clipEdge);
                        contactData.Add(ref item);
                    }
                }
            }

            #endregion

            //Compute depths.
            postClipCount = contactData.Count;
            tempData = contactData;
            for (int i = postClipCount - 1; i >= previousCount; i--)
                contactData.RemoveAt(i);

            for (int i = previousCount; i < tempData.Count; i++)
            {
                tempData.Get(i, out item);
                Vector3.Dot(ref item.Position, ref mtd, out faceDot);
                item.Depth = faceDot - clipFaceDot;
                if (item.Depth <= 0)
                {
                    contactData.Add(ref item);
                }
            }
        }
        //private static void ClipFacesDirect(ref BoxFace clipFace, ref BoxFace face, ref Vector3 mtd, out TinyStructList<BoxContactData> contactData)
        //{
        //    contactData = new TinyStructList<BoxContactData>();
        //    //BoxContactData* data = &contactData.d1;
        //    //BoxContactData* temp = &tempData.d1;

        //    //Local directions on the clip face.  Their length is equal to the length of an edge.
        //    Vector3 clipX, clipY;
        //    Vector3.Subtract(ref clipFace.V4, ref clipFace.V3, out clipX);
        //    Vector3.Subtract(ref clipFace.V2, ref clipFace.V3, out clipY);
        //    Fix64 inverse = 1 / clipX.LengthSquared();
        //    clipX.X *= inverse;
        //    clipX.Y *= inverse;
        //    clipX.Z *= inverse;
        //    inverse = 1 / clipY.LengthSquared();
        //    clipY.X *= inverse;
        //    clipY.Y *= inverse;
        //    clipY.Z *= inverse;

        //    //Local directions on the opposing face.  Their length is equal to the length of an edge.
        //    Vector3 faceX, faceY;
        //    Vector3.Subtract(ref face.V4, ref face.V3, out faceX);
        //    Vector3.Subtract(ref face.V2, ref face.V3, out faceY);
        //    inverse = 1 / faceX.LengthSquared();
        //    faceX.X *= inverse;
        //    faceX.Y *= inverse;
        //    faceX.Z *= inverse;
        //    inverse = 1 / faceY.LengthSquared();
        //    faceY.X *= inverse;
        //    faceY.Y *= inverse;
        //    faceY.Z *= inverse;

        //    Vector3 clipCenter;
        //    Vector3.Add(ref clipFace.V1, ref clipFace.V3, out clipCenter);
        //    //Defer division until after dot product (2 multiplies instead of 3)
        //    Fix64 clipCenterX, clipCenterY;
        //    Vector3.Dot(ref clipCenter, ref clipX, out clipCenterX);
        //    Vector3.Dot(ref clipCenter, ref clipY, out clipCenterY);
        //    clipCenterX *= .5f;
        //    clipCenterY *= .5f;

        //    Vector3 faceCenter;
        //    Vector3.Add(ref face.V1, ref face.V3, out faceCenter);
        //    //Defer division until after dot product (2 multiplies instead of 3)
        //    Fix64 faceCenterX, faceCenterY;
        //    Vector3.Dot(ref faceCenter, ref faceX, out faceCenterX);
        //    Vector3.Dot(ref faceCenter, ref faceY, out faceCenterY);
        //    faceCenterX *= .5f;
        //    faceCenterY *= .5f;

        //    //To test bounds, recall that clipX is the length of the X edge.
        //    //Going from the center to the max or min goes half of the length of X edge, or +/- 0.5.
        //    //Bias could be added here.
        //    Fix64 extent = .5f; //.5f is the default, extra could be added for robustness or speed.
        //    Fix64 clipCenterMaxX = clipCenterX + extent;
        //    Fix64 clipCenterMaxY = clipCenterY + extent;
        //    Fix64 clipCenterMinX = clipCenterX - extent;
        //    Fix64 clipCenterMinY = clipCenterY - extent;

        //    Fix64 faceCenterMaxX = faceCenterX + extent;
        //    Fix64 faceCenterMaxY = faceCenterY + extent;
        //    Fix64 faceCenterMinX = faceCenterX - extent;
        //    Fix64 faceCenterMinY = faceCenterY - extent;

        //    //Find out where the opposing face is.
        //    Fix64 dotX, dotY;

        //    //The four edges can be thought of as minX, maxX, minY and maxY.

        //    //Face v1
        //    Vector3.Dot(ref clipX, ref face.V1, out dotX);
        //    bool v1MaxXInside = dotX < clipCenterMaxX;
        //    bool v1MinXInside = dotX > clipCenterMinX;
        //    Vector3.Dot(ref clipY, ref face.V1, out dotY);
        //    bool v1MaxYInside = dotY < clipCenterMaxY;
        //    bool v1MinYInside = dotY > clipCenterMinY;

        //    //Face v2
        //    Vector3.Dot(ref clipX, ref face.V2, out dotX);
        //    bool v2MaxXInside = dotX < clipCenterMaxX;
        //    bool v2MinXInside = dotX > clipCenterMinX;
        //    Vector3.Dot(ref clipY, ref face.V2, out dotY);
        //    bool v2MaxYInside = dotY < clipCenterMaxY;
        //    bool v2MinYInside = dotY > clipCenterMinY;

        //    //Face v3
        //    Vector3.Dot(ref clipX, ref face.V3, out dotX);
        //    bool v3MaxXInside = dotX < clipCenterMaxX;
        //    bool v3MinXInside = dotX > clipCenterMinX;
        //    Vector3.Dot(ref clipY, ref face.V3, out dotY);
        //    bool v3MaxYInside = dotY < clipCenterMaxY;
        //    bool v3MinYInside = dotY > clipCenterMinY;

        //    //Face v4
        //    Vector3.Dot(ref clipX, ref face.V4, out dotX);
        //    bool v4MaxXInside = dotX < clipCenterMaxX;
        //    bool v4MinXInside = dotX > clipCenterMinX;
        //    Vector3.Dot(ref clipY, ref face.V4, out dotY);
        //    bool v4MaxYInside = dotY < clipCenterMaxY;
        //    bool v4MinYInside = dotY > clipCenterMinY;

        //    //Find out where the clip face is.
        //    //Clip v1
        //    Vector3.Dot(ref faceX, ref clipFace.V1, out dotX);
        //    bool clipv1MaxXInside = dotX < faceCenterMaxX;
        //    bool clipv1MinXInside = dotX > faceCenterMinX;
        //    Vector3.Dot(ref faceY, ref clipFace.V1, out dotY);
        //    bool clipv1MaxYInside = dotY < faceCenterMaxY;
        //    bool clipv1MinYInside = dotY > faceCenterMinY;

        //    //Clip v2
        //    Vector3.Dot(ref faceX, ref clipFace.V2, out dotX);
        //    bool clipv2MaxXInside = dotX < faceCenterMaxX;
        //    bool clipv2MinXInside = dotX > faceCenterMinX;
        //    Vector3.Dot(ref faceY, ref clipFace.V2, out dotY);
        //    bool clipv2MaxYInside = dotY < faceCenterMaxY;
        //    bool clipv2MinYInside = dotY > faceCenterMinY;

        //    //Clip v3
        //    Vector3.Dot(ref faceX, ref clipFace.V3, out dotX);
        //    bool clipv3MaxXInside = dotX < faceCenterMaxX;
        //    bool clipv3MinXInside = dotX > faceCenterMinX;
        //    Vector3.Dot(ref faceY, ref clipFace.V3, out dotY);
        //    bool clipv3MaxYInside = dotY < faceCenterMaxY;
        //    bool clipv3MinYInside = dotY > faceCenterMinY;

        //    //Clip v4
        //    Vector3.Dot(ref faceX, ref clipFace.V4, out dotX);
        //    bool clipv4MaxXInside = dotX < faceCenterMaxX;
        //    bool clipv4MinXInside = dotX > faceCenterMinX;
        //    Vector3.Dot(ref faceY, ref clipFace.V4, out dotY);
        //    bool clipv4MaxYInside = dotY < faceCenterMaxY;
        //    bool clipv4MinYInside = dotY > faceCenterMinY;

        //    var item = new BoxContactData();

        //    #region Face Vertices

        //    if (v1MinXInside && v1MaxXInside && v1MinYInside && v1MaxYInside)
        //    {
        //        item.Position = face.V1;
        //        item.Id = face.Id1;
        //        contactData.Add(ref item);
        //    }

        //    if (v2MinXInside && v2MaxXInside && v2MinYInside && v2MaxYInside)
        //    {
        //        item.Position = face.V2;
        //        item.Id = face.Id2;
        //        contactData.Add(ref item);
        //    }

        //    if (v3MinXInside && v3MaxXInside && v3MinYInside && v3MaxYInside)
        //    {
        //        item.Position = face.V3;
        //        item.Id = face.Id3;
        //        contactData.Add(ref item);
        //    }

        //    if (v4MinXInside && v4MaxXInside && v4MinYInside && v4MaxYInside)
        //    {
        //        item.Position = face.V4;
        //        item.Id = face.Id4;
        //        contactData.Add(ref item);
        //    }

        //    #endregion

        //    //Compute depths.
        //    TinyStructList<BoxContactData> tempData = contactData;
        //    contactData.Clear();
        //    Fix64 clipFaceDot, faceDot;
        //    Vector3.Dot(ref clipFace.V1, ref mtd, out clipFaceDot);
        //    for (int i = 0; i < tempData.Count; i++)
        //    {
        //        tempData.Get(i, out item);
        //        Vector3.Dot(ref item.Position, ref mtd, out faceDot);
        //        item.Depth = faceDot - clipFaceDot;
        //        if (item.Depth <= 0)
        //        {
        //            contactData.Add(ref item);
        //        }
        //    }

        //    int previousCount = contactData.Count;
        //    if (previousCount >= 4) //Early finish :)
        //    {
        //        return;
        //    }

        //    #region Clip face vertices

        //    Vector3 faceNormal;
        //    Vector3.Cross(ref faceY, ref faceX, out faceNormal);
        //    //inverse = 1 / faceNormal.LengthSquared();
        //    //faceNormal.X *= inverse;
        //    //faceNormal.Y *= inverse;
        //    //faceNormal.Z *= inverse;
        //    faceNormal.Normalize();
        //    Vector3 v;
        //    Fix64 a, b;
        //    Vector3.Dot(ref face.V1, ref faceNormal, out b);
        //    //CLIP FACE
        //    if (clipv1MinXInside && clipv1MaxXInside && clipv1MinYInside && clipv1MaxYInside)
        //    {
        //        Vector3.Dot(ref clipFace.V1, ref faceNormal, out a);
        //        Vector3.Multiply(ref faceNormal, a - b, out v);
        //        Vector3.Subtract(ref clipFace.V1, ref v, out v);
        //        item.Position = v;
        //        item.Id = clipFace.Id1 + 8;
        //        contactData.Add(ref item);
        //    }

        //    if (clipv2MinXInside && clipv2MaxXInside && clipv2MinYInside && clipv2MaxYInside)
        //    {
        //        Vector3.Dot(ref clipFace.V2, ref faceNormal, out a);
        //        Vector3.Multiply(ref faceNormal, a - b, out v);
        //        Vector3.Subtract(ref clipFace.V2, ref v, out v);
        //        item.Position = v;
        //        item.Id = clipFace.Id2 + 8;
        //        contactData.Add(ref item);
        //    }

        //    if (clipv3MinXInside && clipv3MaxXInside && clipv3MinYInside && clipv3MaxYInside)
        //    {
        //        Vector3.Dot(ref clipFace.V3, ref faceNormal, out a);
        //        Vector3.Multiply(ref faceNormal, a - b, out v);
        //        Vector3.Subtract(ref clipFace.V3, ref v, out v);
        //        item.Position = v;
        //        item.Id = clipFace.Id3 + 8;
        //        contactData.Add(ref item);
        //    }

        //    if (clipv4MinXInside && clipv4MaxXInside && clipv4MinYInside && clipv4MaxYInside)
        //    {
        //        Vector3.Dot(ref clipFace.V4, ref faceNormal, out a);
        //        Vector3.Multiply(ref faceNormal, a - b, out v);
        //        Vector3.Subtract(ref clipFace.V4, ref v, out v);
        //        item.Position = v;
        //        item.Id = clipFace.Id4 + 8;
        //        contactData.Add(ref item);
        //    }

        //    #endregion

        //    //Compute depths.
        //    int postClipCount = contactData.Count;
        //    tempData = contactData;
        //    for (int i = postClipCount - 1; i >= previousCount; i--) //TODO: >=?
        //        contactData.RemoveAt(i);


        //    for (int i = previousCount; i < tempData.Count; i++)
        //    {
        //        tempData.Get(i, out item);
        //        Vector3.Dot(ref item.Position, ref mtd, out faceDot);
        //        item.Depth = faceDot - clipFaceDot;
        //        if (item.Depth <= 0)
        //        {
        //            contactData.Add(ref item);
        //        }
        //    }

        //    previousCount = contactData.Count;
        //    if (previousCount >= 4) //Early finish :)
        //    {
        //        return;
        //    }

        //    //Intersect edges.

        //    //maxX maxY -> v1
        //    //minX maxY -> v2
        //    //minX minY -> v3
        //    //maxX minY -> v4

        //    //Once we get here there can only be 3 contacts or less.
        //    //Once 4 possible contacts have been added, switch to using safe increments.
        //    Fix64 dot;

        //    #region CLIP EDGE: v1 v2

        //    FaceEdge clipEdge;
        //    clipFace.GetEdge(0, ref mtd, out clipEdge);
        //    if (!v1MaxYInside)
        //    {
        //        if (v2MaxYInside)
        //        {
        //            ComputeIntersection(ref face.V1, ref face.V2, ref clipEdge, out v);
        //            Vector3.Dot(ref clipX, ref v, out dot);
        //            if (dot > clipCenterMinX && dot < clipCenterMaxX)
        //            {
        //                item.Position = v;
        //                item.Id = GetContactId(face.Id1, face.Id2, ref clipEdge);
        //                contactData.Add(ref item);
        //            }
        //        }
        //        if (v4MaxYInside)
        //        {
        //            ComputeIntersection(ref face.V4, ref face.V1, ref clipEdge, out v);
        //            Vector3.Dot(ref clipX, ref v, out dot);
        //            if (dot > clipCenterMinX && dot < clipCenterMaxX)
        //            {
        //                item.Position = v;
        //                item.Id = GetContactId(face.Id4, face.Id1, ref clipEdge);
        //                contactData.Add(ref item);
        //            }
        //        }
        //    }
        //    if (!v2MaxYInside)
        //    {
        //        if (v1MaxYInside)
        //        {
        //            ComputeIntersection(ref face.V1, ref face.V2, ref clipEdge, out v);
        //            Vector3.Dot(ref clipX, ref v, out dot);
        //            if (dot > clipCenterMinX && dot < clipCenterMaxX)
        //            {
        //                item.Position = v;
        //                item.Id = GetContactId(face.Id1, face.Id2, ref clipEdge);
        //                contactData.Add(ref item);
        //            }
        //        }
        //        if (v3MaxYInside)
        //        {
        //            ComputeIntersection(ref face.V2, ref face.V3, ref clipEdge, out v);
        //            Vector3.Dot(ref clipX, ref v, out dot);
        //            if (dot > clipCenterMinX && dot < clipCenterMaxX)
        //            {
        //                item.Position = v;
        //                item.Id = GetContactId(face.Id2, face.Id3, ref clipEdge);
        //                contactData.Add(ref item);
        //            }
        //        }
        //    }
        //    if (!v3MaxYInside)
        //    {
        //        if (v2MaxYInside)
        //        {
        //            ComputeIntersection(ref face.V2, ref face.V3, ref clipEdge, out v);
        //            Vector3.Dot(ref clipX, ref v, out dot);
        //            if (dot > clipCenterMinX && dot < clipCenterMaxX)
        //            {
        //                item.Position = v;
        //                item.Id = GetContactId(face.Id2, face.Id3, ref clipEdge);
        //                contactData.Add(ref item);
        //            }
        //        }
        //        if (v4MaxYInside && contactData.Count < 8)
        //        {
        //            ComputeIntersection(ref face.V3, ref face.V4, ref clipEdge, out v);
        //            Vector3.Dot(ref clipX, ref v, out dot);
        //            if (dot > clipCenterMinX && dot < clipCenterMaxX)
        //            {
        //                item.Position = v;
        //                item.Id = GetContactId(face.Id3, face.Id4, ref clipEdge);
        //                contactData.Add(ref item);
        //            }
        //        }
        //    }
        //    if (!v4MaxYInside)
        //    {
        //        if (v1MaxYInside && contactData.Count < 8)
        //        {
        //            ComputeIntersection(ref face.V4, ref face.V1, ref clipEdge, out v);
        //            Vector3.Dot(ref clipX, ref v, out dot);
        //            if (dot > clipCenterMinX && dot < clipCenterMaxX)
        //            {
        //                item.Position = v;
        //                item.Id = GetContactId(face.Id4, face.Id1, ref clipEdge);
        //                contactData.Add(ref item);
        //            }
        //        }
        //        if (v3MaxYInside && contactData.Count < 8)
        //        {
        //            ComputeIntersection(ref face.V3, ref face.V4, ref clipEdge, out v);
        //            Vector3.Dot(ref clipX, ref v, out dot);
        //            if (dot > clipCenterMinX && dot < clipCenterMaxX)
        //            {
        //                item.Position = v;
        //                item.Id = GetContactId(face.Id3, face.Id4, ref clipEdge);
        //                contactData.Add(ref item);
        //            }
        //        }
        //    }

        //    #endregion

        //    #region CLIP EDGE: v2 v3

        //    clipFace.GetEdge(1, ref mtd, out clipEdge);
        //    if (!v1MinXInside)
        //    {
        //        if (v2MinXInside && contactData.Count < 8)
        //        {
        //            //test v1-v2 against minXminY-minXmaxY
        //            ComputeIntersection(ref face.V1, ref face.V2, ref clipEdge, out v);
        //            Vector3.Dot(ref clipY, ref v, out dot);
        //            if (dot > clipCenterMinY && dot < clipCenterMaxY)
        //            {
        //                item.Position = v;
        //                item.Id = GetContactId(face.Id1, face.Id2, ref clipEdge);
        //                contactData.Add(ref item);
        //            }
        //        }
        //        if (v4MinXInside && contactData.Count < 8)
        //        {
        //            //test v1-v3 against minXminY-minXmaxY
        //            ComputeIntersection(ref face.V4, ref face.V1, ref clipEdge, out v);
        //            Vector3.Dot(ref clipY, ref v, out dot);
        //            if (dot > clipCenterMinY && dot < clipCenterMaxY)
        //            {
        //                item.Position = v;
        //                item.Id = GetContactId(face.Id4, face.Id1, ref clipEdge);
        //                contactData.Add(ref item);
        //            }
        //        }
        //    }
        //    if (!v2MinXInside)
        //    {
        //        if (v1MinXInside && contactData.Count < 8)
        //        {
        //            ComputeIntersection(ref face.V1, ref face.V2, ref clipEdge, out v);
        //            Vector3.Dot(ref clipY, ref v, out dot);
        //            if (dot > clipCenterMinY && dot < clipCenterMaxY)
        //            {
        //                item.Position = v;
        //                item.Id = GetContactId(face.Id1, face.Id2, ref clipEdge);
        //                contactData.Add(ref item);
        //            }
        //        }
        //        if (v3MinXInside && contactData.Count < 8)
        //        {
        //            ComputeIntersection(ref face.V2, ref face.V3, ref clipEdge, out v);
        //            Vector3.Dot(ref clipY, ref v, out dot);
        //            if (dot > clipCenterMinY && dot < clipCenterMaxY)
        //            {
        //                item.Position = v;
        //                item.Id = GetContactId(face.Id2, face.Id3, ref clipEdge);
        //                contactData.Add(ref item);
        //            }
        //        }
        //    }
        //    if (!v3MinXInside)
        //    {
        //        if (v2MinXInside && contactData.Count < 8)
        //        {
        //            ComputeIntersection(ref face.V2, ref face.V3, ref clipEdge, out v);
        //            Vector3.Dot(ref clipY, ref v, out dot);
        //            if (dot > clipCenterMinY && dot < clipCenterMaxY)
        //            {
        //                item.Position = v;
        //                item.Id = GetContactId(face.Id2, face.Id3, ref clipEdge);
        //                contactData.Add(ref item);
        //            }
        //        }
        //        if (v4MinXInside && contactData.Count < 8)
        //        {
        //            ComputeIntersection(ref face.V3, ref face.V4, ref clipEdge, out v);
        //            Vector3.Dot(ref clipY, ref v, out dot);
        //            if (dot > clipCenterMinY && dot < clipCenterMaxY)
        //            {
        //                item.Position = v;
        //                item.Id = GetContactId(face.Id3, face.Id4, ref clipEdge);
        //                contactData.Add(ref item);
        //            }
        //        }
        //    }
        //    if (!v4MinXInside)
        //    {
        //        if (v1MinXInside && contactData.Count < 8)
        //        {
        //            //test v2-v4 against minXminY-minXmaxY
        //            ComputeIntersection(ref face.V4, ref face.V1, ref clipEdge, out v);
        //            Vector3.Dot(ref clipY, ref v, out dot);
        //            if (dot > clipCenterMinY && dot < clipCenterMaxY)
        //            {
        //                item.Position = v;
        //                item.Id = GetContactId(face.Id4, face.Id1, ref clipEdge);
        //                contactData.Add(ref item);
        //            }
        //        }
        //        if (v3MinXInside && contactData.Count < 8)
        //        {
        //            //test v3-v4 against minXminY-minXmaxY
        //            ComputeIntersection(ref face.V3, ref face.V4, ref clipEdge, out v);
        //            Vector3.Dot(ref clipY, ref v, out dot);
        //            if (dot > clipCenterMinY && dot < clipCenterMaxY)
        //            {
        //                item.Position = v;
        //                item.Id = GetContactId(face.Id3, face.Id4, ref clipEdge);
        //                contactData.Add(ref item);
        //            }
        //        }
        //    }

        //    #endregion

        //    #region CLIP EDGE: v3 v4

        //    clipFace.GetEdge(2, ref mtd, out clipEdge);
        //    if (!v1MinYInside)
        //    {
        //        if (v2MinYInside && contactData.Count < 8)
        //        {
        //            ComputeIntersection(ref face.V1, ref face.V2, ref clipEdge, out v);
        //            Vector3.Dot(ref clipX, ref v, out dot);
        //            if (dot > clipCenterMinX && dot < clipCenterMaxX)
        //            {
        //                item.Position = v;
        //                item.Id = GetContactId(face.Id1, face.Id2, ref clipEdge);
        //                contactData.Add(ref item);
        //            }
        //        }
        //        if (v4MinYInside && contactData.Count < 8)
        //        {
        //            ComputeIntersection(ref face.V4, ref face.V1, ref clipEdge, out v);
        //            Vector3.Dot(ref clipX, ref v, out dot);
        //            if (dot > clipCenterMinX && dot < clipCenterMaxX)
        //            {
        //                item.Position = v;
        //                item.Id = GetContactId(face.Id4, face.Id1, ref clipEdge);
        //                contactData.Add(ref item);
        //            }
        //        }
        //    }
        //    if (!v2MinYInside)
        //    {
        //        if (v1MinYInside && contactData.Count < 8)
        //        {
        //            ComputeIntersection(ref face.V1, ref face.V2, ref clipEdge, out v);
        //            Vector3.Dot(ref clipX, ref v, out dot);
        //            if (dot > clipCenterMinX && dot < clipCenterMaxX)
        //            {
        //                item.Position = v;
        //                item.Id = GetContactId(face.Id1, face.Id2, ref clipEdge);
        //                contactData.Add(ref item);
        //            }
        //        }
        //        if (v3MinYInside && contactData.Count < 8)
        //        {
        //            ComputeIntersection(ref face.V2, ref face.V3, ref clipEdge, out v);
        //            Vector3.Dot(ref clipX, ref v, out dot);
        //            if (dot > clipCenterMinX && dot < clipCenterMaxX)
        //            {
        //                item.Position = v;
        //                item.Id = GetContactId(face.Id2, face.Id3, ref clipEdge);
        //                contactData.Add(ref item);
        //            }
        //        }
        //    }
        //    if (!v3MinYInside)
        //    {
        //        if (v2MinYInside && contactData.Count < 8)
        //        {
        //            ComputeIntersection(ref face.V2, ref face.V3, ref clipEdge, out v);
        //            Vector3.Dot(ref clipX, ref v, out dot);
        //            if (dot > clipCenterMinX && dot < clipCenterMaxX)
        //            {
        //                item.Position = v;
        //                item.Id = GetContactId(face.Id2, face.Id3, ref clipEdge);
        //                contactData.Add(ref item);
        //            }
        //        }
        //        if (v4MinYInside && contactData.Count < 8)
        //        {
        //            ComputeIntersection(ref face.V3, ref face.V4, ref clipEdge, out v);
        //            Vector3.Dot(ref clipX, ref v, out dot);
        //            if (dot > clipCenterMinX && dot < clipCenterMaxX)
        //            {
        //                item.Position = v;
        //                item.Id = GetContactId(face.Id3, face.Id4, ref clipEdge);
        //                contactData.Add(ref item);
        //            }
        //        }
        //    }
        //    if (!v4MinYInside)
        //    {
        //        if (v3MinYInside && contactData.Count < 8)
        //        {
        //            ComputeIntersection(ref face.V3, ref face.V4, ref clipEdge, out v);
        //            Vector3.Dot(ref clipX, ref v, out dot);
        //            if (dot > clipCenterMinX && dot < clipCenterMaxX)
        //            {
        //                item.Position = v;
        //                item.Id = GetContactId(face.Id3, face.Id4, ref clipEdge);
        //                contactData.Add(ref item);
        //            }
        //        }
        //        if (v1MinYInside && contactData.Count < 8)
        //        {
        //            ComputeIntersection(ref face.V4, ref face.V1, ref clipEdge, out v);
        //            Vector3.Dot(ref clipX, ref v, out dot);
        //            if (dot > clipCenterMinX && dot < clipCenterMaxX)
        //            {
        //                item.Position = v;
        //                item.Id = GetContactId(face.Id4, face.Id1, ref clipEdge);
        //                contactData.Add(ref item);
        //            }
        //        }
        //    }

        //    #endregion

        //    #region CLIP EDGE: v4 v1

        //    clipFace.GetEdge(3, ref mtd, out clipEdge);
        //    if (!v1MaxXInside)
        //    {
        //        if (v2MaxXInside && contactData.Count < 8)
        //        {
        //            ComputeIntersection(ref face.V1, ref face.V2, ref clipEdge, out v);
        //            Vector3.Dot(ref clipY, ref v, out dot);
        //            if (dot > clipCenterMinY && dot < clipCenterMaxY)
        //            {
        //                item.Position = v;
        //                item.Id = GetContactId(face.Id1, face.Id2, ref clipEdge);
        //                contactData.Add(ref item);
        //            }
        //        }
        //        if (v4MaxXInside && contactData.Count < 8)
        //        {
        //            ComputeIntersection(ref face.V4, ref face.V1, ref clipEdge, out v);
        //            Vector3.Dot(ref clipY, ref v, out dot);
        //            if (dot > clipCenterMinY && dot < clipCenterMaxY)
        //            {
        //                item.Position = v;
        //                item.Id = GetContactId(face.Id4, face.Id1, ref clipEdge);
        //                contactData.Add(ref item);
        //            }
        //        }
        //    }
        //    if (!v2MaxXInside)
        //    {
        //        if (v1MaxXInside && contactData.Count < 8)
        //        {
        //            ComputeIntersection(ref face.V1, ref face.V2, ref clipEdge, out v);
        //            Vector3.Dot(ref clipY, ref v, out dot);
        //            if (dot > clipCenterMinY && dot < clipCenterMaxY)
        //            {
        //                item.Position = v;
        //                item.Id = GetContactId(face.Id1, face.Id2, ref clipEdge);
        //                contactData.Add(ref item);
        //            }
        //        }
        //        if (v3MaxXInside && contactData.Count < 8)
        //        {
        //            ComputeIntersection(ref face.V2, ref face.V3, ref clipEdge, out v);
        //            Vector3.Dot(ref clipY, ref v, out dot);
        //            if (dot > clipCenterMinY && dot < clipCenterMaxY)
        //            {
        //                item.Position = v;
        //                item.Id = GetContactId(face.Id2, face.Id3, ref clipEdge);
        //                contactData.Add(ref item);
        //            }
        //        }
        //    }
        //    if (!v3MaxXInside)
        //    {
        //        if (v2MaxXInside && contactData.Count < 8)
        //        {
        //            ComputeIntersection(ref face.V2, ref face.V3, ref clipEdge, out v);
        //            Vector3.Dot(ref clipY, ref v, out dot);
        //            if (dot > clipCenterMinY && dot < clipCenterMaxY)
        //            {
        //                item.Position = v;
        //                item.Id = GetContactId(face.Id2, face.Id3, ref clipEdge);
        //                contactData.Add(ref item);
        //            }
        //        }
        //        if (v4MaxXInside && contactData.Count < 8)
        //        {
        //            ComputeIntersection(ref face.V3, ref face.V4, ref clipEdge, out v);
        //            Vector3.Dot(ref clipY, ref v, out dot);
        //            if (dot > clipCenterMinY && dot < clipCenterMaxY)
        //            {
        //                item.Position = v;
        //                item.Id = GetContactId(face.Id3, face.Id4, ref clipEdge);
        //                contactData.Add(ref item);
        //            }
        //        }
        //    }
        //    if (!v4MaxXInside)
        //    {
        //        if (v1MaxXInside && contactData.Count < 8)
        //        {
        //            ComputeIntersection(ref face.V4, ref face.V1, ref clipEdge, out v);
        //            Vector3.Dot(ref clipY, ref v, out dot);
        //            if (dot > clipCenterMinY && dot < clipCenterMaxY)
        //            {
        //                item.Position = v;
        //                item.Id = GetContactId(face.Id4, face.Id1, ref clipEdge);
        //                contactData.Add(ref item);
        //            }
        //        }
        //        if (v3MaxXInside && contactData.Count < 8)
        //        {
        //            ComputeIntersection(ref face.V3, ref face.V4, ref clipEdge, out v);
        //            Vector3.Dot(ref clipY, ref v, out dot);
        //            if (dot > clipCenterMinY && dot < clipCenterMaxY)
        //            {
        //                item.Position = v;
        //                item.Id = GetContactId(face.Id3, face.Id4, ref clipEdge);
        //                contactData.Add(ref item);
        //            }
        //        }
        //    }

        //    #endregion

        //    //Compute depths.
        //    postClipCount = contactData.Count;
        //    tempData = contactData;
        //    for (int i = postClipCount - 1; i >= previousCount; i--)
        //        contactData.RemoveAt(i);

        //    for (int i = previousCount; i < tempData.Count; i++)
        //    {
        //        tempData.Get(i, out item);
        //        Vector3.Dot(ref item.Position, ref mtd, out faceDot);
        //        item.Depth = faceDot - clipFaceDot;
        //        if (item.Depth <= 0)
        //        {
        //            contactData.Add(ref item);
        //        }
        //    }
        //}
#endif

        private static bool ComputeIntersection(ref Vector3 edgeA1, ref Vector3 edgeA2, ref FaceEdge clippingEdge, out Vector3 intersection)
        {
            //Intersect the incoming edge (edgeA1, edgeA2) with the clipping edge's PLANE.  Nicely given by one of its positions and its 'perpendicular,'
            //which is its normal.

            Vector3 offset;
            Vector3.Subtract(ref clippingEdge.A, ref edgeA1, out offset);

            Vector3 edgeDirection;
            Vector3.Subtract(ref edgeA2, ref edgeA1, out edgeDirection);
            Fix64 distanceToPlane;
            Vector3.Dot(ref offset, ref clippingEdge.Perpendicular, out distanceToPlane);
            Fix64 edgeDirectionLength;
            Vector3.Dot(ref edgeDirection, ref clippingEdge.Perpendicular, out edgeDirectionLength);
            Fix64 t = distanceToPlane / edgeDirectionLength;
            if (t < F64.C0 || t > F64.C1)
            {
                //It's outside of the incoming edge!
                intersection = new Vector3();
                return false;
            }
            Vector3.Multiply(ref edgeDirection, t, out offset);
            Vector3.Add(ref offset, ref edgeA1, out intersection);

            Vector3.Subtract(ref intersection, ref clippingEdge.A, out offset);
            Vector3.Subtract(ref clippingEdge.B, ref clippingEdge.A, out edgeDirection);
            Vector3.Dot(ref edgeDirection, ref offset, out t);
            if (t < F64.C0 || t > edgeDirection.LengthSquared())
            {
                //It's outside of the clipping edge!
                return false;
            }
            return true;
        }

        private static void GetNearestFace(ref Vector3 position, ref Matrix3x3 orientation, ref Vector3 mtd, Fix64 halfWidth, Fix64 halfHeight, Fix64 halfLength, out BoxFace boxFace)
        {
            boxFace = new BoxFace();

            Fix64 xDot = orientation.M11 * mtd.X +
                         orientation.M12 * mtd.Y +
                         orientation.M13 * mtd.Z;
            Fix64 yDot = orientation.M21 * mtd.X +
                         orientation.M22 * mtd.Y +
                         orientation.M23 * mtd.Z;
            Fix64 zDot = orientation.M31 * mtd.X +
                         orientation.M32 * mtd.Y +
                         orientation.M33 * mtd.Z;

            Fix64 absX = Fix64.Abs(xDot);
            Fix64 absY = Fix64.Abs(yDot);
            Fix64 absZ = Fix64.Abs(zDot);

            Matrix worldTransform;
            Matrix3x3.ToMatrix4X4(ref orientation, out worldTransform);
            worldTransform.M41 = position.X;
            worldTransform.M42 = position.Y;
            worldTransform.M43 = position.Z;
            worldTransform.M44 = F64.C1;

            Vector3 candidate;
            int bit;
            if (absX > absY && absX > absZ)
            {
                //"X" faces are candidates
                if (xDot < F64.C0)
                {
                    halfWidth = -halfWidth;
                    bit = 0;
                }
                else
                    bit = 1;
                candidate = new Vector3(halfWidth, halfHeight, halfLength);
                Matrix.Transform(ref candidate, ref worldTransform, out candidate);
                boxFace.V1 = candidate;
                candidate = new Vector3(halfWidth, -halfHeight, halfLength);
                Matrix.Transform(ref candidate, ref worldTransform, out candidate);
                boxFace.V2 = candidate;
                candidate = new Vector3(halfWidth, -halfHeight, -halfLength);
                Matrix.Transform(ref candidate, ref worldTransform, out candidate);
                boxFace.V3 = candidate;
                candidate = new Vector3(halfWidth, halfHeight, -halfLength);
                Matrix.Transform(ref candidate, ref worldTransform, out candidate);
                boxFace.V4 = candidate;

                if (xDot < F64.C0)
                    boxFace.Normal = orientation.Left;
                else
                    boxFace.Normal = orientation.Right;

                boxFace.Width = halfHeight * F64.C2;
                boxFace.Height = halfLength * F64.C2;

                boxFace.Id1 = bit + 2 + 4;
                boxFace.Id2 = bit + 4;
                boxFace.Id3 = bit + 2;
                boxFace.Id4 = bit;
            }
            else if (absY > absX && absY > absZ)
            {
                //"Y" faces are candidates
                if (yDot < F64.C0)
                {
                    halfHeight = -halfHeight;
                    bit = 0;
                }
                else
                    bit = 2;
                candidate = new Vector3(halfWidth, halfHeight, halfLength);
                Matrix.Transform(ref candidate, ref worldTransform, out candidate);
                boxFace.V1 = candidate;
                candidate = new Vector3(-halfWidth, halfHeight, halfLength);
                Matrix.Transform(ref candidate, ref worldTransform, out candidate);
                boxFace.V2 = candidate;
                candidate = new Vector3(-halfWidth, halfHeight, -halfLength);
                Matrix.Transform(ref candidate, ref worldTransform, out candidate);
                boxFace.V3 = candidate;
                candidate = new Vector3(halfWidth, halfHeight, -halfLength);
                Matrix.Transform(ref candidate, ref worldTransform, out candidate);
                boxFace.V4 = candidate;

                if (yDot < F64.C0)
                    boxFace.Normal = orientation.Down;
                else
                    boxFace.Normal = orientation.Up;

                boxFace.Width = halfWidth * F64.C2;
                boxFace.Height = halfLength * F64.C2;

                boxFace.Id1 = 1 + bit + 4;
                boxFace.Id2 = bit + 4;
                boxFace.Id3 = 1 + bit;
                boxFace.Id4 = bit;
            }
            else if (absZ > absX && absZ > absY)
            {
                //"Z" faces are candidates
                if (zDot < F64.C0)
                {
                    halfLength = -halfLength;
                    bit = 0;
                }
                else
                    bit = 4;
                candidate = new Vector3(halfWidth, halfHeight, halfLength);
                Matrix.Transform(ref candidate, ref worldTransform, out candidate);
                boxFace.V1 = candidate;
                candidate = new Vector3(-halfWidth, halfHeight, halfLength);
                Matrix.Transform(ref candidate, ref worldTransform, out candidate);
                boxFace.V2 = candidate;
                candidate = new Vector3(-halfWidth, -halfHeight, halfLength);
                Matrix.Transform(ref candidate, ref worldTransform, out candidate);
                boxFace.V3 = candidate;
                candidate = new Vector3(halfWidth, -halfHeight, halfLength);
                Matrix.Transform(ref candidate, ref worldTransform, out candidate);
                boxFace.V4 = candidate;

                if (zDot < F64.C0)
                    boxFace.Normal = orientation.Forward;
                else
                    boxFace.Normal = orientation.Backward;

                boxFace.Width = halfWidth * F64.C2;
                boxFace.Height = halfHeight * F64.C2;

                boxFace.Id1 = 1 + 2 + bit;
                boxFace.Id2 = 2 + bit;
                boxFace.Id3 = 1 + bit;
                boxFace.Id4 = bit;
            }
        }


        private struct BoxFace
        {
            public int Id1, Id2, Id3, Id4;
            public Vector3 V1, V2, V3, V4;
            public Vector3 Normal;
            public Fix64 Width, Height;

            public int GetId(int i)
            {
                switch (i)
                {
                    case 0:
                        return Id1;
                    case 1:
                        return Id2;
                    case 2:
                        return Id3;
                    case 3:
                        return Id4;
                }
                return -1;
            }

            public void GetVertex(int i, out Vector3 v)
            {
                switch (i)
                {
                    case 0:
                        v = V1;
                        return;
                    case 1:
                        v = V2;
                        return;
                    case 2:
                        v = V3;
                        return;
                    case 3:
                        v = V4;
                        return;
                }
                v = Toolbox.NoVector;
            }

            internal void GetEdge(int i, out FaceEdge clippingEdge)
            {
                Vector3 insidePoint;
                switch (i)
                {
                    case 0:
                        clippingEdge.A = V1;
                        clippingEdge.B = V2;
                        insidePoint = V3;
                        clippingEdge.Id = GetEdgeId(Id1, Id2);
                        break;
                    case 1:
                        clippingEdge.A = V2;
                        clippingEdge.B = V3;
                        insidePoint = V4;
                        clippingEdge.Id = GetEdgeId(Id2, Id3);
                        break;
                    case 2:
                        clippingEdge.A = V3;
                        clippingEdge.B = V4;
                        insidePoint = V1;
                        clippingEdge.Id = GetEdgeId(Id3, Id4);
                        break;
                    case 3:
                        clippingEdge.A = V4;
                        clippingEdge.B = V1;
                        insidePoint = V2;
                        clippingEdge.Id = GetEdgeId(Id4, Id1);
                        break;
                    default:
                        throw new IndexOutOfRangeException();
                }
                //TODO: Edge direction and perpendicular not normalized.
                Vector3 edgeDirection;
                Vector3.Subtract(ref clippingEdge.B, ref clippingEdge.A, out edgeDirection);
                edgeDirection.Normalize();
                Vector3.Cross(ref edgeDirection, ref Normal, out clippingEdge.Perpendicular);

                Fix64 dot;
                Vector3 offset;
                Vector3.Subtract(ref insidePoint, ref clippingEdge.A, out offset);
                Vector3.Dot(ref clippingEdge.Perpendicular, ref offset, out dot);
                if (dot > F64.C0)
                {
                    clippingEdge.Perpendicular.X = -clippingEdge.Perpendicular.X;
                    clippingEdge.Perpendicular.Y = -clippingEdge.Perpendicular.Y;
                    clippingEdge.Perpendicular.Z = -clippingEdge.Perpendicular.Z;
                }
                Vector3.Dot(ref clippingEdge.A, ref clippingEdge.Perpendicular, out clippingEdge.EdgeDistance);
            }
        }

        private static int GetContactId(int vertexAEdgeA, int vertexBEdgeA, int vertexAEdgeB, int vertexBEdgeB)
        {
            return GetEdgeId(vertexAEdgeA, vertexBEdgeA) * 2549 + GetEdgeId(vertexAEdgeB, vertexBEdgeB) * 2857;
        }

        private static int GetContactId(int vertexAEdgeA, int vertexBEdgeA, ref FaceEdge clippingEdge)
        {
            return GetEdgeId(vertexAEdgeA, vertexBEdgeA) * 2549 + clippingEdge.Id * 2857;
        }

        private static int GetEdgeId(int id1, int id2)
        {
            return (id1 + 1) * 571 + (id2 + 1) * 577;
        }

        private struct FaceEdge : IEquatable<FaceEdge>
        {
            public Vector3 A, B;
            public Fix64 EdgeDistance;
            public int Id;
            public Vector3 Perpendicular;

            #region IEquatable<FaceEdge> Members

            public bool Equals(FaceEdge other)
            {
                return other.Id == Id;
            }

            #endregion

            public bool IsPointInside(ref Vector3 point)
            {
                Fix64 distance;
                Vector3.Dot(ref point, ref Perpendicular, out distance);
                return distance < EdgeDistance; // +1; //TODO: Bias this a little?
            }
        }
    }
}