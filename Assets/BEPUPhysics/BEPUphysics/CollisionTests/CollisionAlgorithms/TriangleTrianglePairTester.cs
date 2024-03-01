﻿using BEPUphysics.CollisionShapes.ConvexShapes;
using BEPUutilities;
using BEPUutilities.DataStructures;
using FixMath.NET;

namespace BEPUphysics.CollisionTests.CollisionAlgorithms
{
    /// <summary>
    /// Generates candidates between two triangles and manages the persistent state of the pair.
    /// </summary>
    public class TriangleTrianglePairTester : TriangleConvexPairTester
    {
        //TODO: Having a specialized triangle-triangle pair test would be nice.  Even if it didn't use an actual triangle-triangle test, certain assumptions could still make it speedier and more elegant.
        //"Closest points between triangles" + persistent manifolding would probably be the best approach (a lot faster than the triangle-convex general case anyway).
        public override bool GenerateContactCandidates(TriangleShape triangle, out TinyStructList<ContactData> contactList)
        {
            if (base.GenerateContactCandidates(triangle, out contactList))
            {
                //The triangle-convex pair test has already rejected contacts whose normals would violate the first triangle's sidedness.
                //However, since it's a vanilla triangle-convex test, it doesn't know about the sidedness of the other triangle!
                var shape = ((TriangleShape)convex);
                Vector3 normal;
                //Lots of recalculating ab-bc!
                Vector3 ab, ac;
                Vector3.Subtract(ref shape.vB, ref shape.vA, out ab);
                Vector3.Subtract(ref shape.vC, ref shape.vA, out ac);
                Vector3.Cross(ref ab, ref ac, out normal);
                var sidedness = shape.sidedness;
                if (sidedness != TriangleSidedness.DoubleSided)
                {
                    for (int i = contactList.Count - 1; i >= 0; i--)
                    {
                        ContactData item;
                        contactList.Get(i, out item);

                        Fix64 dot;
                        Vector3.Dot(ref item.Normal, ref normal, out dot);
                        if (sidedness == TriangleSidedness.Clockwise)
                        {
                            if (dot < F64.C0)
                            {
                                contactList.RemoveAt(i);
                            }
                        }
                        else
                        {
                            if (dot > F64.C0)
                            {
                                contactList.RemoveAt(i);
                            }
                        }
                    }
                }
                return contactList.Count > 0;
            }
            return false;
        }
    }
}
