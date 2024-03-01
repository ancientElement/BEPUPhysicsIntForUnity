﻿using System.Collections.Generic;
using BEPUutilities.DataStructures;
using BEPUutilities;
using FixMath.NET;

namespace BEPUutilities.ResourceManagement
{
    /// <summary>
    /// Handles allocation and management of commonly used resources.
    /// </summary>
    public static class CommonResources
    {
        static CommonResources()
        {
            ResetPools();
        }

        public static void ResetPools()
        {
            SubPoolIntList = new LockingResourcePool<RawList<int>>();
            SubPoolIntSet = new LockingResourcePool<HashSet<int>>();
            SubPoolFloatList = new LockingResourcePool<RawList<Fix64>>();
            SubPoolVectorList = new LockingResourcePool<RawList<Vector3>>();
            SubPoolRayHitList = new LockingResourcePool<RawList<RayHit>>();

        }

        static LockingResourcePool<RawList<RayHit>> SubPoolRayHitList;
        static LockingResourcePool<RawList<int>> SubPoolIntList;
        static LockingResourcePool<HashSet<int>> SubPoolIntSet;
        static LockingResourcePool<RawList<Fix64>> SubPoolFloatList;
        static LockingResourcePool<RawList<Vector3>> SubPoolVectorList;

        /// <summary>
        /// Retrieves a ray hit list from the resource pool.
        /// </summary>
        /// <returns>Empty ray hit list.</returns>
        public static RawList<RayHit> GetRayHitList()
        {
            return SubPoolRayHitList.Take();
        }

        /// <summary>
        /// Returns a resource to the pool.
        /// </summary>
        /// <param name="list">List to return.</param>
        public static void GiveBack(RawList<RayHit> list)
        {
            list.Clear();
            SubPoolRayHitList.GiveBack(list);
        }

        

        /// <summary>
        /// Retrieves a int list from the resource pool.
        /// </summary>
        /// <returns>Empty int list.</returns>
        public static RawList<int> GetIntList()
        {
            return SubPoolIntList.Take();
        }

        /// <summary>
        /// Returns a resource to the pool.
        /// </summary>
        /// <param name="list">List to return.</param>
        public static void GiveBack(RawList<int> list)
        {
            list.Clear();
            SubPoolIntList.GiveBack(list);
        }

        /// <summary>
        /// Retrieves a int hash set from the resource pool.
        /// </summary>
        /// <returns>Empty int set.</returns>
        public static HashSet<int> GetIntSet()
        {
            return SubPoolIntSet.Take();
        }

        /// <summary>
        /// Returns a resource to the pool.
        /// </summary>
        /// <param name="set">Set to return.</param>
        public static void GiveBack(HashSet<int> set)
        {
            set.Clear();
            SubPoolIntSet.GiveBack(set);
        }

        /// <summary>
        /// Retrieves a float list from the resource pool.
        /// </summary>
        /// <returns>Empty float list.</returns>
        public static RawList<Fix64> GetFloatList()
        {
            return SubPoolFloatList.Take();
        }

        /// <summary>
        /// Returns a resource to the pool.
        /// </summary>
        /// <param name="list">List to return.</param>
        public static void GiveBack(RawList<Fix64> list)
        {
            list.Clear();
            SubPoolFloatList.GiveBack(list);
        }

        /// <summary>
        /// Retrieves a Vector3 list from the resource pool.
        /// </summary>
        /// <returns>Empty Vector3 list.</returns>
        public static RawList<Vector3> GetVectorList()
        {
            return SubPoolVectorList.Take();
        }

        /// <summary>
        /// Returns a resource to the pool.
        /// </summary>
        /// <param name="list">List to return.</param>
        public static void GiveBack(RawList<Vector3> list)
        {
            list.Clear();
            SubPoolVectorList.GiveBack(list);
        }

       
    }
}