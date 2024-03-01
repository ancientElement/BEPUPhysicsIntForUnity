using System;
using AE_BEPUPhysics_Addition;
using FixMath.NET;
using UnityEngine;
using Vector3 = UnityEngine.Vector3;

namespace DefaultNamespace
{
    public class Test : MonoBehaviour
    {
        private AEPhysicsMgr mPhysicsMgr;
        [SerializeField] private AEBoxCollider m_box;
        [SerializeField] private AEBoxCollider m_plane;
        [SerializeField] private Vector3 m_grivity;

        private void Awake()
        {
            Physics.autoSimulation = false; // 关闭原来物理引擎迭代;
            Physics.autoSyncTransforms = false; // 关闭射线检测功能4
            mPhysicsMgr =
                new AEPhysicsMgr(new BEPUutilities.Vector3((Fix64)m_grivity.x, (Fix64)m_grivity.y,
                    (Fix64)m_grivity.x));
            mPhysicsMgr.RegisterColloder(m_box);
            mPhysicsMgr.RegisterColloder(m_plane);
        }

        private void FixedUpdate()
        {
            mPhysicsMgr.PhysicsUpdate(Time.fixedDeltaTime);
        }
    }
}