using System.Collections;
using System.Collections.Generic;
using AE_BEPUPhysics_Addition.Interface;
using UnityEngine;

namespace AE_BEPUPhysics_Addition
{
    public class Test2 : MonoBehaviour
    {
        private AEPhysicsMgr mPhysicsMgr;
        [SerializeField] private BaseCollider Ground;
        [SerializeField] private float Interval;
        [SerializeField] private int TestCount;
        bool testing;
        [SerializeField] int Count;
        private int count3;

        [ContextMenu("开始测试")]
        private void StartTest()
        {
            Count = 0;
            count3 = 0;
            Physics.autoSimulation = false; // 关闭原来物理引擎迭代;
            Physics.autoSyncTransforms = false; // 关闭射线检测功能;
            mPhysicsMgr = new AEPhysicsMgr(new BEPUutilities.Vector3(0, -9.8m, 0));
            testing = true;
            mPhysicsMgr.RegisterCollider(Ground);
            StartCoroutine(Test());
        }

        IEnumerator Test()
        {
            while (testing)
            {
                yield return new WaitForSeconds(Interval);
                GameObject prefab = null;
                
                switch (count3)
                {
                    case 0:
                        prefab = Resources.Load<GameObject>("SphereBlue");
                        break;
                    case 1:
                        prefab = Resources.Load<GameObject>("SphereGreen");
                        break;
                    case 2:
                        prefab = Resources.Load<GameObject>("SphereRed");
                        break;
                }

                count3++;

                if (count3 == 3)
                {
                    count3 = 0;
                }

                
                var collider = GameObject.Instantiate(prefab);
                collider.transform.position = transform.position;
                mPhysicsMgr.RegisterCollider(collider.GetComponent<BaseCollider>());
                Count++;
                if (TestCount == Count)
                {
                    testing = false;
                    Debug.LogWarning("测试结束");
                    yield break;
                }
            }
        }

        private void Update()
        {
            if(testing)
            mPhysicsMgr.PhysicsUpdate();
        }

        private void LateUpdate()
        {
            if(testing)
            mPhysicsMgr.UpdatePosition();
        }
    }
}