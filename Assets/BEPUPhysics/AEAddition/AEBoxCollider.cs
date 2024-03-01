using System;
using System.Collections;
using System.Collections.Generic;
using BEPUphysics.Entities;
using BEPUphysics.Entities.Prefabs;
using FixMath.NET;
using Unity.VisualScripting;
using UnityEditor;
using UnityEngine;
using Space = BEPUphysics.Space;

namespace AE_BEPUPhysics_Addition
{
    public class AEBoxCollider : BaseCollider
    {
        private BEPUphysics.Entities.Prefabs.Box m_box;

        public float Length;
        public float Width;
        public float Height;

#if UNITY_EDITOR
        private void OnDrawGizmos()
        {
            Vector3 boxCenter = new Vector3(CenterX, CenterY, CenterZ);
            Vector3 halfExtents = new Vector3((float)Width / 2f, (float)Height / 2f, (float)Length / 2f);

            //这里的center是相对目标中心而言，因为旋转cube与目标位置相同所以是zero
            Handles.color = Color.green;
            Handles.DrawWireCube(transform.TransformPoint(boxCenter), halfExtents * 2);
        }

        [ContextMenu("测试Fixed")]
        private void Test()
        {
            Fix64 fixValue = (Fix64)12.12;
            Debug.Log(fixValue);
            float floatValue = (float)fixValue;
            Debug.Log(floatValue);
        }

        private void Update()
        {
            OnUpdateView();
        }
#endif
        public BEPUutilities.Vector3 GetOriginPosition()
        {
            Vector3 boxCenter = new Vector3(CenterX, CenterY, CenterZ);
            Vector3 pos = transform.TransformPoint(boxCenter);
            var OriginBoxWorldPos = new BEPUutilities.Vector3((Fix64)pos.x, (Fix64)pos.y, (Fix64)pos.z);
            return OriginBoxWorldPos;
        }

        //添加进入Space
        public void AddIntoSpace(Space space)
        {
            if (IsStatic)
            {
                m_box = new Box(GetOriginPosition(), (Fix64)Width, (Fix64)Height, (Fix64)Length);
            }
            else
            {
                m_box = new Box(GetOriginPosition(), (Fix64)Width, (Fix64)Height, (Fix64)Length, 1);
            }

            m_inited = true;
            space.Add(m_box);
        }

        //从Space移除
        public void RemoveFromSpace(Space space)
        {
            space.Remove(m_box);
        }

        //渲染帧的更新
        private void OnUpdateView()
        {
            if (!m_inited) return;
            var pos = new Vector3((float)m_box.position.X, (float)m_box.position.Y, (float)m_box.position.Z);
            transform.position = pos;
            Debug.Log("位置更新回调" + pos.x + "..." + pos.y + "..." + pos.z);
        }
    }
}