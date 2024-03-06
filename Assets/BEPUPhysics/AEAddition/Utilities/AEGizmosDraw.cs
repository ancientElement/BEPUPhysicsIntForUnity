using System.Collections.Generic;
using UnityEngine;

namespace AE_BEPUPhysics_Addition
{
    public static class AEGizmosDraw
    {
        // 画圆
        public static void DrawWireCircle(Vector3 center, Vector3 forward, Vector3 right, float radius, float angle,
            float stepAngle = 15f)
        {
            // 检查两个向量是不是平行的，平行就画不了
            if (Vector3.Dot(forward, right) > 0.00001f || forward == right) return;
            radius = Mathf.Abs(radius);
            if (radius == 0) return;
            stepAngle = stepAngle <= 5 ? 5 : stepAngle;
            stepAngle = stepAngle >= 45 ? 45 : stepAngle;
            List<Vector3> vertices = new List<Vector3>();
            Vector3 startPoint = center + forward * radius;
            Vector3 endPoint = center - forward * radius;
            vertices.Add(startPoint);
            float tempAngle = 0;
            while (tempAngle < angle)
            {
                tempAngle += stepAngle;
                if (tempAngle <= angle)
                {
                    float x = radius * Mathf.Cos(Mathf.Deg2Rad * tempAngle);
                    float y = radius * Mathf.Sin(Mathf.Deg2Rad * tempAngle);
                    Vector3 vertex = center + forward * x + right * y;
                    vertices.Add(vertex);
                }
                else
                {
                    vertices.Add(endPoint);
                }
            }

            for (int i = 1; i < vertices.Count; i++)
            {
                Gizmos.DrawLine(vertices[i - 1], vertices[i]);
            }
        }

        // 画胶囊体
        public static void DrawWirePlaneCapsule(Vector3 center, Vector3 forward, Vector3 up, float radius, float height,
            float stepAngle = 15f)
        {
            if (Vector3.Dot(forward, up) > 0.00001f || forward == up) return;
            radius = Mathf.Abs(radius);
            height = Mathf.Abs(height);
            if (radius == 0 || height == 0) return;
            stepAngle = stepAngle <= 5 ? 5 : stepAngle;
            stepAngle = stepAngle >= 45 ? 45 : stepAngle;
            if (radius * 2 >= height)
            {
                DrawWireCircle(center, forward, up, radius, stepAngle);
            }
            else
            {
                float heightOfCylinder = height - 2 * radius;
                Vector3 upCircleCenter = center + heightOfCylinder * 0.5f * up;
                Vector3 downCircleCenter = center - heightOfCylinder * 0.5f * up;
                DrawWireCircle(downCircleCenter, -forward, -up, radius, 180f, stepAngle);
                DrawWireCircle(upCircleCenter, forward, up, radius, 180f, stepAngle);
                Vector3[] vertices = new Vector3[]
                {
                    upCircleCenter + forward * radius,
                    upCircleCenter + forward * radius - up * heightOfCylinder,

                    upCircleCenter - forward * radius,
                    upCircleCenter - forward * radius - up * heightOfCylinder,
                };
                for (int i = 1; i < vertices.Length; i += 2)
                {
                    Gizmos.DrawLine(vertices[i - 1], vertices[i]);
                }
            }
        }
    }
}