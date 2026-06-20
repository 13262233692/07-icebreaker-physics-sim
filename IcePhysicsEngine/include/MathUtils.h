#pragma once

#include "IcePhysicsAPI.h"
#include <cmath>
#include <algorithm>

namespace IcePhysics
{
    namespace Math
    {
        const float PI = 3.14159265359f;
        const float HALF_PI = PI * 0.5f;
        const float TWO_PI = PI * 2.0f;
        const float EPSILON = 0.00001f;
        const float DEG_TO_RAD = PI / 180.0f;
        const float RAD_TO_DEG = 180.0f / PI;

        inline float Abs(float x) { return fabsf(x); }
        inline float Sqrt(float x) { return sqrtf(x); }
        inline float Pow(float x, float y) { return powf(x, y); }
        inline float Exp(float x) { return expf(x); }
        inline float Sin(float x) { return sinf(x); }
        inline float Cos(float x) { return cosf(x); }
        inline float Tan(float x) { return tanf(x); }
        inline float Asin(float x) { return asinf(x); }
        inline float Acos(float x) { return acosf(x); }
        inline float Atan2(float y, float x) { return atan2f(y, x); }
        inline float Log10(float x) { return log10f(x); }
        inline float Min(float a, float b) { return (a < b) ? a : b; }
        inline float Max(float a, float b) { return (a > b) ? a : b; }
        inline float Clamp(float x, float min, float max) { return Max(min, Min(max, x)); }
        inline float Lerp(float a, float b, float t) { return a + (b - a) * t; }
        inline float Sign(float x) { return (x > 0.0f) ? 1.0f : ((x < 0.0f) ? -1.0f : 0.0f); }

        inline Vector3 Vec3(float x, float y, float z) { return Vector3(x, y, z); }
        inline Vector3 Vec3Zero() { return Vector3(0, 0, 0); }
        inline Vector3 Vec3One() { return Vector3(1, 1, 1); }
        inline Vector3 Vec3Right() { return Vector3(1, 0, 0); }
        inline Vector3 Vec3Up() { return Vector3(0, 1, 0); }
        inline Vector3 Vec3Forward() { return Vector3(0, 0, 1); }

        inline Vector3 operator+(const Vector3& a, const Vector3& b) { return Vector3(a.x + b.x, a.y + b.y, a.z + b.z); }
        inline Vector3 operator-(const Vector3& a, const Vector3& b) { return Vector3(a.x - b.x, a.y - b.y, a.z - b.z); }
        inline Vector3 operator*(const Vector3& a, float s) { return Vector3(a.x * s, a.y * s, a.z * s); }
        inline Vector3 operator*(float s, const Vector3& a) { return Vector3(a.x * s, a.y * s, a.z * s); }
        inline Vector3 operator-(const Vector3& a) { return Vector3(-a.x, -a.y, -a.z); }
        inline Vector3 operator/(const Vector3& a, float s) { return Vector3(a.x / s, a.y / s, a.z / s); }
        inline Vector3& operator+=(Vector3& a, const Vector3& b) { a.x += b.x; a.y += b.y; a.z += b.z; return a; }
        inline Vector3& operator-=(Vector3& a, const Vector3& b) { a.x -= b.x; a.y -= b.y; a.z -= b.z; return a; }
        inline Vector3& operator*=(Vector3& a, float s) { a.x *= s; a.y *= s; a.z *= s; return a; }
        inline Vector3& operator/=(Vector3& a, float s) { a.x /= s; a.y /= s; a.z /= s; return a; }

        inline float Dot(const Vector3& a, const Vector3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
        inline Vector3 Cross(const Vector3& a, const Vector3& b)
        {
            return Vector3(
                a.y * b.z - a.z * b.y,
                a.z * b.x - a.x * b.z,
                a.x * b.y - a.y * b.x
            );
        }
        inline float LengthSquared(const Vector3& v) { return Dot(v, v); }
        inline float Length(const Vector3& v) { return Sqrt(LengthSquared(v)); }
        inline Vector3 Normalize(const Vector3& v)
        {
            float len = Length(v);
            if (len < EPSILON) return Vec3Zero();
            return v / len;
        }
        inline float DistanceSquared(const Vector3& a, const Vector3& b) { return LengthSquared(b - a); }
        inline float Distance(const Vector3& a, const Vector3& b) { return Length(b - a); }
        inline Vector3 Negate(const Vector3& v) { return Vector3(-v.x, -v.y, -v.z); }

        inline Vector2 Vec2(float x, float y) { return Vector2(x, y); }
        inline Vector2 Vec2Zero() { return Vector2(0, 0); }
        inline Vector2 operator+(const Vector2& a, const Vector2& b) { return Vector2(a.x + b.x, a.y + b.y); }
        inline Vector2 operator-(const Vector2& a, const Vector2& b) { return Vector2(a.x - b.x, a.y - b.y); }
        inline Vector2 operator*(const Vector2& a, float s) { return Vector2(a.x * s, a.y * s); }
        inline Vector2 operator-(const Vector2& a) { return Vector2(-a.x, -a.y); }
        inline Vector2 operator*(float s, const Vector2& a) { return Vector2(a.x * s, a.y * s); }
        inline Vector2 operator/(const Vector2& a, float s) { return Vector2(a.x / s, a.y / s); }
        inline Vector2& operator+=(Vector2& a, const Vector2& b) { a.x += b.x; a.y += b.y; return a; }
        inline Vector2& operator-=(Vector2& a, const Vector2& b) { a.x -= b.x; a.y -= b.y; return a; }
        inline Vector2& operator*=(Vector2& a, float s) { a.x *= s; a.y *= s; return a; }
        inline Vector2& operator/=(Vector2& a, float s) { a.x /= s; a.y /= s; return a; }
        inline float Dot(const Vector2& a, const Vector2& b) { return a.x * b.x + a.y * b.y; }
        inline float LengthSquared(const Vector2& v) { return Dot(v, v); }
        inline float Length(const Vector2& v) { return Sqrt(LengthSquared(v)); }
        inline Vector2 Normalize(const Vector2& v)
        {
            float len = Length(v);
            if (len < EPSILON) return Vector2(0, 0);
            return v / len;
        }
        inline float Distance(const Vector2& a, const Vector2& b) { return Length(b - a); }
        inline float Cross(const Vector2& a, const Vector2& b) { return a.x * b.y - a.y * b.x; }

        inline Quaternion Quat(float x, float y, float z, float w) { return Quaternion(x, y, z, w); }
        inline Quaternion QuatIdentity() { return Quaternion(0, 0, 0, 1); }

        inline Quaternion QuatFromAngleAxis(float angle, const Vector3& axis)
        {
            float halfAngle = angle * 0.5f;
            float s = Sin(halfAngle);
            Vector3 n = Normalize(axis);
            return Quaternion(n.x * s, n.y * s, n.z * s, Cos(halfAngle));
        }

        inline Quaternion QuatFromEuler(float xDeg, float yDeg, float zDeg)
        {
            float xRad = xDeg * DEG_TO_RAD * 0.5f;
            float yRad = yDeg * DEG_TO_RAD * 0.5f;
            float zRad = zDeg * DEG_TO_RAD * 0.5f;

            float sx = Sin(xRad), cx = Cos(xRad);
            float sy = Sin(yRad), cy = Cos(yRad);
            float sz = Sin(zRad), cz = Cos(zRad);

            return Quaternion(
                sx * cy * cz - cx * sy * sz,
                cx * sy * cz + sx * cy * sz,
                cx * cy * sz - sx * sy * cz,
                cx * cy * cz + sx * sy * sz
            );
        }

        inline Quaternion operator*(const Quaternion& a, const Quaternion& b)
        {
            return Quaternion(
                a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
                a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
                a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
                a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z
            );
        }

        inline Vector3 operator*(const Quaternion& q, const Vector3& v)
        {
            Vector3 qv(q.x, q.y, q.z);
            Vector3 t = 2.0f * Cross(qv, v);
            return v + q.w * t + Cross(qv, t);
        }

        inline float QuatDot(const Quaternion& a, const Quaternion& b)
        {
            return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
        }

        inline Quaternion QuatNormalize(const Quaternion& q)
        {
            float len = Sqrt(QuatDot(q, q));
            if (len < EPSILON) return QuatIdentity();
            return Quaternion(q.x / len, q.y / len, q.z / len, q.w / len);
        }

        inline Quaternion QuatInverse(const Quaternion& q)
        {
            float lenSq = QuatDot(q, q);
            if (lenSq < EPSILON) return QuatIdentity();
            return Quaternion(-q.x / lenSq, -q.y / lenSq, -q.z / lenSq, q.w / lenSq);
        }

        inline Vector3 QuatToEuler(const Quaternion& q)
        {
            float test = q.x * q.y + q.z * q.w;

            if (test > 0.499f)
            {
                return Vector3(
                    2.0f * Atan2(q.x, q.w),
                    HALF_PI,
                    0.0f
                );
            }
            if (test < -0.499f)
            {
                return Vector3(
                    -2.0f * Atan2(q.x, q.w),
                    -HALF_PI,
                    0.0f
                );
            }

            float sqx = q.x * q.x;
            float sqy = q.y * q.y;
            float sqz = q.z * q.z;

            return Vector3(
                Atan2(2.0f * q.y * q.w - 2.0f * q.x * q.z, 1.0f - 2.0f * sqy - 2.0f * sqz),
                Asin(2.0f * test),
                Atan2(2.0f * q.x * q.w - 2.0f * q.y * q.z, 1.0f - 2.0f * sqx - 2.0f * sqz)
            ) * RAD_TO_DEG;
        }

        inline Matrix3x3 Mat3Identity()
        {
            Matrix3x3 m = {};
            m.m[0] = 1; m.m[4] = 1; m.m[8] = 1;
            return m;
        }

        inline Matrix3x3 Mat3FromQuat(const Quaternion& q)
        {
            Matrix3x3 m = {};
            float xx = q.x * q.x;
            float yy = q.y * q.y;
            float zz = q.z * q.z;
            float xy = q.x * q.y;
            float xz = q.x * q.z;
            float yz = q.y * q.z;
            float wx = q.w * q.x;
            float wy = q.w * q.y;
            float wz = q.w * q.z;

            m.m[0] = 1.0f - 2.0f * (yy + zz);
            m.m[1] = 2.0f * (xy + wz);
            m.m[2] = 2.0f * (xz - wy);
            m.m[3] = 2.0f * (xy - wz);
            m.m[4] = 1.0f - 2.0f * (xx + zz);
            m.m[5] = 2.0f * (yz + wx);
            m.m[6] = 2.0f * (xz + wy);
            m.m[7] = 2.0f * (yz - wx);
            m.m[8] = 1.0f - 2.0f * (xx + yy);

            return m;
        }

        inline Vector3 Mat3MulVec3(const Matrix3x3& m, const Vector3& v)
        {
            return Vector3(
                m.m[0] * v.x + m.m[1] * v.y + m.m[2] * v.z,
                m.m[3] * v.x + m.m[4] * v.y + m.m[5] * v.z,
                m.m[6] * v.x + m.m[7] * v.y + m.m[8] * v.z
            );
        }

        inline Matrix3x3 Mat3Inverse(const Matrix3x3& m)
        {
            float det =
                m.m[0] * (m.m[4] * m.m[8] - m.m[5] * m.m[7]) -
                m.m[1] * (m.m[3] * m.m[8] - m.m[5] * m.m[6]) +
                m.m[2] * (m.m[3] * m.m[7] - m.m[4] * m.m[6]);

            if (Abs(det) < EPSILON) return Mat3Identity();

            float invDet = 1.0f / det;

            Matrix3x3 result = {};
            result.m[0] = (m.m[4] * m.m[8] - m.m[5] * m.m[7]) * invDet;
            result.m[1] = (m.m[2] * m.m[7] - m.m[1] * m.m[8]) * invDet;
            result.m[2] = (m.m[1] * m.m[5] - m.m[2] * m.m[4]) * invDet;
            result.m[3] = (m.m[5] * m.m[6] - m.m[3] * m.m[8]) * invDet;
            result.m[4] = (m.m[0] * m.m[8] - m.m[2] * m.m[6]) * invDet;
            result.m[5] = (m.m[3] * m.m[2] - m.m[0] * m.m[5]) * invDet;
            result.m[6] = (m.m[3] * m.m[7] - m.m[4] * m.m[6]) * invDet;
            result.m[7] = (m.m[1] * m.m[6] - m.m[0] * m.m[7]) * invDet;
            result.m[8] = (m.m[0] * m.m[4] - m.m[1] * m.m[3]) * invDet;

            return result;
        }

        inline Matrix3x3 Mat3Transpose(const Matrix3x3& m)
        {
            Matrix3x3 result = {};
            result.m[0] = m.m[0];
            result.m[1] = m.m[3];
            result.m[2] = m.m[6];
            result.m[3] = m.m[1];
            result.m[4] = m.m[4];
            result.m[5] = m.m[7];
            result.m[6] = m.m[2];
            result.m[7] = m.m[5];
            result.m[8] = m.m[8];
            return result;
        }

        inline Matrix3x3 operator+(const Matrix3x3& a, const Matrix3x3& b)
        {
            Matrix3x3 result = {};
            for (int i = 0; i < 9; i++)
                result.m[i] = a.m[i] + b.m[i];
            return result;
        }

        inline Matrix3x3 operator*(const Matrix3x3& a, float s)
        {
            Matrix3x3 result = {};
            for (int i = 0; i < 9; i++)
                result.m[i] = a.m[i] * s;
            return result;
        }

        inline AABB AABBCreate(const Vector3& min, const Vector3& max)
        {
            AABB result;
            result.min = min;
            result.max = max;
            return result;
        }

        inline AABB AABBFromCenterExtents(const Vector3& center, const Vector3& extents)
        {
            AABB result;
            result.min = center - extents;
            result.max = center + extents;
            return result;
        }

        inline Vector3 AABBGetCenter(const AABB& aabb)
        {
            return (aabb.min + aabb.max) * 0.5f;
        }

        inline Vector3 AABBGetExtents(const AABB& aabb)
        {
            return (aabb.max - aabb.min) * 0.5f;
        }

        inline bool AABBContains(const AABB& aabb, const Vector3& point)
        {
            return point.x >= aabb.min.x && point.x <= aabb.max.x &&
                   point.y >= aabb.min.y && point.y <= aabb.max.y &&
                   point.z >= aabb.min.z && point.z <= aabb.max.z;
        }

        inline bool AABBIntersects(const AABB& a, const AABB& b)
        {
            return a.min.x <= b.max.x && a.max.x >= b.min.x &&
                   a.min.y <= b.max.y && a.max.y >= b.min.y &&
                   a.min.z <= b.max.z && a.max.z >= b.min.z;
        }

        inline AABB AABBExpand(const AABB& aabb, const Vector3& point)
        {
            AABB result;
            result.min = Vector3(Min(aabb.min.x, point.x), Min(aabb.min.y, point.y), Min(aabb.min.z, point.z));
            result.max = Vector3(Max(aabb.max.x, point.x), Max(aabb.max.y, point.y), Max(aabb.max.z, point.z));
            return result;
        }

        inline AABB AABBTransform(const AABB& aabb, const Vector3& position, const Quaternion& rotation)
        {
            Vector3 center = AABBGetCenter(aabb);
            Vector3 extents = AABBGetExtents(aabb);

            Vector3 rotatedCenter = position + rotation * center;

            Matrix3x3 rotMat = Mat3FromQuat(rotation);
            Vector3 rotatedExtents(
                Abs(rotMat.m[0]) * extents.x + Abs(rotMat.m[1]) * extents.y + Abs(rotMat.m[2]) * extents.z,
                Abs(rotMat.m[3]) * extents.x + Abs(rotMat.m[4]) * extents.y + Abs(rotMat.m[5]) * extents.z,
                Abs(rotMat.m[6]) * extents.x + Abs(rotMat.m[7]) * extents.y + Abs(rotMat.m[8]) * extents.z
            );

            return AABBFromCenterExtents(rotatedCenter, rotatedExtents);
        }

        inline bool IsPointInTriangle(const Vector3& p, const Vector3& a, const Vector3& b, const Vector3& c)
        {
            Vector3 v0 = c - a;
            Vector3 v1 = b - a;
            Vector3 v2 = p - a;

            float dot00 = Dot(v0, v0);
            float dot01 = Dot(v0, v1);
            float dot02 = Dot(v0, v2);
            float dot11 = Dot(v1, v1);
            float dot12 = Dot(v1, v2);

            float invDenom = 1.0f / (dot00 * dot11 - dot01 * dot01);
            float u = (dot11 * dot02 - dot01 * dot12) * invDenom;
            float v = (dot00 * dot12 - dot01 * dot02) * invDenom;

            return (u >= 0) && (v >= 0) && (u + v < 1);
        }

        inline float TriangleArea(const Vector3& a, const Vector3& b, const Vector3& c)
        {
            return 0.5f * Length(Cross(b - a, c - a));
        }

        inline Vector3 TriangleNormal(const Vector3& a, const Vector3& b, const Vector3& c)
        {
            return Normalize(Cross(b - a, c - a));
        }

        inline float SignedVolume(const Vector3& a, const Vector3& b, const Vector3& c, const Vector3& d)
        {
            return Dot(a - d, Cross(b - d, c - d)) / 6.0f;
        }
    }

    using namespace Math;
}
