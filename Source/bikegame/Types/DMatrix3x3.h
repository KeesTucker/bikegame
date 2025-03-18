#pragma once

#include "DVector.h"
#include <cmath>

struct DMatrix3x3 {
    double m[3][3];

    DMatrix3x3() {
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                m[i][j] = 0.0;
    }

    DMatrix3x3(double m00, double m01, double m02,
               double m10, double m11, double m12,
               double m20, double m21, double m22) {
        m[0][0] = m00; m[0][1] = m01; m[0][2] = m02;
        m[1][0] = m10; m[1][1] = m11; m[1][2] = m12;
        m[2][0] = m20; m[2][1] = m21; m[2][2] = m22;
    }

    DMatrix3x3 operator*(const DMatrix3x3& other) const {
        DMatrix3x3 result;
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                result.m[i][j] = 0.0;
                for (int k = 0; k < 3; k++) {
                    result.m[i][j] += m[i][k] * other.m[k][j];
                }
            }
        }
        return result;
    }

    DVector operator*(const DVector& vec) const {
        return DVector(
            m[0][0] * vec.x + m[0][1] * vec.y + m[0][2] * vec.z,
            m[1][0] * vec.x + m[1][1] * vec.y + m[1][2] * vec.z,
            m[2][0] * vec.x + m[2][1] * vec.y + m[2][2] * vec.z
        );
    }

    static DMatrix3x3 Identity() {
        return DMatrix3x3(1.0, 0.0, 0.0,
                          0.0, 1.0, 0.0,
                          0.0, 0.0, 1.0);
    }

    static DMatrix3x3 Transpose(const DMatrix3x3& mat) {
        return DMatrix3x3(
            mat.m[0][0], mat.m[1][0], mat.m[2][0],
            mat.m[0][1], mat.m[1][1], mat.m[2][1],
            mat.m[0][2], mat.m[1][2], mat.m[2][2]
        );
    }

    static double Determinant(const DMatrix3x3& mat) {
        return mat.m[0][0]*(mat.m[1][1]*mat.m[2][2] - mat.m[1][2]*mat.m[2][1])
             - mat.m[0][1]*(mat.m[1][0]*mat.m[2][2] - mat.m[1][2]*mat.m[2][0])
             + mat.m[0][2]*(mat.m[1][0]*mat.m[2][1] - mat.m[1][1]*mat.m[2][0]);
    }

    static DMatrix3x3 Inverse(const DMatrix3x3& mat) {
        double det = Determinant(mat);
        if (std::abs(det) < 1e-8) {
            return Identity();
        }
        double invDet = 1.0 / det;
        return DMatrix3x3(
            (mat.m[1][1]*mat.m[2][2] - mat.m[1][2]*mat.m[2][1]) * invDet,
            -(mat.m[0][1]*mat.m[2][2] - mat.m[0][2]*mat.m[2][1]) * invDet,
            (mat.m[0][1]*mat.m[1][2] - mat.m[0][2]*mat.m[1][1]) * invDet,

            -(mat.m[1][0]*mat.m[2][2] - mat.m[1][2]*mat.m[2][0]) * invDet,
            (mat.m[0][0]*mat.m[2][2] - mat.m[0][2]*mat.m[2][0]) * invDet,
            -(mat.m[0][0]*mat.m[1][2] - mat.m[0][2]*mat.m[1][0]) * invDet,

            (mat.m[1][0]*mat.m[2][1] - mat.m[1][1]*mat.m[2][0]) * invDet,
            -(mat.m[0][0]*mat.m[2][1] - mat.m[0][1]*mat.m[2][0]) * invDet,
            (mat.m[0][0]*mat.m[1][1] - mat.m[0][1]*mat.m[1][0]) * invDet
        );
    }
};
