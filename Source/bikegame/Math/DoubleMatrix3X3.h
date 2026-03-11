#pragma once

#include <cmath>
#include "DoubleVector.h"

#include "DoubleMatrix3X3.generated.h"

USTRUCT()
struct FDoubleMatrix3X3
{
    GENERATED_BODY()
    
    double M[3][3];

    FDoubleMatrix3X3()
    {
        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                M[i][j] = 0.0;
            }
        }
    }

    FDoubleMatrix3X3(const double M00, const double M01, const double M02,
                     const double M10, const double M11, const double M12,
                     const double M20, const double M21, const double M22)
    {
        M[0][0] = M00; M[0][1] = M01; M[0][2] = M02;
        M[1][0] = M10; M[1][1] = M11; M[1][2] = M12;
        M[2][0] = M20; M[2][1] = M21; M[2][2] = M22;
    }

    FDoubleMatrix3X3 operator*(const FDoubleMatrix3X3& Other) const
    {
        FDoubleMatrix3X3 Result;
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                Result.M[i][j] = 0.0;
                for (int k = 0; k < 3; k++)
                {
                    Result.M[i][j] += M[i][k] * Other.M[k][j];
                }
            }
        }
        return Result;
    }

    FDoubleVector operator*(const FDoubleVector& Vec) const
    {
        return FDoubleVector(
            M[0][0] * Vec.X + M[0][1] * Vec.Y + M[0][2] * Vec.Z,
            M[1][0] * Vec.X + M[1][1] * Vec.Y + M[1][2] * Vec.Z,
            M[2][0] * Vec.X + M[2][1] * Vec.Y + M[2][2] * Vec.Z
        );
    }

    static FDoubleMatrix3X3 Identity()
    {
        return FDoubleMatrix3X3(1.0, 0.0, 0.0,
                                 0.0, 1.0, 0.0,
                                 0.0, 0.0, 1.0);
    }

    static FDoubleMatrix3X3 Transpose(const FDoubleMatrix3X3& Mat)
    {
        return FDoubleMatrix3X3(
            Mat.M[0][0], Mat.M[1][0], Mat.M[2][0],
            Mat.M[0][1], Mat.M[1][1], Mat.M[2][1],
            Mat.M[0][2], Mat.M[1][2], Mat.M[2][2]
        );
    }

    static double Determinant(const FDoubleMatrix3X3& Mat)
    {
        return Mat.M[0][0] * (Mat.M[1][1] * Mat.M[2][2] - Mat.M[1][2] * Mat.M[2][1])
             - Mat.M[0][1] * (Mat.M[1][0] * Mat.M[2][2] - Mat.M[1][2] * Mat.M[2][0])
             + Mat.M[0][2] * (Mat.M[1][0] * Mat.M[2][1] - Mat.M[1][1] * Mat.M[2][0]);
    }

    static FDoubleMatrix3X3 Inverse(const FDoubleMatrix3X3& Mat)
    {
        const double Det = Determinant(Mat);
        if (std::abs(Det) < 1e-8)
        {
            return Identity();
        }
        const double InvDet = 1.0 / Det;
        return FDoubleMatrix3X3(
            (Mat.M[1][1] * Mat.M[2][2] - Mat.M[1][2] * Mat.M[2][1]) * InvDet,
            -(Mat.M[0][1] * Mat.M[2][2] - Mat.M[0][2] * Mat.M[2][1]) * InvDet,
            (Mat.M[0][1] * Mat.M[1][2] - Mat.M[0][2] * Mat.M[1][1]) * InvDet,

            -(Mat.M[1][0] * Mat.M[2][2] - Mat.M[1][2] * Mat.M[2][0]) * InvDet,
            (Mat.M[0][0] * Mat.M[2][2] - Mat.M[0][2] * Mat.M[2][0]) * InvDet,
            -(Mat.M[0][0] * Mat.M[1][2] - Mat.M[0][2] * Mat.M[1][0]) * InvDet,

            (Mat.M[1][0] * Mat.M[2][1] - Mat.M[1][1] * Mat.M[2][0]) * InvDet,
            -(Mat.M[0][0] * Mat.M[2][1] - Mat.M[0][1] * Mat.M[2][0]) * InvDet,
            (Mat.M[0][0] * Mat.M[1][1] - Mat.M[0][1] * Mat.M[1][0]) * InvDet
        );
    }

    static FDoubleMatrix3X3 WorldInertiaTensor(const FVector& BodyInertiaTensor, const FDoubleMatrix3X3& RotationMatrix)
    {
        const FDoubleMatrix3X3 LocalInertia(
            BodyInertiaTensor.X, 0.0, 0.0,
            0.0, BodyInertiaTensor.Y, 0.0,
            0.0, 0.0, BodyInertiaTensor.Z
        );
        return RotationMatrix * LocalInertia * Transpose(RotationMatrix);
    }
};
