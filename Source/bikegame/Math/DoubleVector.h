#pragma once

#include "CoreMinimal.h"
#include <cmath>
#include "DoubleMath.h"

#include "DoubleVector.generated.h"

USTRUCT()
struct FDoubleVector
{
    GENERATED_BODY()

    UPROPERTY(EditAnywhere)
    double X;
    UPROPERTY(EditAnywhere)
    double Y;
    UPROPERTY(EditAnywhere)
    double Z;

    FDoubleVector()
        : X(0.0), Y(0.0), Z(0.0)
    {
    }

    FDoubleVector(const double InX, const double InY, const double InZ)
        : X(InX), Y(InY), Z(InZ)
    {
    }

    explicit FDoubleVector(const FVector& Vec)
        : X(Vec.X), Y(Vec.Y), Z(Vec.Z)
    {
    }

    explicit operator FVector() const
    {
        return FVector(static_cast<float>(X),
                       static_cast<float>(Y),
                       static_cast<float>(Z));
    }

    FDoubleVector operator+(const FDoubleVector& Other) const
    {
        return FDoubleVector(X + Other.X, Y + Other.Y, Z + Other.Z);
    }

    FDoubleVector operator-(const FDoubleVector& Other) const
    {
        return FDoubleVector(X - Other.X, Y - Other.Y, Z - Other.Z);
    }

    FDoubleVector operator*(const double Scalar) const
    {
        return FDoubleVector(X * Scalar, Y * Scalar, Z * Scalar);
    }
    
    FDoubleVector& operator*=(const double Scalar)
    {
        X *= Scalar;
        Y *= Scalar;
        Z *= Scalar;
        return *this;
    }
    
    FDoubleVector operator/(const double Scalar) const
    {
        return FDoubleVector(X / Scalar, Y / Scalar, Z / Scalar);
    }

    FDoubleVector& operator+=(const FDoubleVector& Other)
    {
        X += Other.X;
        Y += Other.Y;
        Z += Other.Z;
        return *this;
    }

    FDoubleVector& operator-=(const FDoubleVector& Other)
    {
        X -= Other.X;
        Y -= Other.Y;
        Z -= Other.Z;
        return *this;
    }

    double Size() const
    {
        return std::sqrt(X * X + Y * Y + Z * Z);
    }
    
    bool IsNearlyZero() const
    {
        return std::abs(X) < DSmallNumber && std::abs(Y) < DSmallNumber && std::abs(Z) < DSmallNumber;
    }

    void Normalize()
    {
        if (const double Magnitude = Size(); Magnitude > DSmallNumber)
        {
            *this = *this / Magnitude;
        }
        else
        {
            *this = Zero();
        }
    }

    // Get the normalized quaternion
    FDoubleVector GetNormalized() const
    {
        FDoubleVector Result = *this;
        Result.Normalize();
        return Result;
    }

    static FDoubleVector Zero()
    {
        return FDoubleVector(0.0, 0.0, 0.0);
    }

    static FDoubleVector Forward()
    {
        return FDoubleVector(1, 0.0, 0.0);
    }

    static FDoubleVector Right()
    {
        return FDoubleVector(0.0, 1, 0.0);
    }

    static FDoubleVector Up()
    {
        return FDoubleVector(0.0, 0.0, 1);
    }
    
    static double Dot(const FDoubleVector& A, const FDoubleVector& B)
    {
        return A.X * B.X + A.Y * B.Y + A.Z * B.Z;
    }

    static FDoubleVector Cross(const FDoubleVector& A, const FDoubleVector& B)
    {
        return FDoubleVector(
            A.Y * B.Z - A.Z * B.Y,
            A.Z * B.X - A.X * B.Z,
            A.X * B.Y - A.Y * B.X
        );
    }

    static FDoubleVector Lerp(const FDoubleVector& A, const FDoubleVector& B, const double InAlpha)
    {
        return A + (B - A) * InAlpha;
    }

    static void NormalDifferenceToAxisAngle(const FDoubleVector& Normal1, const FDoubleVector& Normal2, FDoubleVector& OutAxis, double& OutAngle)
    {
        // Compute the dot product (cosine of the angle)
        double Dot = FDoubleVector::Dot(Normal1, Normal2);

        // Clamp the dot product to account for floating point precision issues
        Dot = FMath::Clamp(Dot, -1.0, 1.0);

        // Calculate the rotation angle between the two normals
        OutAngle = FMath::Acos(Dot);

        // Compute the rotation axis via cross product
        OutAxis = Cross(Normal1, Normal2);

        // Handle edge cases where the vectors are parallel or opposite
        if (OutAxis.IsNearlyZero())
        {
            // If the normals are nearly identical, no rotation is needed
            if (Dot > 0)
            {
                OutAxis = Up();
                OutAngle = 0;
                return;
            }
            else
            {
                // If the normals are opposite, choose an arbitrary orthogonal axis.
                OutAxis = Up();
                if (FMath::Abs(Normal1.Z) > 0.999f)
                {
                    OutAxis = Right();
                }
                OutAxis = Cross(Normal1, OutAxis).GetNormalized();
                OutAngle = PI;
            }
        }
        else
        {
            OutAxis.Normalize();
        }
    }
};

inline FDoubleVector operator*(const double Scalar, const FDoubleVector& Vec)
{
    return FDoubleVector(Vec.X * Scalar, Vec.Y * Scalar, Vec.Z * Scalar);
}