#pragma once

#include "CoreMinimal.h"
#include <cmath>
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

    FDoubleVector GetSafeNormal() const
    {
        const double SizeValue = Size();
        return (SizeValue > 1e-8) ? (*this / SizeValue) : FDoubleVector(0.0, 0.0, 0.0);
    }

    static FDoubleVector Zero()
    {
        return FDoubleVector(0.0, 0.0, 0.0);
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
};
