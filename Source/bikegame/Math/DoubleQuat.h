#pragma once

#include "CoreMinimal.h"
#include "DoubleMatrix3X3.h"
#include "DoubleVector.h"

#include "DoubleQuat.generated.h"

USTRUCT()
struct FDoubleQuat
{
    GENERATED_BODY()

    UPROPERTY(EditAnywhere)
    double X;
    UPROPERTY(EditAnywhere)
    double Y;
    UPROPERTY(EditAnywhere)
    double Z;
    UPROPERTY(EditAnywhere)
    double W;
    
    FDoubleQuat()
        : X(0.0), Y(0.0), Z(0.0), W(1.0)
    {}
    
    FDoubleQuat(const double InX, const double InY, const double InZ, const double InW)
        : X(InX), Y(InY), Z(InZ), W(InW)
    {}
    
    explicit FDoubleQuat(const FQuat& Quat)
        : X(Quat.X), Y(Quat.Y), Z(Quat.Z), W(Quat.W)
    {}
    
    static FDoubleQuat Identity()
    {
        return FDoubleQuat(0.0, 0.0, 0.0, 1.0);
    }

    explicit operator FQuat() const
    {
        return FQuat(static_cast<float>(X), static_cast<float>(Y), static_cast<float>(Z), static_cast<float>(W));
    }
    
    void Normalize()
    {
        if (const double Magnitude = std::sqrt(X * X + Y * Y + Z * Z + W * W); Magnitude > DSmallNumber)
        {
            X /= Magnitude;
            Y /= Magnitude;
            Z /= Magnitude;
            W /= Magnitude;
        }
        else
        {
            *this = Identity();
        }
    }

    FDoubleQuat GetNormalized() const
    {
        FDoubleQuat Result = *this;
        Result.Normalize();
        return Result;
    }

    FDoubleQuat operator*(const FDoubleQuat& Other) const
    {
        return FDoubleQuat(
            W * Other.X + X * Other.W + Y * Other.Z - Z * Other.Y,
            W * Other.Y - X * Other.Z + Y * Other.W + Z * Other.X,
            W * Other.Z + X * Other.Y - Y * Other.X + Z * Other.W,
            W * Other.W - X * Other.X - Y * Other.Y - Z * Other.Z
        );
    }

    FDoubleQuat Inverse() const
    {
        return FDoubleQuat(-X, -Y, -Z, W);
    }

    FDoubleVector RotateVector(const FDoubleVector& Vector) const
    {
        const FDoubleQuat VectorQuat(Vector.X, Vector.Y, Vector.Z, 0.0);
        const FDoubleQuat RotatedQuat = (*this) * VectorQuat * Inverse();
        return FDoubleVector(RotatedQuat.X, RotatedQuat.Y, RotatedQuat.Z);
    }

    FDoubleMatrix3X3 ToRotationMatrix() const
    {
        const FDoubleQuat Norm = GetNormalized();

        const double Xx = Norm.X * Norm.X;
        const double Yy = Norm.Y * Norm.Y;
        const double Zz = Norm.Z * Norm.Z;
        const double Xy = Norm.X * Norm.Y;
        const double Xz = Norm.X * Norm.Z;
        const double Yz = Norm.Y * Norm.Z;
        const double Wx = Norm.W * Norm.X;
        const double Wy = Norm.W * Norm.Y;
        const double Wz = Norm.W * Norm.Z;

        return FDoubleMatrix3X3(
            1.0 - 2.0 * (Yy + Zz), 2.0 * (Xy - Wz),       2.0 * (Xz + Wy),
            2.0 * (Xy + Wz),       1.0 - 2.0 * (Xx + Zz), 2.0 * (Yz - Wx),
            2.0 * (Xz - Wy),       2.0 * (Yz + Wx),       1.0 - 2.0 * (Xx + Yy)
        );
    }

    void ToAxisAndAngle(FDoubleVector& OutAxis, double& OutAngle) const
    {
        const FDoubleQuat Norm = GetNormalized();
        OutAngle = 2.0 * std::acos(Norm.W);

        // If scale factor is close to zero, the axis direction is not well-defined;
        // arbitrarily choose (1,0,0) as the axis.
        if (const double S = std::sqrt(1.0 - Norm.W * Norm.W); S < 1e-8)
        {
            OutAxis = FDoubleVector(1.0, 0.0, 0.0); 
        }
        else
        {
            OutAxis = FDoubleVector(Norm.X / S, Norm.Y / S, Norm.Z / S);
        }
    }

    static FDoubleQuat FromAxisAndAngle(const FDoubleVector& Axis, const double AngleRad)
    {
        const double HalfAngle = AngleRad * 0.5;
        const double Sine = std::sin(HalfAngle);
        const double Cosine = std::cos(HalfAngle);
        return FDoubleQuat(Axis.X * Sine, Axis.Y * Sine, Axis.Z * Sine, Cosine);
    }

    static FDoubleQuat FromNormalDifference(const FDoubleVector& Normal1, const FDoubleVector& Normal2)
    {
        FDoubleVector Axis;
        double Angle;
        FDoubleVector::NormalDifferenceToAxisAngle(Normal1, Normal2, Axis, Angle);

        return FromAxisAndAngle(Axis, Angle);
    }

    static FDoubleQuat Slerp(const FDoubleQuat& A, const FDoubleQuat& B, double Alpha)
    {
        const double Dot = A.X * B.X + A.Y * B.Y + A.Z * B.Z + A.W * B.W;
        const double Theta = std::acos(Dot);

        if (const double SinTheta = std::sin(Theta); SinTheta > 0.0)
        {
            const double WeightA = std::sin((1.0 - Alpha) * Theta) / SinTheta;
            const double WeightB = std::sin(Alpha * Theta) / SinTheta;
            return FDoubleQuat(
                A.X * WeightA + B.X * WeightB,
                A.Y * WeightA + B.Y * WeightB,
                A.Z * WeightA + B.Z * WeightB,
                A.W * WeightA + B.W * WeightB
            );
        }
        return A;
    }
    
    static double GetTwistAngleRadians(const FDoubleQuat& In, const FDoubleVector& TwistAxis)
    {
        const FDoubleQuat Norm = In.GetNormalized();

        // Project the quaternion's vector part onto the twist axis.
        const double Dot = Norm.X * TwistAxis.X + Norm.Y * TwistAxis.Y + Norm.Z * TwistAxis.Z;
        const FDoubleVector Projected(TwistAxis.X * Dot, TwistAxis.Y * Dot, TwistAxis.Z * Dot);
    
        // Create a quaternion representing the twist.
        // We keep the same scalar part (W) and the projected vector part.
        FDoubleQuat TwistQuat(Projected.X, Projected.Y, Projected.Z, Norm.W);
        TwistQuat.Normalize();

        // Extract the twist angle using the relation: angle = 2 * acos(W)
        double TwistAngle = 2.0 * std::acos(TwistQuat.W);

        // Optionally, determine the sign of the angle.
        // If the dot product between the twist vector part and TwistAxis is negative, reverse the sign.
        const FDoubleVector TwistVector(TwistQuat.X, TwistQuat.Y, TwistQuat.Z);
        if (
            const double SignTest = TwistVector.X * TwistAxis.X + TwistVector.Y * TwistAxis.Y + TwistVector.Z * TwistAxis.Z;
            SignTest < 0.0
        )
        {
            TwistAngle = -TwistAngle;
        }
    
        return TwistAngle;
    }
};