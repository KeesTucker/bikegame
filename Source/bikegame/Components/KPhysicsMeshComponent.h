#pragma once

#include "CoreMinimal.h"
#include "Components/StaticMeshComponent.h"
#include "KPhysicsMeshComponent.generated.h"

constexpr double DSmall_Number = 1e-8;

//---------------------------------------------------------------------
// DMath: Helper functions for double-precision math operations.
//---------------------------------------------------------------------
struct DMath {
	static double Lerp(double A, double B, double Alpha) {
		return A + (B - A) * Alpha;
	}
	static double Clamp(double Value, double Min, double Max) {
		return (Value < Min) ? Min : (Value > Max ? Max : Value);
	}
};

//---------------------------------------------------------------------
// DVector: A double-precision 3D vector type
//---------------------------------------------------------------------
struct DVector {
    double x, y, z;

    DVector() : x(0.0), y(0.0), z(0.0) {}
    DVector(double _x, double _y, double _z) : x(_x), y(_y), z(_z) {}
    DVector(const FVector& vec) : x(vec.X), y(vec.Y), z(vec.Z) {}

    operator FVector() const {
        return FVector(static_cast<float>(x),
                       static_cast<float>(y),
                       static_cast<float>(z));
    }

    DVector operator+(const DVector& other) const {
        return DVector(x + other.x, y + other.y, z + other.z);
    }
    DVector operator-(const DVector& other) const {
        return DVector(x - other.x, y - other.y, z - other.z);
    }
    DVector operator*(double scalar) const {
        return DVector(x * scalar, y * scalar, z * scalar);
    }
    DVector operator/(double scalar) const {
        return DVector(x / scalar, y / scalar, z / scalar);
    }
    DVector& operator+=(const DVector& other) {
        x += other.x; y += other.y; z += other.z;
        return *this;
    }
    DVector& operator-=(const DVector& other) {
        x -= other.x; y -= other.y; z -= other.z;
        return *this;
    }
    double Size() const {
        return std::sqrt(x * x + y * y + z * z);
    }
    DVector GetSafeNormal() const {
        double s = Size();
        return (s > 1e-8) ? (*this / s) : DVector(0.0, 0.0, 0.0);
    }

	static double Dot(const DVector& a, const DVector& b) {
    	return a.x * b.x + a.y * b.y + a.z * b.z;
    }
	static DVector Cross(const DVector& a, const DVector& b) {
    	return DVector(
			a.y * b.z - a.z * b.y,
			a.z * b.x - a.x * b.z,
			a.x * b.y - a.y * b.x
		);
    }
	static DVector Lerp(const DVector& A, const DVector& B, double Alpha) {
    	return A + (B - A) * Alpha;
    }
};

//---------------------------------------------------------------------
// DMatrix3x3: A simple double-precision 3x3 matrix type with helper functions.
//---------------------------------------------------------------------
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

/**
 * UKPhysics is a custom physics simulation component.
 * It now inherits from UStaticMeshComponent and applies physics directly to itself.
 */
UCLASS(ClassGroup=(Physics), meta=(BlueprintSpawnableComponent))
class BIKEGAME_API UKPhysicsMeshComponent : public UStaticMeshComponent
{
	GENERATED_BODY()

public:
	UKPhysicsMeshComponent();

protected:
	// Called when the game starts.
	virtual void BeginPlay() override;
	
	// Number of integration substeps per frame for improved simulation accuracy.
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	double NumHzPhysics = 1000;

	// Mass in kilograms.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Physics")
	double Mass = 10;
	
	// Friction and restitution properties.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Friction")
	double StaticFrictionCoefficient = 0.6;
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Friction")
	double DynamicFrictionCoefficient = 0.5;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Physics")
	double RestitutionCoefficient = 0.5;

	// Damping factors.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Physics")
	double LinearDampingFactor = 0.01;
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Physics")
	double AngularDampingFactor = 0;
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Friction")
	double SlipBias = 0.001;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Friction")
	double MinInducedSlip = 0.01;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Friction")
	double InducedSlipBlend = 0.1;

public:	
	// Called every frame to update the physics simulation.
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;
	
private:
	// Handles collision response when a collision is detected.
	void ResolveCollision(FHitResult& Hit, UPrimitiveComponent* PrimitiveComponent);
	
	// Current velocities.
	DVector LinearVelocity;
	DVector AngularVelocity;
};
