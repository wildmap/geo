package geo

import "math"

// Vector represents a position vector in 2D space.
// A position vector (or position) is a vector from the coordinate origin
// to a point in space.
//
// Reference: https://en.wikipedia.org/wiki/Position_(vector)
//
// In this implementation:
//   - X component represents the horizontal axis
//   - Z component represents the vertical axis (using Z instead of Y for consistency
//     with 3D coordinate systems where Y is often the vertical axis)
type Vector struct {
	X, Z int64 // The X and Z components of the vector
}

// NewVector creates a new vector from a start point to an end point.
// The resulting vector represents the displacement from start to end.
//
// Parameters:
//
//	start - the starting coordinate
//	end - the ending coordinate
//
// Returns a Vector pointing from start to end.
func NewVector(start, end Coord) Vector {
	return Vector{
		X: end.X - start.X,
		Z: end.Z - start.Z,
	}
}

// NewVectorByCoord creates a position vector from the origin to the given coordinate.
// This is equivalent to treating the coordinate as a vector.
//
// Parameters:
//
//	p - the coordinate to convert to a vector
//
// Returns a Vector from the origin to point p.
func NewVectorByCoord(p Coord) Vector {
	return Vector(p)
}

// Add performs vector addition.
// Returns a new vector that is the sum of this vector and vec.
//
// Formula: (x1, z1) + (x2, z2) = (x1+x2, z1+z2)
func (v *Vector) Add(vec *Vector) Vector {
	return Vector{
		X: v.X + vec.X,
		Z: v.Z + vec.Z,
	}
}

// Minus performs vector subtraction.
// Returns a new vector that is the difference of this vector and vec.
//
// Formula: (x1, z1) - (x2, z2) = (x1-x2, z1-z2)
func (v *Vector) Minus(vec *Vector) Vector {
	return Vector{
		X: v.X - vec.X,
		Z: v.Z - vec.Z,
	}
}

// Dot calculates the dot product (scalar product) of two vectors.
// The dot product measures how much two vectors point in the same direction.
//
// Formula: v · vec = v.X * vec.X + v.Z * vec.Z
//
// Properties:
// - If dot product > 0: vectors point in generally the same direction
// - If dot product = 0: vectors are perpendicular
// - If dot product < 0: vectors point in generally opposite directions
//
// Returns the dot product as a float64 value.
func (v *Vector) Dot(vec *Vector) float64 {
	result := v.X * vec.X
	result += v.Z * vec.Z
	return float64(result)
}

// Cross calculates the cross product (vector product) of two 2D vectors.
// In 2D, the cross product returns a scalar value representing the Z component
// of the 3D cross product (assuming X,Z plane with Y=0).
//
// Formula: v × vec = v.X * vec.Z - v.Z * vec.X
//
// Interpretation:
// - result > 0: vec is on the left side of v (counter-clockwise)
// - result < 0: vec is on the right side of v (clockwise)
// - result = 0: vectors are collinear (parallel or anti-parallel)
//
// The magnitude of the result equals twice the area of the triangle
// formed by the two vectors.
//
// Returns the cross product as an int64 value.
func (v *Vector) Cross(vec *Vector) int64 {
	return v.X*vec.Z - v.Z*vec.X
}

// LengthSquared calculates the squared length (magnitude) of the vector.
// This is more efficient than Length() when you only need to compare lengths
// or when the actual length value isn't needed.
//
// Formula: |v|² = X² + Z²
//
// Returns the squared length as a float64 value.
func (v *Vector) LengthSquared() float64 {
	return float64(v.X*v.X + v.Z*v.Z)
}

// Length calculates the Euclidean length (magnitude) of the vector.
//
// Formula: |v| = √(X² + Z²)
//
// Returns the length as a float64 value.
func (v *Vector) Length() float64 {
	return math.Sqrt(v.LengthSquared())
}

// Trunc truncates (scales) the vector by a given ratio.
// This creates a new vector in the same direction but with different length.
//
// Formula: v_new = (ratio * v.X, ratio * v.Z)
//
// The result is rounded to the nearest integer.
//
// Parameters:
//
//	ratio - the scaling factor (e.g., 0.5 halves the length, 2.0 doubles it)
//
// Returns a new scaled Vector.
func (v *Vector) Trunc(ratio float64) Vector {
	return Vector{
		X: int64(math.Round(ratio * float64(v.X))),
		Z: int64(math.Round(ratio * float64(v.Z))),
	}
}

// TruncEdge truncates an edge to create a unit vector scaled to length 1000.
// This is useful for creating consistent step sizes along an edge.
//
// Algorithm:
// 1. Create a vector from start to end
// 2. Calculate the scaling ratio: 1000 / vector_length
// 3. Scale the vector by this ratio
// 4. Convert back to coordinate relative to start point
//
// Parameters:
//
//	start - the starting coordinate of the edge
//	end - the ending coordinate of the edge
//
// Returns a Coord representing a point 1000 units away from start along the edge.
func TruncEdge(start, end Coord) Coord {
	// Create vector from start to end
	vec0 := NewVector(start, end)

	// Scale to unit vector with length 1000
	vec0 = vec0.Trunc(1000 / vec0.Length())

	return vec0.ToCoord(start)
}

// ToCoord converts the vector to a coordinate by adding it to a start point.
// This translates a point by the vector displacement.
//
// Formula: result = start + v
//
// Parameters:
//
//	start - the starting coordinate
//
// Returns the coordinate obtained by adding this vector to start.
func (v *Vector) ToCoord(start Coord) Coord {
	return Coord{
		X: start.X + v.X,
		Z: start.Z + v.Z,
	}
}

// Rotate rotates the vector by a given angle.
// In a left-handed coordinate system, positive angles rotate counter-clockwise.
//
// Rotation formula (2D rotation matrix):
// x' = x*cos(θ) - z*sin(θ)
// z' = x*sin(θ) + z*cos(θ)
//
// Where (x, z) is the original vector and (x', z') is the rotated vector.
//
// Reference: https://blog.csdn.net/u013445530/article/details/44904017
//
// Parameters:
//
//	angle - the rotation angle in radians (positive = counter-clockwise)
//
// Returns a new rotated Vector.
func (v *Vector) Rotate(angle float64) Vector {
	x0 := float64(v.X)
	z0 := float64(v.Z)

	cos := math.Cos(angle)
	sin := math.Sin(angle)

	// Apply rotation matrix
	x := x0*cos - z0*sin
	z := x0*sin + z0*cos

	return Vector{
		X: int64(math.Round(x)),
		Z: int64(math.Round(z)),
	}
}

// CalCoordDst calculates the perpendicular distance from a point to the line
// defined by this vector.
//
// Algorithm:
// 1. Create a vector from start point to target point
// 2. Calculate the angle between this vector and the vector to the target
// 3. Use trigonometry: distance = |target_vector| * sin(angle)
//
// This gives the perpendicular distance from target to the infinite line
// defined by the vector.
//
// Parameters:
//
//	start - a point on the line
//	target - the point to measure distance from
//
// Returns the perpendicular distance as a float64 value.
func (v *Vector) CalCoordDst(start, target Coord) float64 {
	vec := NewVector(start, target)

	angle := v.GetAngle(&vec)
	return vec.Length() * math.Sin(angle)
}

// GetAngle calculates the angle between this vector and another vector.
// The angle is always in the range [0, π] radians.
//
// Formula: θ = arccos((v · vec) / (|v| * |vec|))
//
// Special cases:
// - If arccos returns NaN and dot product > 0: angle = 0 (parallel, same direction)
// - If arccos returns NaN and dot product <= 0: angle = π (parallel, opposite direction)
//
// Parameters:
//
//	vec - the other vector to measure angle with
//
// Returns the angle in radians (0 to π).
func (v *Vector) GetAngle(vec *Vector) float64 {
	// Calculate dot product
	a := v.Dot(vec)
	// Calculate product of magnitudes
	b := v.Length() * vec.Length()
	// Calculate cosine of angle
	t := a / b
	// Calculate angle
	angle := math.Acos(t)

	// Handle special cases where arccos might return NaN
	if math.IsNaN(angle) {
		if t > 0 {
			return 0 // Vectors are parallel (same direction)
		}
		return math.Pi // Vectors are anti-parallel (opposite direction)
	}

	return angle
}

// cross is a helper function that calculates the cross product for three points.
// It computes the cross product: (p1-p3) × (p2-p3)
//
// This is used to determine the orientation of three points:
// - result > 0: p1, p2, p3 form a counter-clockwise turn
// - result < 0: p1, p2, p3 form a clockwise turn
// - result = 0: p1, p2, p3 are collinear
//
// Formula: (p1.X - p3.X) * (p2.Z - p3.Z) - (p2.X - p3.X) * (p1.Z - p3.Z)
//
// Parameters:
//
//	p1, p2, p3 - the three points to evaluate
//
// Returns the cross product value as int64.
func cross(p1, p2, p3 Coord) int64 {
	s := (p1.X-p3.X)*(p2.Z-p3.Z) - (p2.X-p3.X)*(p1.Z-p3.Z)
	return s
}

// CalCoordByRatio calculates a point along the line segment from startCoord to endCoord.
// The ratio parameter determines the position along the segment.
//
// Algorithm:
// 1. Create a vector from start to end
// 2. Scale the vector by the given ratio
// 3. Add the scaled vector to the start coordinate
//
// Parameters:
//
//	startCoord - the starting coordinate
//	endCoord - the ending coordinate
//	ratio - the position along the segment (0.0 = start, 1.0 = end, 0.5 = midpoint)
//
// Returns the coordinate at the specified ratio along the segment.
//
// Example:
//
//	ratio = 0.0 returns startCoord
//	ratio = 1.0 returns endCoord
//	ratio = 0.5 returns the midpoint
//	ratio = 2.0 returns a point beyond endCoord (extrapolation)
func CalCoordByRatio(startCoord, endCoord Coord, ratio float64) Coord {
	vec := NewVector(startCoord, endCoord)
	trunc := vec.Trunc(ratio)
	return trunc.ToCoord(startCoord)
}
