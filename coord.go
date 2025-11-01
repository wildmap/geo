package geo

import (
	"math"
)

// Coord represents a 2D coordinate in a game world or geometric space
// X and Z are used instead of X and Y, which is common in game development
// where Y often represents the vertical/height axis
type Coord struct {
	X int64 `json:"x"` // X-axis coordinate value
	Z int64 `json:"z"` // Z-axis coordinate value (depth/horizontal axis in 3D space)
}

// NewCoord creates and returns a new Coord instance with the specified x and z values
// Parameters:
//
//	x: the X-axis coordinate value
//	z: the Z-axis coordinate value
//
// Returns:
//
//	A new Coord instance
func NewCoord(x, z int64) Coord {
	return Coord{
		X: x,
		Z: z,
	}
}

// IsEqual checks whether the current coordinate is equal to the target coordinate
// Two coordinates are considered equal if both their X and Z values match
// Parameters:
//
//	target: the coordinate to compare with
//
// Returns:
//
//	true if coordinates are equal, false otherwise
func (c *Coord) IsEqual(target Coord) bool {
	return *c == target
}

// GetLocationToBorder determines the positional relationship between this coordinate and a border
// Parameters:
//
//	b: pointer to the Border to check against
//
// Returns:
//
//	LocationState indicating whether the coordinate is inside, outside, or on the border
func (c *Coord) GetLocationToBorder(b *Border) LocationState {
	return b.CoordLocation(*c)
}

// CalDstCoordToCoord calculates the Euclidean distance between two coordinates
// This function uses the Pythagorean theorem: distance = sqrt((x1-x2)² + (z1-z2)²)
// Parameters:
//
//	coord1: first coordinate
//	coord2: second coordinate
//
// Returns:
//
//	The distance between the two coordinates as a float64
func CalDstCoordToCoord(coord1, coord2 Coord) float64 {
	dx := coord1.X - coord2.X
	dz := coord1.Z - coord2.Z

	squared := dx*dx + dz*dz
	return math.Sqrt(float64(squared))
}

// CalDstCoordToCoordWithoutSqrt calculates the squared distance between two coordinates
// This is useful for comparison operations where the actual distance isn't needed,
// as it avoids the expensive square root calculation
// Parameters:
//
//	coord1: first coordinate
//	coord2: second coordinate
//
// Returns:
//
//	The squared distance between the two coordinates as a float64
func CalDstCoordToCoordWithoutSqrt(coord1, coord2 Coord) float64 {
	dx := coord1.X - coord2.X
	dz := coord1.Z - coord2.Z

	squared := dx*dx + dz*dz
	return float64(squared)
}
