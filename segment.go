package geo

import (
	"github.com/wildmap/utility"
)

// Segment represents a line segment defined by two endpoint coordinates A and B.
// A segment is a finite portion of a line bounded by two distinct end points.
type Segment struct {
	A, B Coord // The two endpoints of the segment
}

// NewSegment creates and returns a new Segment with the given endpoints.
// Parameters:
//
//	a - the first endpoint
//	b - the second endpoint
func NewSegment(a, b Coord) Segment {
	return Segment{
		A: a,
		B: b,
	}
}

// ToVector converts the segment to a vector representation.
// The vector points from endpoint A to endpoint B.
// Returns a Vector from A to B.
func (s *Segment) ToVector() Vector {
	return NewVector(s.A, s.B)
}

// CalCoordDst calculates the shortest distance from a point to this line segment.
//
// Algorithm:
// 1. Calculate distances from the point to both endpoints (a, b)
// 2. Find the minimum of these two distances
// 3. Check if the perpendicular from the point to the line intersects the segment
// 4. If the perpendicular intersects, calculate the perpendicular distance
// 5. Return the minimum of the endpoint distances and perpendicular distance
//
// Returns the shortest distance as a float64 value.
func (s *Segment) CalCoordDst(coord Coord) float64 {
	// Calculate distances from point to both endpoints
	a := CalDstCoordToCoord(coord, s.A)
	b := CalDstCoordToCoord(coord, s.B)
	dst := min(a, b)

	// Check if the perpendicular from the point intersects the segment
	ab := NewVector(s.A, s.B)
	ap := NewVector(s.A, coord)
	// If the projection exceeds the segment length, use endpoint distance
	if lab := ab.Length(); utility.Greater(ap.Dot(&ab)/lab, lab) {
		return dst
	}

	// Calculate perpendicular distance from point to the line
	vec := s.ToVector()
	c := vec.CalCoordDst(s.A, coord)

	return min(dst, c)
}

// Pan translates (shifts) the segment parallel to itself by a given distance.
// In a left-handed coordinate system, the segment is moved in the direction
// where the cross product is positive (or negative if positive=false).
//
// Algorithm:
// 1. Calculate the normal vector perpendicular to the segment
// 2. Normalize the normal vector and scale it by the desired distance
// 3. Translate both endpoints by this scaled normal vector
//
// Parameters:
//
//	dst - the distance to translate the segment
//	positive - if true, move in the direction of positive cross product;
//	           if false, move in the opposite direction
//
// Returns a new Segment that is parallel to the original, offset by dst.
func (s *Segment) Pan(dst int64, positive bool) Segment {
	v := s.ToVector()

	// Calculate the normal vector (perpendicular in the direction of positive cross product)
	normalV := NewVectorByCoord(Coord{X: -v.Z, Z: v.X})
	if !positive {
		// Reverse the normal vector direction
		normalV = NewVectorByCoord(Coord{X: v.Z, Z: -v.X})
	}

	// Scale the normal vector to the desired distance
	ratio := float64(dst) / v.Length()
	newV := normalV.Trunc(ratio)

	// Translate both endpoints by the scaled normal vector
	return NewSegment(newV.ToCoord(s.A), newV.ToCoord(s.B))
}

// CrossCircle determines if the segment intersects with a circle.
// If an intersection exists, it returns the first intersection point.
//
// Returns:
//
//	Coord - the first intersection point (if exists)
//	bool - true if intersection exists, false otherwise
func (s *Segment) CrossCircle(circle Circle) (Coord, bool) {
	return GetLineCrossCircle(s.A, s.B, circle.Center, circle.Radius)
}

// IsRectCross performs a quick rejection test to check if two line segments
// might intersect by checking if their bounding rectangles overlap.
//
// This is the "rejection test" - if the bounding rectangles don't overlap,
// the segments definitely don't intersect. This is a fast preliminary check.
//
// Parameters:
//
//	p0, p1 - endpoints of the first segment
//	q0, q1 - endpoints of the second segment
//
// Returns true if the bounding rectangles overlap, false otherwise.
func IsRectCross(p0, p1, q0, q1 Coord) bool {
	ret := min(p0.X, p1.X) <= max(q0.X, q1.X) &&
		min(q0.X, q1.X) <= max(p0.X, p1.X) &&
		min(p0.Z, p1.Z) <= max(q0.Z, q1.Z) &&
		min(q0.Z, q1.Z) <= max(p0.Z, p1.Z)
	return ret
}

// IsLineSegmentCross performs the "straddle test" to determine if two line segments
// actually intersect.
//
// Algorithm (Cross Product Method):
// Two segments P0P1 and Q0Q1 intersect if and only if:
// 1. Q0 and Q1 are on opposite sides of line P0P1, AND
// 2. P0 and P1 are on opposite sides of line Q0Q1
//
// The cross product determines which side of a line a point is on:
// - If cross product = 0, the point is collinear with the line
// - If cross products have opposite signs, points are on opposite sides
//
// Special case: If any cross product is 0 (collinear), consider it as intersection
//
// Parameters:
//
//	p0, p1 - endpoints of the first segment
//	q0, q1 - endpoints of the second segment
//
// Returns true if the segments intersect or touch, false otherwise.
func IsLineSegmentCross(p0, p1, q0, q1 Coord) bool {
	// Calculate cross products to determine positions of Q0 and Q1 relative to P0P1
	// q0q1 × q0p0 (cross product of vector Q0Q1 with vector Q0P0)
	b1 := cross(q1, p0, q0)
	// q0q1 × q0p1 (cross product of vector Q0Q1 with vector Q0P1)
	b2 := cross(q1, p1, q0)

	// If cross product is 0, one point is collinear with the other segment
	if b1 == 0 || b2 == 0 {
		return true
	}

	// Calculate cross products to determine positions of P0 and P1 relative to Q0Q1
	// p0p1 × p0q0 (cross product of vector P0P1 with vector P0Q0)
	a1 := cross(p1, q0, p0)
	// p0p1 × p0q1 (cross product of vector P0P1 with vector P0Q1)
	a2 := cross(p1, q1, p0)

	// If cross product is 0, one point is collinear with the other segment
	if a1 == 0 || a2 == 0 {
		return true
	}

	// Segments intersect if points are on opposite sides of both lines
	// XOR operation: true if signs are different (points on opposite sides)
	return ((b1 < 0) != (b2 < 0)) && ((a1 < 0) != (a2 < 0))
}

// GetCrossCoord calculates the exact intersection point of two line segments.
// Uses the parametric line equation method.
//
// Algorithm based on:
// https://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect/565282#
//
// Given segments P0P1 and Q0Q1, the intersection point is calculated as:
// P = P0 + t * (P1 - P0), where t is the parameter
//
// Steps:
// 1. Check if segments are parallel (collinear)
// 2. Perform quick rejection test (bounding box check)
// 3. Perform straddle test
// 4. Calculate the exact intersection point using parametric equations
//
// Parameters:
//
//	p0, p1 - endpoints of the first segment
//	q0, q1 - endpoints of the second segment
//
// Returns:
//
//	Coord - the intersection point (if exists)
//	bool - true if intersection exists, false if parallel or no intersection
func GetCrossCoord(p0, p1, q0, q1 Coord) (Coord, bool) {
	v1 := NewVector(p0, p1)
	v2 := NewVector(q0, q1)

	// Check if segments are parallel (cross product = 0 means collinear/parallel)
	if v1.Cross(&v2) == 0 {
		return Coord{}, false
	}

	// Perform quick rejection test
	if IsRectCross(p0, p1, q0, q1) {
		// Perform straddle test
		if IsLineSegmentCross(p0, p1, q0, q1) {
			// Calculate the exact intersection point
			// Convert to float64 for precise calculation
			s1X := float64(p1.X - p0.X)
			s1Z := float64(p1.Z - p0.Z)
			s2X := float64(q1.X - q0.X)
			s2Z := float64(q1.Z - q0.Z)

			// Calculate parameter t for the intersection point
			// P = P0 + t * (P1 - P0)
			t := (s2X*float64(p0.Z-q0.Z) - s2Z*float64(p0.X-q0.X)) / (-s2X*s1Z + s1X*s2Z)

			// Calculate and return the intersection point
			return Coord{X: p0.X + int64(t*s1X), Z: p0.Z + int64(t*s1Z)}, true
		}
	}
	return Coord{}, false
}
