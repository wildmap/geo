package geo

import (
	"math"

	"github.com/wildmap/utility"
)

// Circle represents a geometric circle with a center coordinate and radius
type Circle struct {
	Center Coord // Center coordinate of the circle
	Radius int64 // Radius of the circle
}

// NewCirCle creates and returns a new Circle instance
// Parameters:
//   - center: the center coordinate of the circle
//   - radius: the radius of the circle
//
// Returns:
//   - Circle: a new circle object
func NewCirCle(center Coord, radius int64) Circle {
	return Circle{
		Center: center,
		Radius: radius,
	}
}

// GetLocationToBorder calculates the relative position between the circle and a border
// Parameters:
//   - b: pointer to a Border object
//
// Returns:
//   - LocationState: the relative position state between the circle and border
func (c *Circle) GetLocationToBorder(b *Border) LocationState {
	minX, minZ, maxX, maxZ := c.ToRect()
	return b.RectLocation(minX, minZ, maxX, maxZ)
}

// ToRect converts the circle to its bounding rectangle
// Returns:
//   - minX: minimum X coordinate of the bounding rectangle
//   - minZ: minimum Z coordinate of the bounding rectangle
//   - maxX: maximum X coordinate of the bounding rectangle
//   - maxZ: maximum Z coordinate of the bounding rectangle
func (c *Circle) ToRect() (minX, minZ, maxX, maxZ int64) {
	minX = c.Center.X - c.Radius
	minZ = c.Center.Z - c.Radius
	maxX = c.Center.X + c.Radius
	maxZ = c.Center.Z + c.Radius
	return
}

// GetIntersectCoord calculates the intersection point between the circle and
// the line segment from the circle's center to an external point
// Parameters:
//   - p: an external coordinate point
//
// Returns:
//   - Coord: the intersection point on the circle's perimeter
//
// Note: if point p coincides with the center, returns the center directly
func (c *Circle) GetIntersectCoord(p Coord) Coord {
	// If the center point coincides with p, return the center directly
	if c.Center == p {
		return c.Center
	}
	vec := NewVector(c.Center, p)
	length := vec.Length()
	ratio := float64(c.Radius) / length
	vec = vec.Trunc(ratio)
	return vec.ToCoord(c.Center)
}

// IsIntersect checks whether a line segment intersects with the circle
// Parameters:
//   - s: pointer to a Segment object
//
// Returns:
//   - bool: true if the segment intersects with the circle, false otherwise
func (c *Circle) IsIntersect(s *Segment) bool {
	_, ok := c.GetLineCross(s)
	return ok
}

// GetLineCross calculates the intersection points between a line segment and the circle
// If intersection points exist, returns the first one; otherwise returns false
// Parameters:
//   - s: pointer to a Segment object
//
// Returns:
//   - Coord: the first intersection point (if exists)
//   - bool: true if intersection exists, false otherwise
func (c *Circle) GetLineCross(s *Segment) (Coord, bool) {
	var coord1 *Coord
	var coord2 *Coord
	// Calculate the distance between segment endpoints
	fDis := CalDstCoordToCoord(s.A, s.B)

	// Calculate normalized direction vector components
	dx := float64(s.B.X-s.A.X) / fDis
	dz := float64(s.B.Z-s.A.Z) / fDis

	// Calculate vector from segment start point to circle center
	ex := float64(c.Center.X - s.A.X)
	ez := float64(c.Center.Z - s.A.Z)

	// Calculate projection of center onto segment line
	a := ex*dx + ez*dz
	a2 := a * a
	e2 := ex*ex + ez*ez
	r2 := float64(c.Radius * c.Radius)

	// Check if intersection exists using discriminant
	if utility.Smaller(r2-e2+a2, 0) {
		return Coord{}, false
	}

	// Calculate distance from projection point to intersection points
	f := math.Sqrt(r2 - e2 + a2)

	// Calculate first potential intersection point
	t := a - f
	if t > -utility.Epsilon && (t-fDis) < utility.Epsilon {
		coord1 = &Coord{
			X: s.A.X + int64(t*dx),
			Z: s.A.Z + int64(t*dz),
		}
	}

	// Calculate second potential intersection point
	t = a + f
	if t > -utility.Epsilon && (t-fDis) < utility.Epsilon {
		coord2 = &Coord{
			X: s.A.X + int64(t*dx),
			Z: s.A.Z + int64(t*dz),
		}
	}

	// Return the first valid intersection point
	if coord1 == nil {
		coord1 = coord2
	}
	if coord1 == nil {
		return Coord{}, false
	}
	return *coord1, true
}

// IsInterPolygon determines whether the circle intersects with a polygon
// This intersection detection is similar to collision detection
// Parameters:
//   - vectors: array of polygon vectors in counter-clockwise order
//
// Returns:
//   - bool: true if intersection exists, false otherwise
//
// Intersection cases (all return true):
//   - Circle is inside the polygon
//   - Polygon is inside the circle
//   - Circle and polygon partially overlap
//
// Non-intersection case (returns false):
//   - Circle and polygon are completely separated
//
// Reference: https://bitlush.com/blog/circle-vs-polygon-collision-detection-in-c-sharp
func (c *Circle) IsInterPolygon(vectors []Vector) bool {
	radiusSquared := float64(c.Radius * c.Radius)

	vertex := vectors[len(vectors)-1]
	center := NewVectorByCoord(c.Center)

	nearestDistance := math.MaxFloat64
	nearestIsInside := false
	nearestVertex := -1
	lastIsInside := false

	for i := 0; i < len(vectors); i++ {
		nextVertex := vectors[i]
		// Calculate axis from current vertex to circle center
		axis := center.Minus(&vertex)
		distance := axis.LengthSquared() - radiusSquared

		// If distance is negative or zero, the vertex is inside the circle
		if utility.SmallerOrEqual(distance, 0) {
			return true
		}

		isInside := false
		edge := nextVertex.Minus(&vertex)
		edgeLengthSquared := edge.LengthSquared()

		// Check if the circle intersects with the current edge
		if !utility.Equal(edgeLengthSquared, 0) {
			dot := edge.Dot(&axis)
			// Check if projection point is on the edge
			if utility.GreaterOrEqual(dot, 0) && utility.SmallerOrEqual(dot, edgeLengthSquared) {
				// Calculate projection point on the edge
				projection := edge.Trunc(dot / edgeLengthSquared)
				projection = vertex.Add(&projection)

				// Calculate axis from projection to center
				axis = projection.Minus(&center)
				// If projection distance is within radius, intersection exists
				if utility.SmallerOrEqual(axis.LengthSquared(), radiusSquared) {
					return true
				}

				// Check if the center is on the inside of the edge
				if !isInsideEdge(&edge, &axis) {
					return false
				}

				isInside = true
			}
		}

		// Track the nearest vertex
		if utility.Smaller(distance, nearestDistance) {
			nearestDistance = distance
			nearestIsInside = isInside || lastIsInside
			nearestVertex = i
		}

		vertex = nextVertex
		lastIsInside = isInside
	}

	// Final check for the first vertex
	if nearestVertex == 0 {
		return nearestIsInside || lastIsInside
	}
	return nearestIsInside
}

// isInsideEdge determines whether a point is on the inside of an edge
// Uses cross product to determine the side of the edge
// Parameters:
//   - edge: pointer to the edge vector
//   - axis: pointer to the axis vector from projection point to circle center
//
// Returns:
//   - bool: true if the point is inside the edge, false otherwise
func isInsideEdge(edge, axis *Vector) bool {
	switch {
	case edge.X > 0 && axis.Z > 0:
		return false
	case edge.X < 0 && axis.Z < 0:
		return false
	case edge.X == 0 && edge.Z > 0 && axis.X < 0:
		return false
	case edge.X == 0 && edge.Z <= 0 && axis.X > 0:
		return false
	}
	return true
}
