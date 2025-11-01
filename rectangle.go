package geo

import "math/rand"

// Rectangle represents a rectangular shape defined by its bottom-left corner
// and its width and height dimensions.
// The rectangle is aligned with the coordinate axes (axis-aligned bounding box).
type Rectangle struct {
	Coord        // Bottom-left corner point (embedded Coord struct)
	Width  int64 // Width of the rectangle (along X-axis)
	Height int64 // Height of the rectangle (along Z-axis)
}

// NewRectangle creates and returns a new Rectangle instance.
// Parameters:
//
//	x, z - coordinates of the bottom-left corner
//	width - width of the rectangle
//	height - height of the rectangle
func NewRectangle(x, z, width, height int64) *Rectangle {
	return &Rectangle{
		Coord: Coord{
			X: x,
			Z: z,
		},
		Width:  width,
		Height: height,
	}
}

// RandCoord generates a random coordinate point within the rectangle.
// The random point is uniformly distributed within the rectangle's area.
// Returns a Coord with random X and Z values within the rectangle bounds.
func (rec *Rectangle) RandCoord() Coord {
	return Coord{
		X: rec.X + rand.Int63n(rec.Width),
		Z: rec.Z + rand.Int63n(rec.Height),
	}
}

// GetVerticeCoords returns the four corner vertices of the rectangle.
// The vertices are arranged in counter-clockwise order starting from
// the bottom-left corner:
// [0] = bottom-left
// [1] = bottom-right
// [2] = top-right
// [3] = top-left
func (rec *Rectangle) GetVerticeCoords() [4]Coord {
	var p [4]Coord
	p[0] = Coord{X: rec.Coord.X, Z: rec.Coord.Z}                          // Bottom-left
	p[1] = Coord{X: rec.Coord.X + rec.Width, Z: rec.Coord.Z}              // Bottom-right
	p[2] = Coord{X: rec.Coord.X + rec.Width, Z: rec.Coord.Z + rec.Height} // Top-right
	p[3] = Coord{X: rec.Coord.X, Z: rec.Coord.Z + rec.Height}             // Top-left
	return p
}

// GetVectors returns the four edge vectors of the rectangle.
// The vectors are arranged in counter-clockwise order, representing
// the four edges of the rectangle as position vectors from the origin.
// Each vector points to a vertex of the rectangle.
func (rec *Rectangle) GetVectors() [4]Vector {
	coords := rec.GetVerticeCoords()
	return [4]Vector{
		NewVectorByCoord(coords[0]),
		NewVectorByCoord(coords[1]),
		NewVectorByCoord(coords[2]),
		NewVectorByCoord(coords[3]),
	}
}

// GetLocationToBorder determines the spatial relationship between this rectangle
// and a given border.
// Returns a LocationState indicating whether the rectangle is inside, outside,
// or intersecting the border.
func (rec *Rectangle) GetLocationToBorder(b *Border) LocationState {
	minX := rec.X
	maxX := rec.X + rec.Width
	minZ := rec.Z
	maxZ := rec.Z + rec.Height
	return b.RectLocation(minX, minZ, maxX, maxZ)
}

// IsCoordInside checks whether a given point is inside or on the boundary of the rectangle.
//
// Algorithm:
// The rectangle's vectors are arranged counter-clockwise, so when calculating
// the cross product between point p and each edge vector, if all cross products
// are >= 0, the point is on the same side of all edges, meaning it's inside.
//
// Note: Collinear cases (cross product = 0) are also considered as being on the
// same side, so points on the boundary are considered inside.
//
// Steps:
// 1. Create vectors from point p to each of the four vertices
// 2. Calculate cross products between consecutive vertex vectors
// 3. If all cross products have the same sign (>=0), point is inside
//
// Returns true if the point is inside or on the rectangle, false otherwise.
func (rec *Rectangle) IsCoordInside(p Coord) bool {
	// Get the four vertices of the rectangle
	pts := rec.GetVerticeCoords()

	// Create vectors from point p to each vertex
	pa := NewVector(p, pts[0])
	pb := NewVector(p, pts[1])
	pc := NewVector(p, pts[2])

	// Check cross product between first two edges
	b1 := pa.Cross(&pb) >= 0
	b2 := pb.Cross(&pc) >= 0
	if b1 != b2 {
		return false
	}

	// Check cross product with third edge
	pd := NewVector(p, pts[3])
	b3 := pc.Cross(&pd) >= 0
	if b2 != b3 {
		return false
	}

	// Check cross product with fourth edge
	b4 := pd.Cross(&pa) >= 0
	return b3 == b4
}
