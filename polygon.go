package geo

// Polygon represents a polygon interface that defines common operations for polygon shapes.
// Any polygon implementation must support coordinate containment checking, vector retrieval,
// bounding rectangle calculation, and vertex/edge access.
type Polygon interface {
	// IsCoordInside checks whether a given coordinate point is inside the polygon.
	// Returns true if the point is inside or on the boundary, false otherwise.
	IsCoordInside(p Coord) bool

	// GetVectors returns all edge vectors of the polygon.
	// Vectors are typically arranged in counter-clockwise order.
	GetVectors() []Vector

	// ToRect returns the minimum bounding rectangle of the polygon.
	// Returns minX, minZ, maxX, maxZ coordinates of the bounding box.
	ToRect() (minX, minZ, maxX, maxZ int64)

	// GetIndex returns the unique identifier of the polygon.
	GetIndex() int64

	// GetEdgeIDs returns the unique IDs of all edges in the polygon.
	GetEdgeIDs() []int64

	// GetEdgeMidCoords returns the midpoint coordinates of all edges.
	GetEdgeMidCoords() []Coord

	// GetVertices returns all vertices of the polygon.
	GetVertices() []Vertice
}

// CrossProduct calculates the cross product of three vertices (p1, p2, p3).
// The cross product is calculated using the vectors formed by p1->p2 and p2->p3.
// Returns:
//
//	 1 if the cross product is positive (counter-clockwise turn)
//	-1 if the cross product is negative (clockwise turn)
//	 0 if the points are collinear
//
// Formula: (p2-p1) × (p3-p2) = (ax)(bz) - (az)(bx)
// where a = p2-p1, b = p3-p2
func CrossProduct(p1, p2, p3 Vertice) int64 {
	// Calculate vector components from p1 to p2
	ax := p2.Coord.X - p1.Coord.X
	az := p2.Coord.Z - p1.Coord.Z

	// Calculate vector components from p2 to p3
	bx := p3.Coord.X - p2.Coord.X
	bz := p3.Coord.Z - p2.Coord.Z

	// Calculate cross product
	cp := ax*bz - az*bx

	// Return the sign of the cross product
	if cp > 0 {
		return 1
	} else if cp < 0 {
		return -1
	} else {
		return 0
	}
}

// IsConvex determines whether a polygon is convex.
// A polygon is convex if all cross products of consecutive edge vectors
// have the same sign (all positive or all negative).
//
// Algorithm:
// 1. For each vertex i, calculate the cross product of vectors:
//   - current edge: vertices[i] -> vertices[i+1]
//   - next edge: vertices[i] -> vertices[i+2]
//
// 2. If all cross products have the same sign, the polygon is convex
// 3. If there are both positive and negative cross products, it's concave
//
// Returns true if the polygon is convex, false if concave.
func IsConvex(vertices []Vertice) bool {
	numPoints := len(vertices)
	negativeFlag := false
	positiveFlag := false

	// Check the sign of cross products for all vertices
	for i := 0; i < numPoints; i++ {
		// Create vector from vertex i to vertex i+1
		curvec := NewVector(vertices[i].Coord, vertices[(i+1)%numPoints].Coord)

		// Create vector from vertex i to vertex i+2
		vec2next := NewVector(vertices[i].Coord, vertices[(i+2)%numPoints].Coord)

		// Calculate cross product and set flags
		if curvec.Cross(&vec2next) > 0 {
			positiveFlag = true
		} else {
			negativeFlag = true
		}
	}

	// Polygon is convex if all cross products have the same sign
	// XOR operation: returns true if only one flag is set
	return positiveFlag != negativeFlag
}
