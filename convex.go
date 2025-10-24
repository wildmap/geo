package geo

import (
	"fmt"
	"log/slog"
	"math"
)

// Convex represents a convex polygon
type Convex struct {
	Index          int64       // Unique identifier for the convex polygon
	Vertices       []Vertice   // Vertices of the polygon
	MergeTriangles []*Triangle // Triangles that compose this convex polygon
	EdgeIDs        []int64     // IDs of the edges
	WtCoord        Coord       // Weight coordinate (center point)
}

// NewConvex converts a triangle to a convex polygon
// Parameters:
//   - t: pointer to a Triangle object
//   - id: unique identifier for the convex polygon
//
// Returns:
//   - *Convex: pointer to the newly created Convex object
func NewConvex(t *Triangle, id int64) *Convex {
	return &Convex{
		Index: id,
		Vertices: []Vertice{
			t.Vertices[0],
			t.Vertices[1],
			t.Vertices[2],
		},
		MergeTriangles: []*Triangle{t},
		EdgeIDs:        t.EdgeIDs,
	}
}

// ToRect gets the rectangular boundary of the convex polygon
// Returns:
//   - minX: minimum X coordinate
//   - minZ: minimum Z coordinate
//   - maxX: maximum X coordinate
//   - maxZ: maximum Z coordinate
func (c *Convex) ToRect() (minX, minZ, maxX, maxZ int64) {
	minX = int64(math.MaxInt64)
	minZ = int64(math.MaxInt64)
	for _, v := range c.Vertices {
		minX = min(v.Coord.X, minX)
		minZ = min(v.Coord.Z, minZ)
		maxX = max(v.Coord.X, maxX)
		maxZ = max(v.Coord.Z, maxZ)
	}
	return minX, minZ, maxX, maxZ
}

// MergeTriangle merges a new triangle into the convex polygon
// Attempts to insert new vertices while maintaining convexity
// Parameters:
//   - p1, p2: two shared vertices between the convex and triangle
//   - p3: new vertices from the triangle to be merged
//
// Returns:
//   - bool: true if merge is successful, false otherwise
func (c *Convex) MergeTriangle(p1, p2 Vertice, p3 []Vertice) bool {
	for index, v := range c.Vertices {
		if v.Index == p1.Index || v.Index == p2.Index {
			newVertices := make([]Vertice, len(c.Vertices))
			copy(newVertices, c.Vertices)
			// Insert new vertices at the correct position
			if index != 0 || c.Vertices[index+1].Index == p2.Index || c.Vertices[index+1].Index == p1.Index {
				// Try inserting twice, as the order of vertices is uncertain
				for i := 0; i < 2; i++ {
					newVertices = make([]Vertice, len(c.Vertices)+len(p3))
					insertIndex := 0
					// Copy vertices before insertion point
					for insertIndex <= index {
						newVertices[insertIndex] = c.Vertices[insertIndex]
						insertIndex++
					}
					// Insert new vertices
					for _, p := range p3 {
						newVertices[insertIndex] = p
						insertIndex++
					}
					// Copy remaining vertices
					for _, p := range c.Vertices[index+1:] {
						newVertices[insertIndex] = p
						insertIndex++
					}
					// Check if the result is still convex
					if IsConvex(newVertices) {
						c.Vertices = newVertices
						return true
					}
					// Try reverse order
					for i, j := 0, len(p3)-1; i < j; i, j = i+1, j-1 {
						p3[i], p3[j] = p3[j], p3[i]
					}
				}
			} else {
				// Append new vertices at the end
				for i := 0; i < 2; i++ {
					newVertices = make([]Vertice, len(c.Vertices))
					copy(newVertices, c.Vertices)
					newVertices = append(newVertices, p3...)
					if IsConvex(newVertices) {
						c.Vertices = newVertices
						return true
					}
					// Try reverse order
					for i, j := 0, len(p3)-1; i < j; i, j = i+1, j-1 {
						p3[i], p3[j] = p3[j], p3[i]
					}
				}
			}
			return false
		}
	}
	return false
}

// GetVectors gets the vector array of the convex polygon in counter-clockwise order
// If adjacent edges' cross product is greater than 0, it's counter-clockwise
// Returns:
//   - []Vector: array of vectors representing the polygon edges
func (c *Convex) GetVectors() []Vector {
	c.CounterClockWiseSort()
	vecs := make([]Vector, 0, len(c.Vertices))
	for _, v := range c.Vertices {
		vecs = append(vecs, NewVectorByCoord(v.Coord))
	}
	return vecs
}

// TriangleHasCoord returns the triangle that contains the given point
// Parameters:
//   - p: coordinate point to check
//
// Returns:
//   - int64: index of the triangle containing the point, or -1 if not found
func (c *Convex) TriangleHasCoord(p Coord) int64 {
	for _, t := range c.MergeTriangles {
		if t.IsCoordInside(p) {
			return t.GetIndex()
		}
	}
	return -1
}

// IsCoordInside1 determines if a point is inside the convex polygon using ray casting method
// Parameters:
//   - p: coordinate point to check
//
// Returns:
//   - bool: true if the point is inside the polygon, false otherwise
func (c *Convex) IsCoordInside1(p Coord) bool {
	// If only one triangle, use triangle's method directly
	if len(c.MergeTriangles) == 1 {
		return c.MergeTriangles[0].IsCoordInside(p)
	}
	c.CounterClockWiseSort()
	x := p.X
	z := p.Z
	sz := len(c.Vertices)
	isIn := false

	for i := 0; i < sz; i++ {
		j := i - 1
		if i == 0 {
			j = sz - 1
		}
		vi := c.Vertices[i]
		vj := c.Vertices[j]

		// Calculate bounding box for edge
		xmin := vi.Coord.X
		xmax := vj.Coord.X
		if xmin > xmax {
			t := xmin
			xmin = xmax
			xmax = t
		}
		zmin := vi.Coord.Z
		zmax := vj.Coord.Z
		if zmin > zmax {
			t := zmin
			zmin = zmax
			zmax = t
		}
		// Check if edge is parallel to x-axis
		if vj.Coord.Z == vi.Coord.Z {
			if z == vi.Coord.Z && xmin <= x && x <= xmax {
				return true // Point is on the edge
			}
			continue
		}

		// Calculate intersection point with horizontal ray
		xt := (vj.Coord.X-vi.Coord.X)*(z-vi.Coord.Z)/(vj.Coord.Z-vi.Coord.Z) + vi.Coord.X
		if xt == x && zmin <= z && z <= zmax {
			// Point is on the edge [vj,vi]
			return true
		}
		if x < xt && zmin <= z && z < zmax {
			isIn = !isIn
		}
	}
	return isIn
}

// IsCoordInside2 checks if a point is inside the convex using binary search method
// This method is more efficient for convex polygons with many vertices
// Parameters:
//   - p: coordinate point to check
//
// Returns:
//   - bool: true if the point is inside the polygon, false otherwise
func (c *Convex) IsCoordInside2(p Coord) bool {
	numOfVertice := len(c.Vertices)
	target := Vertice{Coord: p}
	vec1 := NewVector(c.Vertices[0].Coord, c.Vertices[1].Coord)
	vec2 := NewVector(c.Vertices[0].Coord, c.Vertices[numOfVertice-1].Coord)
	vec2Coord := NewVector(c.Vertices[0].Coord, p)
	vec2CoordLen := vec2Coord.Length()
	cp1 := vec1.Cross(&vec2Coord)
	cp2 := vec2.Cross(&vec2Coord)
	isCounterClockwise := cp1 > 0

	// Check if point is on the edges connecting to vertex 0
	if cp1 == 0 && vec2CoordLen <= vec1.Length() || cp2 == 0 && vec2CoordLen <= vec2.Length() {
		return true
	}

	// Step 1: Check if the point is between two vectors from point 0
	if (cp1 > 0) == (cp2 > 0) {
		return false
	}

	s := 1
	e := numOfVertice - 1
	// Step 2: Use binary search to determine which two vertices the point is between
	for e != s+1 {
		m := (s + e) / 2
		if (CrossProduct(target, c.Vertices[m], c.Vertices[0]) > 0) == isCounterClockwise {
			// Target is in clockwise direction from m
			e = m
		} else {
			// Target is in counter-clockwise direction from m
			s = m
		}
	}

	// Step 3: Check the final triangle formed by vertices 0, s, and e
	vec3 := NewVector(c.Vertices[s].Coord, c.Vertices[e].Coord)
	vecS2Coord := NewVector(c.Vertices[s].Coord, p)
	return vec3.Cross(&vecS2Coord) > 0 == isCounterClockwise
}

// IsCoordInside checks if a point is inside the convex polygon
// This is the recommended method for general use
// Parameters:
//   - p: coordinate point to check
//
// Returns:
//   - bool: true if the point is inside the polygon, false otherwise
func (c *Convex) IsCoordInside(p Coord) bool {
	numOfVertice := len(c.Vertices)
	vec1 := NewVector(c.Vertices[0].Coord, c.Vertices[1].Coord)
	vec2 := NewVector(c.Vertices[0].Coord, p)
	isCounterClockwise := vec1.Cross(&vec2) > 0

	// Check all edges to ensure point is on the same side
	for i := 1; i < numOfVertice; i++ {
		vec1 := NewVector(c.Vertices[i].Coord, c.Vertices[(i+1)%numOfVertice].Coord)
		vec2 := NewVector(c.Vertices[i].Coord, p)
		if (vec1.Cross(&vec2) > 0) != isCounterClockwise {
			return false
		}
	}
	return true
}

// CheckConvex validates if the convex polygon is correctly composed
// Used for testing and debugging purposes
// Returns:
//   - bool: true if the convex is valid, false otherwise
func (c *Convex) CheckConvex() bool {
	vers := make(map[int64]bool)
	for _, ver := range c.Vertices {
		vers[ver.Index] = false
	}

	// Check if the polygon is actually convex
	if !IsConvex(c.Vertices) {
		slog.Error("convex error not a convex")
		return false
	}

	count1 := len(vers)
	// Mark all vertices from merged triangles
	for _, triangle := range c.MergeTriangles {
		for _, ver := range triangle.GetVertices() {
			vers[ver.Index] = true
		}
	}
	count2 := len(vers)

	// Check if all vertices are covered by merged triangles
	for key, value := range vers {
		if !value {
			slog.Error(fmt.Sprintf("convex error has not exist value %d", key))
			return false
		}
	}

	// Verify vertex counts match
	if count1 != count2 {
		slog.Error(fmt.Sprintf("convex error has not merged. vertice convex vertices count: %d merge vertices count: %d", count1, count2))
		return false
	}
	return true
}

// CounterClockWiseSort sorts vertices in counter-clockwise order
// If vertices are in clockwise order, reverses them
func (c *Convex) CounterClockWiseSort() {
	if CrossProduct(c.Vertices[0], c.Vertices[1], c.Vertices[2]) < 0 {
		// Currently in clockwise order, reverse the array
		for i, j := 0, len(c.Vertices)-1; i < j; i, j = i+1, j-1 {
			c.Vertices[i], c.Vertices[j] = c.Vertices[j], c.Vertices[i]
		}
	}
}

// GetIndex returns the unique identifier of the convex polygon
// Returns:
//   - int64: the index of the convex polygon
func (c *Convex) GetIndex() int64 {
	return c.Index
}

// GetNeighborPoints gets the neighboring points between two convex polygons
// Parameters:
//   - t2: the other polygon to check adjacency with
//
// Returns:
//   - []Vertice: array of vertices with the first two being on the shared edge,
//     or nil if not adjacent (requires exactly 2 shared vertices)
func (c *Convex) GetNeighborPoints(t2 Polygon) []Vertice {
	vertices := t2.GetVertices()
	numOfVecs := len(vertices)
	pointsIndexs := make([]int, 0, numOfVecs)

	// Find all shared vertices
	for _, v := range c.Vertices {
		for j := range vertices {
			if v.Index == vertices[j].Index {
				pointsIndexs = append(pointsIndexs, j)
			}
		}
	}

	// Return vertices with shared edge first, only if exactly 2 vertices are shared
	if len(pointsIndexs) == 2 {
		points := make([]Vertice, numOfVecs)
		first := pointsIndexs[0]
		second := pointsIndexs[1]
		// If first's next vertex is not second, swap them
		if (first+1)%numOfVecs != second {
			first = second
		}
		// Rearrange vertices starting from the shared edge
		for i := 0; i < numOfVecs; i++ {
			points[i] = vertices[(first+i)%numOfVecs]
		}
		return points
	}
	return nil
}

// GetVertices returns the list of vertices
// Returns:
//   - []Vertice: array of all vertices in the polygon
func (c *Convex) GetVertices() []Vertice {
	return c.Vertices
}

// GetCenterCoord calculates the geometric center (centroid) of the polygon
// Uses simple average of all vertex coordinates
// Returns:
//   - Coord: the center coordinate
func (c *Convex) GetCenterCoord() Coord {
	coordx := int64(0)
	coordz := int64(0)
	length := int64(len(c.Vertices))
	for _, v := range c.Vertices {
		coordx += v.Coord.X
		coordz += v.Coord.Z
	}
	return Coord{coordx / length, coordz / length}
}

// GetCenterCoord1 calculates the center of mass (barycenter) of the polygon
// Uses the area-weighted method for more accurate results
// Returns:
//   - Coord: the center of mass coordinate
func (c *Convex) GetCenterCoord1() Coord {
	// Calculate total signed area
	S := int64(0)
	for i := 0; i < len(c.Vertices)-1; i++ {
		S += c.Vertices[i].Coord.X*c.Vertices[i+1].Coord.Z - c.Vertices[i+1].Coord.X*c.Vertices[i].Coord.Z
	}
	S /= 2

	// Calculate center of mass coordinates
	centerX := int64(0)
	centerY := int64(0)
	for i := 0; i < len(c.Vertices)-1; i++ {
		xi := c.Vertices[i].Coord.X
		zi := c.Vertices[i].Coord.Z
		xni := c.Vertices[i+1].Coord.X
		zni := c.Vertices[i+1].Coord.Z
		centerX += (xi + xni) * (xi*zni - xni*zi)
		centerY += (zi + zni) * (xi*zni - xni*zi)
	}
	centerX /= 6 * S
	centerY /= 6 * S
	return Coord{
		X: centerX,
		Z: centerY,
	}
}

// GetEdgeIDs returns the list of edge IDs for the polygon
// Returns:
//   - []int64: array of edge identifiers
func (c *Convex) GetEdgeIDs() []int64 {
	return c.EdgeIDs
}

// GetEdgeMidCoords gets the midpoint coordinates of all edges
// Returns:
//   - []Coord: array of midpoint coordinates for each edge
func (c *Convex) GetEdgeMidCoords() []Coord {
	nums := len(c.Vertices)
	coords := make([]Coord, 0, nums)
	for i, v := range c.Vertices {
		coords = append(coords, CalMidCoord(v.Coord, c.Vertices[(i+1)%nums].Coord))
	}
	return coords
}
