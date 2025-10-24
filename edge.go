package geo

// Vertice represents a vertex point in a geometric structure
// Each vertex has a unique identifier and a coordinate position
type Vertice struct {
	Index int64 // Unique identifier for the vertex
	Coord Coord // Coordinate position of the vertex
}

// Edge represents an edge connecting two vertices in a geometric structure
// It can be used in triangulation, pathfinding, or other geometric algorithms
type Edge struct {
	WtCoord            Coord       // Centroid/weighted center coordinate of the edge
	Vertices           [2]Vertice  // Array containing the two endpoint vertices (vertex 0 and vertex 1)
	AdjacenctTriangles []*Triangle // Adjacent triangles that share this edge (maximum of 2)
	Inflects           [2]Vertice  // Inflection points array - special points that may affect the edge's geometry
	IsAdjacency        bool        // Flag indicating whether this is an adjacency edge in the structure
}

// CalMidCoord calculates and returns the midpoint coordinate of the edge
// The midpoint is the center point between the two vertices of the edge
// Returns:
//
//	The coordinate of the edge's midpoint
func (e *Edge) CalMidCoord() Coord {
	return CalMidCoord(e.Vertices[0].Coord, e.Vertices[1].Coord)
}

// GenKey generates a unique key for this edge based on its vertex indices
// The key is used to uniquely identify the edge in data structures like maps
// Returns:
//
//	A unique int64 key for this edge
func (e *Edge) GenKey() int64 {
	return GenEdgeKey(e.Vertices[0].Index, e.Vertices[1].Index)
}

// GenEdgeKey generates a unique key for an edge given two vertex indices
// The key is order-independent: edge(i,j) and edge(j,i) will have the same key
// This is achieved by always putting the smaller index first
// Formula: key = 10000 * min(i,j) + max(i,j)
// Note: This assumes vertex indices are less than 10000
// Parameters:
//
//	i: index of the first vertex
//	j: index of the second vertex
//
// Returns:
//
//	A unique int64 key for the edge
func GenEdgeKey(i, j int64) int64 {
	if i < j {
		return 10000*i + j
	}
	return 10000*j + i
}
