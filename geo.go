package geo

import (
	"math"

	"github.com/wildmap/utility"
)

// GetCoordsAround calculates coordinates around a circle to navigate from a point on the circle to an external point
// This is useful for pathfinding around circular obstacles
// Parameters:
//
//	startCoord: a point on the circle (starting position)
//	endCoord: a point outside the circle (target position)
//	centerCoord: the center of the circle
//
// Returns:
//
//	A slice of coordinates representing the path around the circle
func GetCoordsAround(startCoord, endCoord, centerCoord Coord) []Coord {
	centerVector := NewVector(centerCoord, startCoord)
	endVector := NewVector(centerCoord, endCoord)

	// Calculate the angle between the two vectors
	angle := centerVector.GetAngle(&endVector)
	radius := CalDstCoordToCoord(startCoord, centerCoord)
	// Calculate the tangent angle from the external point to the circle
	cutAngle := GetCutOffCoordAngle(endCoord, centerCoord, radius)
	angle -= cutAngle
	// Use cross product to determine the rotation direction
	if centerVector.Cross(&endVector) > 0 {
		angle = -angle
	}
	coords := GetArcCoords(startCoord, centerCoord, angle)

	return coords
}

// GetCoordsAround2 generates evenly distributed points around a circle
// Parameters:
//
//	circleCoord: a point on the circle's circumference
//	centerCoord: the center of the circle
//	n: the number of evenly distributed points to generate
//
// Returns:
//
//	A slice of n coordinates evenly distributed around the circle
func GetCoordsAround2(circleCoord, centerCoord Coord, n int) []Coord {
	return getCoordsAround(circleCoord, centerCoord, n, 2*math.Pi)
}

// getCoordsAround generates points along a circular arc starting from startCoord
// Parameters:
//
//	startCoord: the starting point on the circle
//	centerCoord: the center of the circle
//	n: the number of sample points to generate (must be >= 2)
//	angle: the total angle to cover (in radians)
//
// Returns:
//
//	A slice of n coordinates along the arc, including startCoord
func getCoordsAround(startCoord, centerCoord Coord, n int, angle float64) []Coord {
	if n < 2 {
		return nil
	}

	ret := make([]Coord, n)
	ret[0] = startCoord

	vec := NewVector(centerCoord, startCoord)
	ave := angle / float64(n-1) // Average angle between consecutive points
	angle = 0
	for i := 1; i < n; i++ {
		angle += ave
		v := vec.Rotate(angle)
		ret[i] = v.ToCoord(centerCoord)
	}

	return ret
}

// GetArcCoords generates coordinates along a circular arc
// The arc starts from startCoord and covers the specified angle
// Parameters:
//
//	startCoord: the starting point of the arc
//	centerCoord: the center of the circle
//	angle: the angle to cover in radians (>0 for clockwise, <0 for counterclockwise)
//
// Returns:
//
//	A slice of coordinates representing points along the arc
func GetArcCoords(startCoord, centerCoord Coord, angle float64) []Coord {
	// Generate approximately 34 points per pi radians for smooth curves
	n := max(int64(utility.Abs(angle)*10), 2)
	return getCoordsAround(startCoord, centerCoord, int(n), -angle)
}

// GetSpiralCoords generates coordinates along a spiral path
// The spiral starts at startCoord and gradually changes its radius while rotating
// Parameters:
//
//	startCoord: the starting coordinate of the spiral
//	centerCoord: the center point of the spiral
//	angle: the total angle to rotate in radians (>0 for clockwise, <0 for counterclockwise)
//	delta: the change in radius from start to end (distance difference from center)
//
// Returns:
//
//	A slice of coordinates representing the spiral path
func GetSpiralCoords(startCoord, centerCoord Coord, angle, delta float64) []Coord {
	n := int64(utility.Abs(angle) * 10)
	if n < 2 {
		// If angle is too small, just return two points with radius adjustment
		dst := CalDstCoordToCoord(startCoord, centerCoord)
		vec := NewVector(centerCoord, startCoord)
		vec = vec.Trunc((delta + dst) / dst)
		return []Coord{startCoord, vec.ToCoord(centerCoord)}
	}

	ret := make([]Coord, n)
	ret[0] = startCoord

	aveAngle := angle / float64(n)   // Average angle increment per step
	aveDelta := (delta) / float64(n) // Average radius increment per step
	angle = 0
	delta = 0

	vec := NewVector(centerCoord, startCoord)
	radius := vec.Length()
	for i := int64(1); i < n; i++ {
		// Rotate the original vector by the accumulated angle
		angle += aveAngle
		v := vec.Rotate(angle)
		// Increase the vector length to create the spiral effect
		delta += aveDelta
		v = v.Trunc((radius + delta) / radius)

		ret[i] = v.ToCoord(centerCoord)
	}

	return ret
}

// GetIntersectCoord finds the intersection point between a line segment and a circle
// The line segment goes from the circle's center to an external point
// Parameters:
//
//	centerP: the center of the circle
//	endP: an external point
//	radius: the radius of the circle
//
// Returns:
//
//	The coordinate where the line from centerP to endP intersects the circle
func GetIntersectCoord(centerP, endP Coord, radius int64) Coord {
	c := NewCirCle(centerP, radius)
	return c.GetIntersectCoord(endP)
}

// GetLineCrossCircle finds the intersection point between a line segment and a circle
// Parameters:
//
//	startP: the starting point of the line segment
//	endP: the ending point of the line segment
//	centerP: the center of the circle
//	radius: the radius of the circle
//
// Returns:
//
//	coord: the first intersection point if it exists
//	ok: true if an intersection exists, false otherwise
func GetLineCrossCircle(startP, endP, centerP Coord, radius int64) (Coord, bool) {
	c := NewCirCle(centerP, radius)
	seg := NewSegment(startP, endP)
	return c.GetLineCross(&seg)
}

// GetCutOffCoordAngle calculates the angle of the tangent line from an external point to a circle
// Given a point outside a circle, this calculates the angle between the line to the center
// and the tangent line to the circle
// Mathematical basis: In a right triangle formed by the tangent point (p), center (c), and external point (e):
//
//	ep ⊥ cp (tangent is perpendicular to radius)
//	cos(∠ecp) = cp / ce = radius / distance
//
// Parameters:
//
//	endCoord: the external point
//	centerCoord: the center of the circle
//	radius: the radius of the circle
//
// Returns:
//
//	The angle in radians between the line to center and the tangent line
func GetCutOffCoordAngle(endCoord, centerCoord Coord, radius float64) float64 {
	dst := CalDstCoordToCoord(endCoord, centerCoord)
	a := math.Acos(radius / dst)

	if math.IsNaN(a) {
		return 0.
	}

	return a
}

// GetBresenhamCoord generates coordinates along a line using Bresenham's line algorithm
// This algorithm efficiently rasterizes a line by using only integer arithmetic
// Reference: https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm
// Parameters:
//
//	p1: starting point of the line
//	p2: ending point of the line
//
// Returns:
//
//	A slice of coordinates representing the rasterized line from p1 to p2
func GetBresenhamCoord(p1, p2 Coord) []Coord {
	xstart := p1.X
	zstart := p1.Z
	xend := p2.X
	zend := p2.Z

	var steep = false   // Flag for steep lines (|slope| > 1)
	var swapped = false // Flag to track if endpoints were swapped

	// If the line is steep (more vertical than horizontal), transpose it
	if utility.Abs(zend-zstart) > utility.Abs(xend-xstart) {
		steep = true
	}

	if steep {
		xstart, zstart = zstart, xstart
		xend, zend = zend, xend
	}
	// Ensure we're always drawing from left to right
	if xstart > xend {
		xstart, zstart = zstart, xstart
		xend, zend = zend, xend
		swapped = true
	}
	var deltax = xend - xstart
	var deltaz = utility.Abs(zend - zstart)
	var err = deltax / 2 // Error accumulator
	var zstep int64      // Direction to step in z
	z := zstart
	if zstart < zend {
		zstep = 1
	} else {
		zstep = -1
	}
	tmpCoordList := make([]Coord, 0, xend-xstart+1)
	for x := xstart; x <= xend; x++ {
		var c Coord
		// If the line was steep, transpose back
		if steep {
			c = Coord{
				X: z, Z: x,
			}
		} else {
			c = Coord{
				X: x, Z: z,
			}
		}
		tmpCoordList = append(tmpCoordList, c)
		err -= deltaz
		if err < 0 {
			z += zstep
			err += deltax
		}
	}

	// If we swapped endpoints, reverse the result to maintain correct order
	if swapped {
		reverse(tmpCoordList)
	}
	return tmpCoordList
}

// reverse reverses a slice of coordinates in place
// This is a helper function used by GetBresenhamCoord
// Parameters:
//
//	slice: the slice to reverse
func reverse(slice []Coord) {
	for i, j := 0, len(slice)-1; j > i; i, j = i+1, j-1 {
		slice[i], slice[j] = slice[j], slice[i]
	}
}

// CalMidCoord calculates the midpoint between two coordinates
// Parameters:
//
//	p1: first coordinate
//	p2: second coordinate
//
// Returns:
//
//	The coordinate at the midpoint between p1 and p2
func CalMidCoord(p1, p2 Coord) Coord {
	return Coord{
		X: (p1.X + p2.X) / 2,
		Z: (p1.Z + p2.Z) / 2,
	}
}

// GetCrossRect calculates all the rectangular grid cells that a line segment passes through
// The area is divided into a grid of small rectangles, and this function returns
// all rectangles that the line from p0 to p1 crosses
// Parameters:
//
//	p0, p1: the start and end points of the line segment
//	rWidth, rHeight: the width and height of each small rectangle in the grid
//	width, height: the total width and height of the area
//
// Returns:
//
//	A map where keys are the bottom-left corner coordinates of rectangles that the line crosses
func GetCrossRect(p0, p1 Coord, rWidth, rHeight, width, height int64) map[Coord]bool {
	coordSet := map[Coord]bool{}
	// Add the rectangles containing the start and end points
	startCoords := []Coord{p0, p1}
	for i := 0; i < len(startCoords); i++ {
		c := Coord{X: (startCoords[i].X / rWidth) * rWidth, Z: (startCoords[i].Z / rHeight) * rHeight}
		_, exist := coordSet[c]
		if !exist {
			coordSet[c] = true
		}
	}
	// Sort points by Z coordinate (ascending order)
	zStart := p0
	zEnd := p1
	if p1.Z < p0.Z {
		zStart = p1
		zEnd = p0
	}
	startZ := zStart.Z / rHeight
	endZ := zEnd.Z / rHeight
	// Traverse horizontal grid lines
	for i := startZ; i <= endZ; i++ {
		q0 := &Coord{X: 0, Z: i * rHeight}
		q1 := &Coord{X: width, Z: i * rHeight}
		// Calculate intersection point with horizontal grid line
		coord, ok := GetCrossCoord(p0, p1, *q0, *q1)
		if ok {
			// Correct for rounding errors
			coord.Z = i * rHeight
			upCoord := Coord{X: (coord.X / rHeight) * rHeight, Z: (coord.Z / rHeight) * rHeight}
			downCoord := Coord{X: upCoord.X, Z: (coord.Z/rHeight - 1) * rHeight}
			// Add both rectangles above and below the intersection
			// If the intersection is exactly at a grid point, this may include some extra rectangles
			_, exist := coordSet[upCoord]
			if !exist {
				coordSet[upCoord] = true
			}
			_, exist = coordSet[downCoord]
			if !exist {
				coordSet[downCoord] = true
			}
		}
	}

	// Sort points by X coordinate (ascending order)
	xStart := p0
	xEnd := p1
	if p1.X < p0.X {
		xStart = p1
		xEnd = p0
	}
	// Traverse vertical grid lines
	startX := xStart.X / rWidth
	endX := xEnd.X / rWidth
	for i := startX; i <= endX; i++ {
		q0 := &Coord{X: i * rWidth, Z: 0}
		q1 := &Coord{X: i * rWidth, Z: height}
		// Calculate intersection point with vertical grid line
		coord, ok := GetCrossCoord(p0, p1, *q0, *q1)
		if ok {
			coord.X = i * rWidth
			rightCoord := Coord{X: (coord.X / rWidth) * rWidth, Z: (coord.Z / rHeight) * rHeight}
			leftCoord := Coord{X: (coord.X/rWidth - 1) * rWidth, Z: rightCoord.Z}
			// Add both rectangles to the left and right of the intersection
			_, exist := coordSet[rightCoord]
			if !exist {
				coordSet[rightCoord] = true
			}
			_, exist = coordSet[leftCoord]
			if !exist {
				coordSet[leftCoord] = true
			}
		}
	}

	return coordSet
}

// GetIntersectRect calculates the intersection area of two rectangles
// Parameters:
//
//	r0: the first rectangle
//	r1: the second rectangle
//
// Returns:
//
//	rect: the intersecting rectangle
//	ok: true if the rectangles intersect, false otherwise
func GetIntersectRect(r0 Rectangle, r1 Rectangle) (Rectangle, bool) {
	rect := Rectangle{}
	// The intersection rectangle's bottom-left corner is at the maximum of both rectangles' bottom-left corners
	rect.X = max(r0.X, r1.X)
	rect.Z = max(r0.Z, r1.Z)
	// The intersection rectangle's width and height are determined by the minimum of the top-right corners
	rect.Width = min(r0.X+r0.Width, r1.X+r1.Width) - rect.X
	rect.Height = min(r0.Z+r0.Height, r1.Z+r1.Height) - rect.Z

	// Check if there is a valid intersection (width and height must be positive)
	if rect.X >= rect.X+rect.Width || rect.Z >= rect.Z+rect.Height {
		return rect, false
	}
	return rect, true
}
