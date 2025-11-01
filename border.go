package geo

// LocationState represents the position state within a border region
// It uses bit flags to indicate which quadrant(s) a shape or point occupies
type LocationState int

// LocationState constants define the four quadrants of a border
// Using bit flags allows combining multiple quadrants (e.g., LeftTop | RightTop)
const (
	LeftTop     = 1 << iota // Top-left quadrant (bit 0)
	RightTop                // Top-right quadrant (bit 1)
	LeftBottom              // Bottom-left quadrant (bit 2)
	RightBottom             // Bottom-right quadrant (bit 3)
)

// Border represents a rectangular boundary in 2D space
// It embeds a Rectangle structure for basic geometric operations
type Border struct {
	*Rectangle
}

// NewBorder creates a new Border instance with specified position and dimensions
// Parameters:
//
//	x: X-coordinate of the border's origin point
//	z: Z-coordinate of the border's origin point
//	width: Width of the border
//	height: Height of the border
//
// Returns:
//
//	A new Border instance
func NewBorder(x, z, width, height int64) Border {
	return Border{
		Rectangle: NewRectangle(x, z, width, height),
	}
}

// RectLocation determines which quadrant(s) of the border a rectangle overlaps
// It divides the border into 4 quadrants and checks rectangle intersection
// Parameters:
//
//	minX: Minimum X-coordinate of the rectangle
//	minZ: Minimum Z-coordinate of the rectangle
//	maxX: Maximum X-coordinate of the rectangle
//	maxZ: Maximum Z-coordinate of the rectangle
//
// Returns:
//
//	LocationState bit flags indicating overlapping quadrants, or 0 if no overlap
func (b *Border) RectLocation(minX, minZ, maxX, maxZ int64) LocationState {
	// Check if rectangle is completely outside the border
	if minX > b.X+b.Width ||
		minZ > b.Z+b.Height ||
		maxX < b.X ||
		maxZ < b.Z {
		return 0 // No intersection
	}

	var location LocationState
	// Calculate center point of the border
	centerX := b.X + b.Width/2
	centerZ := b.Z + b.Height/2

	// Check left half of the border
	if minX <= centerX {
		// Check if rectangle overlaps top-left quadrant
		if maxZ >= centerZ {
			location |= LeftTop
		}
		// Check if rectangle overlaps bottom-left quadrant
		if minZ <= centerZ {
			location |= LeftBottom
		}
	}

	// Check right half of the border
	if maxX > centerX {
		// Check if rectangle overlaps top-right quadrant
		if maxZ > centerZ {
			location |= RightTop
		}
		// Check if rectangle overlaps bottom-right quadrant
		if minZ < centerZ {
			location |= RightBottom
		}
	}
	return location
}

// CoordLocation determines which quadrant of the border a coordinate point is in
// Parameters:
//
//	p: Coordinate point to check
//
// Returns:
//
//	LocationState indicating the quadrant, or 0 if point is outside the border
func (b *Border) CoordLocation(p Coord) LocationState {
	// Check if point is outside the border
	if !b.IsCoordInside(p) {
		return 0
	}

	// Calculate center point of the border
	centerX := b.X + b.Width/2
	centerZ := b.Z + b.Height/2

	// Determine quadrant based on comparison with center point
	if p.X <= centerX {
		if p.Z >= centerZ {
			return LeftTop // Point is in top-left quadrant
		}
		return LeftBottom // Point is in bottom-left quadrant
	}
	if p.Z >= centerZ {
		return RightTop // Point is in top-right quadrant
	}
	return RightBottom // Point is in bottom-right quadrant
}
