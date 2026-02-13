package geo

// Line 直线 整型
type Line struct {
	A int32
	B int32
	C int32
}

// NewLine 新的直线方程
func NewLine(a, b Coord) Line {
	l := Line{
		A: b.Z - a.Z,
		B: a.X - b.X,
	}
	l.C = b.X*a.Z - a.X*b.Z
	return l
}

// IsCoordOnLine 点是否在直线上
func (l Line) IsCoordOnLine(c Coord) bool {
	return l.A*c.X+l.B*c.Z+l.C == 0
}

// IsValid 是否是有效的线
func (l Line) IsValid() bool {
	return !((l.A == 0) && (l.B == 0)) && (l.C == 0)
}
