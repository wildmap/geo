package geo

// Line 表示二维平面上的整数直线，以一般式方程 A*x + B*z + C = 0 描述。
// 使用 int64 系数是为了防止由两个 int32 坐标相减再相乘时的溢出问题，
// 保证在全坐标范围内点与直线关系判断的正确性。
type Line struct {
	A int64
	B int64
	C int64
}

// NewLine 通过两个坐标点构造直线方程 A*x + B*z + C = 0。
// 推导过程：直线经过点 a(xa, za) 和 b(xb, zb)，
// 则 A = zb - za，B = xa - xb，C = xb*za - xa*zb。
// 全部使用 int64 运算，避免 int32 溢出。
func NewLine(a, b Coord) Line {
	l := Line{
		A: int64(b.Z) - int64(a.Z),
		B: int64(a.X) - int64(b.X),
	}
	l.C = int64(b.X)*int64(a.Z) - int64(a.X)*int64(b.Z)
	return l
}

// IsCoordOnLine 判断给定坐标点是否严格位于直线上（代入方程后结果为 0）。
func (l Line) IsCoordOnLine(c Coord) bool {
	return l.A*int64(c.X)+l.B*int64(c.Z)+l.C == 0
}

// IsValid 判断直线方程是否有效（A 和 B 不能同时为 0，否则方程退化为常数）。
func (l Line) IsValid() bool {
	return !(l.A == 0 && l.B == 0)
}
