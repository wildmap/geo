package geo

// Segment 表示由两个端点 A、B 定义的有向线段。
// 有向性体现在法向量方向（Pan 方法）及部分交点算法中，
// 但点包含、距离等对称计算与方向无关。
type Segment struct {
	A, B Coord
}

// NewSegment 以两个端点创建线段对象。
func NewSegment(a, b Coord) Segment {
	return Segment{
		A: a,
		B: b,
	}
}

// ToVector 将线段转换为从 A 指向 B 的向量。
func (s *Segment) ToVector() Vector {
	return NewVector(s.A, s.B)
}

// CalCoordDst 计算给定点到线段的最短距离。
// 复用 ClosestPoint 确保点在端点延长线方向时也能正确返回端点距离，
// 避免重复的距离计算逻辑。
func (s *Segment) CalCoordDst(coord Coord) float64 {
	return CalDstCoordToCoord(coord, s.ClosestPoint(coord))
}

// Pan 将线段沿法向量方向平行移动指定距离。
// 在左手坐标系中，法向量通过将方向向量旋转 90° 得到：
// 正方向法向量为 (-Z, X)（叉积为正的方向），负方向为 (Z, -X)。
// 移动比例 = 目标距离 / 线段长度，再对法向量等比缩放，保证移动精度。
// 退化线段（A == B，长度为 0）无法确定方向，直接返回原线段。
func (s *Segment) Pan(dst int32, positive bool) Segment {
	v := s.ToVector()
	length := v.Length()
	// 退化线段无法确定法向量方向，安全返回原线段
	if length < 1e-9 {
		return *s
	}
	// 法向量：将方向向量左旋 90° 得到正方向法向量 (-Z, X)
	normalV := NewVectorByCoord(Coord{X: -v.Z, Z: v.X})
	if !positive {
		normalV = NewVectorByCoord(Coord{X: v.Z, Z: -v.X})
	}
	ratio := float64(dst) / length
	newV := normalV.Trunc(ratio)

	return NewSegment(newV.ToCoord(s.A), newV.ToCoord(s.B))
}

// CrossCircle 判断线段是否与圆相交，相交时返回第一个交点。
func (s *Segment) CrossCircle(circle Circle) (Coord, bool) {
	return GetLineCrossCircle(s.A, s.B, circle.Center, circle.Radius)
}

// IsRectCross 快速排斥实验：判断两条线段的轴对齐包围盒是否重叠。
// 排斥实验是线段相交判断的第一步：若包围盒不重叠，线段必不相交，
// 可以提前剪枝，避免后续更昂贵的跨立实验。
func IsRectCross(p0, p1, q0, q1 Coord) bool {
	ret := min(p0.X, p1.X) <= max(q0.X, q1.X) &&
		min(q0.X, q1.X) <= max(p0.X, p1.X) &&
		min(p0.Z, p1.Z) <= max(q0.Z, q1.Z) &&
		min(q0.Z, q1.Z) <= max(p0.Z, p1.Z)
	return ret
}

// IsLineSegmentCross 跨立实验：通过叉积判断两线段是否真正相交。
// 算法原理：若 p0p1 跨立 q0q1（q0、q1 分别在 p0p1 两侧），
// 且 q0q1 跨立 p0p1（p0、p1 分别在 q0q1 两侧），则两线段相交。
// 叉积为 0 表示端点共线（恰好落在另一线段上），视为相交。
func IsLineSegmentCross(p0, p1, q0, q1 Coord) bool {
	// q0q1 相对于 q0 的叉积值：判断 p0、p1 是否在 q0q1 两侧
	b1 := cross(q1, p0, q0)
	b2 := cross(q1, p1, q0)

	// 叉积为 0 表示端点在另一线段所在直线上，视为相交
	if b1 == 0 || b2 == 0 {
		return true
	}

	// p0p1 相对于 p0 的叉积值：判断 q0、q1 是否在 p0p1 两侧
	a1 := cross(p1, q0, p0)
	a2 := cross(p1, q1, p0)

	if a1 == 0 || a2 == 0 {
		return true
	}

	// 两侧的叉积符号相反，说明两端点分居直线两侧（真正跨立）
	return ((b1 < 0) != (b2 < 0)) && ((a1 < 0) != (a2 < 0))
}

// DistanceToPoint 计算给定点到线段的最短距离，是 CalCoordDst 的别名方法。
func (s *Segment) DistanceToPoint(p Coord) float64 {
	return s.CalCoordDst(p)
}

// ClosestPoint 计算线段上距离给定点最近的点（投影点）。
// 算法：将向量 AP 投影到 AB 上，得到参数 t = (AP·AB) / |AB|²，
// t < 0 时最近点为端点 A，t > 1 时为端点 B，否则为线段内的投影点。
// 使用 int32 截断投影点坐标，精度损失在游戏场景中可接受。
func (s *Segment) ClosestPoint(p Coord) Coord {
	ab := NewVector(s.A, s.B)
	ap := NewVector(s.A, p)

	// 线段退化为点（A == B）时，最近点即为端点 A
	abLenSq := ab.LengthSquared()
	if abLenSq == 0 {
		return s.A
	}

	// 投影参数 t：AP 在 AB 方向上的归一化分量
	t := ap.Dot(&ab) / abLenSq

	// 将 t 限制在 [0, 1] 范围内，确保最近点落在线段上
	if t < 0 {
		return s.A
	}
	if t > 1 {
		return s.B
	}

	return Coord{
		X: s.A.X + int32(float64(ab.X)*t),
		Z: s.A.Z + int32(float64(ab.Z)*t),
	}
}

// GetCrossCoord 计算两线段 P0P1 与 Q0Q1 的交点坐标。
// 采用参数化方程法：先判断共线（叉积为 0 则无唯一交点），
// 再依次通过排斥实验和跨立实验确认相交，最后用参数 t 求交点精确坐标。
// 参考算法：https://stackoverflow.com/questions/563198
func GetCrossCoord(p0, p1, q0, q1 Coord) (Coord, bool) {
	v1 := NewVector(p0, p1)
	v2 := NewVector(q0, q1)
	// 两线段方向平行（叉积为 0）时无唯一交点
	if v1.Cross(&v2) == 0 {
		return Coord{}, false
	}

	if IsRectCross(p0, p1, q0, q1) {
		if IsLineSegmentCross(p0, p1, q0, q1) {
			// 参数化求交点：t = [(Q→P0) × S2] / (S1 × S2)，其中 S1=P1-P0, S2=Q1-Q0
			s1X := float64(p1.X - p0.X)
			s1Z := float64(p1.Z - p0.Z)
			s2X := float64(q1.X - q0.X)
			s2Z := float64(q1.Z - q0.Z)

			t := (s2X*float64(p0.Z-q0.Z) - s2Z*float64(p0.X-q0.X)) / (-s2X*s1Z + s1X*s2Z)

			return Coord{X: p0.X + int32(t*s1X), Z: p0.Z + int32(t*s1Z)}, true
		}
	}
	return Coord{}, false
}
