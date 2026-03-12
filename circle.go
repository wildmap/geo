package geo

import (
	"math"

	"github.com/wildmap/utility"
)

// Circle 表示二维平面上的圆形，由圆心坐标和半径定义。
// 整数坐标与整数半径的设计避免了浮点误差，适合游戏服务端的碰撞检测场景。
type Circle struct {
	Center Coord
	Radius int32
}

// NewCirCle 以圆心和半径创建圆形对象。
func NewCirCle(center Coord, radius int32) Circle {
	return Circle{
		Center: center,
		Radius: radius,
	}
}

// GetLocationToBorder 获取圆形外接矩形与边界的象限重叠关系。
// 将圆退化为 AABB（轴对齐包围盒）后复用 RectLocation，
// 以整数运算快速完成四叉树空间分类，避免精确的圆-矩形相交计算。
func (c *Circle) GetLocationToBorder(b *Border) LocationState {
	minX, minZ, maxX, maxZ := c.ToRect()
	return b.RectLocation(minX, minZ, maxX, maxZ)
}

// ToRect 将圆形退化为其轴对齐外接矩形（AABB）。
// 外接矩形以圆心为中心、边长为 2*Radius，用于空间索引的快速粗筛。
func (c *Circle) ToRect() (minX, minZ, maxX, maxZ int32) {
	minX = c.Center.X - c.Radius
	minZ = c.Center.Z - c.Radius
	maxX = c.Center.X + c.Radius
	maxZ = c.Center.Z + c.Radius
	return
}

// GetIntersectCoord 计算从圆心到外部点连线方向上圆周的交点。
// 本质是对圆心→目标点向量按半径/向量长度的比例进行截断，
// 得到圆周上距离目标点最近的点，常用于将单位"推回"圆边界。
func (c *Circle) GetIntersectCoord(p Coord) Coord {
	// 目标点与圆心重合时，方向向量无意义，直接返回圆心
	if c.Center == p {
		return c.Center
	}
	vec := NewVector(c.Center, p)
	length := vec.Length()
	ratio := float64(c.Radius) / length
	vec = vec.Trunc(ratio)
	return vec.ToCoord(c.Center)
}

// IsIntersect 判断线段是否与圆相交。
func (c *Circle) IsIntersect(s *Segment) bool {
	_, ok := c.GetLineCross(s)
	return ok
}

// GetLineCross 计算线段与圆的交点，返回距离线段起点最近的第一个交点。
// 算法采用参数化线段方程与圆方程联立求解：
//  1. 将线段方向单位化，以参数 t 描述线段上的点；
//  2. 将圆心到线段起点的向量投影到线段方向，得到最近垂足参数 a；
//  3. 利用勾股定理判断判别式（r² - e² + a²）是否大于等于 0；
//  4. 限制 t 在 [0, fDis] 范围内，确保交点落在线段上而非延长线。
func (c *Circle) GetLineCross(s *Segment) (Coord, bool) {
	var coord1 *Coord
	var coord2 *Coord
	fDis := CalDstCoordToCoord(s.A, s.B)

	// 线段退化为点（两端点重合）时 fDis≈0，后续除法会产生 NaN/Inf。
	// 退化情况下改为直接判断该点是否在圆内（到圆心距离 ≤ 半径）。
	if fDis < 1e-9 {
		if CalDstCoordToCoord(s.A, c.Center) <= float64(c.Radius) {
			return s.A, true
		}
		return Coord{}, false
	}

	// 计算线段方向的单位向量分量，避免 math.Sqrt 带来的性能损耗
	dx := float64(s.B.X-s.A.X) / fDis
	dz := float64(s.B.Z-s.A.Z) / fDis

	// 圆心相对线段起点的偏移向量
	ex := float64(c.Center.X - s.A.X)
	ez := float64(c.Center.Z - s.A.Z)

	// a 为圆心在线段方向上的投影长度（最近垂足参数）
	a := ex*dx + ez*dz
	a2 := a * a
	// e2 为圆心到线段起点的距离平方
	e2 := ex*ex + ez*ez

	r2 := float64(c.Radius) * float64(c.Radius)
	// 判别式 < 0 表示圆心到直线的距离大于半径，无交点
	if utility.Smaller(r2-e2+a2, 0) {
		return Coord{}, false
	}
	// f 为交点到垂足的距离（半弦长）
	f := math.Sqrt(r2 - e2 + a2)

	// t1 = a - f 对应距起点较近的交点，限制在线段范围内
	t := a - f
	if t > -utility.Epsilon && (t-fDis) < utility.Epsilon {
		coord1 = &Coord{
			X: s.A.X + int32(t*dx),
			Z: s.A.Z + int32(t*dz),
		}
	}
	// t2 = a + f 对应距起点较远的交点
	t = a + f
	if t > -utility.Epsilon && (t-fDis) < utility.Epsilon {
		coord2 = &Coord{
			X: s.A.X + int32(t*dx),
			Z: s.A.Z + int32(t*dz),
		}
	}

	// 优先返回 coord1（较近交点）；若 coord1 不在线段上，尝试返回 coord2
	if coord1 == nil {
		coord1 = coord2
	}
	if coord1 == nil {
		return Coord{}, false
	}
	return *coord1, true
}

// IsInterPolygon 判断圆与凸多边形是否发生碰撞（相交或包含均视为碰撞）。
// 涵盖以下三种情形：圆在多边形内部、多边形在圆内部、两者部分重叠。
// 完全分离时返回 false。
//
// 算法基于分离轴定理（SAT）的变体，遍历多边形每条边，
// 通过顶点到圆心的距离和边上的投影两步骤判定碰撞：
//  1. 若顶点到圆心距离 ≤ 半径，顶点在圆内，直接碰撞；
//  2. 将圆心向量投影到边上，若投影点到圆心距离 ≤ 半径，边与圆相交；
//  3. 所有边均未相交且方向一致，则判断圆心是否在多边形内部。
//
// 多边形向量数组须按逆时针排列。
// 参考实现：https://bitlush.com/blog/circle-vs-polygon-collision-detection-in-c-sharp
func (c *Circle) IsInterPolygon(vectors []Vector) bool {
	// 少于 3 个顶点无法构成多边形，直接返回 false，防止切片越界
	if len(vectors) < 3 {
		return false
	}

	radiusSquared := float64(c.Radius) * float64(c.Radius)
	// 从最后一个顶点开始，与第一个顶点构成第一条边
	vertex := vectors[len(vectors)-1]
	center := NewVectorByCoord(c.Center)

	nearestDistance := math.MaxFloat64
	nearestIsInside := false
	nearestVertex := -1
	lastIsInside := false

	for i := range vectors {
		nextVertex := vectors[i]
		// axis 为当前顶点到圆心的向量
		axis := center.Minus(&vertex)
		// distance 为顶点到圆心距离平方减去半径平方（负值表示顶点在圆内）
		distance := axis.LengthSquared() - radiusSquared

		// 顶点在圆内，直接判定碰撞
		if utility.SmallerOrEqual(distance, 0) {
			return true
		}

		isInside := false
		// 当前边向量
		edge := nextVertex.Minus(&vertex)
		edgeLengthSquared := edge.LengthSquared()
		if !utility.Equal(edgeLengthSquared, 0) {
			// 计算圆心在边上的投影参数（dot / edgeLengthSquared ∈ [0,1] 表示投影点在边内）
			dot := edge.Dot(&axis)
			if utility.GreaterOrEqual(dot, 0) && utility.SmallerOrEqual(dot, edgeLengthSquared) {
				// 投影点坐标（相对原点）
				projection := edge.Trunc(dot / edgeLengthSquared)
				projection = vertex.Add(&projection)

				// 投影点到圆心的向量
				axis = projection.Minus(&center)
				// 投影点在圆内，边与圆相交，直接碰撞
				if utility.SmallerOrEqual(axis.LengthSquared(), radiusSquared) {
					return true
				}

				// 圆心在边的外侧（法向量方向判断），表示圆在多边形外部
				if !isInsideEdge(&edge, &axis) {
					return false
				}

				isInside = true
			}
		}

		// 记录距离圆心最近的顶点信息，用于最终的圆心在多边形内部判断
		if utility.Smaller(distance, nearestDistance) {
			nearestDistance = distance
			nearestIsInside = isInside || lastIsInside
			nearestVertex = i
		}

		vertex = nextVertex
		lastIsInside = isInside
	}

	// 处理首尾衔接：nearestVertex==0 时，还需考虑最后一条边的 lastIsInside
	if nearestVertex == 0 {
		return nearestIsInside || lastIsInside
	}
	return nearestIsInside
}

// isInsideEdge 判断圆心（由 axis 描述）是否位于边的内侧（多边形内部方向）。
// 在逆时针排列的多边形中，边的"内侧"由边向量左旋 90° 的法向量方向确定。
// 具体规则通过 edge 与 axis 各分量的符号组合来快速判断，避免浮点叉积运算。
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
