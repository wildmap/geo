package geo

// Polygon 定义多边形的统一抽象接口。
// 通过接口抽象，Triangle 和 Convex 等具体多边形类型可以统一参与
// 导航网格构建、空间查询及碰撞检测等算法，无需关心底层几何形状。
type Polygon interface {
	// IsCoordInside 判断给定坐标点是否位于多边形内部（含边界）
	IsCoordInside(p Coord) bool
	// GetVectors 返回按逆时针排列的顶点位置向量列表
	GetVectors() []Vector
	// ToRect 返回多边形的轴对齐包围盒（AABB）的四个极值坐标
	ToRect() (minX, minZ, maxX, maxZ int32)
	// GetIndex 返回多边形的全局唯一序号
	GetIndex() int32
	// GetEdgeIDs 返回多边形各边的唯一序号列表
	GetEdgeIDs() []int32
	// GetEdgeMidCoords 返回多边形各边的中点坐标列表
	GetEdgeMidCoords() []Coord
	// GetVertices 返回多边形的顶点列表
	GetVertices() []Vertice
}

// CrossProduct 计算以 p1→p2→p3 顺序连接的三个顶点处的有向叉积符号。
// 返回 1 表示逆时针转弯（左转），-1 表示顺时针转弯（右转），0 表示共线。
// 使用 int64 中间量防止 int32 坐标差值相乘时溢出，保证大坐标场景下的正确性。
func CrossProduct(p1, p2, p3 Vertice) int32 {
	ax := int64(p2.Coord.X - p1.Coord.X)
	az := int64(p2.Coord.Z - p1.Coord.Z)
	bx := int64(p3.Coord.X - p2.Coord.X)
	bz := int64(p3.Coord.Z - p2.Coord.Z)
	cp := ax*bz - az*bx
	if cp > 0 {
		return 1
	} else if cp < 0 {
		return -1
	}

	return 0
}

// IsConvex 验证给定顶点列表是否构成合法的凸多边形。
// 凸多边形的判断依据：遍历所有相邻边对，若所有叉积符号一致
// （全为正或全为负），则为凸多边形；若正负混合，则含凹角，不是凸多边形。
// 注意：使用 positiveFlag != negativeFlag 而非直接统计数量，
// 可以在发现两种符号都出现时立即识别为非凸（两个 bool 的异或），代码简洁高效。
func IsConvex(vertices []Vertice) bool {
	numPoints := len(vertices)
	if numPoints < 3 {
		return false
	}
	negativeFlag := false
	positiveFlag := false
	for i := range numPoints {
		curvec := NewVector(vertices[i].Coord, vertices[(i+1)%numPoints].Coord)
		vec2next := NewVector(vertices[i].Coord, vertices[(i+2)%numPoints].Coord)
		cp := curvec.Cross(&vec2next)
		if cp > 0 {
			positiveFlag = true
		} else if cp < 0 {
			negativeFlag = true
		}
	}
	// 只有一种符号出现时才是凸多边形
	return positiveFlag != negativeFlag
}
