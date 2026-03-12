package geo

import "math/rand/v2"

// Rectangle 表示轴对齐矩形（AABB），由左下角坐标及宽高描述。
// 嵌入 Coord 作为左下角锚点，Width 和 Height 分别为沿 X 轴和 Z 轴的长度。
// 整数坐标设计避免浮点误差，适合游戏服务端的快速区域判断和空间索引。
type Rectangle struct {
	Coord        // 左下角坐标（锚点）
	Width  int32 // 沿 X 轴方向的宽度
	Height int32 // 沿 Z 轴方向的高度
}

// NewRectangle 以左下角坐标及宽高创建矩形对象。
func NewRectangle(x, z, width, height int32) Rectangle {
	return Rectangle{
		Coord: Coord{
			X: x,
			Z: z,
		},
		Width:  width,
		Height: height,
	}
}

// RandCoord 在矩形区域内随机生成一个坐标点。
// 使用 math/rand/v2 的 Int32N 生成 [0, Width) 和 [0, Height) 范围内的随机偏移，
// 再加上左下角锚点坐标，确保随机点始终落在矩形内部。
func (rec *Rectangle) RandCoord() Coord {
	return Coord{
		X: rec.X + rand.Int32N(rec.Width),
		Z: rec.Z + rand.Int32N(rec.Height),
	}
}

// GetVerticeCoords 返回矩形按逆时针排列的四个顶点坐标（左下→右下→右上→左上）。
// 逆时针排列是叉积法点包含判断和 SAT 碰撞检测的约定前提，
// 保持统一方向可避免在各算法调用前手动检查排列方向。
func (rec *Rectangle) GetVerticeCoords() [4]Coord {
	var p [4]Coord
	p[0] = Coord{X: rec.Coord.X, Z: rec.Coord.Z}                          // 左下
	p[1] = Coord{X: rec.Coord.X + rec.Width, Z: rec.Coord.Z}              // 右下
	p[2] = Coord{X: rec.Coord.X + rec.Width, Z: rec.Coord.Z + rec.Height} // 右上
	p[3] = Coord{X: rec.Coord.X, Z: rec.Coord.Z + rec.Height}             // 左上
	return p
}

// GetVectors 返回矩形按逆时针排列的四个顶点位置向量。
// 位置向量以坐标原点为起点，与坐标等价，便于向量运算。
func (rec *Rectangle) GetVectors() [4]Vector {
	coords := rec.GetVerticeCoords()
	return [4]Vector{
		NewVectorByCoord(coords[0]),
		NewVectorByCoord(coords[1]),
		NewVectorByCoord(coords[2]),
		NewVectorByCoord(coords[3]),
	}
}

// GetLocationToBorder 获取矩形与给定边界的象限重叠关系，委托给 Border.RectLocation 实现。
func (rec *Rectangle) GetLocationToBorder(b *Border) LocationState {
	minX := rec.X
	maxX := rec.X + rec.Width
	minZ := rec.Z
	maxZ := rec.Z + rec.Height
	return b.RectLocation(minX, minZ, maxX, maxZ)
}

// IsCoordInside 通过叉积法判断点是否在矩形内部（含边界）。
// 原理：矩形顶点按逆时针排列，内部点与每条边做叉积均 ≥ 0（同侧）；
// 叉积为 0 表示点在边上，视为内部；分批计算减少不必要的向量构造，
// 一旦发现符号不一致（b1 ≠ b2）立即返回 false，利用短路求值提升效率。
func (rec *Rectangle) IsCoordInside(p Coord) bool {
	pts := rec.GetVerticeCoords()

	// 构造从点 p 指向各顶点的向量，逐对进行叉积比较
	pa := NewVector(p, pts[0])
	pb := NewVector(p, pts[1])
	pc := NewVector(p, pts[2])

	b1 := pa.Cross(&pb) >= 0
	b2 := pb.Cross(&pc) >= 0
	if b1 != b2 {
		return false
	}

	pd := NewVector(p, pts[3])
	b3 := pc.Cross(&pd) >= 0
	if b2 != b3 {
		return false
	}

	b4 := pd.Cross(&pa) >= 0
	return b3 == b4
}
