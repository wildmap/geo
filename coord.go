package geo

import "math"

// Coord 表示 X-Z 平面上的二维整数坐标点。
// 使用 int32 而非 float64，从根本上规避浮点精度误差，
// 适合游戏服务端对大量坐标进行高频比较和运算的场景。
// JSON 标签便于坐标在网络协议中的序列化与反序列化。
type Coord struct {
	X int32 `json:"x"`
	Z int32 `json:"z"`
}

// NewCoord 以 X、Z 分量创建坐标点。
func NewCoord(x, z int32) Coord {
	return Coord{
		X: x,
		Z: z,
	}
}

// IsEqual 判断两个坐标点是否完全相等。
// 直接利用 Go 结构体的值比较语义，无需逐字段判断，简洁高效。
func (c Coord) IsEqual(target Coord) bool {
	return c == target
}

// GetLocationToBorder 获取该坐标点相对于边界的象限位置。
func (c *Coord) GetLocationToBorder(b *Border) LocationState {
	return b.CoordLocation(*c)
}

// CalDstCoordToCoord 计算两个坐标点之间的欧几里得距离。
// 先将差值提升为 int64 再平方，防止两个 int32 相减后乘法溢出，
// 保证在全坐标值范围内（约 ±2.1×10⁹）的计算正确性。
func CalDstCoordToCoord(coord1, coord2 Coord) float64 {
	dx := int64(coord1.X - coord2.X)
	dz := int64(coord1.Z - coord2.Z)

	squared := dx*dx + dz*dz
	return math.Sqrt(float64(squared))
}

// CalDstCoordToCoordWithoutSqrt 计算两个坐标点之间的距离平方（不开根号）。
// 用于只需比较距离大小的场景（如碰撞检测），避免 math.Sqrt 的性能开销，
// 是一种常见的性能优化手段：比较 d² 与 r² 等价于比较 d 与 r。
func CalDstCoordToCoordWithoutSqrt(coord1, coord2 Coord) float64 {
	dx := int64(coord1.X - coord2.X)
	dz := int64(coord1.Z - coord2.Z)

	squared := dx*dx + dz*dz
	return float64(squared)
}
