package geo

import "math"

// Vector 表示从坐标原点指向某点的位置向量（Position Vector）。
// 使用 int32 分量与 Coord 保持一致，避免混用浮点坐标带来的精度问题。
// 向量与坐标在内存布局上完全相同，可通过 NewVectorByCoord 零拷贝转换。
// 参考：https://zh.wikipedia.org/wiki/位置向量
type Vector struct {
	X, Z int32
}

// NewVector 创建从 start 指向 end 的位移向量。
func NewVector(start, end Coord) Vector {
	return Vector{
		X: end.X - start.X,
		Z: end.Z - start.Z,
	}
}

// NewVectorByCoord 将坐标直接转换为位置向量（原点→坐标点）。
// 利用 Vector 与 Coord 内存布局相同的特性进行类型转换，无额外开销。
func NewVectorByCoord(p Coord) Vector {
	return Vector(p)
}

// Add 计算两向量之和，返回新向量。
func (v *Vector) Add(vec *Vector) Vector {
	return Vector{
		X: v.X + vec.X,
		Z: v.Z + vec.Z,
	}
}

// Minus 计算向量 v 减去 vec 的差，返回新向量。
func (v *Vector) Minus(vec *Vector) Vector {
	return Vector{
		X: v.X - vec.X,
		Z: v.Z - vec.Z,
	}
}

// Dot 计算两向量的点积（内积）。
// 使用 int64 中间量防止两个 int32 分量相乘时溢出，结果以 float64 返回
// 以供后续除法等浮点运算使用（如投影参数 t 的计算）。
func (v *Vector) Dot(vec *Vector) float64 {
	result := int64(v.X) * int64(vec.X)
	result += int64(v.Z) * int64(vec.Z)
	return float64(result)
}

// Cross 计算两向量的二维叉积（Z 轴分量）。
// 结果 > 0：vec 在 v 的左侧（逆时针方向）；
// 结果 < 0：vec 在 v 的右侧（顺时针方向）；
// 结果 = 0：两向量共线。
// 使用 int64 防止溢出，适合大坐标场景下的方向判断。
func (v *Vector) Cross(vec *Vector) int64 {
	return int64(v.X)*int64(vec.Z) - int64(v.Z)*int64(vec.X)
}

// LengthSquared 计算向量长度的平方。
// 用于仅需比较向量大小的场景，避免 math.Sqrt 的性能开销。
// 同样使用 int64 防止 int32 分量平方溢出。
func (v *Vector) LengthSquared() float64 {
	return float64(int64(v.X)*int64(v.X) + int64(v.Z)*int64(v.Z))
}

// Length 计算向量的欧几里得长度（模）。
func (v *Vector) Length() float64 {
	return math.Sqrt(v.LengthSquared())
}

// Trunc 按比例缩放向量，返回新向量。
// ratio > 1 放大，ratio < 1 缩小，ratio < 0 反向。
// 使用 int32 截断会引入舍入误差，精度要求高时应注意累积误差。
func (v *Vector) Trunc(ratio float64) Vector {
	return Vector{
		X: int32(ratio * float64(v.X)),
		Z: int32(ratio * float64(v.Z)),
	}
}

// TruncEdge 将从 start 到 end 的向量截断为长度为 1000 的单位向量，并返回终点坐标。
// 固定长度 1000 是为了在整数坐标系中近似表示单位方向，避免浮点小数截断为 0。
func TruncEdge(start, end Coord) Coord {
	vec0 := NewVector(start, end)
	// 将向量长度截断为 1000（整数域下的"单位向量"近似）
	vec0 = vec0.Trunc(1000 / vec0.Length())

	return vec0.ToCoord(start)
}

// ToCoord 将向量作为位移加到起点坐标，返回终点坐标。
func (v Vector) ToCoord(start Coord) Coord {
	return Coord{
		X: start.X + v.X,
		Z: start.Z + v.Z,
	}
}

// Rotate 将向量按指定弧度旋转，返回旋转后的新向量。
// 左手坐标系下，正角度为顺时针旋转（X-Z 平面上 Z 轴向上时）。
// 旋转矩阵：
//
//	x' = x·cosθ - z·sinθ
//	z' = x·sinθ + z·cosθ
//
// 结果分量以 int32 截断，存在浮点→整数的舍入误差，大角度连续旋转时误差会累积。
// 参考：https://blog.csdn.net/u013445530/article/details/44904017
func (v *Vector) Rotate(angle float64) Vector {
	x0 := float64(v.X)
	z0 := float64(v.Z)

	cos := math.Cos(angle)
	sin := math.Sin(angle)

	x := x0*cos - z0*sin
	z := x0*sin + z0*cos

	return Vector{
		X: int32(x),
		Z: int32(z),
	}
}

// CalCoordDst 计算目标点到本向量所在直线的垂直距离。
// 先构造从 start 到 target 的向量，再利用 sin(夹角) * 向量长度 得到垂直分量。
// 适用于判断点是否偏离路径方向的场景。
func (v *Vector) CalCoordDst(start, target Coord) float64 {
	vec := NewVector(start, target)

	angle := v.GetAngle(&vec)
	return vec.Length() * math.Sin(angle)
}

// GetAngle 计算两向量之间的夹角（单位：弧度，范围 [0, π]）。
// 使用点积公式：cosθ = (v·vec) / (|v|·|vec|)，再取反余弦得角度。
// 当任意一个向量为零向量时（模为 0）无法定义夹角，返回 0 作为安全默认值。
// 当 cosθ 因浮点误差略超出 [-1, 1] 时，math.Acos 返回 NaN，
// 此时根据符号修正为 0 或 π，保证结果合法。
func (v *Vector) GetAngle(vec *Vector) float64 {
	a := v.Dot(vec)
	b := v.Length() * vec.Length()
	if b == 0 {
		// 零向量无法定义夹角，返回 0
		return 0
	}
	t := a / b
	angle := math.Acos(t)
	if math.IsNaN(angle) {
		// 浮点误差导致 t 略超出 [-1, 1]，根据符号修正为边界值
		if t > 0 {
			return 0
		}
		return math.Pi
	}

	return angle
}

// cross 计算以 p3 为基点的向量 p3→p1 与 p3→p2 的叉积。
// 公式：(p1-p3) × (p2-p3)，正值表示 p1 在 p3→p2 的左侧。
// 使用 int64 防止坐标差值乘法溢出。
func cross(p1, p2, p3 Coord) int64 {
	s := int64(p1.X-p3.X)*int64(p2.Z-p3.Z) - int64(p2.X-p3.X)*int64(p1.Z-p3.Z)
	return s
}

// CalCoordByRatio 沿 startCoord→endCoord 方向，按比例 ratio 计算插值坐标。
// ratio = 0 返回 startCoord，ratio = 1 返回 endCoord，
// ratio > 1 则超出 endCoord 继续延伸，适合路径插值和外推计算。
func CalCoordByRatio(startCoord, endCoord Coord, ratio float64) Coord {
	vec := NewVector(startCoord, endCoord)
	return vec.Trunc(ratio).ToCoord(startCoord)
}
