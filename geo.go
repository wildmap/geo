package geo

import (
	"math"

	"github.com/wildmap/utility"
)

// GetCoordsAround 计算从圆周起点出发、绕圆心到达外部目标点切线方向的弧线路径点集。
// 通过计算起点向量与目标点向量的夹角，并减去目标点到圆的切线角度，
// 得到需要旋转的有效弧度，再按叉积符号确定旋转方向（顺时针或逆时针）。
// 常用于游戏中绕障碍物圆形边界的路径规划。
func GetCoordsAround(startCoord, endCoord, centerCoord Coord) []Coord {
	centerVector := NewVector(centerCoord, startCoord)
	endVector := NewVector(centerCoord, endCoord)

	// 计算圆心→起点与圆心→终点的夹角
	angle := centerVector.GetAngle(&endVector)
	radius := CalDstCoordToCoord(startCoord, centerCoord)
	// 减去终点到圆的切线角度，得到实际需要走的圆弧弧度
	cutAngle := GetCutOffCoordAngle(endCoord, centerCoord, radius)
	angle -= cutAngle
	// 叉积 > 0 表示 endVector 在 centerVector 左侧，需逆时针旋转（取负角）
	if centerVector.Cross(&endVector) > 0 {
		angle = -angle
	}
	coords := GetArcCoords(startCoord, centerCoord, angle)

	return coords
}

// GetCoordsAround2 在圆周上以均匀角度间隔采样 n 个点，包含起点。
// 覆盖完整的 2π 弧度（整圈），适合生成圆形 AOE 范围的边界点集。
func GetCoordsAround2(circleCoord, centerCoord Coord, n int) []Coord {
	return getCoordsAround(circleCoord, centerCoord, n, 2*math.Pi)
}

// getCoordsAround 在圆周上从起点出发、按指定弧度均匀采样 n 个点（含起点）。
// 以起点向量为基准，将总弧度等分为 n-1 份，依次旋转得到各采样点。
// 采用增量旋转而非从零累积，减少累积误差。
func getCoordsAround(startCoord, centerCoord Coord, n int, angle float64) []Coord {
	if n < 2 {
		return nil
	}

	ret := make([]Coord, n)
	ret[0] = startCoord

	vec := NewVector(centerCoord, startCoord)
	// 将总弧度均分为 n-1 段
	ave := angle / float64(n-1)
	angle = 0
	for i := 1; i < n; i++ {
		angle += ave
		v := vec.Rotate(angle)
		ret[i] = v.ToCoord(centerCoord)
	}

	return ret
}

// GetArcCoords 生成从圆周起点出发、旋转指定弧度的圆弧采样点集。
// angle > 0 为顺时针方向，angle < 0 为逆时针方向（左手坐标系约定）。
// 采样密度约为每弧度 10 个点（π 对应约 34 个点），最少 2 个点。
func GetArcCoords(startCoord, centerCoord Coord, angle float64) []Coord {
	n := max(int32(utility.Abs(angle)*10), 2)
	// GetCoordsAround 内部使用 -angle，此处传入原始角度取负以修正方向
	return getCoordsAround(startCoord, centerCoord, int(n), -angle)
}

// GetSpiralCoords 生成以中心点为基准的阿基米德螺旋线路径点集。
// 起点到终点之间同时发生旋转（angle 弧度）和半径增量（delta 单位），
// 两者均匀分配到各采样点，实现平滑的螺旋扩散效果。
// angle > 0 为顺时针，angle < 0 为逆时针；delta > 0 向外扩散，delta < 0 向内收缩。
func GetSpiralCoords(startCoord, centerCoord Coord, angle, delta float64) []Coord {
	n := int32(utility.Abs(angle) * 10)
	if n < 2 {
		// 旋转弧度极小时退化为直线延伸，沿起点→圆心方向缩放 delta
		dst := CalDstCoordToCoord(startCoord, centerCoord)
		if dst < 1e-9 {
			return []Coord{startCoord}
		}
		vec := NewVector(centerCoord, startCoord)
		vec = vec.Trunc((delta + dst) / dst)
		return []Coord{startCoord, vec.ToCoord(centerCoord)}
	}

	ret := make([]Coord, n)
	ret[0] = startCoord

	aveAngle := angle / float64(n)
	aveDelta := (delta) / float64(n)
	angle = 0
	delta = 0

	vec := NewVector(centerCoord, startCoord)
	radius := vec.Length()
	if radius < 1e-9 {
		// 起点与圆心重合时无法确定方向，直接返回单点
		return []Coord{startCoord}
	}
	for i := int32(1); i < n; i++ {
		// 在基础向量上叠加旋转角度
		angle += aveAngle
		v := vec.Rotate(angle)
		// 在旋转后的方向上叠加半径增量，实现螺旋扩展
		delta += aveDelta
		v = v.Trunc((radius + delta) / radius)

		ret[i] = v.ToCoord(centerCoord)
	}

	return ret
}

// GetIntersectCoord 计算从圆外一点到圆心连线方向上的圆周交点。
// 是 Circle.GetIntersectCoord 的包装函数，提供更直观的函数式调用接口。
func GetIntersectCoord(centerP, endP Coord, radius int32) Coord {
	c := NewCirCle(centerP, radius)
	return c.GetIntersectCoord(endP)
}

// GetLineCrossCircle 计算线段与圆的第一个交点。
// 是 Circle.GetLineCross 的包装函数，适合不需要构造 Circle 对象的场景。
func GetLineCrossCircle(startP, endP, centerP Coord, radius int32) (Coord, bool) {
	c := NewCirCle(centerP, radius)
	seg := NewSegment(startP, endP)
	return c.GetLineCross(&seg)
}

// GetCutOffCoordAngle 计算从圆外一点到圆的切线角度（∠ECenterP 中切点所在角）。
// 几何原理：设终点为 E，圆心为 C，切点为 P，则 EP ⊥ CP，
// 因此 cos(∠ECP) = CP/CE = radius/dst，即 ∠ECP = arccos(radius/dst)。
// 当点在圆内（dst < radius）时 arccos 结果为 NaN，返回 0 表示无切线。
func GetCutOffCoordAngle(endCoord, centerCoord Coord, radius float64) float64 {
	dst := CalDstCoordToCoord(endCoord, centerCoord)
	a := math.Acos(radius / dst)

	if math.IsNaN(a) {
		return 0.
	}

	return a
}

// GetBresenhamCoord 使用 Bresenham 直线算法生成两点间的整数栅格坐标序列。
// Bresenham 算法完全基于整数运算，无需浮点除法，
// 适合在游戏地图格子上绘制视线、判断障碍物遮挡等高频操作。
// 参考：https://zh.wikipedia.org/wiki/布雷森漢姆直線演算法
func GetBresenhamCoord(p1, p2 Coord) []Coord {
	xstart := p1.X
	zstart := p1.Z
	xend := p2.X
	zend := p2.Z

	var steep = false
	var swapped = false

	// 斜率绝对值 > 1 时，通过交换 X 和 Z 轴将问题转化为斜率 ≤ 1 的情形
	if utility.Abs(zend-zstart) > utility.Abs(xend-xstart) {
		steep = true
	}

	if steep {
		xstart, zstart = zstart, xstart
		xend, zend = zend, xend
	}
	// 确保沿 X 轴正方向迭代，若发生交换需最后反转结果序列
	if xstart > xend {
		xstart, xend = xend, xstart
		zstart, zend = zend, zstart
		swapped = true
	}
	var deltax = xend - xstart
	var deltaz = utility.Abs(zend - zstart)
	// err 初始化为 deltax/2，平衡误差使首个点尽量居中
	var err = deltax / 2
	var zstep int32
	z := zstart
	if zstart < zend {
		zstep = 1
	} else {
		zstep = -1
	}
	tmpCoordList := make([]Coord, 0, xend-xstart+1)
	for x := xstart; x <= xend; x++ {
		var c Coord
		if steep {
			// 还原坐标轴交换，将 (x, z) 映射回 (Z, X)
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

	// 如果发生了起点/终点交换，反转坐标序列以恢复原始方向
	if swapped {
		reverse(tmpCoordList)
	}
	return tmpCoordList
}

// reverse 将坐标切片原地反转。
func reverse(slice []Coord) {
	for i, j := 0, len(slice)-1; j > i; i, j = i+1, j-1 {
		slice[i], slice[j] = slice[j], slice[i]
	}
}

// CalMidCoord 计算两个坐标点的中点。
// 使用差值的一半而非两坐标之和除以二，可避免在两坐标均为较大正数时
// 中间求和值溢出 int32 的问题。
func CalMidCoord(p1, p2 Coord) Coord {
	return Coord{
		X: p1.X + (p2.X-p1.X)/2,
		Z: p1.Z + (p2.Z-p1.Z)/2,
	}
}

// GetCrossRect 计算线段经过的所有子矩形格子集合。
// 将整个空间划分为 rWidth×rHeight 的均匀网格，通过分别求线段与
// 所有水平网格线和垂直网格线的交点，确定线段经过的格子集合。
// 适用于技能路径判断、地图障碍检测等需要快速定位经过区域的场景。
// 注意：若交点恰好落在格子十字路口，相邻四个格子均会被收录（保守策略）。
func GetCrossRect(p0, p1 Coord, rWidth, rHeight, width, height int32) map[Coord]bool {
	coordSet := map[Coord]bool{}
	// 将线段起点和终点所在的格子直接加入集合（端点必定经过其所在格子）
	startCoords := []Coord{p0, p1}
	for i := range startCoords {
		c := Coord{X: (startCoords[i].X / rWidth) * rWidth, Z: (startCoords[i].Z / rHeight) * rHeight}
		_, exist := coordSet[c]
		if !exist {
			coordSet[c] = true
		}
	}
	// 按 Z 坐标从小到大遍历所有水平分割线，求线段与各分割线的交点
	zStart := p0
	zEnd := p1
	if p1.Z < p0.Z {
		zStart = p1
		zEnd = p0
	}
	startZ := zStart.Z / rHeight
	endZ := zEnd.Z / rHeight
	for i := startZ; i <= endZ; i++ {
		q0 := &Coord{X: 0, Z: i * rHeight}
		q1 := &Coord{X: width, Z: i * rHeight}
		coord, ok := GetCrossCoord(p0, p1, *q0, *q1)
		if ok {
			// 修正误差：将交点 Z 坐标对齐到分割线
			coord.Z = i * rHeight
			// 交点上方和下方的格子均为线段经过的格子
			upCoord := Coord{X: (coord.X / rWidth) * rWidth, Z: (coord.Z / rHeight) * rHeight}
			downCoord := Coord{X: upCoord.X, Z: (coord.Z/rHeight - 1) * rHeight}
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

	// 按 X 坐标从小到大遍历所有垂直分割线，求线段与各分割线的交点
	xStart := p0
	xEnd := p1
	if p1.X < p0.X {
		xStart = p1
		xEnd = p0
	}
	startX := xStart.X / rWidth
	endX := xEnd.X / rWidth
	for i := startX; i <= endX; i++ {
		q0 := &Coord{X: i * rWidth, Z: 0}
		q1 := &Coord{X: i * rWidth, Z: height}
		coord, ok := GetCrossCoord(p0, p1, *q0, *q1)
		if ok {
			coord.X = i * rWidth
			// 交点左侧和右侧的格子均为线段经过的格子
			rightCoord := Coord{X: (coord.X / rWidth) * rWidth, Z: (coord.Z / rHeight) * rHeight}
			leftCoord := Coord{X: (coord.X/rWidth - 1) * rWidth, Z: rightCoord.Z}
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

// GetIntersectRect 计算两个轴对齐矩形的相交区域。
// 相交矩形的左下角取两矩形左下角坐标的较大值，右上角取较小值，
// 若宽或高为非正值则两矩形不相交，返回 false。
func GetIntersectRect(r0 Rectangle, r1 Rectangle) (Rectangle, bool) {
	rect := Rectangle{}
	rect.X = max(r0.X, r1.X)
	rect.Z = max(r0.Z, r1.Z)
	rect.Width = min(r0.X+r0.Width, r1.X+r1.Width) - rect.X
	rect.Height = min(r0.Z+r0.Height, r1.Z+r1.Height) - rect.Z

	if rect.Width <= 0 || rect.Height <= 0 {
		return rect, false
	}
	return rect, true
}
