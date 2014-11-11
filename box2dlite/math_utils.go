package box2dlite

import (
	"math"
)

const Pi float64 = 3.14159265358979323846264

// Vec2

type Vec2 struct {
	X, Y float64
}

func (v *Vec2) Negative() Vec2 {
	return Vec2{
		-v.X,
		-v.Y,
	}
}

func (v *Vec2) Add(o Vec2) Vec2 {
	return Vec2{
		v.X + o.X,
		v.Y + o.Y,
	}
}

func (v *Vec2) Sub(o Vec2) Vec2 {
	return Vec2{
		v.X - o.X,
		v.Y - o.Y,
	}
}

func (v *Vec2) Mul(a float64) Vec2 {
	return Vec2{
		v.X * a,
		v.Y * a,
	}
}

func (v *Vec2) Abs() Vec2 {
	return Vec2{
		math.Abs(v.X),
		math.Abs(v.Y),
	}
}

func (v *Vec2) Dot(o Vec2) float64 {
	return v.X*o.X + v.Y*o.Y
}

func (v *Vec2) Length() float64 {
	return math.Hypot(v.X, v.Y)
}

// Mat22

type Mat22 struct {
	Col1, Col2 Vec2
}

func Mat22ByAngle(angle float64) Mat22 {
	var c float64 = math.Cos(angle)
	var s float64 = math.Sin(angle)
	return Mat22{
		Vec2{c, s},
		Vec2{-s, c},
	}
}

func (m *Mat22) Transpose() Mat22 {
	return Mat22{
		Vec2{m.Col1.X, m.Col2.X},
		Vec2{m.Col1.Y, m.Col2.Y},
	}
}

func (m *Mat22) Invert() Mat22 {
	var det float64 = m.Col1.X*m.Col2.Y - m.Col1.Y*m.Col2.X
	if det == 0.0 {
		panic("Mat22Invert fails because det == 0.0")
	}
	det = 1.0 / det

	return Mat22{
		Vec2{det * m.Col2.Y, -det * m.Col2.X},
		Vec2{-det * m.Col1.Y, det * m.Col1.X},
	}
}

func (m *Mat22) Add(o Mat22) Mat22 {
	return Mat22{
		m.Col1.Add(o.Col1),
		m.Col2.Add(o.Col2),
	}
}

func (m *Mat22) MulV(v Vec2) Vec2 {
	return Vec2{
		m.Col1.X*v.X + m.Col2.X*v.Y,
		m.Col1.Y*v.X + m.Col2.Y*v.Y,
	}
}

func (m *Mat22) MulM(o Mat22) Mat22 {
	return Mat22{
		m.MulV(o.Col1), m.MulV(o.Col2),
	}
}

func (m *Mat22) Abs() Mat22 {
	return Mat22{
		m.Col1.Abs(), m.Col2.Abs(),
	}
}

// functions

func Dot(a, b Vec2) float64 {
	return a.X*b.X + a.Y*b.Y
}

func CrossVV(a, b Vec2) float64 {
	return a.X*b.Y - a.Y*b.X
}

func CrossVS(a Vec2, s float64) Vec2 {
	return Vec2{s * a.Y, -s * a.X}
}

func CrossSV(s float64, a Vec2) Vec2 {
	return Vec2{-s * a.Y, s * a.X}
}

func MulSV(s float64, v Vec2) Vec2 {
	return Vec2{s * v.X, s * v.Y}
}

func Clamp(a, l, h float64) float64 {
	return math.Max(l, math.Min(a, h))
}
