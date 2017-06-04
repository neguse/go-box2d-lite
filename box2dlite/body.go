package box2dlite

import (
	"math"
)

type Body struct {
	Position Vec2
	Rotation float64

	Velocity        Vec2
	AngularVelocity float64

	Force  Vec2
	Torque float64

	Width Vec2

	Friction float64
	Mass     float64
	invMass  float64
	I        float64
	invI     float64
}

func (b *Body) Set(w *Vec2, m float64) {
	b.Position = Vec2{0.0, 0.0}
	b.Rotation = 0.0
	b.Velocity = Vec2{0.0, 0.0}
	b.AngularVelocity = 0.0
	b.Force = Vec2{0.0, 0.0}
	b.Torque = 0.0
	b.Friction = 0.2

	b.Width = *w
	b.Mass = m

	if m < math.MaxFloat64 {
		b.invMass = 1.0 / m
		b.I = m * (w.X*w.X + w.Y*w.Y) / 12.0
		b.invI = 1.0 / b.I
	} else {
		b.invMass = 0.0
		b.I = math.MaxFloat64
		b.invI = 0.0
	}
}
