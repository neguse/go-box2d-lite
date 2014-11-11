package box2dlite

type Joint struct {
	M                          Mat22
	LocalAnchor1, LocalAnchor2 Vec2
	R1, R2                     Vec2
	Bias                       Vec2
	P                          Vec2
	Body1, Body2               *Body
	BiasFactor                 float64
	Softness                   float64
}

func (j *Joint) Set(b1 *Body, b2 *Body, anchor *Vec2) {
	j.Body1 = b1
	j.Body2 = b2

	rot1 := Mat22ByAngle(b1.Rotation)
	rot2 := Mat22ByAngle(b2.Rotation)
	rot1T := rot1.Transpose()
	rot2T := rot2.Transpose()

	j.LocalAnchor1 = rot1T.MulV(anchor.Sub(b1.Position))
	j.LocalAnchor2 = rot2T.MulV(anchor.Sub(b2.Position))

	j.P = Vec2{0.0, 0.0}

	j.Softness = 0.0
	j.BiasFactor = 0.2
}

func (j *Joint) PreStep(inv_dt float64) {
	rot1 := Mat22ByAngle(j.Body1.Rotation)
	rot2 := Mat22ByAngle(j.Body2.Rotation)

	j.R1 = rot1.MulV(j.LocalAnchor1)
	j.R2 = rot2.MulV(j.LocalAnchor2)

	// deltaV = deltaV0 + K * impulse
	// invM = [(1/m1 + 1/m2) * eye(2) - skew(r1) * invI1 * skew(r1) - skew(r2) * invI2 * skew(r2)]
	//      = [1/m1+1/m2     0    ] + invI1 * [r1.y*r1.y -r1.x*r1.y] + invI2 * [r1.y*r1.y -r1.x*r1.y]
	//        [    0     1/m1+1/m2]           [-r1.x*r1.y r1.x*r1.x]           [-r1.x*r1.y r1.x*r1.x]
	var k1, k2, k3 Mat22
	k1.Col1.X = j.Body1.invMass + j.Body2.invMass
	k1.Col2.X = 0.0
	k1.Col1.Y = 0.0
	k1.Col2.Y = j.Body1.invMass + j.Body2.invMass

	k2.Col1.X = j.Body1.invI * j.R1.Y * j.R1.Y
	k2.Col2.X = -j.Body1.invI * j.R1.X * j.R1.Y
	k2.Col1.Y = -j.Body1.invI * j.R1.X * j.R1.Y
	k2.Col2.Y = j.Body1.invI * j.R1.X * j.R1.X

	k3.Col1.X = j.Body2.invI * j.R2.Y * j.R2.Y
	k3.Col2.X = -j.Body2.invI * j.R2.X * j.R2.Y
	k3.Col1.Y = -j.Body2.invI * j.R2.X * j.R2.Y
	k3.Col2.Y = j.Body2.invI * j.R2.X * j.R2.X

	k := k1.Add(k2.Add(k3))
	k.Col1.X += j.Softness
	k.Col2.Y += j.Softness

	j.M = k.Invert()

	p1 := j.Body1.Position.Add(j.R1)
	p2 := j.Body2.Position.Add(j.R2)
	dp := p2.Sub(p1)

	if positionCorrection {
		j.Bias = MulSV(-j.BiasFactor*inv_dt, dp)
	} else {
		j.Bias = Vec2{0.0, 0.0}
	}

	if warmStarting {
		// Apply accumulated impulse.
		j.Body1.Velocity = j.Body1.Velocity.Sub(MulSV(j.Body1.invMass, j.P))
		j.Body1.AngularVelocity -= j.Body1.invI * CrossVV(j.R1, j.P)

		j.Body2.Velocity = j.Body2.Velocity.Add(MulSV(j.Body2.invMass, j.P))
		j.Body2.AngularVelocity += j.Body2.invI * CrossVV(j.R2, j.P)
	} else {
		j.P = Vec2{0.0, 0.0}
	}
}

func (j *Joint) ApplyImpulse() {
	// Vec2 dv = body2->velocity + Cross(body2->angularVelocity, r2) - body1->velocity - Cross(body1->angularVelocity, r1);
	var dv Vec2
	dv = j.Body2.Velocity
	dv = dv.Add(CrossSV(j.Body2.AngularVelocity, j.R2))
	dv = dv.Sub(j.Body1.Velocity)
	dv = dv.Sub(CrossSV(j.Body1.AngularVelocity, j.R1))

	// impulse = M * (bias - dv - softness * P);
	in := j.Bias.Sub(dv)
	in = in.Sub(MulSV(j.Softness, j.P))
	impulse := j.M.MulV(in)

	j.Body1.Velocity = j.Body1.Velocity.Sub(MulSV(j.Body1.invMass, impulse))
	j.Body1.AngularVelocity -= j.Body1.invI * CrossVV(j.R1, impulse)

	j.Body2.Velocity = j.Body2.Velocity.Add(MulSV(j.Body2.invMass, impulse))
	j.Body2.AngularVelocity += j.Body2.invI * CrossVV(j.R2, impulse)

	j.P = j.P.Add(impulse)
}

