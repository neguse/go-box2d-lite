package box2dlite

var accumulateImpulses bool = true
var warmStarting bool = true
var positionCorrection bool = true

type World struct {
	Bodies     []*Body
	Joints     []*Joint
	Arbiters   map[ArbiterKey]*Arbiter
	Gravity    Vec2
	Iterations int
}

func NewWorld(g Vec2, i int) *World {
	return &World{
		Arbiters:   make(map[ArbiterKey]*Arbiter),
		Gravity:    g,
		Iterations: i,
	}
}

func (w *World) AddBody(b *Body) {
	w.Bodies = append(w.Bodies, b)
}

func (w *World) AddJoint(j *Joint) {
	w.Joints = append(w.Joints, j)
}

func (w *World) Clear() {
	w.Bodies = nil
	w.Joints = nil
	w.Arbiters = make(map[ArbiterKey]*Arbiter)
}

func (w *World) BroadPhase() {
	// O(n^2) broad-phase
	for i, bi := range w.Bodies {
		for _, bj := range w.Bodies[i+1:] {

			if bi.invMass == 0.0 && bj.invMass == 0.0 {
				continue
			}

			var key ArbiterKey
			key.Set(bi, bj)
			// ArbiterKey.Set() orders Body1 and Body2.
			// Then don't using bi and bj.
			// Use key.Body1, key.Body2.
			contacts := Collide(key.Body1, key.Body2)

			if len(contacts) > 0 {
				it, ok := w.Arbiters[key]
				if !ok {
					var a Arbiter
					a.Set(bi, bj, contacts)
					w.Arbiters[key] = &a
				} else {
					it.Update(contacts)
				}
			} else {
				delete(w.Arbiters, key)
			}
		}
	}
}

func (w *World) Step(dt float64) {
	var inv_dt float64
	if dt > 0.0 {
		inv_dt = 1.0 / dt
	} else {
		inv_dt = 0.0
	}

	// Determine overlapping bodies and update contact points.
	w.BroadPhase()

	// Integrate forces.
	for _, b := range w.Bodies {
		if b.invMass == 0.0 {
			continue
		}

		b.Velocity = b.Velocity.Add(MulSV(dt, (w.Gravity.Add(MulSV(b.invMass, b.Force)))))
		b.AngularVelocity += dt * b.invI * b.Torque
	}

	// Perform pre-steps.
	for _, arb := range w.Arbiters {
		arb.PreStep(inv_dt)
	}

	for _, j := range w.Joints {
		j.PreStep(inv_dt)
	}

	// Perform iterations
	for i := 0; i < w.Iterations; i++ {
		for _, arb := range w.Arbiters {
			arb.ApplyImpulse()
		}

		for _, j := range w.Joints {
			j.ApplyImpulse()
		}
	}

	// Integrate Velocities
	for _, b := range w.Bodies {
		b.Position = b.Position.Add(MulSV(dt, b.Velocity))
		b.Rotation += dt * b.AngularVelocity

		b.Force = Vec2{0.0, 0.0}
		b.Torque = 0.0
	}
}

