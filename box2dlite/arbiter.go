package box2dlite

import (
	"math"
	"unsafe"
)

type EdgeNumbers uint8

type FeaturePair struct {
	InEdge1  EdgeNumbers
	OutEdge1 EdgeNumbers
	InEdge2  EdgeNumbers
	OutEdge2 EdgeNumbers
}

func (fp *FeaturePair) Value() uint32 {
	return ((uint32(fp.InEdge1) << 0) | (uint32(fp.OutEdge1) << 8) | (uint32(fp.InEdge2) << 16) | (uint32(fp.OutEdge2) << 24))
}

func (fp *FeaturePair) Flip() {
	*fp = FeaturePair {
		InEdge1 : fp.InEdge2,
		OutEdge1 : fp.OutEdge2,
		InEdge2 : fp.InEdge1,
		OutEdge2 : fp.OutEdge1,
	}
}

type Contact struct {
	Position                Vec2
	Normal                  Vec2
	R1, R2                  Vec2
	Separation              float64
	Pn                      float64 // accumulated normal impulse
	Pt                      float64 // accumulated tangent impulse
	Pnb                     float64 // accumulated normal impulse for position bias
	MassNormal, MassTangent float64
	Bias                    float64
	Feature                 FeaturePair
}

type ArbiterKey struct {
	Body1, Body2 *Body
}

func (k *ArbiterKey) Set(b1, b2 *Body) {
	if uintptr(unsafe.Pointer(b1)) < uintptr(unsafe.Pointer(b2)) {
		k.Body1 = b1
		k.Body2 = b2
	} else {
		k.Body1 = b2
		k.Body2 = b1
	}
}

type ArbiterKeys []ArbiterKey

func (k ArbiterKeys) Less(i, j int) bool {
	a1 := k[i]
	a2 := k[j]
	if uintptr(unsafe.Pointer(a1.Body1)) < uintptr(unsafe.Pointer(a2.Body1)) {
		return true
	}

	if (a1.Body1 == a2.Body1) && uintptr(unsafe.Pointer(a1.Body2)) < uintptr(unsafe.Pointer(a2.Body2)) {
		return true
	}

	return false
}

func (k ArbiterKeys) Swap(i, j int) {
	k[i], k[j] = k[j], k[i]
}

func (k ArbiterKeys) Len() int {
	return len(k)
}

const (
	MAX_POINTS = 2
)

type Arbiter struct {
	Contacts []*Contact

	Body1, Body2 *Body

	// Combined friction
	Friction float64
}

func (a *Arbiter) Set(b1 *Body, b2 *Body, contacts []*Contact) {
	if uintptr(unsafe.Pointer(b1)) < uintptr(unsafe.Pointer(b2)) {
		a.Body1 = b1
		a.Body2 = b2
	} else {
		a.Body1 = b2
		a.Body2 = b1
	}

	a.Contacts = contacts

	a.Friction = math.Sqrt(b1.Friction * b2.Friction)
}

func (a *Arbiter) Update(newContacts []*Contact) {
	var mergedContacts []*Contact

	for _, cNew := range newContacts {
		var cMatch *Contact
		for _, cOld := range a.Contacts {
			if cOld.Feature.Value() == cNew.Feature.Value() {
				cMatch = cOld
				break
			}
		}

		c := cNew
		if cMatch != nil {
			if warmStarting {
				c.Pn = cMatch.Pn
				c.Pt = cMatch.Pt
				c.Pnb = cMatch.Pnb
			} else {
				c.Pn = 0.0
				c.Pt = 0.0
				c.Pnb = 0.0
			}
		}
		mergedContacts = append(mergedContacts, c)
	}

	a.Contacts = mergedContacts
}

func (a *Arbiter) PreStep(inv_dt float64) {
	var k_allowedPenetration float64 = 0.01
	var k_biasFactor float64
	if positionCorrection {
		k_biasFactor = 0.2
	} else {
		k_biasFactor = 0.0
	}

	for _, c := range a.Contacts {
		r1 := c.Position.Sub(a.Body1.Position)
		r2 := c.Position.Sub(a.Body2.Position)

		// Precompute normal mass, tangent mass, and bias.
		rn1 := Dot(r1, c.Normal)
		rn2 := Dot(r2, c.Normal)
		kNormal := a.Body1.invMass + a.Body2.invMass
		kNormal += a.Body1.invI * (Dot(r1, r1) - rn1 * rn1) + a.Body2.invI * (Dot(r2, r2) - rn2 * rn2)
		c.MassNormal = 1.0 / kNormal

		tangent := CrossVS(c.Normal, 1.0)
		rt1 := Dot(r1, tangent)
		rt2 := Dot(r2, tangent)
		kTangent := a.Body1.invMass + a.Body2.invMass
		kTangent += a.Body1.invI * (Dot(r1, r1) - rt1 * rt1) + a.Body2.invI * (Dot(r2, r2) - rt2 * rt2)
		c.MassTangent = 1.0 /  kTangent;

		c.Bias = -k_biasFactor * inv_dt * math.Min(0.0, c.Separation + k_allowedPenetration)

		if accumulateImpulses {
			// Apply normal + friction impulse
			// Vec2 P = c->Pn * c->normal + c->Pt * tangent;
			P := MulSV(c.Pn, c.Normal)
			P = P.Add(MulSV(c.Pt, tangent))

			a.Body1.Velocity = a.Body1.Velocity.Sub(MulSV(a.Body1.invMass, P))
			a.Body1.AngularVelocity -= a.Body1.invI * CrossVV(r1, P)

			a.Body2.Velocity = a.Body2.Velocity.Add(MulSV(a.Body2.invMass, P))
			a.Body2.AngularVelocity += a.Body2.invI * CrossVV(r2, P)
		}
	}
}

func (a *Arbiter) ApplyImpulse() {
	b1 := a.Body1
	b2 := a.Body2

	for _, c := range a.Contacts {
		c.R1 = c.Position.Sub(b1.Position)
		c.R2 = c.Position.Sub(b2.Position)

		// Relative velocity at contact
		// Vec2 dv = b2->velocity + Cross(b2->angularVelocity, c->r2) - b1->velocity - Cross(b1->angularVelocity, c->r1);
		dv := b2.Velocity.Add(CrossSV(b2.AngularVelocity, c.R2))
		dv = dv.Sub(b1.Velocity)
		dv = dv.Sub(CrossSV(b1.AngularVelocity, c.R1))

		// Compute normal impulse
		vn := Dot(dv, c.Normal)

		dPn := c.MassNormal * (-vn + c.Bias)

		if accumulateImpulses {
			Pn0 := c.Pn
			c.Pn = math.Max(Pn0 + dPn, 0.0)
			dPn = c.Pn - Pn0
		} else {
			dPn = math.Max(dPn, 0.0)
		}

		// Apply contact impulse
		Pn := MulSV(dPn, c.Normal)

		b1.Velocity = b1.Velocity.Sub(MulSV(b1.invMass, Pn))
		b1.AngularVelocity -= b1.invI * CrossVV(c.R1, Pn)

		b2.Velocity = b2.Velocity.Add(MulSV(b2.invMass, Pn))
		b2.AngularVelocity += b2.invI * CrossVV(c.R2, Pn)

		// Relative velocity at contact
		// dv = b2->velocity + Cross(b2->angularVelocity, c->r2) - b1->velocity - Cross(b1->angularVelocity, c->r1);
		dv = b2.Velocity.Add(CrossSV(b2.AngularVelocity, c.R2))
		dv = dv.Sub(b1.Velocity)
		dv = dv.Sub(CrossSV(b1.AngularVelocity, c.R1))

		tangent := CrossVS(c.Normal, 1.0)
		vt := Dot(dv, tangent)
		dPt := c.MassTangent * (-vt)

		if accumulateImpulses {
			// Compute friction impulse
			maxPt := a.Friction * c.Pn

			// Clamp friction
			oldTangentImpulse := c.Pt
			c.Pt = Clamp(oldTangentImpulse + dPt, -maxPt, maxPt)
			dPt = c.Pt - oldTangentImpulse
		}

		// Apply contact impulse
		Pt := MulSV(dPt, tangent)

		b1.Velocity = b1.Velocity.Sub(MulSV(b1.invMass, Pt))
		b1.AngularVelocity -= b1.invI * CrossVV(c.R1, Pt)

		b2.Velocity = b2.Velocity.Add(MulSV(b2.invMass, Pt))
		b2.AngularVelocity += b2.invI * CrossVV(c.R2, Pt)
	}
}

