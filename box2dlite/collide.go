package box2dlite

// Box vertex and edge numbering:
//
//        ^ y
//        |
//        e1
//   v2 ------ v1
//    |        |
// e2 |        | e4  --> x
//    |        |
//   v3 ------ v4
//        e3

type Axis uint8

const (
	FACE_A_X = iota
	FACE_A_Y
	FACE_B_X
	FACE_B_Y
)

const (
	NO_EDGE = iota
	EDGE1
	EDGE2
	EDGE3
	EDGE4
)

type ClipVertex struct {
	v  Vec2
	fp FeaturePair
}

func ClipSegmentToLine(vOut []ClipVertex, vIn []ClipVertex, normal Vec2, offset float64, clipEdge EdgeNumbers) int {
	// Start with no output points
	var numOut int = 0

	// Calculate the distance of end points to the line
	distance0 := Dot(normal, vIn[0].v) - offset
	distance1 := Dot(normal, vIn[1].v) - offset

	// If the points are behind the plane
	if distance0 <= 0.0 {
		vOut[numOut] = vIn[0]
		numOut++
	}
	if distance1 <= 0.0 {
		vOut[numOut] = vIn[1]
		numOut++
	}

	// If the points are on different sides of the plane
	if distance0*distance1 < 0.0 {
		// Find intersection point of edge and plane
		interp := distance0 / (distance0 - distance1)
		vOut[numOut].v = vIn[0].v.Add(MulSV(interp, (vIn[1].v.Sub(vIn[0].v))))
		if distance0 > 0.0 {
			vOut[numOut].fp = vIn[0].fp
			vOut[numOut].fp.InEdge1 = clipEdge
			vOut[numOut].fp.InEdge2 = NO_EDGE
		} else {
			vOut[numOut].fp = vIn[1].fp
			vOut[numOut].fp.OutEdge1 = clipEdge
			vOut[numOut].fp.OutEdge2 = NO_EDGE
		}
		numOut++
	}

	return numOut
}

func ComputeIncidentEdge(c []ClipVertex, h Vec2, pos Vec2, rot Mat22, normal Vec2) {
	// the normal is from the reference box. convert it
	// to the incident boxe's frame and flip sign.
	rotT := rot.Transpose()
	// Vec2 n = -(RotT * normal);
	n := rotT.MulV(normal)
	n = n.Negative()
	nAbs := n.Abs()

	if nAbs.X > nAbs.Y {
		if n.X > 0.0 {
			c[0].v = Vec2{h.X, -h.Y}
			c[0].fp.InEdge2 = EDGE3
			c[0].fp.OutEdge2 = EDGE4

			c[1].v = Vec2{h.X, h.Y}
			c[1].fp.InEdge2 = EDGE4
			c[1].fp.OutEdge2 = EDGE1
		} else {
			c[0].v = Vec2{-h.X, h.Y}
			c[0].fp.InEdge2 = EDGE1
			c[0].fp.OutEdge2 = EDGE2

			c[1].v = Vec2{-h.X, -h.Y}
			c[1].fp.InEdge2 = EDGE2
			c[1].fp.OutEdge2 = EDGE3
		}
	} else {
		if n.Y > 0.0 {
			c[0].v = Vec2{h.X, h.Y}
			c[0].fp.InEdge2 = EDGE4
			c[0].fp.OutEdge2 = EDGE1

			c[1].v = Vec2{-h.X, h.Y}
			c[1].fp.InEdge2 = EDGE1
			c[1].fp.OutEdge2 = EDGE2
		} else {
			c[0].v = Vec2{-h.X, -h.Y}
			c[0].fp.InEdge2 = EDGE2
			c[0].fp.OutEdge2 = EDGE3

			c[1].v = Vec2{h.X, -h.Y}
			c[1].fp.InEdge2 = EDGE3
			c[1].fp.OutEdge2 = EDGE4
		}
	}

	c[0].v = pos.Add(rot.MulV(c[0].v))
	c[1].v = pos.Add(rot.MulV(c[1].v))
}

// The normal points from A to B
func Collide(bodyA, bodyB *Body) []*Contact {
	// Setup
	hA := MulSV(0.5, bodyA.Width)
	hB := MulSV(0.5, bodyB.Width)

	posA := bodyA.Position
	posB := bodyB.Position

	RotA := Mat22ByAngle(bodyA.Rotation)
	RotB := Mat22ByAngle(bodyB.Rotation)

	RotAT := RotA.Transpose()
	RotBT := RotB.Transpose()

	dp := posB.Sub(posA)
	dA := RotAT.MulV(dp)
	dB := RotBT.MulV(dp)

	var C Mat22 = RotAT.MulM(RotB)
	var absC Mat22 = C.Abs()
	var absCT Mat22 = absC.Transpose()

	// Box A faces
	// Vec2 faceA = Abs(dA) - hA - absC * hB;
	var faceA Vec2 = dA.Abs()
	faceA = faceA.Sub(hA)
	faceA = faceA.Sub(absC.MulV(hB))
	if faceA.X > 0.0 || faceA.Y > 0.0 {
		return nil
	}

	// Box B faces
	// Vec2 faceB = Abs(dB) - absCT * hA - hB;
	var faceB Vec2 = dB.Abs()
	faceB = faceB.Sub(absCT.MulV(hA))
	faceB = faceB.Sub(hB)
	if faceB.X > 0.0 || faceB.Y > 0.0 {
		return nil
	}

	// Find best axis
	var axis Axis
	var separation float64
	var normal Vec2

	// Box A faces
	axis = FACE_A_X
	separation = faceA.X
	if dA.X > 0.0 {
		normal = RotA.Col1
	} else {
		normal = RotA.Col1.Negative()
	}

	relativeTol := 0.95
	absoluteTol := 0.01

	if faceA.Y > relativeTol*separation+absoluteTol*hA.Y {
		axis = FACE_A_Y
		separation = faceA.Y
		if dA.Y > 0.0 {
			normal = RotA.Col2
		} else {
			normal = RotA.Col2.Negative()
		}
	}

	// Box B faces
	if faceB.X > relativeTol*separation+absoluteTol*hB.X {
		axis = FACE_B_X
		separation = faceB.X
		if dB.X > 0.0 {
			normal = RotB.Col1
		} else {
			normal = RotB.Col1.Negative()
		}
	}

	if faceB.Y > relativeTol*separation+absoluteTol*hB.Y {
		axis = FACE_B_Y
		separation = faceB.Y
		if dB.Y > 0.0 {
			normal = RotB.Col2
		} else {
			normal = RotB.Col2.Negative()
		}
	}

	// Setup clipping plane data based on the separating axis
	var frontNormal, sideNormal Vec2
	var incidentEdge [2]ClipVertex
	var front, negSide, posSide float64
	var negEdge, posEdge EdgeNumbers

	// Compute the clipping lines and the line segment to be clipped.
	switch axis {
	case FACE_A_X:
		{
			frontNormal = normal
			front = Dot(posA, frontNormal) + hA.X
			sideNormal = RotA.Col2
			var side float64 = Dot(posA, sideNormal)
			negSide = -side + hA.Y
			posSide = side + hA.Y
			negEdge = EDGE3
			posEdge = EDGE1
			ComputeIncidentEdge(incidentEdge[:], hB, posB, RotB, frontNormal)
		}
		break

	case FACE_A_Y:
		{
			frontNormal = normal
			front = Dot(posA, frontNormal) + hA.Y
			sideNormal = RotA.Col1
			var side float64 = Dot(posA, sideNormal)
			negSide = -side + hA.X
			posSide = side + hA.X
			negEdge = EDGE2
			posEdge = EDGE4
			ComputeIncidentEdge(incidentEdge[:], hB, posB, RotB, frontNormal)
		}
		break

	case FACE_B_X:
		{
			frontNormal = normal.Negative()
			front = Dot(posB, frontNormal) + hB.X
			sideNormal = RotB.Col2
			var side float64 = Dot(posB, sideNormal)
			negSide = -side + hB.Y
			posSide = side + hB.Y
			negEdge = EDGE3
			posEdge = EDGE1
			ComputeIncidentEdge(incidentEdge[:], hA, posA, RotA, frontNormal)
		}
		break

	case FACE_B_Y:
		{
			frontNormal = normal.Negative()
			front = Dot(posB, frontNormal) + hB.Y
			sideNormal = RotB.Col1
			var side float64 = Dot(posB, sideNormal)
			negSide = -side + hB.X
			posSide = side + hB.X
			negEdge = EDGE2
			posEdge = EDGE4
			ComputeIncidentEdge(incidentEdge[:], hA, posA, RotA, frontNormal)
		}
		break
	}

	// clip other face with 5 box planes (1 face plane, 4 edge planes)

	var clipPoints1 [2]ClipVertex
	var clipPoints2 [2]ClipVertex
	var np int

	// Clip to box side 1
	np = ClipSegmentToLine(clipPoints1[:], incidentEdge[:], sideNormal.Negative(), negSide, negEdge)

	if np < 2 {
		return nil
	}

	// Clip to negative box side 1
	np = ClipSegmentToLine(clipPoints2[:], clipPoints1[:], sideNormal, posSide, posEdge)

	if np < 2 {
		return nil
	}

	// Now clipPoints2 contains the clipping points.
	// Due to roundoff, it is possible that clipping removes all points.

	var contacts []*Contact
	for i := 0; i < 2; i++ {
		var separation float64 = Dot(frontNormal, clipPoints2[i].v) - front

		if separation <= 0 {
			var c Contact
			c.Separation = separation
			c.Normal = normal
			// slide contact point onto reference face (easy to cull)
			c.Position = clipPoints2[i].v.Sub(MulSV(separation, frontNormal))
			c.Feature = clipPoints2[i].fp
			if axis == FACE_B_X || axis == FACE_B_Y {
				c.Feature.Flip()
			}
			contacts = append(contacts, &c)
		}
	}

	return contacts
}
