package main

import (
	"log"
	"math"
	"math/rand"
	"runtime"

	b2d "github.com/neguse/go-box2d-lite/box2dlite"
	"github.com/veandco/go-sdl2/sdl"
)

type App struct {
	Window   *sdl.Window
	Renderer *sdl.Renderer

	World *b2d.World

	quit bool
}

const timeStep = 1.0 / 60

func NewApp() *App {
	window, err := sdl.CreateWindow(
		"box2d-lite", sdl.WINDOWPOS_CENTERED, sdl.WINDOWPOS_CENTERED,
		800, 600, sdl.WINDOW_OPENGL)
	if err != nil {
		log.Println(err)
		return nil
	}

	renderer, err := sdl.CreateRenderer(window, -2, sdl.RENDERER_SOFTWARE)
	if err != nil {
		log.Println(err)
		return nil
	}

	gravity := b2d.Vec2{0.0, -10.0}
	iterations := 10
	world := b2d.NewWorld(gravity, iterations)

	return &App{
		Window:   window,
		Renderer: renderer,
		World:    world,
	}
}

func (app *App) Destroy() {
	app.Renderer.Destroy()
	app.Window.Destroy()
}

func (app *App) EventLoop() {
	t1 := sdl.GetTicks()

	for {
		app.DoEvents()

		t2 := sdl.GetTicks()
		app.OnUpdate(t2 - t1)
		app.OnRender()
		t1 = t2

		sdl.Delay(16)
		app.Present()

		if app.quit {
			break
		}
	}
}

func (app *App) DoEvents() {
	for {
		e := sdl.PollEvent()
		if e == nil {
			break
		}
		app.ProcessEvent(e)
	}
}

// Single box
func (app *App) Demo1() {
	app.World.Clear()

	var b1, b2 b2d.Body

	b1.Set(&b2d.Vec2{100.0, 20.0}, math.MaxFloat64)
	b1.Position = b2d.Vec2{0.0, -0.5 * b1.Width.Y}
	app.World.AddBody(&b1)

	b2.Set(&b2d.Vec2{1.0, 1.0}, 200.0)
	b2.Position = b2d.Vec2{0.0, 4.0}
	app.World.AddBody(&b2)
}

// A simple pendulum
func (app *App) Demo2() {
	app.World.Clear()

	var b2, b1 b2d.Body
	var j b2d.Joint

	b1.Set(&b2d.Vec2{100.0, 20.0}, math.MaxFloat64)
	b1.Friction = 0.2
	b1.Position = b2d.Vec2{0.0, -0.5 * b1.Width.Y}
	b1.Rotation = 0.0
	app.World.AddBody(&b1)

	b2.Set(&b2d.Vec2{1.0, 1.0}, 100.0)
	b2.Friction = 0.2
	b2.Position = b2d.Vec2{9.0, 11.0}
	b2.Rotation = 0.0
	app.World.AddBody(&b2)

	j.Set(&b1, &b2, &b2d.Vec2{0.0, 11.0})
	app.World.AddJoint(&j)
}

// Varying friction coefficients
func (app *App) Demo3() {
	app.World.Clear()

	{
		var b b2d.Body
		b.Set(&b2d.Vec2{100.0, 20.0}, math.MaxFloat64)
		b.Position = b2d.Vec2{0.0, -0.5 * b.Width.Y}
		app.World.AddBody(&b)
	}

	{
		var b b2d.Body
		b.Set(&b2d.Vec2{13.0, 0.25}, math.MaxFloat64)
		b.Position = b2d.Vec2{-2.0, 11.0}
		b.Rotation = -0.25
		app.World.AddBody(&b)
	}

	{
		var b b2d.Body
		b.Set(&b2d.Vec2{0.25, 1.0}, math.MaxFloat64)
		b.Position = b2d.Vec2{5.25, 9.5}
		app.World.AddBody(&b)
	}

	{
		var b b2d.Body
		b.Set(&b2d.Vec2{13.0, 0.25}, math.MaxFloat64)
		b.Position = b2d.Vec2{2.0, 7.0}
		b.Rotation = 0.25
		app.World.AddBody(&b)
	}

	{
		var b b2d.Body
		b.Set(&b2d.Vec2{0.25, 1.0}, math.MaxFloat64)
		b.Position = b2d.Vec2{-5.25, 5.5}
		app.World.AddBody(&b)
	}

	frictions := []float64{0.75, 0.5, 0.35, 0.1, 0.0}
	for i := 0; i < 5; i++ {
		var b b2d.Body
		b.Set(&b2d.Vec2{0.5, 0.5}, 25.0)
		b.Friction = frictions[i]
		b.Position = b2d.Vec2{-7.5 + 2.0*float64(i), 14.0}
		app.World.AddBody(&b)
	}

}

// A vertical stack
func (app *App) Demo4() {
	app.World.Clear()

	{
		var b b2d.Body
		b.Set(&b2d.Vec2{100.0, 20.0}, math.MaxFloat64)
		b.Friction = 0.2
		b.Position = b2d.Vec2{0.0, -0.5 * b.Width.Y}
		b.Rotation = 0.0
		app.World.AddBody(&b)
	}

	for i := 0; i < 10; i++ {
		var b b2d.Body
		b.Set(&b2d.Vec2{1.0, 1.0}, 1.0)
		b.Friction = 0.2
		x := rand.Float64()*0.2 - 0.1
		b.Position = b2d.Vec2{x, 0.51 + 1.05*float64(i)}
		app.World.AddBody(&b)
	}

}

// A pyramid
func (app *App) Demo5() {
	app.World.Clear()

	{
		var b b2d.Body
		b.Set(&b2d.Vec2{100.0, 20.0}, math.MaxFloat64)
		b.Friction = 0.2
		b.Position = b2d.Vec2{0.0, -0.5 * b.Width.Y}
		b.Rotation = 0.0
		app.World.AddBody(&b)
	}

	x := b2d.Vec2{-6.0, 0.75}

	for i := 0; i < 12; i++ {
		y := x
		for j := i; j < 12; j++ {
			var b b2d.Body
			b.Set(&b2d.Vec2{1.0, 1.0}, 10.0)
			b.Friction = 0.2
			b.Position = y
			app.World.AddBody(&b)

			y = y.Add(b2d.Vec2{1.125, 0.0})
		}

		x = x.Add(b2d.Vec2{0.5625, 2.0})
	}
}

// A teeter
func (app *App) Demo6() {
	app.World.Clear()

	var b1, b2, b3, b4, b5 b2d.Body
	b1.Set(&b2d.Vec2{100.0, 20.0}, math.MaxFloat64)
	b1.Position = b2d.Vec2{0.0, -0.5 * b1.Width.Y}
	app.World.AddBody(&b1)

	b2.Set(&b2d.Vec2{12.0, 0.25}, 100)
	b2.Position = b2d.Vec2{0.0, 1.0}
	app.World.AddBody(&b2)

	b3.Set(&b2d.Vec2{0.5, 0.5}, 25.0)
	b3.Position = b2d.Vec2{-5.0, 2.0}
	app.World.AddBody(&b3)

	b4.Set(&b2d.Vec2{0.5, 0.5}, 25.0)
	b4.Position = b2d.Vec2{-5.5, 2.0}
	app.World.AddBody(&b4)

	b5.Set(&b2d.Vec2{1.0, 1.0}, 100)
	b5.Position = b2d.Vec2{5.5, 15.0}
	app.World.AddBody(&b5)

	{
		var j b2d.Joint
		j.Set(&b1, &b2, &b2d.Vec2{0.0, 1.0})
		app.World.AddJoint(&j)
	}

}

// A suspension bridge
func (app *App) Demo7() {
	app.World.Clear()

	var ba []*b2d.Body

	{
		var b b2d.Body
		b.Set(&b2d.Vec2{100.0, 20.0}, math.MaxFloat64)
		b.Friction = 0.2
		b.Position = b2d.Vec2{0.0, -0.5 * b.Width.Y}
		b.Rotation = 0.0
		app.World.AddBody(&b)
		ba = append(ba, &b)
	}

	const numPlunks = 15
	const mass = 50.0

	for i := 0; i < numPlunks; i++ {
		var b b2d.Body
		b.Set(&b2d.Vec2{1.0, 0.25}, mass)
		b.Friction = 0.2
		b.Position = b2d.Vec2{-8.5 + 1.25*float64(i), 5.0}
		app.World.AddBody(&b)
		ba = append(ba, &b)
	}

	// Tuning
	const frequencyHz = 2.0
	const dampingRatio = 0.7

	// frequency in radians
	const omega = 2.0 * math.Pi * frequencyHz

	// damping coefficient
	const d = 2.0 * mass * dampingRatio * omega

	// spring stifness
	const k = mass * omega * omega

	// magic formulas
	const softness = 1.0 / (d + timeStep*k)
	const biasFactor = timeStep * k / (d + timeStep*k)

	for i := 0; i <= numPlunks; i++ {
		var j b2d.Joint
		j.Set(ba[i], ba[(i+1)%(numPlunks+1)], &b2d.Vec2{-9.125 + 1.25*float64(i), 5.0})
		j.Softness = softness
		j.BiasFactor = biasFactor
		app.World.AddJoint(&j)
	}

}

// Dominos
func (app *App) Demo8() {
	app.World.Clear()

	var b1 b2d.Body
	b1.Set(&b2d.Vec2{100.0, 20.0}, math.MaxFloat64)
	b1.Position = b2d.Vec2{0.0, -0.5 * b1.Width.Y}
	app.World.AddBody(&b1)

	{
		var b b2d.Body
		b.Set(&b2d.Vec2{12.0, 0.5}, math.MaxFloat64)
		b.Position = b2d.Vec2{-1.5, 10.0}
		app.World.AddBody(&b)
	}

	for i := 0; i < 10; i++ {
		var b b2d.Body
		b.Set(&b2d.Vec2{0.2, 2.0}, 10.0)
		b.Position = b2d.Vec2{-6.0 + 1.0*float64(i), 11.125}
		b.Friction = 0.1
		app.World.AddBody(&b)
	}

	{
		var b b2d.Body
		b.Set(&b2d.Vec2{14.0, 0.5}, math.MaxFloat64)
		b.Position = b2d.Vec2{1.0, 6.0}
		b.Rotation = 0.3
		app.World.AddBody(&b)
	}

	var b2 b2d.Body
	b2.Set(&b2d.Vec2{0.5, 3.0}, math.MaxFloat64)
	b2.Position = b2d.Vec2{-7.0, 4.0}
	app.World.AddBody(&b2)

	var b3 b2d.Body
	b3.Set(&b2d.Vec2{12.0, 0.25}, 20.0)
	b3.Position = b2d.Vec2{-0.9, 1.0}
	app.World.AddBody(&b3)

	{
		var j b2d.Joint
		j.Set(&b1, &b3, &b2d.Vec2{-2.0, 1.0})
		app.World.AddJoint(&j)
	}

	var b4 b2d.Body
	b4.Set(&b2d.Vec2{0.5, 0.5}, 10.0)
	b4.Position = b2d.Vec2{-10.0, 15.0}
	app.World.AddBody(&b4)

	{
		var j b2d.Joint
		j.Set(&b2, &b4, &b2d.Vec2{-7.0, 15.0})
		app.World.AddJoint(&j)
	}

	var b5 b2d.Body
	b5.Set(&b2d.Vec2{2.0, 2.0}, 20.0)
	b5.Position = b2d.Vec2{6.0, 2.5}
	b5.Friction = 0.1
	app.World.AddBody(&b5)

	{
		var j b2d.Joint
		j.Set(&b1, &b5, &b2d.Vec2{6.0, 2.6})
		app.World.AddJoint(&j)
	}

	var b6 b2d.Body
	b6.Set(&b2d.Vec2{2.0, 0.2}, 10.0)
	b6.Position = b2d.Vec2{6.0, 3.6}
	app.World.AddBody(&b6)

	{
		var j b2d.Joint
		j.Set(&b5, &b6, &b2d.Vec2{7.0, 3.5})
		app.World.AddJoint(&j)
	}

}

// A multi-pendulum
func (app *App) Demo9() {
	app.World.Clear()

	var b1 *b2d.Body

	{
		var b b2d.Body
		b.Set(&b2d.Vec2{100.0, 20.0}, math.MaxFloat64)
		b.Position = b2d.Vec2{0.0, -0.5 * b.Width.Y}
		app.World.AddBody(&b)
		b1 = &b
	}

	const mass = 10.0

	// Tuning
	const frequencyHz = 4.0
	const dampingRatio = 0.7

	// frequency in radians
	const omega = 2.0 * math.Pi * frequencyHz

	// damping coefficient
	const d = 2.0 * mass * dampingRatio * omega

	// spring stiffness
	const k = mass * omega * omega

	// magic formulas
	const softness = 1.0 / (d + timeStep*k)
	const biasFactor = timeStep * k / (d + timeStep*k)

	const y = 12.0

	for i := 0; i < 15; i++ {
		x := b2d.Vec2{0.5 + float64(i), y}

		var b b2d.Body
		b.Set(&b2d.Vec2{0.75, 0.25}, mass)
		b.Friction = 0.2
		b.Position = x
		b.Rotation = 0.0
		app.World.AddBody(&b)

		var j b2d.Joint
		j.Set(b1, &b, &b2d.Vec2{float64(i), y})
		j.Softness = softness
		j.BiasFactor = biasFactor
		app.World.AddJoint(&j)

		b1 = &b
	}

}

func (app *App) ProcessEvent(e interface{}) {
	// fmt.Println(reflect.TypeOf(e), e)

	switch t := e.(type) {
	case *sdl.QuitEvent:
		app.quit = true
	case *sdl.KeyDownEvent:
		switch t.Keysym.Sym {
		case sdl.K_1:
			app.Demo1()
		case sdl.K_2:
			app.Demo2()
		case sdl.K_3:
			app.Demo3()
		case sdl.K_4:
			app.Demo4()
		case sdl.K_5:
			app.Demo5()
		case sdl.K_6:
			app.Demo6()
		case sdl.K_7:
			app.Demo7()
		case sdl.K_8:
			app.Demo8()
		case sdl.K_9:
			app.Demo9()
		}
	}
}

func (app *App) OnUpdate(ms uint32) {
	app.World.Step(timeStep)
}

func (app *App) RenderBody(b *b2d.Body) {
	app.Renderer.SetDrawColor(0xff, 0xff, 0xff, 0xff)

	R := b2d.Mat22ByAngle(b.Rotation)
	x := b.Position
	h := b2d.MulSV(0.5, b.Width)

	o := b2d.Vec2{400, 400}
	S := b2d.Mat22{b2d.Vec2{20.0, 0.0}, b2d.Vec2{0.0, -20.0}}

	v1 := o.Add(S.MulV(x.Add(R.MulV(b2d.Vec2{-h.X, -h.Y}))))
	v2 := o.Add(S.MulV(x.Add(R.MulV(b2d.Vec2{h.X, -h.Y}))))
	v3 := o.Add(S.MulV(x.Add(R.MulV(b2d.Vec2{h.X, h.Y}))))
	v4 := o.Add(S.MulV(x.Add(R.MulV(b2d.Vec2{-h.X, h.Y}))))

	app.Renderer.DrawLine(int(v1.X), int(v1.Y), int(v2.X), int(v2.Y))
	app.Renderer.DrawLine(int(v2.X), int(v2.Y), int(v3.X), int(v3.Y))
	app.Renderer.DrawLine(int(v3.X), int(v3.Y), int(v4.X), int(v4.Y))
	app.Renderer.DrawLine(int(v4.X), int(v4.Y), int(v1.X), int(v1.Y))
}

func (app *App) RenderJoint(j *b2d.Joint) {
	app.Renderer.SetDrawColor(0x80, 0x80, 0x80, 0x80)

	b1 := j.Body1
	b2 := j.Body2

	R1 := b2d.Mat22ByAngle(b1.Rotation)
	R2 := b2d.Mat22ByAngle(b2.Rotation)

	x1 := b1.Position
	p1 := x1.Add(R1.MulV(j.LocalAnchor1))

	x2 := b2.Position
	p2 := x2.Add(R2.MulV(j.LocalAnchor2))

	o := b2d.Vec2{400, 400}
	S := b2d.Mat22{b2d.Vec2{20.0, 0.0}, b2d.Vec2{0.0, -20.0}}

	x1 = o.Add(S.MulV(x1))
	p1 = o.Add(S.MulV(p1))
	x2 = o.Add(S.MulV(x2))
	p2 = o.Add(S.MulV(p2))

	app.Renderer.DrawLine(int(x1.X), int(x1.Y), int(p1.X), int(p1.Y))
	app.Renderer.DrawLine(int(x2.X), int(x2.Y), int(p2.X), int(p2.Y))
}

func (app *App) OnRender() {
	app.Renderer.SetDrawColor(0x00, 0x00, 0x00, 0x00)
	app.Renderer.Clear()

	for _, b := range app.World.Bodies {
		app.RenderBody(b)
	}
	for _, j := range app.World.Joints {
		app.RenderJoint(j)
	}
}

func (app *App) Present() {
	app.Renderer.Present()
}

func init() {
	// http://www.oki-osk.jp/esc/golang/cgo-osx.html#3
	runtime.LockOSThread()
}

func main() {

	app := NewApp()

	app.Demo1()

	app.EventLoop()

	app.Destroy()
}
