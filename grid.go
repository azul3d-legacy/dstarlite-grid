// Copyright 2014 The Azul3D Authors. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

// Package grid implements D* Lite grid-based pathfinding.
package grid

import (
	"math"

	"azul3d.org/dstarlite.v1"
)

// Coord represents an single grid coordinate pair [x, y]
type Coord [2]int

// Implements dstarlite.State interface.
func (a Coord) Equals(b dstarlite.State) bool {
	return a == b.(Coord)
}

// Dist returns manhattan distance between two coordinates
func (a Coord) Dist(b Coord) float64 {
	return math.Abs(float64(b[0]-a[0])) + math.Abs(float64(b[1]-a[1]))
}

// Data represents an actual grid's data.
type Data struct {
	dsl           *dstarlite.Planner
	coords        map[Coord]float64
	width, height int
	start, goal   Coord
}

// Start returns the start coordinate, as it is currently.
func (d *Data) Start() Coord {
	return d.dsl.Start().(Coord)
}

// Goal returns the goal coordinate, as it is currently.
func (d *Data) Goal() Coord {
	return d.dsl.Goal().(Coord)
}

// Size returns the width and height of the grid.
func (d *Data) Size() (width, height int) {
	return d.width, d.height
}

// Implements dstarlite.Data interface.
func (d *Data) Cost(aa, bb dstarlite.State) float64 {
	a := aa.(Coord)
	b := bb.(Coord)
	if a.Dist(b) != 1 {
		return math.Inf(1)
	} else {
		costA := d.coords[a]
		if costA < 0 {
			return math.Inf(1)
		}

		costB := d.coords[b]
		if costB < 0 {
			return math.Inf(1)
		}

		cost := ((costA + costB) / 2.0) + 1.0
		if cost <= 0 {
			panic("cost <= 0; should not happen!")
		}
		return cost
	}
}

// Implements dstarlite.Data interface.
func (d *Data) Dist(aa, bb dstarlite.State) float64 {
	a := aa.(Coord)
	b := bb.(Coord)
	return a.Dist(b)
}

// Returns an slice of neighbors to the current grid data cell excluding
// neighbor cells whom have an negative cost value.
func (d *Data) neighbors(ss dstarlite.State) (sl []dstarlite.State) {
	s := ss.(Coord)

	next := func(x, y int) {
		c := Coord{x, y}
		_, ok := d.Get(c)
		if ok {
			sl = append(sl, c)
		}
	}

	x := s[0]
	y := s[1]
	next(x-1, y+1) // Top Left
	next(x, y+1)   // Top Center
	next(x+1, y+1) // Top Right

	next(x-1, y) // Center Left
	next(x+1, y) // Center Right

	next(x-1, y-1) // Bottom Left
	next(x, y-1)   // Bottom Center
	next(x+1, y-1) // Bottom Right

	return sl
}

// Implements dstarlite.Data interface.
func (d *Data) Succ(s dstarlite.State) []dstarlite.State {
	return d.neighbors(s)
}

// Implements dstarlite.Data interface.
func (d *Data) Pred(s dstarlite.State) []dstarlite.State {
	return d.neighbors(s)
}

// Set changes the cost of traversal to the given coordinate on the grid to the
// specified value.
//
// If the coordinate is outside of the grid's dimensions, an panic will occur.
func (d *Data) Set(pos Coord, value float64) {
	if pos[0] > d.width || pos[0] < 0 || pos[1] > d.height || pos[1] < 0 {
		panic("Set(): Coordinate outside of grid's dimensions!")
	}

	if value == d.coords[pos] {
		// Cost of traversal has not changed; simply do nothing.
		return
	}

	// Find all affected neighbor cells
	preds := d.Pred(pos)
	succs := d.Succ(pos)

	// Keep track of old costs
	predVals := make(map[Coord]float64)
	for _, sPrime := range preds {
		predVals[sPrime.(Coord)] = d.Cost(sPrime, pos)
	}

	succVals := make(map[Coord]float64)
	for _, sPrime := range succs {
		succVals[sPrime.(Coord)] = d.Cost(pos, sPrime)
	}

	// Change stored cost
	d.coords[pos] = value

	// Flag changed to new costs
	for _, sPrime := range preds {
		d.dsl.FlagChanged(sPrime, pos, predVals[sPrime.(Coord)], d.Cost(sPrime, pos))
	}

	for _, sPrime := range succs {
		d.dsl.FlagChanged(pos, sPrime, succVals[sPrime.(Coord)], d.Cost(pos, sPrime))
	}
}

// Get returns the attached interface value from the given coordinate on the
// grid.
//
// If the coordinate is outside of the grid's dimensions, ok will equal false.
func (d *Data) Get(pos Coord) (value float64, ok bool) {
	if pos[0] > d.width || pos[0] < 0 || pos[1] > d.height || pos[1] < 0 {
		ok = false
		return
	}

	value, ok = d.coords[pos]
	return
}

// UpdateStart updates the starting position. This can be used to move a long
// the path efficiently.
func (d *Data) UpdateStart(start Coord) {
	d.dsl.UpdateStart(start)
}

// Plan recomputes the lowest cost path through the map, taking into account
// changes in start location and edge costs.
//
// If no path is found, nil is returned.
func (d *Data) Plan() (path []Coord) {
	sPath := d.dsl.Plan()

	path = make([]Coord, len(sPath))
	for i, s := range sPath {
		path[i] = s.(Coord)
	}

	return
}

// New returns an new grid data structure given an width and height where each
// cell indicates the cost of traversal.
//
// If the width or height are <= 0; an panic will occur.
//
// If either start or goal coordinates are outside the coordinates of the grid,
// an panic will occur.
func New(width, height int, start, goal Coord) *Data {
	if width <= 0 || height <= 0 {
		panic("New(): Cannot create grid of size <= 0")
	}
	if start[0] > width || start[0] < 0 || start[1] > height || start[1] < 0 {
		panic("New(): Start coordinate outside of grid's dimensions!")
	}
	if goal[0] > width || goal[0] < 0 || goal[1] > height || goal[1] < 0 {
		panic("New(): Goal coordinate outside of grid's dimensions!")
	}

	d := new(Data)
	d.width = width
	d.height = height
	d.start = start
	d.goal = goal
	d.coords = make(map[Coord]float64)
	d.dsl = dstarlite.New(d, start, goal)
	return d
}
