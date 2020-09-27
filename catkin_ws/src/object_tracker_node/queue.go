package main

import "image"

// Queue is a fixed-size queue that discards old
// elements once it reached the maximum size.
type Queue struct {
	data []image.Point
	size int
}

// New creates a new Queue with the specified size.
func New(size uint) *Queue {
	return &Queue{
		data: []image.Point{},
		size: int(size),
	}
}

// Clear clears all elements in the queue.
func (q *Queue) Clear() {
	q.data = []image.Point{}
}

// Push pushes a new element into the queue.
func (q *Queue) Push(p image.Point) {
	if len(q.data) == q.size {
		q.data = q.data[1 : q.size-1]
	}
	q.data = append(q.data, p)
}

// Range iterates over the elements of the queue
// calling f for each element.
func (q *Queue) Range(f func(p image.Point)) {
	for _, p := range q.data {
		f(p)
	}
}

// RangePrevious iterates over the elements of the queue
// calling f for each pair of previous-current elements.
func (q *Queue) RangePrevious(f func(current image.Point, previous image.Point)) {
	for i := 1; i < len(q.data); i++ {
		f(q.data[i], q.data[i-1])
	}
}
