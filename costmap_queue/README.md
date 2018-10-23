# costmap_queue

This package contains helper classes that implement one particular common costmap operation: figuring out the distance of every cell to a subset of those cells.

## Inflation Example
Let's assume you have a grid `G` and you know that there are lethal obstacles in a subset of those cells `O`. You want to mark all cells within distance `r` with one value, and all cells within a greater distance `r2` with another.

One costly way to do this would be to to iterate through all the cells `g` in `G`, calculate the distance from `g` to each cell in `O`, find the minimum distance, and compare to `r` and `r2`. This is roughly quadratic with the number of cells.

The more efficient way to do it is to start with all the cells in `O` in a queue. For each cell you dequeue, you calculate the distance to its origin cell from `O` and then enqueue all of its neighbors. This approach is roughly linear with the number of cells, although you need to be careful to not incur too many costs from resorting the queue.

## Map Based Queue
While this operation could be done with the standard priority queue implementation, it can be optimized by using a custom priority queue based on `std::map`. This relies on the fact that many of the items inserted into the queue will have the same priority. In the `costmap_queue`, the priorities used are distances on the grid, of which there are a finite number. Thus by grouping all of the elements of the same weight into a vector together, and storing them in the `std::map` with their priority, we can eliminate the need to resort after each individual item is popped. This is based on the [optimizations to the Inflation algorithm](https://github.com/ros-planning/navigation/pull/525).

Also, since the same (or similar) distances will be used in the `costmap_queue` from iteration to iteration, additional time can be saved by not destroying the vectors in the map, and reusing them iteration to iteration.

## Costmap Queue
The `CostmapQueue` class operates on a `nav_core2::Costmap`. First, you must enqueue all of the "subset" of cells (i.e. `O` in the above example) using `enqueueCell`. Then, while the queue is not empty (i.e. `isEmpty` is false), you can call `costmap_queue::CellData cell = q.getNextCell();` to get the next cell in the queue.

The `CellData` class contains 5 values: `x_` and `y_` are the coordinates for the current cell, `src_x_` and `src_y_` are the coordinates for the original cell, and `distance_` is the distance between them.

By default, `CostmapQueue` will iterate through all the cells in the `Costmap`. If you want to limit it to only cells within a certain distance, you can use `LimitedCostmapQueue` instead.
