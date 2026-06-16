# Map Layers

First map size:

- field: 5 m x 3 m
- resolution: 0.10 m/cell
- grid: 50 x 30 cells

Grid convention:

- x -> column
- y -> row

Initial layers:

- coverage: cell has been seen
- semantic_trash_probability: likely trash
- obstacle_probability: likely obstacle
- confidence: trust in cell value
- source_id: sensor or robot that updated cell
- last_observed_time: freshness of data

First goal:

A mock detection at x=2.0, y=1.0 should update the correct grid cell.
