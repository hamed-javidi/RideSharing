This repository contains benchmark problem instances and road networks to use
with [Cargo](https://github.com/jamjpan/Cargo) or other ridesharing platforms
that support them.

The benchmark/ folder contains problem instances of the following format:

```*.instance``` file format:

    Line 1: (instance name)
    Line 2: (road network) (percentage taxis)
    Line 3: VEHICLES (number of vehicles)
    Line 4: CUSTOMERS (number of customers)
    Line 5: (blank line)
    Line 6: (header row)
    Line 7-end: vehicles and customers

The columns for lines 7 and beyond are described by the header, line 6. Some
notes about the columns:

- ORIGIN and DEST are node IDs. The road network in line 2 references an rnet
  file, and these node IDs are supposed to match with the IDs in that file.
- A negative Q indicates a vehicle. Q stands for "load", and a negative load
  represents the ability to accept positive load, hence it stands for "capacity".
  Only vehicles can accept load and have capacity.
- A DEST of -1 indicates a "taxi". In other words, it indicates that the vehicle
  has no destination of its own. Similarly, a LATE of -1 indicates the vehicle
  never ends its service.

The roadnetwork/ contains real-world road networks. These were hand-crafted,
starting from shape files obtained from various sources on the Internet, then
cleaned using [shp2rnet](https://github.com/jamjpan/shp2rnet), Matlab, and
other custom scripts. Duplicate edges, self-referencing loops, and disconnected
components have been removed, and no edge is longer than 50 meters.

Each road network is represented as two files, an "rnet" file and an "edges"
file. The edges file can be deduced from the rnet file, but for historic
reasons (read: Cargo/tool/gtreebuilder requires an edges file), both are used.

```*.edges``` file format:

    Line 1: (number nodes) (number edges)
    Line 2-end: (node_1) (node_2) (weight)

```*.rnet``` file format:

    Col 1: (edge id)      , a 0-indexed ID for the edge
    Col 2: (node_1)       , the ID of node_1 from edges file
    Col 3: (node_2)       , the ID of node_2 from edges file
    Col 4: (lng)          , longitude of node_1
    Col 5: (lat)          , latitude of node_1
    Col 6: (lng)          , longitude of node_2
    Col 7: (lat)          , latitude of node_2

Some notes:

- Each edge (pair formed by node_1 and node_2) corresponds to a line in
  the edges file.
- For historic reasons, the weights in the edges file are integer.
- Rnet files can be conveniently visualized using gnuplot with the following
  one-liner: ```plot 'bj5.rnet' u 4:5:($6-$4):($7-$5) w vectors nohead```

